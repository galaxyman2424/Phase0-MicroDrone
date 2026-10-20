"""Velocity control: SET_VELOCITY through the shared contract.

Motion owns position *and* velocity control. These tests cover the
velocity half: what is accepted, what is rejected and why, how a velocity
ends (replacement, expiry, emergency, a limit), and that every run stays
inside the trace checker's rules.
"""

import math
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

import pytest

from motion_engine.contracts import (
    Command,
    CommandStatus,
    MemoryLogSink,
    MotionCommandType as K,
    Pose,
    Reason,
    SimulationClock,
    Velocity,
)
from motion_engine.motion_stubs import MotionLimits, MotionStub
from motion_engine.trace_checker import TraceRecorder, check_trace

DT = 0.1
DT_NS = 100_000_000
VALIDITY_NS = 200_000_000


def cmd(cid, kind, now_ns, *, velocity=None, target=None, expires=True, **kw):
    return Command(
        command_id=cid,
        kind=kind,
        mission_state="TRACKING",
        issued_ns=now_ns,
        expires_ns=(now_ns + VALIDITY_NS) if expires else None,
        source="test",
        reason="test",
        target=target,
        velocity=velocity,
        **kw,
    )


class Run:
    """A recorded MotionStub airborne at `alt`, driven tick by tick."""

    def __init__(self, alt=10.0, limits=None, home=Pose(0.0, 0.0, 0.0, 0.0), start=None):
        self.log = MemoryLogSink()
        self.clock = SimulationClock()
        self.motion = MotionStub(logger=self.log, limits=limits, clock=self.clock, home=home)
        if start is not None:  # place the drone before the first sample, for edge cases
            self.motion._pose = start
        self.rec = TraceRecorder(self.motion)
        self.rec.snapshot(0)
        self.submit(cmd("arm", K.ARM, self.now, expires=False))
        self.submit(cmd("takeoff", K.TAKEOFF, self.now, target=Pose(0, 0, alt, 0), expires=False))
        while self.motion.pose.z != alt:  # exact: at_target() is true 0.25 m short
            self.tick()

    @property
    def now(self):
        return self.clock.timestamp_ns

    def submit(self, command):
        return self.rec.submit(command, self.now)

    def tick(self, n=1):
        for _ in range(n):
            self.rec.step(DT, self.now)
            self.clock.advance(DT)

    def fly(self, cid, velocity, ticks):
        """Send and renew one velocity command for `ticks` ticks."""
        for _ in range(ticks):
            ack = self.submit(cmd(cid, K.SET_VELOCITY, self.now, velocity=velocity))
            assert ack.accepted, ack
            self.tick()

    def assert_clean(self):
        report = check_trace(self.rec.samples, limits=self.motion.limits, home=self.motion.home)
        assert report.ok, report.format_text()


# -- accepted ----------------------------------------------------------------


def test_velocity_moves_exactly_v_dt_per_tick():
    r = Run()
    start = r.motion.pose
    r.fly("v-1", Velocity(2.0, -1.0, 0.5, 0.0), ticks=10)
    p = r.motion.pose
    assert p.x == pytest.approx(start.x + 2.0)
    assert p.y == pytest.approx(start.y - 1.0)
    assert p.z == pytest.approx(start.z + 0.5)
    r.assert_clean()


def test_feedback_reports_velocity_in_progress_with_no_target():
    r = Run()
    r.fly("v-1", Velocity(1.0, 0.0, 0.0, 0.0), ticks=1)
    fb = r.motion.feedback(r.now)
    assert fb.status == CommandStatus.IN_PROGRESS
    assert fb.active_kind == "SET_VELOCITY"
    assert fb.active_command_id == "v-1"
    assert fb.commanded_velocity == Velocity(1.0, 0.0, 0.0, 0.0)
    assert fb.target is None and fb.arrived is False
    assert fb.to_dict()["commanded_velocity"] == {
        "vx_mps": 1.0, "vy_mps": 0.0, "vz_mps": 0.0, "yaw_rate_dps": 0.0,
    }


def test_yaw_rate_turns_and_wraps():
    r = Run()
    r.fly("spin", Velocity(0.0, 0.0, 0.0, -90.0), ticks=10)  # 90 deg clockwise
    assert r.motion.pose.yaw_deg == pytest.approx(270.0)
    r.assert_clean()


def test_renewal_keeps_flying_without_new_log_lines():
    r = Run()
    r.fly("v-1", Velocity(1.0, 0.0, 0.0, 0.0), ticks=1)
    before = len(r.log.records)
    ack = r.submit(cmd("v-1", K.SET_VELOCITY, r.now, velocity=Velocity(1.0, 0.0, 0.0, 0.0)))
    assert ack.accepted and ack.reason == Reason.RENEWED
    assert len(r.log.records) == before


def test_command_serialises_its_velocity():
    c = cmd("v-1", K.SET_VELOCITY, 0, velocity=Velocity(1.0, 2.0, -0.5, 10.0))
    assert c.to_dict()["velocity"] == {
        "vx_mps": 1.0, "vy_mps": 2.0, "vz_mps": -0.5, "yaw_rate_dps": 10.0,
    }
    assert cmd("h", K.HOVER, 0).to_dict()["velocity"] is None


# -- rejected ------------------------------------------------------------------


@pytest.mark.parametrize("velocity, reason", [
    (None, Reason.MISSING_VELOCITY),
    (Velocity(float("nan"), 0.0, 0.0, 0.0), Reason.INVALID_COMMAND),
    (Velocity(0.0, float("inf"), 0.0, 0.0), Reason.INVALID_COMMAND),
    (Velocity(4.0, 4.0, 0.0, 0.0), Reason.VELOCITY_OUTSIDE_LIMITS),   # 5.66 m/s > 5
    (Velocity(0.0, 0.0, -2.5, 0.0), Reason.VELOCITY_OUTSIDE_LIMITS),  # > 2 m/s
    (Velocity(0.0, 0.0, 0.0, 120.0), Reason.VELOCITY_OUTSIDE_LIMITS), # > 90 deg/s
])
def test_bad_velocities_are_rejected_and_the_hold_keeps_flying(velocity, reason):
    r = Run()
    pose = r.motion.pose
    ack = r.submit(cmd("bad", K.SET_VELOCITY, r.now, velocity=velocity))
    assert not ack.accepted and ack.reason == reason
    assert ack.active_command_id == "takeoff"
    r.tick(3)
    assert r.motion.pose == pose
    r.assert_clean()


def test_command_speed_cap_is_respected():
    r = Run()
    ack = r.submit(cmd("v", K.SET_VELOCITY, r.now, velocity=Velocity(3.0, 0.0, 0.0, 0.0),
                       max_horizontal_speed_mps=2.0))
    assert not ack.accepted and ack.reason == Reason.VELOCITY_OUTSIDE_LIMITS


def test_velocity_needs_an_expiry():
    r = Run()
    ack = r.submit(cmd("v", K.SET_VELOCITY, r.now, velocity=Velocity(1.0, 0, 0, 0), expires=False))
    assert not ack.accepted and ack.reason == Reason.MISSING_EXPIRY


def test_velocity_is_refused_on_the_ground():
    m = MotionStub()
    m.arm()
    ack = m.submit(cmd("v", K.SET_VELOCITY, 0, velocity=Velocity(0.0, 0.0, 1.0, 0.0)), 0)
    assert not ack.accepted and ack.reason == Reason.NOT_AIRBORNE


def test_velocity_on_another_kind_is_ignored():
    """Safety's fallback HOVER is built with dataclasses.replace() and keeps
    the field: Motion must still accept it."""
    r = Run()
    ack = r.submit(cmd("h", K.HOVER, r.now, velocity=Velocity(3.0, 0, 0, 0)))
    assert ack.accepted
    pose = r.motion.pose
    r.tick(3)
    assert r.motion.pose == pose


# -- how a velocity ends -------------------------------------------------------


def test_hold_replaces_velocity_this_tick():
    r = Run()
    r.fly("v-1", Velocity(2.0, 0.0, 0.0, 0.0), ticks=5)
    pose = r.motion.pose
    assert r.submit(cmd("h-1", K.HOVER, r.now)).accepted
    r.tick(5)
    assert r.motion.pose == pose
    assert r.motion.commanded_velocity is None
    r.assert_clean()


def test_unrenewed_velocity_expires_into_a_hold():
    r = Run()
    r.submit(cmd("v-1", K.SET_VELOCITY, r.now, velocity=Velocity(2.0, 0.0, 0.0, 0.0)))
    r.tick(2)  # 0.2 s validity: flies two ticks
    pose = r.motion.pose
    assert pose.x == pytest.approx(0.4)
    r.tick(10)
    assert r.motion.pose == pose
    fb = r.motion.feedback(r.now)
    assert fb.status == CommandStatus.EXPIRED_HOLD and fb.commanded_velocity is None
    r.assert_clean()


def test_emergency_land_overrides_velocity_and_descends_in_place():
    r = Run()
    r.fly("v-1", Velocity(2.0, 0.0, 0.0, 0.0), ticks=3)
    x = r.motion.pose.x
    assert r.submit(cmd("e", K.EMERGENCY_LAND, r.now, expires=False)).accepted
    r.tick(150)
    assert r.motion.pose.x == pytest.approx(x)
    assert r.motion.grounded and not r.motion.armed
    r.assert_clean()


@pytest.mark.parametrize("velocity, start_xy, limit", [
    (Velocity(5.0, 0.0, 0.0, 0.0), (148.0, 0.0), "geofence"),
    (Velocity(0.0, 0.0, 2.0, 0.0), (0.0, 0.0), "ceiling"),
    (Velocity(0.0, 0.0, -2.0, 0.0), (0.0, 0.0), "velocity_floor"),
])
def test_limits_stop_the_drone_at_the_boundary(velocity, start_xy, limit):
    r = Run(alt=29.0 if limit == "ceiling" else 3.0,
            start=Pose(start_xy[0], start_xy[1], 0.0, 0.0))
    r.fly("v-1", velocity, ticks=1)
    for _ in range(30):  # keep renewing: the drone must stay held anyway
        r.submit(cmd("v-1", K.SET_VELOCITY, r.now, velocity=velocity))
        r.tick()
    p, lim = r.motion.pose, r.motion.limits
    if limit == "geofence":
        assert math.hypot(p.x, p.y) == pytest.approx(lim.geofence_radius_m)
    elif limit == "ceiling":
        assert p.z == pytest.approx(lim.max_altitude_m)
    else:
        assert p.z == pytest.approx(lim.velocity_floor_m)
    fb = r.motion.feedback(r.now)
    assert fb.active_kind == "HOVER" and fb.active_command_id == "v-1"
    assert fb.commanded_velocity is None
    held = [e for e in r.log.records if e.decision == "hold_at_limit"]
    assert len(held) == 1 and limit in held[0].fields["limit"]
    r.assert_clean()


def test_a_new_velocity_command_flies_again_after_a_limit_hold():
    r = Run(alt=3.0)
    r.fly("down", Velocity(0.0, 0.0, -2.0, 0.0), ticks=15)
    assert r.motion.pose.z == pytest.approx(r.motion.limits.velocity_floor_m)
    r.fly("up", Velocity(0.0, 0.0, 1.0, 0.0), ticks=10)
    assert r.motion.pose.z == pytest.approx(r.motion.limits.velocity_floor_m + 1.0)
    r.assert_clean()


# -- direct API --------------------------------------------------------------


def test_direct_set_velocity_clamps_and_logs_it():
    log = MemoryLogSink()
    m = MotionStub(logger=log)
    m.arm()
    m.takeoff(10.0)
    while m.pose.z != 10.0:
        m.step(DT)
    assert m.set_velocity(10.0, 0.0, -5.0, 200.0)
    assert m.commanded_velocity == Velocity(5.0, 0.0, -2.0, 90.0)
    assert log.records[-1].decision == "clamped_to_limits"


def test_direct_set_velocity_refuses_on_the_ground_and_bad_numbers():
    log = MemoryLogSink()
    m = MotionStub(logger=log)
    m.arm()
    assert not m.set_velocity(0.0, 0.0, 1.0)
    assert log.records[-1].decision == "rejected_not_airborne"
    m.takeoff(5.0)
    while not m.at_target():
        m.step(DT)
    assert not m.set_velocity(float("nan"), 0.0)
    assert log.records[-1].decision == "rejected_invalid_velocity"


def test_custom_floor_is_used():
    r = Run(alt=6.0, limits=MotionLimits(velocity_floor_m=4.0))
    r.fly("down", Velocity(0.0, 0.0, -2.0, 0.0), ticks=15)
    assert r.motion.pose.z == pytest.approx(4.0)
