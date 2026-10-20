"""Motion <-> Safety integration (INTEGRATION_README §3-§5, PRs #61 and #65).

Drives the real `safety_layer.command_check.check_command` and the real
`MotionStub` through `motion_engine.safety_adapter`, the way the coordinator
will: Safety rules on each command, Motion executes whatever Safety approved
or substituted. Since #65 both sides share Motion's contract `Command`.
"""

import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

import pytest

from motion_engine.contracts import (
    CONTRACT_KINDS,
    VELOCITY_KINDS,
    Command,
    MotionCommandType as K,
    Pose,
    Velocity,
)
from motion_engine.motion_stubs import MotionLimits, MotionStub
from motion_engine.safety_adapter import check_for_motion
from safety_layer import config
from safety_layer.command_check import KIND_TO_ACTION, CommandDecision
from safety_layer.safety_policy import ACTIONS, SafetyAssessment

S = 1_000_000_000
DT_NS = 100_000_000
VALIDITY_NS = 200_000_000


def assessment(blocked=(), emergency=False, abort=False):
    return SafetyAssessment(
        permissions={a: a not in blocked for a in ACTIONS},
        emergency=emergency,
        abort=abort,
    )


ALL_OK = assessment()


def cmd(cid, kind, now_ns, target=None, expires=True, **kw):
    return Command(
        command_id=cid,
        kind=kind,
        mission_state="TRACKING",
        issued_ns=now_ns,
        expires_ns=(now_ns + VALIDITY_NS) if expires else None,
        source="test",
        reason="test",
        target=target,
        **kw,
    )


def airborne_motion(alt=10.0):
    """MotionStub armed and hovering at `alt`, clock at 0."""
    m = MotionStub()
    m.arm()
    m.takeoff(alt)
    while not m.at_target():
        m.step(0.1)
    return m


# -- shared configuration --------------------------------------------------


def test_motion_and_safety_use_the_same_limits():
    """README §4: 'Safety and Motion receive the same configured limits.'"""
    lim = MotionLimits()
    assert lim.max_horizontal_speed_mps == config.MAX_HORIZONTAL_SPEED_MPS
    assert lim.max_vertical_speed_mps == config.MAX_VERTICAL_SPEED_MPS
    assert lim.max_yaw_rate_dps == config.MAX_YAW_RATE_DPS
    assert lim.max_altitude_m == config.ALTITUDE_CEILING_M
    assert lim.geofence_radius_m == config.GEOFENCE_RADIUS_M


def test_safety_rules_on_every_kind_motion_accepts():
    """A Motion kind Safety has no permission for would always be held.

    SET_VELOCITY is the one known exception: Motion added it for velocity
    control, and Safety has not mapped it to a permission yet. Until it does,
    the next test pins the fail-safe behaviour. When Safety adds the mapping,
    this assertion fails -- drop the `- VELOCITY_KINDS` and the next test.
    """
    assert set(KIND_TO_ACTION) == set(CONTRACT_KINDS) - VELOCITY_KINDS
    assert set(KIND_TO_ACTION.values()) <= set(ACTIONS)


def test_set_velocity_is_held_until_safety_maps_it():
    """Unmapped kind -> Safety substitutes a HOVER; Motion flies the hold."""
    m = airborne_motion()
    c = cmd("vel-1", K.SET_VELOCITY, 0, velocity=Velocity(2.0, 0.0, 0.0, 0.0))
    out, decision = check_for_motion(c, ALL_OK, 0)
    assert not decision.allowed
    assert MotionCommandType_of(out) == K.HOVER
    assert m.submit(out, 0).accepted
    x0 = m.pose.x
    m.step(0.1, now_ns=DT_NS)
    assert m.pose.x == x0


def MotionCommandType_of(command):
    return K(command.kind)


# -- pass-through ------------------------------------------------------------


def test_allowed_track_command_passes_through_and_motion_flies_it():
    m = airborne_motion()
    c = cmd("trk-1", K.TRACK_TARGET, 0, target=Pose(20.0, 0.0, 10.0, 0.0))
    out, decision = check_for_motion(c, ALL_OK, 0)
    assert decision.allowed and out is c
    assert m.submit(out, 0).accepted
    m.step(0.1, now_ns=DT_NS)
    assert m.pose.x > 0.0


@pytest.mark.parametrize("kind", [K.HOVER, K.STOP, K.LAND])
def test_targetless_flight_kinds_are_allowed(kind):
    c = cmd("c-1", kind, 0, expires=kind != K.LAND)
    _, decision = check_for_motion(c, ALL_OK, 0)
    assert decision.allowed


@pytest.mark.parametrize("kind,target", [
    (K.MOVE_TO, Pose(5.0, 0.0, 10.0, 0.0)),
    (K.SET_YAW, Pose(0.0, 0.0, 10.0, 90.0)),
])
def test_move_to_and_set_yaw_follow_the_tracking_permission(kind, target):
    m = airborne_motion()
    out, decision = check_for_motion(cmd("x-1", kind, 0, target=target), ALL_OK, 0)
    assert decision.allowed and m.submit(out, 0).accepted

    blocked = assessment(blocked=("tracking",))
    out, decision = check_for_motion(cmd("x-2", kind, 0, target=target), blocked, 0)
    assert "action_not_permitted" in decision.reasons and out.kind == K.HOVER


# -- replacements Motion must accept ----------------------------------------------


def test_blocked_tracking_becomes_a_hover_motion_accepts_and_stops():
    m = airborne_motion()
    go = cmd("trk-1", K.TRACK_TARGET, 0, target=Pose(40.0, 0.0, 10.0, 0.0))
    assert m.submit(go, 0).accepted
    m.step(0.1, now_ns=DT_NS)

    blocked = assessment(blocked=("tracking",))
    nxt = cmd("trk-2", K.TRACK_TARGET, DT_NS, target=Pose(40.0, 0.0, 10.0, 0.0))
    out, decision = check_for_motion(nxt, blocked, DT_NS)
    assert not decision.allowed and "action_not_permitted" in decision.reasons
    assert out.kind == K.HOVER and out.source == "safety"
    assert out.command_id == "trk-2-safety"
    ack = m.submit(out, DT_NS)
    assert ack.accepted, ack.reason

    held = m.pose
    for i in range(2, 5):
        m.step(0.1, now_ns=i * DT_NS)
    assert m.pose == held  # old target no longer pulling


def test_emergency_becomes_emergency_land_motion_accepts():
    m = airborne_motion()
    c = cmd("trk-1", K.TRACK_TARGET, 0, target=Pose(10.0, 0.0, 10.0, 0.0))
    out, decision = check_for_motion(c, assessment(emergency=True), 0)
    assert not decision.allowed and "emergency" in decision.reasons
    assert out.kind == K.EMERGENCY_LAND and out.expires_ns is None
    assert m.submit(out, 0).accepted
    assert m.emergency_landing


def test_no_hold_permission_becomes_a_land_motion_accepts():
    m = airborne_motion()
    c = cmd("trk-1", K.TRACK_TARGET, 0, target=Pose(10.0, 0.0, 10.0, 0.0))
    out, decision = check_for_motion(c, assessment(blocked=("tracking", "hold")), 0)
    assert not decision.allowed
    assert out.kind == K.LAND and out.expires_ns is None
    assert m.submit(out, 0).accepted


def test_repeated_hold_replacement_is_a_renewal():
    m = airborne_motion()
    blocked = assessment(blocked=("tracking",))
    c = cmd("trk-9", K.TRACK_TARGET, 0, target=Pose(5.0, 0.0, 10.0, 0.0))
    first, _ = check_for_motion(c, blocked, 0)
    assert m.submit(first, 0).accepted
    again, _ = check_for_motion(c, blocked, DT_NS)
    ack = m.submit(again, DT_NS)
    assert ack.accepted and ack.reason == "renewed"


def test_target_outside_geofence_is_replaced_before_motion_sees_it():
    c = cmd("trk-1", K.TRACK_TARGET, 0, target=Pose(200.0, 0.0, 10.0, 0.0))
    out, decision = check_for_motion(c, ALL_OK, 0)
    assert "geofence_breach" in decision.reasons and out.kind == K.HOVER


def test_speed_cap_above_the_limit_is_replaced():
    """Caps may only lower the limit (README §4); Safety now treats a higher
    cap as malformed and holds instead of passing it to Motion."""
    c = cmd("trk-1", K.TRACK_TARGET, 0, target=Pose(5.0, 0.0, 10.0, 0.0),
            max_horizontal_speed_mps=config.MAX_HORIZONTAL_SPEED_MPS + 4.0)
    out, decision = check_for_motion(c, ALL_OK, 0)
    assert "speed_limit" in decision.reasons and out.kind == K.HOVER

    ok = cmd("trk-2", K.TRACK_TARGET, 0, target=Pose(5.0, 0.0, 10.0, 0.0),
             max_horizontal_speed_mps=1.0)
    assert check_for_motion(ok, ALL_OK, 0)[1].allowed


# -- renewal / expiry ---------------------------------------------------------------


def test_renewed_command_is_not_flagged_expired_by_safety():
    """Coordinator keeps issued_ns and pushes expires_ns forward (README §3)."""
    t_ns = 3 * S
    renewed = Command(
        command_id="trk-1", kind=K.TRACK_TARGET, mission_state="TRACKING",
        issued_ns=0, expires_ns=t_ns + VALIDITY_NS, source="test", reason="test",
        target=Pose(5.0, 0.0, 10.0, 0.0),
    )
    out, decision = check_for_motion(renewed, ALL_OK, t_ns)
    assert decision.allowed, decision.reasons
    assert out is renewed


def test_unrenewed_command_is_expired_and_held():
    c = cmd("trk-1", K.TRACK_TARGET, 0, target=Pose(5.0, 0.0, 10.0, 0.0))
    out, decision = check_for_motion(c, ALL_OK, 1 * S)
    assert "command_expired" in decision.reasons and out.kind == K.HOVER


# -- guard -------------------------------------------------------------------------


def test_non_motion_replacement_is_refused(monkeypatch):
    from motion_engine import safety_adapter

    c = cmd("trk-1", K.TRACK_TARGET, 0, target=Pose(5.0, 0.0, 10.0, 0.0))
    monkeypatch.setattr(
        safety_adapter, "check_command",
        lambda *_: CommandDecision(allowed=False, command=object()),
    )
    with pytest.raises(TypeError):
        check_for_motion(c, ALL_OK, 0)
