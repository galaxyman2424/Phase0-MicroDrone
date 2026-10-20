"""Motion Engine side of the shared Phase-0 contract (INTEGRATION_README.md §4).

The integration README asks every module to talk to the central coordinator
through one versioned contract (`schema_version = 1`): commands go *in*,
an acknowledgement comes straight back, and feedback reports progress on
later ticks. This file holds the Motion Engine's half of that contract:

* `Command`         -- what the coordinator dispatches after Safety approves it
* `CommandAck`      -- Motion's immediate answer: accepted or rejected, and why
* `VehicleFeedback` -- the `vehicle` section of the snapshot, read each tick

plus the value types they share (`Pose`, `Velocity`, `MotionCommandType`), a stable set
of reason strings, a simulation clock, and a tiny in-memory log sink.

Each interface is specified field by field (purpose, publisher, subscriber,
units, update rate, validation rules, example payloads) in
`motion_engine/INTERFACES.md`. Keep the two in step.

PROVISIONAL LOCATION: the README suggests the shared types eventually live in
`state_machine/contracts.py` and the logger in `state_machine/logging_format.py`,
owned by the state-machine team. Neither exists yet. Until they do, the
Motion types live here with the README's field names, so moving them is a
cut-and-paste plus a re-export, not a rename.

Conventions (README §4):

* Time is integer simulation nanoseconds (`timestamp_ns`), starting at 0.
  Wall time is never used for a decision.
* Position is local ENU metres (x east, y north, z up), pad and ground at
  z = 0. Yaw is degrees, 0 = east, counter-clockwise positive.
* A missing required field is a *validation result* (a rejection with a
  reason), never a silently-filled "safe" default.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional, Protocol, Tuple

SCHEMA_VERSION: int = 1

#: The only frame accepted in this milestone.
FRAME_LOCAL_ENU: str = "local_enu"

#: Source id stamped on Motion feedback.
MOTION_SOURCE_ID: str = "motion_stub"


# --------------------------------------------------------------------------
# Value types
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class Pose:
    """Position (m, local ENU, z up) plus heading (deg, 0 = east, CCW +).

    Frozen on purpose: `MotionStub.pose` hands this out to the coordinator
    and the synthetic world every tick. If it were mutable, one consumer
    doing `pose.z = 0` would silently teleport the drone for everyone else.
    """

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw_deg: float = 0.0

    def distance_to(self, other: "Pose") -> float:
        """3-D Euclidean distance in metres (yaw ignored)."""
        return math.sqrt(
            (self.x - other.x) ** 2 + (self.y - other.y) ** 2 + (self.z - other.z) ** 2
        )

    def horizontal_distance_to(self, other: "Pose") -> float:
        """Distance in the x/y plane only, metres."""
        return math.hypot(self.x - other.x, self.y - other.y)

    def as_tuple(self) -> Tuple[float, float, float, float]:
        return (self.x, self.y, self.z, self.yaw_deg)

    def is_finite(self) -> bool:
        return all(_is_number(v) and math.isfinite(v) for v in self.as_tuple())

    def to_dict(self) -> Dict[str, float]:
        return {"x": self.x, "y": self.y, "z": self.z, "yaw_deg": self.yaw_deg}


@dataclass(frozen=True)
class Velocity:
    """Commanded velocity in the same local ENU frame as `Pose`.

    `vx_mps` east, `vy_mps` north, `vz_mps` up (m/s); `yaw_rate_dps` in
    deg/s, counter-clockwise positive (same sign convention as yaw).
    World frame, not body frame: "forward" is the caller's job to rotate.
    """

    vx_mps: float = 0.0
    vy_mps: float = 0.0
    vz_mps: float = 0.0
    yaw_rate_dps: float = 0.0

    def as_tuple(self) -> Tuple[float, float, float, float]:
        return (self.vx_mps, self.vy_mps, self.vz_mps, self.yaw_rate_dps)

    def is_finite(self) -> bool:
        return all(_is_number(v) and math.isfinite(v) for v in self.as_tuple())

    @property
    def horizontal_speed_mps(self) -> float:
        return math.hypot(self.vx_mps, self.vy_mps)

    def to_dict(self) -> Dict[str, float]:
        return {
            "vx_mps": self.vx_mps,
            "vy_mps": self.vy_mps,
            "vz_mps": self.vz_mps,
            "yaw_rate_dps": self.yaw_rate_dps,
        }


class MotionCommandType(str, Enum):
    """Every command kind Motion understands. One-to-one with MotionStub's
    public command methods; the string values are what goes in the logs."""

    ARM = "ARM"
    DISARM = "DISARM"
    TAKEOFF = "TAKEOFF"
    MOVE_TO = "MOVE_TO"
    HOVER = "HOVER"
    SET_YAW = "SET_YAW"
    SET_ORIENTATION = "SET_ORIENTATION"
    TRACK_TARGET = "TRACK_TARGET"
    GOTO_DOCK_APPROACH = "GOTO_DOCK_APPROACH"
    DESCEND_TO_DOCK = "DESCEND_TO_DOCK"
    LAND = "LAND"
    EMERGENCY_LAND = "EMERGENCY_LAND"
    STOP = "STOP"
    SET_VELOCITY = "SET_VELOCITY"


#: Kinds the coordinator may send through `MotionStub.submit()`.
#: SET_ORIENTATION is not one: roll/pitch are not part of this milestone.
CONTRACT_KINDS = frozenset(MotionCommandType) - {MotionCommandType.SET_ORIENTATION}

#: Kinds that need a target pose in the command.
TARGET_KINDS = frozenset({
    MotionCommandType.TAKEOFF,           # only target.z is used (climb in place)
    MotionCommandType.MOVE_TO,
    MotionCommandType.TRACK_TARGET,      # Navigation's proposed tracking pose
    MotionCommandType.SET_YAW,           # only target.yaw_deg is used
    MotionCommandType.GOTO_DOCK_APPROACH,  # the approach point itself
    MotionCommandType.DESCEND_TO_DOCK,   # the pad pose
})

#: Kinds that need a `velocity` in the command instead of a target pose.
#: Never a lifecycle kind: a velocity must always expire, or a lost
#: coordinator would leave the drone flying in a straight line.
VELOCITY_KINDS = frozenset({
    MotionCommandType.SET_VELOCITY,
})

#: Entry actions / lifecycle steps (README §3). These may omit `expires_ns`;
#: every other kind must carry one and be renewed by the coordinator.
LIFECYCLE_KINDS = frozenset({
    MotionCommandType.ARM,
    MotionCommandType.DISARM,
    MotionCommandType.TAKEOFF,
    MotionCommandType.LAND,
    MotionCommandType.EMERGENCY_LAND,
})

#: Kinds whose job is to put the vehicle on the ground.
LANDING_KINDS = frozenset({
    MotionCommandType.LAND,
    MotionCommandType.DESCEND_TO_DOCK,
    MotionCommandType.EMERGENCY_LAND,
})


class Reason:
    """Stable reason strings for acks, feedback and log decisions.

    Compare against these constants, never against free text. Optional
    human-readable detail goes in a separate `detail` field.
    """

    # accepted
    ACCEPTED = "accepted"
    RENEWED = "renewed"
    ALREADY_EMERGENCY_LANDING = "already_emergency_landing"
    # rejected -- malformed input
    INVALID_COMMAND = "invalid_command"
    UNSUPPORTED_SCHEMA = "unsupported_schema"
    UNSUPPORTED_FRAME = "unsupported_frame"
    UNSUPPORTED_KIND = "unsupported_kind"
    MISSING_TARGET = "missing_target"
    MISSING_VELOCITY = "missing_velocity"
    MISSING_EXPIRY = "missing_expiry"
    FUTURE_TIMESTAMP = "future_timestamp"
    COMMAND_ID_REUSED = "command_id_reused"
    # rejected -- not executable right now
    COMMAND_EXPIRED = "command_expired"
    NOT_ARMED = "not_armed"
    AIRBORNE = "airborne"
    EMERGENCY_DESCENT_ACTIVE = "emergency_descent_active"
    TARGET_OUTSIDE_LIMITS = "target_outside_limits"
    VELOCITY_OUTSIDE_LIMITS = "velocity_outside_limits"
    NOT_AIRBORNE = "not_airborne"


class CommandStatus(str, Enum):
    """Execution status of the active command, reported in feedback.

    Deliberately separate states, because the README says acceptance,
    arrival, settled landing and disarming are different facts:
    """

    NONE = "none"                    # nothing active (e.g. on the pad, disarmed)
    IN_PROGRESS = "in_progress"      # accepted, still moving toward the target
    ARRIVED = "arrived"              # within arrival tolerance -- NOT "landed"
    LANDED = "landed"                # landing command settled exactly at z = 0
    EXPIRED_HOLD = "expired_hold"    # command expired; Motion replaced it with a hold
    EMERGENCY_DESCENT = "emergency_descent"


# --------------------------------------------------------------------------
# Command / acknowledgement / feedback
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class Command:
    """One command from the coordinator (after Safety), README §4.

    `expires_ns` is required for every kind except `LIFECYCLE_KINDS`. The
    coordinator renews a tracking/hold/docking target by re-sending the
    *same* `command_id` with a later `expires_ns`; Motion treats that as a
    renewal (no restart, no new log line) as long as kind and target match.

    `max_horizontal_speed_mps` / `max_vertical_speed_mps` can only lower the
    configured limits, never raise them.

    `velocity` is used only by `VELOCITY_KINDS` (and required there); every
    other kind ignores it, the same way kinds without a target ignore `target`. Adding it kept `schema_version = 1`: it is
    optional with a None default, so existing producers are unaffected.
    """

    command_id: str
    kind: MotionCommandType
    mission_state: str
    issued_ns: int
    expires_ns: Optional[int]
    source: str
    reason: str
    target: Optional[Pose] = None
    frame: str = FRAME_LOCAL_ENU
    max_horizontal_speed_mps: Optional[float] = None
    max_vertical_speed_mps: Optional[float] = None
    velocity: Optional[Velocity] = None
    schema_version: int = SCHEMA_VERSION

    def validate(self) -> Optional[str]:
        """Return a `Reason` if the command is malformed, else None.

        Only checks the command on its own; whether it can run *now*
        (armed, expired, inside limits) is MotionStub's call.
        """
        if self.schema_version != SCHEMA_VERSION:
            return Reason.UNSUPPORTED_SCHEMA
        if not isinstance(self.command_id, str) or not self.command_id:
            return Reason.INVALID_COMMAND
        for text in (self.mission_state, self.source, self.reason):
            if not isinstance(text, str):
                return Reason.INVALID_COMMAND
        try:
            kind = MotionCommandType(self.kind)
        except ValueError:
            return Reason.UNSUPPORTED_KIND
        if kind not in CONTRACT_KINDS:
            return Reason.UNSUPPORTED_KIND
        if self.frame != FRAME_LOCAL_ENU:
            return Reason.UNSUPPORTED_FRAME
        if not _is_int(self.issued_ns) or self.issued_ns < 0:
            return Reason.INVALID_COMMAND
        if self.expires_ns is None:
            if kind not in LIFECYCLE_KINDS:
                return Reason.MISSING_EXPIRY
        elif not _is_int(self.expires_ns) or self.expires_ns <= self.issued_ns:
            return Reason.INVALID_COMMAND
        if kind in TARGET_KINDS:
            if self.target is None:
                return Reason.MISSING_TARGET
            if not isinstance(self.target, Pose) or not self.target.is_finite():
                return Reason.INVALID_COMMAND
        if kind in VELOCITY_KINDS:
            if self.velocity is None:
                return Reason.MISSING_VELOCITY
            if not isinstance(self.velocity, Velocity) or not self.velocity.is_finite():
                return Reason.INVALID_COMMAND
        # Any other kind ignores `velocity`, exactly as HOVER ignores `target`.
        # It must not be rejected: Safety builds its fallback HOVER/LAND with
        # dataclasses.replace(original, ...), which carries the field over,
        # and Motion refusing Safety's own hold would be the worst outcome.
        for cap in (self.max_horizontal_speed_mps, self.max_vertical_speed_mps):
            if cap is not None and not (_is_number(cap) and math.isfinite(cap) and cap > 0.0):
                return Reason.INVALID_COMMAND
        return None

    def same_request_as(self, other: "Command") -> bool:
        """True if `other` asks for exactly the same thing (renewal check)."""
        return (
            MotionCommandType(self.kind) == MotionCommandType(other.kind)
            and self.target == other.target
            and self.velocity == other.velocity
            and self.max_horizontal_speed_mps == other.max_horizontal_speed_mps
            and self.max_vertical_speed_mps == other.max_vertical_speed_mps
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "command_id": self.command_id,
            "kind": _enum_value(self.kind),
            "mission_state": self.mission_state,
            "target": self.target.to_dict() if isinstance(self.target, Pose) else None,
            "velocity": self.velocity.to_dict() if isinstance(self.velocity, Velocity) else None,
            "frame": self.frame,
            "max_horizontal_speed_mps": self.max_horizontal_speed_mps,
            "max_vertical_speed_mps": self.max_vertical_speed_mps,
            "issued_ns": self.issued_ns,
            "expires_ns": self.expires_ns,
            "source": self.source,
            "reason": self.reason,
        }


@dataclass(frozen=True)
class CommandAck:
    """Motion's immediate answer to one `Command`.

    `accepted=True` means "I am now executing this", not "done". Progress
    toward the target is reported later through `VehicleFeedback` under the
    same `command_id`. `active_command_id` is whatever Motion is executing
    after handling this command -- on a rejection it is the *previous*
    command, so the coordinator can see what is still flying.
    """

    command_id: str
    accepted: bool
    reason: str
    timestamp_ns: int
    kind: Optional[str] = None
    active_command_id: Optional[str] = None
    detail: Optional[str] = None
    schema_version: int = SCHEMA_VERSION

    def to_dict(self) -> Dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "command_id": self.command_id,
            "kind": self.kind,
            "accepted": self.accepted,
            "reason": self.reason,
            "detail": self.detail,
            "active_command_id": self.active_command_id,
            "timestamp_ns": self.timestamp_ns,
        }


@dataclass(frozen=True)
class VehicleFeedback:
    """The `vehicle` section of the coordinator's snapshot (README §4).

    Arrival and landing are separate on purpose (README §8): `arrived` can
    be true while still airborne (e.g. 0.2 m above the pad), so docking
    must use `landing_settled` + `grounded`, never `arrived`, to confirm
    touchdown.
    """

    timestamp_ns: int
    pose: Pose
    armed: bool
    airborne: bool
    grounded: bool
    emergency_descent: bool
    landing_settled: bool
    active_command_id: Optional[str]
    active_kind: Optional[str]
    target: Optional[Pose]
    status: CommandStatus
    arrived: bool
    distance_to_target_m: Optional[float]
    horizontal_error_m: Optional[float]
    vertical_error_m: Optional[float]
    yaw_error_deg: Optional[float]
    #: The active SET_VELOCITY command's velocity, else None. A velocity
    #: command has no target pose, so `target` and the error fields are None.
    commanded_velocity: Optional[Velocity] = None
    valid: bool = True
    reason: Optional[str] = None
    source_id: str = MOTION_SOURCE_ID
    synthetic: bool = True
    schema_version: int = SCHEMA_VERSION

    def to_dict(self) -> Dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "timestamp_ns": self.timestamp_ns,
            "valid": self.valid,
            "reason": self.reason,
            "source_id": self.source_id,
            "synthetic": self.synthetic,
            "pose": self.pose.to_dict(),
            "armed": self.armed,
            "airborne": self.airborne,
            "grounded": self.grounded,
            "emergency_descent": self.emergency_descent,
            "landing_settled": self.landing_settled,
            "active_command_id": self.active_command_id,
            "active_kind": self.active_kind,
            "target": self.target.to_dict() if self.target is not None else None,
            "status": self.status.value,
            "arrived": self.arrived,
            "distance_to_target_m": self.distance_to_target_m,
            "horizontal_error_m": self.horizontal_error_m,
            "vertical_error_m": self.vertical_error_m,
            "yaw_error_deg": self.yaw_error_deg,
            "commanded_velocity": (
                self.commanded_velocity.to_dict() if self.commanded_velocity is not None else None
            ),
        }


# --------------------------------------------------------------------------
# Clock and logging stand-ins
# --------------------------------------------------------------------------


class SimulationClock:
    """Integer-nanosecond simulation clock (README §3/§4), starting at 0.

    Owned by the coordinator; Motion only reads it. Calling the instance
    returns seconds as a float, so it can be passed straight in as
    `MotionStub(clock=...)` and log timestamps become simulation time --
    which is what makes runs reproducible at any wall-clock speed.
    """

    def __init__(self, start_ns: int = 0) -> None:
        self.timestamp_ns = int(start_ns)

    def advance(self, dt_s: float) -> int:
        self.timestamp_ns += int(round(dt_s * 1e9))
        return self.timestamp_ns

    def __call__(self) -> float:
        return self.timestamp_ns / 1e9


class LogSink(Protocol):
    """What MotionStub needs from a logger: one `log()` call per record.

    Matches the agreed six-column format (timestamp, module, event, state,
    synthetic_truth, decision). The shared implementation is planned for
    `state_machine/logging_format.py`; anything with this method works.
    """

    def log(
        self,
        module: Any,
        event: str,
        *,
        fields: Optional[Dict[str, Any]] = None,
        state: Optional[str] = None,
        truth: Optional[Dict[str, Any]] = None,
        decision: Optional[str] = None,
        t: Optional[float] = None,
    ) -> Any: ...


@dataclass
class LogEntry:
    t: Optional[float]
    module: str
    event: str
    fields: Dict[str, Any] = field(default_factory=dict)
    state: Optional[str] = None
    truth: Optional[Dict[str, Any]] = None
    decision: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "t": self.t,
            "module": self.module,
            "event": self.event,
            "fields": dict(self.fields),
            "state": self.state,
            "synthetic_truth": self.truth,
            "decision": self.decision,
        }


class MemoryLogSink:
    """Minimal in-memory `LogSink`: keeps every record in `.records`.

    Used by the motion tests and usable by anyone wiring Motion up before
    the shared logger lands. Not a replacement for it.
    """

    def __init__(self) -> None:
        self.records: List[LogEntry] = []

    def log(self, module, event, *, fields=None, state=None, truth=None, decision=None, t=None):
        entry = LogEntry(
            t=t,
            module=_enum_value(module),
            event=event,
            fields=dict(fields or {}),
            state=state,
            truth=truth,
            decision=decision,
        )
        self.records.append(entry)
        return entry


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------


def _is_number(v: Any) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _is_int(v: Any) -> bool:
    return isinstance(v, int) and not isinstance(v, bool)


def _enum_value(v: Any) -> Any:
    return v.value if isinstance(v, Enum) else v
