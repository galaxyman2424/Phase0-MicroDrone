# Motion Engine Interfaces

The authoritative spec for the three types that cross Motion's boundary.
The code is [`contracts.py`](contracts.py); the rules for *when* they are
exchanged (tick order, ownership) are in
[INTEGRATION_README.md](../INTEGRATION_README.md) §3-§4, and the configured
limits are in its §5 table. This page does not repeat those; it specifies
the messages.

## Where these fit in the project's interface list

| Project interface | Contract type | Owner | Status |
|---|---|---|---|
| `MotionCommand` | [`Command`](#motioncommand--contractscommand) | Motion | Implemented, `schema_version = 1` |
| (ack for `MotionCommand`) | [`CommandAck`](#commandack) | Motion | Implemented, `schema_version = 1` |
| `DroneState` (vehicle part) | [`VehicleFeedback`](#dronestate-vehicle-section--contractsvehiclefeedback) | Motion | Implemented, `schema_version = 1`. Battery, environment and mission context come from other producers (INTEGRATION_README §4 snapshot table). |
| `DetectionResult` | — | CV | Not defined yet |
| `DockStatus` | — | Docking | Not defined yet (`docking/states/docking_controller.py` has local `DockingUpdate`) |
| `SafetyEvent` | — | Safety | Not defined yet (`safety_layer` has local `SafetyAssessment` / `CommandDecision`) |
| `MissionStatus` | — | State machine | Not defined yet |

**Location.** These types live in `motion_engine/contracts.py` for now, but
`state_machine/`, `safety_layer/` and `docking/` already import them.
INTEGRATION_README §1 names `state_machine/contracts.py` as the eventual
shared home. Moving them is a cut-and-paste plus a re-export, not a rename.
That move should be agreed by every team, not made from inside Motion.

## Conventions (all three types)

| Item | Rule |
|---|---|
| Frame | `local_enu`: x east, y north, z up, metres. Origin = pad, ground z = 0 |
| Heading | `yaw_deg` degrees, 0 = east, counter-clockwise positive, [0, 360) |
| Yaw rate | `yaw_rate_dps` deg/s, counter-clockwise positive |
| Time | Integer simulation nanoseconds (`*_ns`), starting at 0. Never wall time |
| Speeds | m/s; horizontal and vertical capped separately |
| Reasons | Stable lowercase strings from `contracts.Reason`; free text only in `detail` |
| Versioning | Every message carries `schema_version`. A consumer must reject a version it does not know. Adding an optional field with a `null` default keeps the version; renaming, removing or changing the meaning of a field bumps it |
| Missing data | A missing required field is a rejection with a reason, never a default "safe" value |

---

## MotionCommand — `contracts.Command`

**Purpose.** Ask Motion to do one thing: arm, take off, fly to a pose, fly
at a velocity, hold, land, and so on. Motion carries the command out; it
never decides what the command should be.

**Publisher.** The coordinator (`state_machine/run_loop.py`), after Safety's
command check (`safety_layer/command_check.py`) has allowed it or swapped
in a replacement.

**Subscriber.** Motion (`MotionStub.submit()`). Safety also reads every
command before it is sent.

**Update rate.** Event-driven for lifecycle kinds (sent once). Every other
kind is renewed every coordinator tick (10 Hz) by re-sending the same
`command_id` with a later `expires_ns`. A command that stops being renewed
expires 0.2 s after issue and Motion holds in place.

### Fields

| Field | Type | Units | Required | Notes |
|---|---|---|---|---|
| `schema_version` | int | — | yes | `1` |
| `command_id` | str | — | yes | Non-empty. Unique per request; reuse only to renew the same request |
| `kind` | `MotionCommandType` | — | yes | See kinds table below |
| `mission_state` | str | — | yes | Mission state that issued it, for the log. Motion never interprets it |
| `target` | `Pose` or null | m, deg | for target kinds | `{x, y, z, yaw_deg}`, local ENU |
| `velocity` | `Velocity` or null | m/s, deg/s | for `SET_VELOCITY` | `{vx_mps, vy_mps, vz_mps, yaw_rate_dps}`, world frame. Ignored by other kinds |
| `frame` | str | — | yes | Must be `local_enu` |
| `max_horizontal_speed_mps` | float or null | m/s | no | Can only lower the configured limit |
| `max_vertical_speed_mps` | float or null | m/s | no | Can only lower the configured limit |
| `issued_ns` | int | ns | yes | ≥ 0, not in the future |
| `expires_ns` | int or null | ns | all but lifecycle kinds | > `issued_ns` |
| `source` | str | — | yes | Producer, e.g. `state_machine.run_loop`, `safety` |
| `reason` | str | — | yes | Why it was sent, for the log |

### Kinds

| Kind | Needs | Expiry | Effect |
|---|---|---|---|
| `ARM` | — | no | Arm |
| `DISARM` | grounded | no | Disarm; refused in the air |
| `TAKEOFF` | `target.z` | no | Climb in place to `target.z` |
| `MOVE_TO` | `target` | yes | Fly straight to the pose (position control) |
| `TRACK_TARGET` | `target` | yes | Same as `MOVE_TO`, logged as tracking (pose from Navigation) |
| `SET_YAW` | `target.yaw_deg` | yes | Turn in place |
| `SET_VELOCITY` | `velocity`, airborne | yes | Fly at that velocity until replaced or expired (velocity control) |
| `HOVER` / `STOP` | — | yes | Hold the current pose |
| `GOTO_DOCK_APPROACH` | `target` | yes | Fly to the approach point above the pad |
| `DESCEND_TO_DOCK` | `target` (pad) | yes | Descend onto the pad; ends `landed` |
| `LAND` | — | no | Land in place |
| `EMERGENCY_LAND` | — | no | Descend in place at 1 m/s; latches until touchdown, then auto-disarms |

### Validation rules

Motion answers every command with a `CommandAck`. Rejections use these reasons:

| Check | Reason |
|---|---|
| Not a `Command`, empty ID, non-string text fields, bad `issued_ns` / `expires_ns` | `invalid_command` |
| `schema_version` ≠ 1 | `unsupported_schema` |
| Unknown kind, or `SET_ORIENTATION` | `unsupported_kind` |
| `frame` ≠ `local_enu` | `unsupported_frame` |
| Target kind without a target / target with NaN or inf | `missing_target` / `invalid_command` |
| `SET_VELOCITY` without a velocity / velocity with NaN or inf | `missing_velocity` / `invalid_command` |
| Non-lifecycle kind with no `expires_ns` | `missing_expiry` |
| `issued_ns` after Motion's current time | `future_timestamp` |
| Already expired on arrival | `command_expired` |
| Known `command_id` with a different request | `command_id_reused` |
| Anything but `ARM` / `DISARM` while disarmed | `not_armed` |
| `DISARM` while airborne | `airborne` |
| Anything but `EMERGENCY_LAND` during an emergency descent | `emergency_descent_active` |
| Target beyond the geofence radius or the ceiling | `target_outside_limits` (rejected, never clamped) |
| `SET_VELOCITY` while grounded | `not_airborne` |
| Velocity above the horizontal / vertical / yaw-rate limit or the command's own cap | `velocity_outside_limits` |

While a `SET_VELOCITY` is flying, the geofence, the ceiling and
`MotionLimits.velocity_floor_m` (1 m) are enforced by stopping at the
boundary: the active kind becomes `HOVER` under the same `command_id` and the
log shows `decision = hold_at_limit`. The coordinator must then send
something new.

### Example payloads

```json
{
  "schema_version": 1,
  "command_id": "sm-42",
  "kind": "MOVE_TO",
  "mission_state": "TRACKING",
  "target": {"x": 20.0, "y": 5.0, "z": 5.0, "yaw_deg": 90.0},
  "velocity": null,
  "frame": "local_enu",
  "max_horizontal_speed_mps": null,
  "max_vertical_speed_mps": null,
  "issued_ns": 12300000000,
  "expires_ns": 12500000000,
  "source": "state_machine.run_loop",
  "reason": "tracking_target"
}
```

```json
{
  "schema_version": 1,
  "command_id": "sm-43",
  "kind": "SET_VELOCITY",
  "mission_state": "TRACKING",
  "target": null,
  "velocity": {"vx_mps": 1.5, "vy_mps": 0.0, "vz_mps": 0.0, "yaw_rate_dps": 10.0},
  "frame": "local_enu",
  "max_horizontal_speed_mps": null,
  "max_vertical_speed_mps": null,
  "issued_ns": 12400000000,
  "expires_ns": 12600000000,
  "source": "state_machine.run_loop",
  "reason": "tracking_target"
}
```

---

## CommandAck

**Purpose.** Motion's immediate answer to one `Command`: accepted or
rejected, and why. Accepted means "now executing", not "done".

**Publisher.** Motion, returned from `submit()`.

**Subscriber.** The coordinator, which records it in the tick result and
passes a rejection to the next mission decision (INTEGRATION_README §3).

**Update rate.** Exactly one per submitted command, in the same tick.

### Fields

| Field | Type | Units | Notes |
|---|---|---|---|
| `schema_version` | int | — | `1` |
| `command_id` | str | — | The command being answered |
| `kind` | str or null | — | The command's kind |
| `accepted` | bool | — | |
| `reason` | str | — | `accepted`, `renewed`, `already_emergency_landing`, or a rejection reason above |
| `detail` | str or null | — | Optional free text; never compare against it |
| `active_command_id` | str or null | — | What Motion is flying *after* handling this command. On a rejection this is the previous command |
| `timestamp_ns` | int | ns | Simulation time of the answer |

### Validation rules

Produced only by Motion. A consumer should treat an unknown `reason` as a
rejection.

### Example payload

```json
{
  "schema_version": 1,
  "command_id": "sm-9",
  "kind": "MOVE_TO",
  "accepted": false,
  "reason": "target_outside_limits",
  "detail": null,
  "active_command_id": "takeoff-1",
  "timestamp_ns": 300000000
}
```

---

## DroneState (vehicle section) — `contracts.VehicleFeedback`

**Purpose.** Where the drone is and what Motion is doing, read once per tick.
It keeps acceptance, arrival, settled landing and disarming as separate facts
(INTEGRATION_README §8).

**Publisher.** Motion (`MotionStub.feedback()`). Read-only: calling it never
changes Motion's state.

**Subscriber.** The coordinator, which puts it in the snapshot's `vehicle`
section. From there it is read by the mission transition function, Safety
(vehicle validity, hold permission) and Docking (touchdown confirmation),
and written to the run log.

**Update rate.** Once per coordinator tick (10 Hz), after `motion.step()`.

### Fields

| Field | Type | Units | Notes |
|---|---|---|---|
| `schema_version` | int | — | `1` |
| `timestamp_ns` | int | ns | Simulation time of this reading |
| `valid` | bool | — | False if the reading cannot be trusted (always true in the stub) |
| `reason` | str or null | — | Why `valid` is false |
| `source_id` | str | — | `motion_stub` |
| `synthetic` | bool | — | True in Phase 0 |
| `pose` | `Pose` | m, deg | Current position and heading |
| `armed` | bool | — | |
| `airborne` | bool | — | z > 0.05 m |
| `grounded` | bool | — | Not airborne |
| `emergency_descent` | bool | — | Emergency latch active |
| `landing_settled` | bool | — | Landing command finished exactly on its target at z = 0. **Use this, not `arrived`, to confirm touchdown** |
| `active_command_id` | str or null | — | Command Motion is executing |
| `active_kind` | str or null | — | Its kind. Becomes `HOVER` after an expiry or a velocity limit hold |
| `target` | `Pose` or null | m, deg | Null when nothing is active or under `SET_VELOCITY` |
| `commanded_velocity` | `Velocity` or null | m/s, deg/s | Set only while a `SET_VELOCITY` is flying |
| `status` | str | — | `none`, `in_progress`, `arrived`, `landed`, `expired_hold`, `emergency_descent` |
| `arrived` | bool | — | Within 0.25 m and 3° of `target`. Can be true while still in the air |
| `distance_to_target_m` | float or null | m | 3-D |
| `horizontal_error_m` | float or null | m | |
| `vertical_error_m` | float or null | m | target.z − pose.z |
| `yaw_error_deg` | float or null | deg | Absolute, shortest way round |

### Validation rules

- `airborne` and `grounded` are always opposite.
- `landing_settled` implies `grounded` and `status = landed` (or the disarm after it).
- `target` and `commanded_velocity` are never both set.
- Consumers must check `valid` and the age of `timestamp_ns` (max 0.5 s,
  INTEGRATION_README §5) before using the pose.
- The trace checker ([`trace_checker.py`](trace_checker.py)) checks these and
  the speed, geofence and emergency rules across whole runs.

### Example payload

```json
{
  "schema_version": 1,
  "timestamp_ns": 300000000,
  "valid": true,
  "reason": null,
  "source_id": "motion_stub",
  "synthetic": true,
  "pose": {"x": 0.0, "y": 0.0, "z": 0.6, "yaw_deg": 0.0},
  "armed": true,
  "airborne": true,
  "grounded": false,
  "emergency_descent": false,
  "landing_settled": false,
  "active_command_id": "takeoff-1",
  "active_kind": "TAKEOFF",
  "target": {"x": 0.0, "y": 0.0, "z": 5.0, "yaw_deg": 0.0},
  "status": "in_progress",
  "arrived": false,
  "distance_to_target_m": 4.4,
  "horizontal_error_m": 0.0,
  "vertical_error_m": 4.4,
  "yaw_error_deg": 0.0,
  "commanded_velocity": null
}
```
