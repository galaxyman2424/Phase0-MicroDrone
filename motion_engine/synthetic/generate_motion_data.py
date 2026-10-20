"""Synthetic motion data for the Motion Engine -- simple, readable scenarios.

Each scenario is a short list of (move, seconds) steps -- "go up for 2 s,
go left for 3 s, spin right for 4 s". The generator does not model the
drone itself: it flies every scenario through the real `MotionStub` using
the shared contract (`contracts.Command`, INTEGRATION_README.md section 4),
exactly as the coordinator would:

* ARM, then TAKEOFF to `TAKEOFF_ALTITUDE_M` (lifecycle commands).
* Each move becomes one SET_VELOCITY command (or HOVER), renewed every tick
  with the standard 0.2 s expiry.
* LAND, wait for `landing_settled`, then DISARM.

So the CSVs obey the same speed limits, geofence and command rules as
everything else, and a move that breaks a limit is rejected by Motion (the
generator stops with an error) instead of being written to a file.

`Move` is only this script's shorthand for writing scenarios. It is not a
command format: every move is turned into a `contracts.Command` before it
reaches Motion.

Frame (same as motion_stubs.py): x = east, y = north, z = up (metres),
yaw in degrees, 0 = facing east, counter-clockwise positive.
FORWARD/BACKWARD/LEFT/RIGHT are relative to where the drone is facing when
the move starts, so after a 90 deg yaw-left, "forward" flies north.

Usage (from the repo root):
    py -3 motion_engine/synthetic/generate_motion_data.py            # all scenarios
    py -3 motion_engine/synthetic/generate_motion_data.py --noise 0.02
    py -3 motion_engine/synthetic/generate_motion_data.py --rate 20 --out some/folder
    py -3 motion_engine/synthetic/generate_motion_data.py random_walk --seed 7

Outputs one CSV per scenario, plus a PNG plot if matplotlib is installed,
into motion_engine/motion_logs/ (change with --out).

Each CSV starts with a few `# key=value` metadata lines (scenario, rate,
noise, seed, ...). Read them back with `load_csv()` below, or with pandas:
`pd.read_csv(path, comment="#")`. Pass --no-meta for a bare CSV.

Time is written twice: `t_s` (float seconds, for eyes and plots) and
`timestamp_ns` (integer simulation nanoseconds starting at 0), which is the
clock every module uses under INTEGRATION_README.md section 4.
"""

from __future__ import annotations

import argparse
import csv
import math
import random
import sys
from enum import Enum
from pathlib import Path
from typing import Callable, Dict, List, Optional, Tuple

_MOTION_ENGINE_ROOT = Path(__file__).resolve().parents[1]
_REPO_ROOT = _MOTION_ENGINE_ROOT.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from motion_engine.contracts import (  # noqa: E402
    LIFECYCLE_KINDS,
    Command,
    MotionCommandType as K,
    Pose,
    SimulationClock,
    Velocity,
)
from motion_engine.motion_stubs import MotionLimits, MotionStub  # noqa: E402


class Move(Enum):
    """Scenario shorthand. Each value is turned into a contract Command."""

    HOVER = "hover"
    FORWARD = "forward"
    BACKWARD = "backward"
    LEFT = "left"
    RIGHT = "right"
    UP = "up"
    DOWN = "down"
    YAW_LEFT = "yaw_left"
    YAW_RIGHT = "yaw_right"


D = Move

# --------------------------------------------------------------------------
# Commanded speeds. Motion checks these against the shared MotionLimits
# (INTEGRATION_README.md section 5) and rejects anything above them.
# --------------------------------------------------------------------------
HORIZONTAL_SPEED_MPS = 1.0   # forward / backward / left / right
VERTICAL_SPEED_MPS = 0.5     # up / down
YAW_RATE_DPS = 45.0          # yaw_left / yaw_right

#: Every scenario takes off to this height before its first move. Moves
#: never go below `MotionLimits.velocity_floor_m` (1 m); landing is done by
#: a LAND command at the end, not by a DOWN move.
TAKEOFF_ALTITUDE_M = 2.0

#: Command validity (INTEGRATION_README.md section 5): 0.2 s, renewed every tick.
COMMAND_VALIDITY_NS = 200_000_000

#: Ticks allowed for takeoff or landing before the generator gives up.
_MAX_LIFECYCLE_TICKS = 10_000

# --------------------------------------------------------------------------
# Scenarios -- each is a list of (move, how many seconds to hold it).
# Every scenario starts on the pad at (0, 0, 0) facing east, takes off to
# TAKEOFF_ALTITUDE_M, flies the moves, then lands where it is.
# --------------------------------------------------------------------------
SCENARIOS: Dict[str, List[Tuple[Move, float]]] = {
    # Climb 1 m above takeoff height, pause, come back down.
    "up_down": [
        (D.HOVER, 1), (D.UP, 2), (D.HOVER, 2), (D.DOWN, 2), (D.HOVER, 1),
    ],
    # Slide 3 m left, 3 m back right.
    "left_right": [
        (D.HOVER, 1), (D.LEFT, 3), (D.HOVER, 1), (D.RIGHT, 3), (D.HOVER, 1),
    ],
    # Fly 3 m forward, 3 m back.
    "forward_backward": [
        (D.HOVER, 1), (D.FORWARD, 3), (D.HOVER, 1), (D.BACKWARD, 3), (D.HOVER, 1),
    ],
    # Spin a full circle left (8 s x 45 deg/s = 360), then right.
    "rotate": [
        (D.HOVER, 1), (D.YAW_LEFT, 8), (D.HOVER, 1), (D.YAW_RIGHT, 8), (D.HOVER, 1),
    ],
    # Fly a 3 m square by turning 90 deg left at each corner -- shows that
    # "forward" follows the heading.
    "square": [
        (D.HOVER, 1),
        (D.FORWARD, 3), (D.YAW_LEFT, 2),
        (D.FORWARD, 3), (D.YAW_LEFT, 2),
        (D.FORWARD, 3), (D.YAW_LEFT, 2),
        (D.FORWARD, 3), (D.YAW_LEFT, 2),
        (D.HOVER, 1),
    ],
    # Everything once, in order.
    "full_tour": [
        (D.HOVER, 1),
        (D.FORWARD, 2), (D.BACKWARD, 2), (D.LEFT, 2), (D.RIGHT, 2),
        (D.UP, 2), (D.DOWN, 2),
        (D.YAW_LEFT, 2), (D.YAW_RIGHT, 2),
        (D.HOVER, 1),
    ],
    # Try to hover while a steady wind pushes the drone (see DRIFT_MPS) --
    # the command says "hover", the position says otherwise. Useful for
    # checking that consumers do not assume commanded == actual.
    "drift": [
        (D.HOVER, 10),
    ],
}

#: Constant wind (east, north) in m/s applied to scenarios listed here.
#: MotionStub has no wind model, so the wind is an *environment* overlay:
#: it is added to the reported position (and velocity) while airborne, on
#: top of what Motion flew. Motion's own commands and limits are unchanged.
DRIFT_MPS: Dict[str, Tuple[float, float]] = {
    "drift": (0.3, 0.15),
}

#: Random-walk length and the hold times it picks from (seconds).
RANDOM_WALK_MOVES = 12
RANDOM_WALK_HOLDS = (0.5, 1.0, 1.5, 2.0)


def random_walk(seed: int) -> List[Tuple[Move, float]]:
    """Make RANDOM_WALK_MOVES random moves between a hover at each end.

    Deterministic for a given seed, so a robustness test that fails can be
    replayed exactly with `--seed`.
    """
    rng = random.Random(seed)
    choices = [D.FORWARD, D.BACKWARD, D.LEFT, D.RIGHT, D.YAW_LEFT, D.YAW_RIGHT, D.HOVER]
    moves: List[Tuple[Move, float]] = [(D.HOVER, 1)]
    for _ in range(RANDOM_WALK_MOVES):
        moves.append((rng.choice(choices), rng.choice(RANDOM_WALK_HOLDS)))
    moves.append((D.HOVER, 1))
    return moves


#: Scenarios built from the seed at run time instead of written out by hand.
SEEDED_SCENARIOS: Dict[str, Callable[[int], List[Tuple[Move, float]]]] = {
    "random_walk": random_walk,
}

ALL_SCENARIOS: List[str] = list(SCENARIOS) + list(SEEDED_SCENARIOS)


def scenario_moves(name: str, seed: int = 0) -> List[Tuple[Move, float]]:
    """The (move, seconds) list for any scenario name."""
    if name in SEEDED_SCENARIOS:
        return SEEDED_SCENARIOS[name](seed)
    return SCENARIOS[name]


COLUMNS = [
    "t_s", "timestamp_ns", "step", "direction",
    "command_id", "command_kind", "status", "armed", "airborne",
    "x_m", "y_m", "z_m", "yaw_deg",
    "vx_mps", "vy_mps", "vz_mps", "yaw_rate_dps",
]


def move_velocity(move: Move, yaw_deg: float) -> Optional[Velocity]:
    """World-frame velocity for holding `move` at heading `yaw_deg`.

    None for HOVER (sent as a HOVER command, not a zero velocity).
    """
    fwd = left = up = yaw_rate = 0.0
    if move == D.HOVER:
        return None
    if move == D.FORWARD:
        fwd = HORIZONTAL_SPEED_MPS
    elif move == D.BACKWARD:
        fwd = -HORIZONTAL_SPEED_MPS
    elif move == D.LEFT:
        left = HORIZONTAL_SPEED_MPS
    elif move == D.RIGHT:
        left = -HORIZONTAL_SPEED_MPS
    elif move == D.UP:
        up = VERTICAL_SPEED_MPS
    elif move == D.DOWN:
        up = -VERTICAL_SPEED_MPS
    elif move == D.YAW_LEFT:
        yaw_rate = YAW_RATE_DPS
    elif move == D.YAW_RIGHT:
        yaw_rate = -YAW_RATE_DPS
    # Rotate body (forward, left) into world (east, north). Round away the
    # 6e-17 that cos(90 deg) leaves, so a "north" move is exactly north.
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    vx, vy = round(fwd * c - left * s, 12), round(fwd * s + left * c, 12)
    return Velocity(vx + 0.0, vy + 0.0, up, yaw_rate)


class GeneratorError(RuntimeError):
    """Motion rejected a command or a phase never finished."""


def generate(moves: List[Tuple[Move, float]], rate_hz: float = 10.0,
             noise_m: float = 0.0, seed: int = 0,
             drift_mps: Tuple[float, float] = (0.0, 0.0),
             limits: Optional[MotionLimits] = None) -> List[dict]:
    """Fly `moves` through MotionStub; return one dict per sample.

    The first row is the drone on the pad before arming; then one row per
    tick, stamped at the end of the tick (when that pose is true).
    `drift_mps` is a constant (east, north) wind added to the reported
    position while airborne (see DRIFT_MPS). Raises `GeneratorError` if
    Motion rejects any command.
    """
    rng = random.Random(seed)
    dt = 1.0 / rate_hz
    clock = SimulationClock()
    motion = MotionStub(limits=limits, clock=clock)
    wind_x = wind_y = 0.0
    rows: List[dict] = []
    ids = iter(range(1, 1_000_000))

    def noise() -> float:
        return rng.gauss(0.0, noise_m) if noise_m > 0 else 0.0

    def emit(step: int, label: str, prev: Optional[dict]) -> dict:
        fb = motion.feedback(clock.timestamp_ns)
        p = fb.pose
        x, y = p.x + wind_x, p.y + wind_y
        row = {
            "t_s": round(clock.timestamp_ns / 1e9, 3), "timestamp_ns": clock.timestamp_ns,
            "step": step, "direction": label,
            "command_id": fb.active_command_id or "", "command_kind": fb.active_kind or "",
            "status": fb.status.value, "armed": fb.armed, "airborne": fb.airborne,
            "x_m": round(x + noise(), 4) + 0.0, "y_m": round(y + noise(), 4) + 0.0,
            "z_m": round(max(0.0, p.z + noise()), 4), "yaw_deg": round(p.yaw_deg, 2),
            "_x": x, "_y": y, "_z": p.z, "_yaw": p.yaw_deg,
        }
        if prev is None:
            vx = vy = vz = yr = 0.0
        else:
            vx, vy = (x - prev["_x"]) / dt, (y - prev["_y"]) / dt
            vz = (p.z - prev["_z"]) / dt
            yr = ((p.yaw_deg - prev["_yaw"] + 180.0) % 360.0 - 180.0) / dt
        row.update({
            "vx_mps": round(vx, 4) + 0.0, "vy_mps": round(vy, 4) + 0.0,
            "vz_mps": round(vz, 4) + 0.0, "yaw_rate_dps": round(yr, 2) + 0.0,
        })
        rows.append(row)
        return row

    def submit(kind: K, *, cid: Optional[str] = None, target: Optional[Pose] = None,
               velocity: Optional[Velocity] = None) -> str:
        now = clock.timestamp_ns
        cid = cid or f"gen-{next(ids)}"
        command = Command(
            command_id=cid, kind=kind, mission_state="SYNTHETIC", issued_ns=now,
            expires_ns=None if kind in LIFECYCLE_KINDS else now + COMMAND_VALIDITY_NS,
            source="synthetic.generate_motion_data", reason="scenario",
            target=target, velocity=velocity,
        )
        ack = motion.submit(command, now)
        if not ack.accepted:
            raise GeneratorError(f"Motion rejected {kind.value} ({cid}): {ack.reason}")
        return cid

    def tick(step: int, label: str) -> None:
        nonlocal wind_x, wind_y
        motion.step(dt, now_ns=clock.timestamp_ns)
        clock.advance(dt)
        if motion.airborne:  # wind only moves the drone once it is off the ground
            wind_x += drift_mps[0] * dt
            wind_y += drift_mps[1] * dt
        emit(step, label, rows[-1])

    def until(done: Callable[[], bool], step: int, label: str) -> None:
        for _ in range(_MAX_LIFECYCLE_TICKS):
            if done():
                return
            tick(step, label)
        raise GeneratorError(f"{label} never finished")

    emit(-1, "pad", None)
    submit(K.ARM)
    submit(K.TAKEOFF, target=Pose(0.0, 0.0, TAKEOFF_ALTITUDE_M, 0.0))
    until(lambda: motion.pose.z == TAKEOFF_ALTITUDE_M, -1, "takeoff")

    for step, (move, seconds) in enumerate(moves):
        velocity = move_velocity(move, motion.pose.yaw_deg)
        kind = K.HOVER if velocity is None else K.SET_VELOCITY
        cid = f"move-{step}"
        for _ in range(int(round(seconds * rate_hz))):
            submit(kind, cid=cid, velocity=velocity)  # first send, then renewals
            tick(step, move.value)

    submit(K.LAND)
    until(lambda: motion.landing_settled, len(moves), "land")
    submit(K.DISARM)
    tick(len(moves), "disarm")

    for row in rows:
        for key in ("_x", "_y", "_z", "_yaw"):
            del row[key]
    return rows


def metadata(name: str, moves: List[Tuple[Move, float]], rate_hz: float,
             noise_m: float, seed: int, drift_mps: Tuple[float, float],
             rows: Optional[List[dict]] = None) -> Dict[str, object]:
    """The `# key=value` lines written above the CSV header.

    `duration_s` is the length of the file when `rows` is given, else the
    planned move time (excluding takeoff and landing).
    """
    duration = rows[-1]["t_s"] if rows else round(sum(s for _, s in moves), 3)
    return {
        "schema_version": 2,
        "scenario": name,
        "rate_hz": rate_hz,
        "dt_s": round(1.0 / rate_hz, 6),
        "noise_m": noise_m,
        "seed": seed,
        "drift_mps": f"{drift_mps[0]},{drift_mps[1]}",
        "moves": len(moves),
        "takeoff_altitude_m": TAKEOFF_ALTITUDE_M,
        "duration_s": duration,
        "frame": "local_enu",
        "clock": "simulation",
        "synthetic": "true",
        "motion_model": "motion_engine.motion_stubs.MotionStub via contracts.Command",
        "generator": "motion_engine/synthetic/generate_motion_data.py",
    }


def write_csv(rows: List[dict], path: Path,
              meta: Optional[Dict[str, object]] = None) -> None:
    with path.open("w", newline="") as f:
        for key, value in (meta or {}).items():
            f.write(f"# {key}={value}\n")
        w = csv.DictWriter(f, fieldnames=COLUMNS)
        w.writeheader()
        w.writerows(rows)


def load_csv(path: Path) -> Tuple[Dict[str, str], List[Dict[str, str]]]:
    """Read a CSV written by this script: returns (metadata, rows).

    Values come back as strings, like any `csv.DictReader`. Files written
    with --no-meta (or before the metadata header existed) just return an
    empty metadata dict.
    """
    meta: Dict[str, str] = {}
    with Path(path).open(newline="") as f:
        lines = f.read().splitlines()
    body = []
    for line in lines:
        if line.startswith("#") and not body:
            key, _, value = line[1:].strip().partition("=")
            meta[key] = value
        else:
            body.append(line)
    return meta, list(csv.DictReader(body))


def plot(all_rows: Dict[str, List[dict]], path: Path) -> bool:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return False
    names = list(all_rows)
    # squeeze=False keeps axes 2-D even when only one scenario is plotted.
    fig, axes = plt.subplots(len(names), 2, figsize=(11, 2.6 * len(names)), squeeze=False)
    for i, name in enumerate(names):
        r = all_rows[name]
        t = [row["t_s"] for row in r]
        a = axes[i][0]
        for key, label in (("x_m", "x (east)"), ("y_m", "y (north)"), ("z_m", "z (up)")):
            a.plot(t, [row[key] for row in r], label=label)
        a.set_title(f"{name}: position")
        a.set_ylabel("m")
        a.legend(fontsize=7, loc="upper right")
        b = axes[i][1]
        b.plot(t, [row["yaw_deg"] for row in r], color="tab:purple")
        b.set_title(f"{name}: heading")
        b.set_ylabel("yaw (deg)")
        b.set_ylim(-10, 370)
    for a in axes[-1]:
        a.set_xlabel("time (s)")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return True


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--out", type=Path, default=_MOTION_ENGINE_ROOT / "motion_logs")
    p.add_argument("--rate", type=float, default=10.0, help="samples per second (default 10)")
    p.add_argument("--noise", type=float, default=0.0,
                   help="std-dev of position noise in metres (default 0 = perfectly clean)")
    p.add_argument("--seed", type=int, default=0,
                   help="seeds the noise and the random_walk scenario (default 0)")
    p.add_argument("--no-meta", action="store_true",
                   help="omit the '# key=value' metadata lines at the top of each CSV")
    p.add_argument("scenarios", nargs="*", help=f"subset of: {', '.join(ALL_SCENARIOS)}")
    args = p.parse_args()

    names = args.scenarios or ALL_SCENARIOS
    unknown = [n for n in names if n not in ALL_SCENARIOS]
    if unknown:
        sys.exit(f"Unknown scenario(s): {unknown}. Choose from {ALL_SCENARIOS}")

    args.out.mkdir(parents=True, exist_ok=True)
    all_rows = {}
    for name in names:
        moves = scenario_moves(name, args.seed)
        drift = DRIFT_MPS.get(name, (0.0, 0.0))
        try:
            rows = generate(moves, args.rate, args.noise, args.seed, drift)
        except GeneratorError as exc:
            sys.exit(f"{name}: {exc}")
        meta = None if args.no_meta else metadata(
            name, moves, args.rate, args.noise, args.seed, drift, rows)
        write_csv(rows, args.out / f"{name}.csv", meta)
        all_rows[name] = rows
        end = rows[-1]
        print(f"{name:17s} {len(rows):4d} rows, {end['t_s']:5.1f} s, ends at "
              f"x={end['x_m']:+.2f} y={end['y_m']:+.2f} z={end['z_m']:+.2f} yaw={end['yaw_deg']:.0f}")
    if plot(all_rows, args.out / "overview.png"):
        print(f"plot -> {args.out / 'overview.png'}")


if __name__ == "__main__":
    main()
