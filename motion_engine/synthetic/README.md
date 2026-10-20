# Synthetic motion data

Simple, hand-readable flight data for the Motion Engine: the drone going up,
down, left, right, forward, back, and spinning in place.

```bash
py -3 motion_engine/synthetic/generate_motion_data.py              # all scenarios -> motion_engine/motion_logs/
py -3 motion_engine/synthetic/generate_motion_data.py rotate square
py -3 motion_engine/synthetic/generate_motion_data.py --noise 0.02 # add a little sensor jitter
py -3 motion_engine/synthetic/generate_motion_data.py random_walk --seed 7  # a different random walk
py -3 motion_engine/synthetic/generate_motion_data.py --no-meta    # bare CSVs, no metadata lines
```

## How it works

The generator has no motion model of its own. It flies each scenario through
the real `MotionStub` using the shared contract, the same way the coordinator
does:

1. `ARM`, then `TAKEOFF` to 2 m.
2. Each move becomes one `SET_VELOCITY` command (or `HOVER`), renewed every
   tick with the standard 0.2 s expiry.
3. `LAND`, wait for `landing_settled`, then `DISARM`.

So the data obeys the shared limits ([INTEGRATION_README §5](../../INTEGRATION_README.md#5-default-configuration)),
geofence and command rules automatically. A move that breaks a limit is
rejected by Motion and the generator stops with an error, instead of
writing a file no real run could produce.

## Scenarios

Output goes to `motion_engine/motion_logs/` (one CSV per scenario + `overview.png`).
Every scenario takes off from the pad, flies its moves, and lands where it is.

| File | What the drone does |
|---|---|
| `up_down.csv` | Climbs 1 m above takeoff height, pauses, comes back down |
| `left_right.csv` | Slides 3 m left, 3 m back right |
| `forward_backward.csv` | Flies 3 m forward, 3 m back |
| `rotate.csv` | Spins a full 360° left, then 360° right |
| `square.csv` | Flies a 3 m square, turning 90° left at each corner |
| `full_tour.csv` | Every move once, in order |
| `drift.csv` | *Tries* to hover while a steady wind (0.3 m/s east, 0.15 m/s north) pushes it away |
| `random_walk.csv` | Makes 12 random moves (picked from `--seed`) |

To make a new one, add a list of `(Move, seconds)` to `SCENARIOS` in the script.
To add wind to a scenario, give it an entry in `DRIFT_MPS`.

`drift` and `random_walk` are for robustness testing: `drift` checks that a
consumer does not assume "commanded hover" means "not moving", and
`random_walk` produces an unpredictable but exactly reproducible path (same
`--seed` gives the same file).

## Details

- Commanded speeds: **1 m/s** horizontal, **0.5 m/s** vertical, **45°/s** yaw
  (`HORIZONTAL_SPEED_MPS` etc. in the script). Takeoff and landing run at
  Motion's own limits (2 m/s vertical).
- Forward/back/left/right are **relative to where the drone faces** when the
  move starts. After a 90° yaw-left, "forward" goes north.
- `DOWN` moves stop at `MotionLimits.velocity_floor_m` (1 m). Landing is the
  `LAND` command at the end, not a `DOWN` move.
- Wind (`drift` only) is an environment overlay: Motion does not model wind,
  so the generator adds it to the reported position and velocity while
  airborne. The `command_kind` column still says `HOVER`.
- `Move` is only the script's shorthand for writing scenarios. It is not a
  command format; every move is sent as a `contracts.Command`.
- Frame matches `motion_stubs.py`: x = east, y = north, z = up (m); yaw 0 = east, counter-clockwise positive.

## Metadata header

Each CSV starts with `# key=value` lines describing how it was made:

```text
# schema_version=2
# scenario=drift
# rate_hz=10.0
# dt_s=0.1
# noise_m=0.0
# seed=0
# drift_mps=0.3,0.15
# moves=1
# takeoff_altitude_m=2.0
# duration_s=12.1
# frame=local_enu
# clock=simulation
# synthetic=true
# motion_model=motion_engine.motion_stubs.MotionStub via contracts.Command
# generator=motion_engine/synthetic/generate_motion_data.py
t_s,timestamp_ns,step,direction,command_id,command_kind,status,...
```

`schema_version=2` marks the move to MotionStub: the motor columns are gone
and the command columns are new.

Plain `csv.DictReader` does not skip `#` lines, so read the files with the
helper in the script or with pandas:

```python
from generate_motion_data import load_csv          # motion_engine/synthetic on sys.path
meta, rows = load_csv("motion_engine/motion_logs/square.csv")
meta["scenario"], meta["rate_hz"]                   # ('square', '10.0')

import pandas as pd
df = pd.read_csv("motion_engine/motion_logs/square.csv", comment="#")
```

## Columns

| Column | Meaning |
|---|---|
| `t_s` | time (s), 10 samples per second by default |
| `timestamp_ns` | the same time as an integer number of simulation nanoseconds from 0, the shared clock in [INTEGRATION_README.md](../../INTEGRATION_README.md) section 4 |
| `step` | index of the move in the scenario; `-1` = pad and takeoff, `len(moves)` = landing |
| `direction` | the move being held (`hover`, `up`, `yaw_left`, …) or `pad` / `takeoff` / `land` / `disarm` |
| `command_id`, `command_kind` | the command Motion is executing (`VehicleFeedback.active_command_id` / `active_kind`) |
| `status` | `VehicleFeedback.status`: `in_progress`, `arrived`, `landed`, ... |
| `armed`, `airborne` | from `VehicleFeedback` |
| `x_m`, `y_m`, `z_m` | position (m), including wind and noise |
| `yaw_deg` | heading, 0–360 |
| `vx_mps`, `vy_mps`, `vz_mps`, `yaw_rate_dps` | the drone's actual velocity over the last tick (world frame, includes wind) |

## Expected output

Running the script with no arguments prints one line per scenario:

```text
up_down            102 rows,  10.1 s, ends at x=+0.00 y=+0.00 z=+0.00 yaw=0
left_right         112 rows,  11.1 s, ends at x=+0.00 y=+0.00 z=+0.00 yaw=0
forward_backward   112 rows,  11.1 s, ends at x=+0.00 y=+0.00 z=+0.00 yaw=0
rotate             212 rows,  21.1 s, ends at x=+0.00 y=+0.00 z=+0.00 yaw=0
square             242 rows,  24.1 s, ends at x=+0.00 y=+0.00 z=+0.00 yaw=0
full_tour          202 rows,  20.1 s, ends at x=+0.00 y=+0.00 z=+0.00 yaw=0
drift              122 rows,  12.1 s, ends at x=+3.57 y=+1.78 z=+0.00 yaw=0
random_walk        207 rows,  20.6 s, ends at x=+4.50 y=-1.00 z=+0.00 yaw=292
plot -> motion_engine/motion_logs/overview.png
```

Every hand-written scenario ends back at the origin; `drift` and
`random_walk` do not, which is the point. The last rows of `square.csv`
show the landing settling and the disarm:

```text
t_s,timestamp_ns,step,direction,command_id,command_kind,status,armed,airborne,x_m,y_m,z_m,yaw_deg,vx_mps,vy_mps,vz_mps,yaw_rate_dps
23.9,23900000000,10,land,gen-3,LAND,arrived,True,True,0.0,0.0,0.2,0.0,0.0,0.0,-2.0,0.0
24.0,24000000000,10,land,gen-3,LAND,landed,True,False,0.0,0.0,0.0,0.0,0.0,0.0,-2.0,0.0
24.1,24100000000,10,disarm,gen-4,DISARM,landed,False,False,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0
```

Note `arrived` at 0.2 m: arrival tolerance is not a landing. Only `landed`
(`landing_settled`) is.

Check any of the files against the motion rules with the trace checker:

```bash
py -3 -m motion_engine.trace_checker motion_engine/motion_logs/*.csv
```

`overview.png` (written when matplotlib is installed) has one row per
scenario: x/y/z position against time on the left, heading on the right.
To plot a single file yourself:

```python
import matplotlib.pyplot as plt
import pandas as pd

df = pd.read_csv("motion_engine/motion_logs/drift.csv", comment="#")
fig, (pos, top) = plt.subplots(1, 2, figsize=(10, 3.5))
for col in ("x_m", "y_m", "z_m"):
    pos.plot(df.t_s, df[col], label=col)
pos.set_xlabel("time (s)"); pos.set_ylabel("m"); pos.legend()
top.plot(df.x_m, df.y_m); top.set_xlabel("x east (m)"); top.set_ylabel("y north (m)")
top.set_aspect("equal"); top.set_title("ground track")
plt.tight_layout(); plt.show()
```
