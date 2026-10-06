# Memory and stress harness

The Jetson Nano has **4 GB of RAM shared by the OS, the GPU and our code**. This harness measures what the 10 Hz mission loop costs today and how that cost grows with longer runs, bigger observations, more cameras and kept history, so we find out before a new module tips us over.

## Run it

From the repo root:

```bash
python3 -m state_machine.stress                  # quick suite, about 1 minute
python3 -m state_machine.stress --suite full     # long runs, about 15 minutes
python3 -m state_machine.stress --list           # every case and what it shows
python3 -m state_machine.stress --only pipeline_10min,sm_patrol_1h
python3 -m state_machine.stress --json out.json --markdown out.md
```

(Windows: `py -3 -m state_machine.stress`.)

Every case runs in its own Python process, so its peak memory is its own. `psutil` is used when it is installed; on Linux (the Jetson) nothing extra is needed. The pipeline cases need `numpy` and `opencv-python` (see `cv/requirements.txt`).

Tests:

```bash
python3 -m pytest state_machine -v               # fast memory tests (~15 s)
STRESS=1 python3 -m pytest state_machine -v      # + long runs (several minutes)
```

PowerShell: `$env:STRESS=1; py -3 -m pytest state_machine -v`.

## Reading the report

| Column | Meaning |
|---|---|
| ms/tick | Wall time per loop iteration. The loop has 100 ms per tick. |
| start MB | RSS after imports and setup, before the loop starts (interpreter + libraries). |
| peak MB | Highest RSS reached. |
| growth MB | RSS at the end minus at the start. |
| leak MB/h | Slope of RSS over **simulated** hours, measured on the second half of the run (warm-up is ignored). Near 0 means flat. |
| 8 h proj. MB | End RSS plus the leak rate extrapolated to an 8-hour day (`--project-hours`). |
| % budget | The worse of peak and projection, against the budget. `WATCH` above 50%, `OVER` above 100%. |

**Budget.** Default is 4096 MB total with 1024 MB assumed for the OS, desktop and GPU, leaving 3072 MB for us. The reserve is a placeholder: measure `free -m` on the Nano with JetPack running and pass `--reserve-mb`. In tests, set `RAM_TOTAL_MB` / `RAM_RESERVE_MB`.

**Timing caveat.** ms/tick is measured on whatever machine runs it. The Nano's ARM cores are much slower than a laptop, so treat any case above roughly 10 ms/tick on a laptop as a risk on the device. Memory sizes carry over closely (both are 64-bit).

## Cases

Quick suite:

- `sm_scenarios_x200`: every named scenario in `state_machine/scenarios.py`, 200 times.
- `sm_patrol_1h`: 60 back-to-back missions. Flat memory means the loop does not leak.
- `sm_fuzz_1h`: random module outputs, 5 events/s.
- `sm_flapping_1h`: a bird flickering every tick, the most transitions possible.
- `sm_payload_64kb`: the shared observation grows by 64 KB for each of 4 future modules. `run_world` deep-copies it every tick.
- `sm_history_all_10min` / `sm_history_ring_10min`: keeping every tick's snapshot compared with keeping only the last 60 s.
- `pipeline_10min`: the real modules in one loop: CV synthetic frames (640x480 @ 15 fps + grayscale), Safety `assess()`, `DockingController`, the state machine and `MotionStub`.

The full suite adds 8-hour versions, a 1 MB observation, two 720p and two 1080p cameras with frame buffers, motion logs kept compared with dropped, and a 512 MB stand-in for a future detector model.

In the pipeline, decisions still come from the scripted world, as in `run_loop.py`. The modules run so that their memory and CPU cost is real, not so that their outputs drive the state machine.

## Adding a module

When a module joins the loop:

1. Call it inside `pipeline()` in `workloads.py`, once per tick, the way the coordinator will.
2. If it can keep data (buffers, history, logs), add a parameter for how much it keeps and a case in `__main__.py` with a realistic and a worst-case setting.
3. Add a test in `state_machine/tests/test_memory.py` that fails if the module's memory keeps growing (see `test_motion_stub_memory_is_bounded_while_tracking`).
4. Run `python3 -m state_machine.stress --suite full` and compare with the last report.

## Baseline

[BASELINE.md](BASELINE.md) has the full-suite numbers from 2026-10-06. In short: the loop itself is flat (8 h of missions adds about 1 MB), and the real modules together use about 60 MB. What can actually fill 4 GB is anything that keeps data: frame buffers (two 1080p cameras with 5 s each reach about 1.25 GB), kept snapshots (about 50 MB per hour) and unbounded logs or history. The CPU risk is the per-tick `copy.deepcopy` of the world, which takes about 7 ms for a 265 KB observation and about 30 ms for 1 MB, before the Nano's slower cores.

## Known issues found by these tests

Both are pinned as `xfail(strict=True)` tests. The test starts failing (XPASS) once the issue is fixed, as a reminder to remove the marker.

- **MotionStub keeps every command forever** (`_history`, `_seen_ids`). About 500 B per command, and `track_target()` runs every tick while tracking, so that is about 18 MB per hour of tracking. With `MemoryLogSink` it is about 42 MB/h. Phase 1 streams setpoints at 20-50 Hz, which makes this 2-5x worse. Fix: a bounded `deque` for history.
- **Held operator abort makes the state machine flap.** While Safety keeps `abort=True`, ABORT → DOCKING_INIT (hold permitted) → ABORT (global abort rule) repeats every tick (scenario `operator_abort_held`).
