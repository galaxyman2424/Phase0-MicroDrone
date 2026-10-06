"""Memory tests for the mission loop against the Jetson Nano's 4 GB.

The fast tests (default run, a few seconds) use tracemalloc to check that the
loop itself does not hold on to memory. The long ones run only with
STRESS=1, because they take minutes:

    python3 -m pytest state_machine -v                # fast
    STRESS=1 python3 -m pytest state_machine -v       # + long stress runs
    (PowerShell: $env:STRESS=1; py -3 -m pytest state_machine -v)

For a full report with numbers per case, run `python3 -m state_machine.stress`.
"""

from collections import deque
import gc
import json
import os
import subprocess
import sys
import tracemalloc
from pathlib import Path

import pytest

from state_machine.run_loop import TICK_MS, init_world, run_world
from state_machine.scenarios import padded_payload, patrol_cycles, random_fuzz, sensor_flapping
from state_machine.state_machine import State, step

KB = 1024
MB = 1024 * 1024
HOUR_MS = 3_600_000

# RAM our code may use: 4 GB Jetson Nano minus what the OS/desktop/GPU take.
# The reserve is an assumption; override it once it is measured on the device.
TOTAL_MB = float(os.environ.get("RAM_TOTAL_MB", 4096))
RESERVE_MB = float(os.environ.get("RAM_RESERVE_MB", 1024))
BUDGET_MB = TOTAL_MB - RESERVE_MB

REPO_ROOT = Path(__file__).resolve().parents[2]
stress = pytest.mark.skipif(os.environ.get("STRESS") != "1",
                            reason="long stress run; set STRESS=1 to include")


class Heap:
    """tracemalloc around a block: Heap.mark() records the current Python heap."""

    def __enter__(self):
        gc.collect()
        tracemalloc.start()
        self.marks = []
        return self

    def mark(self):
        self.marks.append(tracemalloc.get_traced_memory()[0])

    def __exit__(self, *exc):
        tracemalloc.stop()


def run_with_marks(duration_ms, events, mark_at_ms, on_tick=None):
    """run_world() and record the heap at each time in mark_at_ms (and at the end)."""
    with Heap() as heap:
        def hook(t, snap, state):
            if on_tick:
                on_tick(t, snap, state)
            if t in mark_at_ms:
                heap.mark()
        transitions = run_world(duration_ms, scenario=events, on_tick=hook)
        heap.mark()
    return heap.marks, transitions


# --------------------------------------------------------------------------
# The loop itself must not keep memory
# --------------------------------------------------------------------------

def test_step_keeps_no_reference_to_observations():
    observation = init_world()
    observation["bird"]["detected"] = True
    with Heap() as heap:
        state = State.IDLE
        heap.mark()
        for _ in range(50_000):
            state, _ = step(state, observation)
        heap.mark()
    assert heap.marks[1] - heap.marks[0] < 16 * KB


def test_loop_heap_is_flat_over_one_hour_of_missions():
    # 60 full missions. Between minute 15 and minute 60 only the transitions list may grow
    # (7 per mission), so anything beyond a few KB per mission is a leak.
    marks, transitions = run_with_marks(HOUR_MS, patrol_cycles(60), {15 * 60_000})
    missions_between = 45
    assert len(transitions) == 7 * 60
    assert (marks[1] - marks[0]) / missions_between < 2 * KB, f"{(marks[1]-marks[0])/KB:.0f} KB"


def test_transitions_list_cost_stays_small_even_when_flapping():
    # Worst case: a state change every tick. run_world keeps every transition, so this is
    # the one thing in the loop that grows. Check the cost per transition and project 8 h.
    marks, transitions = run_with_marks(10 * 60_000, sensor_flapping(10 * 60_000), {60_000})
    per_transition = (marks[1] - marks[0]) / (len(transitions) - 60_000 // TICK_MS)
    eight_hours_mb = per_transition * 8 * HOUR_MS / TICK_MS / MB
    assert per_transition < 400, f"{per_transition:.0f} B per transition"
    assert eight_hours_mb < 0.05 * BUDGET_MB, f"{eight_hours_mb:.0f} MB after 8 h of flapping"


def test_random_inputs_do_not_grow_the_heap():
    duration = 30 * 60_000
    events = random_fuzz(duration, seed=1, events_per_s=10)
    marks, transitions = run_with_marks(duration, events, {10 * 60_000})
    # Allow ~200 B per transition recorded after the 10-minute mark plus a little slack
    later = sum(1 for t, *_ in transitions if t > 10 * 60_000)
    assert marks[1] - marks[0] < later * 200 + 64 * KB


# --------------------------------------------------------------------------
# What future modules could cost
# --------------------------------------------------------------------------

def test_snapshot_history_must_be_bounded():
    # A module that keeps every tick's observation (e.g. for logging or replay) grows
    # forever; a ring buffer does not. This pins down both so the trade-off is visible.
    keep_all, ring = [], deque(maxlen=600)  # ring = last 60 s
    events = patrol_cycles(20)
    all_marks, _ = run_with_marks(20 * 60_000, events, {10 * 60_000},
                                  on_tick=lambda t, s, st: keep_all.append(s))
    del keep_all
    ring_marks, _ = run_with_marks(20 * 60_000, events, {10 * 60_000},
                                   on_tick=lambda t, s, st: ring.append(s))

    per_hour_mb = (all_marks[1] - all_marks[0]) * 6 / MB  # 10 min -> 1 h
    assert per_hour_mb > 1, "keeping every snapshot should visibly grow"
    assert per_hour_mb * 8 < 0.25 * BUDGET_MB, f"{per_hour_mb:.0f} MB/h of kept snapshots"
    assert ring_marks[1] - ring_marks[0] < 64 * KB


def test_bigger_observation_does_not_leak():
    # 4 future modules adding 8 KB each: the world is deep-copied every tick, which costs
    # time (see the stress report) but must not keep memory around.
    events = padded_payload(8) + patrol_cycles(2)
    marks, _ = run_with_marks(90_000, events, {30_000})
    assert marks[1] - marks[0] < 64 * KB


@pytest.mark.xfail(strict=True, reason=(
    "Known issue: MotionStub appends every command to _history (and every contract id to "
    "_seen_ids) forever, ~500 B per command. track_target() runs every tick while TRACKING, "
    "so that is ~18 MB per hour of tracking at 10 Hz (more at Phase 1's 20-50 Hz). "
    "Fix: keep a bounded deque for history."))
def test_motion_stub_memory_is_bounded_while_tracking():
    motion_stubs = pytest.importorskip("motion_engine.motion_stubs")
    motion = motion_stubs.MotionStub(logger=None)
    motion.arm()
    motion.takeoff(10.0)
    with Heap() as heap:
        for i in range(36_000):  # one hour of tracking at 10 Hz
            if i == 6_000:
                heap.mark()
            motion.track_target(0.0, 30.0)
            motion.step(TICK_MS / 1000)
        heap.mark()
    assert heap.marks[1] - heap.marks[0] < 256 * KB


# --------------------------------------------------------------------------
# Whole pipeline in a separate process (real RSS)
# --------------------------------------------------------------------------

def run_workload(name, **params):
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join(filter(None, [str(REPO_ROOT), env.get("PYTHONPATH")]))
    proc = subprocess.run([sys.executable, "-m", "state_machine.stress.workloads", name,
                           json.dumps(params)], cwd=REPO_ROOT, env=env,
                          capture_output=True, text=True, timeout=1800)
    assert proc.returncode == 0, proc.stderr[-2000:]
    return json.loads(proc.stdout.strip().splitlines()[-1])


def test_pipeline_one_minute_fits_easily():
    pytest.importorskip("numpy")
    pytest.importorskip("cv2")
    r = run_workload("pipeline", minutes=1)
    assert r["ticks"] == 600
    assert r["peak_rss_mb"] < 0.1 * BUDGET_MB, f"{r['peak_rss_mb']:.0f} MB"


@stress
def test_state_machine_eight_hour_day_is_flat():
    r = run_workload("sm_patrol", hours=8)
    assert r["leak_mb_per_hour"] < 0.5, r["leak_mb_per_hour"]
    assert r["peak_rss_mb"] < 0.05 * BUDGET_MB


@stress
def test_pipeline_one_hour_projected_to_eight_hours_fits_budget():
    pytest.importorskip("cv2")
    r = run_workload("pipeline", minutes=60, motion_log="null")
    projected = r["end_rss_mb"] + max(0, r["leak_mb_per_hour"] or 0) * 7
    assert r["ticks_over_budget"] == 0
    assert projected < 0.5 * BUDGET_MB, f"{projected:.0f} MB projected after 8 h"


@stress
def test_pipeline_with_model_stand_in_fits_budget():
    # Room for a future ~512 MB detector next to everything that runs today
    pytest.importorskip("cv2")
    r = run_workload("pipeline", minutes=10, reserve_mb=512)
    assert r["peak_rss_mb"] < BUDGET_MB, f"{r['peak_rss_mb']:.0f} MB"


@stress
def test_two_hd_cameras_with_frame_buffer_fit_budget():
    pytest.importorskip("cv2")
    r = run_workload("pipeline", minutes=10, cameras=2, width=1280, height=720, frame_buffer_s=2)
    assert r["peak_rss_mb"] < 0.5 * BUDGET_MB, f"{r['peak_rss_mb']:.0f} MB"
