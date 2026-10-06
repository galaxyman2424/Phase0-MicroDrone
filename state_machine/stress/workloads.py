"""Workloads for the memory/stress harness.

Each workload is a function (probe, **params) -> dict of extra facts. It must
call probe.start() once setup and imports are done, probe.tick() per loop
iteration and probe.checkpoint() roughly once per simulated minute.

Run one directly (this is what the harness does, one process per case):

    python3 -m state_machine.stress.workloads sm_patrol '{"hours": 1}'
"""

from collections import deque
import copy
import json
import sys
import time

from state_machine.run_loop import TICK_MS, init_world, run_world
from state_machine.scenarios import (SCENARIOS, padded_payload, patrol_cycles, random_fuzz,
                                     sensor_flapping)
from state_machine.state_machine import State, step
from state_machine.stress.memory import Probe

CHECKPOINT_TICKS = 600  # one simulated minute at 10 Hz


def _hours_to_ms(hours: float) -> int:
    return int(hours * 3600 * 1000) // TICK_MS * TICK_MS


def _timed_run(probe: Probe, duration_ms: int, events, on_tick=None) -> list:
    """run_world() with per-tick timing and memory checkpoints."""
    last = [None]

    def hook(t_ms, snapshot, state):
        now = time.perf_counter()
        if last[0] is not None:
            probe.tick(now - last[0])
        last[0] = now
        if on_tick is not None:
            on_tick(t_ms, snapshot, state)
        if (t_ms // TICK_MS) % CHECKPOINT_TICKS == 0:
            probe.checkpoint(t_ms)

    probe.start()
    transitions = run_world(duration_ms, scenario=events, on_tick=hook)
    probe.checkpoint(duration_ms)
    probe.finish()  # before the caller's data (e.g. kept history) can be freed
    return transitions


# --------------------------------------------------------------------------
# State machine only
# --------------------------------------------------------------------------

def sm_scenarios(probe: Probe, repeats: int = 200) -> dict:
    """Every named demo scenario, `repeats` times each (baseline cost)."""
    probe.start()
    total = 0
    sim_ms = 0
    for _ in range(repeats):
        for s in SCENARIOS.values():
            t0 = time.perf_counter()
            total += len(run_world(s.duration_ms, scenario=s.events))
            elapsed = time.perf_counter() - t0
            ticks = s.duration_ms // TICK_MS
            # run_world has no per-tick hook here, so book the run's mean tick time
            probe.tick_count += ticks
            probe.tick_total_s += elapsed
            probe.tick_max_s = max(probe.tick_max_s, elapsed / ticks)
            sim_ms += s.duration_ms
        probe.checkpoint(sim_ms)
    probe.finish()
    return {"transitions": total, "sim_s": sim_ms / 1000}


def sm_patrol(probe: Probe, hours: float = 1.0) -> dict:
    """Back-to-back one-minute missions (launch, track, dock, recharge)."""
    cycles = max(1, round(hours * 60))
    events = patrol_cycles(cycles)
    transitions = _timed_run(probe, cycles * 60_000, events)
    return {"transitions": len(transitions), "sim_s": cycles * 60, "events": len(events)}


def sm_fuzz(probe: Probe, hours: float = 1.0, events_per_s: float = 5.0, seed: int = 0) -> dict:
    """Random module outputs at `events_per_s`; checks step() never raises."""
    duration = _hours_to_ms(hours)
    events = random_fuzz(duration, seed=seed, events_per_s=events_per_s)
    transitions = _timed_run(probe, duration, events)
    return {"transitions": len(transitions), "sim_s": duration / 1000, "events": len(events)}


def sm_flapping(probe: Probe, hours: float = 1.0) -> dict:
    """Bird detection toggles every tick: maximum transition churn."""
    duration = _hours_to_ms(hours)
    events = sensor_flapping(duration)
    transitions = _timed_run(probe, duration, events)
    return {"transitions": len(transitions), "sim_s": duration / 1000, "events": len(events)}


def sm_payload(probe: Probe, minutes: float = 10, kb_per_module: int = 64) -> dict:
    """Observation padded with `kb_per_module` KB for each of 4 future modules.

    run_world deep-copies the whole world every tick, so this shows what a
    bigger shared observation costs in time and memory.
    """
    duration = _hours_to_ms(minutes / 60)
    events = padded_payload(kb_per_module) + patrol_cycles(max(1, int(minutes)))
    transitions = _timed_run(probe, duration, events)
    world = init_world()
    for _, change in events[:1]:
        world.update(copy.deepcopy(change))
    size = len(json.dumps(world))
    return {"transitions": len(transitions), "sim_s": duration / 1000,
            "observation_json_kb": round(size / 1024, 1)}


def sm_history(probe: Probe, minutes: float = 10, keep: str = "all", ring_s: float = 60) -> dict:
    """Keep every tick's snapshot in memory (keep="all") or only the last `ring_s` s.

    This is what happens if a module (logging, replay, telemetry) holds on to
    observations instead of writing them out.
    """
    duration = _hours_to_ms(minutes / 60)
    history = [] if keep == "all" else deque(maxlen=int(ring_s * 1000 / TICK_MS))
    events = patrol_cycles(max(1, int(minutes)))
    _timed_run(probe, duration, events,
               on_tick=lambda t, snap, state: history.append((t, state, snap)))
    return {"sim_s": duration / 1000, "kept_snapshots": len(history)}


# --------------------------------------------------------------------------
# Full module pipeline
# --------------------------------------------------------------------------

def pipeline(probe: Probe, minutes: float = 10, cameras: int = 1, width: int = 640,
             height: int = 480, fps: float = 15, frame_buffer_s: float = 0,
             motion_log: str = "memory", reserve_mb: int = 0) -> dict:
    """The real Phase 0 modules in one 10 Hz loop, driven by patrol missions.

    Per tick: CV synthetic frames (+ grayscale) for each camera, Safety's
    assess(), the Docking controller, the state machine and MotionStub.
    Decisions still come from the scripted world (as in run_loop.py); the
    modules run so their memory and CPU cost is real.

    frame_buffer_s: keep the last N seconds of frames per camera (e.g. for a
        future detector that looks at several frames).
    motion_log: "memory" keeps every motion log record (contracts.MemoryLogSink,
        what the tests use); "null" serialises records like UniversalLogSink
        but discards them, so no disk is touched.
    reserve_mb: allocate and touch this much extra memory up front, standing
        in for a future model (e.g. a TensorRT bird detector).
    """
    import numpy as np
    import cv2
    from cv import synthetic_frames
    from safety_layer.safety_policy import assess
    from docking.states import DockingController, DockingInput, DockingState
    from motion_engine.motion_stubs import MotionStub
    from motion_engine.contracts import MemoryLogSink, Pose
    from motion_engine.universal_logging import UniversalLogSink

    synthetic_frames.WIDTH, synthetic_frames.HEIGHT = width, height
    scenario = json.loads((synthetic_frames.Path(synthetic_frames.__file__).parent /
                           "scenarios" / "integration_cycle.json").read_text(encoding="utf-8"))
    scenario["loop"] = True
    scenario["fps"] = fps
    sources = [synthetic_frames.SyntheticSource(scenario, camera_id=cid)
               for cid in ("front", "downward")[:cameras]]
    buffers = [deque(maxlen=max(1, int(frame_buffer_s * fps))) for _ in sources] \
        if frame_buffer_s > 0 else None
    frames_per_tick = fps * TICK_MS / 1000

    if motion_log == "memory":
        sink = MemoryLogSink()
    else:
        sink = UniversalLogSink(write=lambda record: json.dumps(record))
    sim_clock = [0.0]
    motion = MotionStub(logger=sink, clock=lambda: sim_clock[0])
    home = Pose(0.0, 0.0, 0.0, 0.0)
    docking = DockingController()

    reserve = None
    if reserve_mb:
        reserve = np.ones(reserve_mb * 1024 * 1024, dtype=np.uint8)  # touched, so resident

    duration = _hours_to_ms(minutes / 60)
    cycles = max(1, int(minutes))
    from state_machine.run_loop import index_scenario
    events = index_scenario(patrol_cycles(cycles))
    world = init_world()
    state = State.IDLE
    transitions = 0
    frame_index = [0] * len(sources)
    frame_debt = 0.0
    frames_made = 0

    probe.start()
    for tick in range(duration // TICK_MS):
        t0 = time.perf_counter()
        now_ms = tick * TICK_MS
        now_ns = now_ms * 1_000_000
        sim_clock[0] = now_ms / 1000

        for change in events.get(now_ms, ()):
            for section, values in change.items():
                world.setdefault(section, {}).update(values)

        # CV: produce the frames that fall due this tick
        frame_debt += frames_per_tick
        while frame_debt >= 1:
            frame_debt -= 1
            for i, src in enumerate(sources):
                frame, _obs = src.sample(frame_index[i])
                frame_index[i] += 1
                frames_made += 1
                if frame is not None:
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    if buffers is not None:
                        buffers[i].append((frame, gray))

        # Safety: assess a fresh synthetic snapshot
        fresh = {"valid": True, "timestamp_ns": now_ns}
        assessment = assess({
            "vehicle": dict(fresh),
            "battery": {**fresh, "percent": world["battery"]["percent"]},
            "environment": {**fresh, "people_nearby": False, "cart_moving": False,
                            "camera_ok": True, "comm_ok": True, "weather": "clear"},
            "dock": {**fresh, "alignment_valid": True},
            "context": {"flight_active": motion.airborne},
        }, now_ns)

        # Docking controller runs while the mission is approaching
        if docking.state in (DockingState.COMPLETE, DockingState.ABORT):
            docking = DockingController()
        if state == State.DOCKING_APPROACH:
            docking.update(DockingInput(docking_requested=True, near_pad=True,
                                        alignment_valid=True, alignment_timestamp_s=now_ms / 1000,
                                        north_error_m=0.0, east_error_m=0.0,
                                        landing_confirmed=bool(world["docking"].get("landed_on_pad"))),
                           now_ms / 1000)

        # State machine
        snapshot = copy.deepcopy(world)
        snapshot["timestamp_ns"] = now_ns
        snapshot["safety_assessment"] = assessment.permissions
        new_state, _reason = step(state, snapshot)
        entered = new_state != state
        if entered:
            transitions += 1
        prev, state = state, new_state

        # Motion acts on the state (README "Motion action (proposed)")
        motion.set_log_context(state=state.value)
        if state == State.HOVERING and prev == State.IDLE:
            motion.arm()
            motion.takeoff(12.0)
        elif state in (State.HOVERING, State.DOCKING_INIT) and entered:
            motion.hover()
        elif state == State.TRACKING:
            motion.track_target(bearing_deg=0.0, distance_m=world["bird"].get("distance_m") or 30.0)
        elif state == State.DOCKING_APPROACH:
            if entered:
                motion.goto_dock_approach(home)
            elif world["docking"].get("phase") == "descend" and motion.at_target():
                motion.descend_to_dock(home)
        elif state == State.DOCKED and entered:
            motion.land()
        elif state == State.IDLE and entered:
            motion.disarm()
        elif state == State.EMERGENCY_LAND and entered:
            motion.emergency_land()
        motion.step(TICK_MS / 1000, now_ns=now_ns)

        probe.tick(time.perf_counter() - t0)
        if tick % CHECKPOINT_TICKS == 0:
            probe.checkpoint(now_ms)
    probe.checkpoint(duration)
    probe.finish()  # frame buffers, logs and the reserve are still alive here

    return {
        "sim_s": duration / 1000,
        "transitions": transitions,
        "frames": frames_made,
        "frame_mb": round(width * height * 3 / 1024 / 1024, 2),
        "motion_history_len": len(motion.history),
        "motion_log_records": len(sink.records) if motion_log == "memory" else None,
        "reserve_mb": reserve_mb,
        "reserve_alive": reserve is not None,
    }


WORKLOADS = {f.__name__: f for f in
             [sm_scenarios, sm_patrol, sm_fuzz, sm_flapping, sm_payload, sm_history, pipeline]}


def run(name: str, params: dict, trace_heap: bool = False) -> dict:
    probe = Probe(trace_heap=trace_heap)
    extra = WORKLOADS[name](probe, **params)
    return {**probe.result(), **extra}


if __name__ == "__main__":
    name = sys.argv[1]
    params = json.loads(sys.argv[2]) if len(sys.argv) > 2 else {}
    trace = "--trace-heap" in sys.argv
    print(json.dumps(run(name, params, trace_heap=trace)))
