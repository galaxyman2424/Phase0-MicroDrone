"""Demo and stress scenarios for the state machine loop (run_loop.py).

A scenario uses the same format as run_loop.DEMO_SCENARIO: a list of
(time_ms, changes) tuples, where `changes` only lists the fields that change
at that time. Event times must be multiples of run_loop.TICK_MS (100 ms).

Two kinds live here:

* Named demo scenarios (SCENARIOS) that each exercise one part of the
  transition table, with the transitions they are expected to produce checked
  in state_machine/tests/test_scenarios.py.
* Generators (patrol_cycles, sensor_flapping, random_fuzz, ...) that build long
  or dense scenarios for the memory/stress harness in state_machine/stress/.

Run one from the repo root:

    python3 -m state_machine.scenarios                 # list scenarios
    python3 -m state_machine.scenarios cart_moving_hold
"""

from dataclasses import dataclass
import random
import sys

from state_machine.run_loop import DEMO_SCENARIO, TICK_MS, run_world

ACTIONS = ("launch", "tracking", "approach", "descent", "hold")


def perms(*blocked: str) -> dict:
    # Safety's permissions with every action allowed except the blocked ones
    return {"permissions": {action: action not in blocked for action in ACTIONS}}


def safety(*blocked: str, **fields) -> dict:
    # A "safety" change: permissions plus any other SafetyAssessment fields
    return {"safety": {**perms(*blocked), **fields}}


# What Safety blocks for a moving cart or people nearby (INTEGRATION_README.md section 6)
HAZARD = ("launch", "tracking", "approach", "descent")
BIRD = {"bird": {"detected": True, "type": "hawk", "distance_m": 30.0}}
NO_BIRD = {"bird": {"detected": False, "type": None, "distance_m": None}}
ALL_CLEAR = safety(emergency=False, abort=False, battery_low=False, battery_critical=False,
                   battery_recovered=True, reasons=[])


def merge(*changes: dict) -> dict:
    # Combine several change dicts into one event (later sections win)
    out: dict = {}
    for change in changes:
        for section, values in change.items():
            out.setdefault(section, {}).update(values)
    return out


@dataclass(frozen=True)
class Scenario:
    name: str
    description: str
    events: list
    duration_ms: int


# --------------------------------------------------------------------------
# Named demo scenarios
# --------------------------------------------------------------------------

def _bird_flyby_return():
    # A bird comes and goes; Docking asks to return because no bird is left.
    return [
        (500, BIRD),
        (3000, NO_BIRD),
        (4000, {"docking": {"should_dock": True, "reason": "no_bird"}}),
        (4500, {"docking": {"phase": "approach"}}),
        (5500, {"docking": {"phase": "descend"}}),
        (6000, {"docking": {"phase": "complete", "landed_on_pad": True, "should_dock": False, "reason": None}}),
    ]


def _cart_moving_hold():
    # Docking is requested while the cart is moving: hold in DOCKING_INIT until it stops.
    return [
        (500, BIRD),
        (2000, merge({"docking": {"should_dock": True, "reason": "cart_moving"}},
                     safety(*HAZARD, reasons=["cart_moving"]))),
        (6000, safety(reasons=[])),  # cart stopped
        (6500, {"docking": {"phase": "approach"}}),
        (7500, {"docking": {"phase": "descend"}}),
        (8000, merge(NO_BIRD, {"docking": {"phase": "complete", "landed_on_pad": True,
                                           "should_dock": False}})),
    ]


def _docking_failure_retry():
    # The first approach fails (Docking phase abort), ABORT retries docking, the second one lands.
    return [
        (500, BIRD),
        (1500, merge(NO_BIRD, {"docking": {"should_dock": True, "reason": "no_bird"}})),
        (2000, {"docking": {"phase": "approach"}}),
        (3000, {"docking": {"phase": "abort"}}),
        (3100, {"docking": {"phase": "search"}}),   # Docking restarts its attempt
        (4000, {"docking": {"phase": "align"}}),
        (4500, {"docking": {"phase": "descend"}}),
        (5000, {"docking": {"phase": "complete", "landed_on_pad": True, "should_dock": False}}),
    ]


def _people_at_pad_abort():
    # People walk up to the pad during the approach: ABORT, wait in DOCKING_INIT, then land.
    return [
        (500, BIRD),
        (1500, merge(NO_BIRD, {"docking": {"should_dock": True, "reason": "no_bird"}})),
        (2000, {"docking": {"phase": "approach"}}),
        (2500, safety(*HAZARD, reasons=["people_nearby"])),
        (5000, merge(safety(reasons=[]), {"docking": {"phase": "approach"}})),
        (6000, {"docking": {"phase": "descend"}}),
        (6500, {"docking": {"phase": "complete", "landed_on_pad": True, "should_dock": False}}),
    ]


def _critical_battery_emergency():
    # Battery goes critical while tracking: Safety raises an emergency, EMERGENCY_LAND is final.
    return [
        (500, BIRD),
        (2000, merge({"battery": {"percent": 18}},
                     safety("launch", "tracking", battery_low=True, battery_recovered=False,
                            reasons=["low_battery"]))),
        (3000, merge({"battery": {"percent": 9}},
                     safety(*ACTIONS, emergency=True, battery_low=True, battery_critical=True,
                            battery_recovered=False, reasons=["critical_battery"]))),
    ]


def _operator_abort_pulse():
    # The operator presses stop for one tick while tracking; the drone aborts and docks.
    return [
        (500, BIRD),
        (2000, merge(safety(*HAZARD, abort=True, reasons=["operator_abort"]),
                     {"docking": {"should_dock": True, "reason": "operator_abort"}})),
        (2100, safety(abort=False, reasons=[])),
        (2500, {"docking": {"phase": "approach"}}),
        (3500, {"docking": {"phase": "descend"}}),
        (4000, merge(NO_BIRD, {"docking": {"phase": "complete", "landed_on_pad": True,
                                           "should_dock": False}})),
    ]


def _operator_abort_held():
    # Same as the pulse, but Safety keeps abort=True for 2 s (the button is held).
    # Known issue: ABORT -> DOCKING_INIT (hold permitted) -> ABORT (global abort rule)
    # flaps every tick while abort stays set. See test_scenarios.py.
    return [
        (500, BIRD),
        (2000, merge(safety(*HAZARD, abort=True, reasons=["operator_abort"]),
                     {"docking": {"should_dock": True, "reason": "operator_abort"}})),
        (4000, safety(abort=False, reasons=[])),
    ]


def _camera_failure_hold():
    # The camera drops out while tracking: hover until it is back, then track again.
    return [
        (500, BIRD),
        (2000, safety("launch", "tracking", reasons=["camera_failure"])),
        (4000, safety(reasons=[])),
    ]


def _simultaneous_triggers():
    # Emergency, abort and a docking request all in the same tick: emergency wins.
    return [
        (500, BIRD),
        (2000, merge(safety(*ACTIONS, emergency=True, abort=True,
                            reasons=["comm_loss", "operator_abort"]),
                     {"docking": {"should_dock": True, "reason": "comm_loss"}})),
    ]


def _grounded_ignores_everything():
    # On the dock with launch blocked: birds, docking requests and an emergency change nothing.
    return [
        (0, safety("launch", reasons=["low_battery"], battery_recovered=False)),
        (500, BIRD),
        (1000, {"docking": {"should_dock": True, "reason": "low_battery"}}),
        (1500, safety(*ACTIONS, emergency=True, reasons=["weather_hazard"])),
    ]


SCENARIOS: dict[str, Scenario] = {s.name: s for s in [
    Scenario("demo", "Original run_loop demo: track, low battery + cart, dock, recharge",
             DEMO_SCENARIO, 10_000),
    Scenario("bird_flyby_return", "Bird comes and goes, Docking returns the drone (no_bird)",
             _bird_flyby_return(), 7_000),
    Scenario("cart_moving_hold", "Docking requested while the cart moves; holds until it stops",
             _cart_moving_hold(), 9_000),
    Scenario("docking_failure_retry", "First approach fails, ABORT retries, second approach lands",
             _docking_failure_retry(), 6_000),
    Scenario("people_at_pad_abort", "People at the pad during approach; abort, wait, land",
             _people_at_pad_abort(), 7_500),
    Scenario("critical_battery_emergency", "Battery goes critical in flight; EMERGENCY_LAND",
             _critical_battery_emergency(), 5_000),
    Scenario("operator_abort_pulse", "One-tick operator abort while tracking, then dock",
             _operator_abort_pulse(), 5_000),
    Scenario("operator_abort_held", "Operator abort held for 2 s (shows ABORT/DOCKING_INIT flapping)",
             _operator_abort_held(), 5_000),
    Scenario("camera_failure_hold", "Camera drops out while tracking; hover, then resume",
             _camera_failure_hold(), 5_000),
    Scenario("simultaneous_triggers", "Emergency + abort + dock request in one tick",
             _simultaneous_triggers(), 3_000),
    Scenario("grounded_ignores_everything", "On the dock with launch blocked; nothing moves it",
             _grounded_ignores_everything(), 3_000),
]}


# --------------------------------------------------------------------------
# Generators for long / dense stress runs
# --------------------------------------------------------------------------

def patrol_cycles(cycles: int, cycle_ms: int = 60_000) -> list:
    """Repeat a full launch -> track -> dock -> recharge mission `cycles` times.

    Each cycle takes `cycle_ms` of simulated time (default 1 minute), so
    patrol_cycles(480) is an 8-hour day of back-to-back missions.
    """
    if cycle_ms < 10_000 or cycle_ms % TICK_MS:
        raise ValueError("cycle_ms must be a multiple of TICK_MS and at least 10 s")
    s = cycle_ms // 10  # one tenth of a cycle
    events = []
    for c in range(cycles):
        t0 = c * cycle_ms
        events += [
            (t0 + 1 * s, merge(BIRD, ALL_CLEAR, {"battery": {"percent": 100}},
                               {"docking": {"phase": "idle", "landed_on_pad": False,
                                            "should_dock": False, "reason": None}})),
            (t0 + 4 * s, NO_BIRD),
            (t0 + 5 * s, merge({"battery": {"percent": 18}},
                               {"docking": {"should_dock": True, "reason": "low_battery"}},
                               safety("launch", "tracking", battery_low=True,
                                      battery_recovered=False, reasons=["low_battery"]))),
            (t0 + 6 * s, {"docking": {"phase": "approach"}}),
            (t0 + 7 * s, {"docking": {"phase": "descend"}}),
            (t0 + 8 * s, {"docking": {"phase": "complete", "landed_on_pad": True,
                                      "should_dock": False, "reason": None}}),
            (t0 + 9 * s, merge({"battery": {"percent": 90}}, ALL_CLEAR)),
        ]
    return events


def sensor_flapping(duration_ms: int, period_ms: int = TICK_MS) -> list:
    """Bird detection toggles every `period_ms`: worst case for transition churn."""
    events = []
    for i, t in enumerate(range(0, duration_ms, period_ms)):
        events.append((t, BIRD if i % 2 == 0 else NO_BIRD))
    return events


def random_fuzz(duration_ms: int, seed: int = 0, events_per_s: float = 5.0,
                allow_emergency: bool = False) -> list:
    """Random but reproducible events drawn from realistic module outputs.

    Used to make sure step() never raises and memory stays flat whatever the
    inputs do. Same seed -> same scenario. EMERGENCY_LAND is terminal, so
    emergencies are left out by default to keep the whole run exercising the
    other states.
    """
    rng = random.Random(seed)
    choices = [
        lambda: BIRD, lambda: NO_BIRD,
        lambda: {"bird": {"detected": rng.random() < 0.5, "distance_m": rng.uniform(0, 80)}},
        lambda: {"battery": {"percent": rng.randint(0, 100)}},
        lambda: safety(*rng.sample(ACTIONS, rng.randint(0, len(ACTIONS)))),
        lambda: {"safety": {"emergency": allow_emergency and rng.random() < 0.05}},
        lambda: {"safety": {"abort": rng.random() < 0.1}},
        lambda: {"safety": {"battery_recovered": rng.random() < 0.5}},
        lambda: {"docking": {"should_dock": rng.random() < 0.5}},
        lambda: {"docking": {"phase": rng.choice(["idle", "approach", "search", "align",
                                                  "descend", "complete", "abort"])}},
        lambda: {"docking": {"landed_on_pad": rng.random() < 0.3}},
        lambda: {"bird": {}},          # empty update
    ]
    ticks = duration_ms // TICK_MS
    count = int(duration_ms / 1000 * events_per_s)
    events = []
    for _ in range(count):
        t = rng.randrange(ticks) * TICK_MS
        events.append((t, rng.choice(choices)()))
    events.sort(key=lambda e: e[0])
    return events


def padded_payload(kb_per_module: int, modules=("cv", "navigation", "telemetry", "comms")) -> list:
    """One event at t=0 that adds `kb_per_module` KB of extra fields per future module.

    Simulates the observation growing as modules add their own sections. The
    world is deep-copied every tick, so this is what bigger observations cost.
    """
    # 32 floats + 32 short strings is ~1 KB as JSON
    block = {f"f{i}": float(i) * 0.5 for i in range(32)}
    block.update({f"s{i}": f"value_{i:04d}" for i in range(32)})
    return [(0, {m: {f"blk{j}": dict(block) for j in range(kb_per_module)} for m in modules})]


def main(argv: list[str]) -> None:
    if not argv:
        for s in SCENARIOS.values():
            print(f"{s.name:28} {s.duration_ms/1000:5.1f}s  {s.description}")
        return
    for name in argv:
        s = SCENARIOS[name]
        print(f"== {s.name}: {s.description}")
        for t, old, new, reason in run_world(s.duration_ms, scenario=s.events):
            print(f"({t}ms) {old.value} -> {new.value}: Reason: {reason}")


if __name__ == "__main__":
    main(sys.argv[1:])
