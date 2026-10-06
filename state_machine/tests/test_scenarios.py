"""Each demo scenario in state_machine/scenarios.py produces the transitions it is meant to."""

import pytest

from state_machine.run_loop import DEMO_SCENARIO, TICK_MS, index_scenario, init_world, run_world
from state_machine.scenarios import (SCENARIOS, padded_payload, patrol_cycles, random_fuzz,
                                     sensor_flapping)
from state_machine.state_machine import State

S = State

EXPECTED = {
    "demo": [S.HOVERING, S.TRACKING, S.HOVERING, S.DOCKING_INIT, S.DOCKING_APPROACH, S.DOCKED, S.IDLE],
    "bird_flyby_return": [S.HOVERING, S.TRACKING, S.HOVERING, S.DOCKING_INIT, S.DOCKING_APPROACH,
                          S.DOCKED, S.IDLE],
    "cart_moving_hold": [S.HOVERING, S.TRACKING, S.DOCKING_INIT, S.DOCKING_APPROACH, S.DOCKED, S.IDLE],
    "docking_failure_retry": [S.HOVERING, S.TRACKING, S.DOCKING_INIT, S.DOCKING_APPROACH, S.ABORT,
                              S.DOCKING_INIT, S.DOCKING_APPROACH, S.DOCKED, S.IDLE],
    "people_at_pad_abort": [S.HOVERING, S.TRACKING, S.DOCKING_INIT, S.DOCKING_APPROACH, S.ABORT,
                            S.DOCKING_INIT, S.DOCKING_APPROACH, S.DOCKED, S.IDLE],
    "critical_battery_emergency": [S.HOVERING, S.TRACKING, S.HOVERING, S.EMERGENCY_LAND],
    "operator_abort_pulse": [S.HOVERING, S.TRACKING, S.ABORT, S.DOCKING_INIT, S.DOCKING_APPROACH,
                             S.DOCKED, S.IDLE],
    "camera_failure_hold": [S.HOVERING, S.TRACKING, S.HOVERING, S.TRACKING],
    "simultaneous_triggers": [S.HOVERING, S.TRACKING, S.EMERGENCY_LAND],
    "grounded_ignores_everything": [],
}


def states(name):
    s = SCENARIOS[name]
    return [new for _, _, new, _ in run_world(s.duration_ms, scenario=s.events)]


@pytest.mark.parametrize("name", sorted(EXPECTED))
def test_scenario_transitions(name):
    assert states(name) == EXPECTED[name]


def test_every_scenario_has_an_expectation():
    # A new scenario should come with the transitions it is meant to produce
    assert set(SCENARIOS) - set(EXPECTED) == {"operator_abort_held"}


@pytest.mark.parametrize("name", sorted(SCENARIOS))
def test_scenario_events_are_well_formed(name):
    # Event times land on ticks and inside the run; sections exist in the world (catches typos)
    s = SCENARIOS[name]
    known = set(init_world())
    for t, changes in s.events:
        assert t % TICK_MS == 0 and 0 <= t < s.duration_ms, (name, t)
        assert set(changes) <= known, (name, set(changes) - known)


def test_cart_moving_holds_in_docking_init_until_cart_stops():
    s = SCENARIOS["cart_moving_hold"]
    times = {new: t for t, _, new, _ in run_world(s.duration_ms, scenario=s.events)}
    assert times[S.DOCKING_INIT] == 2000
    assert times[S.DOCKING_APPROACH] == 6000  # first tick the cart is stopped


def test_emergency_wins_over_abort_and_docking_in_same_tick():
    s = SCENARIOS["simultaneous_triggers"]
    last = run_world(s.duration_ms, scenario=s.events)[-1]
    assert last[2] == S.EMERGENCY_LAND and last[3] == "Safety emergency"


@pytest.mark.xfail(strict=True, reason=(
    "Known issue: while Safety keeps abort=True, ABORT -> DOCKING_INIT (hold permitted) -> ABORT "
    "(global abort rule) flips every tick. Needs a rule such as 'stay in ABORT while abort is set'."))
def test_held_operator_abort_does_not_flap():
    s = SCENARIOS["operator_abort_held"]
    transitions = run_world(s.duration_ms, scenario=s.events)
    aborts = [t for t, _, new, _ in transitions if new == S.ABORT]
    assert len(aborts) == 1, f"{len(aborts)} entries into ABORT in 2 s"


# --- run_loop changes -----------------------------------------------------

def test_run_world_default_is_still_the_demo():
    assert run_world(10_000) == run_world(10_000, scenario=DEMO_SCENARIO)


def test_index_scenario_keeps_every_event_at_the_same_time():
    events = [(100, {"bird": {"detected": True}}), (100, {"battery": {"percent": 50}}),
              (200, {"bird": {"detected": False}})]
    assert index_scenario(events) == {100: [events[0][1], events[1][1]], 200: [events[2][1]]}


def test_on_tick_sees_every_tick_and_the_current_state():
    seen = []
    run_world(1_000, scenario=[(500, {"bird": {"detected": True}})],
              on_tick=lambda t, snap, state: seen.append((t, state, snap["timestamp_ns"])))
    assert [t for t, _, _ in seen] == list(range(0, 1_000, TICK_MS))
    assert seen[5][1] == S.HOVERING and seen[6][1] == S.TRACKING
    assert seen[3][2] == 300 * 1_000_000


def test_unknown_section_in_scenario_is_added_not_a_crash():
    # Future modules can add their own section without touching run_loop
    run_world(500, scenario=[(0, {"navigation": {"near_pad": False}})])


# --- generators -------------------------------------------------------------

def test_patrol_cycles_repeat_the_full_mission():
    cycles = 5
    transitions = run_world(cycles * 60_000, scenario=patrol_cycles(cycles))
    one = [S.HOVERING, S.TRACKING, S.HOVERING, S.DOCKING_INIT, S.DOCKING_APPROACH, S.DOCKED, S.IDLE]
    assert [new for _, _, new, _ in transitions] == one * cycles


def test_sensor_flapping_changes_state_every_tick_once_airborne():
    # Every tick except the second: losing the bird while still launching (HOVERING) is no change
    transitions = run_world(10_000, scenario=sensor_flapping(10_000))
    assert len(transitions) == 10_000 // TICK_MS - 1


@pytest.mark.parametrize("seed", range(10))
def test_random_fuzz_never_raises_and_stays_in_known_states(seed):
    duration = 10 * 60_000
    transitions = run_world(duration, scenario=random_fuzz(duration, seed=seed, events_per_s=20,
                                                           allow_emergency=True))
    assert all(isinstance(new, State) for _, _, new, _ in transitions)


def test_random_fuzz_is_reproducible():
    assert random_fuzz(60_000, seed=3) == random_fuzz(60_000, seed=3)
    assert random_fuzz(60_000, seed=3) != random_fuzz(60_000, seed=4)


def test_padded_payload_is_about_the_requested_size():
    import json
    (_, change), = padded_payload(10, modules=("cv",))
    assert 9_000 < len(json.dumps(change)) < 12_000
