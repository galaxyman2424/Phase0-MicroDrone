"""The synthetic data generator flies MotionStub through the shared contract.

It used to have its own motion model and motor mixer (with its own speeds);
these tests pin the replacement: every row comes from `MotionStub` driven by
`contracts.Command`, so the CSVs obey the same limits as everything else.
"""

import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

import pytest

from motion_engine.contracts import CONTRACT_KINDS
from motion_engine.motion_stubs import MotionLimits
from motion_engine.synthetic import generate_motion_data as gen
from motion_engine.trace_checker import check_trace, load_synthetic_csv


@pytest.mark.parametrize("scenario", gen.ALL_SCENARIOS)
def test_every_scenario_flies_cleanly(tmp_path, scenario):
    moves = gen.scenario_moves(scenario, seed=3)
    drift = gen.DRIFT_MPS.get(scenario, (0.0, 0.0))
    rows = gen.generate(moves, drift_mps=drift)
    path = tmp_path / f"{scenario}.csv"
    gen.write_csv(rows, path, gen.metadata(scenario, moves, 10.0, 0.0, 3, drift, rows))
    report = check_trace(load_synthetic_csv(path), name=scenario)
    assert report.ok, report.format_text()

    first, last = rows[0], rows[-1]
    assert (first["direction"], first["armed"], first["z_m"]) == ("pad", False, 0.0)
    assert (last["direction"], last["armed"], last["z_m"]) == ("disarm", False, 0.0)
    assert {r["command_kind"] for r in rows[1:]} <= {k.value for k in CONTRACT_KINDS}


def test_hand_written_scenarios_end_back_on_the_pad():
    for name in gen.SCENARIOS:
        if name in gen.DRIFT_MPS:
            continue
        last = gen.generate(gen.scenario_moves(name))[-1]
        assert (last["x_m"], last["y_m"], last["z_m"]) == (0.0, 0.0, 0.0), name


def test_moves_are_sent_as_set_velocity_and_hover():
    rows = gen.generate([(gen.Move.HOVER, 1), (gen.Move.FORWARD, 1)])
    kinds = [r["command_kind"] for r in rows if r["step"] in (0, 1)]
    assert kinds[:10] == ["HOVER"] * 10 and kinds[10:] == ["SET_VELOCITY"] * 10


def test_forward_follows_the_heading():
    rows = gen.generate([(gen.Move.YAW_LEFT, 2), (gen.Move.FORWARD, 2)])
    end_of_forward = [r for r in rows if r["step"] == 1][-1]
    assert end_of_forward["x_m"] == pytest.approx(0.0, abs=1e-9)
    assert end_of_forward["y_m"] == pytest.approx(2.0)


def test_drift_moves_the_reported_position_but_not_the_command():
    rows = gen.generate(gen.scenario_moves("drift"), drift_mps=gen.DRIFT_MPS["drift"])
    hover = [r for r in rows if r["direction"] == "hover"]
    assert {r["command_kind"] for r in hover} == {"HOVER"}
    assert hover[-1]["x_m"] > hover[0]["x_m"]


def test_random_walk_is_reproducible():
    a = gen.generate(gen.scenario_moves("random_walk", seed=7), seed=7)
    b = gen.generate(gen.scenario_moves("random_walk", seed=7), seed=7)
    assert a == b


def test_speeds_above_the_shared_limits_are_refused(monkeypatch):
    """The generator cannot quietly fly faster than MotionLimits allows."""
    monkeypatch.setattr(gen, "HORIZONTAL_SPEED_MPS", MotionLimits().max_horizontal_speed_mps + 1)
    with pytest.raises(gen.GeneratorError, match="velocity_outside_limits"):
        gen.generate([(gen.Move.FORWARD, 1)])


def test_down_moves_stop_at_the_velocity_floor():
    rows = gen.generate([(gen.Move.DOWN, 5)])  # would reach -0.5 m without the floor
    down = [r for r in rows if r["direction"] == "down"]
    assert min(r["z_m"] for r in down) == pytest.approx(MotionLimits().velocity_floor_m)
