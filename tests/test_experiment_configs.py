from pathlib import Path

import pytest

from uav_planning.config import load_config


ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize(
    "filename",
    [
        "e1_exact_benchmark.yaml",
        "e2_time_objective.yaml",
        "e3_travel_model.yaml",
        "e4_dynamic_strategy.yaml",
        "e5_scale_template.yaml",
    ],
)
def test_frozen_experiment_config_is_valid(filename: str) -> None:
    config = load_config(ROOT / "configs" / "experiments" / filename)
    assert config.execution_travel_model == "dubins"
    assert config.planner.commitment_horizon == 0
