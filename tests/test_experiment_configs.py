from pathlib import Path

import pytest

from uav_planning.config import load_config


ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize(
    "filename",
    [
        "e1_exact_benchmark.yaml",
        "e2_time_objective.yaml",
        "e2_v2_time_objective.yaml",
        "e3_travel_model.yaml",
        "e4_dynamic_strategy.yaml",
        "e5_scale_template.yaml",
    ],
)
def test_frozen_experiment_config_is_valid(filename: str) -> None:
    config = load_config(ROOT / "configs" / "experiments" / filename)
    assert config.execution_travel_model == "dubins"
    assert config.planner.commitment_horizon == 0


@pytest.mark.parametrize(
    "filename, expected_seed, expected_initial, expected_dynamic, expected_radius",
    [
        ("e2_time_objective.yaml", 1000, 10, 10, 10.0),
        ("e2_v2_time_objective.yaml", 1000, 10, 10, 10.0),
        ("e3_travel_model.yaml", 1000, 10, 10, 5.0),
        ("e4_dynamic_strategy.yaml", 1000, 10, 10, 10.0),
        ("e5_scale_template.yaml", 2000, 10, 10, 10.0),
    ],
)
def test_formal_dynamic_base_parameters_are_frozen(
    filename: str,
    expected_seed: int,
    expected_initial: int,
    expected_dynamic: int,
    expected_radius: float,
) -> None:
    config = load_config(ROOT / "configs" / "experiments" / filename)
    assert config.seed == expected_seed
    assert config.tasks.initial_count == expected_initial
    assert config.tasks.dynamic_count == expected_dynamic
    assert config.uav.min_turn_radius == expected_radius
    assert config.planner.affected_uav_count_h == 2
    assert config.planner.local_search_max_iterations == 100
    assert config.planner.local_search_time_limit_sec == 0.1
