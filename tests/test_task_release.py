from pathlib import Path

import pytest

from uav_planning.config import load_config
from uav_planning.planners.initial_planner import InitialPlanner
from uav_planning.routing.evaluator import EuclideanTravelTimeProvider, RouteEvaluator
from uav_planning.routing.insertion import RouteOptimizer
from uav_planning.scenario.generator import ScenarioGenerator


ROOT = Path(__file__).resolve().parents[1]


def test_initial_planner_cannot_see_future_tasks() -> None:
    config = load_config(ROOT / "configs" / "small_debug.yaml")
    scenario = ScenarioGenerator(config).generate()
    planner = InitialPlanner(RouteOptimizer(RouteEvaluator(EuclideanTravelTimeProvider())))
    with pytest.raises(ValueError, match="not yet released"):
        planner.plan(list(scenario.uavs), list(scenario.all_tasks), current_time=0.0)

