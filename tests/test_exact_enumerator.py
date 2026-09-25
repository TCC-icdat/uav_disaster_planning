from pathlib import Path

import pytest

from uav_planning.config import load_config
from uav_planning.exact import ExactEnumerator
from uav_planning.models import Pose2D, Task, UAV
from uav_planning.planners import InitialPlanner
from uav_planning.routing.evaluator import EuclideanTravelTimeProvider, RouteEvaluator
from uav_planning.routing.insertion import RouteOptimizer
from uav_planning.scenario.generator import ScenarioGenerator


ROOT = Path(__file__).resolve().parents[1]


def test_exact_enumerator_matches_handcrafted_instance() -> None:
    uav = UAV(1, 1.0, 1.0, 100.0, Pose2D(0.0, 0.0, 0.0))
    tasks = {
        1: Task(1, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0),
        2: Task(2, 2.0, 0.0, 0.0, 1.0, 0.0, 0.0),
    }
    optimizer = RouteOptimizer(RouteEvaluator(EuclideanTravelTimeProvider()))
    solution = ExactEnumerator(optimizer).solve({1: uav}, tasks)
    assert solution.plan.routes[1].task_ids == [1, 2]
    assert solution.objective == pytest.approx(3.0)


def test_exact_objective_not_worse_than_heuristic() -> None:
    config = load_config(ROOT / "configs" / "small_debug.yaml")
    scenario = ScenarioGenerator(config).generate()
    tasks = {task.task_id: task for task in scenario.initial_tasks}
    uavs = {uav.uav_id: uav for uav in scenario.uavs}
    optimizer = RouteOptimizer(RouteEvaluator(EuclideanTravelTimeProvider()), 30, 0.1)
    heuristic = InitialPlanner(optimizer).plan(
        list(scenario.uavs), list(scenario.initial_tasks)
    )
    heuristic_objective = optimizer.evaluate_plan(heuristic, uavs, tasks).objective
    exact = ExactEnumerator(optimizer).solve(uavs, tasks)
    assert exact.objective <= heuristic_objective + 1e-9


def test_exact_guard_rejects_large_instance() -> None:
    uav = UAV(1, 1.0, 1.0, 100.0, Pose2D(0.0, 0.0, 0.0))
    tasks = {
        index: Task(index, float(index), 0.0, 0.0, 1.0, 0.0, 0.0)
        for index in range(1, 10)
    }
    optimizer = RouteOptimizer(RouteEvaluator(EuclideanTravelTimeProvider()))
    with pytest.raises(ValueError, match="at most 8 tasks"):
        ExactEnumerator(optimizer).solve({1: uav}, tasks)
