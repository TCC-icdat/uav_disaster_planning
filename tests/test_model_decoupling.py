from math import pi

import pytest

from uav_planning.models import Plan, Pose2D, Route, Task, UAV
from uav_planning.planners import InitialPlanner, NoReorderInsertionPlanner
from uav_planning.routing.evaluator import (
    DubinsTravelTimeProvider,
    EuclideanTravelTimeProvider,
    RouteEvaluator,
)
from uav_planning.routing.insertion import RouteOptimizer
from uav_planning.scenario.generator import Scenario
from uav_planning.simulation.simulator import Simulator


def _single_task_scenario() -> Scenario:
    uav = UAV(1, 10.0, 5.0, 100.0, Pose2D(0.0, 0.0, 0.0))
    task = Task(1, 10.0, 0.0, 0.0, 5.0, 1.0, pi)
    return Scenario(1, 20.0, 20.0, (uav,), (task,), ())


def test_euclidean_planning_with_dubins_execution() -> None:
    scenario = _single_task_scenario()
    planning = RouteOptimizer(RouteEvaluator(EuclideanTravelTimeProvider()))
    execution = RouteEvaluator(DubinsTravelTimeProvider())
    result = Simulator(
        scenario,
        InitialPlanner(planning),
        planning,
        execution_evaluator=execution,
    ).run(NoReorderInsertionPlanner(planning))
    record = result.history[1]
    expected = execution.travel_time_provider.travel_time(
        scenario.uavs[0], scenario.uavs[0].start_pose, scenario.initial_tasks[0].pose
    )
    euclidean = EuclideanTravelTimeProvider().travel_time(
        scenario.uavs[0], scenario.uavs[0].start_pose, scenario.initial_tasks[0].pose
    )
    assert record.travel_time == pytest.approx(expected)
    assert record.travel_time > euclidean
    assert result.metrics["planning_travel_model"] == "euclidean"
    assert result.metrics["execution_travel_model"] == "dubins"
    assert result.metrics["time_consistency_check"]


def test_execution_history_uses_physical_model() -> None:
    scenario = _single_task_scenario()
    planning = RouteOptimizer(RouteEvaluator(EuclideanTravelTimeProvider()))
    execution = RouteEvaluator(DubinsTravelTimeProvider())
    result = Simulator(
        scenario,
        InitialPlanner(planning),
        planning,
        execution_evaluator=execution,
    ).run(NoReorderInsertionPlanner(planning))
    physical_travel = execution.travel_time_provider.travel_time(
        scenario.uavs[0], scenario.uavs[0].start_pose, scenario.initial_tasks[0].pose
    )
    physical_return = execution.travel_time_provider.travel_time(
        scenario.uavs[0], scenario.initial_tasks[0].pose, scenario.uavs[0].start_pose
    )
    assert result.history[1].travel_time == pytest.approx(physical_travel)
    assert result.metrics["total_travel_time"] == pytest.approx(
        physical_travel + physical_return
    )


def test_default_planning_and_execution_models_match_previous_behavior() -> None:
    scenario = _single_task_scenario()
    optimizer = RouteOptimizer(RouteEvaluator(DubinsTravelTimeProvider()))
    implicit = Simulator(scenario, InitialPlanner(optimizer), optimizer).run(
        NoReorderInsertionPlanner(optimizer)
    )
    explicit = Simulator(
        scenario,
        InitialPlanner(optimizer),
        optimizer,
        execution_evaluator=optimizer.evaluator,
    ).run(NoReorderInsertionPlanner(optimizer))
    assert implicit.history == explicit.history
    assert implicit.metrics["weighted_delay"] == pytest.approx(
        explicit.metrics["weighted_delay"]
    )
    assert implicit.metrics["total_travel_time"] == pytest.approx(
        explicit.metrics["total_travel_time"]
    )


def test_total_travel_objective_ignores_priority_in_planning_score() -> None:
    uav = UAV(1, 10.0, 2.0, 100.0, Pose2D(0.0, 0.0, 0.0))
    low = {
        1: Task(1, 5.0, 0.0, 0.0, 1.0, 1.0, 0.0),
        2: Task(2, 10.0, 0.0, 0.0, 2.0, 1.0, 0.0),
    }
    high = {
        1: Task(1, 5.0, 0.0, 0.0, 100.0, 1.0, 0.0),
        2: Task(2, 10.0, 0.0, 0.0, 200.0, 1.0, 0.0),
    }
    plan = Plan({1: Route(1, [1, 2])})
    optimizer = RouteOptimizer(
        RouteEvaluator(EuclideanTravelTimeProvider()),
        objective_mode="total_travel_time",
    )
    first = optimizer.evaluate_plan(plan, {1: uav}, low)
    second = optimizer.evaluate_plan(plan, {1: uav}, high)
    assert first.objective == pytest.approx(second.objective)
    assert first.weighted_delay != second.weighted_delay


def test_weighted_delay_metric_is_still_reported_for_distance_baseline() -> None:
    scenario = _single_task_scenario()
    optimizer = RouteOptimizer(
        RouteEvaluator(DubinsTravelTimeProvider()),
        objective_mode="total_travel_time",
    )
    result = Simulator(scenario, InitialPlanner(optimizer), optimizer).run(
        NoReorderInsertionPlanner(optimizer)
    )
    assert result.metrics["objective_mode"] == "total_travel_time"
    assert result.metrics["weighted_delay"] > 0.0
