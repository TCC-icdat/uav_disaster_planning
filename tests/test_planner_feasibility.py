from pathlib import Path

import pytest

from uav_planning.config import load_config
from uav_planning.planners import InitialPlanner, LocalReplanner
from uav_planning.routing.evaluator import DubinsTravelTimeProvider, RouteEvaluator
from uav_planning.routing.insertion import RouteOptimizer
from uav_planning.scenario.generator import ScenarioGenerator
from uav_planning.simulation.simulator import Simulator


ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def completed_run():
    config = load_config(ROOT / "configs" / "small_debug.yaml")
    scenario = ScenarioGenerator(config).generate()
    optimizer = RouteOptimizer(
        RouteEvaluator(DubinsTravelTimeProvider()),
        config.planner.vns_max_iterations,
        config.planner.vns_time_limit_sec,
    )
    result = Simulator(
        scenario, InitialPlanner(optimizer), optimizer, config.high_priority_threshold
    ).run(LocalReplanner(optimizer, config.planner.affected_uav_count_h))
    return scenario, optimizer, result


def test_every_task_is_assigned_exactly_once(completed_run) -> None:
    scenario, _, result = completed_run
    assigned = [
        task_id for route in result.final_plan.routes.values() for task_id in route.task_ids
    ]
    assert sorted(assigned) == sorted(task.task_id for task in scenario.all_tasks)
    assert len(assigned) == len(set(assigned))


def test_all_routes_return_within_endurance(completed_run) -> None:
    scenario, optimizer, result = completed_run
    tasks = {task.task_id: task for task in scenario.all_tasks}
    evaluation = optimizer.evaluate_plan(
        result.final_plan,
        {uav.uav_id: uav for uav in scenario.uavs},
        tasks,
        set(tasks),
    )
    assert evaluation.feasible
    for uav in scenario.uavs:
        assert evaluation.route_evaluations[uav.uav_id].return_time <= (
            uav.max_mission_time + 1e-9
        )


def test_no_task_starts_before_release(completed_run) -> None:
    scenario, optimizer, result = completed_run
    tasks = {task.task_id: task for task in scenario.all_tasks}
    evaluation = optimizer.evaluate_plan(
        result.final_plan,
        {uav.uav_id: uav for uav in scenario.uavs},
        tasks,
        set(tasks),
    )
    for route in evaluation.route_evaluations.values():
        for task_id, start in route.start_times.items():
            assert start >= tasks[task_id].release_time

