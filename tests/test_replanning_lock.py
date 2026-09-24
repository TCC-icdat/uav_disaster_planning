from uav_planning.models import Plan, Pose2D, Route, SimulationState, Task, UAV, UAVExecutionState
from uav_planning.planners import FullReplanner, LocalReplanner
from uav_planning.routing.evaluator import EuclideanTravelTimeProvider, RouteEvaluator
from uav_planning.routing.insertion import RouteOptimizer


def _state() -> tuple[SimulationState, Task, RouteOptimizer]:
    uavs = {
        index: UAV(index, 10.0, 2.0, 500.0, Pose2D(0.0, 0.0, 0.0))
        for index in (1, 2, 3)
    }
    tasks = {
        index: Task(index, index * 5.0, index * 2.0, 0.0, float(index), 1.0, 0.0)
        for index in range(1, 7)
    }
    new_task = Task(7, 9.0, 9.0, 2.0, 10.0, 1.0, 0.0)
    tasks[7] = new_task
    plan = Plan({1: Route(1, [1, 2]), 2: Route(2, [3, 4]), 3: Route(3, [5, 6])})
    execution = {
        uid: UAVExecutionState(uid, 2.0, uav.start_pose)
        for uid, uav in uavs.items()
    }
    state = SimulationState(
        current_time=2.0,
        uavs=uavs,
        tasks=tasks,
        plan=plan,
        execution_states=execution,
        completed_task_ids={1},
        executing_task_ids={1: 1},
        locked_prefixes={1: [1], 2: [], 3: []},
        commitment_prefixes={1: [1, 2], 2: [3], 3: [5]},
    )
    optimizer = RouteOptimizer(RouteEvaluator(EuclideanTravelTimeProvider()), 20, 0.1)
    return state, new_task, optimizer


def test_local_preserves_commitment_and_nonaffected_routes() -> None:
    state, new_task, optimizer = _state()
    planner = LocalReplanner(optimizer, h=1)
    result = planner.replan(state, new_task)
    affected = planner.last_affected_uav_ids
    assert len(affected) == 1
    for uav_id in state.uavs:
        prefix = state.commitment_prefixes[uav_id]
        assert result.routes[uav_id].task_ids[: len(prefix)] == prefix
        if uav_id not in affected:
            assert result.routes[uav_id].task_ids == state.plan.routes[uav_id].task_ids


def test_full_and_local_share_optimizer_components() -> None:
    _, _, optimizer = _state()
    full = FullReplanner(optimizer)
    local = LocalReplanner(optimizer)
    assert full.optimizer is local.optimizer
    assert full.optimizer.evaluator is local.optimizer.evaluator
