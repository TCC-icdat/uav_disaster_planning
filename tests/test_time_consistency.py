from uav_planning.models import (
    Plan,
    PlanningAnchor,
    Pose2D,
    Route,
    SimulationState,
    Task,
    TaskExecutionRecord,
    UAV,
    UAVExecutionState,
)
from uav_planning.planners import FullReplanner, InitialPlanner
from uav_planning.routing.evaluator import EuclideanTravelTimeProvider, RouteEvaluator
from uav_planning.routing.insertion import RouteOptimizer
from uav_planning.scenario.generator import Scenario
from uav_planning.simulation.simulator import Simulator


def test_replanning_cannot_schedule_pending_task_in_the_past() -> None:
    uavs = {
        1: UAV(1, 10.0, 2.0, 100.0, Pose2D(0.0, 0.0, 0.0)),
        2: UAV(2, 10.0, 2.0, 100.0, Pose2D(0.0, 0.0, 0.0)),
    }
    task1 = Task(1, 100.0, 0.0, 0.0, 1.0, 2.0, 0.0)
    task2 = Task(2, 1.0, 0.0, 0.0, 5.0, 1.0, 0.0)
    task3 = Task(3, 2.0, 0.0, 10.0, 10.0, 1.0, 0.0)
    optimizer = RouteOptimizer(RouteEvaluator(EuclideanTravelTimeProvider()), 20, 0.1)
    state = SimulationState(
        current_time=10.0,
        uavs=uavs,
        tasks={1: task1, 2: task2, 3: task3},
        plan=Plan({1: Route(1, [2]), 2: Route(2, [])}),
        execution_states={
            1: UAVExecutionState(1, 10.0, task1.pose, committed_task_ids=[1], remaining_task_ids=[2]),
            2: UAVExecutionState(2, 10.0, uavs[2].start_pose),
        },
        completed_task_ids=set(),
        executing_task_ids={1: 1},
        anchors={
            1: PlanningAnchor(1, 12.0, task1.pose),
            2: PlanningAnchor(2, 10.0, uavs[2].start_pose),
        },
        locked_task_ids={1: 1},
        history={},
    )
    plan = FullReplanner(optimizer).replan(state, task3)
    assert 2 in plan.routes[2].task_ids
    evaluation = optimizer.evaluate_plan(plan, uavs, state.tasks, anchors=state.anchors)
    assert evaluation.route_evaluations[2].start_times[2] >= 10.0


def test_completed_history_is_unchanged_after_later_replanning() -> None:
    uavs = (
        UAV(1, 10.0, 1.0, 100.0, Pose2D(0.0, 0.0, 0.0)),
        UAV(2, 10.0, 1.0, 100.0, Pose2D(0.0, 0.0, 0.0)),
    )
    scenario = Scenario(
        seed=7,
        width=20.0,
        height=20.0,
        uavs=uavs,
        initial_tasks=(Task(1, 1.0, 0.0, 0.0, 3.0, 1.0, 0.0),),
        dynamic_tasks=(
            Task(2, 2.0, 0.0, 5.0, 4.0, 1.0, 0.0),
            Task(3, 3.0, 0.0, 10.0, 5.0, 1.0, 0.0),
        ),
    )
    optimizer = RouteOptimizer(RouteEvaluator(EuclideanTravelTimeProvider()), 20, 0.1)
    result = Simulator(scenario, InitialPlanner(optimizer), optimizer).run(FullReplanner(optimizer))
    first_snapshot = result.history_snapshots[0][1][1]
    second_snapshot = result.history_snapshots[1][1][1]
    assert first_snapshot == second_snapshot == result.history[1]
    assert isinstance(result.history[1], TaskExecutionRecord)
