"""Best insertion without changing any existing free-task order."""

from __future__ import annotations

from dataclasses import dataclass
from math import inf

from uav_planning.models import Plan, SimulationState, Task
from uav_planning.routing.insertion import PlanningError, RouteOptimizer


@dataclass(frozen=True)
class InsertionResult:
    """Best global insertion and the best cost available on each UAV."""

    plan: Plan
    best_uav_id: int
    objective: float
    per_uav_objectives: dict[int, float]


def best_no_reorder_insertion(
    optimizer: RouteOptimizer,
    state: SimulationState,
    new_task: Task,
) -> InsertionResult:
    """Insert one new task while preserving every old free-task sequence."""

    objective_ids = {
        task_id for route in state.plan.routes.values() for task_id in route.task_ids
    } | {new_task.task_id}
    best_plan: Plan | None = None
    best_uav_id = -1
    best_score = inf
    per_uav: dict[int, float] = {}
    for uav_id in sorted(state.uavs):
        route = state.plan.routes[uav_id].task_ids
        uav_best = inf
        for position in range(len(route) + 1):
            candidate = state.plan.copy()
            candidate.routes[uav_id].task_ids.insert(position, new_task.task_id)
            evaluation = optimizer.evaluate_plan(
                candidate,
                state.uavs,
                state.tasks,
                objective_task_ids=objective_ids,
                anchors=state.anchors,
            )
            if evaluation.feasible and evaluation.objective < uav_best:
                uav_best = evaluation.objective
            if evaluation.feasible and evaluation.objective < best_score:
                best_score = evaluation.objective
                best_uav_id = uav_id
                best_plan = candidate
        per_uav[uav_id] = uav_best
    if best_plan is None:
        raise PlanningError(f"no feasible insertion for task {new_task.task_id}")
    return InsertionResult(best_plan, best_uav_id, best_score, per_uav)


class NoReorderInsertionPlanner:
    """Preserve every old free-task assignment and relative order."""

    name = "no_reorder"

    def __init__(self, optimizer: RouteOptimizer) -> None:
        self.optimizer = optimizer

    def replan(self, state: SimulationState, new_task: Task) -> Plan:
        return best_no_reorder_insertion(self.optimizer, state, new_task).plan

