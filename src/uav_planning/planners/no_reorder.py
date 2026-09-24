"""Dynamic baseline that only inserts the newly released task."""

from __future__ import annotations

from math import inf

from uav_planning.models import Plan, SimulationState, Task
from uav_planning.routing.insertion import PlanningError, RouteOptimizer


class NoReorderInsertionPlanner:
    """Preserve every old assignment and relative order."""

    name = "no_reorder"

    def __init__(self, optimizer: RouteOptimizer) -> None:
        self.optimizer = optimizer

    def replan(self, state: SimulationState, new_task: Task) -> Plan:
        """Insert only the new task at the least-cost feasible position."""

        best_plan: Plan | None = None
        best_score = inf
        for uav_id in sorted(state.uavs):
            sequence = state.plan.routes[uav_id].task_ids
            first_position = len(state.locked_prefixes[uav_id])
            for position in range(first_position, len(sequence) + 1):
                candidate = state.plan.copy()
                candidate.routes[uav_id].task_ids.insert(position, new_task.task_id)
                evaluation = self.optimizer.evaluate_plan(
                    candidate,
                    state.uavs,
                    state.tasks,
                    objective_task_ids=set(state.tasks),
                )
                if evaluation.feasible and evaluation.objective < best_score:
                    best_score = evaluation.objective
                    best_plan = candidate
        if best_plan is None:
            raise PlanningError(f"no feasible insertion for task {new_task.task_id}")
        return best_plan

