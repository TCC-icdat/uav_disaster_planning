"""Best insertion followed by affected-UAV suffix local improvement."""

from __future__ import annotations

from math import inf

from uav_planning.models import Plan, SimulationState, Task
from uav_planning.planners.no_reorder import best_no_reorder_insertion
from uav_planning.routing.insertion import RouteOptimizer


class LocalReplanner:
    """Improve a no-reorder incumbent within at most ``h`` UAV routes."""

    name = "local"

    def __init__(self, optimizer: RouteOptimizer, h: int = 2) -> None:
        if h <= 0:
            raise ValueError("h must be positive")
        self.optimizer = optimizer
        self.h = h
        self.last_affected_uav_ids: set[int] = set()
        self.last_incumbent_objective = inf
        self.last_final_objective = inf

    def replan(
        self,
        state: SimulationState,
        new_task: Task,
        h: int | None = None,
    ) -> Plan:
        """Insert first, then strictly improve only affected free suffixes."""

        insertion = best_no_reorder_insertion(self.optimizer, state, new_task)
        affected_count = min(h or self.h, len(state.uavs))
        ranked = sorted(
            insertion.per_uav_objectives.items(), key=lambda item: (item[1], item[0])
        )
        affected = {uav_id for uav_id, _ in ranked[:affected_count]}
        affected.add(insertion.best_uav_id)
        if len(affected) > affected_count:
            removable = sorted(
                affected - {insertion.best_uav_id},
                key=lambda uav_id: insertion.per_uav_objectives[uav_id],
                reverse=True,
            )
            while len(affected) > affected_count:
                affected.remove(removable.pop(0))
        self.last_affected_uav_ids = affected

        objective_ids = {
            task_id
            for route in insertion.plan.routes.values()
            for task_id in route.task_ids
        }
        self.last_incumbent_objective = insertion.objective
        result = self.optimizer.local_search(
            insertion.plan,
            state.uavs,
            state.tasks,
            fixed_prefix_lengths={uav_id: 0 for uav_id in state.uavs},
            mutable_uav_ids=affected,
            objective_task_ids=objective_ids,
            anchors=state.anchors,
        )
        self.last_final_objective = self.optimizer.evaluate_plan(
            result,
            state.uavs,
            state.tasks,
            objective_task_ids=objective_ids,
            anchors=state.anchors,
        ).objective
        return result
