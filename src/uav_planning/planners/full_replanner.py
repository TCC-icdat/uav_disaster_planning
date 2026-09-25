"""Full replanning baseline with only executed/in-progress prefixes locked."""

from __future__ import annotations

from uav_planning.models import SimulationState, Task
from uav_planning.routing.insertion import RouteOptimizer, empty_plan


class FullReplanner:
    """Reassign every free task from the forward planning anchors."""

    name = "full"

    def __init__(self, optimizer: RouteOptimizer) -> None:
        self.optimizer = optimizer

    def replan(self, state: SimulationState, new_task: Task) -> Plan:
        """Apply shared Regret-2 and local search to all unstarted tasks."""

        base = empty_plan(state.uavs)
        free = sorted(
            task_id
            for route in state.plan.routes.values()
            for task_id in route.task_ids
        )
        free.append(new_task.task_id)
        return self.optimizer.optimize(
            state.uavs,
            state.tasks,
            base,
            free,
            objective_task_ids=set(free),
            anchors=state.anchors,
        )

