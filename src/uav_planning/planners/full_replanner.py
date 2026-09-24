"""Full replanning baseline with only executed/in-progress prefixes locked."""

from __future__ import annotations

from uav_planning.models import Plan, Route, SimulationState, Task
from uav_planning.routing.insertion import RouteOptimizer


class FullReplanner:
    """Release every not-yet-locked task across all UAVs."""

    name = "full"

    def __init__(self, optimizer: RouteOptimizer) -> None:
        self.optimizer = optimizer

    def replan(self, state: SimulationState, new_task: Task) -> Plan:
        """Apply the shared Regret-2 and VNS engine to the entire free suffix."""

        base = Plan(
            {
                uav_id: Route(uav_id, list(state.locked_prefixes[uav_id]))
                for uav_id in sorted(state.uavs)
            }
        )
        locked = {
            task_id for route in base.routes.values() for task_id in route.task_ids
        }
        free = sorted(set(state.tasks) - locked)
        return self.optimizer.optimize(
            state.uavs,
            state.tasks,
            base,
            free,
            fixed_prefix_lengths={
                uav_id: len(route.task_ids) for uav_id, route in base.routes.items()
            },
            objective_task_ids=set(state.tasks),
        )

