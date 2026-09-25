"""Initial Regret-2 plus multi-neighborhood local-search routing."""

from __future__ import annotations

from uav_planning.models import Plan, Task, UAV
from uav_planning.routing.insertion import RouteOptimizer, empty_plan


class InitialPlanner:
    """Build a plan for tasks known at the current decision time."""

    def __init__(self, optimizer: RouteOptimizer) -> None:
        self.optimizer = optimizer

    def plan(
        self,
        uavs: list[UAV],
        tasks: list[Task],
        current_time: float = 0.0,
    ) -> Plan:
        """Plan released tasks; reject accidental access to future events."""

        visible = [task for task in tasks if task.release_time <= current_time + 1e-9]
        if len(visible) != len(tasks):
            raise ValueError("InitialPlanner received tasks that are not yet released")
        uav_map = {uav.uav_id: uav for uav in uavs}
        task_map = {task.task_id: task for task in visible}
        return self.optimizer.optimize(
            uav_map,
            task_map,
            empty_plan(uav_map),
            sorted(task_map),
            objective_task_ids=set(task_map),
        )

