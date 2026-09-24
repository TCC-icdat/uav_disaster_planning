"""Execution-prefix-preserving local suffix replanning."""

from __future__ import annotations

from math import inf

from uav_planning.models import Plan, Route, SimulationState, Task
from uav_planning.routing.insertion import PlanningError, RouteOptimizer


class LocalReplanner:
    """Optimize only the free suffixes of the most affected UAVs."""

    name = "local"

    def __init__(self, optimizer: RouteOptimizer, h: int = 2) -> None:
        if h <= 0:
            raise ValueError("h must be positive")
        self.optimizer = optimizer
        self.h = h
        self.last_affected_uav_ids: set[int] = set()

    def replan(
        self,
        state: SimulationState,
        new_task: Task,
        h: int | None = None,
    ) -> Plan:
        """Select affected UAVs by insertion cost and reoptimize locally."""

        affected_count = min(h or self.h, len(state.uavs))
        scores: list[tuple[float, int]] = []
        old_eval = self.optimizer.evaluate_plan(
            state.plan,
            state.uavs,
            state.tasks,
            objective_task_ids=set(state.tasks) - {new_task.task_id},
        )
        for uav_id in sorted(state.uavs):
            best = inf
            route = state.plan.routes[uav_id].task_ids
            first_position = len(state.commitment_prefixes[uav_id])
            for position in range(first_position, len(route) + 1):
                candidate = state.plan.copy()
                candidate.routes[uav_id].task_ids.insert(position, new_task.task_id)
                evaluation = self.optimizer.evaluate_plan(
                    candidate,
                    state.uavs,
                    state.tasks,
                    objective_task_ids=set(state.tasks),
                )
                if evaluation.feasible:
                    best = min(best, evaluation.objective - old_eval.objective)
            scores.append((best, uav_id))
        finite = [item for item in scores if item[0] < inf]
        if not finite:
            raise PlanningError(f"no feasible local insertion for task {new_task.task_id}")
        finite.sort()
        affected = {uav_id for _, uav_id in finite[:affected_count]}
        self.last_affected_uav_ids = affected

        routes: dict[int, Route] = {}
        released: list[int] = [new_task.task_id]
        prefix_lengths: dict[int, int] = {}
        for uav_id in sorted(state.uavs):
            old_route = state.plan.routes[uav_id].task_ids
            if uav_id in affected:
                prefix = list(state.commitment_prefixes[uav_id])
                routes[uav_id] = Route(uav_id, prefix)
                prefix_lengths[uav_id] = len(prefix)
                released.extend(old_route[len(prefix) :])
            else:
                routes[uav_id] = Route(uav_id, list(old_route))
                prefix_lengths[uav_id] = len(old_route)

        return self.optimizer.optimize(
            state.uavs,
            state.tasks,
            Plan(routes),
            sorted(released),
            fixed_prefix_lengths=prefix_lengths,
            mutable_uav_ids=affected,
            objective_task_ids=set(state.tasks),
        )
