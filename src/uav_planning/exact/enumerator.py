"""Exhaustive labeled-route enumeration for at most eight tasks."""

from __future__ import annotations

from dataclasses import dataclass
from itertools import permutations
from time import perf_counter

from uav_planning.models import Plan, PlanningAnchor, Route, Task, UAV
from uav_planning.routing.insertion import PlanEvaluation, PlanningError, RouteOptimizer


@dataclass(frozen=True)
class ExactSolution:
    """Optimal small-instance plan and enumeration diagnostics."""

    plan: Plan
    objective: float
    evaluation: PlanEvaluation
    runtime_sec: float
    evaluated_plan_count: int


def _weak_compositions(total: int, parts: int):
    """Yield ordered nonnegative lengths summing to ``total``."""

    if parts == 1:
        yield (total,)
        return
    for first in range(total + 1):
        for rest in _weak_compositions(total - first, parts - 1):
            yield (first, *rest)


class ExactEnumerator:
    """Enumerate every task order and split across labeled UAV routes.

    This solver is intentionally guarded against use in scale experiments.
    """

    def __init__(
        self,
        optimizer: RouteOptimizer,
        max_tasks: int = 8,
        max_uavs: int = 3,
    ) -> None:
        self.optimizer = optimizer
        self.max_tasks = max_tasks
        self.max_uavs = max_uavs

    def solve(
        self,
        uavs: dict[int, UAV],
        tasks: dict[int, Task],
        anchors: dict[int, PlanningAnchor] | None = None,
    ) -> ExactSolution:
        """Return the globally optimal feasible plan under the chosen objective."""

        if len(tasks) > self.max_tasks:
            raise ValueError(
                f"ExactEnumerator supports at most {self.max_tasks} tasks"
            )
        if len(uavs) > self.max_uavs:
            raise ValueError(
                f"ExactEnumerator supports at most {self.max_uavs} UAVs"
            )
        if not uavs:
            raise ValueError("at least one UAV is required")

        uav_ids = sorted(uavs)
        task_ids = sorted(tasks)
        splits = tuple(_weak_compositions(len(task_ids), len(uav_ids)))
        best_plan: Plan | None = None
        best_evaluation: PlanEvaluation | None = None
        evaluated = 0
        started = perf_counter()
        for ordering in permutations(task_ids):
            for lengths in splits:
                routes: dict[int, Route] = {}
                offset = 0
                for uav_id, length in zip(uav_ids, lengths):
                    routes[uav_id] = Route(
                        uav_id, list(ordering[offset : offset + length])
                    )
                    offset += length
                plan = Plan(routes)
                evaluation = self.optimizer.evaluate_plan(
                    plan,
                    uavs,
                    tasks,
                    objective_task_ids=set(task_ids),
                    anchors=anchors,
                )
                evaluated += 1
                if not evaluation.feasible:
                    continue
                if (
                    best_evaluation is None
                    or evaluation.objective < best_evaluation.objective - 1e-12
                ):
                    best_plan = plan
                    best_evaluation = evaluation
        runtime = perf_counter() - started
        if best_plan is None or best_evaluation is None:
            raise PlanningError("exact enumeration found no feasible plan")
        return ExactSolution(
            plan=best_plan,
            objective=best_evaluation.objective,
            evaluation=best_evaluation,
            runtime_sec=runtime,
            evaluated_plan_count=evaluated,
        )
