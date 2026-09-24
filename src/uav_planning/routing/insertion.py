"""Shared Regret-2 insertion and first-improvement VNS engine."""

from __future__ import annotations

from dataclasses import dataclass
from math import inf
from time import perf_counter

from uav_planning.metrics.metrics import weighted_response_delay
from uav_planning.models import Plan, Route, Task, UAV
from uav_planning.routing.evaluator import RouteEvaluation, RouteEvaluator
from uav_planning.routing.neighborhoods import NEIGHBORHOODS


class PlanningError(RuntimeError):
    """Raised when no feasible complete assignment can be constructed."""


@dataclass(frozen=True)
class PlanEvaluation:
    """Feasibility, objective, and schedules for a complete plan."""

    feasible: bool
    objective: float
    route_evaluations: dict[int, RouteEvaluation]
    completion_times: dict[int, float]


class RouteOptimizer:
    """Regret-2 construction followed by the four MVP VNS neighborhoods."""

    def __init__(
        self,
        evaluator: RouteEvaluator,
        vns_max_iterations: int = 100,
        vns_time_limit_sec: float = 0.2,
    ) -> None:
        self.evaluator = evaluator
        self.vns_max_iterations = vns_max_iterations
        self.vns_time_limit_sec = vns_time_limit_sec

    def evaluate_plan(
        self,
        plan: Plan,
        uavs: dict[int, UAV],
        tasks: dict[int, Task],
        objective_task_ids: set[int] | None = None,
    ) -> PlanEvaluation:
        """Evaluate all routes and the frozen weighted-delay objective."""

        route_evaluations: dict[int, RouteEvaluation] = {}
        completion_times: dict[int, float] = {}
        seen: set[int] = set()
        feasible = True
        for uav_id in sorted(uavs):
            route = plan.routes[uav_id]
            duplicates = seen.intersection(route.task_ids)
            if duplicates:
                raise ValueError(f"tasks assigned more than once: {sorted(duplicates)}")
            seen.update(route.task_ids)
            evaluation = self.evaluator.evaluate_route(
                uavs[uav_id], [tasks[task_id] for task_id in route.task_ids]
            )
            route_evaluations[uav_id] = evaluation
            completion_times.update(evaluation.completion_times)
            feasible = feasible and evaluation.feasible

        selected = objective_task_ids if objective_task_ids is not None else seen
        if not selected.issubset(completion_times):
            feasible = False
            objective = inf
        else:
            objective = weighted_response_delay(tasks, completion_times, selected)
        return PlanEvaluation(feasible, objective, route_evaluations, completion_times)

    def optimize(
        self,
        uavs: dict[int, UAV],
        tasks: dict[int, Task],
        base_plan: Plan,
        tasks_to_insert: list[int],
        fixed_prefix_lengths: dict[int, int] | None = None,
        mutable_uav_ids: set[int] | None = None,
        objective_task_ids: set[int] | None = None,
    ) -> Plan:
        """Construct a feasible plan and improve it without crossing locks."""

        prefixes = fixed_prefix_lengths or {uav_id: 0 for uav_id in uavs}
        mutable = mutable_uav_ids or set(uavs)
        plan = base_plan.copy()
        remaining = list(tasks_to_insert)
        assigned = {
            task_id for route in plan.routes.values() for task_id in route.task_ids
        }
        if assigned.intersection(remaining):
            raise ValueError("tasks_to_insert already occur in base_plan")

        while remaining:
            choices: list[tuple[float, int, float, Plan]] = []
            for task_id in sorted(remaining):
                candidates: list[tuple[float, Plan]] = []
                selected = assigned | {task_id}
                for uav_id in sorted(mutable):
                    route = plan.routes[uav_id].task_ids
                    for position in range(prefixes.get(uav_id, 0), len(route) + 1):
                        candidate = plan.copy()
                        candidate.routes[uav_id].task_ids.insert(position, task_id)
                        evaluation = self.evaluate_plan(
                            candidate, uavs, tasks, objective_task_ids=selected
                        )
                        if evaluation.feasible:
                            candidates.append((evaluation.objective, candidate))
                if not candidates:
                    continue
                candidates.sort(key=lambda item: item[0])
                best_cost, best_plan = candidates[0]
                second_cost = candidates[1][0] if len(candidates) > 1 else best_cost
                regret = second_cost - best_cost
                choices.append((regret, task_id, best_cost, best_plan))

            if not choices:
                raise PlanningError(
                    f"no feasible insertion for remaining tasks {sorted(remaining)}"
                )
            choices.sort(key=lambda item: (-item[0], item[2], item[1]))
            _, chosen_task, _, plan = choices[0]
            remaining.remove(chosen_task)
            assigned.add(chosen_task)

        selected_objective = objective_task_ids or assigned
        return self._vns(
            plan,
            uavs,
            tasks,
            prefixes,
            mutable,
            selected_objective,
        )

    def _vns(
        self,
        initial_plan: Plan,
        uavs: dict[int, UAV],
        tasks: dict[int, Task],
        fixed_prefix_lengths: dict[int, int],
        mutable_uav_ids: set[int],
        objective_task_ids: set[int],
    ) -> Plan:
        current = initial_plan
        current_score = self.evaluate_plan(
            current, uavs, tasks, objective_task_ids
        ).objective
        started = perf_counter()
        iterations = 0
        while iterations < self.vns_max_iterations:
            if perf_counter() - started >= self.vns_time_limit_sec:
                break
            improved = False
            for neighborhood in NEIGHBORHOODS:
                for candidate in neighborhood(
                    current, fixed_prefix_lengths, mutable_uav_ids
                ):
                    if perf_counter() - started >= self.vns_time_limit_sec:
                        return current
                    evaluation = self.evaluate_plan(
                        candidate, uavs, tasks, objective_task_ids
                    )
                    if evaluation.feasible and evaluation.objective < current_score - 1e-9:
                        current = candidate
                        current_score = evaluation.objective
                        improved = True
                        break
                if improved:
                    break
            iterations += 1
            if not improved:
                break
        return current


def empty_plan(uavs: dict[int, UAV]) -> Plan:
    """Create an empty route for every UAV."""

    return Plan({uav_id: Route(uav_id, []) for uav_id in sorted(uavs)})

