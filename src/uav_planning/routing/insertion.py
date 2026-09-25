"""Shared Regret-2 insertion and multi-neighborhood local search engine."""

from __future__ import annotations

from dataclasses import dataclass
from math import inf
from time import perf_counter

from uav_planning.metrics.metrics import weighted_response_delay
from uav_planning.models import Plan, PlanningAnchor, Route, Task, UAV
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


def regret2_value(candidate_costs: list[float]) -> float:
    """Return Regret-2 value with feasibility-first single-option handling."""

    if not candidate_costs:
        raise ValueError("candidate_costs must not be empty")
    ordered = sorted(candidate_costs)
    if len(ordered) == 1:
        return inf
    return ordered[1] - ordered[0]


class RouteOptimizer:
    """Regret-2 plus deterministic multi-neighborhood local search."""

    def __init__(
        self,
        evaluator: RouteEvaluator,
        local_search_max_iterations: int = 100,
        local_search_time_limit_sec: float = 0.2,
    ) -> None:
        self.evaluator = evaluator
        self.local_search_max_iterations = local_search_max_iterations
        self.local_search_time_limit_sec = local_search_time_limit_sec

    def evaluate_plan(
        self,
        plan: Plan,
        uavs: dict[int, UAV],
        tasks: dict[int, Task],
        objective_task_ids: set[int] | None = None,
        anchors: dict[int, PlanningAnchor] | None = None,
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
            anchor = (
                anchors[uav_id]
                if anchors is not None
                else PlanningAnchor(uav_id, 0.0, uavs[uav_id].start_pose)
            )
            evaluation = self.evaluator.evaluate_route(
                uavs[uav_id],
                [tasks[task_id] for task_id in route.task_ids],
                start_time=anchor.time,
                start_pose=anchor.pose,
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
        anchors: dict[int, PlanningAnchor] | None = None,
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
                            candidate,
                            uavs,
                            tasks,
                            objective_task_ids=selected,
                            anchors=anchors,
                        )
                        if evaluation.feasible:
                            candidates.append((evaluation.objective, candidate))
                if not candidates:
                    continue
                candidates.sort(key=lambda item: item[0])
                best_cost, best_plan = candidates[0]
                regret = regret2_value([cost for cost, _ in candidates])
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
        return self.local_search(
            plan,
            uavs,
            tasks,
            prefixes,
            mutable,
            selected_objective,
            anchors=anchors,
        )

    def local_search(
        self,
        initial_plan: Plan,
        uavs: dict[int, UAV],
        tasks: dict[int, Task],
        fixed_prefix_lengths: dict[int, int],
        mutable_uav_ids: set[int],
        objective_task_ids: set[int],
        anchors: dict[int, PlanningAnchor] | None = None,
    ) -> Plan:
        """Apply strict first-improvement descent over the four neighborhoods."""

        current = initial_plan
        current_score = self.evaluate_plan(
            current, uavs, tasks, objective_task_ids, anchors=anchors
        ).objective
        started = perf_counter()
        iterations = 0
        while iterations < self.local_search_max_iterations:
            if perf_counter() - started >= self.local_search_time_limit_sec:
                break
            improved = False
            for neighborhood in NEIGHBORHOODS:
                for candidate in neighborhood(
                    current, fixed_prefix_lengths, mutable_uav_ids
                ):
                    if perf_counter() - started >= self.local_search_time_limit_sec:
                        return current
                    evaluation = self.evaluate_plan(
                        candidate,
                        uavs,
                        tasks,
                        objective_task_ids,
                        anchors=anchors,
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

