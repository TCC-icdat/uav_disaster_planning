"""Forward-only release-event simulation with immutable execution history."""

from __future__ import annotations

from dataclasses import dataclass, field
from time import perf_counter
from typing import Protocol

from uav_planning.metrics.metrics import (
    assignment_change_count,
    changed_successor_edges,
    weighted_response_delay,
)
from uav_planning.models import (
    Plan,
    PlanningAnchor,
    ReplanningRecord,
    ReturnExecutionRecord,
    Route,
    SimulationState,
    Task,
    TaskExecutionRecord,
    UAVExecutionState,
)
from uav_planning.planners.initial_planner import InitialPlanner
from uav_planning.routing.evaluator import RouteEvaluation
from uav_planning.routing.insertion import RouteOptimizer, empty_plan
from uav_planning.scenario.event_generator import EventGenerator
from uav_planning.scenario.generator import Scenario
from uav_planning.simulation.event_manager import EventManager, EventType

_EPS = 1e-9


class DynamicPlanner(Protocol):
    name: str

    def replan(self, state: SimulationState, new_task: Task) -> Plan: ...


@dataclass
class _RuntimeState:
    """Mutable simulator-owned state; planners only receive copies."""

    plan: Plan
    anchors: dict[int, PlanningAnchor]
    fixed_tasks: dict[int, TaskExecutionRecord] = field(default_factory=dict)
    returns: dict[int, ReturnExecutionRecord] = field(default_factory=dict)
    history: dict[int, TaskExecutionRecord] = field(default_factory=dict)
    return_history: list[ReturnExecutionRecord] = field(default_factory=list)


@dataclass(frozen=True)
class SimulationResult:
    """Final metrics, immutable execution history, and plan snapshots."""

    metrics: dict[str, float | int | str | bool]
    final_plan: Plan
    initial_plan: Plan
    snapshots: tuple[tuple[float, Plan], ...]
    history_snapshots: tuple[tuple[float, dict[int, TaskExecutionRecord]], ...]
    history: dict[int, TaskExecutionRecord]
    records: tuple[ReplanningRecord, ...]
    event_log: tuple[dict[str, float | int | str], ...]


class Simulator:
    """Advance execution to each release before planning any future suffix."""

    def __init__(
        self,
        scenario: Scenario,
        initial_planner: InitialPlanner,
        optimizer: RouteOptimizer,
        high_priority_threshold: float = 7.0,
    ) -> None:
        self.scenario = scenario
        self.initial_planner = initial_planner
        self.optimizer = optimizer
        self.high_priority_threshold = high_priority_threshold
        self.uavs = {uav.uav_id: uav for uav in scenario.uavs}

    def run(self, dynamic_planner: DynamicPlanner) -> SimulationResult:
        """Run one dynamic policy without allowing replanning into the past."""

        released = {task.task_id: task for task in self.scenario.initial_tasks}
        initial_plan = self.initial_planner.plan(
            list(self.scenario.uavs), list(self.scenario.initial_tasks), current_time=0.0
        )
        runtime = _RuntimeState(
            plan=initial_plan.copy(),
            anchors={
                uav_id: PlanningAnchor(uav_id, 0.0, uav.start_pose)
                for uav_id, uav in self.uavs.items()
            },
        )
        snapshots: list[tuple[float, Plan]] = [(0.0, initial_plan.copy())]
        history_snapshots: list[
            tuple[float, dict[int, TaskExecutionRecord]]
        ] = []
        records: list[ReplanningRecord] = []
        release_events: list[dict[str, float | int | str]] = []
        time_consistent = True

        manager = EventManager()
        for task in EventGenerator(self.scenario.dynamic_tasks).releases():
            manager.push(task.release_time, EventType.TASK_RELEASE, task_id=task.task_id)
        dynamic_by_id = {task.task_id: task for task in self.scenario.dynamic_tasks}

        while manager:
            event = manager.pop()
            task = dynamic_by_id[int(event.payload["task_id"])]
            self._advance_to(runtime, event.time, released)
            history_before = dict(runtime.history)
            old_free_plan = runtime.plan.copy()
            released[task.task_id] = task
            state = self._make_planning_state(runtime, released, event.time)

            started = perf_counter()
            new_plan = dynamic_planner.replan(state, task)
            runtime_sec = perf_counter() - started
            expected_free = {
                task_id
                for route in old_free_plan.routes.values()
                for task_id in route.task_ids
            } | {task.task_id}
            self._validate_free_task_set(new_plan, expected_free)
            time_consistent = time_consistent and self._is_forward_plan(
                new_plan, runtime.anchors, released, event.time
            )
            if runtime.history != history_before:
                raise AssertionError("a replanner modified immutable execution history")

            common_free = expected_free - {task.task_id}
            records.append(
                ReplanningRecord(
                    release_time=event.time,
                    task_id=task.task_id,
                    runtime_sec=runtime_sec,
                    assignment_changes=assignment_change_count(
                        old_free_plan, new_plan, common_free
                    ),
                    successor_edge_changes=changed_successor_edges(
                        old_free_plan, new_plan, common_free
                    ),
                )
            )
            release_events.append(
                {
                    "time": event.time,
                    "event": EventType.TASK_RELEASE.value,
                    "task_id": task.task_id,
                }
            )
            runtime.plan = new_plan
            history_snapshots.append((event.time, dict(runtime.history)))
            snapshots.append((event.time, self._compose_plan(runtime)))

        final_return_times = self._finish_all(runtime, released)
        self._validate_complete_history(runtime.history, released)
        final_plan = self._history_plan(runtime.history)
        event_log = self._build_event_log(
            release_events, runtime.history, runtime.return_history
        )
        metrics = self._metrics(
            dynamic_planner.name,
            released,
            runtime,
            final_return_times,
            records,
            time_consistent,
        )
        return SimulationResult(
            metrics=metrics,
            final_plan=final_plan,
            initial_plan=initial_plan,
            snapshots=tuple(snapshots),
            history_snapshots=tuple(history_snapshots),
            history=dict(runtime.history),
            records=tuple(records),
            event_log=tuple(event_log),
        )

    def _advance_to(
        self,
        runtime: _RuntimeState,
        current_time: float,
        tasks: dict[int, Task],
    ) -> None:
        """Commit everything that has physically begun by ``current_time``."""

        for uav_id, uav in self.uavs.items():
            fixed = runtime.fixed_tasks.get(uav_id)
            if fixed is not None:
                if fixed.completion_time <= current_time + _EPS:
                    self._store_history(runtime.history, fixed)
                    del runtime.fixed_tasks[uav_id]
                else:
                    continue

            returning = runtime.returns.get(uav_id)
            if returning is not None:
                if returning.completion_time <= current_time + _EPS:
                    runtime.return_history.append(returning)
                    del runtime.returns[uav_id]
                else:
                    continue

            route_ids = list(runtime.plan.routes[uav_id].task_ids)
            anchor = runtime.anchors[uav_id]
            evaluation = self.optimizer.evaluator.evaluate_route(
                uav,
                [tasks[task_id] for task_id in route_ids],
                start_time=anchor.time,
                start_pose=anchor.pose,
            )
            consumed = 0
            locked_now = False
            for task_id in route_ids:
                record = self._task_record(uav_id, task_id, evaluation)
                if record.completion_time <= current_time + _EPS:
                    self._store_history(runtime.history, record)
                    runtime.anchors[uav_id] = PlanningAnchor(
                        uav_id, record.completion_time, tasks[task_id].pose
                    )
                    consumed += 1
                    continue
                if record.departure_time <= current_time + _EPS:
                    runtime.fixed_tasks[uav_id] = record
                    runtime.anchors[uav_id] = PlanningAnchor(
                        uav_id, record.completion_time, tasks[task_id].pose
                    )
                    consumed += 1
                    locked_now = True
                break

            if consumed:
                runtime.plan.routes[uav_id].task_ids = route_ids[consumed:]
            if locked_now or runtime.plan.routes[uav_id].task_ids:
                continue

            # With no free work, return is non-preemptible once it has begun.
            return_departure = (
                evaluation.completion_times[route_ids[-1]]
                if route_ids
                else anchor.time
            )
            return_travel = evaluation.total_travel_time - sum(
                evaluation.leg_travel_times.values()
            )
            return_record = ReturnExecutionRecord(
                uav_id=uav_id,
                departure_time=return_departure,
                completion_time=evaluation.return_time,
                travel_time=return_travel,
            )
            if return_record.completion_time <= current_time + _EPS:
                if return_record.travel_time > _EPS:
                    runtime.return_history.append(return_record)
                runtime.anchors[uav_id] = PlanningAnchor(
                    uav_id, current_time, uav.start_pose
                )
            elif return_record.departure_time <= current_time + _EPS:
                runtime.returns[uav_id] = return_record
                runtime.anchors[uav_id] = PlanningAnchor(
                    uav_id, return_record.completion_time, uav.start_pose
                )

        if any(anchor.time < current_time - _EPS for anchor in runtime.anchors.values()):
            raise AssertionError("free-plan anchor remained in the past")

    def _make_planning_state(
        self,
        runtime: _RuntimeState,
        tasks: dict[int, Task],
        current_time: float,
    ) -> SimulationState:
        execution_states: dict[int, UAVExecutionState] = {}
        completed_by_uav: dict[int, list[int]] = {uav_id: [] for uav_id in self.uavs}
        for record in runtime.history.values():
            completed_by_uav[record.uav_id].append(record.task_id)
        executing: dict[int, int] = {}
        for uav_id, record in runtime.fixed_tasks.items():
            if record.start_time <= current_time < record.completion_time:
                executing[uav_id] = record.task_id
        for uav_id in self.uavs:
            fixed = runtime.fixed_tasks.get(uav_id)
            execution_states[uav_id] = UAVExecutionState(
                uav_id=uav_id,
                current_time=current_time,
                current_pose=runtime.anchors[uav_id].pose,
                completed_task_ids=sorted(completed_by_uav[uav_id]),
                committed_task_ids=[fixed.task_id] if fixed is not None else [],
                remaining_task_ids=list(runtime.plan.routes[uav_id].task_ids),
            )
        return SimulationState(
            current_time=current_time,
            uavs=self.uavs,
            tasks=tasks,
            plan=runtime.plan.copy(),
            execution_states=execution_states,
            completed_task_ids=set(runtime.history),
            executing_task_ids=executing,
            anchors=dict(runtime.anchors),
            locked_task_ids={
                uav_id: record.task_id
                for uav_id, record in runtime.fixed_tasks.items()
            },
            history=dict(runtime.history),
        )

    def _is_forward_plan(
        self,
        plan: Plan,
        anchors: dict[int, PlanningAnchor],
        tasks: dict[int, Task],
        event_time: float,
    ) -> bool:
        evaluation = self.optimizer.evaluate_plan(
            plan, self.uavs, tasks, anchors=anchors
        )
        return evaluation.feasible and all(
            start + _EPS >= event_time
            for route in evaluation.route_evaluations.values()
            for start in route.start_times.values()
        )

    def _finish_all(
        self,
        runtime: _RuntimeState,
        tasks: dict[int, Task],
    ) -> dict[int, float]:
        final_return_times: dict[int, float] = {}
        for uav_id, uav in self.uavs.items():
            fixed = runtime.fixed_tasks.pop(uav_id, None)
            if fixed is not None:
                self._store_history(runtime.history, fixed)
            returning = runtime.returns.pop(uav_id, None)
            if returning is not None:
                runtime.return_history.append(returning)
            anchor = runtime.anchors[uav_id]
            route_ids = runtime.plan.routes[uav_id].task_ids
            evaluation = self.optimizer.evaluator.evaluate_route(
                uav,
                [tasks[task_id] for task_id in route_ids],
                start_time=anchor.time,
                start_pose=anchor.pose,
            )
            for task_id in route_ids:
                self._store_history(
                    runtime.history, self._task_record(uav_id, task_id, evaluation)
                )
            return_travel = evaluation.total_travel_time - sum(
                evaluation.leg_travel_times.values()
            )
            runtime.return_history.append(
                ReturnExecutionRecord(
                    uav_id=uav_id,
                    departure_time=(
                        evaluation.completion_times[route_ids[-1]]
                        if route_ids
                        else anchor.time
                    ),
                    completion_time=evaluation.return_time,
                    travel_time=return_travel,
                )
            )
            final_return_times[uav_id] = evaluation.return_time
            runtime.anchors[uav_id] = PlanningAnchor(
                uav_id, evaluation.return_time, uav.start_pose
            )
        runtime.plan = empty_plan(self.uavs)
        return final_return_times

    @staticmethod
    def _task_record(
        uav_id: int, task_id: int, evaluation: RouteEvaluation
    ) -> TaskExecutionRecord:
        return TaskExecutionRecord(
            task_id=task_id,
            uav_id=uav_id,
            departure_time=evaluation.departure_times[task_id],
            arrival_time=evaluation.arrival_times[task_id],
            start_time=evaluation.start_times[task_id],
            completion_time=evaluation.completion_times[task_id],
            travel_time=evaluation.leg_travel_times[task_id],
        )

    @staticmethod
    def _store_history(
        history: dict[int, TaskExecutionRecord], record: TaskExecutionRecord
    ) -> None:
        previous = history.get(record.task_id)
        if previous is not None and previous != record:
            raise AssertionError(f"execution history changed for task {record.task_id}")
        history[record.task_id] = record

    @staticmethod
    def _validate_free_task_set(plan: Plan, expected: set[int]) -> None:
        assigned = [task_id for route in plan.routes.values() for task_id in route.task_ids]
        if len(assigned) != len(set(assigned)) or set(assigned) != expected:
            raise ValueError("replanner did not preserve the exact free-task set")

    @staticmethod
    def _validate_complete_history(
        history: dict[int, TaskExecutionRecord], tasks: dict[int, Task]
    ) -> None:
        if set(history) != set(tasks):
            raise ValueError("final execution history is incomplete")

    def _compose_plan(self, runtime: _RuntimeState) -> Plan:
        routes: dict[int, Route] = {}
        for uav_id in self.uavs:
            completed = sorted(
                (
                    record
                    for record in runtime.history.values()
                    if record.uav_id == uav_id
                ),
                key=lambda record: record.departure_time,
            )
            task_ids = [record.task_id for record in completed]
            fixed = runtime.fixed_tasks.get(uav_id)
            if fixed is not None:
                task_ids.append(fixed.task_id)
            task_ids.extend(runtime.plan.routes[uav_id].task_ids)
            routes[uav_id] = Route(uav_id, task_ids)
        return Plan(routes)

    def _history_plan(self, history: dict[int, TaskExecutionRecord]) -> Plan:
        routes: dict[int, Route] = {}
        for uav_id in self.uavs:
            records = sorted(
                (record for record in history.values() if record.uav_id == uav_id),
                key=lambda record: record.departure_time,
            )
            routes[uav_id] = Route(uav_id, [record.task_id for record in records])
        return Plan(routes)

    @staticmethod
    def _build_event_log(releases, history, returns):
        log = list(releases)
        for record in history.values():
            log.append(
                {
                    "time": record.start_time,
                    "event": EventType.TASK_START.value,
                    "task_id": record.task_id,
                    "uav_id": record.uav_id,
                }
            )
            log.append(
                {
                    "time": record.completion_time,
                    "event": EventType.TASK_COMPLETE.value,
                    "task_id": record.task_id,
                    "uav_id": record.uav_id,
                }
            )
        for record in returns:
            log.append(
                {
                    "time": record.completion_time,
                    "event": EventType.RETURN_DEPOT.value,
                    "uav_id": record.uav_id,
                }
            )
        return sorted(log, key=lambda item: (float(item["time"]), str(item["event"])))

    def _metrics(
        self,
        strategy: str,
        tasks: dict[int, Task],
        runtime: _RuntimeState,
        final_return_times: dict[int, float],
        records: list[ReplanningRecord],
        time_consistent: bool,
    ) -> dict[str, float | int | str | bool]:
        completion = {
            task_id: record.completion_time
            for task_id, record in runtime.history.items()
        }
        delays = {
            task_id: completion[task_id] - task.release_time
            for task_id, task in tasks.items()
        }
        high = [
            delays[task_id]
            for task_id, task in tasks.items()
            if task.priority >= self.high_priority_threshold
        ]
        runtimes = [record.runtime_sec for record in records]
        starts_valid = all(
            runtime.history[task_id].start_time + _EPS >= task.release_time
            for task_id, task in tasks.items()
        )
        endurance_valid = all(
            final_return_times[uav_id] <= self.uavs[uav_id].max_mission_time + _EPS
            for uav_id in self.uavs
        )
        return {
            "seed": self.scenario.seed,
            "strategy": strategy,
            "travel_model": self.optimizer.evaluator.travel_time_provider.name,
            "task_count": len(tasks),
            "uav_count": len(self.uavs),
            "weighted_delay": weighted_response_delay(tasks, completion),
            "mean_delay": sum(delays.values()) / len(delays),
            "high_priority_mean_delay": sum(high) / len(high) if high else 0.0,
            "makespan": max(completion.values(), default=0.0),
            "total_travel_time": sum(
                record.travel_time for record in runtime.history.values()
            )
            + sum(record.travel_time for record in runtime.return_history),
            "total_service_time": sum(task.service_time for task in tasks.values()),
            "replanning_count": len(records),
            "total_replanning_runtime": sum(runtimes),
            "mean_replanning_runtime": sum(runtimes) / len(runtimes) if runtimes else 0.0,
            "max_replanning_runtime": max(runtimes, default=0.0),
            "assignment_changes": sum(item.assignment_changes for item in records),
            "successor_edge_changes": sum(
                item.successor_edge_changes for item in records
            ),
            "time_consistency_check": bool(time_consistent),
            "feasible": bool(time_consistent and starts_valid and endurance_valid),
        }
