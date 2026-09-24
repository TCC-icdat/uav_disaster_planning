"""Release-event simulation and experiment metric collection."""

from __future__ import annotations

from dataclasses import dataclass
from time import perf_counter
from typing import Protocol

from uav_planning.metrics.metrics import (
    assignment_change_count,
    changed_successor_edges,
    weighted_response_delay,
)
from uav_planning.models import (
    Plan,
    ReplanningRecord,
    SimulationState,
    Task,
    UAVExecutionState,
)
from uav_planning.planners.initial_planner import InitialPlanner
from uav_planning.routing.insertion import RouteOptimizer
from uav_planning.scenario.event_generator import EventGenerator
from uav_planning.scenario.generator import Scenario
from uav_planning.simulation.event_manager import EventManager, EventType


class DynamicPlanner(Protocol):
    name: str

    def replan(self, state: SimulationState, new_task: Task) -> Plan: ...


@dataclass(frozen=True)
class SimulationResult:
    """Final plan, scalar metrics, event log, and plan snapshots."""

    metrics: dict[str, float | int | str | bool]
    final_plan: Plan
    initial_plan: Plan
    snapshots: tuple[tuple[float, Plan], ...]
    records: tuple[ReplanningRecord, ...]
    event_log: tuple[dict[str, float | int | str], ...]


class Simulator:
    """Simulate task releases without exposing future tasks to planners."""

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
        """Run one strategy through all chronologically released tasks."""

        released = {task.task_id: task for task in self.scenario.initial_tasks}
        plan = self.initial_planner.plan(
            list(self.scenario.uavs), list(self.scenario.initial_tasks), current_time=0.0
        )
        initial_plan = plan.copy()
        snapshots: list[tuple[float, Plan]] = [(0.0, plan.copy())]
        records: list[ReplanningRecord] = []
        event_log: list[dict[str, float | int | str]] = []

        manager = EventManager()
        for task in EventGenerator(self.scenario.dynamic_tasks).releases():
            manager.push(task.release_time, EventType.TASK_RELEASE, task_id=task.task_id)
        dynamic_by_id = {task.task_id: task for task in self.scenario.dynamic_tasks}

        while manager:
            event = manager.pop()
            if event.event_type is not EventType.TASK_RELEASE:
                continue
            task = dynamic_by_id[int(event.payload["task_id"])]
            visible = dict(released)
            visible[task.task_id] = task
            state = self._build_state(plan, visible, event.time)
            started = perf_counter()
            new_plan = dynamic_planner.replan(state, task)
            runtime = perf_counter() - started
            self._validate_task_set(new_plan, set(visible))
            pending_common = set(released) - state.completed_task_ids
            records.append(
                ReplanningRecord(
                    release_time=event.time,
                    task_id=task.task_id,
                    runtime_sec=runtime,
                    assignment_changes=assignment_change_count(
                        plan, new_plan, pending_common
                    ),
                    successor_edge_changes=changed_successor_edges(
                        plan, new_plan, pending_common
                    ),
                )
            )
            event_log.append(
                {
                    "time": event.time,
                    "event": EventType.TASK_RELEASE.value,
                    "task_id": task.task_id,
                }
            )
            released = visible
            plan = new_plan
            snapshots.append((event.time, plan.copy()))

        final_evaluation = self.optimizer.evaluate_plan(
            plan, self.uavs, released, objective_task_ids=set(released)
        )
        self._append_execution_events(event_log, final_evaluation.route_evaluations)
        event_log.sort(key=lambda item: (float(item["time"]), str(item["event"])))
        metrics = self._metrics(dynamic_planner.name, released, final_evaluation, records)
        return SimulationResult(
            metrics=metrics,
            final_plan=plan,
            initial_plan=initial_plan,
            snapshots=tuple(snapshots),
            records=tuple(records),
            event_log=tuple(event_log),
        )

    def _build_state(
        self,
        plan: Plan,
        tasks: dict[int, Task],
        current_time: float,
    ) -> SimulationState:
        execution_states: dict[int, UAVExecutionState] = {}
        completed_all: set[int] = set()
        executing: dict[int, int] = {}
        locked_prefixes: dict[int, list[int]] = {}
        commitment_prefixes: dict[int, list[int]] = {}

        for uav_id, uav in self.uavs.items():
            sequence = plan.routes[uav_id].task_ids
            evaluation = self.optimizer.evaluator.evaluate_route(
                uav, [tasks[task_id] for task_id in sequence]
            )
            completed = [
                task_id
                for task_id in sequence
                if evaluation.completion_times[task_id] <= current_time + 1e-9
            ]
            completed_all.update(completed)
            base_count = len(completed)
            if base_count < len(sequence):
                task_id = sequence[base_count]
                start = evaluation.start_times[task_id]
                completion = evaluation.completion_times[task_id]
                arrival = evaluation.arrival_times[task_id]
                previous_end = (
                    0.0
                    if base_count == 0
                    else evaluation.completion_times[sequence[base_count - 1]]
                )
                if start <= current_time < completion:
                    base_count += 1
                    executing[uav_id] = task_id
                elif previous_end <= current_time < arrival:
                    base_count += 1
            commitment_count = min(len(sequence), base_count + 1)
            locked = list(sequence[:base_count])
            committed = list(sequence[:commitment_count])
            locked_prefixes[uav_id] = locked
            commitment_prefixes[uav_id] = committed
            pose = tasks[locked[-1]].pose if locked else uav.start_pose
            execution_states[uav_id] = UAVExecutionState(
                uav_id=uav_id,
                current_time=current_time,
                current_pose=pose,
                completed_task_ids=completed,
                committed_task_ids=list(sequence[base_count:commitment_count]),
                remaining_task_ids=list(sequence[base_count:]),
            )

        return SimulationState(
            current_time=current_time,
            uavs=self.uavs,
            tasks=tasks,
            plan=plan.copy(),
            execution_states=execution_states,
            completed_task_ids=completed_all,
            executing_task_ids=executing,
            locked_prefixes=locked_prefixes,
            commitment_prefixes=commitment_prefixes,
        )

    @staticmethod
    def _validate_task_set(plan: Plan, expected: set[int]) -> None:
        assigned = [task_id for route in plan.routes.values() for task_id in route.task_ids]
        if len(assigned) != len(set(assigned)):
            raise ValueError("a task is assigned more than once")
        if set(assigned) != expected:
            raise ValueError(
                f"plan task mismatch: missing={sorted(expected - set(assigned))}, "
                f"extra={sorted(set(assigned) - expected)}"
            )

    def _metrics(self, strategy: str, tasks, evaluation, records):
        completion = evaluation.completion_times
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
            route_eval.start_times[task_id] + 1e-9 >= tasks[task_id].release_time
            for route_eval in evaluation.route_evaluations.values()
            for task_id in route_eval.start_times
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
                item.total_travel_time for item in evaluation.route_evaluations.values()
            ),
            "total_service_time": sum(
                item.total_service_time for item in evaluation.route_evaluations.values()
            ),
            "replanning_count": len(records),
            "total_replanning_runtime": sum(runtimes),
            "mean_replanning_runtime": sum(runtimes) / len(runtimes) if runtimes else 0.0,
            "max_replanning_runtime": max(runtimes, default=0.0),
            "assignment_changes": sum(item.assignment_changes for item in records),
            "successor_edge_changes": sum(
                item.successor_edge_changes for item in records
            ),
            "feasible": bool(evaluation.feasible and starts_valid),
        }

    @staticmethod
    def _append_execution_events(event_log, route_evaluations) -> None:
        for uav_id, evaluation in route_evaluations.items():
            for task_id, start in evaluation.start_times.items():
                event_log.append(
                    {
                        "time": start,
                        "event": EventType.TASK_START.value,
                        "task_id": task_id,
                        "uav_id": uav_id,
                    }
                )
                event_log.append(
                    {
                        "time": evaluation.completion_times[task_id],
                        "event": EventType.TASK_COMPLETE.value,
                        "task_id": task_id,
                        "uav_id": uav_id,
                    }
                )
            event_log.append(
                {
                    "time": evaluation.return_time,
                    "event": EventType.RETURN_DEPOT.value,
                    "uav_id": uav_id,
                }
            )

