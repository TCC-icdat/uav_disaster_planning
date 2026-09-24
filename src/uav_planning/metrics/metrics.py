"""Objective and experiment metric calculations."""

from __future__ import annotations

from collections.abc import Iterable

from uav_planning.models import Plan, Task


def weighted_response_delay(
    tasks: dict[int, Task],
    completion_times: dict[int, float],
    task_ids: Iterable[int] | None = None,
) -> float:
    """Return sum(priority * (completion - release)) for selected tasks."""

    selected = set(task_ids) if task_ids is not None else set(tasks)
    missing = selected - completion_times.keys()
    if missing:
        raise ValueError(f"completion times missing for tasks: {sorted(missing)}")
    return sum(
        tasks[task_id].priority
        * (completion_times[task_id] - tasks[task_id].release_time)
        for task_id in selected
    )


def _assignment(plan: Plan) -> dict[int, int]:
    return {
        task_id: uav_id
        for uav_id, route in plan.routes.items()
        for task_id in route.task_ids
    }


def assignment_change_count(
    old_plan: Plan,
    new_plan: Plan,
    task_ids: Iterable[int],
) -> int:
    """Count common pending tasks assigned to a different UAV."""

    old_assignment = _assignment(old_plan)
    new_assignment = _assignment(new_plan)
    return sum(
        old_assignment.get(task_id) != new_assignment.get(task_id)
        for task_id in task_ids
        if task_id in old_assignment and task_id in new_assignment
    )


def _successor_edges(plan: Plan, selected: set[int]) -> set[tuple[int, int, int]]:
    edges: set[tuple[int, int, int]] = set()
    for uav_id, route in plan.routes.items():
        sequence = [task_id for task_id in route.task_ids if task_id in selected]
        edges.update((uav_id, left, right) for left, right in zip(sequence, sequence[1:]))
    return edges


def changed_successor_edges(
    old_plan: Plan,
    new_plan: Plan,
    task_ids: Iterable[int],
) -> int:
    """Return the symmetric difference of pending-task successor arcs."""

    selected = set(task_ids)
    return len(
        _successor_edges(old_plan, selected)
        ^ _successor_edges(new_plan, selected)
    )
