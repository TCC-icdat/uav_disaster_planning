"""The four local-search neighborhoods frozen for the MVP."""

from __future__ import annotations

from collections.abc import Iterator

from uav_planning.models import Plan


def relocate_within_route(
    plan: Plan,
    fixed_prefix_lengths: dict[int, int],
    mutable_uav_ids: set[int],
) -> Iterator[Plan]:
    """Move one free task to another position on the same route."""

    for uav_id in sorted(mutable_uav_ids):
        sequence = plan.routes[uav_id].task_ids
        prefix = fixed_prefix_lengths.get(uav_id, 0)
        for source in range(prefix, len(sequence)):
            for destination in range(prefix, len(sequence)):
                if source == destination:
                    continue
                candidate = plan.copy()
                task_id = candidate.routes[uav_id].task_ids.pop(source)
                candidate.routes[uav_id].task_ids.insert(destination, task_id)
                yield candidate


def relocate_between_routes(
    plan: Plan,
    fixed_prefix_lengths: dict[int, int],
    mutable_uav_ids: set[int],
) -> Iterator[Plan]:
    """Move one free task to another mutable UAV."""

    for source_uav in sorted(mutable_uav_ids):
        source_sequence = plan.routes[source_uav].task_ids
        source_prefix = fixed_prefix_lengths.get(source_uav, 0)
        for source_index in range(source_prefix, len(source_sequence)):
            for destination_uav in sorted(mutable_uav_ids - {source_uav}):
                destination_sequence = plan.routes[destination_uav].task_ids
                destination_prefix = fixed_prefix_lengths.get(destination_uav, 0)
                for destination_index in range(
                    destination_prefix, len(destination_sequence) + 1
                ):
                    candidate = plan.copy()
                    task_id = candidate.routes[source_uav].task_ids.pop(source_index)
                    candidate.routes[destination_uav].task_ids.insert(
                        destination_index, task_id
                    )
                    yield candidate


def swap_between_routes(
    plan: Plan,
    fixed_prefix_lengths: dict[int, int],
    mutable_uav_ids: set[int],
) -> Iterator[Plan]:
    """Swap two free tasks assigned to different UAVs."""

    ordered = sorted(mutable_uav_ids)
    for left_position, left_uav in enumerate(ordered):
        left_sequence = plan.routes[left_uav].task_ids
        for right_uav in ordered[left_position + 1 :]:
            right_sequence = plan.routes[right_uav].task_ids
            for left_index in range(
                fixed_prefix_lengths.get(left_uav, 0), len(left_sequence)
            ):
                for right_index in range(
                    fixed_prefix_lengths.get(right_uav, 0), len(right_sequence)
                ):
                    candidate = plan.copy()
                    left_tasks = candidate.routes[left_uav].task_ids
                    right_tasks = candidate.routes[right_uav].task_ids
                    left_tasks[left_index], right_tasks[right_index] = (
                        right_tasks[right_index],
                        left_tasks[left_index],
                    )
                    yield candidate


def two_opt_within_route(
    plan: Plan,
    fixed_prefix_lengths: dict[int, int],
    mutable_uav_ids: set[int],
) -> Iterator[Plan]:
    """Reverse a free subsequence on one route."""

    for uav_id in sorted(mutable_uav_ids):
        sequence = plan.routes[uav_id].task_ids
        prefix = fixed_prefix_lengths.get(uav_id, 0)
        for left in range(prefix, len(sequence) - 1):
            for right in range(left + 1, len(sequence)):
                candidate = plan.copy()
                route = candidate.routes[uav_id].task_ids
                route[left : right + 1] = reversed(route[left : right + 1])
                yield candidate


NEIGHBORHOODS = (
    relocate_within_route,
    relocate_between_routes,
    swap_between_routes,
    two_opt_within_route,
)

