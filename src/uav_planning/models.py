"""Core immutable problem data and mutable planning state."""

from __future__ import annotations

from dataclasses import dataclass, field
from math import pi


@dataclass(frozen=True)
class Pose2D:
    """Planar position and heading in radians."""

    x: float
    y: float
    heading: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "heading", self.heading % (2.0 * pi))


@dataclass(frozen=True)
class Task:
    """A released SAR review task."""

    task_id: int
    x: float
    y: float
    release_time: float
    priority: float
    service_time: float
    required_heading: float
    source: str = "EO"

    @property
    def pose(self) -> Pose2D:
        """Return the required service-entry pose."""

        return Pose2D(self.x, self.y, self.required_heading)


@dataclass(frozen=True)
class UAV:
    """Fixed-wing SAR platform parameters."""

    uav_id: int
    speed: float
    min_turn_radius: float
    max_mission_time: float
    start_pose: Pose2D


@dataclass
class Route:
    """Ordered task identifiers assigned to one UAV."""

    uav_id: int
    task_ids: list[int] = field(default_factory=list)


@dataclass
class Plan:
    """A route for each UAV."""

    routes: dict[int, Route]

    def copy(self) -> "Plan":
        """Return a deep-enough copy for neighborhood moves."""

        return Plan(
            routes={
                uav_id: Route(uav_id, list(route.task_ids))
                for uav_id, route in self.routes.items()
            }
        )


@dataclass
class UAVExecutionState:
    """Execution information available at a replanning event."""

    uav_id: int
    current_time: float
    current_pose: Pose2D
    completed_task_ids: list[int] = field(default_factory=list)
    committed_task_ids: list[int] = field(default_factory=list)
    remaining_task_ids: list[int] = field(default_factory=list)


@dataclass(frozen=True)
class PlanningAnchor:
    """Earliest pose and time from which a UAV may execute a free suffix."""

    uav_id: int
    time: float
    pose: Pose2D


@dataclass(frozen=True)
class TaskExecutionRecord:
    """Immutable execution history for one task."""

    task_id: int
    uav_id: int
    departure_time: float
    arrival_time: float
    start_time: float
    completion_time: float
    travel_time: float


@dataclass(frozen=True)
class ReturnExecutionRecord:
    """A non-preemptible return-to-depot action."""

    uav_id: int
    departure_time: float
    completion_time: float
    travel_time: float


@dataclass
class SimulationState:
    """Forward-only snapshot supplied to a dynamic replanner.

    ``plan`` contains only tasks whose flight leg has not started. A task that
    is already being approached or serviced is removed from the free plan and
    represented by the UAV's future ``anchor``.
    """

    current_time: float
    uavs: dict[int, UAV]
    tasks: dict[int, Task]
    plan: Plan
    execution_states: dict[int, UAVExecutionState]
    completed_task_ids: set[int]
    executing_task_ids: dict[int, int]
    anchors: dict[int, PlanningAnchor]
    locked_task_ids: dict[int, int]
    history: dict[int, TaskExecutionRecord]


@dataclass(frozen=True)
class ReplanningRecord:
    """Metrics for one task-release replanning call."""

    release_time: float
    task_id: int
    runtime_sec: float
    assignment_changes: int
    successor_edge_changes: int

