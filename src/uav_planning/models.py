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


@dataclass
class SimulationState:
    """Read-only snapshot supplied to a dynamic replanner."""

    current_time: float
    uavs: dict[int, UAV]
    tasks: dict[int, Task]
    plan: Plan
    execution_states: dict[int, UAVExecutionState]
    completed_task_ids: set[int]
    executing_task_ids: dict[int, int]
    locked_prefixes: dict[int, list[int]]
    commitment_prefixes: dict[int, list[int]]


@dataclass(frozen=True)
class ReplanningRecord:
    """Metrics for one task-release replanning call."""

    release_time: float
    task_id: int
    runtime_sec: float
    assignment_changes: int
    successor_edge_changes: int

