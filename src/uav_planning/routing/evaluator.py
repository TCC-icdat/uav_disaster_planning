"""Single authoritative travel-time and route-schedule evaluation layer."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from math import hypot

from uav_planning.geometry.dubins import dubins_shortest_path_length
from uav_planning.models import Pose2D, Task, UAV


class TravelTimeProvider(ABC):
    """Strategy interface for travel time between two directed poses."""

    name: str

    @abstractmethod
    def travel_time(self, uav: UAV, from_pose: Pose2D, to_pose: Pose2D) -> float:
        """Return flight time for one route leg."""


class EuclideanTravelTimeProvider(TravelTimeProvider):
    """Straight-line diagnostic baseline."""

    name = "euclidean"

    def travel_time(self, uav: UAV, from_pose: Pose2D, to_pose: Pose2D) -> float:
        return hypot(to_pose.x - from_pose.x, to_pose.y - from_pose.y) / uav.speed


class DubinsTravelTimeProvider(TravelTimeProvider):
    """Forward-only fixed-wing travel model."""

    name = "dubins"

    def travel_time(self, uav: UAV, from_pose: Pose2D, to_pose: Pose2D) -> float:
        return (
            dubins_shortest_path_length(from_pose, to_pose, uav.min_turn_radius)
            / uav.speed
        )


@dataclass(frozen=True)
class RouteEvaluation:
    """Deterministic schedule and aggregate costs for one route."""

    feasible: bool
    departure_times: dict[int, float] = field(default_factory=dict)
    arrival_times: dict[int, float] = field(default_factory=dict)
    start_times: dict[int, float] = field(default_factory=dict)
    completion_times: dict[int, float] = field(default_factory=dict)
    leg_travel_times: dict[int, float] = field(default_factory=dict)
    total_travel_time: float = 0.0
    total_service_time: float = 0.0
    return_time: float = 0.0


class RouteEvaluator:
    """Compute task timing, depot return, and mission feasibility."""

    def __init__(self, travel_time_provider: TravelTimeProvider) -> None:
        self.travel_time_provider = travel_time_provider

    def task_completion_pose(self, task: Task) -> Pose2D:
        """Return the pose occupied after service; point tasks stay in place."""

        return task.pose

    def evaluate_route(
        self,
        uav: UAV,
        task_sequence: list[Task],
        start_time: float = 0.0,
        start_pose: Pose2D | None = None,
    ) -> RouteEvaluation:
        """Evaluate a sequence while respecting every task release time."""

        pose = start_pose or uav.start_pose
        time = start_time
        departure_times: dict[int, float] = {}
        arrival_times: dict[int, float] = {}
        start_times: dict[int, float] = {}
        completion_times: dict[int, float] = {}
        leg_travel_times: dict[int, float] = {}
        total_travel = 0.0
        total_service = 0.0

        for task in task_sequence:
            departure_times[task.task_id] = time
            travel = self.travel_time_provider.travel_time(uav, pose, task.pose)
            arrival = time + travel
            service_start = max(arrival, task.release_time)
            completion = service_start + task.service_time
            arrival_times[task.task_id] = arrival
            start_times[task.task_id] = service_start
            completion_times[task.task_id] = completion
            leg_travel_times[task.task_id] = travel
            total_travel += travel
            total_service += task.service_time
            time = completion
            pose = self.task_completion_pose(task)

        return_travel = self.travel_time_provider.travel_time(uav, pose, uav.start_pose)
        total_travel += return_travel
        return_time = time + return_travel
        # H_k is an absolute mission clock, not a fresh budget at every replan.
        feasible = return_time <= uav.max_mission_time + 1e-9
        return RouteEvaluation(
            feasible=feasible,
            departure_times=departure_times,
            arrival_times=arrival_times,
            start_times=start_times,
            completion_times=completion_times,
            leg_travel_times=leg_travel_times,
            total_travel_time=total_travel,
            total_service_time=total_service,
            return_time=return_time,
        )

