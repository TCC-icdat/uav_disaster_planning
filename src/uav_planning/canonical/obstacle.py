"""Canonical-only static-NFZ path planning for directed fixed-wing edges."""

from __future__ import annotations

from heapq import heappop, heappush
from math import pi

import numpy as np

from uav_planning.canonical.models import (
    CanonicalScenario,
    NoFlyZone,
    ObstacleAwarePath,
)
from uav_planning.geometry.dubins import sample_dubins_path
from uav_planning.models import Pose2D, UAV
from uav_planning.routing.evaluator import TravelTimeProvider

_EPS = 1e-9


def point_in_polygon(
    point: tuple[float, float], polygon: tuple[tuple[float, float], ...]
) -> bool:
    """Return true for points inside or on the boundary of a polygon."""

    x, y = point
    inside = False
    for left, right in zip(polygon, polygon[1:] + polygon[:1]):
        x1, y1 = left
        x2, y2 = right
        cross = (x - x1) * (y2 - y1) - (y - y1) * (x2 - x1)
        if (
            abs(cross) <= _EPS
            and min(x1, x2) - _EPS <= x <= max(x1, x2) + _EPS
            and min(y1, y2) - _EPS <= y <= max(y1, y2) + _EPS
        ):
            return True
        if (y1 > y) != (y2 > y):
            x_cross = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x <= x_cross + _EPS:
                inside = not inside
    return inside


def _cross(a: np.ndarray, b: np.ndarray) -> float:
    return float(a[0] * b[1] - a[1] * b[0])


def _segments_intersect(
    p1: np.ndarray,
    p2: np.ndarray,
    q1: np.ndarray,
    q2: np.ndarray,
) -> tuple[float, float] | None:
    r = p2 - p1
    s = q2 - q1
    denominator = _cross(r, s)
    delta = q1 - p1
    if abs(denominator) <= _EPS:
        return None
    t = _cross(delta, s) / denominator
    u = _cross(delta, r) / denominator
    if -_EPS <= t <= 1.0 + _EPS and -_EPS <= u <= 1.0 + _EPS:
        point = p1 + max(0.0, min(1.0, t)) * r
        return float(point[0]), float(point[1])
    return None


def polyline_polygon_first_intersection(
    samples: np.ndarray,
    polygon: tuple[tuple[float, float], ...],
) -> tuple[float, float] | None:
    """Return the first sampled-polyline intersection, including boundaries."""

    if len(samples) == 0:
        return None
    if point_in_polygon((float(samples[0, 0]), float(samples[0, 1])), polygon):
        return float(samples[0, 0]), float(samples[0, 1])
    polygon_arrays = [np.asarray(point, dtype=float) for point in polygon]
    for first, second in zip(samples[:-1, :2], samples[1:, :2]):
        if point_in_polygon((float(second[0]), float(second[1])), polygon):
            return float(second[0]), float(second[1])
        for left, right in zip(
            polygon_arrays,
            polygon_arrays[1:] + polygon_arrays[:1],
        ):
            point = _segments_intersect(first, second, left, right)
            if point is not None:
                return point
    return None


def polyline_intersects_zones(
    samples: np.ndarray, zones: tuple[NoFlyZone, ...]
) -> bool:
    return any(
        polyline_polygon_first_intersection(samples, zone.inflated_polygon)
        is not None
        for zone in zones
    )


def sampled_path_length(samples: np.ndarray) -> float:
    if len(samples) < 2:
        return 0.0
    delta = np.diff(samples[:, :2], axis=0)
    return float(np.hypot(delta[:, 0], delta[:, 1]).sum())


class ObstacleAwareDubinsPlanner:
    """Small directed pose graph around inflated rectangular NFZ corners.

    Candidate positions are deterministic clearance points outside each
    inflated NFZ.  Edges are true shortest-Dubins samples, not straight-line
    surrogates, and every selected path is collision-checked again after
    concatenation.
    """

    def __init__(self, scenario: CanonicalScenario) -> None:
        self.zones = scenario.no_fly_zones
        config = scenario.obstacle_planner
        self.sample_step_m = config.sample_step_m
        self.waypoint_clearance_m = config.waypoint_clearance_m
        self.waypoint_heading_count = config.waypoint_heading_count
        self._waypoint_poses = self._build_waypoint_poses()
        self._adjacency_by_radius: dict[
            float, tuple[tuple[tuple[int, float], ...], ...]
        ] = {}
        self._outgoing_terminal_cache: dict[
            tuple[float, ...], tuple[tuple[int, float], ...]
        ] = {}
        self._incoming_terminal_cache: dict[
            tuple[float, ...], dict[int, float]
        ] = {}
        self._path_cache: dict[tuple[float, ...], ObstacleAwarePath] = {}

    def _build_waypoint_poses(self) -> tuple[Pose2D, ...]:
        positions: set[tuple[float, float]] = set()
        clearance = self.waypoint_clearance_m
        for zone in self.zones:
            xs = [point[0] for point in zone.inflated_polygon]
            ys = [point[1] for point in zone.inflated_polygon]
            left, right = min(xs) - clearance, max(xs) + clearance
            bottom, top = min(ys) - clearance, max(ys) + clearance
            positions.update(
                {(left, bottom), (right, bottom), (right, top), (left, top)}
            )
        headings = tuple(
            2.0 * pi * index / self.waypoint_heading_count
            for index in range(self.waypoint_heading_count)
        )
        return tuple(
            Pose2D(x, y, heading)
            for x, y in sorted(positions)
            for heading in headings
        )

    @staticmethod
    def _pose_key(pose: Pose2D) -> tuple[float, float, float]:
        return (round(pose.x, 9), round(pose.y, 9), round(pose.heading, 9))

    def _cache_key(
        self, start: Pose2D, goal: Pose2D, turning_radius_m: float
    ) -> tuple[float, ...]:
        return (
            *self._pose_key(start),
            *self._pose_key(goal),
            round(turning_radius_m, 9),
        )

    def _direct_samples(
        self, start: Pose2D, goal: Pose2D, turning_radius_m: float
    ) -> np.ndarray:
        return sample_dubins_path(
            start,
            goal,
            turning_radius_m,
            step_size=self.sample_step_m,
        )

    def _safe_edge_length(
        self, start: Pose2D, goal: Pose2D, turning_radius_m: float
    ) -> float | None:
        samples = self._direct_samples(start, goal, turning_radius_m)
        if polyline_intersects_zones(samples, self.zones):
            return None
        return sampled_path_length(samples)

    def _waypoint_adjacency(
        self, turning_radius_m: float
    ) -> tuple[tuple[tuple[int, float], ...], ...]:
        key = round(turning_radius_m, 9)
        cached = self._adjacency_by_radius.get(key)
        if cached is not None:
            return cached
        rows: list[list[tuple[int, float]]] = [
            [] for _ in self._waypoint_poses
        ]
        for source_index, source in enumerate(self._waypoint_poses):
            for target_index, target in enumerate(self._waypoint_poses):
                if source_index == target_index:
                    continue
                if source.x == target.x and source.y == target.y:
                    continue
                length = self._safe_edge_length(
                    source, target, turning_radius_m
                )
                if length is not None:
                    rows[source_index].append((target_index, length))
        adjacency = tuple(tuple(row) for row in rows)
        self._adjacency_by_radius[key] = adjacency
        return adjacency

    def _outgoing_terminal_edges(
        self, pose: Pose2D, turning_radius_m: float
    ) -> tuple[tuple[int, float], ...]:
        key = (*self._pose_key(pose), round(turning_radius_m, 9))
        cached = self._outgoing_terminal_cache.get(key)
        if cached is not None:
            return cached
        edges = tuple(
            (index, length)
            for index, waypoint in enumerate(self._waypoint_poses)
            if (
                length := self._safe_edge_length(
                    pose, waypoint, turning_radius_m
                )
            )
            is not None
        )
        self._outgoing_terminal_cache[key] = edges
        return edges

    def _incoming_terminal_edges(
        self, pose: Pose2D, turning_radius_m: float
    ) -> dict[int, float]:
        key = (*self._pose_key(pose), round(turning_radius_m, 9))
        cached = self._incoming_terminal_cache.get(key)
        if cached is not None:
            return cached
        edges = {
            index: length
            for index, waypoint in enumerate(self._waypoint_poses)
            if (
                length := self._safe_edge_length(
                    waypoint, pose, turning_radius_m
                )
            )
            is not None
        }
        self._incoming_terminal_cache[key] = edges
        return edges

    def plan(
        self, start: Pose2D, goal: Pose2D, turning_radius_m: float
    ) -> ObstacleAwarePath:
        key = self._cache_key(start, goal, turning_radius_m)
        cached = self._path_cache.get(key)
        if cached is not None:
            return cached
        for label, pose in (("start", start), ("goal", goal)):
            if any(
                point_in_polygon((pose.x, pose.y), zone.inflated_polygon)
                for zone in self.zones
            ):
                raise ValueError(f"{label} pose lies inside an inflated NFZ")

        direct = self._direct_samples(start, goal, turning_radius_m)
        if not polyline_intersects_zones(direct, self.zones):
            result = ObstacleAwarePath(
                samples=direct,
                length_m=sampled_path_length(direct),
                detoured=False,
                waypoint_count=0,
            )
            self._path_cache[key] = result
            return result

        waypoint_count = len(self._waypoint_poses)
        goal_index = waypoint_count + 1
        adjacency = self._waypoint_adjacency(turning_radius_m)
        start_edges = self._outgoing_terminal_edges(start, turning_radius_m)
        goal_lengths = self._incoming_terminal_edges(goal, turning_radius_m)

        distances = [float("inf")] * (waypoint_count + 2)
        previous: list[int | None] = [None] * (waypoint_count + 2)
        start_index = waypoint_count
        distances[start_index] = 0.0
        queue: list[tuple[float, int]] = [(0.0, start_index)]
        while queue:
            distance, node = heappop(queue)
            if distance > distances[node] + _EPS:
                continue
            if node == goal_index:
                break
            if node == start_index:
                edges = start_edges
            else:
                edges = list(adjacency[node])
                if node in goal_lengths:
                    edges.append((goal_index, goal_lengths[node]))
            for neighbor, weight in edges:
                candidate = distance + weight
                if candidate + _EPS < distances[neighbor]:
                    distances[neighbor] = candidate
                    previous[neighbor] = node
                    heappush(queue, (candidate, neighbor))
        if previous[goal_index] is None:
            raise RuntimeError("no collision-free obstacle-aware Dubins path found")

        node_path = [goal_index]
        while node_path[-1] != start_index:
            predecessor = previous[node_path[-1]]
            if predecessor is None:
                raise RuntimeError("broken obstacle-aware predecessor chain")
            node_path.append(predecessor)
        node_path.reverse()
        poses = [start]
        poses.extend(
            self._waypoint_poses[index]
            for index in node_path[1:-1]
        )
        poses.append(goal)

        pieces = []
        for first, second in zip(poses[:-1], poses[1:]):
            samples = self._direct_samples(first, second, turning_radius_m)
            pieces.append(samples if not pieces else samples[1:])
        combined = np.vstack(pieces)
        if polyline_intersects_zones(combined, self.zones):
            raise RuntimeError("selected obstacle-aware path failed final verification")
        result = ObstacleAwarePath(
            samples=combined,
            length_m=sampled_path_length(combined),
            detoured=True,
            waypoint_count=len(poses) - 2,
        )
        self._path_cache[key] = result
        return result


class ObstacleAwareDubinsTravelTimeProvider(TravelTimeProvider):
    """Travel-time adapter that leaves the generic evaluator unchanged."""

    name = "obstacle_aware_dubins"

    def __init__(self, planner: ObstacleAwareDubinsPlanner) -> None:
        self.planner = planner

    def travel_time(self, uav: UAV, from_pose: Pose2D, to_pose: Pose2D) -> float:
        return (
            self.planner.plan(from_pose, to_pose, uav.min_turn_radius).length_m
            / uav.speed
        )
