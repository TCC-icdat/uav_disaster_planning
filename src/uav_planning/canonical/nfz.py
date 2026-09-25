"""Sampling-based NFZ diagnostics; no obstacle-aware rerouting is performed."""

from __future__ import annotations

from math import ceil

import numpy as np

from uav_planning.canonical.models import (
    CanonicalRouteEdge,
    EOSweepPlan,
    NFZIntersection,
    NoFlyZone,
    SARServiceGeometry,
)
from uav_planning.geometry.dubins import sample_dubins_path
from uav_planning.models import Pose2D, Task, UAV
from uav_planning.simulation.simulator import SimulationResult

_EPS = 1e-9


def _point_in_polygon(
    point: tuple[float, float], polygon: tuple[tuple[float, float], ...]
) -> bool:
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


def _segment_intersection_point(
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
    if len(samples) == 0:
        return None
    if _point_in_polygon((float(samples[0, 0]), float(samples[0, 1])), polygon):
        return float(samples[0, 0]), float(samples[0, 1])
    polygon_arrays = [np.asarray(point, dtype=float) for point in polygon]
    for first, second in zip(samples[:-1, :2], samples[1:, :2]):
        if _point_in_polygon((float(second[0]), float(second[1])), polygon):
            return float(second[0]), float(second[1])
        for left, right in zip(
            polygon_arrays,
            polygon_arrays[1:] + polygon_arrays[:1],
        ):
            intersection = _segment_intersection_point(first, second, left, right)
            if intersection is not None:
                return intersection
    return None


def does_dubins_path_intersect_nfz(
    start: Pose2D,
    goal: Pose2D,
    turning_radius_m: float,
    zone: NoFlyZone,
    step_size_m: float = 20.0,
) -> bool:
    samples = sample_dubins_path(
        start, goal, turning_radius_m, step_size=step_size_m
    )
    return (
        polyline_polygon_first_intersection(samples, zone.inflated_polygon)
        is not None
    )


def eo_route_edges(sweep: EOSweepPlan) -> tuple[CanonicalRouteEdge, ...]:
    return tuple(
        CanonicalRouteEdge(
            vehicle_type="EO",
            vehicle_id="EO1",
            edge_id=leg.leg_id,
            edge_type=leg.leg_type,
            from_node=leg.leg_id + "_START",
            to_node=leg.leg_id + "_END",
            samples=sweep.samples[leg.start_index : leg.end_index + 1],
        )
        for leg in sweep.legs
    )


def _sample_straight(
    start: Pose2D, goal: Pose2D, step_size_m: float
) -> np.ndarray:
    distance = float(np.hypot(goal.x - start.x, goal.y - start.y))
    steps = max(1, int(ceil(distance / step_size_m)))
    fractions = np.linspace(0.0, 1.0, steps + 1)
    return np.column_stack(
        (
            start.x + fractions * (goal.x - start.x),
            start.y + fractions * (goal.y - start.y),
            np.full_like(fractions, start.heading),
        )
    )


def sar_route_edges(
    result: SimulationResult,
    uavs: dict[int, UAV],
    tasks: dict[int, Task],
    services: dict[int, SARServiceGeometry],
    step_size_m: float = 20.0,
) -> tuple[CanonicalRouteEdge, ...]:
    edges: list[CanonicalRouteEdge] = []
    for uav_id in sorted(uavs):
        uav = uavs[uav_id]
        current_pose = uav.start_pose
        current_node = "Depot"
        previous_completion: float | None = None
        idle_return_count = 0
        records = sorted(
            (record for record in result.history.values() if record.uav_id == uav_id),
            key=lambda record: record.departure_time,
        )
        for sequence, record in enumerate(records, start=1):
            # The simulator starts a non-preemptible depot return whenever a
            # UAV has no free task.  A later departure gap therefore means the
            # next transfer starts at the depot, not at the previous scan exit.
            if (
                previous_completion is not None
                and record.departure_time > previous_completion + _EPS
            ):
                idle_return_count += 1
                edges.append(
                    CanonicalRouteEdge(
                        vehicle_type="SAR",
                        vehicle_id=f"SAR{uav_id}",
                        edge_id=(
                            f"SAR{uav_id}_IDLE_RETURN_{idle_return_count:02d}"
                        ),
                        edge_type="dubins_return",
                        from_node=current_node,
                        to_node="Depot",
                        samples=sample_dubins_path(
                            current_pose,
                            uav.start_pose,
                            uav.min_turn_radius,
                            step_size=step_size_m,
                        ),
                    )
                )
                current_pose = uav.start_pose
                current_node = "Depot"
            task = tasks[record.task_id]
            service = services[record.task_id]
            aoi_id = service.aoi_id
            transfer_id = f"SAR{uav_id}_TRANSFER_{sequence:02d}"
            edges.append(
                CanonicalRouteEdge(
                    vehicle_type="SAR",
                    vehicle_id=f"SAR{uav_id}",
                    edge_id=transfer_id,
                    edge_type="dubins_transfer",
                    from_node=current_node,
                    to_node=aoi_id + "_ENTRY",
                    samples=sample_dubins_path(
                        current_pose,
                        task.pose,
                        uav.min_turn_radius,
                        step_size=step_size_m,
                    ),
                )
            )
            edges.append(
                CanonicalRouteEdge(
                    vehicle_type="SAR",
                    vehicle_id=f"SAR{uav_id}",
                    edge_id=f"SAR{uav_id}_SCAN_{aoi_id}",
                    edge_type="sar_scan_service",
                    from_node=aoi_id + "_ENTRY",
                    to_node=aoi_id + "_EXIT",
                    samples=_sample_straight(
                        service.entry_pose, service.exit_pose, step_size_m
                    ),
                )
            )
            current_pose = service.exit_pose
            current_node = aoi_id + "_EXIT"
            previous_completion = record.completion_time
        edges.append(
            CanonicalRouteEdge(
                vehicle_type="SAR",
                vehicle_id=f"SAR{uav_id}",
                edge_id=f"SAR{uav_id}_RETURN",
                edge_type="dubins_return",
                from_node=current_node,
                to_node="Depot",
                samples=sample_dubins_path(
                    current_pose,
                    uav.start_pose,
                    uav.min_turn_radius,
                    step_size=step_size_m,
                ),
            )
        )
    return tuple(edges)


def diagnose_nfz_intersections(
    edges: tuple[CanonicalRouteEdge, ...],
    zones: tuple[NoFlyZone, ...],
) -> tuple[NFZIntersection, ...]:
    intersections: list[NFZIntersection] = []
    for edge in edges:
        for zone in zones:
            point = polyline_polygon_first_intersection(
                edge.samples, zone.inflated_polygon
            )
            if point is None:
                continue
            intersections.append(
                NFZIntersection(
                    vehicle_type=edge.vehicle_type,
                    vehicle_id=edge.vehicle_id,
                    edge_id=edge.edge_id,
                    edge_type=edge.edge_type,
                    from_node=edge.from_node,
                    to_node=edge.to_node,
                    nfz_id=zone.zone_id,
                    safety_margin_m=zone.safety_margin_m,
                    first_intersection_x=point[0],
                    first_intersection_y=point[1],
                )
            )
    return tuple(intersections)
