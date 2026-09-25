"""Typed data structures for the deterministic canonical case study."""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, pi, sin

import numpy as np

from uav_planning.models import Pose2D


def _rectangle_corners(
    center: tuple[float, float],
    length: float,
    width: float,
    heading: float,
) -> tuple[tuple[float, float], ...]:
    tangent = (cos(heading), sin(heading))
    normal = (-sin(heading), cos(heading))
    half_length = length / 2.0
    half_width = width / 2.0
    return tuple(
        (
            center[0] + along * tangent[0] + cross * normal[0],
            center[1] + along * tangent[1] + cross * normal[1],
        )
        for along, cross in (
            (-half_length, -half_width),
            (half_length, -half_width),
            (half_length, half_width),
            (-half_length, half_width),
        )
    )


@dataclass(frozen=True)
class SemanticAOI:
    """A fixed semantic area of interest represented by a rotated rectangle."""

    aoi_id: str
    task_id: int
    semantic_type: str
    center: tuple[float, float]
    length_m: float
    width_m: float
    priority: float
    initially_known: bool
    orientation_rad: float

    @property
    def polygon(self) -> tuple[tuple[float, float], ...]:
        return _rectangle_corners(
            self.center,
            self.length_m,
            self.width_m,
            self.orientation_rad,
        )


@dataclass(frozen=True)
class SensorFootprint:
    """Rectangular EO footprint dimensions in physical map units."""

    cross_track_width_m: float
    along_track_length_m: float

    def polygon_at(self, pose: Pose2D) -> tuple[tuple[float, float], ...]:
        return _rectangle_corners(
            (pose.x, pose.y),
            self.along_track_length_m,
            self.cross_track_width_m,
            pose.heading,
        )

    def fully_contains(self, pose: Pose2D, polygon) -> bool:
        tangent = (cos(pose.heading), sin(pose.heading))
        normal = (-sin(pose.heading), cos(pose.heading))
        return all(
            abs((x - pose.x) * tangent[0] + (y - pose.y) * tangent[1])
            <= self.along_track_length_m / 2.0 + 1e-9
            and abs((x - pose.x) * normal[0] + (y - pose.y) * normal[1])
            <= self.cross_track_width_m / 2.0 + 1e-9
            for x, y in polygon
        )


@dataclass(frozen=True)
class EOConfig:
    count: int
    speed_mps: float
    min_turn_radius_m: float
    altitude_m: float
    camera_hfov_deg: float
    camera_vfov_deg: float
    cross_track_overlap: float
    sample_step_m: float


@dataclass(frozen=True)
class SARConfig:
    count: int
    speed_mps: float
    min_turn_radius_m: float
    max_mission_time_s: float
    altitude_m: float
    incidence_angle_deg: float
    beamwidth_deg: float
    objective_mode: str
    affected_uav_count_h: int
    commitment_horizon: int
    local_search_max_iterations: int
    local_search_time_limit_sec: float


@dataclass(frozen=True)
class NoFlyZone:
    zone_id: str
    polygon: tuple[tuple[float, float], ...]
    safety_margin_m: float

    @property
    def inflated_polygon(self) -> tuple[tuple[float, float], ...]:
        xs = [point[0] for point in self.polygon]
        ys = [point[1] for point in self.polygon]
        margin = self.safety_margin_m
        return (
            (min(xs) - margin, min(ys) - margin),
            (max(xs) + margin, min(ys) - margin),
            (max(xs) + margin, max(ys) + margin),
            (min(xs) - margin, max(ys) + margin),
        )


@dataclass(frozen=True)
class CanonicalScenario:
    scenario_id: str
    width_m: float
    height_m: float
    depot: Pose2D
    eo: EOConfig
    sar: SARConfig
    aois: tuple[SemanticAOI, ...]
    no_fly_zones: tuple[NoFlyZone, ...]


@dataclass(frozen=True)
class EOSweepLeg:
    leg_id: str
    leg_type: str
    start_index: int
    end_index: int
    from_pose: Pose2D
    to_pose: Pose2D


@dataclass(frozen=True)
class EOSweepPlan:
    samples: np.ndarray
    cumulative_distance_m: np.ndarray
    legs: tuple[EOSweepLeg, ...]
    lane_count: int
    lane_spacing_m: float
    footprint: SensorFootprint

    @property
    def path_length_m(self) -> float:
        return float(self.cumulative_distance_m[-1])


@dataclass(frozen=True)
class EOReleaseEvent:
    aoi_id: str
    task_id: int
    semantic_type: str
    priority: float
    release_time: float
    eo_pose_at_detection: Pose2D | None
    path_sample_index: int | None
    initially_known: bool


@dataclass(frozen=True)
class SARServiceGeometry:
    aoi_id: str
    task_id: int
    entry_pose: Pose2D
    exit_pose: Pose2D
    scan_length_m: float
    service_time_s: float
    scan_heading_rad: float
    swath_width_m: float
    nominal_standoff_m: float
    inner_ground_range_m: float
    outer_ground_range_m: float
    swath_polygon: tuple[tuple[float, float], ...]
    aoi_fits: bool


@dataclass(frozen=True)
class NFZIntersection:
    vehicle_type: str
    vehicle_id: str
    edge_id: str
    edge_type: str
    from_node: str
    to_node: str
    nfz_id: str
    safety_margin_m: float
    first_intersection_x: float
    first_intersection_y: float


@dataclass(frozen=True)
class CanonicalRouteEdge:
    vehicle_type: str
    vehicle_id: str
    edge_id: str
    edge_type: str
    from_node: str
    to_node: str
    samples: np.ndarray


DEGREES_TO_RADIANS = pi / 180.0
