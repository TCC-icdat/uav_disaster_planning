"""Deterministic EO sweep planning and footprint-driven task release."""

from __future__ import annotations

from math import cos, pi, radians, tan

import numpy as np

from uav_planning.canonical.models import (
    CanonicalScenario,
    EOReleaseEvent,
    EOSweepLeg,
    EOSweepPlan,
    SensorFootprint,
)
from uav_planning.canonical.obstacle import (
    ObstacleAwareDubinsPlanner,
    polyline_polygon_first_intersection,
    polyline_intersects_zones,
)
from uav_planning.geometry.dubins import sample_dubins_path
from uav_planning.models import Pose2D


class EOSweepPlanner:
    """Build a fixed boustrophedon sweep with Dubins lane connectors."""

    def __init__(self, scenario: CanonicalScenario) -> None:
        self.scenario = scenario

    def footprint(self) -> SensorFootprint:
        eo = self.scenario.eo
        return SensorFootprint(
            cross_track_width_m=(
                2.0 * eo.altitude_m * tan(radians(eo.camera_hfov_deg) / 2.0)
            ),
            along_track_length_m=(
                2.0 * eo.altitude_m * tan(radians(eo.camera_vfov_deg) / 2.0)
            ),
        )

    def plan(self) -> EOSweepPlan:
        footprint = self.footprint()
        spacing = footprint.cross_track_width_m * (
            1.0 - self.scenario.eo.cross_track_overlap
        )
        lane_centers: list[float] = []
        center = footprint.cross_track_width_m / 2.0
        while True:
            lane_centers.append(center)
            if center + footprint.cross_track_width_m / 2.0 >= (
                self.scenario.height_m - 1e-9
            ):
                break
            center += spacing

        sample_parts = [
            np.asarray(
                [
                    (
                        self.scenario.depot.x,
                        self.scenario.depot.y,
                        self.scenario.depot.heading,
                    )
                ],
                dtype=float,
            )
        ]
        legs: list[EOSweepLeg] = []
        current = self.scenario.depot
        current_index = 0

        def append_leg(
            leg_id: str,
            leg_type: str,
            start: Pose2D,
            goal: Pose2D,
        ) -> None:
            nonlocal current_index
            sampled = sample_dubins_path(
                start,
                goal,
                self.scenario.eo.min_turn_radius_m,
                step_size=self.scenario.eo.sample_step_m,
            )
            start_index = current_index
            if len(sampled) > 1:
                sample_parts.append(sampled[1:])
                current_index += len(sampled) - 1
            legs.append(
                EOSweepLeg(
                    leg_id=leg_id,
                    leg_type=leg_type,
                    start_index=start_index,
                    end_index=current_index,
                    from_pose=start,
                    to_pose=goal,
                )
            )

        for lane_index, y in enumerate(lane_centers, start=1):
            eastbound = lane_index % 2 == 1
            heading = 0.0 if eastbound else pi
            lane_start = Pose2D(
                0.0 if eastbound else self.scenario.width_m,
                y,
                heading,
            )
            lane_end = Pose2D(
                self.scenario.width_m if eastbound else 0.0,
                y,
                heading,
            )
            append_leg(
                "EO_TRANSIT" if lane_index == 1 else f"EO_TURN_{lane_index - 1:02d}",
                "transit" if lane_index == 1 else "connector",
                current,
                lane_start,
            )
            append_leg(
                f"EO_LANE_{lane_index:02d}",
                "sweep_lane",
                lane_start,
                lane_end,
            )
            current = lane_end

        samples = np.vstack(sample_parts)
        deltas = np.diff(samples[:, :2], axis=0)
        distances = np.hypot(deltas[:, 0], deltas[:, 1])
        cumulative = np.concatenate(([0.0], np.cumsum(distances)))
        return EOSweepPlan(
            samples=samples,
            cumulative_distance_m=cumulative,
            legs=tuple(legs),
            lane_count=len(lane_centers),
            lane_spacing_m=spacing,
            footprint=footprint,
        )


class EOEventGenerator:
    """Generate deterministic SAR releases from first complete EO coverage."""

    def __init__(self, scenario: CanonicalScenario, sweep: EOSweepPlan) -> None:
        self.scenario = scenario
        self.sweep = sweep

    def releases(self) -> tuple[EOReleaseEvent, ...]:
        events: list[EOReleaseEvent] = []
        for aoi in self.scenario.aois:
            if aoi.initially_known:
                events.append(
                    EOReleaseEvent(
                        aoi_id=aoi.aoi_id,
                        task_id=aoi.task_id,
                        semantic_type=aoi.semantic_type,
                        priority=aoi.priority,
                        release_time=0.0,
                        eo_pose_at_detection=None,
                        path_sample_index=None,
                        initially_known=True,
                    )
                )
                continue
            detection_index = next(
                (
                    index
                    for index, row in enumerate(self.sweep.samples)
                    if self.sweep.footprint.fully_contains(
                        Pose2D(float(row[0]), float(row[1]), float(row[2])),
                        aoi.polygon,
                    )
                ),
                None,
            )
            if detection_index is None:
                raise ValueError(f"EO sweep did not fully cover hidden AOI {aoi.aoi_id}")
            row = self.sweep.samples[detection_index]
            events.append(
                EOReleaseEvent(
                    aoi_id=aoi.aoi_id,
                    task_id=aoi.task_id,
                    semantic_type=aoi.semantic_type,
                    priority=aoi.priority,
                    release_time=(
                        float(self.sweep.cumulative_distance_m[detection_index])
                        / self.scenario.eo.speed_mps
                    ),
                    eo_pose_at_detection=Pose2D(
                        float(row[0]), float(row[1]), float(row[2])
                    ),
                    path_sample_index=detection_index,
                    initially_known=False,
                )
            )
        hidden_count = sum(not event.initially_known for event in events)
        if hidden_count != 9:
            raise AssertionError("canonical EO chain must release exactly 9 hidden AOIs")
        return tuple(sorted(events, key=lambda event: (event.release_time, event.task_id)))


class SafeEOSweepPlanner:
    """Preserve the frozen lane layout and detour only colliding EO legs."""

    def __init__(
        self,
        scenario: CanonicalScenario,
        obstacle_planner: ObstacleAwareDubinsPlanner,
    ) -> None:
        self.scenario = scenario
        self.obstacle_planner = obstacle_planner

    def plan(self) -> EOSweepPlan:
        original = EOSweepPlanner(self.scenario).plan()
        sample_parts = [original.samples[:1].copy()]
        legs: list[EOSweepLeg] = []
        detoured: list[str] = []
        current_index = 0
        for leg in original.legs:
            original_samples = original.samples[leg.start_index : leg.end_index + 1]
            if polyline_intersects_zones(
                original_samples, self.scenario.no_fly_zones
            ):
                samples = self._detour_leg(leg, original_samples)
                detoured.append(leg.leg_id)
            else:
                samples = original_samples
            start_index = current_index
            if len(samples) > 1:
                sample_parts.append(samples[1:])
                current_index += len(samples) - 1
            legs.append(
                EOSweepLeg(
                    leg_id=leg.leg_id,
                    leg_type=leg.leg_type,
                    start_index=start_index,
                    end_index=current_index,
                    from_pose=leg.from_pose,
                    to_pose=leg.to_pose,
                )
            )
        samples = np.vstack(sample_parts)
        delta = np.diff(samples[:, :2], axis=0)
        distances = np.hypot(delta[:, 0], delta[:, 1])
        cumulative = np.concatenate(([0.0], np.cumsum(distances)))
        safe = EOSweepPlan(
            samples=samples,
            cumulative_distance_m=cumulative,
            legs=tuple(legs),
            lane_count=original.lane_count,
            lane_spacing_m=original.lane_spacing_m,
            footprint=original.footprint,
            detoured_leg_ids=tuple(detoured),
        )
        if polyline_intersects_zones(safe.samples, self.scenario.no_fly_zones):
            raise RuntimeError("safe EO sweep failed final NFZ verification")
        return safe

    def _detour_leg(
        self, leg: EOSweepLeg, original_samples: np.ndarray
    ) -> np.ndarray:
        colliding_zones = [
            zone
            for zone in self.scenario.no_fly_zones
            if polyline_polygon_first_intersection(
                original_samples, zone.inflated_polygon
            )
            is not None
        ]
        if leg.leg_type != "sweep_lane":
            return self.obstacle_planner.plan(
                leg.from_pose,
                leg.to_pose,
                self.scenario.eo.min_turn_radius_m,
            ).samples

        eastbound = cos(leg.from_pose.heading) > 0.0
        colliding_zones.sort(
            key=lambda zone: min(point[0] for point in zone.inflated_polygon),
            reverse=not eastbound,
        )
        approach = self.scenario.obstacle_planner.eo_detour_approach_m
        current = leg.from_pose
        parts: list[np.ndarray] = []

        def append(samples: np.ndarray) -> None:
            parts.append(samples if not parts else samples[1:])

        for zone in colliding_zones:
            xs = [point[0] for point in zone.inflated_polygon]
            before_x = min(xs) - approach if eastbound else max(xs) + approach
            after_x = max(xs) + approach if eastbound else min(xs) - approach
            before = Pose2D(before_x, leg.from_pose.y, leg.from_pose.heading)
            after = Pose2D(after_x, leg.from_pose.y, leg.from_pose.heading)
            append(
                sample_dubins_path(
                    current,
                    before,
                    self.scenario.eo.min_turn_radius_m,
                    step_size=self.scenario.eo.sample_step_m,
                )
            )
            append(
                self.obstacle_planner.plan(
                    before,
                    after,
                    self.scenario.eo.min_turn_radius_m,
                ).samples
            )
            current = after
        append(
            sample_dubins_path(
                current,
                leg.to_pose,
                self.scenario.eo.min_turn_radius_m,
                step_size=self.scenario.eo.sample_step_m,
            )
        )
        return np.vstack(parts)
