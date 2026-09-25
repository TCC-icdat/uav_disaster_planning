"""Canonical SAR service geometry and adapters for the legacy task planner."""

from __future__ import annotations

from math import cos, radians, sin, tan

from uav_planning.canonical.models import (
    CanonicalScenario,
    EOReleaseEvent,
    SARServiceGeometry,
)
from uav_planning.models import Pose2D, Task, UAV
from uav_planning.routing.evaluator import RouteEvaluator, TravelTimeProvider
from uav_planning.scenario.generator import Scenario


def _sar_ranges(scenario: CanonicalScenario) -> tuple[float, float, float, float]:
    sar = scenario.sar
    incidence = radians(sar.incidence_angle_deg)
    half_beam = radians(sar.beamwidth_deg) / 2.0
    inner = sar.altitude_m * tan(incidence - half_beam)
    outer = sar.altitude_m * tan(incidence + half_beam)
    nominal = sar.altitude_m * tan(incidence)
    return inner, outer, outer - inner, nominal


def build_sar_service_geometries(
    scenario: CanonicalScenario,
) -> dict[int, SARServiceGeometry]:
    inner, outer, swath_width, nominal = _sar_ranges(scenario)
    geometries: dict[int, SARServiceGeometry] = {}
    for aoi in scenario.aois:
        heading = aoi.orientation_rad
        tangent = (cos(heading), sin(heading))
        normal = (-sin(heading), cos(heading))
        scan_length = max(aoi.length_m + 300.0, 600.0)
        line_center = (
            aoi.center[0] - nominal * normal[0],
            aoi.center[1] - nominal * normal[1],
        )
        entry = Pose2D(
            line_center[0] - scan_length / 2.0 * tangent[0],
            line_center[1] - scan_length / 2.0 * tangent[1],
            heading,
        )
        exit_pose = Pose2D(
            line_center[0] + scan_length / 2.0 * tangent[0],
            line_center[1] + scan_length / 2.0 * tangent[1],
            heading,
        )
        swath_polygon = (
            (entry.x + inner * normal[0], entry.y + inner * normal[1]),
            (exit_pose.x + inner * normal[0], exit_pose.y + inner * normal[1]),
            (exit_pose.x + outer * normal[0], exit_pose.y + outer * normal[1]),
            (entry.x + outer * normal[0], entry.y + outer * normal[1]),
        )
        fits = all(
            -1e-9
            <= (x - entry.x) * tangent[0] + (y - entry.y) * tangent[1]
            <= scan_length + 1e-9
            and inner - 1e-9
            <= (x - entry.x) * normal[0] + (y - entry.y) * normal[1]
            <= outer + 1e-9
            for x, y in aoi.polygon
        )
        geometry = SARServiceGeometry(
            aoi_id=aoi.aoi_id,
            task_id=aoi.task_id,
            entry_pose=entry,
            exit_pose=exit_pose,
            scan_length_m=scan_length,
            service_time_s=scan_length / scenario.sar.speed_mps,
            scan_heading_rad=heading,
            swath_width_m=swath_width,
            nominal_standoff_m=nominal,
            inner_ground_range_m=inner,
            outer_ground_range_m=outer,
            swath_polygon=swath_polygon,
            aoi_fits=fits,
        )
        if not fits:
            raise ValueError(
                f"AOI {aoi.aoi_id} does not fit its frozen single SAR swath"
            )
        geometries[aoi.task_id] = geometry
    return geometries


class CanonicalRouteEvaluator(RouteEvaluator):
    """Legacy-compatible evaluator whose service completes at scan exit."""

    def __init__(
        self,
        travel_time_provider: TravelTimeProvider,
        services: dict[int, SARServiceGeometry],
    ) -> None:
        super().__init__(travel_time_provider)
        self.services = dict(services)

    def task_completion_pose(self, task: Task) -> Pose2D:
        return self.services[task.task_id].exit_pose


class CanonicalTaskAdapter:
    """Convert semantic AOIs and EO releases into legacy planner Tasks."""

    def __init__(
        self,
        scenario: CanonicalScenario,
        services: dict[int, SARServiceGeometry],
        releases: tuple[EOReleaseEvent, ...],
    ) -> None:
        self.scenario = scenario
        self.services = services
        self.release_by_task = {event.task_id: event for event in releases}

    def tasks(self) -> tuple[Task, ...]:
        tasks = []
        for aoi in self.scenario.aois:
            service = self.services[aoi.task_id]
            release = self.release_by_task[aoi.task_id]
            tasks.append(
                Task(
                    task_id=aoi.task_id,
                    x=service.entry_pose.x,
                    y=service.entry_pose.y,
                    release_time=release.release_time,
                    priority=aoi.priority,
                    service_time=service.service_time_s,
                    required_heading=service.entry_pose.heading,
                    source="INITIAL" if aoi.initially_known else "EO",
                )
            )
        return tuple(tasks)

    def planning_scenario(self) -> Scenario:
        tasks = self.tasks()
        uavs = tuple(
            UAV(
                uav_id=index + 1,
                speed=self.scenario.sar.speed_mps,
                min_turn_radius=self.scenario.sar.min_turn_radius_m,
                max_mission_time=self.scenario.sar.max_mission_time_s,
                start_pose=self.scenario.depot,
            )
            for index in range(self.scenario.sar.count)
        )
        initial = tuple(task for task in tasks if task.release_time <= 1e-9)
        dynamic = tuple(
            sorted(
                (task for task in tasks if task.release_time > 1e-9),
                key=lambda task: (task.release_time, task.task_id),
            )
        )
        return Scenario(
            seed=0,
            width=self.scenario.width_m,
            height=self.scenario.height_m,
            uavs=uavs,
            initial_tasks=initial,
            dynamic_tasks=dynamic,
        )
