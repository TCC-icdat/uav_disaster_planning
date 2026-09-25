"""Load and strictly validate the frozen canonical scenario YAML."""

from __future__ import annotations

from pathlib import Path

import yaml

from uav_planning.canonical.models import (
    CanonicalScenario,
    DEGREES_TO_RADIANS,
    EOConfig,
    InitialInformationConfig,
    NoFlyZone,
    ObstaclePlannerConfig,
    SARConfig,
    SemanticAOI,
)
from uav_planning.models import Pose2D


def load_canonical_scenario(path: str | Path) -> CanonicalScenario:
    with Path(path).open("r", encoding="utf-8") as stream:
        raw = yaml.safe_load(stream)

    map_raw = raw["map"]
    depot_raw = map_raw["depot"]
    eo_raw = raw["eo"]
    sar_raw = raw["sar"]
    obstacle_raw = raw["obstacle_planner"]
    information_raw = raw["initial_information"]
    aois = tuple(
        SemanticAOI(
            aoi_id=str(item["id"]),
            task_id=index,
            semantic_type=str(item["semantic_type"]),
            center=tuple(map(float, item["center_m"])),
            length_m=float(item["size_m"][0]),
            width_m=float(item["size_m"][1]),
            priority=float(item["priority"]),
            initially_known=bool(item["initially_known"]),
            orientation_rad=float(item["orientation_deg"]) * DEGREES_TO_RADIANS,
        )
        for index, item in enumerate(raw["aois"], start=1)
    )
    nfz_raw = raw["no_fly_zones"]
    margin = float(nfz_raw["safety_margin_m"])
    zones = tuple(
        NoFlyZone(
            zone_id=str(item["id"]),
            polygon=tuple(tuple(map(float, point)) for point in item["polygon_m"]),
            safety_margin_m=margin,
        )
        for item in nfz_raw["zones"]
    )
    scenario = CanonicalScenario(
        scenario_id=str(raw["scenario_id"]),
        width_m=float(map_raw["width_m"]),
        height_m=float(map_raw["height_m"]),
        depot=Pose2D(
            float(depot_raw["x_m"]),
            float(depot_raw["y_m"]),
            float(depot_raw["heading_deg"]) * DEGREES_TO_RADIANS,
        ),
        eo=EOConfig(
            count=int(eo_raw["count"]),
            speed_mps=float(eo_raw["speed_mps"]),
            min_turn_radius_m=float(eo_raw["min_turn_radius_m"]),
            altitude_m=float(eo_raw["altitude_m"]),
            camera_hfov_deg=float(eo_raw["camera_hfov_deg"]),
            camera_vfov_deg=float(eo_raw["camera_vfov_deg"]),
            cross_track_overlap=float(eo_raw["cross_track_overlap"]),
            sample_step_m=float(eo_raw["sample_step_m"]),
        ),
        sar=SARConfig(
            count=int(sar_raw["count"]),
            speed_mps=float(sar_raw["speed_mps"]),
            min_turn_radius_m=float(sar_raw["min_turn_radius_m"]),
            max_mission_time_s=float(sar_raw["max_mission_time_s"]),
            altitude_m=float(sar_raw["altitude_m"]),
            incidence_angle_deg=float(sar_raw["incidence_angle_deg"]),
            beamwidth_deg=float(sar_raw["beamwidth_deg"]),
            objective_mode=str(sar_raw["objective_mode"]),
            affected_uav_count_h=int(sar_raw["affected_uav_count_h"]),
            commitment_horizon=int(sar_raw["commitment_horizon"]),
            local_search_max_iterations=int(
                sar_raw["local_search_max_iterations"]
            ),
            local_search_time_limit_sec=float(
                sar_raw["local_search_time_limit_sec"]
            ),
        ),
        obstacle_planner=ObstaclePlannerConfig(
            sample_step_m=float(obstacle_raw["sample_step_m"]),
            waypoint_clearance_m=float(obstacle_raw["waypoint_clearance_m"]),
            waypoint_heading_count=int(obstacle_raw["waypoint_heading_count"]),
            eo_detour_approach_m=float(obstacle_raw["eo_detour_approach_m"]),
        ),
        initial_information=InitialInformationConfig(
            initial_aoi_ids=tuple(map(str, information_raw["initial_aoi_ids"])),
            sources=tuple(map(str, information_raw["sources"])),
            satellite_role=str(information_raw["satellite_role"]),
            satellite_simulation_enabled=bool(
                information_raw["satellite_simulation_enabled"]
            ),
        ),
        aois=aois,
        no_fly_zones=zones,
    )
    _validate(scenario)
    return scenario


def _validate(scenario: CanonicalScenario) -> None:
    if scenario.scenario_id != "post_earthquake_v1":
        raise ValueError("unexpected canonical scenario id")
    if (scenario.width_m, scenario.height_m) != (6000.0, 6000.0):
        raise ValueError("canonical map must be 6000 x 6000 m")
    if len(scenario.aois) != 12 or len(scenario.no_fly_zones) != 3:
        raise ValueError("canonical scenario must contain 12 AOIs and 3 NFZs")
    if len({aoi.aoi_id for aoi in scenario.aois}) != len(scenario.aois):
        raise ValueError("AOI identifiers must be unique")
    if sum(aoi.initially_known for aoi in scenario.aois) != 3:
        raise ValueError("canonical scenario must contain exactly 3 initial AOIs")
    if scenario.eo.count != 1 or scenario.sar.count != 3:
        raise ValueError("canonical fleet must contain 1 EO and 3 SAR UAVs")
    if scenario.sar.commitment_horizon != 0:
        raise ValueError("canonical Local comparison requires commitment_horizon=0")
    expected_initial = tuple(
        aoi.aoi_id for aoi in scenario.aois if aoi.initially_known
    )
    if scenario.initial_information.initial_aoi_ids != expected_initial:
        raise ValueError("initial information AOI ids must match initial AOIs")
    if scenario.initial_information.satellite_simulation_enabled:
        raise ValueError("satellite simulation is outside the canonical model")
    obstacle = scenario.obstacle_planner
    if (
        obstacle.sample_step_m <= 0.0
        or obstacle.waypoint_clearance_m <= 0.0
        or obstacle.eo_detour_approach_m <= 0.0
    ):
        raise ValueError("obstacle planner distances must be positive")
    if obstacle.waypoint_heading_count < 4:
        raise ValueError("obstacle planner requires at least four headings")
