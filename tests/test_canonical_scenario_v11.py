from __future__ import annotations

import hashlib
from pathlib import Path

import numpy as np
import pytest

from uav_planning.canonical.eo import (
    EOEventGenerator,
    EOSweepPlanner,
    SafeEOSweepPlanner,
)
from uav_planning.canonical.loader import load_canonical_scenario
from uav_planning.canonical.nfz import (
    diagnose_nfz_intersections,
    eo_route_edges,
    sar_route_edges,
)
from uav_planning.canonical.obstacle import (
    ObstacleAwareDubinsPlanner,
    ObstacleAwareDubinsTravelTimeProvider,
    polyline_intersects_zones,
)
from uav_planning.canonical.sar import (
    CanonicalRouteEvaluator,
    CanonicalTaskAdapter,
    build_sar_service_geometries,
    build_sar_service_mode_candidates,
)
from uav_planning.models import Pose2D
from uav_planning.planners import InitialPlanner, LocalReplanner
from uav_planning.routing.insertion import RouteOptimizer
from uav_planning.simulation.simulator import Simulator


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "configs" / "canonical" / "post_earthquake_v1.yaml"


@pytest.fixture(scope="module")
def safe_canonical_data():
    scenario = load_canonical_scenario(CONFIG)
    planner = ObstacleAwareDubinsPlanner(scenario)
    original_sweep = EOSweepPlanner(scenario).plan()
    safe_sweep = SafeEOSweepPlanner(scenario, planner).plan()
    original_releases = EOEventGenerator(scenario, original_sweep).releases()
    safe_releases = EOEventGenerator(scenario, safe_sweep).releases()
    candidates = build_sar_service_mode_candidates(scenario)
    services = build_sar_service_geometries(scenario)
    adapter = CanonicalTaskAdapter(scenario, services, safe_releases)
    planning_scenario = adapter.planning_scenario()
    evaluator = CanonicalRouteEvaluator(
        ObstacleAwareDubinsTravelTimeProvider(planner), services
    )
    optimizer = RouteOptimizer(
        evaluator,
        scenario.sar.local_search_max_iterations,
        scenario.sar.local_search_time_limit_sec,
        scenario.sar.objective_mode,
    )
    result = Simulator(
        planning_scenario,
        InitialPlanner(optimizer),
        optimizer,
        execution_evaluator=evaluator,
    ).run(LocalReplanner(optimizer, h=scenario.sar.affected_uav_count_h))
    uavs = {uav.uav_id: uav for uav in planning_scenario.uavs}
    tasks = {task.task_id: task for task in planning_scenario.all_tasks}
    sar_edges = sar_route_edges(
        result,
        uavs,
        tasks,
        services,
        obstacle_planner=planner,
    )
    return {
        "scenario": scenario,
        "planner": planner,
        "original_sweep": original_sweep,
        "safe_sweep": safe_sweep,
        "original_releases": original_releases,
        "safe_releases": safe_releases,
        "candidates": candidates,
        "services": services,
        "result": result,
        "sar_edges": sar_edges,
    }


def _scan_samples(service) -> np.ndarray:
    return np.asarray(
        [
            (service.entry_pose.x, service.entry_pose.y),
            (service.exit_pose.x, service.exit_pose.y),
        ],
        dtype=float,
    )


def test_selected_sar_service_modes_are_nfz_safe(safe_canonical_data) -> None:
    scenario = safe_canonical_data["scenario"]
    services = safe_canonical_data["services"]
    switched = {
        service.aoi_id
        for service in services.values()
        if service.selected_service_mode == "B"
    }
    assert switched == {"A07", "A09", "A10", "A12"}
    assert all(
        not polyline_intersects_zones(_scan_samples(service), scenario.no_fly_zones)
        for service in services.values()
    )


def test_mirrored_left_looking_mode_preserves_aoi_coverage(
    safe_canonical_data,
) -> None:
    candidates = safe_canonical_data["candidates"]
    for mode_a, mode_b in candidates.values():
        assert mode_a.aoi_fits
        assert mode_b.aoi_fits
        assert mode_b.scan_heading_rad == pytest.approx(
            (mode_a.scan_heading_rad + np.pi) % (2.0 * np.pi)
        )


def test_safe_eo_sweep_has_zero_nfz_intersections(safe_canonical_data) -> None:
    scenario = safe_canonical_data["scenario"]
    safe_sweep = safe_canonical_data["safe_sweep"]
    assert not diagnose_nfz_intersections(
        eo_route_edges(safe_sweep), scenario.no_fly_zones
    )
    assert set(safe_sweep.detoured_leg_ids) == {
        "EO_LANE_02",
        "EO_LANE_03",
        "EO_LANE_04",
        "EO_LANE_07",
        "EO_LANE_08",
    }


def test_all_hidden_aois_detected_after_eo_detour(safe_canonical_data) -> None:
    scenario = safe_canonical_data["scenario"]
    releases = safe_canonical_data["safe_releases"]
    expected = {aoi.aoi_id for aoi in scenario.aois if not aoi.initially_known}
    detected = {
        event.aoi_id
        for event in releases
        if not event.initially_known and event.path_sample_index is not None
    }
    assert detected == expected
    assert len(detected) == 9


def test_release_times_recomputed_from_safe_eo_path(safe_canonical_data) -> None:
    scenario = safe_canonical_data["scenario"]
    sweep = safe_canonical_data["safe_sweep"]
    releases = safe_canonical_data["safe_releases"]
    original = {
        event.task_id: event
        for event in safe_canonical_data["original_releases"]
    }
    changed = 0
    for event in releases:
        if event.initially_known:
            assert event.release_time == 0.0
            continue
        assert event.path_sample_index is not None
        assert event.release_time == pytest.approx(
            sweep.cumulative_distance_m[event.path_sample_index]
            / scenario.eo.speed_mps
        )
        changed += abs(event.release_time - original[event.task_id].release_time) > 1e-6
    assert changed >= 1


def test_obstacle_aware_dubins_path_is_collision_free(
    safe_canonical_data,
) -> None:
    scenario = safe_canonical_data["scenario"]
    planner = safe_canonical_data["planner"]
    path = planner.plan(
        Pose2D(1800.0, 3900.0, 0.0),
        Pose2D(3600.0, 3900.0, 0.0),
        scenario.sar.min_turn_radius_m,
    )
    assert path.detoured
    assert path.waypoint_count >= 1
    assert not polyline_intersects_zones(path.samples, scenario.no_fly_zones)


def test_canonical_safe_run_has_zero_sar_nfz_intersections(
    safe_canonical_data,
) -> None:
    scenario = safe_canonical_data["scenario"]
    assert not diagnose_nfz_intersections(
        safe_canonical_data["sar_edges"], scenario.no_fly_zones
    )


def test_canonical_safe_run_completes_all_aois(safe_canonical_data) -> None:
    scenario = safe_canonical_data["scenario"]
    result = safe_canonical_data["result"]
    assert len(result.history) == len(scenario.aois) == 12
    assert result.metrics["feasible"] is True
    assert result.metrics["time_consistency_check"] is True
    assert result.metrics["makespan"] <= scenario.sar.max_mission_time_s


def test_legacy_formal_results_hash_unchanged() -> None:
    expected = {
        "E1": "90ce8f9469d0c47d1b536b54b169c9f9b98d20874b89891780748a950aa776e0",
        "E2": "75cd5d083f763064bdbcf9aa59875cf22121dbcb8be7dba3aafe11e15e0c8da8",
        "E3": "3ffddb04e403558ab7131c000eff6526ea033b9e88cfbe051b0cc440d7bbf59c",
        "E4": "4140284713cba758ab7003f95710f8dcf0a6f76a6410ce4b443e4d40e9adc58d",
    }
    for experiment, digest in expected.items():
        raw = ROOT / "results" / "formal" / experiment / "raw.csv"
        assert hashlib.sha256(raw.read_bytes()).hexdigest() == digest
