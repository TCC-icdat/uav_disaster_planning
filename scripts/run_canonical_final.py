"""Finalize the canonical case with static-NFZ-safe EO and SAR flight."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from dataclasses import asdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from uav_planning.canonical.eo import (
    EOEventGenerator,
    EOSweepPlanner,
    SafeEOSweepPlanner,
)
from uav_planning.canonical.final_visualization import (
    plot_canonical_replanning_snapshot,
    plot_canonical_safe_routes,
    plot_canonical_scene_final,
    plot_eo_safe_sweep_and_detection,
    plot_sar_service_modes_example,
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
)
from uav_planning.canonical.sar import (
    CanonicalRouteEvaluator,
    CanonicalTaskAdapter,
    build_sar_service_geometries,
    build_sar_service_mode_candidates,
)
from uav_planning.planners import InitialPlanner, LocalReplanner
from uav_planning.routing.insertion import RouteOptimizer
from uav_planning.simulation.simulator import Simulator


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config", default="configs/canonical/post_earthquake_v1.yaml"
    )
    return parser.parse_args()


def _resolve(path: str) -> Path:
    candidate = Path(path)
    return candidate if candidate.is_absolute() else ROOT / candidate


def _write_csv(path: Path, rows: list[dict[str, object]], fields: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _release_rows(scenario, releases, original_releases) -> list[dict[str, object]]:
    original_by_task = {event.task_id: event for event in original_releases}
    aoi_by_task = {aoi.task_id: aoi for aoi in scenario.aois}
    rows = []
    for event in sorted(releases, key=lambda item: item.task_id):
        original = original_by_task[event.task_id]
        aoi = aoi_by_task[event.task_id]
        pose = event.eo_pose_at_detection
        rows.append(
            {
                "aoi_id": event.aoi_id,
                "task_id": event.task_id,
                "semantic_type": event.semantic_type,
                "priority": event.priority,
                "initially_known": event.initially_known,
                "initial_information_source": (
                    "multi_source_prior" if event.initially_known else "eo_safe_sweep"
                ),
                "release_time_before_s": original.release_time,
                "release_time_after_s": event.release_time,
                "release_time_change_s": event.release_time - original.release_time,
                "eo_path_sample_index": event.path_sample_index,
                "eo_x_m": None if pose is None else pose.x,
                "eo_y_m": None if pose is None else pose.y,
                "eo_heading_rad": None if pose is None else pose.heading,
                "center_x_m": aoi.center[0],
                "center_y_m": aoi.center[1],
            }
        )
    return rows


def _service_rows(scenario, services) -> list[dict[str, object]]:
    aoi_by_task = {aoi.task_id: aoi for aoi in scenario.aois}
    rows = []
    for task_id, service in sorted(services.items()):
        aoi = aoi_by_task[task_id]
        rows.append(
            {
                "aoi_id": service.aoi_id,
                "task_id": task_id,
                "semantic_type": aoi.semantic_type,
                "selected_service_mode": service.selected_service_mode,
                "default_mode_collision": service.default_mode_collision,
                "mirror_mode_collision": service.mirror_mode_collision,
                "scan_heading_rad": service.scan_heading_rad,
                "entry_x_m": service.entry_pose.x,
                "entry_y_m": service.entry_pose.y,
                "exit_x_m": service.exit_pose.x,
                "exit_y_m": service.exit_pose.y,
                "scan_length_m": service.scan_length_m,
                "service_time_s": service.service_time_s,
                "swath_width_m": service.swath_width_m,
                "nominal_standoff_m": service.nominal_standoff_m,
                "aoi_fits_single_swath": service.aoi_fits,
                "selected_scan_nfz_safe": True,
            }
        )
    return rows


def _write_timeline(path: Path, scenario, releases, result) -> None:
    aoi_by_task = {aoi.task_id: aoi for aoi in scenario.aois}
    events: list[tuple[float, int, str]] = [
        (
            0.0,
            0,
            "A01, A05 and A12 released from the multi-source prior "
            "(satellite remote sensing, historical geographic information and ground emergency reports)",
        )
    ]
    for event in releases:
        if not event.initially_known:
            events.extend(
                [
                    (
                        event.release_time,
                        1,
                        f"Safe EO sweep fully covers {event.aoi_id}; SAR task released",
                    ),
                    (
                        event.release_time,
                        2,
                        f"Local replanning triggered by {event.aoi_id} (h={scenario.sar.affected_uav_count_h})",
                    ),
                ]
            )
    for record in result.history.values():
        aoi = aoi_by_task[record.task_id]
        events.append(
            (
                record.completion_time,
                3,
                f"SAR{record.uav_id} completes {aoi.aoi_id} ({aoi.semantic_type})",
            )
        )
    events.append(
        (
            float(result.metrics["makespan"]),
            4,
            "Mission complete; all SAR UAVs returned to depot",
        )
    )
    lines = [
        "# Collision-Free Canonical Scenario Timeline",
        "",
        "| Time (s) | Event |",
        "|---:|---|",
        *(
            f"| {time_value:.3f} | {description} |"
            for time_value, _, description in sorted(events)
        ),
        "",
        "> Modeling assumption: when no task is pending, a SAR UAV returns to the depot/base state. "
        "A later task may be dispatched from the depot; landing, turnaround and relaunch time are not modeled.",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    args = parse_args()
    scenario = load_canonical_scenario(_resolve(args.config))
    original_sweep = EOSweepPlanner(scenario).plan()
    original_releases = EOEventGenerator(scenario, original_sweep).releases()
    obstacle_planner = ObstacleAwareDubinsPlanner(scenario)
    safe_sweep = SafeEOSweepPlanner(scenario, obstacle_planner).plan()
    releases = EOEventGenerator(scenario, safe_sweep).releases()
    candidates = build_sar_service_mode_candidates(scenario)
    services = build_sar_service_geometries(scenario)

    adapter = CanonicalTaskAdapter(scenario, services, releases)
    planning_scenario = adapter.planning_scenario()
    provider = ObstacleAwareDubinsTravelTimeProvider(obstacle_planner)
    evaluator = CanonicalRouteEvaluator(provider, services)
    optimizer = RouteOptimizer(
        evaluator,
        local_search_max_iterations=scenario.sar.local_search_max_iterations,
        local_search_time_limit_sec=scenario.sar.local_search_time_limit_sec,
        objective_mode=scenario.sar.objective_mode,
    )
    result = Simulator(
        planning_scenario,
        InitialPlanner(optimizer),
        optimizer,
        execution_evaluator=evaluator,
    ).run(LocalReplanner(optimizer, h=scenario.sar.affected_uav_count_h))

    uavs = {uav.uav_id: uav for uav in planning_scenario.uavs}
    tasks = {task.task_id: task for task in planning_scenario.all_tasks}
    original_eo_intersections = diagnose_nfz_intersections(
        eo_route_edges(original_sweep), scenario.no_fly_zones
    )
    safe_eo_intersections = diagnose_nfz_intersections(
        eo_route_edges(safe_sweep), scenario.no_fly_zones
    )
    safe_sar_edges = sar_route_edges(
        result,
        uavs,
        tasks,
        services,
        obstacle_planner=obstacle_planner,
    )
    safe_sar_intersections = diagnose_nfz_intersections(
        safe_sar_edges, scenario.no_fly_zones
    )
    if safe_eo_intersections or safe_sar_intersections:
        raise RuntimeError("collision-free canonical verification failed")
    if not bool(result.metrics["feasible"]):
        raise RuntimeError("collision-free canonical run is infeasible")
    if not bool(result.metrics["time_consistency_check"]):
        raise RuntimeError("collision-free canonical run is time-inconsistent")
    if len(result.history) != len(scenario.aois):
        raise RuntimeError("collision-free canonical run is incomplete")
    if float(result.metrics["makespan"]) > scenario.sar.max_mission_time_s:
        raise RuntimeError("collision-free canonical run exceeds endurance")

    final_dir = ROOT / "results" / "canonical" / "final"
    final_dir.mkdir(parents=True, exist_ok=True)
    release_fields = [
        "aoi_id",
        "task_id",
        "semantic_type",
        "priority",
        "initially_known",
        "initial_information_source",
        "release_time_before_s",
        "release_time_after_s",
        "release_time_change_s",
        "eo_path_sample_index",
        "eo_x_m",
        "eo_y_m",
        "eo_heading_rad",
        "center_x_m",
        "center_y_m",
    ]
    _write_csv(
        final_dir / "aoi_release_table.csv",
        _release_rows(scenario, releases, original_releases),
        release_fields,
    )
    service_fields = [
        "aoi_id",
        "task_id",
        "semantic_type",
        "selected_service_mode",
        "default_mode_collision",
        "mirror_mode_collision",
        "scan_heading_rad",
        "entry_x_m",
        "entry_y_m",
        "exit_x_m",
        "exit_y_m",
        "scan_length_m",
        "service_time_s",
        "swath_width_m",
        "nominal_standoff_m",
        "aoi_fits_single_swath",
        "selected_scan_nfz_safe",
    ]
    _write_csv(
        final_dir / "sar_service_table.csv",
        _service_rows(scenario, services),
        service_fields,
    )
    intersection_fields = [
        "vehicle_type",
        "vehicle_id",
        "edge_id",
        "edge_type",
        "from_node",
        "to_node",
        "nfz_id",
        "safety_margin_m",
        "first_intersection_x",
        "first_intersection_y",
    ]
    _write_csv(
        final_dir / "nfz_intersections.csv",
        [asdict(item) for item in safe_eo_intersections + safe_sar_intersections],
        intersection_fields,
    )
    _write_timeline(
        final_dir / "CANONICAL_SAFE_TIMELINE.md", scenario, releases, result
    )

    plot_canonical_scene_final(scenario, final_dir / "canonical_scene_final.png")
    plot_eo_safe_sweep_and_detection(
        scenario,
        safe_sweep,
        releases,
        final_dir / "eo_safe_sweep_and_detection.png",
    )
    a12_task = next(aoi.task_id for aoi in scenario.aois if aoi.aoi_id == "A12")
    plot_sar_service_modes_example(
        scenario,
        candidates[a12_task][0],
        candidates[a12_task][1],
        final_dir / "sar_service_modes_example.png",
    )
    plot_canonical_safe_routes(
        scenario,
        safe_sweep,
        releases,
        safe_sar_edges,
        final_dir / "canonical_safe_routes.png",
    )
    _, snapshot_metadata = plot_canonical_replanning_snapshot(
        scenario,
        result,
        releases,
        uavs,
        tasks,
        services,
        obstacle_planner,
        final_dir / "canonical_replanning_snapshot.png",
    )

    diagnostic_metrics_path = ROOT / "results" / "canonical" / "canonical_metrics.json"
    diagnostic_metrics = json.loads(
        diagnostic_metrics_path.read_text(encoding="utf-8")
    )
    switched = [
        service.aoi_id
        for service in services.values()
        if service.selected_service_mode == "B"
    ]
    metrics = {
        "scenario_id": scenario.scenario_id,
        "initial_aoi_provenance": asdict(scenario.initial_information),
        "satellite_simulation_added": False,
        "eo_path_length_before": original_sweep.path_length_m,
        "eo_path_length_after": safe_sweep.path_length_m,
        "eo_detour_ratio": safe_sweep.path_length_m / original_sweep.path_length_m,
        "eo_detour_increase_percent": (
            safe_sweep.path_length_m / original_sweep.path_length_m - 1.0
        )
        * 100.0,
        "eo_detoured_legs": list(safe_sweep.detoured_leg_ids),
        "sar_total_travel_time_before": diagnostic_metrics["total_travel_time"],
        "sar_total_travel_time": result.metrics["total_travel_time"],
        "sar_travel_time_increase_percent": (
            float(result.metrics["total_travel_time"])
            / float(diagnostic_metrics["total_travel_time"])
            - 1.0
        )
        * 100.0,
        "weighted_delay": result.metrics["weighted_delay"],
        "weighted_mean_delay": result.metrics["weighted_mean_delay"],
        "makespan": result.metrics["makespan"],
        "eo_nfz_intersections_before": len(
            {item.edge_id for item in original_eo_intersections}
        ),
        "eo_nfz_intersections_after": len(safe_eo_intersections),
        "sar_nfz_intersections_before": diagnostic_metrics[
            "sar_nfz_intersecting_edge_count"
        ],
        "sar_nfz_intersections_after": len(safe_sar_intersections),
        "service_modes_switched_count": len(switched),
        "service_modes_switched_aois": switched,
        "completed_aoi_count": len(result.history),
        "replanning_count": result.metrics["replanning_count"],
        "assignment_changes": result.metrics["assignment_changes"],
        "successor_edge_changes": result.metrics["successor_edge_changes"],
        "time_consistency_check": result.metrics["time_consistency_check"],
        "feasible": result.metrics["feasible"],
        "max_mission_time_s": scenario.sar.max_mission_time_s,
        "nfz_rerouting_enabled": True,
        "replanning_snapshot": snapshot_metadata,
    }
    (final_dir / "canonical_safe_metrics.json").write_text(
        json.dumps(metrics, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(metrics, ensure_ascii=False, indent=2))
    print(f"outputs: {final_dir}")


if __name__ == "__main__":
    main()
