"""Run the frozen canonical EO-to-SAR prototype once.

NFZs are diagnostic objects in this prototype: the planner deliberately uses
the existing obstacle-free Dubins edge cost and reports every intersection.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from dataclasses import asdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from uav_planning.canonical.eo import EOEventGenerator, EOSweepPlanner
from uav_planning.canonical.loader import load_canonical_scenario
from uav_planning.canonical.nfz import (
    diagnose_nfz_intersections,
    eo_route_edges,
    sar_route_edges,
)
from uav_planning.canonical.sar import (
    CanonicalRouteEvaluator,
    CanonicalTaskAdapter,
    build_sar_service_geometries,
)
from uav_planning.canonical.visualization import (
    plot_canonical_route_diagnostic,
    plot_canonical_scene_map,
    plot_eo_sweep_and_detection,
    plot_sar_service_geometry,
)
from uav_planning.planners import InitialPlanner, LocalReplanner
from uav_planning.routing.evaluator import DubinsTravelTimeProvider
from uav_planning.routing.insertion import RouteOptimizer
from uav_planning.simulation.simulator import Simulator


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        default="configs/canonical/post_earthquake_v1.yaml",
        help="Canonical scenario YAML, relative to the project root by default.",
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


def _release_rows(scenario, releases) -> list[dict[str, object]]:
    by_id = {aoi.aoi_id: aoi for aoi in scenario.aois}
    rows = []
    for event in sorted(releases, key=lambda item: item.task_id):
        aoi = by_id[event.aoi_id]
        pose = event.eo_pose_at_detection
        rows.append(
            {
                "aoi_id": event.aoi_id,
                "task_id": event.task_id,
                "semantic_type": event.semantic_type,
                "center_x_m": aoi.center[0],
                "center_y_m": aoi.center[1],
                "length_m": aoi.length_m,
                "width_m": aoi.width_m,
                "priority": event.priority,
                "initially_known": event.initially_known,
                "release_time_s": event.release_time,
                "eo_path_sample_index": event.path_sample_index,
                "eo_x_m": None if pose is None else pose.x,
                "eo_y_m": None if pose is None else pose.y,
                "eo_heading_rad": None if pose is None else pose.heading,
            }
        )
    return rows


def _service_rows(scenario, services) -> list[dict[str, object]]:
    by_task = {aoi.task_id: aoi for aoi in scenario.aois}
    rows = []
    for task_id, service in sorted(services.items()):
        aoi = by_task[task_id]
        rows.append(
            {
                "aoi_id": service.aoi_id,
                "task_id": task_id,
                "semantic_type": aoi.semantic_type,
                "scan_heading_rad": service.scan_heading_rad,
                "entry_x_m": service.entry_pose.x,
                "entry_y_m": service.entry_pose.y,
                "exit_x_m": service.exit_pose.x,
                "exit_y_m": service.exit_pose.y,
                "scan_length_m": service.scan_length_m,
                "service_time_s": service.service_time_s,
                "inner_ground_range_m": service.inner_ground_range_m,
                "outer_ground_range_m": service.outer_ground_range_m,
                "swath_width_m": service.swath_width_m,
                "nominal_standoff_m": service.nominal_standoff_m,
                "aoi_fits_single_swath": service.aoi_fits,
            }
        )
    return rows


def _write_timeline(path: Path, scenario, releases, result) -> None:
    aoi_by_task = {aoi.task_id: aoi for aoi in scenario.aois}
    events: list[tuple[float, int, str]] = []
    initial = [event.aoi_id for event in releases if event.initially_known]
    events.append((0.0, 0, "Initial SAR AOIs released: " + ", ".join(initial)))
    for event in releases:
        if event.initially_known:
            continue
        events.append(
            (
                event.release_time,
                1,
                f"EO fully covers {event.aoi_id} ({event.semantic_type}); SAR task released",
            )
        )
        events.append(
            (
                event.release_time,
                2,
                f"Local replanning triggered by {event.aoi_id} (h={scenario.sar.affected_uav_count_h})",
            )
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
        "# Canonical Scenario Timeline",
        "",
        "| Time (s) | Event |",
        "|---:|---|",
    ]
    lines.extend(
        f"| {time_value:.3f} | {description} |"
        for time_value, _, description in sorted(events)
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    args = parse_args()
    scenario = load_canonical_scenario(_resolve(args.config))
    sweep = EOSweepPlanner(scenario).plan()
    releases = EOEventGenerator(scenario, sweep).releases()
    services = build_sar_service_geometries(scenario)
    adapter = CanonicalTaskAdapter(scenario, services, releases)
    planning_scenario = adapter.planning_scenario()

    evaluator = CanonicalRouteEvaluator(DubinsTravelTimeProvider(), services)
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
    if not bool(result.metrics["feasible"]):
        raise RuntimeError("canonical Local run is infeasible")
    if len(result.history) != len(scenario.aois):
        raise RuntimeError("canonical Local run did not complete every AOI")

    output_dir = ROOT / "results" / "canonical"
    figure_dir = output_dir / "figures"
    output_dir.mkdir(parents=True, exist_ok=True)
    figure_dir.mkdir(parents=True, exist_ok=True)

    release_fields = [
        "aoi_id",
        "task_id",
        "semantic_type",
        "center_x_m",
        "center_y_m",
        "length_m",
        "width_m",
        "priority",
        "initially_known",
        "release_time_s",
        "eo_path_sample_index",
        "eo_x_m",
        "eo_y_m",
        "eo_heading_rad",
    ]
    _write_csv(
        output_dir / "aoi_release_table.csv",
        _release_rows(scenario, releases),
        release_fields,
    )
    service_fields = [
        "aoi_id",
        "task_id",
        "semantic_type",
        "scan_heading_rad",
        "entry_x_m",
        "entry_y_m",
        "exit_x_m",
        "exit_y_m",
        "scan_length_m",
        "service_time_s",
        "inner_ground_range_m",
        "outer_ground_range_m",
        "swath_width_m",
        "nominal_standoff_m",
        "aoi_fits_single_swath",
    ]
    _write_csv(
        output_dir / "sar_service_table.csv",
        _service_rows(scenario, services),
        service_fields,
    )

    uavs = {uav.uav_id: uav for uav in planning_scenario.uavs}
    tasks = {task.task_id: task for task in planning_scenario.all_tasks}
    eo_edges = eo_route_edges(sweep)
    sar_edges = sar_route_edges(result, uavs, tasks, services)
    intersections = diagnose_nfz_intersections(
        eo_edges + sar_edges, scenario.no_fly_zones
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
        output_dir / "nfz_intersections.csv",
        [asdict(item) for item in intersections],
        intersection_fields,
    )

    metrics = {
        **result.metrics,
        "scenario_id": scenario.scenario_id,
        "eo_path_length_m": sweep.path_length_m,
        "eo_total_sweep_time_s": sweep.path_length_m / scenario.eo.speed_mps,
        "eo_lane_count": sweep.lane_count,
        "eo_footprint_cross_track_m": sweep.footprint.cross_track_width_m,
        "eo_footprint_along_track_m": sweep.footprint.along_track_length_m,
        "eo_nfz_intersection_rows": sum(
            item.vehicle_type == "EO" for item in intersections
        ),
        "eo_nfz_intersecting_edge_count": len(
            {item.edge_id for item in intersections if item.vehicle_type == "EO"}
        ),
        "sar_nfz_intersection_rows": sum(
            item.vehicle_type == "SAR" for item in intersections
        ),
        "sar_nfz_intersecting_edge_count": len(
            {item.edge_id for item in intersections if item.vehicle_type == "SAR"}
        ),
        "nfz_rerouting_enabled": False,
    }
    (output_dir / "canonical_metrics.json").write_text(
        json.dumps(metrics, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    _write_timeline(
        output_dir / "CANONICAL_TIMELINE.md", scenario, releases, result
    )

    plot_canonical_scene_map(
        scenario, figure_dir / "canonical_scene_map.png"
    )
    plot_eo_sweep_and_detection(
        scenario, sweep, releases, figure_dir / "eo_sweep_and_detection.png"
    )
    hospital = services[
        next(aoi.task_id for aoi in scenario.aois if aoi.semantic_type == "Hospital")
    ]
    plot_sar_service_geometry(
        scenario, hospital, figure_dir / "sar_service_geometry.png"
    )
    plot_canonical_route_diagnostic(
        scenario,
        sweep,
        releases,
        sar_edges,
        intersections,
        figure_dir / "canonical_route_diagnostic.png",
    )

    print(json.dumps(metrics, ensure_ascii=False, indent=2))
    print(f"outputs: {output_dir}")


if __name__ == "__main__":
    main()
