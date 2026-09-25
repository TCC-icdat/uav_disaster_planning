"""Paper-oriented figures for the collision-free canonical scenario."""

from __future__ import annotations

from math import cos, sin
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon

from uav_planning.canonical.models import (
    CanonicalRouteEdge,
    CanonicalScenario,
    EOReleaseEvent,
    EOSweepPlan,
    SARServiceGeometry,
)
from uav_planning.canonical.obstacle import (
    ObstacleAwareDubinsPlanner,
    polyline_polygon_first_intersection,
)
from uav_planning.models import Pose2D, Task, UAV
from uav_planning.simulation.simulator import SimulationResult


_SAR_COLORS = {"SAR1": "#1F77B4", "SAR2": "#2CA02C", "SAR3": "#9467BD"}


def _polygon(axis, points, **kwargs):
    patch = Polygon(points, closed=True, **kwargs)
    axis.add_patch(patch)
    return patch


def _draw_nfz(axis, scenario: CanonicalScenario) -> None:
    for index, zone in enumerate(scenario.no_fly_zones):
        _polygon(
            axis,
            zone.polygon,
            facecolor="#D62728",
            edgecolor="#9B1C1C",
            alpha=0.16,
            linewidth=1.0,
            label="Restricted airspace" if index == 0 else None,
        )
        _polygon(
            axis,
            zone.inflated_polygon,
            fill=False,
            edgecolor="#D62728",
            linestyle="--",
            linewidth=1.2,
            label="100 m representative buffer" if index == 0 else None,
        )
        cx = sum(point[0] for point in zone.polygon) / len(zone.polygon)
        cy = sum(point[1] for point in zone.polygon) / len(zone.polygon)
        axis.text(cx, cy, zone.zone_id, color="#8B0000", ha="center", va="center", fontsize=8)


def _draw_aois(
    axis,
    scenario: CanonicalScenario,
    annotate: bool = True,
    compact: bool = False,
) -> None:
    initial_labeled = False
    hidden_labeled = False
    for aoi in scenario.aois:
        initial = aoi.initially_known
        color = "#2CA02C" if initial else "#4C9BD3"
        label = None
        if initial and not initial_labeled:
            label = "Initial AOI (multi-source prior)"
            initial_labeled = True
        if not initial and not hidden_labeled:
            label = "EO-discovered AOI"
            hidden_labeled = True
        _polygon(
            axis,
            aoi.polygon,
            facecolor=color,
            edgecolor=color,
            alpha=0.34,
            linewidth=1.1,
            label=label,
        )
        if annotate:
            text = aoi.aoi_id if compact else f"{aoi.aoi_id} {aoi.semantic_type}\np={aoi.priority:g}"
            axis.annotate(
                text,
                aoi.center,
                xytext=(4, 4),
                textcoords="offset points",
                fontsize=7 if compact else 6.8,
            )


def _finish(axis, scenario: CanonicalScenario) -> None:
    axis.set_xlim(-650.0, scenario.width_m + 350.0)
    axis.set_ylim(-350.0, scenario.height_m + 350.0)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("x (m)")
    axis.set_ylabel("y (m)")
    axis.grid(alpha=0.16)


def plot_canonical_scene_final(
    scenario: CanonicalScenario, output_path: str | Path
) -> Path:
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig, axis = plt.subplots(figsize=(9.6, 8.2))
    _draw_nfz(axis, scenario)
    _draw_aois(axis, scenario)
    axis.scatter([scenario.depot.x], [scenario.depot.y], marker="s", s=85, c="black", label="Depot")
    axis.annotate("Depot", (scenario.depot.x, scenario.depot.y), xytext=(5, 5), textcoords="offset points")
    axis.text(
        0.015,
        0.985,
        "Initial AOIs: satellite remote sensing + historical geographic information\n"
        "+ ground emergency reports (prior input only; no satellite simulation)",
        transform=axis.transAxes,
        va="top",
        fontsize=8,
        bbox={"boxstyle": "round,pad=0.35", "facecolor": "white", "alpha": 0.88, "edgecolor": "#777777"},
    )
    _finish(axis, scenario)
    axis.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(output, dpi=250, bbox_inches="tight")
    plt.close(fig)
    return output


def plot_eo_safe_sweep_and_detection(
    scenario: CanonicalScenario,
    safe_sweep: EOSweepPlan,
    releases: tuple[EOReleaseEvent, ...],
    output_path: str | Path,
) -> Path:
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig, axis = plt.subplots(figsize=(9.8, 8.2))
    _draw_nfz(axis, scenario)
    _draw_aois(axis, scenario, annotate=False)
    detoured = set(safe_sweep.detoured_leg_ids)
    safe_label_used = False
    detour_label_used = False
    for leg in safe_sweep.legs:
        samples = safe_sweep.samples[leg.start_index : leg.end_index + 1]
        is_detour = leg.leg_id in detoured
        label = None
        if is_detour and not detour_label_used:
            label = "NFZ-safe local detour"
            detour_label_used = True
        elif not is_detour and not safe_label_used:
            label = "Unchanged EO sweep leg"
            safe_label_used = True
        axis.plot(
            samples[:, 0],
            samples[:, 1],
            color="#F28E2B" if is_detour else "#5B7083",
            linewidth=1.35 if is_detour else 0.65,
            alpha=0.95 if is_detour else 0.7,
            label=label,
        )
    hidden = [event for event in releases if not event.initially_known]
    for order, event in enumerate(hidden, start=1):
        aoi = next(item for item in scenario.aois if item.aoi_id == event.aoi_id)
        axis.annotate(
            f"{order}. {event.aoi_id}\n{event.release_time:.1f}s",
            aoi.center,
            xytext=(4, 4),
            textcoords="offset points",
            fontsize=7,
            color="#0B3C6F",
        )
    _finish(axis, scenario)
    axis.legend(loc="upper right", fontsize=7.5)
    fig.tight_layout()
    fig.savefig(output, dpi=250, bbox_inches="tight")
    plt.close(fig)
    return output


def plot_sar_service_modes_example(
    scenario: CanonicalScenario,
    mode_a: SARServiceGeometry,
    mode_b: SARServiceGeometry,
    output_path: str | Path,
) -> Path:
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    aoi = next(item for item in scenario.aois if item.task_id == mode_a.task_id)
    fig, axis = plt.subplots(figsize=(8.4, 6.7))
    relevant_zones = [
        zone
        for zone in scenario.no_fly_zones
        if polyline_polygon_first_intersection(
            np.asarray(aoi.polygon + (aoi.polygon[0],)), zone.inflated_polygon
        )
        is not None
        or polyline_polygon_first_intersection(
            np.asarray(
                [
                    (mode_a.entry_pose.x, mode_a.entry_pose.y),
                    (mode_a.exit_pose.x, mode_a.exit_pose.y),
                ]
            ),
            zone.inflated_polygon,
        )
        is not None
    ]
    for index, zone in enumerate(relevant_zones):
        _polygon(
            axis,
            zone.polygon,
            facecolor="#D62728",
            edgecolor="#9B1C1C",
            alpha=0.16,
            label="Restricted airspace" if index == 0 else None,
        )
        _polygon(
            axis,
            zone.inflated_polygon,
            fill=False,
            edgecolor="#D62728",
            linestyle="--",
            linewidth=1.2,
            label="100 m representative buffer" if index == 0 else None,
        )
        cx = sum(point[0] for point in zone.polygon) / len(zone.polygon)
        cy = sum(point[1] for point in zone.polygon) / len(zone.polygon)
        axis.text(cx, cy, zone.zone_id, color="#8B0000", ha="center", va="center", fontsize=8)
    _polygon(axis, aoi.polygon, facecolor="#4C9BD3", edgecolor="#1F5A83", alpha=0.55, label=f"{aoi.aoi_id} {aoi.semantic_type}")
    _polygon(axis, mode_a.swath_polygon, facecolor="#D62728", edgecolor="#D62728", alpha=0.08, label="Mode A swath")
    _polygon(axis, mode_b.swath_polygon, facecolor="#2CA02C", edgecolor="#2CA02C", alpha=0.10, label="Mode B swath")
    axis.plot(
        [mode_a.entry_pose.x, mode_a.exit_pose.x],
        [mode_a.entry_pose.y, mode_a.exit_pose.y],
        "--",
        color="#D62728",
        linewidth=2.5,
        label="Mode A: rejected (NFZ collision)",
    )
    axis.plot(
        [mode_b.entry_pose.x, mode_b.exit_pose.x],
        [mode_b.entry_pose.y, mode_b.exit_pose.y],
        color="#2CA02C",
        linewidth=3.0,
        label="Mode B: selected, fixed left-looking",
    )
    mode_a_samples = np.asarray(
        [
            (mode_a.entry_pose.x, mode_a.entry_pose.y),
            (mode_a.exit_pose.x, mode_a.exit_pose.y),
        ]
    )
    collision = next(
        (
            point
            for zone in scenario.no_fly_zones
            if (
                point := polyline_polygon_first_intersection(
                    mode_a_samples, zone.inflated_polygon
                )
            )
            is not None
        ),
        None,
    )
    if collision is not None:
        axis.scatter([collision[0]], [collision[1]], marker="x", s=90, linewidth=2.2, c="#8B0000", label="Mode A collision")
    for service, color in ((mode_a, "#D62728"), (mode_b, "#2CA02C")):
        axis.arrow(
            service.entry_pose.x,
            service.entry_pose.y,
            140.0 * cos(service.scan_heading_rad),
            140.0 * sin(service.scan_heading_rad),
            width=4.0,
            head_width=35.0,
            color=color,
            length_includes_head=True,
        )
    points = (
        list(mode_a.swath_polygon)
        + list(mode_b.swath_polygon)
        + list(aoi.polygon)
        + [
            (mode_a.entry_pose.x, mode_a.entry_pose.y),
            (mode_a.exit_pose.x, mode_a.exit_pose.y),
            (mode_b.entry_pose.x, mode_b.entry_pose.y),
            (mode_b.exit_pose.x, mode_b.exit_pose.y),
        ]
    )
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    axis.set_xlim(min(xs) - 220.0, max(xs) + 220.0)
    axis.set_ylim(min(ys) - 220.0, max(ys) + 220.0)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("x (m)")
    axis.set_ylabel("y (m)")
    axis.grid(alpha=0.18)
    axis.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), fontsize=8)
    fig.subplots_adjust(right=0.70)
    fig.savefig(output, dpi=250, bbox_inches="tight")
    plt.close(fig)
    return output


def plot_canonical_safe_routes(
    scenario: CanonicalScenario,
    safe_sweep: EOSweepPlan,
    releases: tuple[EOReleaseEvent, ...],
    sar_edges: tuple[CanonicalRouteEdge, ...],
    output_path: str | Path,
) -> Path:
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig, axis = plt.subplots(figsize=(10.2, 8.4))
    _draw_nfz(axis, scenario)
    _draw_aois(axis, scenario, annotate=False)
    axis.plot(safe_sweep.samples[:, 0], safe_sweep.samples[:, 1], color="#9AA0A6", linewidth=0.5, alpha=0.35, label="NFZ-safe EO sweep")
    labeled: set[str] = set()
    for edge in sar_edges:
        label = edge.vehicle_id if edge.vehicle_id not in labeled else None
        labeled.add(edge.vehicle_id)
        axis.plot(
            edge.samples[:, 0],
            edge.samples[:, 1],
            color=_SAR_COLORS[edge.vehicle_id],
            linewidth=1.65,
            alpha=0.9,
            label=label,
        )
    for event in releases:
        aoi = next(item for item in scenario.aois if item.aoi_id == event.aoi_id)
        label = f"{aoi.aoi_id}\n{event.release_time:.0f}s" if not event.initially_known else f"{aoi.aoi_id}\nt=0"
        axis.annotate(label, aoi.center, xytext=(3, 3), textcoords="offset points", fontsize=6.5)
    axis.scatter([scenario.depot.x], [scenario.depot.y], marker="s", s=85, c="black", label="Depot")
    _finish(axis, scenario)
    axis.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), fontsize=8)
    fig.tight_layout()
    fig.savefig(output, dpi=250, bbox_inches="tight")
    plt.close(fig)
    return output


def _future_route_segments(
    start: Pose2D,
    task_ids: list[int],
    uav: UAV,
    tasks: dict[int, Task],
    services: dict[int, SARServiceGeometry],
    planner: ObstacleAwareDubinsPlanner,
) -> list[tuple[np.ndarray, str]]:
    segments: list[tuple[np.ndarray, str]] = []
    current = start
    for task_id in task_ids:
        task = tasks[task_id]
        service = services[task_id]
        segments.append((planner.plan(current, task.pose, uav.min_turn_radius).samples, "transfer"))
        segments.append(
            (
                np.asarray(
                    [
                        (service.entry_pose.x, service.entry_pose.y, service.entry_pose.heading),
                        (service.exit_pose.x, service.exit_pose.y, service.exit_pose.heading),
                    ],
                    dtype=float,
                ),
                "service",
            )
        )
        current = service.exit_pose
    return segments


def plot_canonical_replanning_snapshot(
    scenario: CanonicalScenario,
    result: SimulationResult,
    releases: tuple[EOReleaseEvent, ...],
    uavs: dict[int, UAV],
    tasks: dict[int, Task],
    services: dict[int, SARServiceGeometry],
    planner: ObstacleAwareDubinsPlanner,
    output_path: str | Path,
) -> tuple[Path, dict[str, object]]:
    selected = None
    for index, replanning in enumerate(result.records):
        after_plan = result.snapshots[index + 1][1]
        affected = next(
            uav_id
            for uav_id, route in after_plan.routes.items()
            if replanning.task_id in route.task_ids
        )
        executing = next(
            (
                record
                for record in result.history.values()
                if record.uav_id == affected
                and record.task_id != replanning.task_id
                and record.departure_time < replanning.release_time - 1e-9
                and replanning.release_time < record.completion_time
            ),
            None,
        )
        if executing is not None:
            selected = (index, replanning, after_plan, affected, executing)
            break
    if selected is None:
        index = 0
        replanning = result.records[0]
        after_plan = result.snapshots[1][1]
        affected = next(
            uav_id
            for uav_id, route in after_plan.routes.items()
            if replanning.task_id in route.task_ids
        )
        selected = (index, replanning, after_plan, affected, None)
    _, replanning, after_plan, affected, executing = selected
    event_time = replanning.release_time
    new_task_id = replanning.task_id
    release_by_task = {event.task_id: event for event in releases}

    fig, axes = plt.subplots(1, 2, figsize=(15.5, 7.0), sharex=True, sharey=True)
    for panel_index, axis in enumerate(axes):
        _draw_nfz(axis, scenario)
        known_ids = {
            event.task_id
            for event in releases
            if event.release_time < event_time - 1e-9
            or (event.initially_known and event.release_time == 0.0)
        }
        for aoi in scenario.aois:
            if aoi.task_id == new_task_id and panel_index == 0:
                _polygon(axis, aoi.polygon, fill=False, edgecolor="#777777", linestyle=":", linewidth=1.2)
                axis.annotate(f"{aoi.aoi_id}\nnot released", aoi.center, xytext=(3, 3), textcoords="offset points", fontsize=7, color="#666666")
            elif aoi.task_id in known_ids or panel_index == 1:
                highlight = aoi.task_id == new_task_id
                _polygon(
                    axis,
                    aoi.polygon,
                    facecolor="#FFBF00" if highlight else ("#2CA02C" if aoi.initially_known else "#4C9BD3"),
                    edgecolor="#B37400" if highlight else "#4C7890",
                    alpha=0.65 if highlight else 0.25,
                    linewidth=1.5 if highlight else 0.9,
                )
                axis.annotate(aoi.aoi_id, aoi.center, xytext=(3, 3), textcoords="offset points", fontsize=7)

        completed = {
            record.task_id
            for record in result.history.values()
            if record.completion_time <= event_time + 1e-9
        }
        executing_by_uav = {
            record.uav_id: record
            for record in result.history.values()
            if record.departure_time <= event_time < record.completion_time
        }
        for uav_id, route in after_plan.routes.items():
            execution = executing_by_uav.get(uav_id)
            if execution is not None:
                start = services[execution.task_id].exit_pose
            else:
                start = uavs[uav_id].start_pose
            future = [
                task_id
                for task_id in route.task_ids
                if task_id not in completed
                and (execution is None or task_id != execution.task_id)
            ]
            if panel_index == 0:
                future = [task_id for task_id in future if task_id != new_task_id]
            alpha = 0.95 if uav_id == affected else 0.18
            width = 2.4 if uav_id == affected else 1.0
            for samples, segment_type in _future_route_segments(
                start, future, uavs[uav_id], tasks, services, planner
            ):
                axis.plot(
                    samples[:, 0],
                    samples[:, 1],
                    color=_SAR_COLORS[f"SAR{uav_id}"],
                    linewidth=width + (0.5 if segment_type == "service" else 0.0),
                    alpha=alpha,
                )
        if executing is not None:
            service = services[executing.task_id]
            axis.plot(
                [service.entry_pose.x, service.exit_pose.x],
                [service.entry_pose.y, service.exit_pose.y],
                color="#F28E2B",
                linewidth=4.0,
                label=f"SAR{affected} executing {service.aoi_id}",
            )
        axis.scatter([scenario.depot.x], [scenario.depot.y], marker="s", s=65, c="black")
        axis.set_title("Before release" if panel_index == 0 else "After Local replanning")
        _finish(axis, scenario)
    after_future = [
        task_id
        for task_id in after_plan.routes[affected].task_ids
        if task_id not in {
            record.task_id
            for record in result.history.values()
            if record.completion_time <= event_time + 1e-9
        }
        and (executing is None or task_id != executing.task_id)
    ]
    insertion_position = after_future.index(new_task_id) + 1
    fig.suptitle(
        f"t={event_time:.1f}s: {release_by_task[new_task_id].aoi_id} released and inserted into "
        f"SAR{affected} future suffix at position {insertion_position}\n"
        "Existing old-task assignment/order remains unchanged",
        fontsize=11,
    )
    axes[1].legend(loc="upper right", fontsize=8)
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.93))
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=250, bbox_inches="tight")
    plt.close(fig)
    metadata = {
        "release_time_s": event_time,
        "new_task_id": new_task_id,
        "new_aoi_id": release_by_task[new_task_id].aoi_id,
        "affected_uav_id": affected,
        "executing_task_id": None if executing is None else executing.task_id,
        "insertion_position_in_future_suffix": insertion_position,
    }
    return output, metadata
