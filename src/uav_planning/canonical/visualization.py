"""Canonical scenario figures for physical interpretation and diagnostics."""

from __future__ import annotations

from math import cos, degrees, sin
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

from uav_planning.canonical.models import (
    CanonicalRouteEdge,
    CanonicalScenario,
    EOReleaseEvent,
    EOSweepPlan,
    NFZIntersection,
    SARServiceGeometry,
)


def _add_polygon(axis, points, **kwargs):
    patch = Polygon(points, closed=True, **kwargs)
    axis.add_patch(patch)
    return patch


def _draw_nfz(axis, scenario: CanonicalScenario) -> None:
    for index, zone in enumerate(scenario.no_fly_zones):
        _add_polygon(
            axis,
            zone.polygon,
            facecolor="#D62728",
            edgecolor="#8B0000",
            alpha=0.24,
            linewidth=1.4,
            label="NFZ" if index == 0 else None,
        )
        _add_polygon(
            axis,
            zone.inflated_polygon,
            fill=False,
            edgecolor="#D62728",
            linestyle="--",
            linewidth=1.0,
            label="100 m safety margin" if index == 0 else None,
        )
        center_x = sum(point[0] for point in zone.polygon) / len(zone.polygon)
        center_y = sum(point[1] for point in zone.polygon) / len(zone.polygon)
        axis.text(center_x, center_y, zone.zone_id, ha="center", va="center", color="#8B0000", fontsize=8)


def _draw_aois(axis, scenario: CanonicalScenario, annotate: bool = True) -> None:
    for index, aoi in enumerate(scenario.aois):
        color = "#2CA02C" if aoi.initially_known else "#1F77B4"
        _add_polygon(
            axis,
            aoi.polygon,
            facecolor=color,
            edgecolor=color,
            alpha=0.34,
            linewidth=1.2,
            label=(
                "Initial AOI"
                if index == 0
                else "Hidden AOI" if index == 1 else None
            ),
        )
        if annotate:
            axis.annotate(
                f"{aoi.aoi_id} {aoi.semantic_type}\np={aoi.priority:g}",
                aoi.center,
                xytext=(3, 3),
                textcoords="offset points",
                fontsize=6.8,
            )


def _finish_map(axis, scenario: CanonicalScenario) -> None:
    padding = 350.0
    axis.set_xlim(-650.0, scenario.width_m + padding)
    axis.set_ylim(-padding, scenario.height_m + padding)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("x (m)")
    axis.set_ylabel("y (m)")
    axis.grid(alpha=0.18)


def plot_canonical_scene_map(
    scenario: CanonicalScenario, output_path: str | Path
) -> Path:
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig, axis = plt.subplots(figsize=(9.2, 8.2))
    _draw_nfz(axis, scenario)
    _draw_aois(axis, scenario)
    axis.scatter(
        [scenario.depot.x], [scenario.depot.y], marker="s", s=90, c="black", label="Depot"
    )
    axis.annotate("Depot", (scenario.depot.x, scenario.depot.y), xytext=(5, 5), textcoords="offset points")
    _finish_map(axis, scenario)
    axis.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(output, dpi=250, bbox_inches="tight")
    plt.close(fig)
    return output


def plot_eo_sweep_and_detection(
    scenario: CanonicalScenario,
    sweep: EOSweepPlan,
    events: tuple[EOReleaseEvent, ...],
    output_path: str | Path,
) -> Path:
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig, axis = plt.subplots(figsize=(9.2, 8.2))
    _draw_nfz(axis, scenario)
    _draw_aois(axis, scenario, annotate=False)
    axis.plot(sweep.samples[:, 0], sweep.samples[:, 1], color="#606060", linewidth=0.75, label="EO sweep")
    hidden = [event for event in events if not event.initially_known]
    for order, event in enumerate(hidden, start=1):
        aoi = next(item for item in scenario.aois if item.aoi_id == event.aoi_id)
        axis.annotate(
            f"{order}. {event.aoi_id}\nt={event.release_time:.1f}s",
            aoi.center,
            xytext=(4, 4),
            textcoords="offset points",
            fontsize=7,
            color="#0B3C6F",
        )
    representative = hidden[len(hidden) // 2]
    pose = representative.eo_pose_at_detection
    if pose is not None:
        _add_polygon(
            axis,
            sweep.footprint.polygon_at(pose),
            facecolor="#FFBF00",
            edgecolor="#B37400",
            alpha=0.25,
            linewidth=1.3,
            label="Representative EO footprint",
        )
    _finish_map(axis, scenario)
    axis.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(output, dpi=250, bbox_inches="tight")
    plt.close(fig)
    return output


def plot_sar_service_geometry(
    scenario: CanonicalScenario,
    service: SARServiceGeometry,
    output_path: str | Path,
) -> Path:
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    aoi = next(item for item in scenario.aois if item.task_id == service.task_id)
    fig, axis = plt.subplots(figsize=(8.2, 6.3))
    _add_polygon(axis, service.swath_polygon, facecolor="#FFBF00", edgecolor="#B37400", alpha=0.25, label="Effective SAR swath")
    _add_polygon(axis, aoi.polygon, facecolor="#2CA02C", edgecolor="#196619", alpha=0.55, label=f"{aoi.aoi_id} {aoi.semantic_type}")
    axis.plot(
        [service.entry_pose.x, service.exit_pose.x],
        [service.entry_pose.y, service.exit_pose.y],
        color="#1F77B4",
        linewidth=3.0,
        label="SAR scan segment",
    )
    axis.scatter([service.entry_pose.x], [service.entry_pose.y], marker="o", s=70, c="#1F77B4", label="Entry")
    axis.scatter([service.exit_pose.x], [service.exit_pose.y], marker="X", s=80, c="#D62728", label="Exit")
    heading = service.scan_heading_rad
    axis.arrow(
        service.entry_pose.x,
        service.entry_pose.y,
        160.0 * cos(heading),
        160.0 * sin(heading),
        width=4.0,
        head_width=35.0,
        color="#1F77B4",
        length_includes_head=True,
    )
    line_mid = (
        (service.entry_pose.x + service.exit_pose.x) / 2.0,
        (service.entry_pose.y + service.exit_pose.y) / 2.0,
    )
    axis.plot([line_mid[0], aoi.center[0]], [line_mid[1], aoi.center[1]], "--", color="#555555", linewidth=1.2)
    axis.text(
        (line_mid[0] + aoi.center[0]) / 2.0,
        (line_mid[1] + aoi.center[1]) / 2.0,
        f" stand-off {service.nominal_standoff_m:.1f} m",
        fontsize=8,
    )
    axis.text(
        line_mid[0],
        line_mid[1] - 45.0,
        f"scan {service.scan_length_m:.0f} m / {service.service_time_s:.1f} s\nheading {degrees(heading):.0f}°",
        ha="center",
        va="top",
        fontsize=8,
    )
    all_points = list(service.swath_polygon) + list(aoi.polygon) + [
        (service.entry_pose.x, service.entry_pose.y),
        (service.exit_pose.x, service.exit_pose.y),
    ]
    xs = [point[0] for point in all_points]
    ys = [point[1] for point in all_points]
    padding = 180.0
    axis.set_xlim(min(xs) - padding, max(xs) + padding)
    axis.set_ylim(min(ys) - padding, max(ys) + padding)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("x (m)")
    axis.set_ylabel("y (m)")
    axis.grid(alpha=0.2)
    axis.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(output, dpi=250, bbox_inches="tight")
    plt.close(fig)
    return output


def plot_canonical_route_diagnostic(
    scenario: CanonicalScenario,
    sweep: EOSweepPlan,
    releases: tuple[EOReleaseEvent, ...],
    sar_edges: tuple[CanonicalRouteEdge, ...],
    intersections: tuple[NFZIntersection, ...],
    output_path: str | Path,
) -> Path:
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig, axis = plt.subplots(figsize=(10.0, 8.4))
    _draw_nfz(axis, scenario)
    _draw_aois(axis, scenario, annotate=False)
    axis.plot(sweep.samples[:, 0], sweep.samples[:, 1], color="#AAAAAA", linewidth=0.5, alpha=0.45, label="EO sweep")
    intersecting_edges = {item.edge_id for item in intersections if item.vehicle_type == "SAR"}
    colors = {"SAR1": "#1F77B4", "SAR2": "#2CA02C", "SAR3": "#9467BD"}
    labeled: set[str] = set()
    for edge in sar_edges:
        intersects_nfz = edge.edge_id in intersecting_edges
        color = "#D62728" if intersects_nfz else colors[edge.vehicle_id]
        linewidth = 3.0 if intersects_nfz else 1.6
        if intersects_nfz:
            label = (
                "NFZ-intersecting SAR edge"
                if "collision" not in labeled
                else None
            )
            labeled.add("collision")
        else:
            label = edge.vehicle_id if edge.vehicle_id not in labeled else None
            labeled.add(edge.vehicle_id)
        axis.plot(edge.samples[:, 0], edge.samples[:, 1], color=color, linewidth=linewidth, alpha=0.9, label=label)
    for item in intersections:
        if item.vehicle_type == "SAR":
            axis.scatter([item.first_intersection_x], [item.first_intersection_y], marker="x", s=70, linewidth=2.0, c="#8B0000")
    hidden = [event for event in releases if not event.initially_known]
    for event in hidden:
        aoi = next(item for item in scenario.aois if item.aoi_id == event.aoi_id)
        axis.annotate(f"{aoi.aoi_id}\n{event.release_time:.0f}s", aoi.center, xytext=(3, 3), textcoords="offset points", fontsize=6.5)
    axis.scatter([scenario.depot.x], [scenario.depot.y], marker="s", s=85, c="black", label="Depot")
    _finish_map(axis, scenario)
    axis.legend(loc="upper right", fontsize=7.5)
    fig.tight_layout()
    fig.savefig(output, dpi=250, bbox_inches="tight")
    plt.close(fig)
    return output
