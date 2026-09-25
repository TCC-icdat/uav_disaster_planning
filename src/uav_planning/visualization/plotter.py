"""Simple debug visualization for initial and replanned routes."""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from uav_planning.geometry.dubins import sample_dubins_path
from uav_planning.models import Plan, Task, UAV


def _route_xy(route, uav: UAV, tasks: dict[int, Task]):
    pose = uav.start_pose
    xs = [pose.x]
    ys = [pose.y]
    step_size = max(0.25, uav.min_turn_radius / 10.0)
    for task_id in route.task_ids:
        target = tasks[task_id].pose
        path = sample_dubins_path(
            pose, target, uav.min_turn_radius, step_size=step_size
        )
        xs.extend(path[1:, 0])
        ys.extend(path[1:, 1])
        pose = target
    path = sample_dubins_path(
        pose, uav.start_pose, uav.min_turn_radius, step_size=step_size
    )
    xs.extend(path[1:, 0])
    ys.extend(path[1:, 1])
    return xs, ys


def plot_replanning_comparison(
    initial_plan: Plan,
    replanned_plan: Plan,
    uavs: dict[int, UAV],
    tasks: dict[int, Task],
    dynamic_task_ids: set[int],
    output_path: str | Path,
    width: float,
    height: float,
) -> Path:
    """Save side-by-side route schematics with priorities and new tasks."""

    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure, axes = plt.subplots(1, 2, figsize=(13, 6), constrained_layout=True)
    colors = plt.get_cmap("tab10")
    for axis, plan, title in zip(
        axes, (initial_plan, replanned_plan), ("Initial routes", "After replanning")
    ):
        all_x = [0.0, width]
        all_y = [0.0, height]
        axis.scatter([0.0], [0.0], marker="s", s=110, c="black", label="Depot")
        for task_id, task in tasks.items():
            marker = "*" if task_id in dynamic_task_ids else "o"
            size = 150 if marker == "*" else 65
            axis.scatter(task.x, task.y, marker=marker, s=size, c="tab:red" if marker == "*" else "tab:blue")
            axis.annotate(
                f"T{task_id} (p={task.priority:g})",
                (task.x, task.y),
                xytext=(4, 5),
                textcoords="offset points",
                fontsize=8,
            )
        for index, uav_id in enumerate(sorted(uavs)):
            xs, ys = _route_xy(plan.routes[uav_id], uavs[uav_id], tasks)
            all_x.extend(xs)
            all_y.extend(ys)
            axis.plot(xs, ys, "-", linewidth=2.0, color=colors(index), label=f"SAR{uav_id}")
        padding = max(width, height) * 0.04
        axis.set(
            xlim=(min(all_x) - padding, max(all_x) + padding),
            ylim=(min(all_y) - padding, max(all_y) + padding),
            title=title,
            xlabel="x",
            ylabel="y",
        )
        axis.grid(alpha=0.25)
        axis.set_aspect("equal", adjustable="box")
        axis.legend(loc="best", fontsize=8)
    figure.suptitle("Seed 42: initial plan and dynamic-task replanning")
    figure.savefig(output, dpi=180)
    plt.close(figure)
    return output
