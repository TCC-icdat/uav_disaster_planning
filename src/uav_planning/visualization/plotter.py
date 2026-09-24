"""Simple debug visualization for initial and replanned routes."""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from uav_planning.models import Plan, Task, UAV


def _route_xy(route, uav: UAV, tasks: dict[int, Task]):
    xs = [uav.start_pose.x] + [tasks[task_id].x for task_id in route.task_ids]
    ys = [uav.start_pose.y] + [tasks[task_id].y for task_id in route.task_ids]
    xs.append(uav.start_pose.x)
    ys.append(uav.start_pose.y)
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
            axis.plot(xs, ys, "--o", color=colors(index), label=f"SAR{uav_id}")
        axis.set(xlim=(-2, width + 2), ylim=(-2, height + 2), title=title, xlabel="x", ylabel="y")
        axis.grid(alpha=0.25)
        axis.set_aspect("equal", adjustable="box")
        axis.legend(loc="best", fontsize=8)
    figure.suptitle("Seed 42: initial plan and dynamic-task replanning")
    figure.savefig(output, dpi=180)
    plt.close(figure)
    return output
