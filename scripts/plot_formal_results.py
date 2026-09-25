"""Create publication-oriented figures from frozen formal CSV outputs."""

from __future__ import annotations

import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[1]
FORMAL = ROOT / "results" / "formal"
FIGURES = FORMAL / "figures"
COLORS = {
    "time_aware": "#2878B5",
    "mission_completion": "#3A923A",
    "distance_oriented": "#E17C05",
    "dubins_aware": "#2878B5",
    "euclidean_planned": "#E17C05",
    "no_reorder": "#8C8C8C",
    "full": "#E17C05",
    "local": "#2878B5",
}


def _mean_ci(values: pd.Series) -> tuple[float, float]:
    numeric = pd.to_numeric(values, errors="coerce").dropna()
    mean = float(numeric.mean())
    if len(numeric) < 2:
        return mean, 0.0
    return mean, 1.96 * float(numeric.std(ddof=1)) / math.sqrt(len(numeric))


def _finish(fig: plt.Figure, name: str) -> Path:
    FIGURES.mkdir(parents=True, exist_ok=True)
    path = FIGURES / name
    fig.savefig(path, dpi=300, bbox_inches="tight")
    plt.close(fig)
    return path


def plot_e1() -> Path | None:
    path = FORMAL / "E1" / "raw.csv"
    if not path.exists():
        return None
    frame = pd.read_csv(path)
    fig, ax = plt.subplots(figsize=(6.4, 4.2))
    for seed, group in frame.groupby("seed"):
        ax.plot(group["task_count"], group["gap_percent"], color="#B8C7D1", alpha=0.45, linewidth=0.8)
    grouped = frame.groupby("task_count")["gap_percent"]
    x = np.array(sorted(grouped.groups))
    means = np.array([_mean_ci(grouped.get_group(item))[0] for item in x])
    cis = np.array([_mean_ci(grouped.get_group(item))[1] for item in x])
    ax.errorbar(x, means, yerr=cis, color="#2878B5", marker="o", linewidth=2.0, capsize=4, label="Mean and 95% CI")
    ax.set_xlabel("Number of tasks")
    ax.set_ylabel("Heuristic optimality gap (%)")
    ax.set_xticks(x)
    ax.grid(axis="y", alpha=0.25)
    ax.legend(frameon=False)
    return _finish(fig, "exact_gap_vs_size.png")


def plot_e2() -> Path | None:
    path = FORMAL / "E2" / "raw.csv"
    if not path.exists():
        return None
    frame = pd.read_csv(path)
    variants = ["time_aware", "distance_oriented"]
    labels = ["Time-aware", "Distance-oriented"]
    fig, axes = plt.subplots(1, 2, figsize=(8.4, 3.8))
    for ax, metric, ylabel in zip(
        axes,
        ("weighted_mean_delay", "total_travel_time"),
        ("Weighted mean delay", "Total travel time"),
    ):
        stats = [_mean_ci(frame.loc[frame["variant"] == item, metric]) for item in variants]
        means = [item[0] for item in stats]
        errors = [item[1] for item in stats]
        ax.bar(labels, means, yerr=errors, capsize=4, color=[COLORS[item] for item in variants], width=0.65)
        ax.set_ylabel(ylabel)
        ax.grid(axis="y", alpha=0.25)
        ax.tick_params(axis="x", rotation=12)
    fig.tight_layout()
    return _finish(fig, "time_objective_tradeoff.png")


def plot_e2_v2() -> tuple[Path, Path] | None:
    path = FORMAL / "E2_v2" / "raw.csv"
    if not path.exists():
        return None
    frame = pd.read_csv(path)
    variants = ["time_aware", "mission_completion"]
    variant_labels = ["Time-aware", "Mission-completion"]
    x = np.arange(2)
    width = 0.34

    fig, axes = plt.subplots(1, 2, figsize=(9.2, 4.0))
    panels = (
        (
            axes[0],
            ["weighted_mean_delay", "high_priority_mean_delay"],
            ["Weighted mean", "High-priority mean"],
            "Response delay",
        ),
        (
            axes[1],
            ["total_travel_time", "makespan"],
            ["Total travel", "Makespan"],
            "Mission cost / completion time",
        ),
    )
    for ax, metrics, labels, ylabel in panels:
        for offset, (variant, variant_label) in enumerate(
            zip(variants, variant_labels)
        ):
            stats = [
                _mean_ci(frame.loc[frame["variant"] == variant, metric])
                for metric in metrics
            ]
            positions = x + (offset - 0.5) * width
            ax.bar(
                positions,
                [item[0] for item in stats],
                width,
                yerr=[item[1] for item in stats],
                capsize=4,
                color=COLORS.get(variant, "#4C78A8"),
                label=variant_label,
            )
        ax.set_xticks(x, labels)
        ax.set_ylabel(ylabel)
        ax.grid(axis="y", alpha=0.25)
    axes[0].legend(frameon=False)
    fig.tight_layout()
    main_path = _finish(fig, "time_objective_tradeoff_v2.png")

    variants_all = ["time_aware", "mission_completion", "distance_oriented"]
    labels_all = ["Time-aware", "Mission-completion", "Distance-oriented"]
    stats = [
        _mean_ci(frame.loc[frame["variant"] == variant, "active_uav_count"])
        for variant in variants_all
    ]
    fig, ax = plt.subplots(figsize=(6.8, 4.2))
    bars = ax.bar(
        labels_all,
        [item[0] for item in stats],
        yerr=[item[1] for item in stats],
        capsize=4,
        color=[COLORS.get(item, "#4C78A8") for item in variants_all],
        width=0.65,
    )
    for bar in bars:
        ax.text(
            bar.get_x() + bar.get_width() / 2,
            bar.get_height() + 0.06,
            f"{bar.get_height():.1f}",
            ha="center",
            va="bottom",
        )
    ax.set_ylabel("Active UAV count")
    ax.set_ylim(0.0, 3.5)
    ax.grid(axis="y", alpha=0.25)
    ax.tick_params(axis="x", rotation=10)
    resource_path = _finish(fig, "distance_objective_resource_usage.png")
    return main_path, resource_path


def plot_e3() -> Path | None:
    path = FORMAL / "E3" / "paired.csv"
    if not path.exists():
        return None
    frame = pd.read_csv(path)
    frame = frame[frame["metric"] == "weighted_mean_delay"]
    grouped = frame.groupby("turning_radius")["relative_improvement_percent"]
    radii = np.array(sorted(grouped.groups), dtype=float)
    means = np.array([_mean_ci(grouped.get_group(radius))[0] for radius in radii])
    cis = np.array([_mean_ci(grouped.get_group(radius))[1] for radius in radii])
    fig, ax = plt.subplots(figsize=(6.4, 4.2))
    ax.axhline(0.0, color="#666666", linewidth=0.9)
    ax.errorbar(radii, means, yerr=cis, color="#2878B5", marker="o", linewidth=2.0, capsize=4)
    ax.set_xlabel("Minimum turning radius R")
    ax.set_ylabel("Dubins-aware relative improvement (%)")
    ax.set_xticks(radii)
    ax.grid(axis="y", alpha=0.25)
    return _finish(fig, "dubins_radius_sensitivity.png")


def plot_e4() -> Path | None:
    path = FORMAL / "E4" / "raw.csv"
    if not path.exists():
        return None
    frame = pd.read_csv(path)
    variants = ["no_reorder", "full", "local"]
    labels = ["NoReorder", "Full", "Local"]
    fig, axes = plt.subplots(1, 2, figsize=(8.6, 3.8))
    for ax, metric, ylabel in zip(
        axes,
        ("weighted_mean_delay", "total_replanning_runtime"),
        ("Weighted mean delay", "Total replanning runtime (s)"),
    ):
        stats = [_mean_ci(frame.loc[frame["variant"] == item, metric]) for item in variants]
        ax.bar(
            labels,
            [item[0] for item in stats],
            yerr=[item[1] for item in stats],
            capsize=4,
            color=[COLORS[item] for item in variants],
            width=0.65,
        )
        ax.set_ylabel(ylabel)
        ax.grid(axis="y", alpha=0.25)
    fig.tight_layout()
    return _finish(fig, "replanning_quality_runtime.png")


def plot_e5() -> Path | None:
    path = FORMAL / "E5" / "raw.csv"
    if not path.exists():
        return None
    frame = pd.read_csv(path)
    scales = ["small", "medium", "large"]
    labels = ["Small", "Medium", "Large"]
    x = np.arange(len(scales))
    fig, ax = plt.subplots(figsize=(6.8, 4.3))
    for variant in ("no_reorder", "full", "local"):
        means, cis = [], []
        for scale in scales:
            stat = _mean_ci(frame.loc[(frame["variant"] == variant) & (frame["scale"] == scale), "runner_wall_runtime"])
            means.append(stat[0])
            cis.append(stat[1])
        ax.errorbar(x, means, yerr=cis, marker="o", capsize=3, label=variant.replace("_", " ").title(), color=COLORS[variant])
    ax.set_xticks(x, labels)
    ax.set_xlabel("Problem scale")
    ax.set_ylabel("Runner wall runtime (s)")
    ax.grid(axis="y", alpha=0.25)
    ax.legend(frameon=False)
    return _finish(fig, "scale_runtime.png")


def main() -> None:
    e2_v2_outputs = plot_e2_v2()
    e5_output = plot_e5()
    outputs = [plot_e1(), plot_e2(), plot_e3(), plot_e4(), e5_output]
    if e2_v2_outputs is not None:
        outputs.extend(e2_v2_outputs)
    for output in outputs:
        if output is not None:
            print(output)
    if e5_output is None:
        print("E5 raw.csv not found; scale_runtime.png intentionally not generated.")


if __name__ == "__main__":
    main()
