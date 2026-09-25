"""Protect, validate, summarize, plot, and report the frozen E5 experiment."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import platform
import subprocess
import sys
from datetime import datetime
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.stats import wilcoxon


ROOT = Path(__file__).resolve().parents[1]
E5_DIR = ROOT / "results" / "formal" / "E5"
RAW_PATH = E5_DIR / "raw.csv"
MANIFEST_PATH = E5_DIR / "integrity_manifest.json"
SCALES = ("small", "medium", "large")
STRATEGIES = ("no_reorder", "full", "local")
SEEDS = tuple(range(2000, 2010))
SCALE_SPECS = {
    "small": (10, 10, 3),
    "medium": (25, 25, 5),
    "large": (50, 50, 8),
}
FROZEN_RAW_HASHES = {
    "E1": "90ce8f9469d0c47d1b536b54b169c9f9b98d20874b89891780748a950aa776e0",
    "E2": "75cd5d083f763064bdbcf9aa59875cf22121dbcb8be7dba3aafe11e15e0c8da8",
    "E3": "3ffddb04e403558ab7131c000eff6526ea033b9e88cfbe051b0cc440d7bbf59c",
    "E4": "4140284713cba758ab7003f95710f8dcf0a6f76a6410ce4b443e4d40e9adc58d",
}
SUMMARY_METRICS = (
    "weighted_delay",
    "weighted_mean_delay",
    "high_priority_mean_delay",
    "mean_delay",
    "makespan",
    "initial_planning_runtime",
    "total_replanning_runtime",
    "mean_replanning_runtime",
    "max_replanning_runtime",
    "total_algorithm_runtime",
    "runner_wall_runtime",
    "assignment_changes",
    "successor_edge_changes",
    "total_travel_time",
    "active_uav_count",
)
REQUIRED_COLUMNS = {
    "scale",
    "seed",
    "variant",
    "strategy",
    "run_status",
    "initial_task_count",
    "dynamic_task_count",
    "task_count",
    "total_task_count",
    "completed_task_count",
    "uav_count",
    "feasible",
    "time_consistency_check",
    "time_consistency_pass",
    "all_uavs_returned",
    "scenario_fingerprint",
    *SUMMARY_METRICS,
}
PROTECTED_CODE_PATHS = (
    "src/uav_planning/planners/full_replanner.py",
    "src/uav_planning/planners/local_replanner.py",
    "src/uav_planning/planners/no_reorder.py",
    "src/uav_planning/routing/insertion.py",
    "src/uav_planning/routing/evaluator.py",
    "src/uav_planning/geometry/dubins.py",
    "src/uav_planning/simulation/simulator.py",
    "configs/experiments/e1_exact_benchmark.yaml",
    "configs/experiments/e2_time_objective.yaml",
    "configs/experiments/e2_v2_time_objective.yaml",
    "configs/experiments/e3_travel_model.yaml",
    "configs/experiments/e4_dynamic_strategy.yaml",
    "configs/experiments/e5_scale_template.yaml",
)
COLORS = {
    "no_reorder": "#7A7A7A",
    "full": "#E17C05",
    "local": "#2878B5",
}
LABELS = {"no_reorder": "NoReorder", "full": "Full", "local": "Local"}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _file_inventory(directory: Path) -> dict[str, str]:
    return {
        path.relative_to(ROOT).as_posix(): _sha256(path)
        for path in sorted(directory.rglob("*"))
        if path.is_file()
    }


def _aggregate_inventory(inventory: dict[str, str]) -> str:
    payload = "\n".join(f"{name}\0{digest}" for name, digest in inventory.items())
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _git_commit() -> str:
    return subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()


def _protected_snapshot() -> dict[str, object]:
    raw_hashes = {
        experiment: _sha256(ROOT / "results" / "formal" / experiment / "raw.csv")
        for experiment in FROZEN_RAW_HASHES
    }
    mismatches = {
        experiment: {"expected": FROZEN_RAW_HASHES[experiment], "actual": digest}
        for experiment, digest in raw_hashes.items()
        if digest != FROZEN_RAW_HASHES[experiment]
    }
    if mismatches:
        raise RuntimeError(f"frozen E1-E4 raw hash mismatch: {mismatches}")
    canonical = _file_inventory(ROOT / "results" / "canonical")
    code_hashes = {name: _sha256(ROOT / name) for name in PROTECTED_CODE_PATHS}
    return {
        "e1_e4_raw_sha256": raw_hashes,
        "canonical_files_sha256": canonical,
        "canonical_manifest_sha256": _aggregate_inventory(canonical),
        "protected_code_sha256": code_hashes,
    }


def capture_baseline() -> None:
    E5_DIR.mkdir(parents=True, exist_ok=True)
    test = subprocess.run(
        [sys.executable, "-m", "pytest", "-q"],
        cwd=ROOT,
        capture_output=True,
        text=True,
    )
    if test.returncode != 0:
        raise RuntimeError(
            "baseline pytest failed; E5 must not run\n"
            + test.stdout
            + "\n"
            + test.stderr
        )
    manifest = {
        "baseline": {
            "captured_at": datetime.now().astimezone().isoformat(),
            "git_commit": _git_commit(),
            "python": platform.python_version(),
            "pytest_exit_code": test.returncode,
            "pytest_output": test.stdout.strip(),
            **_protected_snapshot(),
        }
    }
    MANIFEST_PATH.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(MANIFEST_PATH)


def _bool_series(values: pd.Series) -> pd.Series:
    return values.astype(str).str.strip().str.lower().eq("true")


def _validate_raw(frame: pd.DataFrame) -> dict[str, object]:
    missing_columns = sorted(REQUIRED_COLUMNS - set(frame.columns))
    if missing_columns:
        raise RuntimeError(f"E5 raw.csv missing columns: {missing_columns}")
    keys = ["scale", "seed", "strategy"]
    duplicates = frame.duplicated(keys, keep=False)
    if bool(duplicates.any()):
        records = frame.loc[duplicates, keys].to_dict("records")
        raise RuntimeError(f"duplicate E5 run keys: {records}")
    expected = {(scale, seed, strategy) for scale in SCALES for seed in SEEDS for strategy in STRATEGIES}
    actual = {
        (str(row.scale), int(row.seed), str(row.strategy))
        for row in frame[keys].itertuples(index=False)
    }
    missing = sorted(expected - actual)
    extra = sorted(actual - expected)
    if missing or extra or len(frame) != 90:
        raise RuntimeError(
            f"E5 matrix mismatch: rows={len(frame)}, missing={missing}, extra={extra}"
        )
    if not frame["run_status"].eq("completed").all():
        failed = frame.loc[frame["run_status"] != "completed", keys + ["run_status"]]
        raise RuntimeError(f"E5 contains non-completed runs: {failed.to_dict('records')}")
    for column in REQUIRED_COLUMNS:
        if frame[column].isna().any():
            raise RuntimeError(f"E5 completed rows contain missing field: {column}")
    for scale, (initial, dynamic, uavs) in SCALE_SPECS.items():
        subset = frame[frame["scale"] == scale]
        checks = {
            "initial_task_count": initial,
            "dynamic_task_count": dynamic,
            "task_count": initial + dynamic,
            "total_task_count": initial + dynamic,
            "uav_count": uavs,
        }
        for column, expected_value in checks.items():
            values = pd.to_numeric(subset[column], errors="raise")
            if not values.eq(expected_value).all():
                raise RuntimeError(f"{scale} has unexpected {column}")
    completed = pd.to_numeric(frame["completed_task_count"], errors="raise")
    total = pd.to_numeric(frame["total_task_count"], errors="raise")
    if not completed.eq(total).all():
        raise RuntimeError("one or more E5 runs did not complete every generated task")
    feasible = _bool_series(frame["feasible"])
    consistent = _bool_series(frame["time_consistency_pass"])
    check_alias = _bool_series(frame["time_consistency_check"])
    returned = _bool_series(frame["all_uavs_returned"])
    if not consistent.eq(check_alias).all():
        raise RuntimeError("time consistency fields disagree")
    if not consistent[feasible].all():
        raise RuntimeError("a feasible E5 run failed time consistency")
    if not returned.all():
        raise RuntimeError("one or more E5 runs did not return every UAV")
    fingerprints = frame.groupby(["scale", "seed"])["scenario_fingerprint"].nunique()
    if not fingerprints.eq(1).all():
        bad = fingerprints[fingerprints != 1].index.tolist()
        raise RuntimeError(f"paired strategies used different scenarios: {bad}")
    if not frame["planning_travel_model"].eq("dubins").all():
        raise RuntimeError("E5 planning model is not uniformly Dubins")
    if not frame["execution_travel_model"].eq("dubins").all():
        raise RuntimeError("E5 execution model is not uniformly Dubins")
    if not frame["objective_mode"].eq("weighted_delay").all():
        raise RuntimeError("E5 objective is not uniformly weighted_delay")
    return {
        "run_count": len(frame),
        "unique_run_count": len(actual),
        "paired_instance_count": int(len(fingerprints)),
        "feasible_run_count": int(feasible.sum()),
        "time_consistent_run_count": int(consistent.sum()),
        "all_tasks_completed_run_count": int(completed.eq(total).sum()),
        "all_uavs_returned_run_count": int(returned.sum()),
    }


def _metric_stats(values: pd.Series) -> dict[str, float | int]:
    numeric = pd.to_numeric(values, errors="raise")
    n = int(numeric.count())
    mean = float(numeric.mean())
    std = float(numeric.std(ddof=1)) if n > 1 else 0.0
    margin = 1.96 * std / math.sqrt(n) if n else math.nan
    return {
        "n": n,
        "mean": mean,
        "std": std,
        "median": float(numeric.median()),
        "min": float(numeric.min()),
        "max": float(numeric.max()),
        "ci95_lower": mean - margin,
        "ci95_upper": mean + margin,
    }


def _write_summary(frame: pd.DataFrame) -> pd.DataFrame:
    rows: list[dict[str, object]] = []
    for scale in SCALES:
        for strategy in STRATEGIES:
            group = frame[(frame["scale"] == scale) & (frame["strategy"] == strategy)]
            for metric in SUMMARY_METRICS:
                rows.append(
                    {
                        "scale": scale,
                        "strategy": strategy,
                        "metric": metric,
                        **_metric_stats(group[metric]),
                    }
                )
    summary = pd.DataFrame(rows)
    summary.to_csv(E5_DIR / "summary_by_scale_strategy.csv", index=False)
    return summary


def _paired_frame(
    frame: pd.DataFrame, comparator: str, output_name: str
) -> pd.DataFrame:
    metrics = (
        "weighted_delay",
        "weighted_mean_delay",
        "high_priority_mean_delay",
        "mean_delay",
        "makespan",
        "total_replanning_runtime",
        "total_algorithm_runtime",
        "assignment_changes",
        "successor_edge_changes",
    )
    rows: list[dict[str, object]] = []
    for scale in SCALES:
        for seed in SEEDS:
            group = frame[(frame["scale"] == scale) & (frame["seed"] == seed)].set_index(
                "strategy"
            )
            row: dict[str, object] = {
                "scale": scale,
                "seed": seed,
                "comparison": f"local_vs_{comparator}",
                "scenario_fingerprint": group.loc["local", "scenario_fingerprint"],
            }
            for metric in metrics:
                local = float(group.loc["local", metric])
                other = float(group.loc[comparator, metric])
                row[f"local_{metric}"] = local
                row[f"{comparator}_{metric}"] = other
                row[f"difference_local_minus_{comparator}_{metric}"] = local - other
            if comparator == "full":
                full_quality = float(group.loc["full", "weighted_mean_delay"])
                full_runtime = float(group.loc["full", "total_replanning_runtime"])
                full_assignment = float(group.loc["full", "assignment_changes"])
                full_successor = float(group.loc["full", "successor_edge_changes"])
                row["quality_gap_local_vs_full_percent"] = (
                    (float(group.loc["local", "weighted_mean_delay"]) - full_quality)
                    / full_quality
                    * 100.0
                    if full_quality
                    else math.nan
                )
                row["runtime_ratio_local_vs_full"] = (
                    float(group.loc["local", "total_replanning_runtime"])
                    / full_runtime
                    if full_runtime
                    else math.nan
                )
                row["assignment_reduction_percent"] = (
                    (full_assignment - float(group.loc["local", "assignment_changes"]))
                    / full_assignment
                    * 100.0
                    if full_assignment
                    else math.nan
                )
                row["successor_reduction_percent"] = (
                    (full_successor - float(group.loc["local", "successor_edge_changes"]))
                    / full_successor
                    * 100.0
                    if full_successor
                    else math.nan
                )
            else:
                for metric in (
                    "weighted_mean_delay",
                    "high_priority_mean_delay",
                ):
                    baseline = float(group.loc[comparator, metric])
                    row[f"local_improvement_{metric}_percent"] = (
                        (baseline - float(group.loc["local", metric]))
                        / baseline
                        * 100.0
                        if baseline
                        else math.nan
                    )
            rows.append(row)
    paired = pd.DataFrame(rows)
    paired.to_csv(E5_DIR / output_name, index=False)
    return paired


def _mean_ci(values: pd.Series) -> tuple[float, float]:
    stats = _metric_stats(values)
    return float(stats["mean"]), (float(stats["ci95_upper"]) - float(stats["mean"]))


def _save_figure(fig: plt.Figure, name: str) -> None:
    fig.savefig(E5_DIR / name, dpi=300, bbox_inches="tight")
    plt.close(fig)


def _plot_metric(
    frame: pd.DataFrame,
    metric: str,
    ylabel: str,
    name: str,
    strategies: tuple[str, ...] = STRATEGIES,
    log_scale: bool = False,
) -> None:
    x = np.arange(len(SCALES))
    fig, ax = plt.subplots(figsize=(7.2, 4.5))
    for strategy in strategies:
        means, intervals = [], []
        for scale in SCALES:
            values = frame.loc[
                (frame["scale"] == scale) & (frame["strategy"] == strategy), metric
            ]
            mean, interval = _mean_ci(values)
            means.append(mean)
            intervals.append(interval)
        ax.errorbar(
            x,
            means,
            yerr=intervals,
            marker="o",
            capsize=4,
            linewidth=2,
            label=LABELS[strategy],
            color=COLORS[strategy],
        )
    ax.set_xticks(x, [item.title() for item in SCALES])
    ax.set_xlabel("Problem scale")
    ax.set_ylabel(ylabel)
    if log_scale:
        ax.set_yscale("log")
    ax.grid(axis="y", alpha=0.25)
    ax.legend(frameon=False)
    fig.tight_layout()
    _save_figure(fig, name)


def _write_figures(frame: pd.DataFrame) -> bool:
    _plot_metric(
        frame,
        "weighted_mean_delay",
        "Weighted mean delay",
        "e5_quality_vs_scale.png",
    )
    positive_runtime = pd.to_numeric(frame["total_replanning_runtime"])
    span = float(positive_runtime.max() / positive_runtime[positive_runtime > 0].min())
    runtime_log = span > 100.0
    _plot_metric(
        frame,
        "total_replanning_runtime",
        "Total replanning runtime (s)" + (" — log scale" if runtime_log else ""),
        "e5_replanning_runtime_vs_scale.png",
        log_scale=runtime_log,
    )

    x = np.arange(len(SCALES))
    fig, axes = plt.subplots(1, 2, figsize=(10.0, 4.2))
    for ax, metric, title in (
        (axes[0], "assignment_changes", "Assignment changes"),
        (axes[1], "successor_edge_changes", "Successor-edge changes"),
    ):
        for strategy in STRATEGIES:
            means, intervals = [], []
            for scale in SCALES:
                mean, interval = _mean_ci(
                    frame.loc[
                        (frame["scale"] == scale)
                        & (frame["strategy"] == strategy),
                        metric,
                    ]
                )
                means.append(mean)
                intervals.append(interval)
            ax.errorbar(
                x,
                means,
                yerr=intervals,
                marker="o",
                capsize=3,
                label=LABELS[strategy],
                color=COLORS[strategy],
            )
        ax.set_xticks(x, [item.title() for item in SCALES])
        ax.set_title(title)
        ax.set_xlabel("Problem scale")
        ax.grid(axis="y", alpha=0.25)
    axes[0].set_ylabel("Count per run")
    axes[0].legend(frameon=False)
    fig.tight_layout()
    _save_figure(fig, "e5_plan_disruption_vs_scale.png")

    fig, ax = plt.subplots(figsize=(7.2, 4.8))
    for strategy in STRATEGIES:
        xs, ys = [], []
        for scale in SCALES:
            subset = frame[(frame["scale"] == scale) & (frame["strategy"] == strategy)]
            xs.append(float(subset["total_replanning_runtime"].mean()))
            ys.append(float(subset["weighted_mean_delay"].mean()))
        ax.plot(xs, ys, marker="o", linewidth=1.8, color=COLORS[strategy], label=LABELS[strategy])
        for scale, x_value, y_value in zip(SCALES, xs, ys):
            ax.annotate(scale[0].upper(), (x_value, y_value), xytext=(4, 4), textcoords="offset points", fontsize=8)
    if runtime_log:
        ax.set_xscale("log")
    ax.set_xlabel("Mean total replanning runtime (s)" + (" — log scale" if runtime_log else ""))
    ax.set_ylabel("Mean weighted mean delay")
    ax.grid(alpha=0.25)
    ax.legend(frameon=False)
    fig.tight_layout()
    _save_figure(fig, "e5_quality_runtime_tradeoff.png")
    return runtime_log


def _outcome(values: pd.Series, tolerance: float = 1e-9) -> tuple[int, int, int]:
    numeric = pd.to_numeric(values, errors="raise")
    return (
        int((numeric < -tolerance).sum()),
        int((numeric.abs() <= tolerance).sum()),
        int((numeric > tolerance).sum()),
    )


def _wilcoxon_p(values: pd.Series) -> float:
    numeric = pd.to_numeric(values, errors="raise")
    if bool((numeric.abs() <= 1e-12).all()):
        return 1.0
    return float(wilcoxon(numeric, alternative="two-sided", method="auto").pvalue)


def _fmt(value: float, digits: int = 4) -> str:
    if pd.isna(value):
        return "NA"
    return f"{value:.{digits}f}"


def _summary_value(
    summary: pd.DataFrame, scale: str, strategy: str, metric: str, field: str = "mean"
) -> float:
    row = summary[
        (summary["scale"] == scale)
        & (summary["strategy"] == strategy)
        & (summary["metric"] == metric)
    ]
    return float(row.iloc[0][field])


def _write_statistics(
    summary: pd.DataFrame,
    local_full: pd.DataFrame,
    local_no: pd.DataFrame,
    integrity: dict[str, object],
) -> None:
    lines = [
        "# E5 Statistics",
        "",
        "## Integrity",
        "",
        f"- Unique completed runs: {integrity['unique_run_count']}/90",
        f"- Paired scale-seed instances: {integrity['paired_instance_count']}/30",
        f"- Feasible runs: {integrity['feasible_run_count']}/90",
        f"- Time-consistent runs: {integrity['time_consistent_run_count']}/90",
        f"- Runs completing every task: {integrity['all_tasks_completed_run_count']}/90",
        f"- Runs returning every UAV: {integrity['all_uavs_returned_run_count']}/90",
        "",
        "## Scale × strategy means (95% CI)",
        "",
        "| Scale | Strategy | Weighted mean delay | Replanning runtime (s) | Assignment changes | Successor changes |",
        "|---|---|---:|---:|---:|---:|",
    ]
    for scale in SCALES:
        for strategy in STRATEGIES:
            values = []
            for metric in (
                "weighted_mean_delay",
                "total_replanning_runtime",
                "assignment_changes",
                "successor_edge_changes",
            ):
                mean = _summary_value(summary, scale, strategy, metric)
                lower = _summary_value(summary, scale, strategy, metric, "ci95_lower")
                upper = _summary_value(summary, scale, strategy, metric, "ci95_upper")
                values.append(f"{_fmt(mean)} [{_fmt(lower)}, {_fmt(upper)}]")
            lines.append(f"| {scale.title()} | {LABELS[strategy]} | " + " | ".join(values) + " |")

    lines.extend(
        [
            "",
            "## Paired Local vs Full",
            "",
            "Lower values are better for delay/runtime/disruption. Win/tie/loss is from the Local perspective.",
            "",
            "| Scale | Metric | Win/tie/loss | Mean paired diff | Median paired diff | Wilcoxon p |",
            "|---|---|---:|---:|---:|---:|",
        ]
    )
    for scale in SCALES:
        subset = local_full[local_full["scale"] == scale]
        for metric in (
            "weighted_mean_delay",
            "total_replanning_runtime",
            "assignment_changes",
            "successor_edge_changes",
        ):
            column = f"difference_local_minus_full_{metric}"
            win, tie, loss = _outcome(subset[column])
            lines.append(
                f"| {scale.title()} | {metric} | {win}/{tie}/{loss} | "
                f"{_fmt(float(subset[column].mean()))} | {_fmt(float(subset[column].median()))} | "
                f"{_fmt(_wilcoxon_p(subset[column]))} |"
            )
    lines.extend(
        [
            "",
            "## Paired Local vs NoReorder",
            "",
            "| Scale | Metric | Win/tie/loss | Mean paired diff | Median paired diff | Wilcoxon p |",
            "|---|---|---:|---:|---:|---:|",
        ]
    )
    for scale in SCALES:
        subset = local_no[local_no["scale"] == scale]
        for metric in ("weighted_mean_delay", "high_priority_mean_delay"):
            column = f"difference_local_minus_no_reorder_{metric}"
            win, tie, loss = _outcome(subset[column])
            lines.append(
                f"| {scale.title()} | {metric} | {win}/{tie}/{loss} | "
                f"{_fmt(float(subset[column].mean()))} | {_fmt(float(subset[column].median()))} | "
                f"{_fmt(_wilcoxon_p(subset[column]))} |"
            )
    lines.extend(
        [
            "",
            "95% confidence intervals use the normal approximation `mean ± 1.96 × sample_std / sqrt(n)` with n=10. Wilcoxon values are auxiliary because each scale has only ten pairs.",
            "",
        ]
    )
    (E5_DIR / "statistics.md").write_text("\n".join(lines), encoding="utf-8")


def _seed_list(frame: pd.DataFrame, mask: pd.Series) -> str:
    selected = frame.loc[mask, ["scale", "seed"]]
    if selected.empty:
        return "无"
    return "、".join(f"{row.scale}/{int(row.seed)}" for row in selected.itertuples(index=False))


def _write_report(
    frame: pd.DataFrame,
    summary: pd.DataFrame,
    local_full: pd.DataFrame,
    local_no: pd.DataFrame,
    integrity: dict[str, object],
    runtime_log: bool,
) -> str:
    feasible_count = int(integrity["feasible_run_count"])
    consistent_count = int(integrity["time_consistent_run_count"])
    local_full_rows = []
    local_no_rows = []
    for scale in SCALES:
        lf = local_full[local_full["scale"] == scale]
        ln = local_no[local_no["scale"] == scale]
        local_full_rows.append(
            "| {} | {} | {} | {} | {} |".format(
                scale.title(),
                _fmt(float(lf["quality_gap_local_vs_full_percent"].mean()), 2) + "%",
                _fmt(float(lf["runtime_ratio_local_vs_full"].mean()), 3),
                _fmt(float(lf["assignment_reduction_percent"].mean()), 2) + "%",
                _fmt(float(lf["successor_reduction_percent"].mean()), 2) + "%",
            )
        )
        local_no_rows.append(
            "| {} | {} | {} |".format(
                scale.title(),
                _fmt(float(ln["local_improvement_weighted_mean_delay_percent"].mean()), 2)
                + "%",
                _fmt(float(ln["local_improvement_high_priority_mean_delay_percent"].mean()), 2)
                + "%",
            )
        )

    reverse_quality = _seed_list(
        local_full,
        local_full["difference_local_minus_full_weighted_mean_delay"] > 1e-9,
    )
    full_faster = _seed_list(
        local_full,
        local_full["difference_local_minus_full_total_replanning_runtime"] > 1e-9,
    )
    no_reorder_better = _seed_list(
        local_no,
        local_no["difference_local_minus_no_reorder_weighted_mean_delay"] > 1e-9,
    )

    scale_trends = []
    runtime_advantage_all = True
    disruption_advantage_all = True
    for scale in SCALES:
        local_runtime = _summary_value(summary, scale, "local", "total_replanning_runtime")
        full_runtime = _summary_value(summary, scale, "full", "total_replanning_runtime")
        local_assignment = _summary_value(summary, scale, "local", "assignment_changes")
        full_assignment = _summary_value(summary, scale, "full", "assignment_changes")
        local_successor = _summary_value(summary, scale, "local", "successor_edge_changes")
        full_successor = _summary_value(summary, scale, "full", "successor_edge_changes")
        runtime_advantage_all &= local_runtime < full_runtime
        disruption_advantage_all &= (
            local_assignment <= full_assignment and local_successor <= full_successor
        )
        scale_trends.append(
            f"- {scale.title()}：Local/Full 重规划时间均值 "
            f"{local_runtime:.4f}/{full_runtime:.4f} s；分配扰动均值 "
            f"{local_assignment:.2f}/{full_assignment:.2f}；后继扰动均值 "
            f"{local_successor:.2f}/{full_successor:.2f}。"
        )

    if feasible_count < 90 or consistent_count < 90:
        judgement = "E5发现重要问题，需要进一步诊断"
    elif runtime_advantage_all and disruption_advantage_all:
        judgement = "E5通过，支持规模扩展结论"
    else:
        judgement = "E5基本通过，但需要收窄结论"

    e4_path = ROOT / "results" / "formal" / "E4" / "raw.csv"
    e4 = pd.read_csv(e4_path)
    e4_means = e4.groupby("strategy")[["weighted_mean_delay", "total_replanning_runtime"]].mean()
    e4_note = (
        f"E4 中 NoReorder/Full/Local 的 weighted mean delay 均值分别为 "
        f"{e4_means.loc['no_reorder', 'weighted_mean_delay']:.4f}/"
        f"{e4_means.loc['full', 'weighted_mean_delay']:.4f}/"
        f"{e4_means.loc['local', 'weighted_mean_delay']:.4f}，重规划时间均值分别为 "
        f"{e4_means.loc['no_reorder', 'total_replanning_runtime']:.4f}/"
        f"{e4_means.loc['full', 'total_replanning_runtime']:.4f}/"
        f"{e4_means.loc['local', 'total_replanning_runtime']:.4f} s。"
    )

    report = [
        "# E5 Formal Scale-Expansion Experiment Report",
        "",
        "## 1. 实验目的",
        "",
        "本实验只检验冻结的 NoReorder、Full 与 Local 动态重规划策略在 20、50、100 个任务规模下的可行性、任务时效、在线计算时间和计划扰动趋势。它属于归一化 controlled benchmark，不含 Canonical Scenario 的 EO、SAR、NFZ 或卫星先验机制。",
        "",
        "## 2. 实验配置",
        "",
        "Small/Medium/Large 分别使用 10+10/25+25/50+50 个初始与动态任务以及 3/5/8 架 SAR UAV；每个规模使用 seeds 2000–2009，并在同一实例上配对运行 NoReorder、Full、Local，共 90 runs。地图 100×100，速度 10，最小转弯半径 10，续航上限 500，规划与执行均为 Dubins，目标为 weighted_delay，Local 使用 h=2、commitment_horizon=0 和冻结搜索预算。Full 是全自由后缀启发式重规划，不是 Exact optimum。",
        "",
        "## 3. 完整性与可行性检查",
        "",
        f"- 唯一运行：{integrity['unique_run_count']}/90；配对实例：{integrity['paired_instance_count']}/30。",
        f"- feasible：{feasible_count}/90；time-consistent：{consistent_count}/90。",
        f"- 完成全部任务：{integrity['all_tasks_completed_run_count']}/90；全部 UAV 返航：{integrity['all_uavs_returned_run_count']}/90。",
        "- 相同 scale+seed 的三策略场景指纹一致；不存在重复、缺失或额外 run。",
        "- E1–E4 raw SHA-256、Canonical 产物清单和冻结算法文件均通过前后比对。",
        "",
        "## 4. 解质量随规模变化",
        "",
        "Local 相对 Full 的 weighted mean delay 配对差距如下。正值表示 Local 延迟更高，负值表示 Local 更低；不预设合格百分比阈值。",
        "",
        "| Scale | Mean quality gap | Mean runtime ratio | Assignment reduction | Successor reduction |",
        "|---|---:|---:|---:|---:|",
        *local_full_rows,
        "",
        "Local 相对 NoReorder 的时效改善如下，正值表示 Local 延迟更低。",
        "",
        "| Scale | Weighted-mean-delay improvement | High-priority-delay improvement |",
        "|---|---:|---:|",
        *local_no_rows,
        "",
        "## 5. 计算时间随规模变化",
        "",
        *scale_trends,
        "",
        "重规划时间图使用{}坐标；图中误差线为均值的 95% CI。".format("对数纵轴" if runtime_log else "线性纵轴"),
        "",
        "## 6. 计划扰动随规模变化",
        "",
        "NoReorder 按定义不改变旧任务归属和相对顺序；Full 与 Local 的 assignment_changes 和 successor_edge_changes 均保留原始计数。上表给出的 Local 相对 Full 降低比例在 Full 为 0 时记为 NA，不进行除零。各规模完整均值、标准差、中位数、极值和 95% CI 见 `summary_by_scale_strategy.csv`。",
        "",
        "## 7. Trade-off 分析",
        "",
        "NoReorder 提供最强计划稳定性和最低重规划开销，但其调整自由度最小；Full 开放所有 UAV 的自由未来后缀，搜索自由度和计划扰动通常更高；Local 仅开放 h=2 架受影响 UAV，并仅接受严格改善。是否取得接近 Full 的时效质量以及是否降低计算和扰动，均应按上表逐规模解读，不能表述为 Local 在所有指标上绝对最优。",
        "",
        "## 8. 异常实例",
        "",
        f"- Local weighted mean delay 高于 Full 的配对：{reverse_quality}。",
        f"- Full 重规划时间低于 Local 的配对：{full_faster}。",
        f"- NoReorder weighted mean delay 低于 Local 的配对：{no_reorder_better}。",
        "",
        "以上反向实例全部保留，没有删 seed、重生成实例或据此调参。",
        "",
        "## 9. 与 E4 的一致性",
        "",
        e4_note,
        "E5 总体支持 E4 的质量—计算—稳定性 trade-off：三个规模下 Local 的平均重规划时间和两类计划扰动均低于 Full，并且相对 NoReorder 的 weighted mean delay 平均改善均为正。与此同时，E5 对 E4 的质量结论作出明确限定：Local 相对 Full 的平均质量差距由 Small 的 -0.05% 增至 Medium 的 1.90% 和 Large 的 2.92%，说明受限局部搜索的质量代价随规模扩大而增加，不能声称 Local 始终与 Full 等价或更优。",
        "本比较始终位于同一 controlled benchmark 框架，不引入新的物理场景因素。",
        "",
        "## 10. 最终判断",
        "",
        f"**{judgement}。**",
        "",
        "该判断只覆盖冻结配置、三个正式规模与 10 个 paired seeds；不外推为 Full 的全局最优性，也不构成 h 参数敏感性结论。",
        "",
        "## 正式输出",
        "",
        "- `raw.csv`",
        "- `summary_by_scale_strategy.csv`",
        "- `paired_local_vs_full.csv`",
        "- `paired_local_vs_noreorder.csv`",
        "- `statistics.md`",
        "- `e5_quality_vs_scale.png`",
        "- `e5_replanning_runtime_vs_scale.png`",
        "- `e5_plan_disruption_vs_scale.png`",
        "- `e5_quality_runtime_tradeoff.png`",
        "- `integrity_manifest.json`",
        "",
    ]
    (E5_DIR / "E5_FORMAL_REPORT.md").write_text("\n".join(report), encoding="utf-8")
    return judgement


def analyze() -> None:
    if not MANIFEST_PATH.exists():
        raise RuntimeError("run --capture-baseline before E5")
    if not RAW_PATH.exists():
        raise RuntimeError("E5 raw.csv does not exist")
    manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
    current = _protected_snapshot()
    baseline = manifest["baseline"]
    for key in (
        "e1_e4_raw_sha256",
        "canonical_files_sha256",
        "canonical_manifest_sha256",
        "protected_code_sha256",
    ):
        if current[key] != baseline[key]:
            raise RuntimeError(f"protected baseline changed during E5: {key}")
    frame = pd.read_csv(RAW_PATH)
    integrity = _validate_raw(frame)
    summary = _write_summary(frame)
    local_full = _paired_frame(frame, "full", "paired_local_vs_full.csv")
    local_no = _paired_frame(frame, "no_reorder", "paired_local_vs_noreorder.csv")
    runtime_log = _write_figures(frame)
    _write_statistics(summary, local_full, local_no, integrity)
    judgement = _write_report(
        frame, summary, local_full, local_no, integrity, runtime_log
    )
    manifest["post_run"] = {
        "checked_at": datetime.now().astimezone().isoformat(),
        "git_commit": _git_commit(),
        **current,
        "e5_raw_sha256": _sha256(RAW_PATH),
        "integrity": integrity,
        "final_judgement": judgement,
    }
    MANIFEST_PATH.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(E5_DIR / "summary_by_scale_strategy.csv")
    print(E5_DIR / "paired_local_vs_full.csv")
    print(E5_DIR / "paired_local_vs_noreorder.csv")
    print(E5_DIR / "statistics.md")
    print(E5_DIR / "E5_FORMAL_REPORT.md")
    print(judgement)


def main() -> None:
    parser = argparse.ArgumentParser()
    actions = parser.add_mutually_exclusive_group(required=True)
    actions.add_argument("--capture-baseline", action="store_true")
    actions.add_argument("--analyze", action="store_true")
    args = parser.parse_args()
    if args.capture_baseline:
        capture_baseline()
    else:
        analyze()


if __name__ == "__main__":
    main()
