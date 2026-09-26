"""Run and analyze the frozen Local affected-UAV-count sensitivity study."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import platform
import subprocess
import sys
from dataclasses import replace
from datetime import datetime
from pathlib import Path
from time import perf_counter

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.stats import wilcoxon


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from uav_planning.config import load_config
from uav_planning.planners import InitialPlanner, LocalReplanner
from uav_planning.routing.evaluator import RouteEvaluator
from uav_planning.scenario.generator import ScenarioGenerator
from uav_planning.simulation.simulator import Simulator

from run_formal_experiments import _optimizer, _provider, _scenario_fingerprint


OUTPUT_DIR = ROOT / "results" / "formal" / "H_SENSITIVITY"
RAW_PATH = OUTPUT_DIR / "raw.csv"
MANIFEST_PATH = OUTPUT_DIR / "integrity_manifest.json"
E5_RAW_PATH = ROOT / "results" / "formal" / "E5" / "raw.csv"
E5_CONFIG_PATH = ROOT / "configs" / "experiments" / "e5_scale_template.yaml"
H_VALUES = (1, 2, 3)
SEEDS = tuple(range(2000, 2010))
FROZEN_RAW_HASHES = {
    "E1": "90ce8f9469d0c47d1b536b54b169c9f9b98d20874b89891780748a950aa776e0",
    "E2": "75cd5d083f763064bdbcf9aa59875cf22121dbcb8be7dba3aafe11e15e0c8da8",
    "E2_v2": "4a62685b89742fb61bbc8cdd841006212a3fb4dde5fd564c52ff0ff12d2e4802",
    "E3": "3ffddb04e403558ab7131c000eff6526ea033b9e88cfbe051b0cc440d7bbf59c",
    "E4": "4140284713cba758ab7003f95710f8dcf0a6f76a6410ce4b443e4d40e9adc58d",
    "E5": "de4e8cc254564ae4e6187307783cd0a713827d234a6f4f6d12ce3b6cbd28e7fb",
}
PROTECTED_CODE_PATHS = (
    "src/uav_planning/planners/local_replanner.py",
    "src/uav_planning/planners/no_reorder.py",
    "src/uav_planning/planners/full_replanner.py",
    "src/uav_planning/routing/insertion.py",
    "src/uav_planning/routing/evaluator.py",
    "src/uav_planning/geometry/dubins.py",
    "src/uav_planning/simulation/simulator.py",
    "configs/experiments/e5_scale_template.yaml",
)
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
)
PAIR_METRICS = (
    "weighted_mean_delay",
    "high_priority_mean_delay",
    "mean_delay",
    "makespan",
    "total_replanning_runtime",
    "mean_replanning_runtime",
    "max_replanning_runtime",
    "assignment_changes",
    "successor_edge_changes",
)
REQUIRED_COMPLETION_FIELDS = (
    "run_status",
    "runner_wall_runtime",
    "total_algorithm_runtime",
    "feasible",
    "time_consistency_pass",
    "completed_task_count",
    "total_task_count",
    "all_uavs_returned",
    "scenario_fingerprint",
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _inventory(directory: Path) -> dict[str, str]:
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
        capture_output=True,
        text=True,
        check=True,
    ).stdout.strip()


def _protected_snapshot() -> dict[str, object]:
    raw_hashes = {
        name: _sha256(ROOT / "results" / "formal" / name / "raw.csv")
        for name in FROZEN_RAW_HASHES
    }
    mismatches = {
        name: {"expected": FROZEN_RAW_HASHES[name], "actual": digest}
        for name, digest in raw_hashes.items()
        if digest != FROZEN_RAW_HASHES[name]
    }
    if mismatches:
        raise RuntimeError(f"frozen E1-E5 raw hash mismatch: {mismatches}")
    canonical = _inventory(ROOT / "results" / "canonical")
    return {
        "formal_raw_sha256": raw_hashes,
        "canonical_files_sha256": canonical,
        "canonical_manifest_sha256": _aggregate_inventory(canonical),
        "protected_code_sha256": {
            name: _sha256(ROOT / name) for name in PROTECTED_CODE_PATHS
        },
    }


def capture_baseline() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    test = subprocess.run(
        [sys.executable, "-m", "pytest", "-q"],
        cwd=ROOT,
        capture_output=True,
        text=True,
    )
    if test.returncode != 0:
        raise RuntimeError("baseline pytest failed; sensitivity run is blocked\n" + test.stdout + test.stderr)
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


def _read_csv_strings(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as stream:
        return [dict(row) for row in csv.DictReader(stream)]


def _key(row: dict[str, object]) -> tuple[str, str]:
    return str(row["seed"]), str(row["h"])


def _upsert(row: dict[str, object]) -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    existing: list[dict[str, object]] = []
    fieldnames = list(row)
    if RAW_PATH.exists() and RAW_PATH.stat().st_size:
        with RAW_PATH.open("r", newline="", encoding="utf-8") as stream:
            reader = csv.DictReader(stream)
            existing = [dict(item) for item in reader]
            fieldnames = list(dict.fromkeys([*(reader.fieldnames or ()), *fieldnames]))
    retained = [item for item in existing if _key(item) != _key(row)]
    temporary = RAW_PATH.with_suffix(".csv.tmp")
    with temporary.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for item in [*retained, row]:
            writer.writerow({field: item.get(field, "") for field in fieldnames})
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(RAW_PATH)


def _completed_keys() -> set[tuple[str, str]]:
    if not RAW_PATH.exists():
        return set()
    rows = _read_csv_strings(RAW_PATH)
    return {
        _key(row)
        for row in rows
        if row.get("run_status") == "completed"
        and all(str(row.get(field, "")).strip() for field in REQUIRED_COMPLETION_FIELDS)
    }


def _e5_medium_rows() -> dict[tuple[int, str], dict[str, str]]:
    selected: dict[tuple[int, str], dict[str, str]] = {}
    for row in _read_csv_strings(E5_RAW_PATH):
        if row["scale"] == "medium":
            selected[(int(row["seed"]), row["strategy"])] = row
    expected = {(seed, strategy) for seed in SEEDS for strategy in ("no_reorder", "full", "local")}
    if set(selected) != expected:
        raise RuntimeError("E5 Medium reference matrix is incomplete")
    return selected


def _reuse_h2(seed: int, e5_rows: dict[tuple[int, str], dict[str, str]]) -> dict[str, object]:
    source = dict(e5_rows[(seed, "local")])
    source.update(
        {
            "experiment": "H_SENSITIVITY",
            "variant": "local_h2",
            "h": 2,
            "source": "E5_medium_reuse",
        }
    )
    return source


def _run_one(seed: int, h: int) -> dict[str, object]:
    wall_started = perf_counter()
    base = load_config(E5_CONFIG_PATH)
    config = replace(
        base,
        seed=seed,
        uav=replace(base.uav, count=5),
        tasks=replace(base.tasks, initial_count=25, dynamic_count=25),
        planner=replace(base.planner, affected_uav_count_h=h),
    )
    scenario = ScenarioGenerator(config).generate()
    optimizer = _optimizer(config)
    result = Simulator(
        scenario,
        InitialPlanner(optimizer),
        optimizer,
        execution_evaluator=RouteEvaluator(_provider(config.execution_travel_model)),
        high_priority_threshold=config.high_priority_threshold,
    ).run(LocalReplanner(optimizer, h=h))
    returned_uavs = {
        int(event["uav_id"])
        for event in result.event_log
        if event["event"] == "RETURN_DEPOT" and "uav_id" in event
    }
    row: dict[str, object] = {
        "experiment": "H_SENSITIVITY",
        "variant": f"local_h{h}",
        "scale": "medium",
        "h": h,
        "source": "new_run",
    }
    row.update(result.metrics)
    row.update(
        {
            "runner_wall_runtime": perf_counter() - wall_started,
            "scenario_fingerprint": _scenario_fingerprint(scenario),
            "run_status": "completed",
            "initial_task_count": 25,
            "dynamic_task_count": 25,
            "total_task_count": len(scenario.all_tasks),
            "completed_task_count": len(result.history),
            "all_uavs_returned": len(returned_uavs) == len(scenario.uavs),
            "time_consistency_pass": bool(result.metrics["time_consistency_check"]),
            "error_type": "",
            "error_message": "",
        }
    )
    return row


def run_matrix(resume: bool) -> None:
    if not MANIFEST_PATH.exists():
        raise RuntimeError("capture the baseline before running sensitivity cases")
    if RAW_PATH.exists() and not resume:
        raise RuntimeError("raw.csv already exists; use --resume instead of overwriting")
    completed = _completed_keys() if resume else set()
    e5_rows = _e5_medium_rows()
    specs = [(seed, h) for h in H_VALUES for seed in SEEDS]
    pending = [(seed, h) for seed, h in specs if (str(seed), str(h)) not in completed]
    print(f"H_SENSITIVITY: total=30, completed={len(completed)}, pending={len(pending)}")
    for index, (seed, h) in enumerate(pending, start=1):
        started = perf_counter()
        try:
            row = _reuse_h2(seed, e5_rows) if h == 2 else _run_one(seed, h)
        except Exception as error:
            row = {
                "experiment": "H_SENSITIVITY",
                "variant": f"local_h{h}",
                "scale": "medium",
                "seed": seed,
                "strategy": "local",
                "h": h,
                "source": "new_run" if h != 2 else "E5_medium_reuse",
                "run_status": "error",
                "runner_wall_runtime": perf_counter() - started,
                "error_type": type(error).__name__,
                "error_message": str(error),
            }
        _upsert(row)
        print(f"[{index}/{len(pending)}] saved seed={seed}, h={h}, status={row['run_status']}")
    rows = _read_csv_strings(RAW_PATH)
    failed = [row for row in rows if row.get("run_status") != "completed"]
    if failed:
        raise RuntimeError("sensitivity matrix has error checkpoints; rerun with --resume")
    print(RAW_PATH)


def _bool_series(values: pd.Series) -> pd.Series:
    return values.astype(str).str.lower().eq("true")


def _validate(frame: pd.DataFrame) -> dict[str, object]:
    required = {
        "seed",
        "h",
        "source",
        "run_status",
        "scenario_fingerprint",
        "feasible",
        "time_consistency_pass",
        "completed_task_count",
        "total_task_count",
        "all_uavs_returned",
        *SUMMARY_METRICS,
    }
    missing_columns = sorted(required - set(frame.columns))
    if missing_columns:
        raise RuntimeError(f"raw.csv missing columns: {missing_columns}")
    if len(frame) != 30 or frame.duplicated(["seed", "h"]).any():
        raise RuntimeError("sensitivity raw matrix must contain 30 unique seed+h rows")
    actual = {(int(row.seed), int(row.h)) for row in frame[["seed", "h"]].itertuples(index=False)}
    expected = {(seed, h) for seed in SEEDS for h in H_VALUES}
    if actual != expected:
        raise RuntimeError(f"sensitivity matrix mismatch: missing={sorted(expected-actual)}, extra={sorted(actual-expected)}")
    if not frame["run_status"].eq("completed").all():
        raise RuntimeError("one or more sensitivity runs are not completed")
    feasible = _bool_series(frame["feasible"])
    consistent = _bool_series(frame["time_consistency_pass"])
    returned = _bool_series(frame["all_uavs_returned"])
    completed = pd.to_numeric(frame["completed_task_count"], errors="raise")
    total = pd.to_numeric(frame["total_task_count"], errors="raise")
    if not feasible.all():
        raise RuntimeError("one or more sensitivity runs are infeasible")
    if not consistent.all():
        raise RuntimeError("one or more sensitivity runs failed time consistency")
    if not completed.eq(total).all():
        raise RuntimeError("one or more sensitivity runs did not complete all tasks")
    if not returned.all():
        raise RuntimeError("one or more sensitivity runs did not return all UAVs")
    fingerprints = frame.groupby("seed")["scenario_fingerprint"].nunique()
    if not fingerprints.eq(1).all():
        raise RuntimeError("h variants used different paired scenarios")
    if not frame["task_count"].eq(50).all() or not frame["uav_count"].eq(5).all():
        raise RuntimeError("sensitivity run departed from E5 Medium scale")
    if not frame["planning_travel_model"].eq("dubins").all() or not frame["execution_travel_model"].eq("dubins").all():
        raise RuntimeError("sensitivity run departed from Dubins planning/execution")
    if not frame["objective_mode"].eq("weighted_delay").all():
        raise RuntimeError("sensitivity run departed from weighted_delay objective")
    return {
        "run_count": len(frame),
        "unique_run_count": len(actual),
        "paired_seed_count": int(len(fingerprints)),
        "feasible_run_count": int(feasible.sum()),
        "time_consistent_run_count": int(consistent.sum()),
        "all_tasks_completed_run_count": int(completed.eq(total).sum()),
        "all_uavs_returned_run_count": int(returned.sum()),
        "new_run_count": int(frame["source"].eq("new_run").sum()),
        "reused_h2_count": int(frame["source"].eq("E5_medium_reuse").sum()),
    }


def _validate_h2_exact_reuse() -> None:
    h_rows = {(int(row["seed"]), int(row["h"])): row for row in _read_csv_strings(RAW_PATH)}
    e5_rows = _e5_medium_rows()
    ignored = {"experiment", "variant", "h", "source"}
    for seed in SEEDS:
        h2 = h_rows[(seed, 2)]
        e5 = e5_rows[(seed, "local")]
        for column, expected in e5.items():
            if column not in ignored and h2.get(column, "") != expected:
                raise RuntimeError(f"reused h=2 row differs from E5 Medium at seed={seed}, column={column}")


def _stats(values: pd.Series) -> dict[str, float | int]:
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


def _write_summary(frame: pd.DataFrame, e5: pd.DataFrame) -> pd.DataFrame:
    groups: list[tuple[str, str, int | None, pd.DataFrame]] = [
        ("NoReorder", "reference", None, e5[e5["strategy"] == "no_reorder"]),
        ("Local h=1", "local", 1, frame[frame["h"] == 1]),
        ("Local h=2", "local", 2, frame[frame["h"] == 2]),
        ("Local h=3", "local", 3, frame[frame["h"] == 3]),
        ("Full", "reference", None, e5[e5["strategy"] == "full"]),
    ]
    rows: list[dict[str, object]] = []
    for method, method_type, h, group in groups:
        for metric in SUMMARY_METRICS:
            rows.append(
                {
                    "method": method,
                    "method_type": method_type,
                    "h": h,
                    "metric": metric,
                    **_stats(group[metric]),
                }
            )
    summary = pd.DataFrame(rows)
    summary.to_csv(OUTPUT_DIR / "summary.csv", index=False)
    return summary


def _wilcoxon(values: pd.Series) -> float:
    numeric = pd.to_numeric(values, errors="raise")
    if (numeric.abs() <= 1e-12).all():
        return 1.0
    return float(wilcoxon(numeric, alternative="two-sided", method="auto").pvalue)


def _write_paired(frame: pd.DataFrame) -> pd.DataFrame:
    comparisons = ((1, 2), (2, 3), (1, 3))
    rows: list[dict[str, object]] = []
    for h_a, h_b in comparisons:
        left = frame[frame["h"] == h_a].set_index("seed")
        right = frame[frame["h"] == h_b].set_index("seed")
        for metric in PAIR_METRICS:
            differences = pd.to_numeric(left[metric]) - pd.to_numeric(right[metric])
            tolerance = 1e-9
            diff_stats = _stats(differences)
            rows.append(
                {
                    "comparison": f"h{h_a}_vs_h{h_b}",
                    "h_A": h_a,
                    "h_B": h_b,
                    "metric": metric,
                    "n": len(differences),
                    "mean_A": float(pd.to_numeric(left[metric]).mean()),
                    "mean_B": float(pd.to_numeric(right[metric]).mean()),
                    "paired_mean_difference_A_minus_B": diff_stats["mean"],
                    "paired_median_difference_A_minus_B": diff_stats["median"],
                    "difference_ci95_lower": diff_stats["ci95_lower"],
                    "difference_ci95_upper": diff_stats["ci95_upper"],
                    "wins_A_lower": int((differences < -tolerance).sum()),
                    "ties": int((differences.abs() <= tolerance).sum()),
                    "losses_A_higher": int((differences > tolerance).sum()),
                    "wilcoxon_p_two_sided": _wilcoxon(differences),
                }
            )
    paired = pd.DataFrame(rows)
    paired.to_csv(OUTPUT_DIR / "paired_comparisons.csv", index=False)
    return paired


def _mean(summary: pd.DataFrame, method: str, metric: str) -> float:
    row = summary[(summary["method"] == method) & (summary["metric"] == metric)]
    return float(row.iloc[0]["mean"])


def _mean_ci(frame: pd.DataFrame, h: int, metric: str) -> tuple[float, float]:
    result = _stats(frame.loc[frame["h"] == h, metric])
    return float(result["mean"]), float(result["ci95_upper"]) - float(result["mean"])


def _save(fig: plt.Figure, name: str) -> None:
    fig.savefig(OUTPUT_DIR / name, dpi=300, bbox_inches="tight")
    plt.close(fig)


def _write_figures(frame: pd.DataFrame, summary: pd.DataFrame) -> None:
    x = np.array(H_VALUES)
    means, errors = zip(*[_mean_ci(frame, h, "weighted_mean_delay") for h in H_VALUES])
    fig, ax = plt.subplots(figsize=(7.0, 4.5))
    ax.errorbar(x, means, yerr=errors, marker="o", capsize=4, linewidth=2, color="#2878B5", label="Local")
    ax.axhline(_mean(summary, "NoReorder", "weighted_mean_delay"), color="#7A7A7A", linestyle="--", label="NoReorder reference")
    ax.axhline(_mean(summary, "Full", "weighted_mean_delay"), color="#E17C05", linestyle="--", label="Full reference")
    ax.set_xticks(x)
    ax.set_xlabel("Affected UAV count h")
    ax.set_ylabel("Weighted mean delay")
    ax.grid(axis="y", alpha=0.25)
    ax.legend(frameon=False)
    fig.tight_layout()
    _save(fig, "h_sensitivity_quality.png")

    means, errors = zip(*[_mean_ci(frame, h, "total_replanning_runtime") for h in H_VALUES])
    fig, ax = plt.subplots(figsize=(7.0, 4.5))
    ax.errorbar(x, means, yerr=errors, marker="o", capsize=4, linewidth=2, color="#2878B5", label="Local")
    ax.axhline(_mean(summary, "Full", "total_replanning_runtime"), color="#E17C05", linestyle="--", label="Full reference")
    ax.set_xticks(x)
    ax.set_xlabel("Affected UAV count h")
    ax.set_ylabel("Total replanning runtime (s)")
    ax.grid(axis="y", alpha=0.25)
    ax.legend(frameon=False)
    fig.tight_layout()
    _save(fig, "h_sensitivity_runtime.png")

    fig, axes = plt.subplots(1, 2, figsize=(10.0, 4.2))
    for ax, metric, title in (
        (axes[0], "assignment_changes", "Assignment changes"),
        (axes[1], "successor_edge_changes", "Successor-edge changes"),
    ):
        values, intervals = zip(*[_mean_ci(frame, h, metric) for h in H_VALUES])
        ax.errorbar(x, values, yerr=intervals, marker="o", capsize=4, linewidth=2, color="#2878B5")
        ax.axhline(_mean(summary, "Full", metric), color="#E17C05", linestyle="--", label="Full reference")
        ax.axhline(_mean(summary, "NoReorder", metric), color="#7A7A7A", linestyle="--", label="NoReorder reference")
        ax.set_xticks(x)
        ax.set_xlabel("Affected UAV count h")
        ax.set_title(title)
        ax.grid(axis="y", alpha=0.25)
    axes[0].set_ylabel("Count per run")
    axes[0].legend(frameon=False)
    fig.tight_layout()
    _save(fig, "h_sensitivity_disruption.png")


def _format(value: float, digits: int = 4) -> str:
    return f"{value:.{digits}f}"


def _paired_row(paired: pd.DataFrame, comparison: str, metric: str) -> pd.Series:
    return paired[(paired["comparison"] == comparison) & (paired["metric"] == metric)].iloc[0]


def _write_statistics(summary: pd.DataFrame, paired: pd.DataFrame, integrity: dict[str, object]) -> None:
    lines = [
        "# Local h Sensitivity Statistics",
        "",
        "## Integrity",
        "",
        f"- Complete unique records: {integrity['unique_run_count']}/30",
        f"- New runs: {integrity['new_run_count']}/20",
        f"- Reused E5 Medium h=2 records: {integrity['reused_h2_count']}/10",
        f"- Feasible/time-consistent/all-tasks-completed/all-UAVs-returned: {integrity['feasible_run_count']}/30, {integrity['time_consistent_run_count']}/30, {integrity['all_tasks_completed_run_count']}/30, {integrity['all_uavs_returned_run_count']}/30",
        "",
        "## Mean and 95% CI",
        "",
        "| Method | Weighted mean delay | Total replanning runtime (s) | Assignment changes | Successor changes |",
        "|---|---:|---:|---:|---:|",
    ]
    for method in ("NoReorder", "Local h=1", "Local h=2", "Local h=3", "Full"):
        cells = []
        for metric in ("weighted_mean_delay", "total_replanning_runtime", "assignment_changes", "successor_edge_changes"):
            row = summary[(summary["method"] == method) & (summary["metric"] == metric)].iloc[0]
            cells.append(f"{_format(float(row['mean']))} [{_format(float(row['ci95_lower']))}, {_format(float(row['ci95_upper']))}]")
        lines.append(f"| {method} | " + " | ".join(cells) + " |")
    lines.extend([
        "",
        "## Paired Local comparisons",
        "",
        "Win/tie/loss is from method A's perspective and lower is better.",
        "",
        "| Comparison | Metric | Win/tie/loss | Mean A-B | Median A-B | Wilcoxon p |",
        "|---|---|---:|---:|---:|---:|",
    ])
    for comparison in ("h1_vs_h2", "h2_vs_h3", "h1_vs_h3"):
        for metric in ("weighted_mean_delay", "total_replanning_runtime", "assignment_changes", "successor_edge_changes"):
            row = _paired_row(paired, comparison, metric)
            lines.append(
                f"| {comparison} | {metric} | {int(row['wins_A_lower'])}/{int(row['ties'])}/{int(row['losses_A_higher'])} | "
                f"{_format(float(row['paired_mean_difference_A_minus_B']))} | {_format(float(row['paired_median_difference_A_minus_B']))} | {_format(float(row['wilcoxon_p_two_sided']))} |"
            )
    lines.extend([
        "",
        "Each h group and each paired comparison uses the same ten seeds. Confidence intervals use `mean ± 1.96 × sample_std / sqrt(n)`; Wilcoxon results are auxiliary only.",
        "",
    ])
    (OUTPUT_DIR / "statistics.md").write_text("\n".join(lines), encoding="utf-8")


def _write_report(frame: pd.DataFrame, summary: pd.DataFrame, paired: pd.DataFrame, integrity: dict[str, object]) -> str:
    q = {h: _mean(summary, f"Local h={h}", "weighted_mean_delay") for h in H_VALUES}
    rt = {h: _mean(summary, f"Local h={h}", "total_replanning_runtime") for h in H_VALUES}
    art = {h: _mean(summary, f"Local h={h}", "mean_replanning_runtime") for h in H_VALUES}
    mrt = {h: _mean(summary, f"Local h={h}", "max_replanning_runtime") for h in H_VALUES}
    ac = {h: _mean(summary, f"Local h={h}", "assignment_changes") for h in H_VALUES}
    sc = {h: _mean(summary, f"Local h={h}", "successor_edge_changes") for h in H_VALUES}

    def dominates(a: int, b: int) -> bool:
        a_values = (q[a], rt[a], ac[a], sc[a])
        b_values = (q[b], rt[b], ac[b], sc[b])
        return all(x <= y + 1e-12 for x, y in zip(a_values, b_values)) and any(x < y - 1e-12 for x, y in zip(a_values, b_values))

    classic_tradeoff = q[1] >= q[2] >= q[3] and rt[1] <= rt[2] <= rt[3] and ac[1] <= ac[2] <= ac[3] and sc[1] <= sc[2] <= sc[3]
    if dominates(1, 2) or dominates(3, 2):
        choice = "C. 数据显示其他 h 更合适，应在论文中重新解释"
        explanation = "按均值看，h=2 被另一个候选值在质量、重规划时间和两类扰动指标上同时支配，因此不能把 h=2 继续解释为数据支持的折中点。"
    elif classic_tradeoff:
        choice = "A. h=2 是合理折中，建议保留"
        explanation = "均值呈现清晰的质量—计算—稳定性折中：扩大 h 改善质量，同时增加计算时间和计划扰动；h=2 位于两个端点之间且未被支配。"
    else:
        choice = "B. h=2 可保留，但不存在明显最优性"
        explanation = "h=2 未被其他候选值全面支配，但质量、时间和扰动并未形成一致的单调关系，因此只能作为代表性设置保留，不能声称明显最优。"

    seed_pivot = frame.pivot(index="seed", columns="h", values="weighted_mean_delay")
    h1_better_h2 = [str(int(seed)) for seed in seed_pivot.index if seed_pivot.loc[seed, 1] < seed_pivot.loc[seed, 2] - 1e-9]
    h2_better_h3 = [str(int(seed)) for seed in seed_pivot.index if seed_pivot.loc[seed, 2] < seed_pivot.loc[seed, 3] - 1e-9]
    h1_h2 = _paired_row(paired, "h1_vs_h2", "weighted_mean_delay")
    h2_h3 = _paired_row(paired, "h2_vs_h3", "weighted_mean_delay")

    report = [
        "# Local Parameter h Sensitivity Report",
        "",
        "## 1. 实验目的",
        "",
        "本实验只用于检查 Local 中受影响 UAV 数量参数 h 的敏感性，并解释默认 h=2 的经验性定位；它不承担新的创新点证明，也不重新设计或调优算法。",
        "",
        "## 2. 实验配置",
        "",
        "实验严格复用 E5 Medium：25 个初始任务、25 个动态任务、5 架 UAV、seeds 2000–2009，规划与执行均为 Dubins，主目标为 weighted_delay，commitment_horizon=0，搜索迭代与时间预算不变。唯一变量为 h=1/2/3。h=2 的 10 条记录直接复用冻结的 E5 Medium Local 结果；NoReorder 与 Full 也只读取 E5 Medium 作为参考，未重跑。Full 是全自由后缀启发式重规划，不是 Exact optimum。",
        "",
        "## 3. 可行性检查",
        "",
        f"- 30/30 条 seed+h 记录完整且唯一，其中新运行 {integrity['new_run_count']} 条、复用 E5 h=2 记录 {integrity['reused_h2_count']} 条。",
        f"- feasible、time-consistent、完成全部任务、全部 UAV 返航分别为 {integrity['feasible_run_count']}/30、{integrity['time_consistent_run_count']}/30、{integrity['all_tasks_completed_run_count']}/30、{integrity['all_uavs_returned_run_count']}/30。",
        "- 同一 seed 的 h=1/2/3 场景指纹一致；复用的 h=2 字段与 E5 Medium 原始记录逐字段一致。",
        "- E1–E5 raw、Canonical 产物和冻结核心算法文件前后哈希一致。",
        "",
        "## 4. 解质量变化",
        "",
        "| h | Weighted mean delay mean |",
        "|---:|---:|",
        *[f"| {h} | {q[h]:.4f} |" for h in H_VALUES],
        "",
        f"h=1 vs h=2 的 paired win/tie/loss（低延迟为胜）为 {int(h1_h2['wins_A_lower'])}/{int(h1_h2['ties'])}/{int(h1_h2['losses_A_higher'])}；h=2 vs h=3 为 {int(h2_h3['wins_A_lower'])}/{int(h2_h3['ties'])}/{int(h2_h3['losses_A_higher'])}。结果不预设单调性，所有反向 seed 均保留。",
        "",
        "## 5. 计算时间变化",
        "",
        "| h | Total replanning runtime | Mean replanning runtime | Max replanning runtime |",
        "|---:|---:|---:|---:|",
        *[f"| {h} | {rt[h]:.4f} s | {art[h]:.4f} s | {mrt[h]:.4f} s |" for h in H_VALUES],
        "",
        f"E5 Medium Full 的 total replanning runtime 参考均值为 {_mean(summary, 'Full', 'total_replanning_runtime'):.4f} s；这里只把它作为全自由后缀启发式参考线。",
        "",
        "## 6. 计划扰动变化",
        "",
        "| h | Assignment changes | Successor-edge changes |",
        "|---:|---:|---:|",
        *[f"| {h} | {ac[h]:.2f} | {sc[h]:.2f} |" for h in H_VALUES],
        "",
        f"参考线：NoReorder 为 {_mean(summary, 'NoReorder', 'assignment_changes'):.2f}/{_mean(summary, 'NoReorder', 'successor_edge_changes'):.2f}，Full 为 {_mean(summary, 'Full', 'assignment_changes'):.2f}/{_mean(summary, 'Full', 'successor_edge_changes'):.2f}（assignment/successor）。",
        "",
        "## 7. 综合 trade-off",
        "",
        "h=1 限制最强，倾向于更低的搜索开销和更稳定的计划；h=3 开放更多 UAV 后缀，倾向于提高搜索自由度，同时可能增加计算与扰动；h=2 位于两者之间。上述倾向必须结合本轮实际均值和 paired 结果理解，不能从机制直接推导为严格单调。",
        f"质量反向实例：h=1 优于 h=2 的 seeds 为 {'、'.join(h1_better_h2) if h1_better_h2 else '无'}；h=2 优于 h=3 的 seeds 为 {'、'.join(h2_better_h3) if h2_better_h3 else '无'}。",
        "",
        "## 8. h=2 是否合理",
        "",
        f"**{choice}。**",
        "",
        explanation,
        "",
        "## 9. 结论边界",
        "",
        "本实验只有 E5 Medium 的 10 个 paired seeds，结论只说明冻结配置下的经验性 trade-off。不得把 h=2 表述为全局最优、所有指标最优或经过充分超参数优化；Wilcoxon p-value 仅作辅助，不作为更改 seed 或参数的依据。",
        "",
    ]
    (OUTPUT_DIR / "H_SENSITIVITY_REPORT.md").write_text("\n".join(report), encoding="utf-8")
    return choice


def analyze() -> None:
    if not MANIFEST_PATH.exists():
        raise RuntimeError("baseline manifest is missing")
    if not RAW_PATH.exists():
        raise RuntimeError("sensitivity raw.csv is missing")
    manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
    current = _protected_snapshot()
    baseline = manifest["baseline"]
    for key in ("formal_raw_sha256", "canonical_files_sha256", "canonical_manifest_sha256", "protected_code_sha256"):
        if current[key] != baseline[key]:
            raise RuntimeError(f"protected baseline changed during sensitivity study: {key}")
    _validate_h2_exact_reuse()
    frame = pd.read_csv(RAW_PATH)
    integrity = _validate(frame)
    e5 = pd.read_csv(E5_RAW_PATH)
    e5 = e5[e5["scale"] == "medium"]
    summary = _write_summary(frame, e5)
    paired = _write_paired(frame)
    _write_figures(frame, summary)
    _write_statistics(summary, paired, integrity)
    judgement = _write_report(frame, summary, paired, integrity)
    manifest["post_run"] = {
        "checked_at": datetime.now().astimezone().isoformat(),
        "git_commit": _git_commit(),
        **current,
        "h_sensitivity_raw_sha256": _sha256(RAW_PATH),
        "integrity": integrity,
        "h2_judgement": judgement,
    }
    MANIFEST_PATH.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    for name in (
        "raw.csv",
        "summary.csv",
        "paired_comparisons.csv",
        "statistics.md",
        "H_SENSITIVITY_REPORT.md",
        "h_sensitivity_quality.png",
        "h_sensitivity_runtime.png",
        "h_sensitivity_disruption.png",
    ):
        print(OUTPUT_DIR / name)
    print(judgement)


def main() -> None:
    parser = argparse.ArgumentParser()
    actions = parser.add_mutually_exclusive_group(required=True)
    actions.add_argument("--capture-baseline", action="store_true")
    actions.add_argument("--run", action="store_true")
    actions.add_argument("--analyze", action="store_true")
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()
    if args.capture_baseline:
        capture_baseline()
    elif args.run:
        run_matrix(args.resume)
    else:
        analyze()


if __name__ == "__main__":
    main()
