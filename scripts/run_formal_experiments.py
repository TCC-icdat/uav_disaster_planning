"""Run the frozen E1-E5 formal experiment matrix one experiment at a time."""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.metadata
import math
import os
import platform
import subprocess
import sys
from dataclasses import replace
from datetime import datetime
from pathlib import Path
from time import perf_counter

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import pandas as pd
from scipy.stats import wilcoxon

from uav_planning.config import ExperimentConfig, load_config
from uav_planning.exact import ExactEnumerator
from uav_planning.planners import (
    FullReplanner,
    InitialPlanner,
    LocalReplanner,
    NoReorderInsertionPlanner,
)
from uav_planning.routing.evaluator import (
    DubinsTravelTimeProvider,
    EuclideanTravelTimeProvider,
    RouteEvaluator,
)
from uav_planning.routing.insertion import RouteOptimizer
from uav_planning.scenario.generator import Scenario, ScenarioGenerator
from uav_planning.simulation.simulator import Simulator


CONFIG_FILES = {
    "E1": "e1_exact_benchmark.yaml",
    "E2": "e2_time_objective.yaml",
    "E2_v2": "e2_v2_time_objective.yaml",
    "E3": "e3_travel_model.yaml",
    "E4": "e4_dynamic_strategy.yaml",
    "E5": "e5_scale_template.yaml",
}
FORMAL_ROOT = ROOT / "results" / "formal"
SEEDS_30 = tuple(range(1000, 1030))


def _provider(name: str):
    if name == "dubins":
        return DubinsTravelTimeProvider()
    if name == "euclidean":
        return EuclideanTravelTimeProvider()
    raise ValueError(f"unsupported travel model: {name}")


def _scenario_fingerprint(scenario: Scenario) -> str:
    """Hash all generated scenario data used by paired variants."""

    parts = [
        str(scenario.seed),
        f"{scenario.width:.17g}",
        f"{scenario.height:.17g}",
    ]
    for uav in scenario.uavs:
        parts.append(
            ":".join(
                [
                    str(uav.uav_id),
                    f"{uav.speed:.17g}",
                    f"{uav.min_turn_radius:.17g}",
                    f"{uav.max_mission_time:.17g}",
                ]
            )
        )
    for task in scenario.all_tasks:
        parts.append(
            ":".join(
                [
                    str(task.task_id),
                    f"{task.x:.17g}",
                    f"{task.y:.17g}",
                    f"{task.release_time:.17g}",
                    f"{task.priority:.17g}",
                    f"{task.service_time:.17g}",
                    f"{task.required_heading:.17g}",
                ]
            )
        )
    return hashlib.sha256("|".join(parts).encode("utf-8")).hexdigest()


def _specs(experiment: str) -> list[dict[str, object]]:
    if experiment == "E1":
        return [
            {"seed": seed, "task_count": task_count}
            for task_count in (5, 6, 7)
            for seed in range(10)
        ]
    if experiment == "E2":
        variants = (
            ("time_aware", "weighted_delay"),
            ("distance_oriented", "total_travel_time"),
        )
        return [
            {"seed": seed, "variant": variant, "objective_mode": objective}
            for seed in SEEDS_30
            for variant, objective in variants
        ]
    if experiment == "E2_v2":
        variants = (
            ("time_aware", "weighted_delay"),
            ("mission_completion", "makespan"),
            ("distance_oriented", "total_travel_time"),
        )
        return [
            {"seed": seed, "variant": variant, "objective_mode": objective}
            for seed in SEEDS_30
            for variant, objective in variants
        ]
    if experiment == "E3":
        variants = (("dubins_aware", "dubins"), ("euclidean_planned", "euclidean"))
        return [
            {
                "seed": seed,
                "variant": variant,
                "planning_travel_model": planning_model,
                "turning_radius": radius,
            }
            for radius in (5.0, 10.0, 15.0)
            for seed in SEEDS_30
            for variant, planning_model in variants
        ]
    if experiment == "E4":
        return [
            {"seed": seed, "variant": strategy, "strategy": strategy}
            for seed in SEEDS_30
            for strategy in ("no_reorder", "full", "local")
        ]
    scales = (
        ("small", 10, 10, 3),
        ("medium", 25, 25, 5),
        ("large", 50, 50, 8),
    )
    return [
        {
            "seed": seed,
            "variant": strategy,
            "strategy": strategy,
            "scale": scale,
            "initial_count": initial,
            "dynamic_count": dynamic,
            "uav_count": uav_count,
        }
        for scale, initial, dynamic, uav_count in scales
        for seed in range(2000, 2010)
        for strategy in ("no_reorder", "full", "local")
    ]


def _key_fields(experiment: str) -> tuple[str, ...]:
    return {
        "E1": ("task_count", "seed"),
        "E2": ("seed", "variant"),
        "E2_v2": ("seed", "variant"),
        "E3": ("turning_radius", "seed", "variant"),
        "E4": ("seed", "variant"),
        "E5": ("scale", "seed", "variant"),
    }[experiment]


def _key(row: dict[str, object], fields: tuple[str, ...]) -> tuple[str, ...]:
    return tuple(str(row[field]) for field in fields)


def _append_row(path: Path, row: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    exists = path.exists() and path.stat().st_size > 0
    with path.open("a", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(row))
        if not exists:
            writer.writeheader()
        writer.writerow(row)
        stream.flush()
        os.fsync(stream.fileno())


def _completed_keys(path: Path, fields: tuple[str, ...]) -> set[tuple[str, ...]]:
    if not path.exists():
        return set()
    with path.open("r", newline="", encoding="utf-8") as stream:
        return {_key(row, fields) for row in csv.DictReader(stream)}


def _optimizer(config: ExperimentConfig) -> RouteOptimizer:
    return RouteOptimizer(
        RouteEvaluator(_provider(config.planning_travel_model)),
        config.planner.local_search_max_iterations,
        config.planner.local_search_time_limit_sec,
        objective_mode=config.objective_mode,
    )


def _run_e1(base: ExperimentConfig, spec: dict[str, object]) -> dict[str, object]:
    wall_started = perf_counter()
    task_count = int(spec["task_count"])
    seed = int(spec["seed"])
    config = replace(
        base,
        seed=seed,
        uav=replace(base.uav, count=2),
        tasks=replace(base.tasks, initial_count=task_count, dynamic_count=0),
    )
    scenario = ScenarioGenerator(config).generate()
    uavs = {uav.uav_id: uav for uav in scenario.uavs}
    tasks = {task.task_id: task for task in scenario.initial_tasks}
    optimizer = _optimizer(config)
    heuristic_started = perf_counter()
    heuristic_plan = InitialPlanner(optimizer).plan(
        list(scenario.uavs), list(scenario.initial_tasks), current_time=0.0
    )
    heuristic_runtime = perf_counter() - heuristic_started
    heuristic_objective = optimizer.evaluate_plan(
        heuristic_plan, uavs, tasks, objective_task_ids=set(tasks)
    ).objective
    exact = ExactEnumerator(optimizer).solve(uavs, tasks)
    gap = (
        (heuristic_objective - exact.objective) / exact.objective * 100.0
        if exact.objective > 0.0
        else 0.0
    )
    return {
        "experiment": "E1",
        "seed": seed,
        "task_count": task_count,
        "uav_count": 2,
        "planning_travel_model": "dubins",
        "objective_mode": "weighted_delay",
        "exact_objective": exact.objective,
        "heuristic_objective": heuristic_objective,
        "gap_percent": gap,
        "exact_runtime": exact.runtime_sec,
        "heuristic_runtime": heuristic_runtime,
        "runner_wall_runtime": perf_counter() - wall_started,
        "evaluated_plan_count": exact.evaluated_plan_count,
        "scenario_fingerprint": _scenario_fingerprint(scenario),
    }


def _run_dynamic(
    experiment: str,
    base: ExperimentConfig,
    spec: dict[str, object],
) -> dict[str, object]:
    wall_started = perf_counter()
    config = replace(base, seed=int(spec["seed"]))
    if experiment in {"E2", "E2_v2"}:
        config = replace(config, objective_mode=str(spec["objective_mode"]))
        strategy_name = "local"
    elif experiment == "E3":
        config = replace(
            config,
            planning_travel_model=str(spec["planning_travel_model"]),
            uav=replace(config.uav, min_turn_radius=float(spec["turning_radius"])),
        )
        strategy_name = "local"
    elif experiment == "E4":
        strategy_name = str(spec["strategy"])
    else:
        config = replace(
            config,
            uav=replace(config.uav, count=int(spec["uav_count"])),
            tasks=replace(
                config.tasks,
                initial_count=int(spec["initial_count"]),
                dynamic_count=int(spec["dynamic_count"]),
            ),
        )
        strategy_name = str(spec["strategy"])

    scenario = ScenarioGenerator(config).generate()
    optimizer = _optimizer(config)
    strategies = {
        "no_reorder": NoReorderInsertionPlanner(optimizer),
        "full": FullReplanner(optimizer),
        "local": LocalReplanner(optimizer, config.planner.affected_uav_count_h),
    }
    result = Simulator(
        scenario,
        InitialPlanner(optimizer),
        optimizer,
        execution_evaluator=RouteEvaluator(_provider(config.execution_travel_model)),
        high_priority_threshold=config.high_priority_threshold,
    ).run(strategies[strategy_name])
    row: dict[str, object] = {
        "experiment": experiment,
        "variant": str(spec["variant"]),
    }
    if experiment == "E3":
        row["turning_radius"] = float(spec["turning_radius"])
    if experiment == "E5":
        row["scale"] = str(spec["scale"])
    row.update(result.metrics)
    row["runner_wall_runtime"] = perf_counter() - wall_started
    row["scenario_fingerprint"] = _scenario_fingerprint(scenario)
    return row


def _stats(
    frame: pd.DataFrame,
    group_columns: list[str],
    metrics: list[str],
) -> pd.DataFrame:
    rows: list[dict[str, object]] = []
    for group_key, group in frame.groupby(group_columns, sort=True, dropna=False):
        keys = group_key if isinstance(group_key, tuple) else (group_key,)
        labels = dict(zip(group_columns, keys))
        for metric in metrics:
            values = pd.to_numeric(group[metric], errors="coerce").dropna()
            n = int(values.count())
            mean = float(values.mean())
            std = float(values.std(ddof=1)) if n > 1 else 0.0
            margin = 1.96 * std / math.sqrt(n) if n else math.nan
            rows.append(
                {
                    **labels,
                    "metric": metric,
                    "n": n,
                    "mean": mean,
                    "std": std,
                    "median": float(values.median()),
                    "min": float(values.min()),
                    "max": float(values.max()),
                    "ci95_lower": mean - margin,
                    "ci95_upper": mean + margin,
                }
            )
    return pd.DataFrame(rows)


def _validate_pairing(frame: pd.DataFrame, group_columns: list[str]) -> None:
    counts = frame.groupby(group_columns, dropna=False)["scenario_fingerprint"].nunique()
    bad = counts[counts != 1]
    if not bad.empty:
        raise RuntimeError(f"paired variants used different scenarios: {bad.index.tolist()}")


def _paired_rows(
    frame: pd.DataFrame,
    group_columns: list[str],
    comparisons: list[tuple[str, str]],
    metrics: list[str],
    experiment: str,
) -> pd.DataFrame:
    _validate_pairing(frame, group_columns)
    rows: list[dict[str, object]] = []
    for group_key, group in frame.groupby(group_columns, sort=True, dropna=False):
        keys = group_key if isinstance(group_key, tuple) else (group_key,)
        labels = dict(zip(group_columns, keys))
        indexed = group.set_index("variant")
        for variant_a, variant_b in comparisons:
            if variant_a not in indexed.index or variant_b not in indexed.index:
                raise RuntimeError(
                    f"missing paired variants {variant_a}/{variant_b} for {labels}"
                )
            for metric in metrics:
                value_a = float(indexed.loc[variant_a, metric])
                value_b = float(indexed.loc[variant_b, metric])
                difference = value_a - value_b
                relative = difference / value_b * 100.0 if value_b != 0.0 else math.nan
                row: dict[str, object] = {
                    **labels,
                    "comparison": f"{variant_a}_vs_{variant_b}",
                    "metric": metric,
                    "variant_A": variant_a,
                    "variant_B": variant_b,
                    "value_A": value_a,
                    "value_B": value_b,
                    "absolute_difference": difference,
                    "relative_difference_percent": relative,
                    "outcome_A_lower_is_better": (
                        "win"
                        if difference < -1e-9
                        else "loss" if difference > 1e-9 else "tie"
                    ),
                    "scenario_fingerprint": indexed.loc[variant_a, "scenario_fingerprint"],
                }
                if experiment == "E3":
                    row["relative_improvement_percent"] = (
                        (value_b - value_a) / value_b * 100.0
                        if value_b != 0.0
                        else math.nan
                    )
                if (
                    experiment == "E4"
                    and variant_a == "local"
                    and variant_b == "full"
                    and metric == "weighted_delay"
                ):
                    row["quality_gap_local_vs_full_percent"] = relative
                if (
                    experiment == "E4"
                    and variant_a == "local"
                    and variant_b == "full"
                    and metric == "total_replanning_runtime"
                ):
                    row["runtime_ratio"] = value_a / value_b if value_b else math.nan
                rows.append(row)
    return pd.DataFrame(rows)


def _write_e1_summary(frame: pd.DataFrame, path: Path) -> None:
    rows = []
    for task_count, group in frame.groupby("task_count", sort=True):
        gap = pd.to_numeric(group["gap_percent"])
        n = len(gap)
        gap_std = float(gap.std(ddof=1)) if n > 1 else 0.0
        margin = 1.96 * gap_std / math.sqrt(n)
        rows.append(
            {
                "task_count": int(task_count),
                "n": n,
                "mean_gap_percent": float(gap.mean()),
                "std_gap_percent": gap_std,
                "median_gap_percent": float(gap.median()),
                "min_gap_percent": float(gap.min()),
                "max_gap_percent": float(gap.max()),
                "gap_ci95_lower": float(gap.mean()) - margin,
                "gap_ci95_upper": float(gap.mean()) + margin,
                "mean_exact_objective": float(group["exact_objective"].mean()),
                "mean_heuristic_objective": float(group["heuristic_objective"].mean()),
                "mean_exact_runtime": float(group["exact_runtime"].mean()),
                "mean_heuristic_runtime": float(group["heuristic_runtime"].mean()),
            }
        )
    pd.DataFrame(rows).to_csv(path, index=False)


def _write_wilcoxon(frame: pd.DataFrame, path: Path) -> None:
    """Write paired two-sided Wilcoxon tests for the E2_v2 main comparison."""

    rows = []
    for metric in (
        "weighted_mean_delay",
        "high_priority_mean_delay",
        "total_travel_time",
        "makespan",
    ):
        pivot = frame.pivot(index="seed", columns="variant", values=metric)
        value_a = pd.to_numeric(pivot["time_aware"])
        value_b = pd.to_numeric(pivot["mission_completion"])
        differences = value_a - value_b
        result = wilcoxon(
            value_a,
            value_b,
            alternative="two-sided",
            zero_method="wilcox",
            method="auto",
        )
        tolerance = 1e-9
        rows.append(
            {
                "comparison": "time_aware_vs_mission_completion",
                "metric": metric,
                "n": len(differences),
                "wilcoxon_statistic": float(result.statistic),
                "p_value_two_sided": float(result.pvalue),
                "mean_time_aware": float(value_a.mean()),
                "mean_mission_completion": float(value_b.mean()),
                "mean_difference": float(differences.mean()),
                "mean_relative_difference_percent": float(
                    ((value_a - value_b) / value_b * 100.0).mean()
                ),
                "wins_time_aware_lower": int((differences < -tolerance).sum()),
                "ties": int((differences.abs() <= tolerance).sum()),
                "losses_time_aware_higher": int((differences > tolerance).sum()),
            }
        )
    pd.DataFrame(rows).to_csv(path, index=False)


def _write_summaries(experiment: str, raw_path: Path) -> None:
    frame = pd.read_csv(raw_path)
    output_dir = raw_path.parent
    if experiment == "E1":
        _write_e1_summary(frame, output_dir / "summary.csv")
        return

    metrics = {
        "E2": [
            "weighted_delay",
            "weighted_mean_delay",
            "high_priority_mean_delay",
            "mean_delay",
            "total_travel_time",
            "runner_wall_runtime",
            "feasible",
            "time_consistency_check",
        ],
        "E2_v2": [
            "weighted_delay",
            "weighted_mean_delay",
            "high_priority_mean_delay",
            "mean_delay",
            "makespan",
            "total_travel_time",
            "active_uav_count",
            "max_tasks_per_uav",
            "min_tasks_per_active_uav",
            "runner_wall_runtime",
            "feasible",
            "time_consistency_check",
        ],
        "E3": [
            "weighted_mean_delay",
            "weighted_delay",
            "high_priority_mean_delay",
            "total_travel_time",
            "runner_wall_runtime",
            "feasible",
            "time_consistency_check",
        ],
        "E4": [
            "weighted_mean_delay",
            "weighted_delay",
            "high_priority_mean_delay",
            "initial_planning_runtime",
            "total_replanning_runtime",
            "mean_replanning_runtime",
            "max_replanning_runtime",
            "total_algorithm_runtime",
            "assignment_changes",
            "successor_edge_changes",
            "runner_wall_runtime",
            "feasible",
            "time_consistency_check",
        ],
        "E5": [
            "weighted_mean_delay",
            "high_priority_mean_delay",
            "initial_planning_runtime",
            "total_replanning_runtime",
            "total_algorithm_runtime",
            "runner_wall_runtime",
            "feasible",
            "time_consistency_check",
        ],
    }[experiment]
    groups = ["variant"]
    if experiment == "E3":
        groups.insert(0, "turning_radius")
    if experiment == "E5":
        groups.insert(0, "scale")
    _stats(frame, groups, metrics).to_csv(output_dir / "summary.csv", index=False)

    if experiment == "E2":
        paired = _paired_rows(
            frame,
            ["seed"],
            [("time_aware", "distance_oriented")],
            metrics[:5],
            experiment,
        )
    elif experiment == "E2_v2":
        paired = _paired_rows(
            frame,
            ["seed"],
            [
                ("time_aware", "mission_completion"),
                ("time_aware", "distance_oriented"),
                ("mission_completion", "distance_oriented"),
            ],
            [
                "weighted_mean_delay",
                "high_priority_mean_delay",
                "mean_delay",
                "makespan",
                "total_travel_time",
            ],
            experiment,
        )
        _write_wilcoxon(frame, output_dir / "wilcoxon.csv")
    elif experiment == "E3":
        paired = _paired_rows(
            frame,
            ["turning_radius", "seed"],
            [("dubins_aware", "euclidean_planned")],
            metrics[:4],
            experiment,
        )
    elif experiment == "E4":
        paired = _paired_rows(
            frame,
            ["seed"],
            [
                ("local", "full"),
                ("local", "no_reorder"),
                ("full", "no_reorder"),
            ],
            metrics[:10],
            experiment,
        )
    else:
        return
    paired.to_csv(output_dir / "paired.csv", index=False)


def _config_path(experiment: str) -> Path:
    return ROOT / "configs" / "experiments" / CONFIG_FILES[experiment]


def _git_commit() -> str:
    try:
        result = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=str(ROOT),
            capture_output=True,
            text=True,
            check=True,
        )
        return result.stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "N/A"


def _ensure_environment_record() -> None:
    path = FORMAL_ROOT / "ENVIRONMENT.md"
    if path.exists():
        return
    FORMAL_ROOT.mkdir(parents=True, exist_ok=True)
    dependencies = []
    for package in ("numpy", "pandas", "matplotlib", "PyYAML", "pytest"):
        try:
            version = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:
            version = "not installed"
        dependencies.append(f"- {package}: {version}")
    hashes = []
    for experiment, filename in CONFIG_FILES.items():
        config_path = ROOT / "configs" / "experiments" / filename
        digest = hashlib.sha256(config_path.read_bytes()).hexdigest()
        hashes.append(f"- {experiment} `{filename}`: `{digest}`")
    cpu = platform.processor() or os.environ.get("PROCESSOR_IDENTIFIER", "N/A")
    content = "\n".join(
        [
            "# Formal Experiment Environment",
            "",
            f"- Formal experiment start time: {datetime.now().astimezone().isoformat()}",
            f"- Python: {platform.python_version()}",
            f"- OS: {platform.platform()}",
            f"- CPU: {cpu}",
            f"- Git commit: {_git_commit()}",
            "",
            "## Dependency versions",
            "",
            *dependencies,
            "",
            "## Frozen configuration SHA-256",
            "",
            *hashes,
            "",
        ]
    )
    path.write_text(content, encoding="utf-8")


def _dry_run(experiment: str) -> None:
    specs = _specs(experiment)
    variants = sorted({str(spec.get("variant", "exact_vs_heuristic")) for spec in specs})
    seeds = sorted({int(spec["seed"]) for spec in specs})
    print(f"experiment: {experiment}")
    print(f"variants: {', '.join(variants)}")
    if experiment == "E3":
        print("turning_radii: 5, 10, 15")
    if experiment == "E5":
        print("scales: small(20/3), medium(50/5), large(100/8)")
    print(f"seeds: {seeds[0]}..{seeds[-1]} ({len(seeds)})")
    print(f"estimated_run_count: {len(specs)}")
    print(f"output_path: {FORMAL_ROOT / experiment / 'raw.csv'}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--experiment", required=True, choices=tuple(CONFIG_FILES))
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()
    experiment = args.experiment
    if args.dry_run:
        _dry_run(experiment)
        return

    _ensure_environment_record()
    output_dir = FORMAL_ROOT / experiment
    output_dir.mkdir(parents=True, exist_ok=True)
    raw_path = output_dir / "raw.csv"
    if raw_path.exists() and not args.resume:
        raw_path.unlink()

    fields = _key_fields(experiment)
    completed = _completed_keys(raw_path, fields) if args.resume else set()
    specs = _specs(experiment)
    pending = [spec for spec in specs if _key(spec, fields) not in completed]
    base = load_config(_config_path(experiment))
    print(
        f"{experiment}: total={len(specs)}, completed={len(completed)}, "
        f"pending={len(pending)}"
    )
    for index, spec in enumerate(pending, start=1):
        row = _run_e1(base, spec) if experiment == "E1" else _run_dynamic(
            experiment, base, spec
        )
        _append_row(raw_path, row)
        print(
            f"[{index}/{len(pending)}] saved "
            + ", ".join(f"{field}={spec[field]}" for field in fields)
        )
    if not raw_path.exists():
        raise RuntimeError("no formal raw results were produced")
    _write_summaries(experiment, raw_path)
    print(raw_path)
    print(output_dir / "summary.csv")
    paired_path = output_dir / "paired.csv"
    if paired_path.exists():
        print(paired_path)
    wilcoxon_path = output_dir / "wilcoxon.csv"
    if wilcoxon_path.exists():
        print(wilcoxon_path)


if __name__ == "__main__":
    main()
