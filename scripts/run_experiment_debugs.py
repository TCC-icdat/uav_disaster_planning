"""Run 3-seed E2/E3/E4 data-pipeline checks, not formal experiments."""

from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import pandas as pd

from uav_planning.config import ExperimentConfig, load_config
from uav_planning.planners import FullReplanner, InitialPlanner, LocalReplanner, NoReorderInsertionPlanner
from uav_planning.routing.evaluator import DubinsTravelTimeProvider, EuclideanTravelTimeProvider, RouteEvaluator
from uav_planning.routing.insertion import RouteOptimizer
from uav_planning.scenario.generator import ScenarioGenerator
from uav_planning.simulation.simulator import Simulator


def _provider(name: str):
    return DubinsTravelTimeProvider() if name == "dubins" else EuclideanTravelTimeProvider()


def _run(
    base: ExperimentConfig,
    seed: int,
    experiment: str,
    variant: str,
    planning_model: str,
    objective_mode: str,
    strategy_name: str,
):
    config = replace(
        base,
        seed=seed,
        planning_travel_model=planning_model,
        execution_travel_model="dubins",
        objective_mode=objective_mode,
    )
    scenario = ScenarioGenerator(config).generate()
    optimizer = RouteOptimizer(
        RouteEvaluator(_provider(config.planning_travel_model)),
        config.planner.local_search_max_iterations,
        config.planner.local_search_time_limit_sec,
        objective_mode=config.objective_mode,
    )
    strategies = {
        "no_reorder": NoReorderInsertionPlanner(optimizer),
        "full": FullReplanner(optimizer),
        "local": LocalReplanner(optimizer, config.planner.affected_uav_count_h),
    }
    result = Simulator(
        scenario,
        InitialPlanner(optimizer),
        optimizer,
        execution_evaluator=RouteEvaluator(DubinsTravelTimeProvider()),
        high_priority_threshold=config.high_priority_threshold,
    ).run(strategies[strategy_name])
    return {"experiment": experiment, "variant": variant, **result.metrics}


def main() -> None:
    base = load_config(ROOT / "configs" / "replanning_debug.yaml")
    rows = []
    for seed in (42, 43, 44):
        rows.extend(
            [
                _run(base, seed, "E2", "time_aware", "dubins", "weighted_delay", "local"),
                _run(base, seed, "E2", "distance_oriented", "dubins", "total_travel_time", "local"),
                _run(base, seed, "E3", "euclidean_planned", "euclidean", "weighted_delay", "local"),
                _run(base, seed, "E3", "dubins_aware", "dubins", "weighted_delay", "local"),
                _run(base, seed, "E4", "no_reorder", "dubins", "weighted_delay", "no_reorder"),
                _run(base, seed, "E4", "full", "dubins", "weighted_delay", "full"),
                _run(base, seed, "E4", "local", "dubins", "weighted_delay", "local"),
            ]
        )
    frame = pd.DataFrame(rows)
    output_dir = ROOT / "results" / "debug"
    output_dir.mkdir(parents=True, exist_ok=True)
    raw_path = output_dir / "experiment_debug_raw.csv"
    summary_path = output_dir / "experiment_debug_summary.csv"
    frame.to_csv(raw_path, index=False)
    metrics = [
        "weighted_delay",
        "high_priority_mean_delay",
        "mean_delay",
        "total_travel_time",
        "total_replanning_runtime",
        "assignment_changes",
        "successor_edge_changes",
    ]
    frame.groupby(["experiment", "variant"], as_index=False)[metrics].mean().to_csv(
        summary_path, index=False
    )
    if not bool(frame["time_consistency_check"].all() and frame["feasible"].all()):
        raise RuntimeError("a debug experiment failed feasibility or time consistency")
    print(raw_path)
    print(summary_path)
    print("DEBUG_PIPELINE_CHECK: PASS")


if __name__ == "__main__":
    main()
