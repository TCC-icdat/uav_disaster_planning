"""Run the seed-42 MVP acceptance scenario for all three strategies."""

from __future__ import annotations

import argparse
import platform
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import pandas as pd

from uav_planning.config import load_config
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
from uav_planning.scenario.generator import ScenarioGenerator
from uav_planning.simulation.simulator import Simulator
from uav_planning.visualization.plotter import plot_replanning_comparison


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="configs/small_debug.yaml")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config_path = Path(args.config)
    if not config_path.is_absolute():
        config_path = ROOT / config_path
    config = load_config(config_path)
    scenario = ScenarioGenerator(config).generate()
    provider = (
        DubinsTravelTimeProvider()
        if config.travel_model == "dubins"
        else EuclideanTravelTimeProvider()
    )
    evaluator = RouteEvaluator(provider)
    optimizer = RouteOptimizer(
        evaluator,
        local_search_max_iterations=config.planner.local_search_max_iterations,
        local_search_time_limit_sec=config.planner.local_search_time_limit_sec,
    )
    initial = InitialPlanner(optimizer)
    simulator = Simulator(
        scenario,
        initial,
        optimizer,
        high_priority_threshold=config.high_priority_threshold,
    )
    strategies = [
        NoReorderInsertionPlanner(optimizer),
        FullReplanner(optimizer),
        LocalReplanner(optimizer, h=config.planner.affected_uav_count_h),
    ]
    results = [simulator.run(strategy) for strategy in strategies]

    raw_dir = ROOT / "results" / "raw"
    summary_dir = ROOT / "results" / "summary"
    figure_dir = ROOT / "results" / "figures"
    for directory in (raw_dir, summary_dir, figure_dir):
        directory.mkdir(parents=True, exist_ok=True)

    metrics = pd.DataFrame([result.metrics for result in results])
    raw_path = raw_dir / f"seed{config.seed}_metrics.csv"
    metrics.to_csv(raw_path, index=False)
    summary_path = summary_dir / f"seed{config.seed}_summary.csv"
    metrics.to_csv(summary_path, index=False)
    consistency_path = summary_dir / "TIME_CONSISTENCY_CHECK.txt"
    checks_passed = bool(metrics["time_consistency_check"].all())
    consistency_path.write_text(
        "TIME_CONSISTENCY_CHECK: " + ("PASS\n" if checks_passed else "FAIL\n")
        + "Invariant: every free task service_start >= its replanning event time.\n"
        + "Invariant: completed TaskExecutionRecord values are immutable.\n"
        + "Metrics source: accumulated execution history, not a t=0 final-route replay.\n",
        encoding="utf-8",
    )
    for result in results:
        pd.DataFrame(result.event_log).to_csv(
            raw_dir / f"seed{config.seed}_{result.metrics['strategy']}_events.csv",
            index=False,
        )

    local_result = next(
        result for result in results if result.metrics["strategy"] == "local"
    )
    tasks = {task.task_id: task for task in scenario.all_tasks}
    figure_path = plot_replanning_comparison(
        local_result.initial_plan,
        local_result.final_plan,
        {uav.uav_id: uav for uav in scenario.uavs},
        tasks,
        {task.task_id for task in scenario.dynamic_tasks},
        figure_dir / f"debug_seed{config.seed}.png",
        scenario.width,
        scenario.height,
    )

    print(f"project path: {ROOT}")
    print(f"Python version: {platform.python_version()}")
    print(f"seed: {config.seed}")
    print("strategies run: " + ", ".join(metrics["strategy"]))
    for row in metrics.itertuples(index=False):
        print(
            f"{row.strategy}: objective={row.weighted_delay:.6f}, "
            f"replanning_runtime={row.total_replanning_runtime:.6f}s"
        )
    print(f"output CSV: {raw_path}")
    print(f"summary CSV: {summary_path}")
    print(f"figure path: {figure_path}")
    print("TIME_CONSISTENCY_CHECK: " + ("PASS" if checks_passed else "FAIL"))


if __name__ == "__main__":
    main()

