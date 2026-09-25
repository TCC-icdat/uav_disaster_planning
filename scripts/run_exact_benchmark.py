"""Run the guarded 2-UAV, 5/6/7-task exact correctness benchmark."""

from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path
from time import perf_counter

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import pandas as pd

from uav_planning.config import load_config
from uav_planning.exact import ExactEnumerator
from uav_planning.planners import InitialPlanner
from uav_planning.routing.evaluator import DubinsTravelTimeProvider, RouteEvaluator
from uav_planning.routing.insertion import RouteOptimizer
from uav_planning.scenario.generator import ScenarioGenerator


def main() -> None:
    base = load_config(ROOT / "configs" / "experiments" / "e1_exact_benchmark.yaml")
    rows: list[dict[str, float | int]] = []
    for task_count in (5, 6, 7):
        for seed in range(10):
            config = replace(
                base,
                seed=seed,
                uav=replace(base.uav, count=2),
                tasks=replace(
                    base.tasks,
                    initial_count=task_count,
                    dynamic_count=0,
                ),
            )
            scenario = ScenarioGenerator(config).generate()
            uavs = {uav.uav_id: uav for uav in scenario.uavs}
            tasks = {task.task_id: task for task in scenario.initial_tasks}
            optimizer = RouteOptimizer(
                RouteEvaluator(DubinsTravelTimeProvider()),
                local_search_max_iterations=config.planner.local_search_max_iterations,
                local_search_time_limit_sec=config.planner.local_search_time_limit_sec,
                objective_mode="weighted_delay",
            )
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
            rows.append(
                {
                    "seed": seed,
                    "task_count": task_count,
                    "uav_count": 2,
                    "objective_exact": exact.objective,
                    "objective_heuristic": heuristic_objective,
                    "gap_percent": gap,
                    "exact_runtime": exact.runtime_sec,
                    "heuristic_runtime": heuristic_runtime,
                    "evaluated_plan_count": exact.evaluated_plan_count,
                }
            )

    frame = pd.DataFrame(rows)
    output_dir = ROOT / "results" / "exact"
    output_dir.mkdir(parents=True, exist_ok=True)
    raw_path = output_dir / "exact_benchmark_raw.csv"
    summary_path = output_dir / "exact_benchmark_summary.csv"
    frame.to_csv(raw_path, index=False)
    frame.groupby("task_count", as_index=False).agg(
        objective_exact_mean=("objective_exact", "mean"),
        objective_heuristic_mean=("objective_heuristic", "mean"),
        gap_percent_mean=("gap_percent", "mean"),
        gap_percent_max=("gap_percent", "max"),
        exact_runtime_mean=("exact_runtime", "mean"),
        heuristic_runtime_mean=("heuristic_runtime", "mean"),
    ).to_csv(summary_path, index=False)
    print(raw_path)
    print(summary_path)
    print(frame.groupby("task_count")[["gap_percent", "exact_runtime", "heuristic_runtime"]].mean())


if __name__ == "__main__":
    main()
