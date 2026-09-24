"""Run a small configurable seed batch without enabling large experiments."""

from __future__ import annotations

import argparse
import sys
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import pandas as pd

from uav_planning.config import load_config
from uav_planning.planners import FullReplanner, InitialPlanner, LocalReplanner, NoReorderInsertionPlanner
from uav_planning.routing.evaluator import DubinsTravelTimeProvider, RouteEvaluator
from uav_planning.routing.insertion import RouteOptimizer
from uav_planning.scenario.generator import ScenarioGenerator
from uav_planning.simulation.simulator import Simulator


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="configs/small_debug.yaml")
    parser.add_argument("--seeds", nargs="+", type=int, default=[42])
    args = parser.parse_args()
    config_path = Path(args.config)
    if not config_path.is_absolute():
        config_path = ROOT / config_path
    base_config = load_config(config_path)
    rows = []
    for seed in args.seeds:
        config = replace(base_config, seed=seed)
        scenario = ScenarioGenerator(config).generate()
        optimizer = RouteOptimizer(
            RouteEvaluator(DubinsTravelTimeProvider()),
            config.planner.vns_max_iterations,
            config.planner.vns_time_limit_sec,
        )
        simulator = Simulator(
            scenario,
            InitialPlanner(optimizer),
            optimizer,
            config.high_priority_threshold,
        )
        for strategy in (
            NoReorderInsertionPlanner(optimizer),
            FullReplanner(optimizer),
            LocalReplanner(optimizer, config.planner.affected_uav_count_h),
        ):
            rows.append(simulator.run(strategy).metrics)
    output = ROOT / "results" / "raw" / "batch_metrics.csv"
    output.parent.mkdir(parents=True, exist_ok=True)
    pd.DataFrame(rows).to_csv(output, index=False)
    print(output)


if __name__ == "__main__":
    main()

