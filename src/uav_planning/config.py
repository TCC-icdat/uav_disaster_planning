"""YAML configuration loading with typed dataclasses."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class MapConfig:
    width: float
    height: float
    depot_x: float
    depot_y: float
    depot_heading: float


@dataclass(frozen=True)
class UAVConfig:
    count: int
    speed: float
    min_turn_radius: float
    max_mission_time: float


@dataclass(frozen=True)
class TaskConfig:
    initial_count: int
    dynamic_count: int
    priority_range: tuple[int, int]
    service_time_range: tuple[float, float]
    release_time_range: tuple[float, float]


@dataclass(frozen=True)
class PlannerConfig:
    affected_uav_count_h: int
    vns_max_iterations: int
    vns_time_limit_sec: float


@dataclass(frozen=True)
class ExperimentConfig:
    seed: int
    travel_model: str
    high_priority_threshold: float
    map: MapConfig
    uav: UAVConfig
    tasks: TaskConfig
    planner: PlannerConfig


def _pair(values: list[Any]) -> tuple[Any, Any]:
    if len(values) != 2:
        raise ValueError("range configuration must contain exactly two values")
    return values[0], values[1]


def load_config(path: str | Path) -> ExperimentConfig:
    """Load and validate an experiment YAML file."""

    with Path(path).open("r", encoding="utf-8") as stream:
        raw = yaml.safe_load(stream)
    map_raw = raw["map"]
    depot = map_raw["depot"]
    uav_raw = raw["sar_uav"]
    task_raw = raw["tasks"]
    planner_raw = raw["local_replanning"]
    config = ExperimentConfig(
        seed=int(raw["seed"]),
        travel_model=str(raw.get("travel_model", "dubins")),
        high_priority_threshold=float(raw.get("high_priority_threshold", 7.0)),
        map=MapConfig(
            width=float(map_raw["width"]),
            height=float(map_raw["height"]),
            depot_x=float(depot["x"]),
            depot_y=float(depot["y"]),
            depot_heading=float(depot["heading"]),
        ),
        uav=UAVConfig(
            count=int(uav_raw["count"]),
            speed=float(uav_raw["speed"]),
            min_turn_radius=float(uav_raw["min_turn_radius"]),
            max_mission_time=float(uav_raw["max_mission_time"]),
        ),
        tasks=TaskConfig(
            initial_count=int(task_raw["initial_count"]),
            dynamic_count=int(task_raw["dynamic_count"]),
            priority_range=tuple(map(int, _pair(task_raw["priority_range"]))),
            service_time_range=tuple(map(float, _pair(task_raw["service_time_range"]))),
            release_time_range=tuple(map(float, _pair(task_raw["release_time_range"]))),
        ),
        planner=PlannerConfig(
            affected_uav_count_h=int(planner_raw["affected_uav_count_h"]),
            vns_max_iterations=int(planner_raw["vns_max_iterations"]),
            vns_time_limit_sec=float(planner_raw["vns_time_limit_sec"]),
        ),
    )
    if config.travel_model not in {"dubins", "euclidean"}:
        raise ValueError("travel_model must be 'dubins' or 'euclidean'")
    return config

