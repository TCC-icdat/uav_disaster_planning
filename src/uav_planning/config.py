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
    commitment_horizon: int
    local_search_max_iterations: int
    local_search_time_limit_sec: float


@dataclass(frozen=True)
class ExperimentConfig:
    seed: int
    planning_travel_model: str
    execution_travel_model: str
    objective_mode: str
    high_priority_threshold: float
    map: MapConfig
    uav: UAVConfig
    tasks: TaskConfig
    planner: PlannerConfig

    @property
    def travel_model(self) -> str:
        """Backward-compatible alias for the planner's travel model."""

        return self.planning_travel_model


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
        planning_travel_model=str(
            raw.get("planning_travel_model", raw.get("travel_model", "dubins"))
        ),
        execution_travel_model=str(
            raw.get(
                "execution_travel_model",
                raw.get("travel_model", "dubins"),
            )
        ),
        objective_mode=str(raw.get("objective_mode", "weighted_delay")),
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
            commitment_horizon=int(planner_raw.get("commitment_horizon", 0)),
            local_search_max_iterations=int(
                planner_raw["local_search_max_iterations"]
            ),
            local_search_time_limit_sec=float(
                planner_raw["local_search_time_limit_sec"]
            ),
        ),
    )
    for field_name, model in (
        ("planning_travel_model", config.planning_travel_model),
        ("execution_travel_model", config.execution_travel_model),
    ):
        if model not in {"dubins", "euclidean"}:
            raise ValueError(f"{field_name} must be 'dubins' or 'euclidean'")
    if config.objective_mode not in {"weighted_delay", "total_travel_time"}:
        raise ValueError(
            "objective_mode must be 'weighted_delay' or 'total_travel_time'"
        )
    if config.planner.commitment_horizon != 0:
        raise ValueError("MVP core comparison requires commitment_horizon = 0")
    return config

