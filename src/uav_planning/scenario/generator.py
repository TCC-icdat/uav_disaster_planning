"""Synthetic task-release scenarios for the MVP experiments."""

from __future__ import annotations

from dataclasses import dataclass
from math import pi

import numpy as np

from uav_planning.config import ExperimentConfig
from uav_planning.models import Pose2D, Task, UAV


@dataclass(frozen=True)
class Scenario:
    """UAV fleet and complete hidden task stream for one seed."""

    seed: int
    width: float
    height: float
    uavs: tuple[UAV, ...]
    initial_tasks: tuple[Task, ...]
    dynamic_tasks: tuple[Task, ...]

    @property
    def all_tasks(self) -> tuple[Task, ...]:
        return self.initial_tasks + self.dynamic_tasks


class ScenarioGenerator:
    """Generate all stochastic values from one numpy Generator."""

    def __init__(self, config: ExperimentConfig) -> None:
        self.config = config
        self.rng = np.random.default_rng(config.seed)

    def generate(self) -> Scenario:
        """Create the configured fleet and synthetic release stream."""

        depot = Pose2D(
            self.config.map.depot_x,
            self.config.map.depot_y,
            self.config.map.depot_heading,
        )
        uavs = tuple(
            UAV(
                uav_id=index + 1,
                speed=self.config.uav.speed,
                min_turn_radius=self.config.uav.min_turn_radius,
                max_mission_time=self.config.uav.max_mission_time,
                start_pose=depot,
            )
            for index in range(self.config.uav.count)
        )
        total = self.config.tasks.initial_count + self.config.tasks.dynamic_count
        xs = self.rng.uniform(0.1 * self.config.map.width, self.config.map.width, total)
        ys = self.rng.uniform(0.1 * self.config.map.height, self.config.map.height, total)
        priorities = self.rng.integers(
            self.config.tasks.priority_range[0],
            self.config.tasks.priority_range[1] + 1,
            total,
        )
        services = self.rng.uniform(*self.config.tasks.service_time_range, total)
        headings = self.rng.uniform(0.0, 2.0 * pi, total)
        releases = np.sort(
            self.rng.uniform(
                *self.config.tasks.release_time_range,
                self.config.tasks.dynamic_count,
            )
        )

        tasks: list[Task] = []
        for index in range(total):
            release_time = (
                0.0
                if index < self.config.tasks.initial_count
                else float(releases[index - self.config.tasks.initial_count])
            )
            tasks.append(
                Task(
                    task_id=index + 1,
                    x=float(xs[index]),
                    y=float(ys[index]),
                    release_time=release_time,
                    priority=float(priorities[index]),
                    service_time=float(services[index]),
                    required_heading=float(headings[index]),
                )
            )
        split = self.config.tasks.initial_count
        return Scenario(
            seed=self.config.seed,
            width=self.config.map.width,
            height=self.config.map.height,
            uavs=uavs,
            initial_tasks=tuple(tasks[:split]),
            dynamic_tasks=tuple(tasks[split:]),
        )

