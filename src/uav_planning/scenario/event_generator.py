"""Task-event source abstraction reserved for later EO sweep support."""

from __future__ import annotations

from collections.abc import Iterator

from uav_planning.models import Task


class EventGenerator:
    """Yield synthetic task releases in chronological order."""

    def __init__(self, dynamic_tasks: list[Task] | tuple[Task, ...]) -> None:
        self._tasks = tuple(sorted(dynamic_tasks, key=lambda task: task.release_time))

    def releases(self) -> Iterator[Task]:
        yield from self._tasks

