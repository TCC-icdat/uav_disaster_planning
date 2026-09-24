"""Minimal priority queue for discrete simulation events."""

from __future__ import annotations

import heapq
from dataclasses import dataclass, field
from enum import Enum
from typing import Any


class EventType(str, Enum):
    TASK_RELEASE = "TASK_RELEASE"
    TASK_START = "TASK_START"
    TASK_COMPLETE = "TASK_COMPLETE"
    RETURN_DEPOT = "RETURN_DEPOT"


@dataclass(order=True, frozen=True)
class Event:
    time: float
    sequence: int
    event_type: EventType = field(compare=False)
    payload: dict[str, Any] = field(default_factory=dict, compare=False)


class EventManager:
    """Chronological event queue with deterministic tie breaking."""

    def __init__(self) -> None:
        self._queue: list[Event] = []
        self._sequence = 0

    def push(self, time: float, event_type: EventType, **payload: Any) -> None:
        self._sequence += 1
        heapq.heappush(
            self._queue, Event(float(time), self._sequence, event_type, payload)
        )

    def pop(self) -> Event:
        return heapq.heappop(self._queue)

    def __bool__(self) -> bool:
        return bool(self._queue)

