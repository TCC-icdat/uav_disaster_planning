"""Pure-Python analytic Dubins shortest-path length.

The six normalized word formulas follow the standard Dubins construction
described by Shkel and Lumelsky (2001), "Classification of the Dubins set".
Only path length is required by the optimizer, so no external geometry library
is needed. Sampling uses the same selected word and is visualization-only.
"""

from __future__ import annotations

from collections.abc import Callable
from math import acos, atan2, cos, hypot, isfinite, pi, sin, sqrt

import numpy as np

from uav_planning.models import Pose2D

_TAU = 2.0 * pi
_EPS = 1e-12
_Word = tuple[float, float, float] | None


def _mod2pi(value: float) -> float:
    return value % _TAU


def _sqrt_nonnegative(value: float) -> float | None:
    if value < -_EPS:
        return None
    return sqrt(max(0.0, value))


def _lsl(alpha: float, beta: float, distance: float) -> _Word:
    p = _sqrt_nonnegative(
        2.0
        + distance**2
        - 2.0 * cos(alpha - beta)
        + 2.0 * distance * (sin(alpha) - sin(beta))
    )
    if p is None:
        return None
    tmp = atan2(cos(beta) - cos(alpha), distance + sin(alpha) - sin(beta))
    return _mod2pi(-alpha + tmp), p, _mod2pi(beta - tmp)


def _rsr(alpha: float, beta: float, distance: float) -> _Word:
    p = _sqrt_nonnegative(
        2.0
        + distance**2
        - 2.0 * cos(alpha - beta)
        + 2.0 * distance * (-sin(alpha) + sin(beta))
    )
    if p is None:
        return None
    tmp = atan2(cos(alpha) - cos(beta), distance - sin(alpha) + sin(beta))
    return _mod2pi(alpha - tmp), p, _mod2pi(-beta + tmp)


def _lsr(alpha: float, beta: float, distance: float) -> _Word:
    p = _sqrt_nonnegative(
        -2.0
        + distance**2
        + 2.0 * cos(alpha - beta)
        + 2.0 * distance * (sin(alpha) + sin(beta))
    )
    if p is None:
        return None
    tmp = atan2(-cos(alpha) - cos(beta), distance + sin(alpha) + sin(beta))
    tmp -= atan2(-2.0, p)
    return _mod2pi(-alpha + tmp), p, _mod2pi(-beta + tmp)


def _rsl(alpha: float, beta: float, distance: float) -> _Word:
    p = _sqrt_nonnegative(
        -2.0
        + distance**2
        + 2.0 * cos(alpha - beta)
        - 2.0 * distance * (sin(alpha) + sin(beta))
    )
    if p is None:
        return None
    tmp = atan2(cos(alpha) + cos(beta), distance - sin(alpha) - sin(beta))
    tmp -= atan2(2.0, p)
    return _mod2pi(alpha - tmp), p, _mod2pi(beta - tmp)


def _rlr(alpha: float, beta: float, distance: float) -> _Word:
    value = (
        6.0
        - distance**2
        + 2.0 * cos(alpha - beta)
        + 2.0 * distance * (sin(alpha) - sin(beta))
    ) / 8.0
    if value < -1.0 - _EPS or value > 1.0 + _EPS:
        return None
    p = _mod2pi(_TAU - acos(max(-1.0, min(1.0, value))))
    tmp = atan2(cos(alpha) - cos(beta), distance - sin(alpha) + sin(beta))
    t = _mod2pi(alpha - tmp + p / 2.0)
    return t, p, _mod2pi(alpha - beta - t + p)


def _lrl(alpha: float, beta: float, distance: float) -> _Word:
    value = (
        6.0
        - distance**2
        + 2.0 * cos(alpha - beta)
        + 2.0 * distance * (-sin(alpha) + sin(beta))
    ) / 8.0
    if value < -1.0 - _EPS or value > 1.0 + _EPS:
        return None
    p = _mod2pi(_TAU - acos(max(-1.0, min(1.0, value))))
    tmp = atan2(cos(alpha) - cos(beta), distance + sin(alpha) - sin(beta))
    t = _mod2pi(-alpha - tmp + p / 2.0)
    return t, p, _mod2pi(beta - alpha - t + p)


_WORDS: tuple[tuple[str, Callable[[float, float, float], _Word]], ...] = (
    ("LSL", _lsl),
    ("RSR", _rsr),
    ("LSR", _lsr),
    ("RSL", _rsl),
    ("RLR", _rlr),
    ("LRL", _lrl),
)


def _shortest_word(
    start: Pose2D, goal: Pose2D, turning_radius: float
) -> tuple[str, tuple[float, float, float]]:
    if not isfinite(turning_radius) or turning_radius <= 0.0:
        raise ValueError("turning_radius must be finite and positive")
    dx = goal.x - start.x
    dy = goal.y - start.y
    euclidean = hypot(dx, dy)
    heading_error = abs((_mod2pi(goal.heading - start.heading + pi)) - pi)
    if euclidean <= _EPS and heading_error <= _EPS:
        return "SSS", (0.0, 0.0, 0.0)
    theta = _mod2pi(atan2(dy, dx))
    alpha = _mod2pi(start.heading - theta)
    beta = _mod2pi(goal.heading - theta)
    normalized_distance = euclidean / turning_radius
    candidates = [
        (sum(word), name, word)
        for name, solver in _WORDS
        if (word := solver(alpha, beta, normalized_distance)) is not None
    ]
    if not candidates:
        raise RuntimeError("no finite Dubins path found")
    _, name, word = min(candidates, key=lambda item: item[0])
    return name, word


def dubins_shortest_path_length(
    start: Pose2D,
    goal: Pose2D,
    turning_radius: float,
) -> float:
    """Return the shortest forward-only Dubins path length.

    Args:
        start: Initial position and heading.
        goal: Terminal position and heading.
        turning_radius: Positive minimum turning radius.
    """

    _, parameters = _shortest_word(start, goal, turning_radius)
    return sum(parameters) * turning_radius


def _advance_segment(
    x: float,
    y: float,
    heading: float,
    segment_type: str,
    distance: float,
    radius: float,
) -> tuple[float, float, float]:
    if segment_type == "S":
        return (
            x + distance * cos(heading),
            y + distance * sin(heading),
            heading,
        )
    angle = distance / radius
    if segment_type == "L":
        new_heading = heading + angle
        return (
            x + radius * (sin(new_heading) - sin(heading)),
            y + radius * (cos(heading) - cos(new_heading)),
            _mod2pi(new_heading),
        )
    if segment_type == "R":
        new_heading = heading - angle
        return (
            x + radius * (sin(heading) - sin(new_heading)),
            y + radius * (cos(new_heading) - cos(heading)),
            _mod2pi(new_heading),
        )
    raise ValueError(f"unknown Dubins segment type: {segment_type}")


def sample_dubins_path(
    start: Pose2D,
    goal: Pose2D,
    turning_radius: float,
    step_size: float = 0.5,
) -> np.ndarray:
    """Sample the selected shortest Dubins word as ``(x, y, heading)`` rows."""

    if not isfinite(step_size) or step_size <= 0.0:
        raise ValueError("step_size must be finite and positive")
    word, parameters = _shortest_word(start, goal, turning_radius)
    x, y, heading = start.x, start.y, start.heading
    samples = [(x, y, heading)]
    for segment_type, parameter in zip(word, parameters):
        remaining = parameter * turning_radius
        while remaining > _EPS:
            distance = min(step_size, remaining)
            x, y, heading = _advance_segment(
                x, y, heading, segment_type, distance, turning_radius
            )
            samples.append((x, y, heading))
            remaining -= distance
    samples[-1] = (goal.x, goal.y, goal.heading)
    return np.asarray(samples, dtype=float)

