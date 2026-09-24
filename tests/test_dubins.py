from math import pi

import pytest

from uav_planning.geometry.dubins import dubins_shortest_path_length
from uav_planning.models import Pose2D


def test_identical_pose_has_zero_length() -> None:
    pose = Pose2D(2.0, 3.0, 0.7)
    assert dubins_shortest_path_length(pose, pose, 5.0) == pytest.approx(0.0)


def test_straight_same_heading_matches_distance() -> None:
    length = dubins_shortest_path_length(
        Pose2D(0.0, 0.0, 0.0), Pose2D(12.0, 0.0, 0.0), 3.0
    )
    assert length == pytest.approx(12.0)


def test_time_reversal_symmetry() -> None:
    start = Pose2D(1.0, 2.0, 0.3)
    goal = Pose2D(17.0, 9.0, 2.0)
    forward = dubins_shortest_path_length(start, goal, 4.0)
    reverse = dubins_shortest_path_length(
        Pose2D(goal.x, goal.y, goal.heading + pi),
        Pose2D(start.x, start.y, start.heading + pi),
        4.0,
    )
    assert forward == pytest.approx(reverse)


def test_larger_radius_not_shorter_for_turning_case() -> None:
    start = Pose2D(0.0, 0.0, 0.0)
    goal = Pose2D(10.0, 10.0, pi / 2.0)
    assert dubins_shortest_path_length(start, goal, 6.0) >= (
        dubins_shortest_path_length(start, goal, 2.0) - 1e-9
    )


@pytest.mark.parametrize("radius", [0.5, 2.0, 10.0])
def test_result_is_finite_and_nonnegative(radius: float) -> None:
    value = dubins_shortest_path_length(
        Pose2D(-1.0, 4.0, 5.8), Pose2D(8.0, -3.0, 1.2), radius
    )
    assert value >= 0.0
    assert value < float("inf")

