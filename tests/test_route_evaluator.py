import pytest

from uav_planning.models import Pose2D, Task, UAV
from uav_planning.routing.evaluator import EuclideanTravelTimeProvider, RouteEvaluator


def test_release_time_and_return_are_enforced() -> None:
    uav = UAV(1, 10.0, 2.0, 20.0, Pose2D(0.0, 0.0, 0.0))
    task = Task(1, 10.0, 0.0, 5.0, 4.0, 2.0, 0.0)
    result = RouteEvaluator(EuclideanTravelTimeProvider()).evaluate_route(uav, [task])
    assert result.arrival_times[1] == pytest.approx(1.0)
    assert result.start_times[1] == pytest.approx(5.0)
    assert result.completion_times[1] == pytest.approx(7.0)
    assert result.return_time == pytest.approx(8.0)
    assert result.feasible


def test_endurance_infeasibility_is_reported() -> None:
    uav = UAV(1, 1.0, 2.0, 5.0, Pose2D(0.0, 0.0, 0.0))
    task = Task(1, 10.0, 0.0, 0.0, 1.0, 1.0, 0.0)
    result = RouteEvaluator(EuclideanTravelTimeProvider()).evaluate_route(uav, [task])
    assert not result.feasible

