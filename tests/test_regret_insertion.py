from math import inf

from uav_planning.routing.insertion import regret2_value


def test_regret_prioritizes_task_with_single_feasible_insertion() -> None:
    single_option_regret = regret2_value([12.0])
    ordinary_regret = regret2_value([10.0, 14.0, 20.0])
    assert single_option_regret == inf
    assert single_option_regret > ordinary_regret
