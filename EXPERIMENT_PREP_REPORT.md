# 实验前准备报告

## 1. 本轮完成范围

本轮完成正式规模实验前的四个阻塞项：

1. 小规模 Exact Enumerator；
2. planning travel model 与 physical execution model 解耦；
3. `weighted_delay` 与 `total_travel_time` 目标切换；
4. E1-E5 实验配置与统一结果字段冻结。

同时完成 Dubins 路径采样和真实曲线 debug 绘图。未运行 20/50/100 任务正式实验，所有本轮数值只用于代码与数据管线验收。

## 2. Exact Enumerator

实现位置：`src/uav_planning/exact/enumerator.py`。

枚举过程：

1. 枚举全部任务排列；
2. 枚举任务数量到有标签 UAV 的全部 weak compositions；
3. 按切分长度生成每架 UAV 的有序路线，允许空路线；
4. 使用现有 `RouteOptimizer.evaluate_plan`、`RouteEvaluator`、目标模式和可选 `PlanningAnchor` 统一评价；
5. 保留全局最优可行计划。

安全限制：默认 `task_count <= 8`、`uav_count <= 3`，超过限制立即抛出异常。没有引入商业求解器或新的运行依赖。

## 3. 小规模精确基准

命令：

```text
python scripts/run_exact_benchmark.py
```

配置：2 UAV，5/6/7 tasks，各 10 seeds，Dubins planning，主目标为 weighted delay。

| tasks | exact objective mean | heuristic objective mean | mean gap | max gap | exact runtime mean | heuristic runtime mean |
|---:|---:|---:|---:|---:|---:|---:|
| 5 | 366.457028 | 366.486166 | 0.0077% | 0.0768% | 0.0394 s | 0.0046 s |
| 6 | 476.445469 | 483.332366 | 1.2653% | 6.3013% | 0.3113 s | 0.0077 s |
| 7 | 599.564593 | 603.222213 | 0.7297% | 3.4554% | 2.8198 s | 0.0128 s |

输出：

- `results/exact/exact_benchmark_raw.csv`；
- `results/exact/exact_benchmark_summary.csv`。

这些数据只证明枚举器和启发式没有明显正确性异常，不是正式论文规模实验。

## 4. Planning 与 Execution 解耦

Simulator 当前接口明确接收：

```python
Simulator(
    scenario,
    initial_planner,
    planning_optimizer,
    execution_evaluator,
)
```

职责边界：

- planner 候选排序、Regret-2 和多邻域局部搜索使用 `planning_optimizer`；
- 事件推进、任务开始/完成、返航、真实 anchor、真实 travel time 和最终执行指标使用 `execution_evaluator`；
- 未显式传入 execution evaluator 时，默认复用 planning evaluator，保持 Dubins/Dubins 旧行为。

输出数据新增：

- `planning_travel_model`；
- `execution_travel_model`；
- `objective_mode`。

## 5. Euclidean baseline 的物理含义

E3 两个对照均按 Dubins 执行：

- Dubins-aware：Dubins planning + Dubins execution；
- Euclidean-planned：Euclidean planning + Dubins execution。

因此 Euclidean 只代表规划器对旅行时间的近似认知。实际执行历史、返航时间和 `total_travel_time` 均来自 Dubins evaluator，不会用欧氏时间推进固定翼。

## 6. 目标函数切换

`RouteOptimizer(objective_mode=...)` 支持：

- `weighted_delay`：`sum w_j(C_j-r_j)`；
- `total_travel_time`：所有 UAV 计划旅行时间之和，包含最终返航，不使用 priority。

这只是两个单目标模式，没有引入多目标权重。无论规划使用哪个模式，Simulator 都会基于真实执行历史统一输出论文主评价指标 `weighted_delay`、高优先级平均延迟、平均延迟和真实总旅行时间。

## 7. 新增测试

测试总数由 19 增至 33，新增覆盖：

- handcrafted exact optimum；
- exact objective 不劣于启发式；
- exact 大实例保护；
- Euclidean planning + Dubins execution；
- 执行历史使用物理模型；
- 默认 Dubins/Dubins 行为保持；
- total-travel planning score 忽略 priority；
- 距离目标仍报告 weighted-delay metric；
- Dubins 采样终点位置与航向。
- E1-E5 五份冻结配置均可被配置加载器读取。

结果：`33 passed`。

## 8. 三类小 debug 数据管线

命令：

```text
python scripts/run_experiment_debugs.py
```

每类使用 seeds 42-44，配置为 3 UAV、8 initial + 8 dynamic。全部运行均满足 feasibility 和 time consistency。

| experiment | variant | mean weighted delay | mean high-priority delay | mean actual travel time |
|---|---|---:|---:|---:|
| E2 | time_aware | 1309.227727 | 14.076923 | 129.997478 |
| E2 | distance_oriented | 3830.246407 | 41.134468 | 65.791659 |
| E3 | dubins_aware | 1309.227727 | 14.076923 | 129.997478 |
| E3 | euclidean_planned | 1332.503397 | 13.579647 | 147.676657 |
| E4 | no_reorder | 1351.865112 | 14.715719 | 140.698965 |
| E4 | full | 1317.958273 | 14.053732 | 137.888341 |
| E4 | local | 1309.227727 | 14.076923 | 129.997478 |

输出：

- `results/debug/experiment_debug_raw.csv`；
- `results/debug/experiment_debug_summary.csv`。

三 seeds 太少，且参数仅为管线检查，不得据此形成论文结论或预设方法优劣。

## 9. Dubins 绘图

`sample_dubins_path` 与 `dubins_shortest_path_length` 使用同一个最短 path family，按给定步长返回 `(x, y, heading)`。当前 debug 路线图已绘制真实转弯曲线；该采样函数仅用于可视化，不进入优化计算。

## 10. 已冻结但尚未运行的正式实验

配置模板位于 `configs/experiments/`：

- E1：5/6/7 tasks exact benchmark；
- E2：time-aware vs distance-oriented；
- E3：Euclidean-planned vs Dubins-aware；
- E4：NoReorder vs Full vs Local；
- E5：20/50/100 tasks 与 3/5/8 UAV scale 模板。

E5 的正式 seed 数仍为 `null`，状态为 `frozen_template_do_not_run`。本轮没有启动任何正式规模批处理。

## 11. 当前限制

- Exact 使用直接枚举，只适合不超过 8 tasks、3 UAV；
- 尚未实现 MILP 交叉验证；
- 尚未统计 E3 的 predicted-vs-actual timing error，该字段仍为可选项；
- `simple_eo_sweep` 尚未实现，synthetic release 继续作为可控实验默认；
- Dubins 路径未加入地图边界、障碍或风场约束；
- 本轮 debug 与 exact 数据不具备正式统计结论资格。

## 12. 下一阶段建议

1. 由导师或下一轮验收冻结正式 seed 数、地图尺度和到达率；
2. 决定是否在 E3 增加 predicted-vs-actual timing error；
3. 先做正式实验执行预算估计，再运行 E2-E4；
4. 最后才开放 E5 规模实验；
5. EO sweep 仅在需要论文场景示意图时补充，不改变主实验 synthetic release。
