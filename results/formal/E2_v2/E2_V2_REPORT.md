# E2_v2 正式实验报告

## 1. 实验定位

E2_v2 将论文主比较更新为：

- `time_aware`：优化 `weighted_delay`；
- `mission_completion`：优化 `makespan = max_k(return_time_k)`；
- `distance_oriented`：优化 `total_travel_time`，仅作为结构性补充对照。

三种 variant 均使用 Dubins planning、Dubins execution、Local strategy、R=10、3 UAV、10 个初始任务、10 个动态任务以及 seeds 1000-1029。同一 seed 的任务、UAV 和地图数据完全相同。

原 `results/formal/E2/raw.csv` 未修改，其 SHA-256 在本轮运行前后均为：

`75CD5D083F763064BDBCF9AA59875CF22121DBCB8BE7DBA3AAFE11E15E0C8DA8`

旧 E2 只保留为“纯总飞行时间目标会主动合并任务”的补充/结构性对照，不再作为唯一核心 baseline。

## 2. 实现与测试

- `RouteOptimizer` 新增 `objective_mode="makespan"`，目标是所有 UAV 路线返航时间的最大值。
- makespan 目标不直接使用 priority 或 per-task weighted delay。
- Simulator 新增 `active_uav_count`、`max_tasks_per_uav` 和 `min_tasks_per_active_uav`。
- 对外报告的 `makespan` 与新目标定义一致，包含最终返航。
- `pytest -q`：44 passed。
- E2_v2 dry-run：90 runs。
- 正式运行：90/90 runs，无重复主键。
- feasibility：90/90；time consistency：90/90。
- 30 个 paired seed 的三种 variant 均通过场景指纹一致性检查。

## 3. Seed 级描述统计

以下区间均为均值的正态近似 95% CI。

### 3.1 Time-aware

| metric | mean | std | median | 95% CI |
|---|---:|---:|---:|---:|
| weighted mean delay | 19.2171 | 1.7886 | 18.9466 | [18.5771, 19.8572] |
| high-priority mean delay | 16.2656 | 2.1468 | 16.5556 | [15.4973, 17.0338] |
| mean delay | 23.1631 | 2.6279 | 22.8242 | [22.2227, 24.1035] |
| makespan | 88.1253 | 4.6422 | 88.0869 | [86.4641, 89.7865] |
| total travel time | 168.1541 | 12.8821 | 166.2878 | [163.5443, 172.7640] |

### 3.2 Mission-completion

| metric | mean | std | median | 95% CI |
|---|---:|---:|---:|---:|
| weighted mean delay | 23.5402 | 2.8162 | 23.0533 | [22.5325, 24.5480] |
| high-priority mean delay | 23.4994 | 4.1186 | 22.8699 | [22.0256, 24.9732] |
| mean delay | 23.7909 | 2.8245 | 23.6113 | [22.7801, 24.8016] |
| makespan | 85.6988 | 4.8890 | 85.9162 | [83.9493, 87.4483] |
| total travel time | 163.6416 | 12.8002 | 163.1548 | [159.0611, 168.2221] |

### 3.3 Distance-oriented（补充）

| metric | mean | std | median | 95% CI |
|---|---:|---:|---:|---:|
| weighted mean delay | 75.3379 | 6.7574 | 75.6041 | [72.9198, 77.7560] |
| high-priority mean delay | 75.1692 | 12.5168 | 76.4376 | [70.6901, 79.6483] |
| mean delay | 75.7596 | 6.3405 | 75.2452 | [73.4907, 78.0285] |
| makespan | 181.1667 | 9.4353 | 181.6548 | [177.7903, 184.5430] |
| total travel time | 101.7957 | 7.8143 | 98.8305 | [98.9994, 104.5920] |

## 4. 主比较：time-aware vs mission-completion

相对差异按 `(time_aware - mission_completion) / mission_completion * 100%` 计算；延迟、飞行时间和 makespan 均为越低越好。

| metric | mean relative difference | 95% CI | win / tie / loss | Wilcoxon W | two-sided p |
|---|---:|---:|---:|---:|---:|
| weighted mean delay | -17.9451% | [-20.0096%, -15.8806%] | 30 / 0 / 0 | 0 | 1.8626e-09 |
| high-priority mean delay | -29.3852% | [-33.6416%, -25.1287%] | 30 / 0 / 0 | 0 | 1.8626e-09 |
| total travel time | +2.9609% | [+0.7278%, +5.1940%] | 8 / 0 / 22 | 110 | 0.0105983 |
| makespan | +2.9835% | [+1.1197%, +4.8473%] | 10 / 0 / 20 | 111 | 0.0113031 |

在该冻结设计下，time-aware 的两个响应时效指标在 30/30 paired seeds 中均低于 mission-completion；相应地，其平均总飞行时间和平均 makespan 较高。该表直接呈现时效收益与整体任务成本之间的 trade-off，不引入多目标权重。

## 5. 纯距离目标的资源使用行为

| variant | active UAV mean | active UAV range | max tasks/UAV mean | min tasks/active UAV mean |
|---|---:|---:|---:|---:|
| time_aware | 3.0 | 3-3 | 7.2667 | 5.8667 |
| mission_completion | 3.0 | 3-3 | 7.2667 | 5.9000 |
| distance_oriented | 1.0 | 1-1 | 20.0000 | 20.0000 |

纯总飞行时间目标在 30/30 seeds 中均只使用 1 架 UAV 完成全部 20 个任务；time-aware 和 mission-completion 在 30/30 seeds 中均实际使用全部 3 架 UAV。没有添加强制负载均衡或“必须使用全部 UAV”的约束。

这说明旧 E2 的巨大时效差异部分来自纯距离目标在允许空路线时的任务集中行为，因此旧结果仅用于补充解释，不作为主论文比较的唯一证据。

## 6. 完整性与异常记录

- 没有不可行 seed。
- 没有时间不一致 seed。
- 没有场景指纹不一致 seed。
- weighted mean delay 和 high-priority mean delay 均无反向 seed。
- total travel time 存在 8 个 seed 的 time-aware 值低于 mission-completion；makespan 存在 10 个。它们均保留在 raw/paired CSV 中，没有删样本或调参。

## 7. 输出

- `results/formal/E2_v2/raw.csv`
- `results/formal/E2_v2/summary.csv`
- `results/formal/E2_v2/paired.csv`
- `results/formal/E2_v2/wilcoxon.csv`
- `results/formal/E2_v2/ENVIRONMENT.md`
- `results/formal/figures/time_objective_tradeoff_v2.png`
- `results/formal/figures/distance_objective_resource_usage.png`

## 8. 停止点

E2_v2 已完成。本轮未重跑 E1、E3、E4，未运行 E5。
