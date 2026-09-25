# 第一阶段 MVP 验收修正报告

## 修正结论

本轮完成了动态重规划时间一致性修正、核心锁定规则统一、Local Replanning 机制调整、算法术语更正和 Regret-2 单可行位置修正。当前代码已通过针对时间回溯反例的自动化测试和端到端 sanity check，但这些结果仍只用于工程验收，不作为论文实验结论。

验证环境：Python 3.13.5。

```text
python -m pytest
19 passed

python scripts/run_single.py --config configs/small_debug.yaml
TIME_CONSISTENCY_CHECK: PASS
```

## P0：在线时间一致性

旧实现会把动态候选路线从 Depot、`t=0` 全量重算，允许自由任务在重规划事件之前“重新执行”。新实现将仿真状态拆为：

- 不可变的 `TaskExecutionRecord` 历史；
- 已经开始、不可抢占的飞行目标或服务任务；
- 不可中断的返航动作；
- 每架 UAV 的未来 `PlanningAnchor(anchor_time, anchor_pose)`；
- 只包含尚未开始任务的自由计划。

每个 release event 的处理顺序为：

1. 从上一个事件向前推进到当前事件；
2. 将已完成任务写入不可变历史；
3. 锁定已经开始的飞行段或服务任务；
4. 为自由后缀计算未来锚点；
5. 只对自由任务调用重规划器；
6. 验证新计划中所有自由任务 `service_start >= event_time`；
7. 最终指标直接使用累计执行历史。

返航中的 UAV 不被中断；其未来锚点为原计划抵达 Depot 的时刻和 Depot pose。已经返航且空闲的 UAV 使用当前事件时刻作为锚点。

新增测试：

- `test_replanning_cannot_schedule_pending_task_in_the_past`；
- `test_completed_history_is_unchanged_after_later_replanning`。

其中第一项覆盖 `t=10` 重规划、空闲 UAV 接收另一架 UAV 的旧 pending task 的确定反例，并断言开始时间不小于 10。

## 统一锁定规则

默认 `commitment_horizon = 0`。NoReorder、Full 和 Local 共同冻结：

- 已完成任务；
- 当前已经开始飞往的任务；
- 当前正在服务的任务；
- 已经开始的返航动作。

Local 不再额外冻结“下一任务”。非抢占动作由 Simulator 在进入所有重规划器之前统一移出自由计划，因此三种策略面对完全相同的物理锁定边界。

## Local 与 NoReorder 的关系

Local 当前实现为 `best insertion + restricted suffix local improvement`：

1. 调用与 NoReorder 相同的最佳可行新任务插入；
2. 将该插入解作为 incumbent；
3. 按各 UAV 的最佳插入目标值选择 `h` 架 affected UAV，且必含最佳插入 UAV；
4. 只在 affected UAV 的自由后缀使用 relocate、cross-route relocate、swap 和 2-opt；
5. 只接受目标值严格改善的候选；
6. 非 affected UAV 路线保持不变。

因此在同一事件、同一执行历史和同一组锚点下，Local 的事件后计划不会比 NoReorder incumbent 更差。不同策略经过多个事件后可能产生不同执行历史，因此整场最终目标不具备逐 seed 的静态支配保证。

Regret-2 只用于 InitialPlanner 和 FullReplanner。Local 不再释放任务池后重新做 Regret-2 构造。

## P1：算法名称和 Regret-2

旧实现没有 shaking，不属于经典 VNS。代码和文档已统一使用：

> Multi-Neighborhood Local Search / 多邻域局部搜索

内部函数已由 `_vns` 改为 `local_search`，配置项改为 `local_search_max_iterations` 和 `local_search_time_limit_sec`。

当某任务只有一个可行插入位置时，`regret2_value` 返回正无穷，使其获得最高插入优先级。新增测试：

- `test_regret_prioritizes_task_with_single_feasible_insertion`。

## seed=42 修正后输出

| strategy | weighted_delay | assignment_changes | TIME_CONSISTENCY_CHECK |
|---|---:|---:|---|
| no_reorder | 347.260661 | 0 | PASS |
| full | 334.222419 | 2 | PASS |
| local | 334.222419 | 2 | PASS |

所有任务完成且只完成一次，服务开始不早于 release time，最终返航满足续航约束。

## replanning_debug：10 seeds sanity check

配置：3 UAV、8 initial、8 dynamic、`h=2`，seeds 40-49。

| strategy | mean weighted_delay | mean replanning runtime (s) | total assignment changes | total successor changes |
|---|---:|---:|---:|---:|
| no_reorder | 1364.395832 | 0.001971 | 0 | 0 |
| full | 1338.826914 | 0.020567 | 44 | 49 |
| local | 1332.498391 | 0.008654 | 28 | 30 |

诊断结论仅限代码行为：

- 30/30 策略运行均可行且通过时间一致性检查；
- Local 在 8/10 seeds 严格优于整场 NoReorder，1 个相同，1 个因跨事件历史路径依赖略差；
- Local 在 10/10 seeds 位于 Full 最终目标的 5% 以内；
- Local 确实产生 assignment/successor 调整，且累计修改量小于 Full；
- 每个 Local 重规划调用都保证不劣于相同状态下的 NoReorder insertion incumbent。

这些数据是 sanity check，不具备正式样本设计、统计检验或论文结论资格。

## 输出文件

- `results/raw/seed42_metrics.csv`
- `results/raw/seed42_<strategy>_events.csv`
- `results/summary/seed42_summary.csv`
- `results/summary/TIME_CONSISTENCY_CHECK.txt`
- `results/raw/replanning_debug_10seeds_metrics.csv`
- `results/summary/replanning_debug_10seeds_summary.csv`
- `results/figures/debug_seed42.png`

## 保留限制与下一步边界

- Debug 图仍为访问顺序折线；正式论文图需要新增纯绘图用途的 `sample_dubins_path`。
- 正式主实验前仍需加入 5-8 tasks、2-3 UAV 的枚举或开源 MILP 精确基准。
- Simple EO sweep 尚未实现；当前仍使用 synthetic release。
- 不开始 20/50/100 任务正式实验。
- 不加入 DRL/MARL、GA/ACO、动态障碍、天气、故障、通信、GUI、数据库、Web 服务或新的加权目标。
