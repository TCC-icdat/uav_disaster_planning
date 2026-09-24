# 第一阶段实现报告

## 完成状态

第一阶段 MVP 已形成可运行闭环：固定 seed 场景生成、SAR 初始计划、两个动态任务释放、三种重规划策略、CSV 指标、离散事件日志、路线图和自动化测试。

验证环境：Python 3.13.5。验证命令与结果：

```text
python -m pytest
15 passed

python scripts/run_single.py --config configs/small_debug.yaml
seed = 42
strategies = no_reorder, full, local
```

## 已完成模块

- `models.py`：Pose2D、Task、UAV、Route、Plan、UAVExecutionState、SimulationState。
- `geometry/dubins.py`：LSL / RSR / LSR / RSL / RLR / LRL 纯 Python 最短路径长度。
- `routing/evaluator.py`：Euclidean / Dubins provider 和唯一权威 RouteEvaluator。
- `scenario/`：统一 `numpy.random.Generator` 的 synthetic release 场景与事件源接口。
- `routing/insertion.py`：Regret-2 构造和共享 VNS 引擎。
- `routing/neighborhoods.py`：四个冻结邻域。
- `planners/`：Initial、NoReorder、Full、Local。
- `simulation/`：TASK_RELEASE 驱动的重规划和完整执行事件日志。
- `metrics/`：加权响应延迟、均值、makespan、运行时间和计划扰动。
- `visualization/`：初始路线与 Local 最终路线对比图。

## 与数学模型的对应关系

| 数学定义 | 代码位置 |
|---|---|
| 任务 `(p_j, r_j, w_j, s_j, phi_j)` | `Task` |
| UAV `(v_k, R_k, H_k)` | `UAV` |
| `tau_ij^k = L_Dubins / v_k` | `DubinsTravelTimeProvider` |
| `S_j >= r_j` 与时间递推 | `RouteEvaluator.evaluate_route` |
| 每任务唯一分配 | `Simulator._validate_task_set` |
| 最大任务时间与返航 | `RouteEvaluation.feasible` |
| `sum w_j(C_j-r_j)` | `weighted_response_delay` |
| 执行前缀锁定 | `Simulator._build_state` 与动态 planners |
| Local 只释放 affected UAV 后缀 | `LocalReplanner.replan` |

## seed=42 验收结果

本次固定输出中的加权响应延迟：

| strategy | weighted_delay | replanning_count | feasible |
|---|---:|---:|---|
| no_reorder | 347.260661 | 2 | True |
| full | 334.222419 | 2 | True |
| local | 364.847349 | 2 | True |

运行时间记录在 `results/raw/seed42_metrics.csv`。运行时间是机器相关量，每次执行会略有波动，不作为固定快照值写入本报告。

## 当前简化假设

- EO 只产生 synthetic release，不模拟图像识别或真实覆盖轨迹。
- SAR 服务为任务点、指定进入航向和服务时长，不建模条带 entry/exit 段。
- 动态时刻通过冻结已完成、正在服务或正在飞往的任务前缀避免回滚；优化仍对完整路线进行确定性重评估。
- Debug 图使用任务点之间的示意折线；优化代价使用真实的解析 Dubins 长度。
- 同类 SAR 在默认配置中参数相同，但数据结构允许逐机设置速度、转弯半径和续航。

## 已知限制

- 尚未实现 Simple EO sweep、连续 Dubins 轨迹采样或静态障碍边代价。
- 尚未加入 5-8 任务的穷举/MILP 正确性基准。
- `small_debug` 只有 2 架 UAV 且 Local 默认 `h=2`，因此受影响 UAV 数量在该场景中等于全部机队；Local 的范围优势需在后续 3/5/8 架实验中评估。
- 当前事件日志在最终锁定一致计划上重建 TASK_START / COMPLETE / RETURN；不执行固定时间步飞行动力学积分。
- 单 seed 结果不能用于论文统计结论。

## 下一阶段建议

1. 增加 5-8 个任务的穷举或开源 MILP 基准，验证启发式目标差距。
2. 在冻结当前 MVP 后，设计 20/50/100 任务与 3/5/8 UAV 的批量实验矩阵。
3. 分别运行 Euclidean 与 Dubins provider，报告计划误差和实际飞行时间差异。
4. 对 `h` 和 commitment horizon 做敏感性实验，并给出统计显著性与置信区间。

## 禁止擅自扩展

在第一阶段结果稳定前，不加入 DRL/MARL、GA/ACO、动态障碍、天气、UAV 故障、通信模型、复杂 GUI、数据库、Web 服务、六自由度动力学或新的加权目标；总距离继续只作为评价指标，不并入主目标。

