# 灾害应急异构固定翼无人机动态任务规划 MVP

本项目实现“EO 初筛产生动态重点复核任务，SAR 固定翼编队进行任务分配、排序和事件触发重规划”的第一阶段可复现实验闭环。EO 在 MVP 中被抽象为 synthetic task event generator；SAR 是唯一在线优化对象。

## 冻结的研究定义

- 主目标：`sum(priority_j * (completion_time_j - release_time_j))`。
- 所有已释放任务必须且只执行一次。
- 服务开始时刻不得早于任务释放时刻。
- 固定翼旅行时间由纯 Python Dubins 最短路径长度计算，支持 LSL、RSR、LSR、RSL、RLR、LRL。
- 动态策略为 `no_reorder`、`full` 和 `local`。
- Full 与 Local 共用同一个 `RouteEvaluator` 和四个多邻域局部搜索算子。
- 路线扰动只作为实验指标，不进入主目标。
- 规划旅行模型与物理执行模型独立配置；真实执行历史始终由 execution model 推进。
- 规划目标可切换为 `weighted_delay` 或仅用于对照的 `total_travel_time`。

## 工程结构

```text
configs/                 YAML 参数
src/uav_planning/        模型、几何、场景、路由、规划器、仿真、指标和绘图
scripts/run_single.py    seed=42 三策略验收入口
scripts/run_batch.py     小批量种子入口
scripts/run_exact_benchmark.py  小规模精确枚举基准
scripts/run_experiment_debugs.py E2/E3/E4 三种数据管线检查
scripts/plot_results.py  结果柱状图
tests/                   正确性、约束和锁定规则测试
results/                 CSV、事件日志和路线图
IMPLEMENTATION_REPORT.md 实现与模型对应报告
```

## 环境

要求 Python 3.11+。依赖见 `requirements.txt`。当前开发机验证使用 Python 3.13.5。

```bash
python -m pip install -r requirements.txt
```

若 Windows 上默认 `python` 不是 3.11+，请显式调用较新的解释器。例如本项目当前机器可用：

```powershell
& 'D:\Anaconda\python.exe' -m pytest
& 'D:\Anaconda\python.exe' scripts\run_single.py --config configs\small_debug.yaml
```

## 验收运行

```bash
pytest -q
python scripts/run_single.py --config configs/small_debug.yaml
python scripts/run_exact_benchmark.py
python scripts/run_experiment_debugs.py
```

输出包括：

- `results/raw/seed42_metrics.csv`
- `results/raw/seed42_<strategy>_events.csv`
- `results/summary/seed42_summary.csv`
- `results/summary/TIME_CONSISTENCY_CHECK.txt`
- `results/figures/debug_seed42.png`
- `results/exact/exact_benchmark_raw.csv`
- `results/exact/exact_benchmark_summary.csv`
- `results/debug/experiment_debug_raw.csv`
- `results/debug/experiment_debug_summary.csv`

`small_debug.yaml` 固定为 2 架 SAR、3 个初始任务、2 个动态任务、50 x 50 地图和 seed 42。动态释放区间刻意设置在初始航线执行期间，以实际触发锁定和重规划。

## 算法说明

初始规划使用 Regret-2 insertion 构造可行解，再用 first-improvement 多邻域局部搜索改进。局部搜索仅包含：

1. route 内 relocate；
2. route 间 relocate；
3. route 间 swap；
4. route 内 2-opt。

每个候选解都通过统一的 `RouteEvaluator` 检查 release time、服务时间、返航和最大任务时间。动态事件按 `TASK_RELEASE` 的时间顺序处理；最终执行日志包含 `TASK_START`、`TASK_COMPLETE` 和 `RETURN_DEPOT`。

## 在线时间一致性

仿真在每个 release event 前先推进真实执行状态。已完成任务写入不可变历史；已经开始的飞行段或服务任务不可抢占。每架 UAV 的自由后缀从 `(anchor_time, anchor_pose)` 开始评价：空闲 UAV 的锚点为当前事件时刻，返航中的 UAV 则从原计划抵达 Depot 后继续。最终指标直接来自累计执行历史，不再把最终路线从 `t=0` 重算。

核心对比统一采用 `commitment_horizon: 0`，三种策略共同冻结且只冻结：已完成任务、当前飞行目标和当前服务任务。Local 不额外冻结“下一任务”。

Local 先生成与 NoReorder 完全相同的最佳插入 incumbent，再选择包含最佳插入 UAV 的 `h` 架 affected UAV，只在这些 UAV 的自由后缀上接受严格改善。因此同一个事件、同一组锚点下，Local 不会比该 NoReorder incumbent 更差；不同策略经过多个事件后仍可能因历史路径依赖得到不同的整场结果。

额外时间一致性诊断使用 `configs/replanning_debug.yaml`，只运行 3 UAV、8+8 任务和 10 seeds 的 sanity check：

```bash
python scripts/run_batch.py --config configs/replanning_debug.yaml --seeds 40 41 42 43 44 45 46 47 48 49
```

该诊断不是正式论文实验。

## 实验前准备

`ExactEnumerator` 穷举任务排列及向有标签 UAV 路线的弱组合切分，允许空路线，并统一调用现有计划评估器。它硬性限制为不超过 8 个任务、3 架 UAV，只用于正确性与 optimality-gap 基准。

旅行模型配置明确分为：

```yaml
planning_travel_model: dubins   # 或 euclidean
execution_travel_model: dubins  # 固定翼物理执行默认始终为 dubins
objective_mode: weighted_delay  # 或 total_travel_time
```

因此 E3 的 `euclidean_planned` 含义是“用欧氏旅行时间规划，但按 Dubins 物理时间执行”，不是让固定翼按欧氏直线执行。无论采用哪种规划目标，输出指标都会统一报告真实执行历史上的 `weighted_delay`。

正式实验模板位于 `configs/experiments/`：E1 精确基准、E2 时效目标、E3 旅行模型、E4 动态策略和 E5 规模模板。E5 v2 已按冻结的 20/50/100 任务规模、seeds 2000–2009 和 NoReorder/Full/Local 三策略完成 90 组正式运行；配置中的历史 `frozen_template_do_not_run` 标签仅记录执行前冻结状态，正式授权来自 E5 v2 执行提示词。

E5 支持逐条检查点与断点续跑，并由独立脚本完成完整性检查、配对统计、绘图和报告：

```bash
python scripts/run_formal_experiments.py --experiment E5 --resume
python scripts/analyze_e5_formal.py --analyze
```

正式输出位于 `results/formal/E5/`，其中 `E5_FORMAL_REPORT.md` 给出十节验收结论，`integrity_manifest.json` 保存 E1–E4、Canonical 和冻结核心文件的前后哈希。

Local 参数 `h` 的轻量敏感性实验已在 E5 Medium 的相同 10 个 paired seeds 上完成。`h=1/2/3` 共 30 条记录，其中 `h=2` 的 10 条记录直接复用 E5 Medium，未重跑 E5；NoReorder 与 Full 仅作为冻结参考线。运行、续跑和分析使用：

```bash
python scripts/run_h_sensitivity.py --run --resume
python scripts/run_h_sensitivity.py --analyze
```

输出位于 `results/formal/H_SENSITIVITY/`。本轮数据支持将 `h=2` 保留为质量、计算时间和计划扰动之间的代表性折中，但不解释为全局最优或所有指标最优。

Dubins 公式采用标准六路径族解析构造。实现依据为 A. M. Shkel and V. Lumelsky, “Classification of the Dubins set,” *Robotics and Autonomous Systems*, 2001；代码为本项目独立的长度计算实现，未引入第三方 Dubins C 扩展。

## 结果解释边界

Debug 图现使用与长度计算相同最短路径族的 Dubins 采样曲线。采样只服务于绘图，不参与优化。当前阶段不包含图像识别、SAR 信号处理、真实飞控、障碍/天气/故障、通信模型、DRL、GA、ACO、Web 服务或数据库。

