# 灾害应急异构固定翼无人机动态任务规划 MVP

本项目实现“EO 初筛产生动态重点复核任务，SAR 固定翼编队进行任务分配、排序和事件触发重规划”的第一阶段可复现实验闭环。EO 在 MVP 中被抽象为 synthetic task event generator；SAR 是唯一在线优化对象。

## 冻结的研究定义

- 主目标：`sum(priority_j * (completion_time_j - release_time_j))`。
- 所有已释放任务必须且只执行一次。
- 服务开始时刻不得早于任务释放时刻。
- 固定翼旅行时间由纯 Python Dubins 最短路径长度计算，支持 LSL、RSR、LSR、RSL、RLR、LRL。
- 动态策略为 `no_reorder`、`full` 和 `local`。
- Full 与 Local 共用同一个 `RouteEvaluator`、Regret-2 插入器和四个 VNS 邻域。
- 路线扰动只作为实验指标，不进入主目标。

## 工程结构

```text
configs/                 YAML 参数
src/uav_planning/        模型、几何、场景、路由、规划器、仿真、指标和绘图
scripts/run_single.py    seed=42 三策略验收入口
scripts/run_batch.py     小批量种子入口
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
```

输出包括：

- `results/raw/seed42_metrics.csv`
- `results/raw/seed42_<strategy>_events.csv`
- `results/summary/seed42_summary.csv`
- `results/figures/debug_seed42.png`

`small_debug.yaml` 固定为 2 架 SAR、3 个初始任务、2 个动态任务、50 x 50 地图和 seed 42。动态释放区间刻意设置在初始航线执行期间，以实际触发锁定和重规划。

## 算法说明

初始规划使用 Regret-2 insertion 构造可行解，再用 first-improvement VNS 改进。VNS 仅包含：

1. route 内 relocate；
2. route 间 relocate；
3. route 间 swap；
4. route 内 2-opt。

每个候选解都通过统一的 `RouteEvaluator` 检查 release time、服务时间、返航和最大任务时间。动态事件按 `TASK_RELEASE` 的时间顺序处理；最终执行日志包含 `TASK_START`、`TASK_COMPLETE` 和 `RETURN_DEPOT`。

Dubins 公式采用标准六路径族解析构造。实现依据为 A. M. Shkel and V. Lumelsky, “Classification of the Dubins set,” *Robotics and Autonomous Systems*, 2001；代码为本项目独立的长度计算实现，未引入第三方 Dubins C 扩展。

## 结果解释边界

Debug 图中的折线用于展示任务访问顺序；算法内部旅行时间仍由 Dubins 代价计算。当前阶段不包含图像识别、SAR 信号处理、真实飞控、障碍/天气/故障、通信模型、DRL、GA、ACO、Web 服务或数据库。

