# 正式实验阶段一报告（E1-E4）

> 后续验收说明：本报告中的原 E2 已降级为纯距离目标的补充/结构性对照。论文 E2 主比较已由 `results/formal/E2_v2/E2_V2_REPORT.md` 更新为 time-aware vs mission-completion。E1、E3、E4 保持不变。

## 1. 执行状态与边界

本轮以 `正式实验阶段执行提示词_v1.0.md` 和当前代码为准，只补充实验记录、正式批处理、统计与绘图，没有修改 Initial、Full、Local、NoReorder 的算法定义，也没有更换主目标或引入新算法。

- E1-E4 已正式运行并完成验收。
- E5 仅完成 dry-run；未创建或运行 E5 正式结果。
- 正式动态场景均保留全部 seed 级数据，没有筛选或删除 seed。
- `results/formal/ENVIRONMENT.md` 已记录 Python、OS、CPU、依赖版本、Git commit 和五份配置的 SHA-256。

## 2. 代码与数据管线验收

### 2.1 新增记录指标

Simulator 现统一输出：

- `weighted_mean_delay = weighted_delay / sum(priority)`；
- `initial_planning_runtime`；
- `total_algorithm_runtime = initial_planning_runtime + total_replanning_runtime`。

正式 runner 在仿真最外层输出 `runner_wall_runtime`。所有原有指标继续保留。

### 2.2 自动化与恢复

- 新增 `scripts/run_formal_experiments.py`，必须通过 `--experiment E1` 至 `--experiment E5` 单独选择实验，不会默认批量运行全部实验。
- `--dry-run` 输出 variants、seeds、run count 和输出路径。
- 每个 run 完成后立即追加并刷盘到 `raw.csv`。
- `--resume` 会按冻结主键跳过完整记录；E1-E4 完成后复测均显示 `pending=0`。
- 配对实验保存场景 SHA-256 指纹；E2、E3、E4 的所有配对组均只有一个场景指纹。

### 2.3 测试与 dry-run

- `pytest -q`：`38 passed`。
- E1 dry-run：30 runs（3 个任务规模 × 10 seeds）。
- E2 dry-run：60 runs（2 variants × 30 seeds）。
- E3 dry-run：180 runs（3 个 R × 2 variants × 30 seeds）。
- E4 dry-run：90 runs（3 strategies × 30 seeds）。
- E5 dry-run：90 runs（3 scales × 3 strategies × 10 seeds），未执行。

## 3. 实际运行完整性

| 实验 | 实际 runs | 预期 runs | 重复主键 | feasible | time consistent |
|---|---:|---:|---:|---:|---:|
| E1 | 30 | 30 | 0 | Exact/heuristic 均成功返回 | N/A（静态） |
| E2 | 60 | 60 | 0 | 60/60 | 60/60 |
| E3 | 180 | 180 | 0 | 180/180 | 180/180 |
| E4 | 90 | 90 | 0 | 90/90 | 90/90 |
| 合计 | 360 | 360 | 0 | 动态实验 330/330 | 动态实验 330/330 |

## 4. E1：Small-scale Exact Benchmark

| tasks | exact objective mean | heuristic objective mean | mean gap | median gap | max gap | exact runtime mean | heuristic runtime mean |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 5 | 366.4570 | 366.4862 | 0.0077% | 0.0000% | 0.0768% | 0.0435 s | 0.0053 s |
| 6 | 476.4455 | 483.3324 | 1.2653% | 0.0000% | 6.3013% | 0.3724 s | 0.0091 s |
| 7 | 599.5646 | 603.2222 | 0.7297% | 0.0000% | 3.4554% | 3.4385 s | 0.0152 s |

需保留说明的较大 gap seed：

- 6 tasks / seed 2：6.3013%；
- 6 tasks / seed 7：5.5253%；
- 7 tasks / seed 1：3.4554%。

这些记录均保留在 raw CSV 中。

## 5. E2：时效目标与飞行成本

| variant | weighted mean delay | high-priority mean delay | total travel time |
|---|---:|---:|---:|
| time_aware | 19.2171 | 16.2656 | 168.1541 |
| distance_oriented | 75.3379 | 75.1692 | 101.7957 |

配对描述统计：

- time-aware 的 weighted mean delay 相对差异均值为 -74.4113%（95% CI：-75.1595% 至 -73.6632%）；30/30 seeds 的该指标更低。
- time-aware 的 total travel time 相对差异均值为 +65.7231%（95% CI：+60.8928% 至 +70.5535%）。
- 未发现不可行、时间不一致或配对场景不一致的 seed。

以上仅为冻结实验下的质量/飞行成本权衡统计，不在本报告中扩展为论文结论。

## 6. E3：Dubins 转弯半径敏感性

Dubins-aware 相对 Euclidean-planned 的 weighted mean delay 改善率定义为：

`(metric_euclidean - metric_dubins) / metric_euclidean * 100%`。

| R | relative improvement mean | 95% CI | win / tie / loss |
|---:|---:|---:|---:|
| 5 | 0.9785% | -0.3738% 至 2.3307% | 18 / 1 / 11 |
| 10 | 7.2200% | 4.5113% 至 9.9286% | 25 / 0 / 5 |
| 15 | 17.2767% | 15.2053% 至 19.3481% | 30 / 0 / 0 |

必须保留的反向 seed：

- R=5：1000、1003、1004、1006、1010、1011、1012、1018、1023、1028、1029；
- R=10：1003、1004、1013、1018、1029；
- R=15：无。

所有反向 seed 均保留；没有据此调参或删样本。

## 7. E4：动态重规划策略

| strategy | weighted mean delay | high-priority mean delay | replanning runtime | assignment changes | successor-edge changes |
|---|---:|---:|---:|---:|---:|
| NoReorder | 19.5656 | 16.4873 | 0.0066 s | 0.0000 | 0.0000 |
| Full | 19.1008 | 16.0360 | 0.1198 s | 12.8333 | 14.6000 |
| Local | 19.2171 | 16.2656 | 0.0418 s | 7.0667 | 8.4000 |

Local vs Full 配对派生量：

- `quality_gap_local_vs_full` 均值 0.6615%，95% CI 为 -0.1658% 至 1.4888%；Local / Full 的 win / tie / loss 为 9 / 4 / 17。
- `runtime_ratio` 均值 0.3799，95% CI 为 0.3467 至 0.4131。
- Local 比 Full 的 weighted delay 更高的 seeds：1000、1001、1003、1005、1007、1009、1012、1013、1015、1017、1018、1019、1024、1025、1026、1028、1029。

Full 在本报告中只作为“全自由后缀重新规划的启发式方法”，不解释为 Exact optimum 或理论上界；Local 优于 Full 的 seed 同样保留。

## 8. 输出清单

CSV：

- `results/formal/E1/raw.csv`
- `results/formal/E1/summary.csv`
- `results/formal/E2/raw.csv`
- `results/formal/E2/summary.csv`
- `results/formal/E2/paired.csv`
- `results/formal/E3/raw.csv`
- `results/formal/E3/summary.csv`
- `results/formal/E3/paired.csv`
- `results/formal/E4/raw.csv`
- `results/formal/E4/summary.csv`
- `results/formal/E4/paired.csv`

Figures：

- `results/formal/figures/exact_gap_vs_size.png`
- `results/formal/figures/time_objective_tradeoff.png`
- `results/formal/figures/dubins_radius_sensitivity.png`
- `results/formal/figures/replanning_quality_runtime.png`

Environment：

- `results/formal/ENVIRONMENT.md`

## 9. 阶段停止点

E1-E4 已完成并通过数据完整性检查。本轮在此停止；E5 未运行，等待下一轮验收后再开放。
