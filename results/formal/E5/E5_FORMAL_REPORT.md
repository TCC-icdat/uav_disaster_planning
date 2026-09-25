# E5 Formal Scale-Expansion Experiment Report

## 1. 实验目的

本实验只检验冻结的 NoReorder、Full 与 Local 动态重规划策略在 20、50、100 个任务规模下的可行性、任务时效、在线计算时间和计划扰动趋势。它属于归一化 controlled benchmark，不含 Canonical Scenario 的 EO、SAR、NFZ 或卫星先验机制。

## 2. 实验配置

Small/Medium/Large 分别使用 10+10/25+25/50+50 个初始与动态任务以及 3/5/8 架 SAR UAV；每个规模使用 seeds 2000–2009，并在同一实例上配对运行 NoReorder、Full、Local，共 90 runs。地图 100×100，速度 10，最小转弯半径 10，续航上限 500，规划与执行均为 Dubins，目标为 weighted_delay，Local 使用 h=2、commitment_horizon=0 和冻结搜索预算。Full 是全自由后缀启发式重规划，不是 Exact optimum。

## 3. 完整性与可行性检查

- 唯一运行：90/90；配对实例：30/30。
- feasible：90/90；time-consistent：90/90。
- 完成全部任务：90/90；全部 UAV 返航：90/90。
- 相同 scale+seed 的三策略场景指纹一致；不存在重复、缺失或额外 run。
- E1–E4 raw SHA-256、Canonical 产物清单和冻结算法文件均通过前后比对。

## 4. 解质量随规模变化

Local 相对 Full 的 weighted mean delay 配对差距如下。正值表示 Local 延迟更高，负值表示 Local 更低；不预设合格百分比阈值。

| Scale | Mean quality gap | Mean runtime ratio | Assignment reduction | Successor reduction |
|---|---:|---:|---:|---:|
| Small | -0.05% | 0.580 | 27.96% | 35.42% |
| Medium | 1.90% | 0.126 | 71.80% | 70.80% |
| Large | 2.92% | 0.039 | 83.91% | 81.92% |

Local 相对 NoReorder 的时效改善如下，正值表示 Local 延迟更低。

| Scale | Weighted-mean-delay improvement | High-priority-delay improvement |
|---|---:|---:|
| Small | 0.97% | 1.91% |
| Medium | 1.72% | 0.94% |
| Large | 1.58% | -0.82% |

## 5. 计算时间随规模变化

- Small：Local/Full 重规划时间均值 0.0107/0.0224 s；分配扰动均值 2.80/4.00；后继扰动均值 2.40/3.30。
- Medium：Local/Full 重规划时间均值 0.2230/1.7969 s；分配扰动均值 16.40/59.60；后继扰动均值 23.90/83.70。
- Large：Local/Full 重规划时间均值 2.1017/55.1098 s；分配扰动均值 63.00/398.50；后继扰动均值 127.60/707.50。

重规划时间图使用对数纵轴坐标；图中误差线为均值的 95% CI。

## 6. 计划扰动随规模变化

NoReorder 按定义不改变旧任务归属和相对顺序；Full 与 Local 的 assignment_changes 和 successor_edge_changes 均保留原始计数。上表给出的 Local 相对 Full 降低比例在 Full 为 0 时记为 NA，不进行除零。各规模完整均值、标准差、中位数、极值和 95% CI 见 `summary_by_scale_strategy.csv`。

## 7. Trade-off 分析

NoReorder 提供最强计划稳定性和最低重规划开销，但其调整自由度最小；Full 开放所有 UAV 的自由未来后缀，搜索自由度和计划扰动通常更高；Local 仅开放 h=2 架受影响 UAV，并仅接受严格改善。是否取得接近 Full 的时效质量以及是否降低计算和扰动，均应按上表逐规模解读，不能表述为 Local 在所有指标上绝对最优。

## 8. 异常实例

- Local weighted mean delay 高于 Full 的配对：small/2002、small/2008、small/2009、medium/2001、medium/2002、medium/2003、medium/2004、medium/2006、medium/2007、medium/2008、medium/2009、large/2000、large/2001、large/2002、large/2003、large/2004、large/2005、large/2006、large/2007、large/2008、large/2009。
- Full 重规划时间低于 Local 的配对：无。
- NoReorder weighted mean delay 低于 Local 的配对：small/2007、medium/2005、medium/2008。

以上反向实例全部保留，没有删 seed、重生成实例或据此调参。

## 9. 与 E4 的一致性

E4 中 NoReorder/Full/Local 的 weighted mean delay 均值分别为 19.5656/19.1008/19.2171，重规划时间均值分别为 0.0066/0.1198/0.0418 s。
E5 总体支持 E4 的质量—计算—稳定性 trade-off：三个规模下 Local 的平均重规划时间和两类计划扰动均低于 Full，并且相对 NoReorder 的 weighted mean delay 平均改善均为正。与此同时，E5 对 E4 的质量结论作出明确限定：Local 相对 Full 的平均质量差距由 Small 的 -0.05% 增至 Medium 的 1.90% 和 Large 的 2.92%，说明受限局部搜索的质量代价随规模扩大而增加，不能声称 Local 始终与 Full 等价或更优。
本比较始终位于同一 controlled benchmark 框架，不引入新的物理场景因素。

## 10. 最终判断

**E5通过，支持规模扩展结论。**

该判断只覆盖冻结配置、三个正式规模与 10 个 paired seeds；不外推为 Full 的全局最优性，也不构成 h 参数敏感性结论。

## 正式输出

- `raw.csv`
- `summary_by_scale_strategy.csv`
- `paired_local_vs_full.csv`
- `paired_local_vs_noreorder.csv`
- `statistics.md`
- `e5_quality_vs_scale.png`
- `e5_replanning_runtime_vs_scale.png`
- `e5_plan_disruption_vs_scale.png`
- `e5_quality_runtime_tradeoff.png`
- `integrity_manifest.json`
