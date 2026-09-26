# Local Parameter h Sensitivity Report

## 1. 实验目的

本实验只用于检查 Local 中受影响 UAV 数量参数 h 的敏感性，并解释默认 h=2 的经验性定位；它不承担新的创新点证明，也不重新设计或调优算法。

## 2. 实验配置

实验严格复用 E5 Medium：25 个初始任务、25 个动态任务、5 架 UAV、seeds 2000–2009，规划与执行均为 Dubins，主目标为 weighted_delay，commitment_horizon=0，搜索迭代与时间预算不变。唯一变量为 h=1/2/3。h=2 的 10 条记录直接复用冻结的 E5 Medium Local 结果；NoReorder 与 Full 也只读取 E5 Medium 作为参考，未重跑。Full 是全自由后缀启发式重规划，不是 Exact optimum。

## 3. 可行性检查

- 30/30 条 seed+h 记录完整且唯一，其中新运行 20 条、复用 E5 h=2 记录 10 条。
- feasible、time-consistent、完成全部任务、全部 UAV 返航分别为 30/30、30/30、30/30、30/30。
- 同一 seed 的 h=1/2/3 场景指纹一致；复用的 h=2 字段与 E5 Medium 原始记录逐字段一致。
- E1–E5 raw、Canonical 产物和冻结核心算法文件前后哈希一致。

## 4. 解质量变化

| h | Weighted mean delay mean |
|---:|---:|
| 1 | 20.9889 |
| 2 | 20.6883 |
| 3 | 20.5869 |

h=1 vs h=2 的 paired win/tie/loss（低延迟为胜）为 2/0/8；h=2 vs h=3 为 3/0/7。结果不预设单调性，所有反向 seed 均保留。

## 5. 计算时间变化

| h | Total replanning runtime | Mean replanning runtime | Max replanning runtime |
|---:|---:|---:|---:|
| 1 | 0.0858 s | 0.0034 s | 0.0124 s |
| 2 | 0.2230 s | 0.0089 s | 0.0452 s |
| 3 | 0.5808 s | 0.0232 s | 0.1043 s |

E5 Medium Full 的 total replanning runtime 参考均值为 1.7969 s；这里只把它作为全自由后缀启发式参考线。

## 6. 计划扰动变化

| h | Assignment changes | Successor-edge changes |
|---:|---:|---:|
| 1 | 0.00 | 1.80 |
| 2 | 16.40 | 23.90 |
| 3 | 26.50 | 39.20 |

参考线：NoReorder 为 0.00/0.00，Full 为 59.60/83.70（assignment/successor）。

## 7. 综合 trade-off

h=1 限制最强，倾向于更低的搜索开销和更稳定的计划；h=3 开放更多 UAV 后缀，倾向于提高搜索自由度，同时可能增加计算与扰动；h=2 位于两者之间。上述倾向必须结合本轮实际均值和 paired 结果理解，不能从机制直接推导为严格单调。
质量反向实例：h=1 优于 h=2 的 seeds 为 2005、2008；h=2 优于 h=3 的 seeds 为 2000、2002、2006。

## 8. h=2 是否合理

**A. h=2 是合理折中，建议保留。**

均值呈现清晰的质量—计算—稳定性折中：扩大 h 改善质量，同时增加计算时间和计划扰动；h=2 位于两个端点之间且未被支配。

## 9. 结论边界

本实验只有 E5 Medium 的 10 个 paired seeds，结论只说明冻结配置下的经验性 trade-off。不得把 h=2 表述为全局最优、所有指标最优或经过充分超参数优化；Wilcoxon p-value 仅作辅助，不作为更改 seed 或参数的依据。
