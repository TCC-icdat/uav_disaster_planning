# Canonical Scenario v1.1 Finalization Report

## 1. 本轮结论

Canonical Scenario 已从 v1.0 碰撞诊断案例升级为 v1.1 静态 NFZ 安全案例。最终结果满足：

- 12/12 AOI 完成；
- 9/9 隐藏 AOI 由安全 EO 路径实际完整覆盖后释放；
- EO inflated-NFZ intersection：0；
- SAR scan、transfer、return inflated-NFZ intersection：0；
- `feasible = true`；
- `time_consistency = true`；
- mission makespan：2526.929 s，小于 3600 s；
- E5 未运行，E1–E4 正式结果未修改。

本轮把 A01、A05、A12 的 `t=0` 来源统一解释为卫星遥感、历史地理信息和地面应急报告形成的多源先验。卫星仅提供宏观态势与初始重点区域输入，不进入本文优化模型；项目没有新增卫星轨道、成像或任务调度仿真代码。

## 2. v1.0 保持不变的内容

以下内容保持冻结：6000 m × 6000 m 场景、Depot、1 EO + 3 SAR、12 个 AOI 的位置/尺寸/priority、3 个 initial AOI、EO footprint 和航带布局、SAR 传感器参数、单航段服务语义、`weighted_delay`、Regret-2、Multi-Neighborhood Local Search、Local `h=2`、Full/NoReorder/Local 定义、Simulator 的返回 Depot 语义以及全部 E1–E4 benchmark 配置与 raw CSV。

原来的四张 v1.0 diagnostic 图仍保留在 `results/canonical/figures/`，没有被终版图覆盖。物理参数仍只定位为 representative simulation parameters，不对应特定商用传感器型号。

## 3. 固定左视 SAR 镜像服务模式

每个 AOI 在 canonical preprocessing 中生成两个物理等价候选：Mode A 沿主轴方向飞行，Mode B 在目标另一侧反向飞行；两者均保持 AOI 位于固定 left-looking SAR 的左侧。选择规则严格为 A 安全则选 A，否则检查 B；两者都碰撞则报错。

发生 Mode A→B 切换的 4 个 AOI 为：

| AOI | Mode A | Mode B | 最终选择 |
|---|---|---|---|
| A07 | 与 inflated NFZ 相交 | 安全且完整覆盖 | B |
| A09 | 与 inflated NFZ 相交 | 安全且完整覆盖 | B |
| A10 | 与 inflated NFZ 相交 | 安全且完整覆盖 | B |
| A12 | 与 inflated NFZ 相交 | 安全且完整覆盖 | B |

其余 A01、A02、A03、A04、A05、A06、A08、A11 保持 Mode A。12/12 个最终 scan segment 均与 inflated NFZ 无交叉，且 AOI polygon 仍完整落入有效 SAR swath。选择记录位于 `results/canonical/final/sar_service_table.csv`。

## 4. EO 安全 sweep 与释放时刻重算

原 11 条 boustrophedon lane 的布局没有改变；只对原来相交的 5 条航带进行局部绕行：

- `EO_LANE_02`
- `EO_LANE_03`
- `EO_LANE_04`
- `EO_LANE_07`
- `EO_LANE_08`

非冲突 sweep leg 继续使用 v1.0 原始采样轨迹。局部绕行使用距 inflated NFZ 250 m 的确定性接入点与同一障碍感知 Dubins 几何层，之后重新累计路径距离并从新轨迹寻找 footprint 首次完整覆盖时刻。

EO 路径从 74921.929 m 增至 86907.009 m，增加 11985.080 m，即 15.997%。安全轨迹与 inflated NFZ 的交叉数由 5 降至 0。

| AOI | v1.0 release (s) | v1.1 safe release (s) | 变化 (s) |
|---|---:|---:|---:|
| A11 | 257.015 | 257.015 | 0.000 |
| A07 | 746.135 | 389.343 | -356.792 |
| A10 | 454.432 | 579.618 | +125.186 |
| A09 | 497.860 | 620.266 | +122.406 |
| A08 | 624.420 | 746.827 | +122.406 |
| A06 | 874.123 | 1061.715 | +187.592 |
| A04 | 1118.112 | 1305.704 | +187.592 |
| A02 | 1708.078 | 2050.509 | +342.431 |
| A03 | 1858.066 | 2200.497 | +342.431 |

A07 提前释放不是沿用旧时间或人工修改，而是安全绕行轨迹在更早位置首次完整覆盖其 polygon 的直接结果。逐采样位姿、前后时间和差值见 `results/canonical/final/aoi_release_table.csv`。

## 5. SAR obstacle-aware edge 的实现

新增实现仅位于 canonical 模块，普通 E1–E4 `DubinsTravelTimeProvider` 未修改。

`ObstacleAwareDubinsPlanner` 对每条有向 start pose→goal pose 先采样最短 Dubins path；若安全则直接使用。若相交，则以 inflated NFZ 外的确定性 clearance corner poses 建立小型有向候选图，候选姿态采用 4 个离散航向，用 Dijkstra 搜索最短安全组合。图中的每条边均为满足 250 m 最小转弯半径的真实 Dubins path，而非欧氏直线替代；最终拼接轨迹再以 10 m 采样和线段—polygon 相交检查复核。

配置参数集中写在 `configs/canonical/post_earthquake_v1.yaml`：

- `sample_step_m: 10`
- `waypoint_clearance_m: 250`
- `waypoint_heading_count: 4`
- `eo_detour_approach_m: 250`

`ObstacleAwareDubinsTravelTimeProvider` 把安全路径长度除以 UAV 速度作为 canonical route edge cost，既用于规划也用于执行评估。它没有改动 Regret-2、local search 邻域或 Local replanning 规则。

## 6. 最终安全性与任务可行性

| 验收项 | v1.0 diagnostic | v1.1 final |
|---|---:|---:|
| EO 相交边数 | 5 | 0 |
| SAR 相交边数 | 19 | 0 |
| 完成 AOI | 12/12 | 12/12 |
| time consistency | 通过 | 通过 |
| makespan (s) | 2163.976 | 2526.929 |
| 3600 s endurance | 满足 | 满足 |

最终 `results/canonical/final/nfz_intersections.csv` 只有表头，表示 EO 与 SAR 的逐边复核均没有碰撞记录。

## 7. 安全路径的代价

- EO path length：增加 11985.080 m（15.997%）。
- SAR total travel time：2799.027 s → 2848.068 s，增加 49.041 s（1.752%）。
- weighted delay：10788.201 → 12452.279，增加 1664.078（15.425%）。
- makespan：2163.976 s → 2526.929 s，增加 362.953 s（16.772%）。

这些 before/after 数值只用于 canonical 工程案例说明，不作为统计显著性结论。延迟和 makespan 的变化同时受到 EO 安全绕行后 release-time 改变、SAR 镜像服务姿态以及安全 transfer/return edge 的影响。

## 8. Local replanning 快照

自动选择的代表事件为 `t=2200.497 s`：安全 EO sweep 释放 A03 时，SAR1 已在执行 A02。Before panel 中 A03 尚未加入；After panel 中 A03 被插入 SAR1 的未来后缀第 1 位，即在已锁定的 A02 执行完成后继续服务 A03。旧任务 assignment/order 没有变化，因此 `assignment_changes = 0`、`successor_edge_changes = 0` 与 v1.1 的解释口径一致，并非异常。

## 9. 五张终版图

1. `results/canonical/final/canonical_scene_final.png`
2. `results/canonical/final/eo_safe_sweep_and_detection.png`
3. `results/canonical/final/sar_service_modes_example.png`
4. `results/canonical/final/canonical_safe_routes.png`
5. `results/canonical/final/canonical_replanning_snapshot.png`

其中 `canonical_safe_routes.png` 只绘制通过 NFZ 验证的轨迹，不再使用 v1.0 diagnostic 图中的红色碰撞边。

## 10. 可支持的论文叙事

当前案例可支持如下窄口径叙事：卫星遥感、历史地理信息与地面应急报告先形成宏观态势和 3 个初始重点区域；在已圈定的局部灾区内，EO 固定翼 UAV 持续执行受静态禁飞空域约束的初筛，footprint 首次完整覆盖产生动态 SAR 精查任务；Local replanning 在冻结执行状态的前提下把新任务插入未来后缀；固定左视 SAR 通过确定性镜像服务预处理和障碍感知 Dubins 边安全完成全部任务。

该案例是问题特定建模与工程验证，不宣称首次提出 EO/SAR 协同或通用局部重规划算法，也不把 UAV 描述为卫星的替代者。

## 11. 明确建模假设与限制

- NFZ 为静态、已知、二维 restricted airspace；不含动态障碍或在线地图更新。
- 100 m buffer、250 m clearance 和传感器参数均为代表性仿真设置，不是法规或特定产品指标。
- 障碍图使用有限候选姿态和采样碰撞复核，保证本案例轨迹安全，但不声称全局连续空间最优或完备。
- SAR service mode 只在 preprocessing 中按 A 优先规则确定，不与任务分配/排序联合优化。
- 不研究 UAV-UAV 实时避碰；假定空域管理、不同高度层或任务时序提供底层 deconfliction。
- EO 识别采用零延迟、无误检/漏检抽象；不包含 SAR 图像处理。
- 卫星只作为多源先验的一部分，不包含卫星仿真，也不假定卫星自动产生本文格式的任务与权重。
- 当没有待执行任务时，SAR UAV 按现有 Simulator 返回 depot/base state；后续任务允许再次从 Depot 分派。模型忽略真实落地、整备和再次起飞的额外 turnaround time，该行为不应包装成真实运营流程。
- 固定高度二维模型不优化爬升/下降，也不显式建模天气、通信、续航不确定性或起降条件。

## 12. 自动验收与冻结结果保护

新增 v1.1 自动测试覆盖镜像左视覆盖、选定 scan 安全性、安全 EO sweep、释放时刻重算、障碍感知 Dubins 路径、SAR 全路径零交叉、12/12 完成与正式结果哈希保护。最终 `pytest -q` 为 **61 passed**。

E1–E4 raw CSV SHA-256 仍为：

- E1: `90CE8F9469D0C47D1B536B54B169C9F9B98D20874B89891780748A950AA776E0`
- E2: `75CD5D083F763064BDBCF9AA59875CF22121DBCB8BE7DBA3AAFE11E15E0C8DA8`
- E3: `3FFDDB04E403558AB7131C000EFF6526EA033B9E88CFBE051B0CC440D7BBF59C`
- E4: `4140284713CBA758AB7003F95710F8DCF0A6F76A6410CE4B443E4D40E9ADC58D`

因此本轮未修改 E1–E4 正式结果或结论，也未运行 E5。
