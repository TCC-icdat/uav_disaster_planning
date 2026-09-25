# Canonical Scenario Prototype Report

## 1. 本轮范围与结论

本轮在现有 `uav_disaster_planning` 工程中新增 canonical 物理场景层，并复用已冻结的 Regret-2、Multi-Neighborhood Local Search、`weighted_delay` 主目标和 Local `h=2`。未运行 E5，未实现 NFZ 绕障，也未改变 E1–E4 的配置、原始结果或结论。

端到端 EO→SAR 原型运行成功：12/12 个 AOI 完成，9 个隐藏 AOI 均由 EO footprint 首次完整覆盖产生释放事件，9 次释放均触发 Local replanning；任务级仿真可行，3 架 SAR 均在 3600 s 航时约束内返回，makespan 为 2163.976 s。

## 2. 场景是什么样

- 场景：震后 6000 m × 6000 m 乡镇/城区快速评估。
- Depot：`(-500, 500, 0°)`，位于主地图西侧。
- 侦察链：1 架 EO 固定翼执行固定 boustrophedon 广域扫描；3 架 SAR 固定翼执行语义 AOI 精查。
- AOI：12 个固定矩形语义区域，其中 A01、A05、A12 在 `t=0` 已知，其余 9 个由 EO 观测动态释放。
- 障碍对象：3 个静态 NFZ，诊断使用 100 m 安全膨胀边界。
- 多机冲突：本研究不处理 UAV-UAV 实时避碰，假定由空域管理、不同高度层或任务时序完成底层 deconfliction。

场景配置位于 `configs/canonical/post_earthquake_v1.yaml`，总览图位于 `results/canonical/figures/canonical_scene_map.png`。

## 3. EO footprint 与 sweep

EO 参数为高度 600 m、HFOV 60°、VFOV 45°、速度 35 m/s、最小转弯半径 250 m。按冻结公式计算：

- cross-track footprint：692.820 m；
- along-track footprint：497.056 m；
- 20% 重叠后的 lane spacing：554.256 m；
- sweep lanes：11 条；
- 含 Depot 转场和 Dubins 航带连接的 EO 路径长度：74921.929 m；
- EO sweep 总时间：2140.627 s。

AOI 释放时刻直接由采样 EO 轨迹的累计飞行距离除以 35 m/s 得到，未使用随机 release time。

## 4. AOI 与释放时刻

| AOI | 语义 | Priority | 初始已知 | Release time (s) |
|---|---|---:|:---:|---:|
| A01 | Hospital | 10 | 是 | 0.000 |
| A02 | School | 8 | 否 | 1708.078 |
| A03 | Bridge | 8 | 否 | 1858.066 |
| A04 | Residential damage | 7 | 否 | 1118.112 |
| A05 | Main-road junction | 7 | 是 | 0.000 |
| A06 | Apartment block | 6 | 否 | 874.123 |
| A07 | Emergency shelter | 9 | 否 | 746.135 |
| A08 | Industrial block | 6 | 否 | 624.420 |
| A09 | Bridge | 8 | 否 | 497.860 |
| A10 | Residential damage | 5 | 否 | 454.432 |
| A11 | School | 8 | 否 | 257.015 |
| A12 | Clinic | 9 | 是 | 0.000 |

完整检测位姿与采样索引见 `results/canonical/aoi_release_table.csv`；按时间排序的事件见 `results/canonical/CANONICAL_TIMELINE.md`。

## 5. SAR 服务几何

SAR 参数为高度 800 m、入射角 35°、波束宽度 20°、速度 40 m/s、最小转弯半径 250 m。冻结公式得到：

- effective ground swath：426.954 m；
- nominal stand-off：560.166 m；
- 12 个 AOI 的 `max(长边 + 300 m, 600 m)` 均为 600 m；
- 每个 AOI 的 scan service time：15.000 s。

所有 AOI polygon 均完整落入各自的单条 SAR swath，因此 12/12 个 AOI 都可由一个扫描航段完成。规划任务位姿是 scan entry，任务完成位姿是 scan exit；此语义只通过 canonical evaluator/adapter 启用，普通 E1–E4 点任务仍保持原行为。完整几何见 `results/canonical/sar_service_table.csv`。

## 6. Local 端到端结果

本轮只运行一次 canonical Local strategy：

- objective mode：`weighted_delay`；
- Local affected UAV count：`h=2`；
- dynamic replanning count：9；
- weighted delay：10788.201；
- mean delay：118.244 s；
- makespan：2163.976 s；
- time consistency：通过；
- mission feasibility：通过；
- NFZ-aware rerouting：关闭。

运行指标见 `results/canonical/canonical_metrics.json`。这些数值用于 canonical 工程案例展示，不替代 E1–E4 controlled benchmark。

## 7. NFZ 诊断

诊断对象为带 100 m 安全膨胀的 NFZ polygon；统计单位同时给出“唯一边”和“边×NFZ 记录”，因为同一条路径边可能穿过多个 NFZ。

| Vehicle | NFZ1 | NFZ2 | NFZ3 | 合计 |
|---|---:|---:|---:|---:|
| EO intersection rows | 2 | 1 | 2 | 5 |
| SAR intersection rows | 6 | 3 | 12 | 21 |

- EO sweep 穿越 NFZ：是，共 5 条不同 EO 边。
- 当前无障碍 SAR route 穿越 NFZ：是，共 19 条不同 SAR 边，对应 21 条边×NFZ 记录。
- SAR 相交记录包括 transfer、return，也包括 A12、A10、A09、A07 的 scan service segment。

逐边的 UAV、起终节点、边类型、NFZ 和首个交点见 `results/canonical/nfz_intersections.csv`。当前路线未因诊断而改变。

## 8. 四张验收图

1. `results/canonical/figures/canonical_scene_map.png`
2. `results/canonical/figures/eo_sweep_and_detection.png`
3. `results/canonical/figures/sar_service_geometry.png`
4. `results/canonical/figures/canonical_route_diagnostic.png`

第 4 张图是诊断图，不是最终论文安全轨迹图；红色路径边及叉号表示与膨胀 NFZ 相交的位置。

## 9. 是否值得实现 obstacle-aware edge cost

值得。EO 有 5 条边、SAR 有 19 条不同边发生相交，说明障碍感知不是只影响极少数展示细节，而会实质改变 canonical 路径几何和代价。

下一阶段的最小建议是：先冻结 100 m 膨胀规则；对 transfer/return 边使用 NFZ 顶点可见图或少量确定性绕行 waypoint 生成候选折线路径，再用可行 Dubins 子段连接并将总飞行时间作为 edge cost。由于 4 条固定 scan service segment 本身也相交，仅增加 transfer edge cost 不足以得到全程安全路线；还需为这些 AOI 提供确定性的另一侧 side-looking 扫描候选，并剔除相交候选。该建议本轮未实现。

## 10. 自动验收与历史实验保护

新增自动检查覆盖：EO footprint 尺寸、9 个隐藏 AOI 全检出、release time 来源、SAR 单条带覆盖、服务段几何、配置确定性、canonical scan-exit 完成位姿，以及 E1–E4 raw CSV SHA-256 不变。

最终 `pytest -q` 结果为 **52 passed**。E1、E2、E3、E4 的 `raw.csv` 哈希分别仍为：

- E1: `90CE8F9469D0C47D1B536B54B169C9F9B98D20874B89891780748A950AA776E0`
- E2: `75CD5D083F763064BDBCF9AA59875CF22121DBCB8BE7DBA3AAFE11E15E0C8DA8`
- E3: `3FFDDB04E403558AB7131C000EFF6526EA033B9E88CFBE051B0CC440D7BBF59C`
- E4: `4140284713CBA758AB7003F95710F8DCF0A6F76A6410CE4B443E4D40E9ADC58D`

因此，本轮没有修改已经通过的 E1–E4 结论。
