# LeSTA–PUTN 风险融合：方法与代码修订

核对日期：2026-10-24。本文依据主工作区当前源码与配置编写，范围包括 `HEAD dca814d` 及尚未提交的融合策略修订。对比起点为融合接入前的 LeSTA 快照 `dc1a464`。本文只记录风险融合方法、代码接口、参数和必要的运行适配，不纳入评估流程、实验结果或实验数据处理脚本。

## 一、方法修订

### 1. 将 LeSTA 可通行概率接入 PUTN 几何风险

PUTN 保留局部平面拟合及三个几何指标：平整度 `flatness`、坡度 `slope`、稀疏度 `sparsity`。几何风险仍按原结构计算：

```text
g_raw = w_total × (w_flatness × flatness + w_slope × slope + w_sparsity × sparsity)
g     = min(1, g_raw)
```

LeSTA 地图给出可通行概率 `p`。接入时将它转为同向的学习风险 `l = 1 - p`，再计算最终风险 `r`。`Plane::traversability` 保存 `r`，原有 PUTN 路径代价函数因此能够直接使用融合风险。几何风险本身保存在单独字段中，便于区分两种来源。[实现](putn/src/putn/putn_planning/src/PUTN_classes.cpp)

几何风险的源码只做上限截断；在当前非负权重配置下它为非负值。成功读取 LeSTA 数据后的融合结果再限制到 `[0, 1]`。权重由配置给定，代码不自动归一化。

### 2. 连续风险融合策略

记 `w_p` 为 PUTN 权重、`w_l` 为 LeSTA 权重。主工作区保留三种可选策略：

| 策略名 | 最终风险 `r` | 含义 |
| --- | --- | --- |
| `legacy_weighted_sum` | `w_p × g + w_l × l` | 初始加权融合，保留原行为 |
| `risk_floor_weighted` | `max(g, w_p × g + w_l × l)` | 以 PUTN 几何风险为下限，LeSTA 不能降低已识别的几何危险 |
| `conservative_max` | `max(g, l)` | 直接取两种风险中较高者 |

风险下限策略针对旧加权可能冲淡几何风险的问题。例如 `g=0.8648`、`p=0.95`、权重 `0.75/0.25` 时，旧加权结果为 `0.6611`，低于 `0.70` 的节点门限；风险下限策略得到 `0.8648`。[策略实现](putn/src/putn/putn_planning/src/PUTN_classes.cpp) · [参数校验](putn/src/putn/putn_planning/src/global_planning_node.cpp)

查询位置超出 LeSTA 地图、格子无有效数据或地图尚不可用时，启用 LeSTA 的配置会使该节点回退到 `min(1, g + unknown_penalty)`；关闭 LeSTA 时则使用 `g`。当前专用融合入口把 `unknown_penalty` 设为 `0`。

### 3. 二值否决与风险门限共同决定新节点准入

对拟加入 PF-RRT* 搜索树的新采样节点，规划器依次检查：

1. 节点和平面有效，且处于 PUTN 地图边界内；
2. LeSTA 地图可用、该位置已观测且二值层判为不可通行时，直接拒绝；
3. 其余节点要求最终风险 `r ≤ lesta_risk_threshold`。

二值否决是**连续风险计算之后的硬准入规则**，不属于上述三种公式。即使加权风险低于门限，二值层判为不可通行的节点仍会被拒绝。主工作区当前没有单独的二值否决启停参数；独立实验副本中的消融开关不计入本文的主工作区实现。完全没有有效 LeSTA 地图层时，`useLeSTATraversability()` 为假，新增风险门限也不执行。[准入实现](putn/src/putn/putn_planning/src/PUTN_planner.cpp)

### 4. 融合风险进入 PF-RRT* 路径代价

节点通过准入后，规划器仍按原流程检查邻接边、选择父节点、重连树和选择目标路径。边代价公式未改写，只是其中两端节点的 `traversability` 现在可以是融合风险：

```text
C(i,j) = distance(i,j) × [1 + 0.1 × (1/(1.0001-r_i) + 1/(1.0001-r_j))]
```

因此，超过门限的候选节点被排除；门限以下的风险仍会提高累计路径代价，使选父、重连和最终路径选择倾向于较低风险的连接。[边代价](putn/src/putn/putn_planning/include/PUTN_classes.h) · [树扩展与重连](putn/src/putn/putn_planning/src/PUTN_planner.cpp)

### 5. 几何风险计算修正

局部平面 SVD 法向量的正负号可能翻转。坡度计算现使用 `acos(min(1, abs(z_axis · normal)))`，使同一平面的法向量取反不改变坡度，并避免浮点误差令 `acos` 输入略大于 1。几何原始风险、上限截断风险和四个几何诊断量分别保存；map1/map2 当前把 `w_slope` 调为 `0.02`、`w_sparsity` 调为 `0.2`，空缺率区间为 `0.02–0.05`，拟合半径分别为 `0.5 m` 和 `1.0 m`。这些改变同时影响关闭和开启 LeSTA 时的 PUTN 几何风险。[平面计算](putn/src/putn/putn_planning/src/PUTN_classes.cpp) · [map1 配置](putn/src/putn/putn_planning/config/for_simulation/map1.yaml) · [map2 配置](putn/src/putn/putn_planning/config/for_simulation/map2.yaml)

## 二、代码与接口修订

| 模块 | 具体改动 | 作用 |
| --- | --- | --- |
| [LeSTA mapping 节点](LeSTA/lesta_ros/src/ros/trav_mapping_node.cpp)及[头文件](LeSTA/lesta_ros/include/lesta/ros/trav_mapping_node.h) | 除原 `PointCloud2` 外，新增 `/lesta/mapping/traversability_grid_map`；包含 `elevation`、当前推理 `traversability/probability` 与 `traversability/binary`、累积 `mapping/probability` 与 `mapping/binary` | 为 PUTN 提供可按二维位置查询的 GridMap |
| [PUTN 数据结构](putn/src/putn/putn_planning/include/PUTN_classes.h) | 新增 `LeSTATraversabilityArg`、`World` 地图缓存与查询接口；`Plane` 增加几何风险、分项指标、LeSTA 概率、观测状态和二值判定字段 | 保存融合参数并区分风险来源 |
| [PUTN 地图查询与融合](putn/src/putn/putn_planning/src/PUTN_classes.cpp) | `updateLeSTATraversabilityMap()` 缓存 GridMap；`queryLeSTATraversability()` 按平面 XY 查格；`Plane` 构造时计算最终风险 | 将 LeSTA 输出转成规划节点风险 |
| [全局规划 ROS 节点](putn/src/putn/putn_planning/src/global_planning_node.cpp) | 订阅 `lesta_traversability`、转换 GridMap、读取策略/权重/图层/门限参数，校验策略名 | 把 LeSTA 地图接入规划器 |
| [PF-RRT* 规划器](putn/src/putn/putn_planning/src/PUTN_planner.cpp) | 增加新节点的二值否决与风险门限；沿用原风险边代价进行选父、重连和路径选择 | 使风险同时参与节点筛选与路径优化 |
| [融合诊断点云](putn/src/putn/putn_planning/src/PUTN_planner.cpp) | 发布 `/global_planning_node/fused_traversability_cloud`，含 XYZ 及 `fused_risk`、几何风险/分项、LeSTA 概率/风险/观测/二值字段 | 显示和区分当前规划树节点上的风险来源 |
| [构建依赖](putn/src/putn/putn_planning/CMakeLists.txt)及[包声明](putn/src/putn/putn_planning/package.xml) | 增加 `grid_map_core`、`grid_map_msgs`、`grid_map_ros` | 支持 GridMap 消息转换、缓存和查询 |

地图查询对有效概率值限制在 `[0,1]`；缺少二值层时以 `p > 0.5` 推导二值，只有二值层时把其结果补成 0/1 概率。地图越界或两个图层均无有效值时查询失败。[查询代码](putn/src/putn/putn_planning/src/PUTN_classes.cpp)

诊断点云来自**当前规划树中的有效节点**，不代表全地图风险分布。未观测节点的默认概率为 `0.5`，须结合 `lesta_observed` 字段解释。[发布代码](putn/src/putn/putn_planning/src/PUTN_planner.cpp)

### 启动配置与 LeSTA 侧适配

| 入口 | 默认策略与权重 | 风险门限 | 地图来源和读取层 |
| --- | --- | --- | --- |
| [通用仿真入口](putn/src/putn/putn_launch/launch/simulation.launch) | 默认关闭 LeSTA；启用时默认 `legacy_weighted_sum`、`0.3/0.7` | `0.75` | 默认 prediction；切换为 mapping 时默认读取累积 `mapping/*` 层 |
| [专用融合入口](putn/src/putn/putn_launch/launch/fusion_simulation.launch) | 默认启用 LeSTA、`risk_floor_weighted`、`0.75/0.25` | `0.70` | 订阅 mapping 消息，读取其中**当前推理** `traversability/probability` 与 `traversability/binary` |

“使用 mapping 节点”与“使用累积 mapping 图层”是两件事。专用融合入口的默认融合输入为当前推理层；累积层仍在 GridMap 中发布，可供另行配置使用。[mapping 图层发布](LeSTA/lesta_ros/src/ros/trav_mapping_node.cpp)

LeSTA 侧还修复了特征配置的 `grid_res: 0.1.` 数值写法；mapping 配置把 `remove_backpoints` 改为 `false`、二值阈值改为 `0.6`；模型加载增加文件可读性检查和异常日志。[特征配置](LeSTA/lesta_ros/config/include/feature_extraction_params.yaml) · [mapping 配置](LeSTA/lesta_ros/config/trav_mapping_node.yaml) · [模型加载](LeSTA/lesta_ros/src/core/TraversabilityNetwork.cpp)

运行配套方面，[local_planner.py](putn/src/putn/putn_mpc/scripts/local_planner.py) 将原固定 300 点的路径缓冲改为按输入长度分配，并处理空路径和可选长度上限；仿真启动、实机启动及 RViz 配置补充 LeSTA 话题和风险显示。这些是接口与路径执行适配，不改变 LeSTA 网络结构、训练损失或 PF-RRT* 的采样、连边碰撞算法。[实机入口](putn/src/putn/putn_launch/launch/bringup.launch) · [RViz 配置](putn/src/putn/putn_launch/rviz_config/simulation.rviz)

## 三、当前实现边界

- PUTN 按节点所在二维坐标查询 LeSTA 风险；原连边碰撞检测没有沿整条边逐格查询 LeSTA。
- GridMap 回调直接缓存收到的地图；规划器自身没有增加图层坐标变换或地图时效校验，启动配置负责使用一致的 `world` 坐标系。
- 新增风险准入检查发生在**新采样节点**进入树前；起终点创建和继承路径没有在同一位置执行该门限检查。
- 本次融合没有重新设计 LeSTA 的特征、网络、损失或 log-odds 时间累积算法。已有 LeSTA 特征与训练改进不应重复列为融合新增。
