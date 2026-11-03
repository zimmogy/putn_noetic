# LeSTA–PUTN 融合算法及实验脚本改动梳理

核对日期：2026-09-23。范围：当前 HEAD `dca814d`，以及当前工作区尚未提交、尚未跟踪的相关源码、配置与脚本。本文按 Git 差异与当前实现整理，不把历史报告中的建议或实验草案当作已实现功能。本次只新增本文，没有修改算法代码。

## 回溯起点与计入规则

- PUTN 起点：`203f51a`（2026-04-24，Migrate PUTN to ROS Noetic and Python3）。Noetic/Python3 迁移已经完成，不再算融合阶段的新改进。
- 独立改进 LeSTA 起点：`dc1a464`（2026-06-18，Update LeSTA files）。它是 `1df0d26`（stage1:algorithm fusion with LeSTA）之前的共同快照，本文以 `git diff dc1a464` 为主要差异依据，并补查未跟踪文件。
- 八维特征、强度均值/方差、基于局部空缺分布的 sparsity、风险敏感损失、风险加权数据集、本体点云过滤等，在独立改进 LeSTA 中已经存在；不能再次归为融合新增。`pylesta` 中大量后续差异是作者/日期、注释、日志文字调整，不能按“文件修改数量”推算算法改进数量。
- 当前关闭 LeSTA 得到的是“当前改动版 PUTN 基线”，并不等于逐字恢复 Noetic 起始版本：坡度修正、几何参数、控制和底盘改动仍然存在。

主要阶段：`1df0d26` 至 `a23a0f1` 完成融合接入和显示；`6b2fa82`、`07a6435`、`3afedad` 修正/诊断几何风险并调参；`667388c` 至 `9801d8b` 增加相机和测试地图；`b373dfb`、`971b589`、`4e3ba32` 增加自动评测、预热和先到起点机制；`dca814d` 增加 mapping 接入及评测工具。风险下限策略、底盘修正、独立会话脚本和 map1_wide 等还包含工作区未提交改动，不能只查看 HEAD。

## 第一部分：融合层面对原代码的改动（包括底盘）

以下路径均相对本文件所在的 `src/`。PUTN 路径前缀为 `putn/src/putn/`，底盘前缀为 `putn/src/scout_simulator/`。

### 1. 新增 LeSTA 地图输入、缓存与查询

代码：

- [PUTN_classes.h](putn/src/putn/putn_planning/include/PUTN_classes.h)：新增 `LeSTATraversabilityArg`、`World` 的地图缓存和接口，扩展 `Plane`。
- [PUTN_classes.cpp](putn/src/putn/putn_planning/src/PUTN_classes.cpp)：新增 `updateLeSTATraversabilityMap()`、`queryLeSTATraversability()`。
- [global_planning_node.cpp](putn/src/putn/putn_planning/src/global_planning_node.cpp)：新增 GridMap 订阅、消息转换、参数读取和初始化。

原来 PUTN 只使用自己的点云地图和平面风险。现在在节点所在二维位置查询 LeSTA GridMap，读取配置指定的概率层和二值层。概率限制在 `[0,1]`；缺少二值层时按概率 `>0.5` 推导二值结果；只有二值层时补为 0/1 概率。地图越界或对应格无有效数据，返回查询失败。

`Plane` 新增几何原始风险、几何截断风险、flatness/slope/sparsity/vacancy_ratio、LeSTA 概率、是否观测和二值判定字段，区分不同风险来源。原 `traversability` 字段保留，改为承载最终融合风险，以接入既有规划代价。

实现边界：PUTN 回调直接保存 GridMap，查询时没有另做坐标变换或地图超时校验；仿真 launch 将 LeSTA 地图统一到 `world`。评测脚本的数据新鲜度检查不能视作规划器本身具有同样检查。

### 2. 修改风险计算，并保留三种融合策略

代码：[PUTN_classes.cpp](putn/src/putn/putn_planning/src/PUTN_classes.cpp) 的 `Plane` 构造函数；策略合法性检查位于 `global_planning_node.cpp`。

记 `g` 为 PUTN 几何风险，`p` 为 LeSTA 可通行概率，`l=1-p` 为学习风险。原始几何风险仍为：

```text
g_raw = w_total × (w_flatness × flatness + w_slope × slope + w_sparsity × sparsity)
g = min(1, g_raw)
```

注意这里源码对几何项是上限截断，并非通用的双侧 clamp。默认非负参数下风险非负。

| 增改项 | 当前计算/作用 |
| --- | --- |
| 初始加权融合 `legacy_weighted_sum` | `r = wp*g + wl*(1-p)`；原实现保留用于复现 |
| 新增 `risk_floor_weighted` | `r = max(g, wp*g + wl*(1-p))`；学习结果不能降低几何风险 |
| 新增 `conservative_max` | `r = max(g, 1-p)`；用于更保守的比较 |
| 未观测回退 | 启用 LeSTA 但查询失败时，使用 `g + unknown_penalty`，上限截到 1 |
| 关闭融合 | 使用几何风险 |

查询成功时的融合输出做 `[0,1]` 截断；权重没有自动归一化。未知策略名会警告并退回 `legacy_weighted_sum`。

新策略针对旧加权可能冲淡几何危险的问题。例如 `g=0.8648`、`p=0.95`、权重 0.75/0.25，旧结果为 0.6611，可能低于 0.70 门限；风险下限策略保留至少 0.8648。其变更位于当前未提交工作区。

### 3. 新增节点准入，复用原有路径代价

代码：[PUTN_planner.cpp](putn/src/putn/putn_planning/src/PUTN_planner.cpp) 的 `isTraversabilityAcceptable()` 和 `planner()`；声明位于 [PUTN_planner.h](putn/src/putn/putn_planning/include/PUTN_planner.h)。

新增采样节点进入树前的条件：

1. 节点/平面有效。
2. 当融合地图可用时，已观测且 LeSTA 二值判定不可通行的节点直接拒绝。
3. 其余节点要求最终风险不超过 `lesta_risk_threshold`。

这里二值否决独立于加权结果：降低 LeSTA 权重不会自动取消否决。`useLeSTATraversability()` 同时要求启用开关和有效地图；完全未收到可用地图时，这个新增风险门限不会执行。

原有风险相关边代价保留：`distance × [1 + 0.1 × (1/(1.0001-r1) + 1/(1.0001-r2))]`。因为 `Plane::traversability` 现在装入融合风险，选父节点和重连等使用该代价的流程间接受融合影响。没有另写一套 PF-RRT*、替换采样/重连算法，也没有新增沿整条边逐格查询 LeSTA 的碰撞判定。

### 4. 修正 PUTN 坡度，调整几何参数

代码：`PUTN_classes.cpp`；配置在 `putn_planning/config/for_simulation/`。

- 坡度从 `acos(z·normal)` 改为 `acos(min(1, abs(z·normal)))`，消除 SVD 法向量正负号导致同一平面坡度不同的问题，并避免浮点略超 1。
- 几何风险保留 raw 和 capped 两种值，增加分项诊断，便于判断风险饱和来源。
- map1/map2 的 `w_slope` 从 0.1 降至 0.02，`w_sparsity` 从 1.0 降至 0.2，减少坡度和稀疏项快速将风险推满的问题。
- 历史上调整过空缺阈值和拟合半径；当前 map1/map2 已与旧报告不同，不能直接照抄历史建议。

| 当前配置 | ratio_min | ratio_max | radius_fit_plane |
| --- | ---: | ---: | ---: |
| map1.yaml | 0.02 | 0.05 | 0.5 |
| map2.yaml | 0.02 | 0.05 | 1.0 |
| map_test.yaml | 0.40 | 0.85 | 0.6 |
| map1_wide.yaml | 0.02 | 0.05 | 0.5 |

PUTN sparsity 的中间段仍受空缺协方差相关判据 `1/trace > conv_thre` 约束，不是单纯对 vacancy_ratio 线性插值。没有在融合阶段把这部分替换成 LeSTA 的特征计算。

### 5. 扩展 LeSTA 发布接口和仿真适配

| 模块 | 增改内容及功能 |
| --- | --- |
| [trav_mapping_node.cpp](LeSTA/lesta_ros/src/ros/trav_mapping_node.cpp) 及对应头文件 | 在原 PointCloud2 输出之外增加 `/lesta/mapping/traversability_grid_map`；发布 elevation、当前预测 probability/binary、累积 mapping probability/binary，供 PUTN 按格查询 |
| [TraversabilityNetwork.cpp](LeSTA/lesta_ros/src/core/TraversabilityNetwork.cpp) | 加载前检查模型文件可读性，恢复 LibTorch 异常输出；不改变网络结构或推理公式 |
| `config/include/feature_extraction_params.yaml` | `grid_res: 0.1.` 修为合法数值 `0.1` |
| `config/trav_mapping_node.yaml` | `remove_backpoints: true→false`；`binary_threshold: 0.5→0.6`，改变 mapping 配置下的处理/判定条件 |
| `config/include/mapping_visualization.yaml` | 新增地图点云和区域显示配置 |

LeSTA 特征提取、MLP、训练损失和 log-odds 累积更新主体没有在本轮融合中重新设计；mapping 节点既有累积算法继续使用，只新增了适合 PUTN 消费的消息接口与可选图层。

### 6. 新增融合诊断点云和启动配置

`PUTN_planner.cpp` 新增 `pubFusedTraversabilityCloud()`，发布 `/global_planning_node/fused_traversability_cloud`。包含 XYZ 和 11 个诊断字段：`fused_risk`、`putn_risk`、`putn_raw_risk`、四个 PUTN 分项、`lesta_probability`、`lesta_risk`、`lesta_observed`、`lesta_traversable`。

该点云来自当前规划树有效节点，不是全地图、也不是专门的最终路径采样集。因此不能直接用其分布代表全场景风险分布；未观测节点的默认概率也必须结合 observed 字段解释。

`putn_launch/launch/simulation.launch` 增加 LeSTA prediction/mapping 分支、话题重映射、模型/权重/图层参数、可选地图补点和控制节点开关；新增 `fusion_simulation.launch` 封装融合运行；`bringup.launch` 补充真实运行接入。`putn_planning` 的 CMake/package 增加 grid_map 相关依赖，真实场景 YAML 增加融合参数；RViz 增加风险显示。以上属于接口/配置，不能另算网络创新。

当前入口默认值应分别记录：

| 入口 | 策略 | PUTN/LeSTA 权重 | 风险门限 | 地图来源与使用层 |
| --- | --- | --- | --- | --- |
| simulation.launch | legacy_weighted_sum | 0.3 / 0.7 | 0.75 | 默认 prediction；切为 mapping 时默认读取 mapping 层 |
| fusion_simulation.launch | risk_floor_weighted | 0.75 / 0.25 | 0.70 | mapping 消息，但读取 traversability 层 |
| map_test_nav_eval_with_sim.launch | risk_floor_weighted | 0.75 / 0.25 | 0.70 | mapping 消息，但读取 traversability 层 |

“使用 mapping 节点”与“使用累积 mapping/probability 层”并非同一件事。现行专用融合入口使用 mapping 消息中的非累积预测层，避免把长期累积饱和概率直接当默认融合输入。

### 7. 控制链路的修补

- [local_planner.py](putn/src/putn/putn_mpc/scripts/local_planner.py)：删除固定 300 点缓冲假设，改为按输入长度动态分配；增加可选最大点数、超长警告和空路径处理。作用是防止长路径越界和无路径时继续使用旧参考，不是修改 MPC 目标函数。
- [controller.py](putn/src/putn/putn_mpc/scripts/controller.py)：新增 `/controller/mode_cmd` 远程手动/自动切换及 `/controller/mode` 状态反馈；手动角速度上限/步长参数化，默认 2.0 rad/s 和 0.2 rad/s。便于自动实验控制启停，增加手动转向能力。
- MPC 求解器、GPR 主体在本次基线差异中没有对应算法重写；`__pycache__` 改动不计入算法内容。

### 8. 底盘定义、动力学和传感器

| 文件 | 原内容→当前内容 | 功能 |
| --- | --- | --- |
| [scout_skid_steer.hpp](putn/src/scout_simulator/scout_gazebo_sim/include/scout_gazebo/scout_skid_steer.hpp) 与 `.cpp` | 转向系数 `SCOUT_WHEELBASE=0.498` → `SCOUT_HALF_TRACK=0.58306/2` | 差速转向使用横向半轮距，轮速为 `(v±ωb/2)/r`；轮半径仍为 0.16459，保留左右关节符号约定 |
| [scout_v2.gazebo](putn/src/scout_simulator/scout_description/urdf/scout_v2.gazebo) | 四轮 `mu1=mu2=10` → `mu1=2.0, mu2=0.20`；`minDepth=0.01→0.001` | 降低侧向抗滑，允许滑移转向，调整接触参数；不是改变车体尺寸 |
| [scout_v2_control.yaml](putn/src/scout_simulator/scout_gazebo_sim/config/scout_v2_control.yaml) | 四轮 PID 的 P 5→100，i_clamp 100→200；I=20、D=0 保留 | 提高轮速跟踪能力；另增 `gazebo_ros_control/pid_gains`，让 Gazebo 硬件适配层读到对应增益 |
| [scout_v2_rgb_camera.xacro](putn/src/scout_simulator/scout_description/urdf/scout_v2_rgb_camera.xacro) | 新增前置 RGB 相机，安装位置 `0.43 0 0.34` | 输出 `/front_rgb_camera/image_raw` 与 camera_info，供观察/采集；当前融合仍使用点云特征，并非 RGB 融合 |
| `spawn_scout_v2.launch`、simulation.launch | 传递 `urdf_extras`，支持加载相机附加定义 | 使传感器扩展随仿真机器人一起加载 |

本次差异未显示重新定义底盘长宽、车体质量、惯量或轮半径。将这些改动写成“底盘转向运动学与轮地接触/驱动参数修正”更准确。底盘参数同时影响纯 PUTN 和融合组，比较时应保持一致。

## 第二部分：后续实验测试编写的脚本及配套文件

### 1. 自动导航主脚本 run_map_test_nav_eval.py

位置：[scripts/run_map_test_nav_eval.py](putn/src/putn/putn_launch/scripts/run_map_test_nav_eval.py)。最初新增于 `b373dfb`，之后持续扩展，目前包含工作区修改。

| 子模块/函数 | 新增或演进的内容 | 功能 |
| --- | --- | --- |
| 参数读取、任务生成、`directional_tasks()` | 从固定任务 YAML 扩展到命令行单任务、双向任务和每方向样本配额 | 统一纯 PUTN/融合实验入口与任务标识 |
| `warm_up_planner()`、`run_fixed_motion_warmup()`、`run_coverage_warmup()` | 从等待/固定运动扩展为覆盖航点预热；支持 O-B-C-D-A-O 路线插值、短距离脱离盲区、分段超时/重试和覆盖检查 | 先积累可规划点云及 LeSTA 地图，再开始正式试验 |
| `prepare_episode_start()`、`navigate_to_start()` | 保留 Gazebo 复位选项，当前组合入口默认自然导航到起点；不默认在预热后瞬移 | 减少 A-LOAM 位姿不连续，确认真实起点满足容差 |
| `update_control_pose()`、`goal_in_control_frame()`、`refresh_compensated_goal()` | 区分 TF 控制位姿和 Gazebo 真值；可用当前二维旋转/平移关系把物理目标映射到漂移后的控制坐标系，必要时重发 | 避免将 SLAM 目标到达误当物理目标到达；组合入口默认开启真值目标补偿，单评测入口默认关闭 |
| `global_path_cb()`、`update_active_path()` | 解析 Float32MultiArray 全局路径；调整路径方向、检查起点偏差、控制切换频率、保留活动路径、检查过期和航向突变 | 降低滚动重规划导致的跟踪振荡 |
| `publish_path_follow_cmd()` | 新增直接发布 `/cmd_vel` 的前视路径跟踪；航向比例控制、近目标降速、原地转向滞回、可选倒车滞回及角速度限制 | 实际执行 PF-RRT* 路径；默认启用，因此该脚本不仅是被动统计器 |
| `rotate_relative()`、`recover_heading_for_projection()` | 按测得航向旋转、改变朝向重新尝试投影/规划 | 应对无路径或局部投影困难；属于实验执行策略 |
| `verify_cmd_vel_ownership()` | 检查命令发布者 | 避免评测跟踪器与原 controller 同时争抢速度话题 |
| `wait_for_lesta_ready()`、`check_session_health()` | 检查地图/位姿/TF 新鲜度、底盘状态和 TF–Gazebo 偏差；严重倾斜锁存停车 | 阻止不健康状态进入正式样本，并请求会话重启 |
| `run_episode()` | 目标到达、超时、无路径、卡住、危险区/碰撞代理、倾斜及定位异常判断 | 记录成功或明确失败原因；不是只保存成功样本 |
| `nearby_risk_values()`、`distances_to_hazards()` | 从邻近规划树风险点取值；按配置坑/柱几何计算距离 | 提供风险暴露和障碍接近指标；几何距离判据不等同于 Gazebo 接触传感器实测碰撞 |
| `update_runtime_metrics()`、报告函数 | 记录轨迹距离、耗时、规划长度、风险、姿态峰值/位置、升降高度、漂移均值/RMS、会话和全 campaign 累计时长/里程；分方向和会话汇总 | 支持定位失败地形及区分累计运行退化与单次失败 |
| `configure_campaign()`、`load_campaign_rows()`、`save_campaign_checkpoint()` | campaign 配置校验、原子 JSON 检查点、恢复未完成配额、方向次序平衡、会话尝试上限 | 支持跨仿真重启续跑，避免不同实验设置混入同一 campaign |

样本口径：准备或健康门控未通过的尝试可标记为不进入导航成功率；已开始的正式失败仍应占有效样本配额。报告中保留排除原因，不能把“有效样本”理解为“成功样本”。

配套文件：

- [map_test_nav_eval.yaml](putn/src/putn/putn_launch/config/map_test_nav_eval.yaml)：任务、场景危险区、阈值、输出字段及实验配置。
- [map_test_nav_eval.launch](putn/src/putn/putn_launch/launch/map_test_nav_eval.launch)：在已有仿真中启动评测。
- [map_test_nav_eval_with_sim.launch](putn/src/putn/putn_launch/launch/map_test_nav_eval_with_sim.launch)：启动仿真、融合/基线、评测及可选诊断录包；默认一次正式尝试后结束会话，便于外层重启。
- `putn_launch/CMakeLists.txt`、`package.xml`：脚本安装和 ROS/Python/录包等运行依赖。

实验解释：使用该默认评测入口，测试的是“PF-RRT* 风险规划 + 评测脚本跟踪控制 + 当前底盘/定位辅助”的系统。不能把这些结果直接描述为未经改动的 PUTN 原生 MPC 全链路成绩。

### 2. 单目标测试 run_manual_goal_eval.py

位置：[scripts/run_manual_goal_eval.py](putn/src/putn/putn_launch/scripts/run_manual_goal_eval.py)；入口 [manual_goal_eval.launch](putn/src/putn/putn_launch/launch/manual_goal_eval.launch)。新增文件，已提交。

发布一个手工指定的 `/goal`，从 `/gazebo/model_states` 读取实际位置和姿态，记录到达时间、成功、超时、倾覆、最大倾角、最近目标距离和控制模式等，输出 CSV。可请求 controller 自动模式；结束后请求手动模式并等待确认，避免试验结束后仍在自动运行。

适合单次联调。它与批量评测脚本不同，主要通过既有 controller 执行，不包含同等规模的覆盖预热、独立会话配额管理。

### 3. 独立会话批量运行 run_isolated_nav_campaign.py

位置：[scripts/run_isolated_nav_campaign.py](putn/src/putn/putn_launch/scripts/run_isolated_nav_campaign.py)。新增、尚未跟踪。

读取 campaign JSON 检查点；未完成则调用 roslaunch；一次会话正常结束后重新启动完整仿真，直到配额完成或达到会话上限。每会话选择独立 ROS/Gazebo master 端口，减轻残留进程/通信影响；支持重启间隔、会话上限和 Ctrl-C 后续跑。

异常退出或没有生成检查点会停止，不是对所有错误无限重试。它不计算融合风险，作用是把“连续耐久实验”和“每次重新预热的独立试验”分开。

### 4. 雷达盲区补点 patch_lidar_blind_spot.py

位置：[scripts/patch_lidar_blind_spot.py](putn/src/putn/putn_launch/scripts/patch_lidar_blind_spot.py)。新增、尚未跟踪。它虽是实验配套脚本，但直接改变规划输入，算法归因时必须单独注明。

输入 `/laser_cloud_map`，通过 TF 找机器人位置，在周围 0.30–1.20 m 环带收集低于机器人的点，取最近至多 40 点的高度中位数；在车底半宽 0.30 m、间隔 0.10 m 的方形区域追加水平点，输出 `/laser_cloud_map_patched`，帮助起始位置平面投影。没有候选地面时原样转发。

它补的是 PUTN 规划地图，不改变原始 `/velodyne_points`，也不直接修改 LeSTA 网络。普通 simulation 默认关闭，组合评测入口默认开启。补点可能影响车底几何风险和空缺判定，比较两组时必须统一开关，并在实验方法中披露。

### 5. 数据采集、离线推理、标签生成与诊断回放

以下是新增 launch 编排文件，当前均尚未跟踪；不应误称为新增训练/标注算法。

| 文件 | 功能 |
| --- | --- |
| [map1_wide_bag_collection.launch](putn/src/putn/putn_launch/launch/map1_wide_bag_collection.launch) | 关闭 LeSTA 和原局部控制，供人工驾驶采集；记录雷达、TF、时钟、Gazebo 真值、里程计、速度命令和建图/路径等，支持 lz4 和分包；本身不发运动命令 |
| [map1_wide_bag_lesta_test.launch](putn/src/putn/putn_launch/launch/map1_wide_bag_lesta_test.launch) | 配置 prediction/mapping 节点、模型、world/base_link 和地图窗口，用已有 bag 输入验证 LeSTA；需配合 bag 播放 |
| [map1_wide_label_generation.launch](putn/src/putn/putn_launch/launch/map1_wide_label_generation.launch) | 调用独立改进 LeSTA 已有 label_generation_node，覆盖仿真话题/坐标系和地图窗口，供离线生成标签 |
| [map1_wide_diagnostic_bag_replay.launch](putn/src/putn/putn_launch/launch/map1_wide_diagnostic_bag_replay.launch) | 使用仿真时钟播放诊断 bag，支持默认暂停、起始时间、速率及 RViz；查看既有记录 |
| `rviz_config/map1_wide_lesta_bag.rviz` | 汇总雷达、LeSTA 图层和融合风险显示 |

`MAP1_WIDE_LESTA_BAG_COLLECTION.md` 是操作说明；`AC_ISOLATED_NAV_TEST_RESULTS...`、`CA_LESTA_W025_DIAGNOSTIC_RECORDING...` 等为结果/诊断记录。`CA_TRAVERSABILITY_VALIDATION_DRAFT...` 和 `RISK_MODULE_EVALUATION_DRAFT...` 是方案草案，不能把其中计划的评测、消融或指标实现算作新增脚本。

### 6. 测试场景与删改情况

- 新增 `putn_map/worlds/map_test.world`、`launch/map_test.launch` 和 `putn_planning/config/for_simulation/map_test.yaml`。场景经过两次简化，最终约 16×16 m，保留 3 个深坑和 12 个柱体；早期轻微起伏/自然坡面方案没有保留在最终测试地图中。
- 新增 `putn_map/map1_wide/model.config`、`model-1_4.sdf`、`worlds/map1_wide.world`、`launch/map1_wide.launch` 和规划 YAML；提供扩展场景，规划 XY 边界为 `[-25,25]`，用于覆盖预热和较长距离导航。
- `putn_map/launch/start.launch` 调整仿真运行配置；场景、出生位姿和可视化开关通过启动参数传递。这些是实验条件，不是风险融合公式。
- 整体没有删除原 PF-RRT*、MPC、LeSTA 网络或 log-odds 建图模块。主要“删改”是替换固定路径缓冲、修正坡度表达式、修正转向系数、调整参数和默认实验流程。旧融合、Gazebo 复位、固定运动预热仍保留为可选模式，并非全部删除。

### 核对结论与使用边界

已实现的融合核心是：LeSTA GridMap 接入 → 统一风险语义 → 可选风险融合 → 二值/阈值准入 → 原 PUTN 风险代价驱动规划。几何修正、底盘修正和评测执行策略是另外几类会影响成绩的变更，报告实验时应分别写明。

本文完成源码/配置/提交差异核对，未重新编译或运行 ROS 仿真实验，不据此声称性能提升或新增实验验证结果。
