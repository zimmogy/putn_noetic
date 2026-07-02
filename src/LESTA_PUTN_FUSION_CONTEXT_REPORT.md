# LeSTA-PUTN 融合修改上下文报告

生成时间：2026-07-02  
工作区：`/Users/mymac/Documents/projects/putn_noetic`

## 1. 当前目标

当前工作的主线是：将 LeSTA 的学习型可通行性判断接入 PUTN 的路径规划流程，在 PUTN 原有仿真系统中完成融合算法可视化、参数调试、测试地图构建和后续自动化评测准备。

融合后的目标不是替换 PUTN，而是让 PUTN 保留原有基于几何地形的风险估计，同时订阅 LeSTA 输出的 traversability map，将两类风险融合后用于节点筛选、路径规划和 RViz 可视化。

## 2. 融合算法当前设计

### 2.1 PUTN 几何风险

PUTN 的原始几何风险仍由局部平面拟合结果得到，核心输入包括：

- `flatness`：局部平面拟合误差或粗糙程度。
- `slope`：局部平面法向量与世界 z 轴夹角。
- `sparsity`：局部点云稀疏或空缺风险。
- `vacancy_ratio`：局部拟合窗口内空缺栅格比例。

当前风险计算逻辑为：

```text
putn_raw_risk =
  w_total * (w_flatness * flatness + w_slope * slope + w_sparsity * sparsity)

putn_risk = clamp(putn_raw_risk, 0, 1)
```

注意：

- `putn_raw_risk` 是未截断风险。
- `putn_risk` 是截断后的几何风险，即 `geometric_traversability`。
- 数值越大表示风险越高、越不可通行。

### 2.2 LeSTA 风险

LeSTA 输出的 `lesta_probability` 表示可通行概率：

```text
lesta_probability 越大 -> 越可通行
lesta_risk = 1.0 - lesta_probability
lesta_risk 越大 -> 越不可通行
```

因此 `putn_risk` 和 `lesta_probability` 含义相反；`putn_risk` 和 `lesta_risk` 含义一致。

### 2.3 融合风险

若当前 PUTN 节点能查询到 LeSTA 对应位置的概率，则：

```text
fused_risk =
  lesta_putn_weight * putn_risk
  + lesta_weight * lesta_risk
```

若启用 LeSTA 但当前位置没有 LeSTA 数据，则：

```text
fused_risk = putn_risk + lesta_unknown_penalty
```

当前默认参数倾向于让 LeSTA 占更大权重：

```yaml
use_lesta_traversability: false
lesta_putn_weight: 0.3
lesta_weight: 0.7
lesta_unknown_penalty: 0.0
lesta_risk_threshold: 0.75
lesta_probability_layer: "traversability/probability"
lesta_binary_layer: "traversability/binary"
```

其中 `lesta_risk_threshold` 用于规划节点风险筛选。融合后如果 `fused_risk > lesta_risk_threshold`，该节点会倾向于被拒绝。

## 3. 已修改的核心代码

### 3.1 PUTN 数据结构扩展

文件：

- `src/putn/src/putn/putn_planning/include/PUTN_classes.h`
- `src/putn/src/putn/putn_planning/src/PUTN_classes.cpp`

主要修改：

- 新增 `LeSTATraversabilityArg`，用于保存 LeSTA 融合配置。
- `Plane` 中新增 LeSTA 与 PUTN 诊断字段：
  - `geometric_traversability`
  - `geometric_traversability_raw`
  - `putn_flatness`
  - `putn_slope`
  - `putn_sparsity`
  - `putn_vacancy_ratio`
  - `lesta_probability`
  - `has_lesta_probability`
  - `lesta_traversable`
- `World` 中新增 LeSTA map 更新与查询接口：
  - `updateLeSTATraversabilityMap`
  - `queryLeSTATraversability`
  - `setLeSTATraversabilityArg`
  - `getLeSTATraversabilityArg`
  - `useLeSTATraversability`

### 3.2 坡度计算修正

此前坡度计算存在方向符号敏感问题，当前修正为使用法向量与 z 轴夹角余弦的绝对值：

```cpp
float cos_slope = fabs(z_axies.dot(normal_vector));
if(cos_slope > 1.0f) cos_slope = 1.0f;
float slope = 180.0f * (float)acos(cos_slope) / PI;
```

这样可以避免平面法向量朝上或朝下导致同一地面被判成不同坡度。

### 3.3 规划节点筛选

文件：

- `src/putn/src/putn/putn_planning/src/PUTN_planner.cpp`

当前逻辑：

- 未启用 LeSTA 时，沿用 PUTN 原有风险判断。
- 启用 LeSTA 且查询到 LeSTA binary 层时，如果 `lesta_traversable == false`，该节点直接不可用。
- 否则使用 `fused_risk <= lesta_risk_threshold` 判断节点是否可用。

### 3.4 融合点云可视化

文件：

- `src/putn/src/putn/putn_planning/src/PUTN_planner.cpp`
- `src/putn/src/putn/putn_planning/src/global_planning_node.cpp`
- `src/putn/src/putn/putn_planning/include/PUTN_planner.h`

新增 RViz 可视化 topic：

```text
/global_planning_node/fused_traversability_cloud
```

点云字段包括：

- `x`
- `y`
- `z`
- `fused_risk`
- `putn_risk`
- `putn_raw_risk`
- `putn_flatness`
- `putn_slope`
- `putn_sparsity`
- `putn_vacancy_ratio`
- `lesta_probability`
- `lesta_risk`
- `lesta_observed`
- `lesta_traversable`

RViz 中建议：

- Display 类型：`PointCloud2`
- Topic：`/global_planning_node/fused_traversability_cloud`
- Color Transformer：`Intensity`
- Channel Name：
  - 查看融合风险：`fused_risk`
  - 查看 PUTN 几何风险：`putn_risk`
  - 查看未截断 PUTN 风险：`putn_raw_risk`
  - 查看 LeSTA 风险：`lesta_risk`
  - 查看 LeSTA 是否观测：`lesta_observed`

## 4. Launch 与仿真接入

### 4.1 普通仿真入口

文件：

- `src/putn/src/putn/putn_launch/launch/simulation.launch`

新增/使用的参数：

```xml
<arg name="use_lesta" .../>
<arg name="lesta_model_path" .../>
<arg name="lesta_putn_weight" .../>
<arg name="lesta_weight" .../>
<arg name="lesta_unknown_penalty" .../>
<arg name="lesta_risk_threshold" .../>
<arg name="scout_urdf_extras" .../>
```

启用 LeSTA 后，launch 会启动 `lesta/trav_prediction_node`，并将 LeSTA GridMap remap 到 PUTN planner 的 LeSTA 输入。

### 4.2 融合仿真入口

文件：

- `src/putn/src/putn/putn_launch/launch/fusion_simulation.launch`

该文件封装 `simulation.launch`，默认 `use_lesta=true`，用于启动 LeSTA-PUTN 融合仿真。

### 4.3 模型权重加载问题

文件：

- `src/LeSTA/lesta_ros/src/core/TraversabilityNetwork.cpp`

新增了对模型路径的可读性检查，并在 `torch::jit::load` 失败时打印更明确的错误。

需要注意：

- `epoch_best.pt` 必须是 TorchScript/LibTorch 可加载模型。
- 如果只是 PyTorch `state_dict`，`torch::jit::load` 会失败。
- 当前报错路径示例：`/home/whr/下载/epoch_best.pt`。
- 应确保路径存在、权限可读、模型格式正确。

## 5. 当前参数调试结论

### 5.1 `putn_risk` 曾经全为 1 的原因

主要原因是 PUTN 的几何风险被过早饱和：

- `w_flatness` 较大。
- `w_slope` 对坡度角度过敏。
- `w_sparsity` 在仿真激光雷达稀疏场景下容易持续为 1。
- 原先 `ratio_max` 过小，导致 vacancy ratio 稍大就被判成高稀疏风险。

### 5.2 当前针对仿真的建议参数

文件：

- `src/putn/src/putn/putn_planning/config/for_simulation/map1.yaml`
- `src/putn/src/putn/putn_planning/config/for_simulation/map2.yaml`
- `src/putn/src/putn/putn_planning/config/for_simulation/map_test.yaml`

当前重点参数：

```yaml
w_slope: 0.02
w_sparsity: 0.2
ratio_min: 0.40
ratio_max: 0.85
radius_fit_plane: 0.6
```

调参后的观测：

- `putn_risk` 能从全 1 收敛到大约 `[0.2, 1]`。
- 最小值约 0.2 的主要原因是 `putn_sparsity` 长期为 1，而 `w_sparsity = 0.2`，即使其他项接近 0，也会贡献约 0.2 风险。
- `putn_vacancy_ratio` 在仿真中约为 `0.45~0.9`。
- `putn_sparsity` 仍可能全为 1，说明当前仿真点云密度/局部窗口/栅格空缺判据仍偏保守。

### 5.3 `vacancy_ratio` 与 `sparsity`

`vacancy_ratio` 是局部拟合窗口内空缺比例，通常只在 `radius_fit_plane` 范围内统计，而不是全局地图统计。

`sparsity` 是由 `vacancy_ratio` 映射得到的风险项：

```text
vacancy_ratio <= ratio_min -> sparsity 接近 0
vacancy_ratio >= ratio_max -> sparsity 接近 1
ratio_min < vacancy_ratio < ratio_max -> 线性过渡
```

因此两者关系是：

- `vacancy_ratio` 是原始观测比例。
- `sparsity` 是规划风险归一化后的惩罚项。

## 6. LeSTA 权重与 PUTN 仿真的适配性判断

用户说明：当前 LeSTA 基于 Rellis-3D 数据集中使用的 Warthog 地面机器人适配。

分析结论：

- 直接用于 PUTN 的 Scout/仿真 URDF 时，不应认为完全适配。
- 差异包括：
  - 机器人尺寸、轮距、底盘高度不同。
  - LiDAR 安装高度和姿态不同。
  - Gazebo 仿真点云噪声、材质、遮挡模式与真实 Rellis-3D 不同。
  - LeSTA 学到的是特定机器人和数据分布下的可通行性概率。
- 当前接入方式可用于算法联调和可视化验证，但如果要严谨评估，建议基于 PUTN 仿真采集数据重新训练或微调 LeSTA 权重。

## 7. 视觉传感器接入

新增 RGB 相机 xacro：

- `src/putn/src/scout_simulator/scout_description/urdf/scout_v2_rgb_camera.xacro`

修改 spawn launch：

- `src/putn/src/scout_simulator/scout_gazebo_sim/launch/spawn_scout_v2.launch`

修改 simulation launch：

- `src/putn/src/putn/putn_launch/launch/simulation.launch`

默认相机位置：

```text
front_rgb_camera_link 相对 base_link:
xyz = 0.43 0 0.34
```

Gazebo 插件：

```text
libgazebo_ros_camera.so
```

图像 topic：

```text
/front_rgb_camera/image_raw
/front_rgb_camera/camera_info
```

查看方式：

```bash
rqt_image_view
```

然后选择 `/front_rgb_camera/image_raw`。

也可以检查频率：

```bash
rostopic hz /front_rgb_camera/image_raw
```

## 8. 测试地图设计

### 8.1 新地图文件

新增测试地图：

- `src/putn/src/putn/putn_map/worlds/map_test.world`
- `src/putn/src/putn/putn_map/launch/map_test.launch`
- `src/putn/src/putn/putn_planning/config/for_simulation/map_test.yaml`

当前版本已经按用户要求简化为：

- 缩小地图范围到约 `16 m x 16 m`。
- 只保留深坑和柱状障碍物。
- 深坑和柱状障碍物随机散落。
- 障碍物与地图边缘保持距离。
- 不再包含轻微起伏地形和自然弧度陡坡。

### 8.2 地图范围

`map_test.yaml` 中规划范围：

```yaml
map_x_l: -8.0
map_x_u: 8.0
map_y_l: -8.0
map_y_u: 8.0
```

### 8.3 深坑

当前 3 个深坑：

| id | center | size | depth |
| --- | --- | --- | --- |
| `pit_a` | `[-4.6, 2.8]` | `[1.6, 1.6]` | `0.9 m` |
| `pit_b` | `[1.8, -3.2]` | `[1.8, 1.6]` | `0.9 m` |
| `pit_c` | `[4.7, 2.8]` | `[1.6, 1.6]` | `0.9 m` |

### 8.4 柱状障碍物

当前 12 个柱状障碍物：

| id | center | radius |
| --- | --- | --- |
| `column_01` | `[-5.6, -1.2]` | `0.32` |
| `column_02` | `[-5.2, 5.4]` | `0.38` |
| `column_03` | `[-3.2, -5.1]` | `0.44` |
| `column_04` | `[-1.3, 4.4]` | `0.34` |
| `column_05` | `[-0.8, -1.1]` | `0.30` |
| `column_06` | `[1.2, 5.4]` | `0.42` |
| `column_07` | `[3.6, -5.5]` | `0.36` |
| `column_08` | `[4.8, -0.6]` | `0.46` |
| `column_09` | `[5.6, 1.0]` | `0.32` |
| `column_10` | `[5.5, -3.8]` | `0.40` |
| `column_11` | `[-2.0, 6.0]` | `0.34` |
| `column_12` | `[2.9, 1.0]` | `0.30` |

## 9. 自动化导航测试方案

### 9.1 评测配置文件

已新增：

- `src/putn/src/putn/putn_launch/config/map_test_nav_eval.yaml`

该配置用于后续自动化测试脚本读取。

### 9.2 Episode 设计

固定 10 组起点/终点，每组运行 3 次，共 30 个 episode：

```text
10 tasks * 3 runs = 30 episodes
```

所有起点和终点都设计在可到达区域，避开深坑和柱状障碍物。

### 9.3 需要统计的指标

每个 episode 记录：

- `success`：是否成功到达目标。
- `failure_reason`：`timeout / collision / pit / stuck / no_path`。
- `time_to_goal`：到达耗时。
- `path_length`：实际行驶路径长度。
- `planned_path_length`：规划路径长度。
- `min_distance_to_pit`：距离坑最近距离。
- `min_distance_to_column`：距离柱最近距离。
- `mean_fused_risk`：路径平均融合风险。
- `max_fused_risk`：路径最大融合风险。
- `replan_count`：重规划次数。

### 9.4 判定阈值

当前配置：

```yaml
goal_tolerance_m: 0.5
timeout_s: 120.0
stuck_speed_threshold_mps: 0.03
stuck_duration_s: 8.0
pit_margin_m: 0.35
column_margin_m: 0.35
roll_pitch_fail_deg: 35.0
```

### 9.5 后续待实现脚本

用户最新需求是“编写测试脚本，根据测试方案进行自动化测试”。

推荐下一步新增：

```text
src/putn/src/putn/putn_launch/scripts/run_map_test_nav_eval.py
```

脚本建议功能：

1. 读取 `map_test_nav_eval.yaml`。
2. 按 10 个 task、每个 3 次循环运行。
3. 使用 Gazebo `/gazebo/set_model_state` 重置机器人位置。
4. 向 `/goal` 发布 `geometry_msgs/PoseStamped` 目标。
5. 订阅机器人位姿：
   - 优先 `/gazebo/model_states`。
   - 可选 `/odom`。
6. 订阅规划路径：
   - 可能为 `/global_planning_node/global_path`。
7. 订阅融合风险点云：
   - `/global_planning_node/fused_traversability_cloud`。
8. 按 episode 统计路径长度、规划长度、最近坑/柱距离、平均/最大融合风险、重规划次数。
9. 将结果写入 CSV。

建议同时新增 launch：

```text
src/putn/src/putn/putn_launch/launch/map_test_nav_eval.launch
```

用于在仿真已经启动后运行自动评测脚本。

## 10. 运行建议

### 10.1 启动 map_test 融合仿真

示例：

```bash
roslaunch putn_launch fusion_simulation.launch map:=map_test
```

如果 launch 参数名称与实际文件不完全一致，应以 `simulation.launch` 中的 arg 为准。

### 10.2 检查 LeSTA 是否正常

```bash
rostopic list | grep lesta
rostopic hz /lesta/prediction/traversability_cloud
```

### 10.3 检查 PUTN 融合点云

```bash
rostopic hz /global_planning_node/fused_traversability_cloud
```

RViz 中查看不同 channel：

- `fused_risk`
- `putn_risk`
- `putn_raw_risk`
- `putn_vacancy_ratio`
- `lesta_probability`
- `lesta_risk`
- `lesta_observed`

### 10.4 检查视觉传感器

```bash
rqt_image_view
```

选择：

```text
/front_rgb_camera/image_raw
```

## 11. 已知问题与注意事项

### 11.1 LeSTA 权重可能不适配 PUTN 仿真

当前 LeSTA 权重来自 Rellis-3D/Warthog 分布，和 PUTN 仿真中的 Scout/URDF/Gazebo 点云分布不同，因此建议后续基于 PUTN 仿真采集数据重新训练或微调。

### 11.2 小土丘未被判不可通行不一定是错误

此前 map1 中下方两个土丘没有显示为 LeSTA 不可通行，原因可能包括：

- 轻微土丘对当前机器人可能本来可通行。
- LiDAR 被地形遮挡导致后方点云缺失，不代表土丘本身应被判为障碍。
- LeSTA 的训练标签与机器人动力学/坡度能力相关，不能仅凭视觉高度判断。

### 11.3 仿真中的 `putn_sparsity` 可能偏保守

如果仿真点云密度、扫描线分布或局部栅格分辨率导致 `vacancy_ratio` 长期偏高，则 `putn_sparsity` 会长期为 1。

这在真实环境中可能缓解，但不一定自动消失，取决于：

- 真实 LiDAR 线数与点云密度。
- 地面反射质量。
- 机器人速度。
- 地图分辨率。
- `radius_fit_plane`、`ratio_min`、`ratio_max` 是否重新标定。

### 11.4 ROS Noetic 编译时不要使用 Anaconda Python 3.10

曾出现错误：

```text
ImportError: cannot import name 'Sequence' from 'collections'
```

原因是 catkin 使用了 `/home/whr/anaconda3/bin/python3` 和 Anaconda 里的旧包组合。建议编译前退出 conda 或显式使用系统 Python：

```bash
conda deactivate
source /opt/ros/noetic/setup.bash
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
```

### 11.5 ROS log 目录过大

曾出现：

```text
WARNING: disk usage in log directory [/home/whr/.ros/log] is over 1GB.
```

可清理：

```bash
rosclean check
rosclean purge
```

## 12. 下一步优先级

1. 实现自动化评测脚本 `run_map_test_nav_eval.py`。
2. 增加对应 launch 文件，便于一键运行 30 个 episode。
3. 在 map_test 上跑一次无 LeSTA PUTN baseline。
4. 在 map_test 上跑一次 LeSTA-PUTN fusion。
5. 对比成功率、失败原因、路径长度、融合风险与最近障碍距离。
6. 如果 LeSTA 权重不稳定，开始基于 PUTN 仿真采集数据，生成适配 Scout 仿真的 LeSTA 训练集。

## 13. 快速上下文恢复摘要

如果后续上下文被压缩，只需记住：

- 融合风险：`fused_risk = 0.3 * putn_risk + 0.7 * (1 - lesta_probability)`。
- `putn_risk` 和 `lesta_risk` 越大越危险；`lesta_probability` 越大越安全。
- RViz 融合点云 topic：`/global_planning_node/fused_traversability_cloud`。
- 已有诊断字段：`fused_risk / putn_risk / putn_raw_risk / putn_vacancy_ratio / lesta_probability / lesta_risk / lesta_observed`。
- 新测试地图：`map_test.world`，16m x 16m，仅深坑和柱状障碍物。
- 自动评测配置：`map_test_nav_eval.yaml`，10 组起终点，每组 3 次，共 30 episodes。
- 下一步就是写自动化评测脚本，读取上述 YAML，重置 Gazebo、发布目标、采集路径和风险指标、输出 CSV。
