# LeSTA-PUTN 今日修改与测试思路总结

日期：2026-08-13

本文档总结今日围绕 PUTN 与改进后 LeSTA 可通行性融合测试所做的主要修改、关键判断、验证方式和后续实验设计。当前重点是：尽量复用 PUTN 自带仿真环境，在 `fusion_simulation.launch` 中手动测试 baseline 与 LeSTA-PUTN 融合算法，并为后续自动化测试和实机移植准备接口。

## 1. 今日核心目标

今日工作的主线是将 LeSTA 的局部 prediction 可通行性输入，逐步扩展为可选择使用 LeSTA mapping 节点维护的全局/累积可通行性地图，并确认其能被 PUTN 的 RRT* 全局规划逻辑正确引用。

目标可以拆成四点：

1. `fusion_simulation.launch` 可以通过参数选择 baseline、prediction 融合、mapping 融合。
2. LeSTA mapping 输出的 GridMap 可以作为 PUTN 的 LeSTA traversability 输入。
3. 确认 LeSTA 数值含义与 PUTN 风险含义没有反向。
4. 规避 RViz 直接显示 GridMap 导致闪退的问题，保留稳定可视化方式。

## 2. 当前 launch 接口

主要手动测试入口仍然是：

```bash
roslaunch putn_launch fusion_simulation.launch
```

关键参数如下：

```text
scene                         仿真地图，例如 map1/map2
use_lesta                     是否启用 LeSTA 融合
lesta_map_source              prediction 或 mapping
lesta_model_path              LeSTA TorchScript/LibTorch 权重
lesta_putn_weight             PUTN 几何风险权重
lesta_weight                  LeSTA 风险权重
lesta_unknown_penalty         LeSTA 无观测区域附加风险
lesta_risk_threshold          融合风险拒绝阈值
lesta_mapping_map_length_x    LeSTA mapping 地图 x 尺寸
lesta_mapping_map_length_y    LeSTA mapping 地图 y 尺寸
lesta_mapping_grid_resolution LeSTA mapping 分辨率
launch_rviz                   是否随仿真启动 RViz
rviz_required                 RViz 崩溃是否终止 roslaunch
launch_local_control          是否启动本地控制器/键盘控制窗口
```

推荐 mapping 融合手动测试命令：

```bash
roslaunch putn_launch fusion_simulation.launch \
  scene:=map1 \
  use_lesta:=true \
  lesta_map_source:=mapping \
  launch_rviz:=false \
  launch_local_control:=false \
  lesta_mapping_map_length_x:=30.0 \
  lesta_mapping_map_length_y:=30.0 \
  lesta_model_path:=/home/whr/Data/test_pt/proposed/epoch_best.pt
```

若需要 baseline PUTN：

```bash
roslaunch putn_launch fusion_simulation.launch \
  scene:=map1 \
  use_lesta:=false \
  launch_rviz:=false \
  launch_local_control:=false
```

若需要 prediction 融合：

```bash
roslaunch putn_launch fusion_simulation.launch \
  scene:=map1 \
  use_lesta:=true \
  lesta_map_source:=prediction \
  launch_rviz:=false \
  launch_local_control:=false \
  lesta_model_path:=/home/whr/Data/test_pt/proposed/epoch_best.pt
```

## 3. LeSTA mapping 接入 PUTN 的方式

当前 `simulation.launch` 中，`lesta_map_source` 决定 LeSTA 数据源：

```text
lesta_map_source:=prediction
  启动 lesta_trav_prediction
  PUTN 订阅 /lesta/prediction/traversability
  使用 layer:
    traversability/probability
    traversability/binary

lesta_map_source:=mapping
  启动 lesta_trav_mapping
  PUTN 订阅 /lesta/mapping/traversability_grid_map
  使用 layer:
    mapping/probability
    mapping/binary
```

今日修正了一个重要问题：LeSTA mapping 实际发布的累积概率 layer 名称是：

```text
mapping/probability
mapping/binary
```

不是：

```text
traversability/log_odds_probability
traversability/log_odds_binary
```

如果使用错误 layer 名称，PUTN 会查不到 LeSTA mapping 的可通行性信息，导致 mapping 融合实际失效。

当前已经在 `simulation.launch` 中修正：

```xml
<param if="$(eval arg('lesta_map_source') == 'mapping')"
       name="planning/lesta_probability_layer"
       value="mapping/probability"/>

<param if="$(eval arg('lesta_map_source') == 'mapping')"
       name="planning/lesta_binary_layer"
       value="mapping/binary"/>
```

## 4. mapping 模式下 lesta_risk 的含义

LeSTA 输出的是“可通行概率”，PUTN 内部用于规划拒绝和融合的是“风险值”。

mapping 模式下：

```text
LeSTA probability = mapping/probability
PUTN lesta_risk  = 1.0 - mapping/probability
```

也就是说：

```text
mapping/probability = 1.0  -> lesta_risk = 0.0，低风险
mapping/probability = 0.5  -> lesta_risk = 0.5，中等/不确定
mapping/probability = 0.0  -> lesta_risk = 1.0，高风险
```

PUTN 融合公式为：

```cpp
fused_risk = lesta_putn_weight * geometric_traversability
           + lesta_weight      * (1.0 - lesta_probability);
```

因此目前数值方向是一致的：

```text
LeSTA probability 越大 -> 越可通行
PUTN fused_risk 越大   -> 越危险
```

二者通过 `1.0 - probability` 完成语义转换。

## 5. RViz 闪退问题与处理方案

今日发现：直接在 RViz 中 Add GridMap 显示 `/lesta/mapping/traversability_grid_map` 容易导致 RViz 闪退。原因大概率是 GridMap 插件在大地图、多 layer、高频更新场景下稳定性较差。

为避免 RViz 闪退导致整个仿真中断，已做两类处理。

### 5.1 RViz 不再作为 required 节点

新增：

```text
rviz_required:=false
```

默认 RViz 崩溃不会杀死 Gazebo、PUTN、LeSTA 等仿真节点。

### 5.2 支持完全不随仿真启动 RViz

新增：

```text
launch_rviz:=false
```

推荐调试阶段先不启动 RViz：

```bash
roslaunch putn_launch fusion_simulation.launch \
  scene:=map1 \
  use_lesta:=true \
  lesta_map_source:=mapping \
  launch_rviz:=false \
  launch_local_control:=false \
  lesta_model_path:=/home/whr/Data/test_pt/proposed/epoch_best.pt
```

确认 topic 正常后，再单独启动 RViz：

```bash
rviz -d $(rospack find putn_launch)/rviz_config/simulation.rviz
```

### 5.3 RViz 中优先显示 PointCloud2，而不是 GridMap

不要在 RViz 中直接 Add：

```text
/lesta/mapping/traversability_grid_map
```

优先显示 LeSTA mapping 节点原生发布的 PointCloud2：

```text
/lesta/mapping/traversability
```

RViz 设置：

```text
Display Type: PointCloud2
Topic: /lesta/mapping/traversability
Color Transformer: Intensity
Channel Name: mapping/probability
```

颜色语义：

```text
mapping/probability 越大，越可通行
```

今日也尝试添加了 `grid_map_visualization` 转换链路，输出：

```text
/lesta/mapping/traversability_cloud
/lesta/mapping/map_region
```

但实际排查中，如果 `/lesta/mapping/traversability_cloud` 没有消息，应优先使用原生：

```text
/lesta/mapping/traversability
```

## 6. 运行时排查顺序

如果 mapping 模式下看不到可通行性地图，建议按下面顺序排查。

### 6.1 检查仿真时间

```bash
rostopic hz /clock
```

如果 `/clock` 没有消息，说明 Gazebo 可能没有启动、已经退出或处于异常状态。

### 6.2 检查 LeSTA 节点

```bash
rosnode list | grep lesta
```

mapping 模式下应至少看到：

```text
/lesta_trav_mapping
/lesta_mapping_visualization
```

其中 `/lesta_mapping_visualization` 是辅助可视化节点，不是 PUTN 融合必需节点。

### 6.3 检查输入激光

```bash
rostopic hz /velodyne_points
```

如果没有 `/velodyne_points`，LeSTA mapping 不会产生输出。

### 6.4 检查 LeSTA mapping 原生输出

```bash
rostopic hz /lesta/mapping/traversability
rostopic echo -n 1 /lesta/mapping/traversability/fields
```

需要能看到类似字段：

```text
mapping/probability
mapping/binary
```

### 6.5 检查 PUTN 使用的 GridMap 输入

```bash
rostopic hz /lesta/mapping/traversability_grid_map
```

该话题是 PUTN mapping 融合使用的数据源。

### 6.6 检查 PUTN 融合输出

```bash
rostopic hz /global_planning_node/fused_traversability_cloud
rostopic echo -n 1 /global_planning_node/fused_traversability_cloud/fields
```

重点字段：

```text
fused_risk
putn_risk
lesta_probability
lesta_risk
lesta_observed
lesta_traversable
```

如果 `lesta_observed` 长期为 `0`，说明 PUTN RRT 节点采样位置没有查到有效 LeSTA map cell，可能是地图坐标系、地图范围、layer 名称或局部/全局覆盖问题。

## 7. 手动测试建议

当前建议手动测试采用“先跑仿真，再单独 RViz，再发布目标点/切换自动模式”的方式。

### 7.1 启动 mapping 融合仿真

```bash
roslaunch putn_launch fusion_simulation.launch \
  scene:=map1 \
  use_lesta:=true \
  lesta_map_source:=mapping \
  launch_rviz:=false \
  launch_local_control:=true \
  lesta_mapping_map_length_x:=30.0 \
  lesta_mapping_map_length_y:=30.0 \
  lesta_model_path:=/home/whr/Data/test_pt/proposed/epoch_best.pt
```

### 7.2 单独启动 RViz

```bash
rviz -d $(rospack find putn_launch)/rviz_config/simulation.rviz
```

RViz 中建议观察：

```text
/global_planning_node/grid_map_vis
/global_planning_node/tree_vis
/global_planning_node/path_vis
/global_planning_node/fused_traversability_cloud
/lesta/mapping/traversability
```

### 7.3 发布目标点

可通过 RViz Goal3DTool 或命令行发布 `/goal`。若使用已有 `manual_goal_eval.launch`，可以记录导航时间、是否到达、是否倾倒。

示例：

```bash
roslaunch putn_launch manual_goal_eval.launch \
  goal_x:=6.0 \
  goal_y:=0.0 \
  goal_z:=0.0 \
  method:=lesta_putn_mapping \
  task_id:=map1_manual_mapping_01 \
  timeout_s:=180.0 \
  goal_tolerance_m:=0.6 \
  roll_pitch_fail_deg:=45.0 \
  auto_manage_controller:=true
```

注意：ROS launch 参数不要写成 `goal_x:= 0.0`，`:=` 后不能有空格。应写成：

```text
goal_x:=0.0
```

## 8. A/B 对照实验思路

为了测试改进后 LeSTA 可通行性评估是否影响 PUTN 路径规划，建议至少对比三组。

### 8.1 Baseline PUTN

```bash
roslaunch putn_launch fusion_simulation.launch \
  scene:=map1 \
  use_lesta:=false \
  launch_rviz:=false \
  launch_local_control:=true
```

### 8.2 PUTN + LeSTA prediction

```bash
roslaunch putn_launch fusion_simulation.launch \
  scene:=map1 \
  use_lesta:=true \
  lesta_map_source:=prediction \
  launch_rviz:=false \
  launch_local_control:=true \
  lesta_model_path:=/home/whr/Data/test_pt/proposed/epoch_best.pt
```

### 8.3 PUTN + LeSTA mapping

```bash
roslaunch putn_launch fusion_simulation.launch \
  scene:=map1 \
  use_lesta:=true \
  lesta_map_source:=mapping \
  launch_rviz:=false \
  launch_local_control:=true \
  lesta_mapping_map_length_x:=30.0 \
  lesta_mapping_map_length_y:=30.0 \
  lesta_model_path:=/home/whr/Data/test_pt/proposed/epoch_best.pt
```

三组测试应保持相同：

```text
scene
起点
目标点
预热方式
控制器参数
评价阈值
随机种子/重复次数，如后续加入
```

重点观察：

```text
是否能规划出路径
RRT* 树扩展是否避开高风险区域
global_path 是否发生绕行
到达时间
实际轨迹长度
最小到目标距离
是否超时
是否倾倒
是否碰撞/陷入不可通行区域
fused_risk 分布
lesta_observed 比例
lesta_probability 与地形区域是否符合直觉
```

## 9. 自动化测试思路

已有自动化入口：

```bash
roslaunch putn_launch map_test_nav_eval_with_sim.launch
```

当前自动化测试已经支持：

```text
单组起点/终点
map1/map2
固定 warmup
固定起点检查
结果 CSV
到达时间
路径长度
final distance
fused risk 统计
```

后续建议将 `lesta_map_source` 也纳入自动化 A/B 测试参数，形成：

```text
putn_baseline
lesta_prediction
lesta_mapping
```

每个 case 使用相同起终点，分别运行三次或更多次，统计均值与方差。

推荐 CSV 对比指标：

```text
success
failure_reason
elapsed_time
path_length
planned_path_length
final_distance
mean_fused_risk
max_fused_risk
min_goal_distance
tipped
```

## 10. 关于是否需要重新训练 LeSTA 权重

当前使用的权重：

```text
/home/whr/Data/test_pt/proposed/epoch_best.pt
```

已确认该权重与当前 8-feature 输入配置匹配。

如果后续 LeSTA 特征集、输入归一化、训练场景或 label 语义改变，则需要重新训练或至少重新导出对应 TorchScript 权重。特别是：

```text
feature 数量改变
feature 顺序改变
归一化参数改变
模型结构改变
训练标签中 traversable/non-traversable 语义改变
```

否则会出现模型输入维度不匹配，或虽能运行但输出语义不可靠的问题。

## 11. 今日验证过的命令

编译：

```bash
cd /home/whr/putn_noetic
source /opt/ros/noetic/setup.bash
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DEMPY_SCRIPT=/usr/lib/python3/dist-packages/em.py
```

launch 节点展开：

```bash
source devel/setup.bash
roslaunch putn_launch fusion_simulation.launch \
  use_lesta:=true \
  lesta_map_source:=mapping \
  launch_local_control:=false \
  launch_rviz:=false \
  --nodes
```

确认 mapping layer 参数：

```bash
roslaunch putn_launch fusion_simulation.launch \
  use_lesta:=true \
  lesta_map_source:=mapping \
  launch_local_control:=false \
  --dump-params | rg "lesta_probability_layer|lesta_binary_layer|use_lesta_traversability"
```

期望输出：

```text
/global_planning_node/planning/lesta_binary_layer: mapping/binary
/global_planning_node/planning/lesta_probability_layer: mapping/probability
/global_planning_node/planning/use_lesta_traversability: true
```

## 12. 后续建议

短期建议：

1. 先用 `launch_rviz:=false` 确认仿真、LeSTA mapping、PUTN 融合 topic 正常。
2. RViz 中避免直接添加 GridMap display，改看 `/lesta/mapping/traversability`。
3. 用 `manual_goal_eval.launch` 做若干固定目标点的手动记录。
4. 选择 3 到 5 组 map1/map2 起终点，形成 baseline/prediction/mapping 三组对照。

中期建议：

1. 在自动化测试脚本中补充 `lesta_map_source` 参数记录。
2. 统计 `lesta_observed` 比例，确认 LeSTA mapping 对 RRT* 节点采样确实生效。
3. 记录每条路径上的平均/最大 `lesta_risk` 和 `fused_risk`。
4. 若 mapping 模式效果明显优于 prediction，再考虑实机移植。

实机前重点风险：

```text
坐标系 world/map/odom/base_link 是否一致
LeSTA mapping 全局地图是否会漂移或错位
SLAM 漂移是否导致 GridMap 与 PUTN 节点查询位置不一致
unknown 区域惩罚是否过强或过弱
lesta_risk_threshold 是否导致过度保守
```

总体结论：今日修改已经让 `fusion_simulation.launch` 具备 baseline、prediction 融合、mapping 融合三种测试入口；mapping 模式下 LeSTA 概率与 PUTN 风险语义已经对齐；RViz 闪退问题建议通过“不直接显示 GridMap、改显示 PointCloud2、必要时不随仿真启动 RViz”的方式规避。
