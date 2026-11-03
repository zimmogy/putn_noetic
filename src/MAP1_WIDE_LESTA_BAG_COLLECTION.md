# map1_wide LeSTA 数据采集

## 采集

```bash
cd /home/whr/putn_noetic
source devel/setup.bash
roslaunch putn_launch map1_wide_bag_collection.launch
```

该入口启动 `map1_wide` 和录包节点，但不会发布任何运动指令。它将 bag 分卷写入
`~/.ros/map1_wide_lesta_raw_<时间戳>_*.bag`，每卷默认 4 GiB，使用 LZ4
压缩。关闭 LeSTA 是为了避免现有模型影响训练数据的地形分布。

在第二个终端启动键盘控制：

```bash
source /home/whr/putn_noetic/devel/setup.bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py \
  _speed:=0.35 _turn:=0.7 cmd_vel:=/cmd_vel
```

保持第二个终端处于焦点，根据终端内的按键说明驾驶。建议低速行驶，依次覆盖
O-B-C-D-A-O、各个平台边缘、斜坡、凹凸和稀疏点云区域；同一区域最好从相反方向
再经过一次。不要长时间原地旋转，否则会产生大量高度重复的样本。

完成路线后，先松开按键让机器人停止，再在第一个终端按 `Ctrl-C`。等待 rosbag
输出 `Closing ...bag` 后再关闭终端，以确保最后一个分卷正确写盘。

结束后检查：

```bash
rosbag info ~/.ros/map1_wide_lesta_raw_*.bag
```

至少应存在且消息数非零：`/velodyne_points`、`/tf`、`/tf_static`、`/clock`
和 `/gazebo/model_states`。采集过程中应确认机器人确实走完覆盖路线；bag 记录时长
本身不代表已经覆盖整个环境。

可修改输出前缀和分卷大小：

```bash
roslaunch putn_launch map1_wide_bag_collection.launch \
  bag_prefix:=/绝对路径/map1_wide_run01 \
  split_size_mb:=2048
```

输出目录必须事先存在。建议至少采集 3 次，并改变行驶方向或起点，以减少单一轨迹
带来的标签偏差。训练/验证/测试划分应按整次 run 划分，不要把同一 bag 的相邻帧
随机拆到不同集合。

## 回放并生成 LeSTA 标签

先启动针对仿真 bag 配置好的标签节点：

```bash
roslaunch putn_launch map1_wide_label_generation.launch
```

另一个终端按顺序回放同一次采集的所有分卷：

```bash
printf '%s\0' ~/.ros/map1_wide_lesta_raw_<时间戳>_*.bag | \
  sort -zV | xargs -0 rosbag play --clock
```

回放完成后保存标签图（第二个字符串参数是输出目录）：

```bash
rosservice call /lesta/save_label_map "map1_wide_run01" "/绝对输出目录"
```

不要使用 `rosbag play -r 3` 作为首次生成标签的默认值；大点云和 500 m 全局网格下，
加速回放可能让处理节点丢帧。先用 1 倍速，确认无 TF lookup 错误后再尝试提速。

## 使用已有权重测试 prediction / mapping

Prediction：

```bash
roslaunch putn_launch map1_wide_bag_lesta_test.launch mode:=prediction
```

Mapping：

```bash
roslaunch putn_launch map1_wide_bag_lesta_test.launch mode:=mapping
```

两个模式都会默认加载专用 RViz 配置
`putn_launch/rviz_config/map1_wide_lesta_bag.rviz`。其中红色表示低可通行概率，
绿色表示高可通行概率；左侧可分别开关 `LeSTA Prediction`、`LeSTA Mapping`、
原始 Velodyne 和 TF。无图形界面运行时添加 `launch_rviz:=false`。

另一个终端回放：

```bash
rosbag play /home/whr/Data/rosbags/lesta_putn/map1_wide_lesta_raw.bag \
  --clock --topics /velodyne_points /tf /tf_static
```

默认加载 `/home/whr/Data/test_pt/proposed/epoch_best.pt`。可通过
`model_path:=/path/to/model.pt` 更换 TorchScript 权重。

这个 bag 已经录有 `/clock`。上面的 `--topics` 会排除录制时钟，再由 `--clock`
生成唯一的回放时钟。不要不加 `--topics` 就直接使用 `--clock`，否则两个时钟源会
引起时间回跳、TF buffer 清空和间歇性 TF lookup 失败；也不要简单去掉 `--clock`，
否则在 `use_sim_time=true` 时播放器可能等待时钟而无法开始。
