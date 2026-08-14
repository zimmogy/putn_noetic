# putn_noetic

This workspace contains a ROS Noetic version of PUTN with LeSTA traversability fusion, simulation launch files, and map-test navigation evaluation tools.

## Build

```bash
cd /home/whr/putn_noetic
source /opt/ros/noetic/setup.bash
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DEMPY_SCRIPT=/usr/lib/python3/dist-packages/em.py
source devel/setup.bash
```

## Simulation Launch Interfaces

Baseline PUTN simulation, without LeSTA fusion:

```bash
roslaunch putn_launch simulation.launch scene:=map1 use_lesta:=false
```

LeSTA-PUTN fusion simulation:

```bash
roslaunch putn_launch fusion_simulation.launch \
  scene:=map1 \
  launch_local_control:=false \
  lesta_model_path:=/home/whr/Data/test_pt/proposed/epoch_best.pt
```

Use the local prediction map, the default mode:

```bash
roslaunch putn_launch fusion_simulation.launch \
  scene:=map1 \
  use_lesta:=true \
  lesta_map_source:=prediction \
  launch_local_control:=false \
  lesta_model_path:=/home/whr/Data/test_pt/proposed/epoch_best.pt
```

Use LeSTA's accumulated traversability mapping node as the PUTN risk source:

```bash
roslaunch putn_launch fusion_simulation.launch \
  scene:=map1 \
  use_lesta:=true \
  lesta_map_source:=mapping \
  rviz_required:=false \
  launch_local_control:=false \
  lesta_model_path:=/home/whr/Data/test_pt/proposed/epoch_best.pt
```

Important launch arguments:

- `scene`: simulation map name, such as `map1`, `map2`, or `map_test`.
- `lesta_model_path`: TorchScript/LibTorch `.pt` model used by `lesta/trav_prediction_node`.
- `lesta_putn_weight`: PUTN geometric risk weight, default `0.3`.
- `lesta_weight`: LeSTA risk weight, default `0.7`.
- `lesta_unknown_penalty`: added risk when LeSTA is enabled but no LeSTA cell is available, default `0.0`.
- `lesta_risk_threshold`: fused risk threshold used to reject planning nodes, default `0.75`.
- `lesta_map_source`: `prediction` uses `/lesta/prediction/traversability` layers `traversability/probability` and `traversability/binary`; `mapping` uses `/lesta/mapping/traversability_grid_map` layers `mapping/probability` and `mapping/binary`.
- `lesta_mapping_map_length_x`, `lesta_mapping_map_length_y`: accumulated LeSTA mapping window size in meters, default `40.0`.
- `lesta_mapping_grid_resolution`: LeSTA mapping grid resolution in meters, default `0.1`.
- `launch_rviz`: whether to start RViz together with the simulation, default `true`.
- `rviz_required`: whether RViz crash should stop the whole launch, default `false`. Keep this false when testing GridMap displays.
- `launch_local_control`: whether to start the local planner and controller terminal nodes.

For the current 8-feature LeSTA configuration, use `/home/whr/Data/test_pt/proposed/epoch_best.pt` or another compatible 8-feature TorchScript model.

## Test Maps

Early simulation evaluation should use the built-in PUTN maps:

- `map1`
- `map2`

The workspace also keeps a dedicated synthetic test map:

- World: `src/putn/src/putn/putn_map/worlds/map_test.world`
- Launch: `src/putn/src/putn/putn_map/launch/map_test.launch`
- Planner config: `src/putn/src/putn/putn_planning/config/for_simulation/map_test.yaml`

## Automated Navigation Evaluation

The evaluator can run one start-goal pair at a time. Change `scene`, `start_*`, and `goal_*` from the launch command line to test different cases on `map1` or `map2`.

For repeatable simulation tests, `map_test_nav_eval_with_sim.launch` uses a fixed warmup sequence by default:

1. Put the robot at the configured `start_*` position with Gazebo.
2. Execute a deterministic `cmd_vel` warmup sequence to collect enough local map and traversability information.
3. Keep SLAM/odometry continuous; do not teleport the robot back after warmup by default.
4. Check whether the warmup ended within `start_tolerance_m` of the configured `start_*`.
5. Clear evaluator-only statistics, publish `goal_*`, then start timing and recording.

This avoids SLAM discontinuities caused by teleporting after warmup. If the fixed warmup motion does not naturally return the robot near the configured start, the episode is marked as `warmup_not_at_start` instead of continuing with an invalid start state.

Single-case test arguments:

- `scene`: test map, recommended `map1` or `map2`.
- `use_lesta`: `true` for LeSTA-PUTN fusion, `false` for PUTN baseline.
- `method`: label written to the CSV, for example `lesta_putn` or `putn_baseline`.
- `task_id`: label for this start-goal case, such as `map1_case_01`.
- `start_x`, `start_y`, `start_z`: robot start position in the `world` frame.
- `spawn_z_offset`: only used by `map_test_nav_eval_with_sim.launch`; Gazebo spawns the robot at `start_z + spawn_z_offset`, default `1.0`, to avoid dropping the URDF into the ground.
- `spawn_yaw`: initial Gazebo yaw angle for the Scout model, default `0.0`.
- `goal_x`, `goal_y`, `goal_z`: navigation goal position in the `world` frame.
- `runs`: repeat count for this single start-goal pair. Use `1` for quick checks.
- `lesta_model_path`: required when `use_lesta:=true`; pass a valid TorchScript/LibTorch `.pt` model.
- `start_mode`: robot placement mode before each run. For simulation fixed-start tests, use the default `gazebo`.
- `warmup_mode`: default `fixed_motion` in `map_test_nav_eval_with_sim.launch`; use `stationary` for old static warmup or `none` to disable warmup.
- `warmup_cmd_sequence`: semicolon-separated `linear_x,angular_z,duration_s` phases. The combined launch defaults to an enlarged near-square warmup loop, about four 1.4 m sides: `0.18,0.00,8.0;0.00,0.45,3.49;...`.
- `warmup_reset_to_start`: default `false`; keep this disabled for SLAM-continuous tests. Setting it to `true` teleports the robot back with Gazebo and may break A-LOAM/SLAM consistency.
- `warmup_end_requires_start`: default `true`; require warmup to end near the configured start before the formal goal is published.

Run simulation and evaluation together on `map1`:

```bash
roslaunch putn_launch map_test_nav_eval_with_sim.launch \
  scene:=map1 \
  use_lesta:=true \
  method:=lesta_putn \
  task_id:=map1_case_01 \
  start_x:=0.0 start_y:=0.0 start_z:=0.0 \
  spawn_z_offset:=1.0 \
  goal_x:=6.0 goal_y:=0.0 goal_z:=0.0 \
  runs:=1 \
  lesta_model_path:=/home/whr/Data/test_pt/proposed/epoch_best.pt
```

Run a single case on `map2`:

```bash
roslaunch putn_launch map_test_nav_eval_with_sim.launch \
  scene:=map2 \
  use_lesta:=true \
  method:=lesta_putn \
  task_id:=map2_case_01 \
  start_x:=0.0 start_y:=0.0 start_z:=0.0 \
  goal_x:=6.0 goal_y:=2.0 goal_z:=0.0 \
  runs:=1 \
  lesta_model_path:=/home/whr/Data/test_pt/proposed/epoch_best.pt
```

Run the same start-goal pair as a PUTN baseline by changing only `use_lesta` and `method`:

```bash
roslaunch putn_launch map_test_nav_eval_with_sim.launch \
  scene:=map1 \
  use_lesta:=false \
  method:=putn_baseline \
  task_id:=map1_case_01 \
  start_x:=0.0 start_y:=0.0 start_z:=0.0 \
  goal_x:=6.0 goal_y:=0.0 goal_z:=0.0 \
  runs:=1
```

Recommended A/B comparison procedure:

1. Run the baseline command with `use_lesta:=false method:=putn_baseline`.
2. Run the fusion command with the same `scene`, `task_id`, `start_*`, `goal_*`, and `runs`, but set `use_lesta:=true method:=lesta_putn`.
3. Compare the CSV files in `~/.ros/putn_nav_eval`.
4. Check whether the fusion run changes `planned_path_length`, `path_length`, `mean_fused_risk`, `max_fused_risk`, `success`, and `failure_reason`.

Example fusion run for the same baseline case:

```bash
roslaunch putn_launch map_test_nav_eval_with_sim.launch \
  scene:=map1 \
  use_lesta:=true \
  method:=lesta_putn \
  task_id:=map1_case_01 \
  start_x:=0.0 start_y:=0.0 start_z:=0.0 \
  goal_x:=6.0 goal_y:=0.0 goal_z:=0.0 \
  runs:=1 \
  lesta_model_path:=/home/whr/Data/test_pt/proposed/epoch_best.pt
```

Override the fixed warmup pattern if a map needs a smaller or larger information-gathering motion. For example, this uses a slightly smaller near-square loop:

```bash
roslaunch putn_launch map_test_nav_eval_with_sim.launch \
  scene:=map1 \
  use_lesta:=true \
  method:=lesta_putn \
  task_id:=map1_case_01 \
  start_x:=0.0 start_y:=0.0 start_z:=0.0 \
  goal_x:=6.0 goal_y:=0.0 goal_z:=0.0 \
  warmup_mode:=fixed_motion \
  warmup_cmd_sequence:="0.15,0.00,6.0;0.00,0.45,3.49;0.15,0.00,6.0;0.00,0.45,3.49;0.15,0.00,6.0;0.00,0.45,3.49;0.15,0.00,6.0;0.00,0.45,3.49" \
  lesta_model_path:=/home/whr/Data/test_pt/proposed/epoch_best.pt
```

Tune `warmup_cmd_sequence` as a repeatable closed-loop or near-closed-loop motion for each map. The useful target is: enough map/traversability coverage for PUTN to start planning, while the robot finishes within `start_tolerance_m` of the configured `start_*`.

Run only the evaluator when simulation is already running:

```bash
roslaunch putn_launch map_test_nav_eval.launch \
  scene:=map1 \
  method:=lesta_putn \
  task_id:=map1_case_01 \
  start_x:=0.0 start_y:=0.0 start_z:=0.0 \
  goal_x:=6.0 goal_y:=0.0 goal_z:=0.0 \
  runs:=1 \
  start_mode:=none \
  warmup_mode:=none \
  config_path:=$(rospack find putn_launch)/config/map_test_nav_eval.yaml \
  output_dir:=$HOME/.ros/putn_nav_eval
```

When using the evaluator-only launch, start the simulator separately first. Use `start_mode:=none` only if the robot is already at the configured start; otherwise use `start_mode:=gazebo` when Gazebo services are available.

## Manual Goal Evaluation

When simulation is already running, use `manual_goal_eval.launch` to publish one manually specified navigation goal and record the result. The script monitors `/gazebo/model_states`, so it records the Gazebo true robot pose rather than the SLAM estimate.

Example:

```bash
roslaunch putn_launch manual_goal_eval.launch \
  goal_x:=6.0 \
  goal_y:=0.0 \
  goal_z:=0.0 \
  method:=lesta_putn \
  task_id:=map1_manual_01 \
  timeout_s:=180.0 \
  goal_tolerance_m:=0.6 \
  roll_pitch_fail_deg:=45.0 \
  auto_manage_controller:=true
```

The script publishes `/goal` once at startup. When `auto_manage_controller:=true`, it also requests `/controller/mode_cmd=auto` before monitoring the run, and requests `/controller/mode_cmd=manual` after success, timeout, or tip-over detection. By default, `require_manual_before_exit:=true`, so the script waits until `/controller/mode` reports `manual` before writing the final result and exiting. The controller publishes its current mode on `/controller/mode`.

It then records:

- `time_to_goal`: elapsed wall time if the robot reaches the target.
- `success`: `1` if the robot reaches within `goal_tolerance_m`, otherwise `0`.
- `tipped`: `1` if absolute roll or pitch exceeds `roll_pitch_fail_deg`.
- `max_tilt_deg`: maximum observed roll/pitch tilt.
- `min_goal_distance`: closest distance reached to the target.
- `initial_controller_mode` and `final_controller_mode`: controller mode observed before and after the test.

CSV files are written to `~/.ros/putn_manual_goal_eval` by default. Override with `output_dir:=...`.

Evaluation files:

- Config: `src/putn/src/putn/putn_launch/config/map_test_nav_eval.yaml`
- Script: `src/putn/src/putn/putn_launch/scripts/run_map_test_nav_eval.py`
- Manual goal script: `src/putn/src/putn/putn_launch/scripts/run_manual_goal_eval.py`
- Combined launch: `src/putn/src/putn/putn_launch/launch/map_test_nav_eval_with_sim.launch`
- Evaluator-only launch: `src/putn/src/putn/putn_launch/launch/map_test_nav_eval.launch`
- Manual goal launch: `src/putn/src/putn/putn_launch/launch/manual_goal_eval.launch`

The evaluator publishes goals to `/goal`, monitors robot state from `/gazebo/model_states`, reads the global path from `/global_planning_node/global_path`, and samples fused risk from `/global_planning_node/fused_traversability_cloud`.

CSV results are written to `~/.ros/putn_nav_eval` by default. The result fields include success state, elapsed time, path length, final distance, collision state, mean fused risk, and max fused risk.

## Runtime Check Topics

Check LeSTA prediction:

```bash
rostopic list | grep lesta
rostopic hz /lesta/prediction/traversability_cloud
```

Check LeSTA mapping:

```bash
rostopic hz /lesta/mapping/traversability_grid_map
rostopic hz /lesta/mapping/traversability
rostopic hz /lesta/mapping/traversability_cloud
```

For RViz, prefer the PointCloud2 topic `/lesta/mapping/traversability` instead of directly adding the GridMap display for `/lesta/mapping/traversability_grid_map`. The GridMap display plugin can crash on large or frequently updated maps, while PointCloud2 is usually stable. Use `mapping/probability` as the color channel: higher value means more traversable. `/lesta/mapping/traversability_cloud` is an optional converted view from `grid_map_visualization`; if it has no messages, use `/lesta/mapping/traversability` first.

If adding `/lesta/mapping/traversability_grid_map` in RViz makes RViz crash, keep the launch running with `rviz_required:=false`, restart RViz separately, and reduce the LeSTA mapping window first, for example:

```bash
roslaunch putn_launch fusion_simulation.launch \
  scene:=map1 \
  use_lesta:=true \
  lesta_map_source:=mapping \
  rviz_required:=false \
  lesta_mapping_map_length_x:=30.0 \
  lesta_mapping_map_length_y:=30.0 \
  lesta_model_path:=/home/whr/Data/test_pt/proposed/epoch_best.pt
```

If RViz still crashes immediately, start the simulation without RViz and open RViz separately after the ROS graph is stable:

```bash
roslaunch putn_launch fusion_simulation.launch \
  scene:=map1 \
  use_lesta:=true \
  lesta_map_source:=mapping \
  launch_rviz:=false \
  lesta_model_path:=/home/whr/Data/test_pt/proposed/epoch_best.pt

rviz -d $(rospack find putn_launch)/rviz_config/simulation.rviz
```

Check PUTN fusion output:

```bash
rostopic hz /global_planning_node/fused_traversability_cloud
rostopic echo -n 1 /global_planning_node/fused_traversability_cloud/fields
```

Useful fused cloud fields:

- `fused_risk`: final risk used by the fusion planner.
- `putn_risk`: clipped PUTN geometric risk.
- `putn_raw_risk`: raw PUTN geometric risk before clipping.
- `putn_flatness`, `putn_slope`, `putn_sparsity`, `putn_vacancy_ratio`: PUTN diagnostic terms.
- `lesta_probability`: LeSTA traversability probability.
- `lesta_risk`: `1.0 - lesta_probability`.
- `lesta_observed`: whether LeSTA data was available at the sampled node.
- `lesta_traversable`: binary LeSTA traversability result.

## ROS Graph Smoke Checks

After sourcing the workspace, these commands should resolve packages and expand the fusion launch:

```bash
rospack find putn_launch
rospack find lesta
roslaunch putn_launch fusion_simulation.launch launch_local_control:=false --nodes
```
