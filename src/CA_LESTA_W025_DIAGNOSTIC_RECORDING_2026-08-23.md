# C→A LeSTA-PUTN w=0.25 diagnostic recording

Campaign: `lesta_putn_ca_rellis_w025_diagnostic_v2_180s_5`

Checkpoint: `/home/whr/.ros/putn_nav_eval/map_test_nav_success_rate_lesta_putn_ca_rellis_w025_diagnostic_v2_180s_5_campaign_checkpoint.json`

Bag directory: `/home/whr/Data/rosbags/lesta_putn/failure_replay`

## Completion and storage

- Campaign complete: yes
- Formal eligible samples: 5
- Formal successes: 5 (100%)
- Excluded setup attempts: 2; both were automatically replaced
- Bags: 7, all indexed and LZ4-compressed
- Total stored size: 17.10 GB (15.92 GiB)
- Every bag contains `/velodyne_points`, TF, Gazebo ground truth, LeSTA mapping cloud/grid map, fused traversability cloud, global path, goal, and commands.

The 95% Wilson interval for 5/5 is approximately 56.6%–100%; this diagnostic set alone is too small to establish a population-level success-rate improvement.

## Formal navigation metrics

| Metric | Mean | Median | Min | Max |
|---|---:|---:|---:|---:|
| Navigation time (s) | 132.017 | 134.488 | 88.499 | 160.954 |
| Navigation time excluding recovery (s) | 121.355 | 122.345 | 86.010 | 147.298 |
| Gazebo travel distance (m) | 34.130 | 33.706 | 28.707 | 39.264 |
| Final position error (m) | 0.485 | 0.483 | 0.478 | 0.492 |
| Maximum TF/Gazebo offset (m) | 2.364 | 2.448 | 2.089 | 2.692 |
| Mean fused risk | 0.229 | 0.232 | 0.187 | 0.271 |
| Maximum fused risk | 0.745 | 0.762 | 0.719 | 0.762 |
| Replans | 39.4 | 40 | 27 | 50 |

## Session classification

| Session | Result | Navigation time (s) | Maximum tilt | Use |
|---:|---|---:|---:|---|
| 1 | success | 142.613 | 30.753° | formal sample |
| 2 | success | 88.499 | 29.766° | formal sample; fastest |
| 3 | excluded | — | — | drift stress data only; pre-episode offset 4.80 m |
| 4 | excluded | — | — | drift stress data only; pre-episode offset 4.75 m |
| 5 | success | 160.954 | 30.575° | formal sample; slowest/longest |
| 6 | success | 134.488 | 35.702° | formal sample; priority terrain-risk replay |
| 7 | success | 133.529 | 32.113° | formal sample; post-episode offset reached 2.69 m |

## Priority region

Across all five formal runs, the maximum body-tilt positions cluster at approximately:

- `x = 7.54–7.81 m`
- `y = 0.18–1.26 m`

Session 6 is the first replay target because it contains the largest body tilt (35.702°) at approximately `(7.737, 1.257)`, while still completing navigation successfully. Sessions 3 and 4 must not be included in navigation success/time statistics because no formal navigation episode was started; they remain useful for studying localization drift after full-map warmup.

The row-level bag/result mapping is stored in `CA_LESTA_W025_DIAGNOSTIC_RECORDING_2026-08-23.csv`.

## Priority replay

Session 6 formal navigation starts at bag offset 546.474 s. Its maximum-tilt
ground-truth pose occurs at bag offset 638.108 s. Start at 625 s and play at
quarter speed to inspect the approach, terrain point cloud, LeSTA probability
map, and PUTN fused traversability cloud:

```bash
source /home/whr/putn_noetic/devel/setup.bash

ROS_MASTER_URI=http://127.0.0.1:11312 \
roslaunch putn_launch map1_wide_diagnostic_bag_replay.launch \
  bag:=/home/whr/Data/rosbags/lesta_putn/failure_replay/ca_w025_diagnostic_v2_2026-08-23-17-48-05_0.bag \
  start:=625 \
  rate:=0.25 \
  paused:=true
```

Playback starts paused. Focus the rosbag terminal and press Space to begin or
pause. The RViz configuration displays raw `/velodyne_points`, LeSTA
`mapping/probability`, and the fused cloud with `AxisColor` on the Z axis.
