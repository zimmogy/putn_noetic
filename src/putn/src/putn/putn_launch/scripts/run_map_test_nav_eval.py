#!/usr/bin/env python3
import csv
import hashlib
import json
import math
import os
import statistics
import time
import traceback

import rosgraph
import rospy
import rospkg
import sensor_msgs.point_cloud2 as pc2
import tf
import yaml
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose, PoseStamped, Twist
from std_msgs.msg import Float32MultiArray


def yaw_to_quaternion(yaw):
    q = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
    return q


def quaternion_to_rpy(q):
    return tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])


def body_up_tilt_deg(q):
    """Angle between the robot body Z axis and world Z, independent of Euler singularities."""
    norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
    if norm_sq <= 1e-12:
        return 180.0
    # Rzz for a normalized quaternion is the world-Z component of body Z.
    up_z = 1.0 - 2.0 * (q.x * q.x + q.y * q.y) / norm_sq
    return math.degrees(math.acos(max(-1.0, min(1.0, up_z))))


def dist2d(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def rect_distance(point, center, size):
    dx = max(abs(point[0] - center[0]) - size[0] * 0.5, 0.0)
    dy = max(abs(point[1] - center[1]) - size[1] * 0.5, 0.0)
    return math.hypot(dx, dy)


def is_within_start_tolerance(pos, start, tolerance):
    return dist2d(pos, start) <= tolerance


def path_length(points):
    if len(points) < 2:
        return 0.0
    return sum(dist2d(points[i - 1], points[i]) for i in range(1, len(points)))


def read_point_param(prefix):
    names = ["~{}_x".format(prefix), "~{}_y".format(prefix), "~{}_z".format(prefix)]
    present = [rospy.has_param(name) for name in names]
    if not any(present):
        return None
    if not all(present):
        raise ValueError("Provide all of {}, {}, and {}".format(*names))
    return [float(rospy.get_param(name)) for name in names]


def build_single_task_from_params():
    start = read_point_param("start")
    goal = read_point_param("goal")
    if start is None and goal is None:
        return None
    if start is None or goal is None:
        raise ValueError("Provide both start_x/start_y/start_z and goal_x/goal_y/goal_z")
    return {
        "id": rospy.get_param("~task_id", "single_start_goal"),
        "category": rospy.get_param("~task_category", "single"),
        "start": start,
        "goal": goal,
        "start_label": rospy.get_param("~start_label", "start"),
        "goal_label": rospy.get_param("~goal_label", "goal"),
        "note": rospy.get_param("~task_note", "single launch-configured start/goal task"),
    }


def read_warmup_cmd_sequence_param():
    sequence_text = rospy.get_param("~warmup_cmd_sequence", "")
    if not sequence_text:
        return []

    phases = []
    for raw_phase in sequence_text.split(";"):
        raw_phase = raw_phase.strip()
        if not raw_phase:
            continue
        values = [item.strip() for item in raw_phase.split(",")]
        if len(values) != 3:
            raise ValueError(
                "Each warmup_cmd_sequence phase must be linear,angular,duration; got '{}'".format(raw_phase)
            )
        linear, angular, duration = [float(value) for value in values]
        if duration <= 0.0:
            raise ValueError("Warmup command duration must be positive; got '{}'".format(raw_phase))
        phases.append((linear, angular, duration))
    return phases


def read_float_list_param(name, default):
    value = rospy.get_param(name, default)
    if isinstance(value, (list, tuple)):
        return [float(item) for item in value]
    return [float(item.strip()) for item in str(value).split(",") if item.strip()]


class MapTestNavEval:
    def __init__(self):
        self.config_path = rospy.get_param("~config_path", self.default_config_path())
        self.output_dir = os.path.expanduser(rospy.get_param("~output_dir", "~/.ros/putn_nav_eval"))
        requested_method = str(rospy.get_param("~method", "auto")).strip()
        self.use_lesta = bool(rospy.get_param("~use_lesta", False))
        self.lesta_model_path = os.path.expanduser(
            str(rospy.get_param("~lesta_model_path", "")).strip()
        )
        self.lesta_putn_weight = float(
            rospy.get_param("~lesta_putn_weight", 0.3)
        )
        self.lesta_weight = float(rospy.get_param("~lesta_weight", 0.7))
        self.lesta_unknown_penalty = float(
            rospy.get_param("~lesta_unknown_penalty", 0.0)
        )
        self.lesta_risk_threshold = float(
            rospy.get_param("~lesta_risk_threshold", 0.75)
        )
        self.lesta_fusion_policy = str(
            rospy.get_param("~lesta_fusion_policy", "legacy_weighted_sum")
        ).strip().lower()
        self.lesta_map_source = str(
            rospy.get_param("~lesta_map_source", "prediction")
        ).strip().lower()
        self.require_lesta_ready = bool(
            rospy.get_param("~require_lesta_ready", True)
        )
        self.lesta_readiness_timeout_s = max(
            0.0, float(rospy.get_param("~lesta_readiness_timeout_s", 30.0))
        )
        self.lesta_map_topic = str(
            rospy.get_param(
                "~lesta_map_topic", "/lesta/prediction/traversability"
            )
        )
        self.lesta_probability_layer = str(
            rospy.get_param(
                "~lesta_probability_layer", "traversability/probability"
            )
        )
        self.lesta_binary_layer = str(
            rospy.get_param("~lesta_binary_layer", "traversability/binary")
        )
        self.lesta_model_sha256 = ""
        if self.use_lesta:
            if self.lesta_map_source not in ("prediction", "mapping"):
                raise ValueError("lesta_map_source must be 'prediction' or 'mapping'")
            if self.lesta_fusion_policy not in (
                "legacy_weighted_sum", "risk_floor_weighted", "conservative_max"
            ):
                raise ValueError(
                    "lesta_fusion_policy must be 'legacy_weighted_sum', "
                    "'risk_floor_weighted', or 'conservative_max'"
                )
            if not self.lesta_model_path or not os.path.isfile(self.lesta_model_path):
                raise ValueError(
                    "LESTA model does not exist: '{}'".format(
                        self.lesta_model_path
                    )
                )
            model_hash = hashlib.sha256()
            with open(self.lesta_model_path, "rb") as model_file:
                for chunk in iter(lambda: model_file.read(1024 * 1024), b""):
                    model_hash.update(chunk)
            self.lesta_model_sha256 = model_hash.hexdigest()
            if self.lesta_putn_weight < 0.0 or self.lesta_weight < 0.0:
                raise ValueError("LESTA/PUTN fusion weights must be non-negative")
            if self.lesta_putn_weight + self.lesta_weight <= 0.0:
                raise ValueError("At least one LESTA/PUTN fusion weight must be positive")
            if abs(self.lesta_putn_weight + self.lesta_weight - 1.0) > 1e-6:
                rospy.logwarn(
                    "LESTA/PUTN fusion weights sum to %.3f rather than 1.0",
                    self.lesta_putn_weight + self.lesta_weight,
                )
        configured_method = "lesta_putn" if self.use_lesta else "putn"
        if requested_method.lower() in ("", "auto"):
            self.method = configured_method
        elif requested_method.lower() in ("putn", "lesta_putn"):
            self.method = configured_method
            if requested_method.lower() != configured_method:
                rospy.logwarn(
                    "Ignoring inconsistent method label '%s'; use_lesta=%s means method='%s'",
                    requested_method, self.use_lesta, configured_method,
                )
        else:
            self.method = requested_method
        self.scene = rospy.get_param("~scene", "")
        self.warmup_coordinate_scale = float(rospy.get_param("~warmup_coordinate_scale", 1.0))
        self.robot_model_name = rospy.get_param("~robot_model_name", "scout/")
        self.goal_topic = rospy.get_param("~goal_topic", "/goal")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.require_exclusive_cmd_vel = rospy.get_param(
            "~require_exclusive_cmd_vel", True
        )
        self.model_states_topic = rospy.get_param("~model_states_topic", "/gazebo/model_states")
        self.global_path_topic = rospy.get_param("~global_path_topic", "/global_planning_node/global_path")
        self.fused_cloud_topic = rospy.get_param(
            "~fused_cloud_topic", "/global_planning_node/fused_traversability_cloud"
        )
        self.start_mode = rospy.get_param("~start_mode", "navigate")
        self.reposition_timeout_s = rospy.get_param("~reposition_timeout_s", 180.0)
        self.start_tolerance_m = rospy.get_param("~start_tolerance_m", 0.6)
        self.reset_settle_s = rospy.get_param("~reset_settle_s", 3.0)
        self.goal_publish_count = max(1, int(rospy.get_param("~goal_publish_count", 5)))
        self.pre_goal_warmup_s = rospy.get_param("~pre_goal_warmup_s", 25.0)
        self.hold_goal_clear_s = rospy.get_param("~hold_goal_clear_s", 3.0)
        self.warmup_mode = rospy.get_param("~warmup_mode", "stationary")
        self.warmup_cmd_sequence = read_warmup_cmd_sequence_param()
        self.warmup_reset_to_start = rospy.get_param("~warmup_reset_to_start", False)
        self.warmup_end_requires_start = rospy.get_param("~warmup_end_requires_start", True)
        self.warmup_once = rospy.get_param("~warmup_once", True)
        self.warmup_bootstrap_enabled = rospy.get_param("~warmup_bootstrap_enabled", True)
        self.warmup_bootstrap_min_displacement_m = float(
            rospy.get_param("~warmup_bootstrap_min_displacement_m", 0.08)
        )
        self.projection_recovery_distances_m = read_float_list_param(
            "~projection_recovery_distances_m", "0.35,-0.55,0.75,-0.95"
        )
        self.projection_recovery_speed_mps = abs(float(
            rospy.get_param("~projection_recovery_speed_mps", 0.12)
        ))
        self.projection_recovery_map_settle_s = float(
            rospy.get_param("~projection_recovery_map_settle_s", 3.0)
        )
        self.projection_recovery_scan_angle_deg = abs(float(
            rospy.get_param("~projection_recovery_scan_angle_deg", 35.0)
        ))
        self.bidirectional = rospy.get_param("~bidirectional", True)
        self.session_direction_order_policy = str(
            rospy.get_param("~session_direction_order_policy", "balanced")
        ).strip().lower()
        # "fixed" is a natural launch-file spelling for preserving the
        # configured task order.  Treat it as an alias instead of terminating
        # the required evaluator node during startup.
        if self.session_direction_order_policy == "fixed":
            self.session_direction_order_policy = "configured"
        if self.session_direction_order_policy not in (
            "configured", "reverse_first", "balanced"
        ):
            raise ValueError(
                "session_direction_order_policy must be 'configured', "
                "'reverse_first', or 'balanced'"
            )
        # Zero preserves the endurance-test behavior.  A value of one creates
        # an algorithm-isolation campaign: one warmed simulator session
        # contributes at most one attempt before it checkpoints and exits.
        self.max_episode_attempts_per_session = max(
            0,
            int(rospy.get_param("~max_episode_attempts_per_session", 0)),
        )
        self.rrt_settle_s = float(rospy.get_param("~rrt_settle_s", 8.0))
        self.warmup_waypoint_spacing_m = float(rospy.get_param("~warmup_waypoint_spacing_m", 2.0))
        self.warmup_waypoint_tolerance_m = float(rospy.get_param("~warmup_waypoint_tolerance_m", 0.8))
        self.warmup_waypoint_timeout_s = float(rospy.get_param("~warmup_waypoint_timeout_s", 60.0))
        self.warmup_waypoint_retries = max(
            0, int(rospy.get_param("~warmup_waypoint_retries", 1))
        )
        self.warmup_no_progress_timeout_s = float(
            rospy.get_param("~warmup_no_progress_timeout_s", 12.0)
        )
        self.warmup_min_progress_m = float(rospy.get_param("~warmup_min_progress_m", 0.15))
        self.warmup_path_stale_s = float(rospy.get_param("~warmup_path_stale_s", 3.0))
        self.warmup_total_timeout_s = float(rospy.get_param("~warmup_total_timeout_s", 720.0))
        self.warmup_target_coverage_ratio = float(
            rospy.get_param("~warmup_target_coverage_ratio", 0.90)
        )
        self.warmup_safety_margin_m = float(rospy.get_param("~warmup_safety_margin_m", 0.8))
        self.path_grace_s = rospy.get_param("~path_grace_s", 20.0)
        self.enable_heading_recovery = rospy.get_param("~enable_heading_recovery", True)
        self.heading_recovery_trigger_s = float(
            rospy.get_param("~heading_recovery_trigger_s", self.path_grace_s)
        )
        self.heading_recovery_angles_deg = read_float_list_param(
            "~heading_recovery_angles_deg", "35,-70,105,-140"
        )
        self.heading_recovery_speed = abs(float(rospy.get_param("~heading_recovery_speed", 2.0)))
        self.heading_recovery_tolerance_deg = float(
            rospy.get_param("~heading_recovery_tolerance_deg", 3.0)
        )
        self.heading_recovery_rotation_timeout_s = float(
            rospy.get_param("~heading_recovery_rotation_timeout_s", 12.0)
        )
        self.heading_recovery_settle_s = float(rospy.get_param("~heading_recovery_settle_s", 2.0))
        self.risk_sample_radius_m = rospy.get_param("~risk_sample_radius_m", 0.75)
        self.enable_cmd_follower = rospy.get_param("~enable_cmd_follower", True)
        self.lookahead_points = int(rospy.get_param("~lookahead_points", 8))
        self.lookahead_distance_m = float(rospy.get_param("~lookahead_distance_m", 0.70))
        self.path_switch_min_interval_s = float(
            rospy.get_param("~path_switch_min_interval_s", 1.5)
        )
        self.path_max_hold_s = float(rospy.get_param("~path_max_hold_s", 3.0))
        self.path_switch_max_heading_deg = float(
            rospy.get_param("~path_switch_max_heading_deg", 55.0)
        )
        self.active_path_end_distance_m = float(
            rospy.get_param("~active_path_end_distance_m", 0.60)
        )
        self.active_path_max_cross_track_m = float(
            rospy.get_param("~active_path_max_cross_track_m", 0.80)
        )
        self.path_origin_max_error_m = float(
            rospy.get_param("~path_origin_max_error_m", 2.0)
        )
        self.control_pose_source = str(
            rospy.get_param("~control_pose_source", "tf")
        ).strip().lower()
        self.localization_drift_fail_m = float(
            rospy.get_param("~localization_drift_fail_m", 1.0)
        )
        self.localization_drift_hold_s = float(
            rospy.get_param("~localization_drift_hold_s", 2.0)
        )
        self.abort_on_start_setup_failure = bool(
            rospy.get_param("~abort_on_start_setup_failure", False)
        )
        self.max_excluded_retries_per_direction = max(
            0,
            int(rospy.get_param("~max_excluded_retries_per_direction", 10)),
        )
        self.resume_incomplete_campaign = bool(
            rospy.get_param("~resume_incomplete_campaign", True)
        )
        self.requested_campaign_id = str(
            rospy.get_param("~campaign_id", "")
        ).strip()
        self.campaign_seed_csv = os.path.expanduser(
            str(rospy.get_param("~campaign_seed_csv", "")).strip()
        )
        self.compensate_goals_with_ground_truth = bool(
            rospy.get_param("~compensate_goals_with_ground_truth", False)
        )
        self.abort_on_localization_drift = bool(
            rospy.get_param("~abort_on_localization_drift", True)
        )
        # A small TF/Gazebo offset can be tolerated or compensated during an
        # algorithm-isolation experiment.  A large persistent offset means the
        # whole localization session is no longer physically meaningful: keep
        # the current attempt as an end-to-end failure, checkpoint it, and ask
        # for a clean simulator restart instead of driving until timeout.
        self.severe_localization_drift_fail_m = max(
            0.0,
            float(rospy.get_param("~severe_localization_drift_fail_m", 4.0)),
        )
        self.severe_localization_drift_hold_s = max(
            0.0,
            float(rospy.get_param("~severe_localization_drift_hold_s", 3.0)),
        )
        self.abort_session_on_severe_localization_drift = bool(
            rospy.get_param(
                "~abort_session_on_severe_localization_drift", True
            )
        )
        self.tilt_fail_hold_s = max(
            0.0, float(rospy.get_param("~tilt_fail_hold_s", 0.8))
        )
        self.severe_tilt_fail_deg = float(
            rospy.get_param("~severe_tilt_fail_deg", 50.0)
        )
        self.goal_compensation_refresh_distance_m = max(
            0.0,
            float(rospy.get_param("~goal_compensation_refresh_distance_m", 0.60)),
        )
        self.goal_compensation_refresh_yaw_deg = max(
            0.0,
            float(rospy.get_param("~goal_compensation_refresh_yaw_deg", 3.0)),
        )
        self.goal_compensation_refresh_min_interval_s = max(
            0.0,
            float(rospy.get_param("~goal_compensation_refresh_min_interval_s", 8.0)),
        )
        self.goal_compensation_max_refreshes = max(
            0, int(rospy.get_param("~goal_compensation_max_refreshes", 12))
        )
        self.goal_compensation_max_forced_refreshes = max(
            0,
            int(rospy.get_param("~goal_compensation_max_forced_refreshes", 3)),
        )
        self.goal_compensation_forced_min_progress_m = max(
            0.01,
            float(
                rospy.get_param(
                    "~goal_compensation_forced_min_progress_m", 0.15
                )
            ),
        )
        self.control_goal_tolerance_m = max(
            0.05, float(rospy.get_param("~control_goal_tolerance_m", 0.30))
        )
        self.control_goal_mismatch_physical_error_m = max(
            self.control_goal_tolerance_m,
            float(
                rospy.get_param(
                    "~control_goal_mismatch_physical_error_m", 1.0
                )
            ),
        )
        self.control_goal_mismatch_hold_s = max(
            0.0, float(rospy.get_param("~control_goal_mismatch_hold_s", 2.0))
        )
        self.physical_stuck_displacement_m = max(
            0.01, float(rospy.get_param("~physical_stuck_displacement_m", 0.12))
        )
        self.physical_stuck_hold_s = max(
            1.0, float(rospy.get_param("~physical_stuck_hold_s", 12.0))
        )
        self.physical_stuck_max_recoveries = max(
            0, int(rospy.get_param("~physical_stuck_max_recoveries", 2))
        )
        # Session health is evaluated only between formal episodes. An unhealthy
        # mapping/localization session is excluded from the navigation success
        # rate, checkpointed, and resumed after a full simulator restart.
        self.pre_episode_health_timeout_s = max(
            0.0, float(rospy.get_param("~pre_episode_health_timeout_s", 8.0))
        )
        self.pre_episode_health_hold_s = max(
            0.0, float(rospy.get_param("~pre_episode_health_hold_s", 1.0))
        )
        self.pre_episode_max_tilt_deg = max(
            0.0, float(rospy.get_param("~pre_episode_max_tilt_deg", 25.0))
        )
        self.pre_episode_max_tf_gazebo_offset_m = max(
            0.0,
            float(
                rospy.get_param(
                    "~pre_episode_max_tf_gazebo_offset_m", 4.0
                )
            ),
        )
        self.pre_episode_max_sensor_age_s = max(
            0.1, float(rospy.get_param("~pre_episode_max_sensor_age_s", 2.0))
        )
        self.post_episode_health_check = bool(
            rospy.get_param("~post_episode_health_check", True)
        )
        self.lesta_map_max_age_s = max(
            0.1, float(rospy.get_param("~lesta_map_max_age_s", 5.0))
        )
        self.max_consecutive_planning_failures_before_restart = max(
            0,
            int(
                rospy.get_param(
                    "~max_consecutive_planning_failures_before_restart", 2
                )
            ),
        )
        self.runtime_metric_sample_period_s = max(
            0.02, float(rospy.get_param("~runtime_metric_sample_period_s", 0.10))
        )
        self.runtime_metric_max_step_m = max(
            0.1, float(rospy.get_param("~runtime_metric_max_step_m", 1.0))
        )
        if self.compensate_goals_with_ground_truth and self.abort_on_localization_drift:
            rospy.logwarn(
                "Goal-frame compensation and localization-drift abort are both enabled; "
                "the abort policy still takes precedence"
            )
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.45)
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 2.0)
        self.heading_gain = rospy.get_param("~heading_gain", 1.4)
        self.goal_slowdown_radius_m = rospy.get_param("~goal_slowdown_radius_m", 1.5)
        # The Scout can rotate in place, so forward-only tracking is the safe
        # default. Selecting reverse near a 90 degree heading error caused the
        # old controller to alternate the sign of both linear and angular
        # commands without making net progress.
        self.allow_reverse_following = rospy.get_param("~allow_reverse_following", False)
        self.reverse_heading_threshold_deg = float(
            rospy.get_param("~reverse_heading_threshold_deg", 100.0)
        )
        self.reverse_exit_threshold_deg = float(
            rospy.get_param("~reverse_exit_threshold_deg", 85.0)
        )
        self.min_follow_linear_speed = abs(float(
            rospy.get_param("~min_follow_linear_speed", 0.10)
        ))
        self.min_follow_angular_speed = abs(float(
            rospy.get_param("~min_follow_angular_speed", 0.20)
        ))
        self.min_turn_in_place_angular_speed = abs(float(
            rospy.get_param("~min_turn_in_place_angular_speed", 2.0)
        ))
        self.rotate_in_place_threshold_deg = float(
            rospy.get_param("~rotate_in_place_threshold_deg", 55.0)
        )
        self.rotate_in_place_exit_threshold_deg = float(
            rospy.get_param("~rotate_in_place_exit_threshold_deg", 20.0)
        )
        self.max_moving_angular_speed = abs(float(
            rospy.get_param("~max_moving_angular_speed", 0.55)
        ))
        self.initial_delay_s = rospy.get_param("~initial_delay_s", 0.0)

        if not (
            0.0 <= self.rotate_in_place_exit_threshold_deg
            < self.rotate_in_place_threshold_deg
            < 180.0
        ):
            raise ValueError(
                "Require 0 <= rotate_in_place_exit_threshold_deg < "
                "rotate_in_place_threshold_deg < 180"
            )

        with open(self.config_path, "r") as f:
            self.config = yaml.safe_load(f)

        self.eval_cfg = self.config["evaluation"]
        if not self.scene:
            self.scene = self.eval_cfg.get("scene", "")
        self.fixed_frame = str(self.eval_cfg.get("fixed_frame", "world")).lstrip("/")
        self.robot_frame = str(self.eval_cfg.get("robot_frame", "base_link")).lstrip("/")
        if self.control_pose_source not in ("tf", "gazebo"):
            raise ValueError("control_pose_source must be 'tf' or 'gazebo'")
        # Copy before applying launch-time overrides so the loaded YAML remains
        # an immutable description of the source configuration.
        self.criteria = dict(self.eval_cfg["success_criteria"])
        self.navigation_timeout_s = max(
            1.0,
            float(
                rospy.get_param(
                    "~navigation_timeout_s", self.criteria.get("timeout_s", 180.0)
                )
            ),
        )
        self.criteria["timeout_s"] = self.navigation_timeout_s
        if self.severe_tilt_fail_deg <= float(
            self.criteria["roll_pitch_fail_deg"]
        ):
            raise ValueError(
                "severe_tilt_fail_deg must be greater than roll_pitch_fail_deg"
            )
        single_task = build_single_task_from_params()
        self.tasks = [single_task] if single_task is not None else self.config["tasks"]
        map_cfg = self.config.get("map", {})
        warmup_cfg = self.config.get("warmup", {})
        self.pits = map_cfg.get("pits")
        self.columns = map_cfg.get("columns")
        if self.pits is None:
            self.pits = warmup_cfg.get("pits", [])
            if self.pits:
                rospy.logwarn("Deprecated YAML layout: move warmup.pits to map.pits")
        if self.columns is None:
            self.columns = warmup_cfg.get("columns", [])
            if self.columns:
                rospy.logwarn("Deprecated YAML layout: move warmup.columns to map.columns")
        hazard_scene = str(map_cfg.get("hazard_scene", self.eval_cfg.get("scene", "")))
        if (self.pits or self.columns) and hazard_scene and hazard_scene != self.scene:
            rospy.logwarn(
                "Disabling YAML pit/column checks: hazards describe scene '%s', "
                "but the launched scene is '%s'",
                hazard_scene, self.scene,
            )
            self.pits = []
            self.columns = []
        self.output_fields = list(self.eval_cfg["output_fields"])
        for field in (
            "direction",
            "attempt_id",
            "navigation_time_excluding_recovery",
            "elapsed_s",
            "planning_latency_s",
            "final_position_error_m",
            "heading_recovery_count",
            "heading_recovery_time",
            "start_x",
            "start_y",
            "goal_x",
            "goal_y",
            "valid_path_update_count",
            "accepted_path_count",
            "online_route_length_estimate_m",
            "failure_stage",
            "eligible_for_success_rate",
            "max_abs_roll_deg",
            "max_abs_pitch_deg",
            "max_tilt_deg",
            "max_body_up_tilt_deg",
            "max_tilt_violation_duration_s",
            "final_ground_truth_x_m",
            "final_ground_truth_y_m",
            "final_ground_truth_z_m",
            "failure_ground_truth_x_m",
            "failure_ground_truth_y_m",
            "failure_ground_truth_z_m",
            "failure_ground_truth_yaw_deg",
            "failure_roll_deg_signed",
            "failure_pitch_deg_signed",
            "failure_body_up_tilt_deg",
            "failure_episode_distance_m",
            "max_tilt_ground_truth_x_m",
            "max_tilt_ground_truth_y_m",
            "max_tilt_ground_truth_z_m",
            "max_tilt_roll_deg_signed",
            "max_tilt_pitch_deg_signed",
            "max_tilt_body_up_tilt_deg_at_pose",
            "max_tilt_episode_distance_m",
            "max_tilt_elapsed_s",
            "tf_gazebo_offset_start_m",
            "tf_gazebo_offset_max_m",
            "tf_gazebo_offset_final_m",
            "tf_gazebo_dx_start_m",
            "tf_gazebo_dy_start_m",
            "tf_gazebo_dyaw_start_deg",
            "tf_gazebo_dx_final_m",
            "tf_gazebo_dy_final_m",
            "tf_gazebo_dyaw_final_deg",
            "path_origin_rejection_count",
            "goal_compensation_refresh_count",
            "goal_compensation_proactive_refresh_count",
            "goal_compensation_forced_refresh_count",
            "goal_compensation_refresh_time",
            "control_goal_mismatch_count",
            "campaign_session_id",
            "session_attempt_id",
            "session_elapsed_start_s",
            "session_elapsed_end_s",
            "campaign_active_elapsed_start_s",
            "campaign_active_elapsed_end_s",
            "episode_ground_truth_distance_m",
            "episode_elevation_gain_m",
            "episode_elevation_loss_m",
            "episode_vertical_travel_m",
            "session_ground_truth_distance_start_m",
            "session_ground_truth_distance_end_m",
            "campaign_ground_truth_distance_start_m",
            "campaign_ground_truth_distance_end_m",
            "session_elevation_gain_start_m",
            "session_elevation_gain_end_m",
            "campaign_elevation_gain_start_m",
            "campaign_elevation_gain_end_m",
            "tf_gazebo_offset_time_mean_m",
            "tf_gazebo_offset_time_rms_m",
            "tf_gazebo_offset_above_threshold_s",
        ):
            if field not in self.output_fields:
                self.output_fields.append(field)

        # self.pose is deliberately the same /world -> /base_link TF pose used
        # by PUTN. Gazebo pose is retained separately as evaluation ground truth.
        self.pose = None
        self.ground_truth_pose = None
        self.last_pose_time = None
        self.last_ground_truth_pose_time = None
        self.current_speed = 0.0
        self.tf_gazebo_offset_m = None
        self.localization_drift_started_wall = None
        self.localization_failure_detected = False
        self.last_published_goal = None
        self.last_requested_physical_goal = None
        self.last_goal_publish_wall = None
        self.last_goal_frame_yaw_deg = None
        self.goal_compensation_refresh_count = 0
        self.goal_compensation_proactive_refresh_count = 0
        self.goal_compensation_forced_refresh_count = 0
        self.control_goal_mismatch_count = 0
        self.last_start_setup_failure_reason = ""
        self.safety_stop_latched = False
        self.safety_stop_reason = ""
        self.safety_stop_wall = None
        self.safety_stop_roll_deg = 0.0
        self.safety_stop_pitch_deg = 0.0
        self.safety_stop_body_up_tilt_deg = 0.0
        self.safety_stop_sample = {}
        self.session_restart_requested = False
        self.session_restart_reason = ""
        self.consecutive_planning_failures = 0
        self.session_started_wall = time.time()
        self.session_attempt_count = 0
        self.session_ground_truth_distance_m = 0.0
        self.session_elevation_gain_m = 0.0
        self.session_elevation_loss_m = 0.0
        self.session_vertical_travel_m = 0.0
        self.session_drift_integral_m_s = 0.0
        self.session_drift_squared_integral_m2_s = 0.0
        self.session_drift_observed_s = 0.0
        self.session_drift_above_threshold_s = 0.0
        self.last_runtime_metric_wall = None
        self.last_runtime_metric_position = None
        self.last_ground_truth_wall = None
        self.last_control_pose_wall = None
        self.formal_episode_metric_start = None
        self.campaign_metric_base = {
            "active_elapsed_s": 0.0,
            "ground_truth_distance_m": 0.0,
            "elevation_gain_m": 0.0,
            "elevation_loss_m": 0.0,
            "vertical_travel_m": 0.0,
        }
        self.sampling_report = {
            "policy": "eligible_quota_with_excluded_record_refill",
            "session_direction_order_policy": self.session_direction_order_policy,
            "session_first_direction_group": "",
            "max_episode_attempts_per_session": (
                self.max_episode_attempts_per_session
            ),
            "target_valid_samples_per_direction": 0,
            "max_excluded_retries_per_direction": (
                self.max_excluded_retries_per_direction
            ),
            "directions": [],
        }
        self.latest_path_points = []
        self.active_path_points = []
        self.active_path_wall = None
        self.active_path_source_wall = None
        self.path_messages = 0
        self.valid_path_messages = 0
        self.last_valid_path_wall = None
        self.last_empty_path_wall = None
        self.first_valid_path_wall = None
        self.first_goal_path_length = None
        self.accepted_path_count = 0
        self.path_origin_rejection_count = 0
        self.episode_start_wall = None
        self.episode_goal = None
        self.episode_distance_m = 0.0
        self.latest_cloud = []
        self.lesta_map_received = False
        self.lesta_map_ready = False
        self.lesta_ready_layer = ""
        self.lesta_map_layers = []
        self.last_lesta_map_wall = None
        self.follow_drive_direction = 1.0
        self.follow_turning_in_place = False

        self.tf_listener = tf.TransformListener()
        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1, latch=True)
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        rospy.Subscriber(self.model_states_topic, ModelStates, self.model_states_cb, queue_size=1)
        rospy.Subscriber(self.global_path_topic, Float32MultiArray, self.global_path_cb, queue_size=10)
        rospy.Subscriber(self.fused_cloud_topic, rospy.AnyMsg, self.fused_cloud_any_cb, queue_size=1)
        if self.use_lesta:
            rospy.Subscriber(
                self.lesta_map_topic,
                rospy.AnyMsg,
                self.lesta_map_any_cb,
                queue_size=1,
            )

        self.set_model_state = None
        if self.start_mode == "gazebo":
            rospy.logwarn(
                "start_mode:=gazebo teleports the model and can desynchronize A-LOAM. "
                "Use start_mode:=navigate for evaluation."
            )
            rospy.wait_for_service("/gazebo/set_model_state")
            self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    @staticmethod
    def default_config_path():
        pkg_path = rospkg.RosPack().get_path("putn_launch")
        return os.path.join(pkg_path, "config", "map_test_nav_eval.yaml")

    def latch_severe_tilt(self, pose, context):
        roll, pitch, yaw = quaternion_to_rpy(pose.orientation)
        roll_signed_deg = math.degrees(roll)
        pitch_signed_deg = math.degrees(pitch)
        roll_deg = abs(math.degrees(roll))
        pitch_deg = abs(math.degrees(pitch))
        body_tilt_deg = body_up_tilt_deg(pose.orientation)
        if body_tilt_deg < self.severe_tilt_fail_deg:
            return False
        if not self.safety_stop_latched:
            self.safety_stop_latched = True
            self.safety_stop_reason = "robot_overturned"
            self.safety_stop_wall = time.time()
            self.safety_stop_roll_deg = roll_deg
            self.safety_stop_pitch_deg = pitch_deg
            self.safety_stop_body_up_tilt_deg = body_tilt_deg
            self.safety_stop_sample = {
                "x_m": pose.position.x,
                "y_m": pose.position.y,
                "z_m": pose.position.z,
                "roll_deg_signed": roll_signed_deg,
                "pitch_deg_signed": pitch_signed_deg,
                "yaw_deg": math.degrees(yaw),
                "body_up_tilt_deg": body_tilt_deg,
            }
            rospy.logfatal(
                "Irrecoverable safety stop in %s: body-up tilt %.1f deg "
                "(roll=%.1f pitch=%.1f) reached %.1f deg; current data will "
                "be saved and the test batch will exit for simulator restart",
                context,
                body_tilt_deg,
                roll_deg,
                pitch_deg,
                self.severe_tilt_fail_deg,
            )
        self.cmd_pub.publish(Twist())
        return True

    def model_states_cb(self, msg):
        names_to_try = [self.robot_model_name, self.robot_model_name.rstrip("/"), "scout", "scout/"]
        index = None
        for name in names_to_try:
            if name in msg.name:
                index = msg.name.index(name)
                break
        if index is None:
            return

        pose = msg.pose[index]
        now = rospy.Time.now()
        if self.ground_truth_pose is not None and self.last_ground_truth_pose_time is not None:
            pose_step = dist2d(
                (pose.position.x, pose.position.y),
                (self.ground_truth_pose.position.x, self.ground_truth_pose.position.y),
            )
            dt = max((now - self.last_ground_truth_pose_time).to_sec(), 1e-3)
            self.current_speed = pose_step / dt
            if self.episode_start_wall is not None:
                self.episode_distance_m += pose_step
        self.ground_truth_pose = pose
        self.last_ground_truth_pose_time = now
        self.last_ground_truth_wall = time.time()
        self.latch_severe_tilt(pose, "Gazebo ground-truth watchdog")
        self.update_control_pose(now)
        self.update_runtime_metrics()

    def update_control_pose(self, now=None):
        """Update the controller pose in the same frame used by PF-RRT*."""
        if now is None:
            now = rospy.Time.now()
        if self.control_pose_source == "gazebo":
            control_pose = self.ground_truth_pose
        else:
            try:
                translation, rotation = self.tf_listener.lookupTransform(
                    self.fixed_frame, self.robot_frame, rospy.Time(0)
                )
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                return False
            control_pose = Pose()
            control_pose.position.x = translation[0]
            control_pose.position.y = translation[1]
            control_pose.position.z = translation[2]
            control_pose.orientation.x = rotation[0]
            control_pose.orientation.y = rotation[1]
            control_pose.orientation.z = rotation[2]
            control_pose.orientation.w = rotation[3]

        if control_pose is None:
            return False
        self.pose = control_pose
        self.last_pose_time = now
        self.last_control_pose_wall = time.time()
        if self.ground_truth_pose is not None:
            self.tf_gazebo_offset_m = dist2d(
                (self.pose.position.x, self.pose.position.y),
                (self.ground_truth_pose.position.x, self.ground_truth_pose.position.y),
            )
            if self.tf_gazebo_offset_m > self.localization_drift_fail_m:
                if self.localization_drift_started_wall is None:
                    self.localization_drift_started_wall = time.time()
            else:
                self.localization_drift_started_wall = None
        return True

    def update_runtime_metrics(self):
        """Accumulate session exposure using throttled Gazebo ground truth.

        Sampling at a fixed wall-time interval avoids counting high-rate Gazebo
        pose jitter as travelled distance. Implausible jumps (for example a
        teleport in a diagnostic launch) are excluded from distance/elevation.
        """
        if self.ground_truth_pose is None:
            return
        now = time.time()
        position = (
            self.ground_truth_pose.position.x,
            self.ground_truth_pose.position.y,
            self.ground_truth_pose.position.z,
        )
        if self.last_runtime_metric_wall is None:
            self.last_runtime_metric_wall = now
            self.last_runtime_metric_position = position
            return
        raw_dt = now - self.last_runtime_metric_wall
        if raw_dt < self.runtime_metric_sample_period_s:
            return
        # Do not assign a long debugger/pause gap to one stale drift sample.
        drift_dt = min(raw_dt, 1.0)
        if self.tf_gazebo_offset_m is not None:
            offset = max(float(self.tf_gazebo_offset_m), 0.0)
            self.session_drift_integral_m_s += offset * drift_dt
            self.session_drift_squared_integral_m2_s += offset * offset * drift_dt
            self.session_drift_observed_s += drift_dt
            if offset > self.localization_drift_fail_m:
                self.session_drift_above_threshold_s += drift_dt

        previous = self.last_runtime_metric_position
        planar_step = dist2d(position, previous)
        vertical_step = position[2] - previous[2]
        if math.hypot(planar_step, vertical_step) <= self.runtime_metric_max_step_m:
            self.session_ground_truth_distance_m += planar_step
            if vertical_step >= 0.0:
                self.session_elevation_gain_m += vertical_step
            else:
                self.session_elevation_loss_m += -vertical_step
            self.session_vertical_travel_m += abs(vertical_step)
        else:
            rospy.logwarn_throttle(
                10.0,
                "Ignoring implausible %.2f m ground-truth metric step",
                math.hypot(planar_step, vertical_step),
            )
        self.last_runtime_metric_wall = now
        self.last_runtime_metric_position = position

    def runtime_metric_snapshot(self):
        return {
            "active_elapsed_s": max(time.time() - self.session_started_wall, 0.0),
            "ground_truth_distance_m": self.session_ground_truth_distance_m,
            "elevation_gain_m": self.session_elevation_gain_m,
            "elevation_loss_m": self.session_elevation_loss_m,
            "vertical_travel_m": self.session_vertical_travel_m,
            "drift_integral_m_s": self.session_drift_integral_m_s,
            "drift_squared_integral_m2_s": self.session_drift_squared_integral_m2_s,
            "drift_observed_s": self.session_drift_observed_s,
            "drift_above_threshold_s": self.session_drift_above_threshold_s,
        }

    def campaign_metric_snapshot(self, session_snapshot=None):
        current = session_snapshot or self.runtime_metric_snapshot()
        return {
            key: float(self.campaign_metric_base.get(key, 0.0))
            + float(current.get(key, 0.0))
            for key in self.campaign_metric_base
        }

    def request_session_restart(self, reason, health_related=True):
        if not self.session_restart_requested:
            self.session_restart_requested = True
            self.session_restart_reason = reason
            if health_related:
                rospy.logerr(
                    "Session health requires a full simulator restart: %s. "
                    "The campaign checkpoint will retain completed/failed samples.",
                    reason,
                )
            else:
                rospy.loginfo(
                    "Session isolation policy requests a clean simulator restart: %s. "
                    "The campaign checkpoint will retain the completed/failed sample.",
                    reason,
                )
        self.cmd_pub.publish(Twist())

    def check_session_health(self, boundary_label):
        """Require a stable, fresh and physically upright episode boundary."""
        deadline = time.time() + self.pre_episode_health_timeout_s
        stable_since = None
        last_reason = "health samples unavailable"
        while not rospy.is_shutdown() and time.time() <= deadline:
            self.update_control_pose()
            now = time.time()
            reasons = []
            if self.safety_stop_latched:
                self.cmd_pub.publish(Twist())
                return False
            if (
                self.last_ground_truth_wall is None
                or now - self.last_ground_truth_wall > self.pre_episode_max_sensor_age_s
            ):
                reasons.append("Gazebo pose is stale")
            if (
                self.last_control_pose_wall is None
                or now - self.last_control_pose_wall > self.pre_episode_max_sensor_age_s
            ):
                reasons.append("control TF is stale")
            if self.ground_truth_pose is not None:
                tilt = body_up_tilt_deg(self.ground_truth_pose.orientation)
                if tilt > self.pre_episode_max_tilt_deg:
                    reasons.append("body tilt {:.1f} deg".format(tilt))
            if (
                self.pre_episode_max_tf_gazebo_offset_m > 0.0
                and self.tf_gazebo_offset_m is not None
                and self.tf_gazebo_offset_m
                > self.pre_episode_max_tf_gazebo_offset_m
            ):
                reasons.append(
                    "TF/Gazebo offset {:.2f} m".format(self.tf_gazebo_offset_m)
                )
            if self.use_lesta and self.require_lesta_ready:
                if (
                    self.last_lesta_map_wall is None
                    or now - self.last_lesta_map_wall > self.lesta_map_max_age_s
                ):
                    reasons.append("LESTA map is stale")

            if not reasons:
                if stable_since is None:
                    stable_since = now
                if now - stable_since >= self.pre_episode_health_hold_s:
                    return True
            else:
                stable_since = None
                last_reason = ", ".join(reasons)
            self.cmd_pub.publish(Twist())
            # A stale /clock is itself a health problem, so use wall sleep here.
            rospy.rostime.wallsleep(0.1)
        self.request_session_restart(
            "{} health check failed: {}".format(boundary_label, last_reason)
        )
        return False

    def check_pre_episode_health(self):
        return self.check_session_health("pre-episode")

    def check_post_episode_health(self, row):
        """Keep the formal result, then stop a degraded simulator session.

        A navigation timeout/success is still an eligible end-to-end sample.
        The guard only prevents the next long repositioning move from starting
        with localization damage accumulated during that sample.
        """
        if not self.post_episode_health_check or self.safety_stop_latched:
            return not self.safety_stop_latched

        try:
            episode_max_offset = float(row.get("tf_gazebo_offset_max_m", ""))
        except (TypeError, ValueError):
            episode_max_offset = None
        if (
            self.pre_episode_max_tf_gazebo_offset_m > 0.0
            and episode_max_offset is not None
            and episode_max_offset > self.pre_episode_max_tf_gazebo_offset_m
        ):
            self.request_session_restart(
                "post-episode TF/Gazebo offset reached {:.2f} m (limit {:.2f} m)".format(
                    episode_max_offset,
                    self.pre_episode_max_tf_gazebo_offset_m,
                )
            )
            return False
        return self.check_session_health("post-episode")

    def localization_drift_exceeded(self):
        if (
            self.control_pose_source != "tf"
            or self.tf_gazebo_offset_m is None
            or self.tf_gazebo_offset_m <= self.localization_drift_fail_m
            or self.localization_drift_started_wall is None
        ):
            return False
        exceeded = (
            time.time() - self.localization_drift_started_wall
            >= self.localization_drift_hold_s
        )
        if exceeded and self.abort_on_localization_drift:
            self.localization_failure_detected = True
            return True
        if exceeded:
            rospy.logwarn_throttle(
                10.0,
                "TF/Gazebo offset %.2f m exceeds %.2f m; continuing because "
                "abort_on_localization_drift is false",
                self.tf_gazebo_offset_m, self.localization_drift_fail_m,
            )
        return False

    def localization_drift_sample(self):
        if self.pose is None or self.ground_truth_pose is None:
            return {}
        _, _, estimated_yaw = quaternion_to_rpy(self.pose.orientation)
        _, _, ground_truth_yaw = quaternion_to_rpy(self.ground_truth_pose.orientation)
        yaw_error = math.atan2(
            math.sin(estimated_yaw - ground_truth_yaw),
            math.cos(estimated_yaw - ground_truth_yaw),
        )
        dx = self.pose.position.x - self.ground_truth_pose.position.x
        dy = self.pose.position.y - self.ground_truth_pose.position.y
        return {
            "dx_m": round(dx, 4),
            "dy_m": round(dy, 4),
            "distance_m": round(math.hypot(dx, dy), 4),
            "dyaw_deg": round(math.degrees(yaw_error), 3),
        }

    def add_start_setup_failure_evidence(self, row):
        """Attach Gazebo evidence to excluded setup failures, including flips."""
        sample = dict(self.safety_stop_sample) if self.safety_stop_sample else {}
        if not sample and self.ground_truth_pose is not None:
            roll, pitch, yaw = quaternion_to_rpy(
                self.ground_truth_pose.orientation
            )
            sample = {
                "x_m": self.ground_truth_pose.position.x,
                "y_m": self.ground_truth_pose.position.y,
                "z_m": self.ground_truth_pose.position.z,
                "roll_deg_signed": math.degrees(roll),
                "pitch_deg_signed": math.degrees(pitch),
                "yaw_deg": math.degrees(yaw),
                "body_up_tilt_deg": body_up_tilt_deg(
                    self.ground_truth_pose.orientation
                ),
            }
        if not sample:
            return row

        def formatted(key):
            return "{:.3f}".format(float(sample[key]))

        tilt_deg = max(
            abs(float(sample["roll_deg_signed"])),
            abs(float(sample["pitch_deg_signed"])),
        )
        row.update({
            "final_ground_truth_x_m": formatted("x_m"),
            "final_ground_truth_y_m": formatted("y_m"),
            "final_ground_truth_z_m": formatted("z_m"),
            "failure_ground_truth_x_m": formatted("x_m"),
            "failure_ground_truth_y_m": formatted("y_m"),
            "failure_ground_truth_z_m": formatted("z_m"),
            "failure_ground_truth_yaw_deg": formatted("yaw_deg"),
            "failure_roll_deg_signed": formatted("roll_deg_signed"),
            "failure_pitch_deg_signed": formatted("pitch_deg_signed"),
            "failure_body_up_tilt_deg": formatted("body_up_tilt_deg"),
            "max_abs_roll_deg": "{:.3f}".format(
                abs(float(sample["roll_deg_signed"]))
            ),
            "max_abs_pitch_deg": "{:.3f}".format(
                abs(float(sample["pitch_deg_signed"]))
            ),
            "max_tilt_deg": "{:.3f}".format(tilt_deg),
            "max_body_up_tilt_deg": formatted("body_up_tilt_deg"),
            "max_tilt_ground_truth_x_m": formatted("x_m"),
            "max_tilt_ground_truth_y_m": formatted("y_m"),
            "max_tilt_ground_truth_z_m": formatted("z_m"),
            "max_tilt_roll_deg_signed": formatted("roll_deg_signed"),
            "max_tilt_pitch_deg_signed": formatted("pitch_deg_signed"),
            "max_tilt_body_up_tilt_deg_at_pose": formatted(
                "body_up_tilt_deg"
            ),
        })
        rospy.logerr(
            "Start-setup failure evidence: reason=%s Gazebo=(%.2f, %.2f, %.2f) "
            "roll=%.1f pitch=%.1f yaw=%.1f body_tilt=%.1f deg",
            row.get("failure_reason", "start_setup_failure"),
            float(sample["x_m"]),
            float(sample["y_m"]),
            float(sample["z_m"]),
            float(sample["roll_deg_signed"]),
            float(sample["pitch_deg_signed"]),
            float(sample["yaw_deg"]),
            float(sample["body_up_tilt_deg"]),
        )
        return row

    def goal_in_control_frame(self, gazebo_goal):
        """Map a fixed physical Gazebo goal into PUTN's drifting world frame."""
        goal = [float(gazebo_goal[0]), float(gazebo_goal[1]), float(gazebo_goal[2])]
        if (
            not self.compensate_goals_with_ground_truth
            or self.pose is None
            or self.ground_truth_pose is None
        ):
            return goal

        _, _, estimated_yaw = quaternion_to_rpy(self.pose.orientation)
        _, _, ground_truth_yaw = quaternion_to_rpy(self.ground_truth_pose.orientation)
        theta = math.atan2(
            math.sin(estimated_yaw - ground_truth_yaw),
            math.cos(estimated_yaw - ground_truth_yaw),
        )
        cosine = math.cos(theta)
        sine = math.sin(theta)
        ground_truth_x = self.ground_truth_pose.position.x
        ground_truth_y = self.ground_truth_pose.position.y
        translation_x = self.pose.position.x - (
            cosine * ground_truth_x - sine * ground_truth_y
        )
        translation_y = self.pose.position.y - (
            sine * ground_truth_x + cosine * ground_truth_y
        )
        return [
            cosine * goal[0] - sine * goal[1] + translation_x,
            sine * goal[0] + cosine * goal[1] + translation_y,
            goal[2],
        ]

    @staticmethod
    def angle_difference_deg(first, second):
        return abs(
            math.degrees(
                math.atan2(
                    math.sin(math.radians(first - second)),
                    math.cos(math.radians(first - second)),
                )
            )
        )

    def discard_followed_paths(self):
        """Stop following paths generated for a goal which is no longer valid."""
        self.latest_path_points = []
        self.active_path_points = []
        self.active_path_wall = None
        self.active_path_source_wall = None

    def refresh_compensated_goal(self, physical_goal, reason, force=False):
        """Refresh a physical goal after the PUTN/Gazebo frame relation changes.

        Proactive updates are position-triggered and rate/budget limited because
        every update restarts PF-RRT*. Forced updates are reserved for verified
        control/physical mismatch or physical-stuck recovery and therefore do
        not consume the proactive budget.
        """
        if not self.compensate_goals_with_ground_truth:
            return "disabled"
        if self.pose is None or self.ground_truth_pose is None:
            return "not_needed"

        now = time.time()
        if (
            not force
            and self.last_goal_publish_wall is not None
            and now - self.last_goal_publish_wall
            < self.goal_compensation_refresh_min_interval_s
        ):
            return "not_needed"

        desired_goal = self.goal_in_control_frame(physical_goal)
        goal_shift = (
            dist2d(desired_goal, self.last_published_goal)
            if self.last_published_goal is not None
            else float("inf")
        )
        sample = self.localization_drift_sample()
        current_yaw = sample.get("dyaw_deg")
        yaw_shift = (
            self.angle_difference_deg(current_yaw, self.last_goal_frame_yaw_deg)
            if current_yaw is not None and self.last_goal_frame_yaw_deg is not None
            else 0.0
        )
        # A navigation target is a point. A pure frame-yaw change must not erase
        # an otherwise usable path; yaw_shift remains a report/log diagnostic.
        if not force and goal_shift < self.goal_compensation_refresh_distance_m:
            return "not_needed"
        if (
            not force
            and self.goal_compensation_proactive_refresh_count
            >= self.goal_compensation_max_refreshes
        ):
            return "proactive_limit"

        old_goal = list(self.last_published_goal) if self.last_published_goal else None
        self.cmd_pub.publish(Twist())
        self.discard_followed_paths()
        self.publish_goal(physical_goal)
        self.goal_compensation_refresh_count += 1
        if force:
            self.goal_compensation_forced_refresh_count += 1
            refresh_index = self.goal_compensation_forced_refresh_count
            refresh_limit = "unbounded helper"
            refresh_kind = "forced"
        else:
            self.goal_compensation_proactive_refresh_count += 1
            refresh_index = self.goal_compensation_proactive_refresh_count
            refresh_limit = str(self.goal_compensation_max_refreshes)
            refresh_kind = "proactive"
        rospy.logwarn(
            "Refreshed compensated goal %s %d/%s (%s): PUTN %s -> [%.2f, %.2f], "
            "shift=%.2f m yaw_change=%.1f deg",
            refresh_kind,
            refresh_index,
            refresh_limit,
            reason,
            "[%.2f, %.2f]" % (old_goal[0], old_goal[1]) if old_goal else "unset",
            self.last_published_goal[0],
            self.last_published_goal[1],
            goal_shift,
            yaw_shift,
        )
        return "refreshed"

    def global_path_cb(self, msg):
        self.path_messages += 1
        if not msg.data:
            # A single failed replanning cycle must not erase a path that is
            # still being followed. Its age is checked by path_is_recent().
            self.last_empty_path_wall = time.time()
            return
        # global_planning_node publishes interpolated xyz triples.
        stride = 3
        points = []
        for i in range(0, len(msg.data) - stride + 1, stride):
            points.append((msg.data[i], msg.data[i + 1]))
        if not self.path_points_are_usable(points):
            # PF-RRT* can publish a one-point/near-zero path after it considers
            # the goal reached. Do not let that overwrite a path which the
            # command follower can still use for the tighter evaluator target.
            self.last_empty_path_wall = time.time()
            return
        self.latest_path_points = points
        self.valid_path_messages += 1
        now = time.time()
        self.last_valid_path_wall = now
        if self.episode_start_wall is not None and self.first_valid_path_wall is None:
            self.first_valid_path_wall = now
        if self.episode_goal is not None and self.first_goal_path_length is None:
            endpoint_error = min(
                dist2d(points[0], self.episode_goal),
                dist2d(points[-1], self.episode_goal),
            )
            if endpoint_error <= float(self.criteria["goal_tolerance_m"]):
                # PF-RRT* may publish rolling sub-paths until the goal becomes
                # projectable. Include distance already travelled so this is a
                # route-length estimate from the episode start, not just the
                # final local segment.
                self.first_goal_path_length = self.episode_distance_m + path_length(points)

    def fused_cloud_any_cb(self, any_msg):
        from sensor_msgs.msg import PointCloud2

        cloud = PointCloud2()
        cloud.deserialize(any_msg._buff)
        fields = {field.name for field in cloud.fields}
        wanted = ["x", "y", "fused_risk"]
        if not all(name in fields for name in wanted):
            return
        self.latest_cloud = [
            (p[0], p[1], p[2])
            for p in pc2.read_points(cloud, field_names=wanted, skip_nans=True)
        ]

    def lesta_map_any_cb(self, any_msg):
        """Confirm that LESTA publishes a usable configured risk layer."""
        if self.lesta_map_ready:
            self.last_lesta_map_wall = time.time()
            return
        try:
            from grid_map_msgs.msg import GridMap

            grid_map = GridMap()
            grid_map.deserialize(any_msg._buff)
            self.lesta_map_received = True
            self.last_lesta_map_wall = time.time()
            self.lesta_map_layers = list(grid_map.layers)
            expected_layers = (
                self.lesta_probability_layer,
                self.lesta_binary_layer,
            )
            for layer_name in expected_layers:
                if layer_name not in grid_map.layers:
                    continue
                layer_index = list(grid_map.layers).index(layer_name)
                if layer_index >= len(grid_map.data):
                    continue
                values = grid_map.data[layer_index].data
                if any(math.isfinite(value) for value in values):
                    self.lesta_map_ready = True
                    self.lesta_ready_layer = layer_name
                    rospy.loginfo(
                        "LESTA readiness confirmed on %s with layer '%s'",
                        self.lesta_map_topic,
                        layer_name,
                    )
                    return
        except Exception as exc:
            rospy.logwarn_throttle(
                5.0, "Unable to inspect LESTA GridMap: %s", exc
            )

    def wait_for_lesta_ready(self):
        if not self.use_lesta or not self.require_lesta_ready:
            return True
        rospy.loginfo(
            "Waiting up to %.1f s for usable LESTA GridMap on %s",
            self.lesta_readiness_timeout_s,
            self.lesta_map_topic,
        )
        deadline = time.time() + self.lesta_readiness_timeout_s
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and time.time() < deadline:
            if self.lesta_map_ready:
                return True
            rate.sleep()
        if self.lesta_map_received:
            rospy.logerr(
                "LESTA GridMap arrived on %s but lacked usable layer '%s' or '%s'; "
                "received layers=%s",
                self.lesta_map_topic,
                self.lesta_probability_layer,
                self.lesta_binary_layer,
                self.lesta_map_layers,
            )
        else:
            rospy.logerr(
                "No LESTA GridMap received on %s; refusing to label fallback PUTN "
                "behavior as lesta_putn",
                self.lesta_map_topic,
            )
        return False

    def reset_episode_state(self):
        self.pose = None
        self.last_pose_time = None
        self.latest_path_points = []
        self.active_path_points = []
        self.active_path_wall = None
        self.active_path_source_wall = None
        self.path_messages = 0
        self.valid_path_messages = 0
        self.last_valid_path_wall = None
        self.last_empty_path_wall = None
        self.first_valid_path_wall = None
        self.first_goal_path_length = None
        self.accepted_path_count = 0
        self.path_origin_rejection_count = 0
        self.episode_start_wall = None
        self.episode_goal = None
        self.episode_distance_m = 0.0
        self.latest_cloud = []
        self.current_speed = 0.0
        self.follow_drive_direction = 1.0
        self.follow_turning_in_place = False
        self.last_published_goal = None
        self.last_requested_physical_goal = None
        self.last_goal_publish_wall = None
        self.last_goal_frame_yaw_deg = None
        self.goal_compensation_refresh_count = 0
        self.goal_compensation_proactive_refresh_count = 0
        self.goal_compensation_forced_refresh_count = 0
        self.control_goal_mismatch_count = 0

    def verify_cmd_vel_ownership(self):
        """Refuse to run when another controller can overwrite our commands."""
        if not self.require_exclusive_cmd_vel:
            rospy.logwarn(
                "Exclusive cmd_vel publisher check is disabled; command conflicts "
                "cannot be detected"
            )
            return True

        resolved_topic = rospy.resolve_name(self.cmd_vel_topic)
        try:
            publishers, _, _ = rosgraph.Master(rospy.get_name()).getSystemState()
        except Exception as exc:
            rospy.logerr("Unable to inspect publishers of %s: %s", resolved_topic, exc)
            return False

        topic_publishers = []
        for topic, nodes in publishers:
            if topic == resolved_topic:
                topic_publishers = list(nodes)
                break
        foreign_publishers = sorted(
            node for node in topic_publishers if node != rospy.get_name()
        )
        if foreign_publishers:
            rospy.logfatal(
                "Refusing autonomous evaluation: %s already has other publishers %s. "
                "Stop the manual/local controller launch; do not run two cmd_vel "
                "controllers at the same time.",
                resolved_topic,
                ", ".join(foreign_publishers),
            )
            return False

        rospy.loginfo(
            "Autonomous command ownership confirmed: %s is controlled only by %s",
            resolved_topic,
            rospy.get_name(),
        )
        return True

    def reset_robot_pose_gazebo(self, start):
        if self.set_model_state is None:
            raise RuntimeError("Gazebo reset requested while set_model_state is not initialized")

        state = ModelState()
        state.model_name = self.robot_model_name
        state.reference_frame = "world"
        state.pose.position.x = start[0]
        state.pose.position.y = start[1]
        state.pose.position.z = max(start[2], 0.25)
        q = yaw_to_quaternion(0.0)
        state.pose.orientation.x = q[0]
        state.pose.orientation.y = q[1]
        state.pose.orientation.z = q[2]
        state.pose.orientation.w = q[3]
        state.twist = Twist()
        try:
            resp = self.set_model_state(state)
            if not resp.success and self.robot_model_name.endswith("/"):
                state.model_name = self.robot_model_name.rstrip("/")
                resp = self.set_model_state(state)
            if not resp.success:
                rospy.logwarn("Failed to reset robot pose: %s", resp.status_message)
        except rospy.ServiceException as exc:
            rospy.logerr("Gazebo reset service failed: %s", exc)
            raise

        stop = Twist()
        for _ in range(10):
            self.cmd_pub.publish(stop)
            rospy.sleep(0.05)
        rospy.sleep(self.reset_settle_s)

    def wait_for_pose(self, timeout_s=10.0):
        deadline = time.time() + timeout_s
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and time.time() < deadline:
            if self.safety_stop_latched:
                self.cmd_pub.publish(Twist())
                rospy.logerr("Stopping pose wait: robot is overturned")
                return False
            self.update_control_pose()
            if self.pose is not None:
                return True
            rate.sleep()
        return False

    def publish_goal(self, goal, begin_episode=False):
        self.update_control_pose()
        published_goal = self.goal_in_control_frame(goal)
        self.last_published_goal = published_goal
        self.last_requested_physical_goal = list(goal)
        if self.episode_start_wall is not None:
            self.episode_goal = list(published_goal)
        msg = PoseStamped()
        msg.header.frame_id = self.eval_cfg.get("fixed_frame", "world")
        msg.pose.position.x = published_goal[0]
        msg.pose.position.y = published_goal[1]
        msg.pose.position.z = max(published_goal[2], 0.1)
        q = yaw_to_quaternion(0.0)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        first_publish_wall = None
        for _ in range(self.goal_publish_count):
            if first_publish_wall is None:
                first_publish_wall = time.time()
                if begin_episode:
                    self.episode_start_wall = first_publish_wall
                    self.episode_goal = list(published_goal)
            msg.header.stamp = rospy.Time.now()
            self.goal_pub.publish(msg)
            rospy.sleep(0.2)
        self.last_goal_publish_wall = time.time()
        if self.compensate_goals_with_ground_truth:
            sample = self.localization_drift_sample()
            self.last_goal_frame_yaw_deg = sample.get("dyaw_deg")
            rospy.loginfo(
                "Goal-frame compensation: Gazebo [%.2f, %.2f] -> PUTN [%.2f, %.2f] "
                "(dx=%s dy=%s dyaw=%s deg)",
                goal[0], goal[1], published_goal[0], published_goal[1],
                sample.get("dx_m", ""), sample.get("dy_m", ""),
                sample.get("dyaw_deg", ""),
            )
        return first_publish_wall

    def point_is_safe(self, point):
        pit_distance, column_distance = self.distances_to_hazards(point)
        return (
            pit_distance > self.warmup_safety_margin_m
            and column_distance > self.warmup_safety_margin_m
        )

    def generate_coverage_waypoints(self):
        warmup_cfg = self.config.get("warmup", {})
        configured = warmup_cfg.get("waypoints", [])
        if configured:
            checkpoints = []
            for index, item in enumerate(configured, start=1):
                point = item.get("point") if isinstance(item, dict) else item
                if not isinstance(point, (list, tuple)) or len(point) != 3:
                    raise ValueError(
                        "warmup.waypoints item {} must contain a three-element point".format(index)
                    )
                point = [float(value) for value in point]
                point[0] *= self.warmup_coordinate_scale
                point[1] *= self.warmup_coordinate_scale
                if not self.point_is_safe(point):
                    rospy.logwarn(
                        "Configured warmup waypoint %d [%.2f, %.2f] is close to a YAML hazard; "
                        "keeping it because explicit waypoints take precedence",
                        index, point[0], point[1],
                    )
                checkpoints.append(point)

            spacing = float(warmup_cfg.get("interpolation_spacing_m", 0.0))
            if spacing <= 0.0:
                rospy.loginfo("Using %d explicit warmup waypoints from YAML", len(checkpoints))
                return checkpoints

            route_start = warmup_cfg.get("route_start", [0.0, 0.0, 0.0])
            if not isinstance(route_start, (list, tuple)) or len(route_start) != 3:
                raise ValueError("warmup.route_start must contain three values")
            previous = [float(value) for value in route_start]
            previous[0] *= self.warmup_coordinate_scale
            previous[1] *= self.warmup_coordinate_scale
            waypoints = []
            for target in checkpoints:
                distance = dist2d(previous, target)
                segments = max(1, int(math.ceil(distance / spacing)))
                for segment in range(1, segments + 1):
                    ratio = float(segment) / segments
                    waypoints.append([
                        previous[0] + (target[0] - previous[0]) * ratio,
                        previous[1] + (target[1] - previous[1]) * ratio,
                        previous[2] + (target[2] - previous[2]) * ratio,
                    ])
                previous = target
            rospy.loginfo(
                "Expanded %d route checkpoints into %d local warmup goals (spacing <= %.2f m)",
                len(checkpoints), len(waypoints), spacing,
            )
            return waypoints

        bounds = self.config.get("map", {}).get("bounds", {})
        required = ("x_min", "x_max", "y_min", "y_max")
        if not all(key in bounds for key in required):
            raise ValueError("coverage_waypoints warmup requires map.bounds in the evaluation YAML")

        margin = float(self.config.get("map", {}).get("safe_start_goal_margin_m", 1.0))
        spacing = self.warmup_waypoint_spacing_m
        if spacing <= 0.0:
            raise ValueError("warmup_waypoint_spacing_m must be positive")

        x_min, x_max = bounds["x_min"] + margin, bounds["x_max"] - margin
        y_min, y_max = bounds["y_min"] + margin, bounds["y_max"] - margin
        rows = max(1, int(math.floor((y_max - y_min) / spacing)) + 1)
        waypoints = []
        for row in range(rows):
            y = min(y_min + row * spacing, y_max)
            xs = (x_min, x_max) if row % 2 == 0 else (x_max, x_min)
            for x in xs:
                point = [float(x), float(y), 0.0]
                if self.point_is_safe(point):
                    waypoints.append(point)
        return waypoints

    def navigate_to_point(self, target, tolerance, timeout_s, label, absolute_deadline=None):
        if not self.wait_for_pose():
            rospy.logwarn("No robot pose while navigating to %s", label)
            return False
        self.reset_episode_state()
        self.publish_goal(target)
        deadline = time.time() + timeout_s
        if absolute_deadline is not None:
            deadline = min(deadline, absolute_deadline)
        no_path_deadline = time.time() + self.path_grace_s
        recovery_attempt = 0
        has_seen_valid_path = False
        best_distance = float("inf")
        last_progress_wall = time.time()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and time.time() < deadline:
            if self.safety_stop_latched:
                self.cmd_pub.publish(Twist())
                rospy.logerr("Stopping navigation to %s: robot is overturned", label)
                return False
            self.update_control_pose()
            if self.pose is None:
                rate.sleep()
                continue
            if self.localization_drift_exceeded():
                self.cmd_pub.publish(Twist())
                rospy.logerr(
                    "Stopping navigation to %s: TF/Gazebo offset %.2f m exceeded "
                    "the %.2f m limit for %.1f s",
                    label, self.tf_gazebo_offset_m,
                    self.localization_drift_fail_m, self.localization_drift_hold_s,
                )
                return False
            control_pos = (self.pose.position.x, self.pose.position.y)
            if self.ground_truth_pose is not None:
                metric_pos = (
                    self.ground_truth_pose.position.x,
                    self.ground_truth_pose.position.y,
                )
            else:
                metric_pos = control_pos
            distance_to_target = dist2d(metric_pos, target)
            if distance_to_target <= tolerance:
                self.cmd_pub.publish(Twist())
                return True
            if distance_to_target <= best_distance - self.warmup_min_progress_m:
                best_distance = distance_to_target
                last_progress_wall = time.time()

            path_is_fresh = self.path_is_recent(self.warmup_path_stale_s)
            active_path_is_followable = self.active_path_is_recent()
            has_followable_path = path_is_fresh or active_path_is_followable
            has_seen_valid_path = has_seen_valid_path or has_followable_path
            no_progress = time.time() - last_progress_wall >= self.warmup_no_progress_timeout_s
            if has_followable_path and not no_progress:
                no_path_deadline = time.time() + self.path_grace_s
                _, _, yaw = quaternion_to_rpy(self.pose.orientation)
                self.publish_path_follow_cmd(
                    control_pos, yaw, self.last_published_goal or target
                )
            elif no_progress and has_followable_path:
                rospy.logwarn(
                    "No progress toward %s for %.1f s (distance %.2f m); "
                    "requesting a new path without projection motion recovery",
                    label, self.warmup_no_progress_timeout_s, distance_to_target,
                )
                path_request_wall = time.time()
                # Do not keep commanding a path which has already failed the
                # progress test. A genuinely new usable path must be received.
                self.latest_path_points = []
                self.active_path_points = []
                self.active_path_wall = None
                self.active_path_source_wall = None
                self.publish_goal(target)
                has_seen_valid_path = False
                no_path_deadline = time.time() + self.path_grace_s
                best_distance = distance_to_target
                last_progress_wall = time.time()
                if self.has_new_usable_path(path_request_wall):
                    has_seen_valid_path = True
            elif time.time() >= no_path_deadline:
                if has_seen_valid_path:
                    rospy.logwarn(
                        "Path to %s became unavailable after a valid sub path was seen; "
                        "re-requesting planning without open-loop projection recovery",
                        label,
                    )
                    self.latest_path_points = []
                    self.publish_goal(target)
                    has_seen_valid_path = False
                    no_path_deadline = time.time() + self.path_grace_s
                    best_distance = distance_to_target
                    last_progress_wall = time.time()
                    rate.sleep()
                    continue
                if recovery_attempt >= len(self.projection_recovery_distances_m):
                    rospy.logwarn(
                        "No path to %s after %d projection recovery attempts",
                        label, recovery_attempt,
                    )
                    break
                if self.projection_recovery_speed_mps <= 0.0:
                    rospy.logerr("projection_recovery_speed_mps must be positive")
                    break
                distance = self.projection_recovery_distances_m[recovery_attempt]
                recovery_attempt += 1
                rospy.logwarn(
                    "No path to %s; projection recovery %d/%d: move %+.2f m",
                    label, recovery_attempt, len(self.projection_recovery_distances_m), distance,
                )
                signed_speed = math.copysign(self.projection_recovery_speed_mps, distance)
                duration = abs(distance) / self.projection_recovery_speed_mps
                recovery_started_wall = time.time()

                def recovery_path_ready():
                    return self.has_new_usable_path(recovery_started_wall)

                interrupted = self.execute_cmd_sequence(
                    [(signed_speed, 0.0, duration)],
                    "Projection recovery",
                    settle_s=0.0,
                    stop_when=recovery_path_ready,
                )
                scan_angle = self.projection_recovery_scan_angle_deg
                if scan_angle > 0.0 and not interrupted:
                    rospy.loginfo(
                        "Projection recovery scan: left %.1f deg, right %.1f deg, return center",
                        scan_angle, scan_angle,
                    )
                    for angle in (scan_angle, -2.0 * scan_angle, scan_angle):
                        _, interrupted = self.rotate_relative(
                            angle, stop_when=recovery_path_ready
                        )
                        if interrupted:
                            break
                self.cmd_pub.publish(Twist())
                if not interrupted:
                    interrupted = self.wait_interruptibly(
                        self.projection_recovery_map_settle_s, recovery_path_ready
                    )
                if interrupted:
                    has_seen_valid_path = True
                    rospy.loginfo(
                        "Planning succeeded during projection recovery for %s; "
                        "stopping recovery and following the new path",
                        label,
                    )
                    no_path_deadline = time.time() + self.path_grace_s
                else:
                    self.latest_path_points = []
                    self.publish_goal(target)
                    no_path_deadline = time.time() + self.path_grace_s
                best_distance = distance_to_target
                last_progress_wall = time.time()
            rate.sleep()
        self.cmd_pub.publish(Twist())
        rospy.logwarn(
            "Navigation attempt to %s failed after %.1f s without arrival",
            label, timeout_s,
        )
        return False

    def run_coverage_warmup(self, return_start):
        waypoints = self.generate_coverage_waypoints()
        if not waypoints:
            rospy.logerr("Coverage warmup generated no safe waypoints")
            return False, {}

        rospy.loginfo("Coverage warmup: %d safe waypoints", len(waypoints))
        started = time.time()
        drift_samples = []
        total_deadline = started + self.warmup_total_timeout_s
        if self.warmup_bootstrap_enabled:
            if not self.warmup_cmd_sequence:
                rospy.logerr(
                    "Coverage bootstrap is enabled but warmup_cmd_sequence is empty; "
                    "PF-RRT* may not project the start through the LiDAR blind spot"
                )
                return False, {"mode": "coverage_waypoints", "passed": False,
                               "failure_reason": "empty_bootstrap_sequence"}
            rospy.loginfo(
                "Running open-loop mapping bootstrap before PF-RRT* coverage (%d phases)",
                len(self.warmup_cmd_sequence),
            )
            if not self.wait_for_pose():
                return False, {"mode": "coverage_waypoints", "passed": False,
                               "failure_reason": "no_pose_before_bootstrap"}
            bootstrap_pose = self.ground_truth_pose or self.pose
            bootstrap_start = (
                bootstrap_pose.position.x,
                bootstrap_pose.position.y,
            )
            self.execute_cmd_sequence(self.warmup_cmd_sequence, "Mapping bootstrap")
            if self.safety_stop_latched:
                return False, {
                    "mode": "coverage_waypoints",
                    "passed": False,
                    "failure_reason": "robot_overturned",
                }
            bootstrap_pose = self.ground_truth_pose or self.pose
            if bootstrap_pose is None:
                bootstrap_displacement = 0.0
            else:
                bootstrap_displacement = dist2d(
                    bootstrap_start,
                    (bootstrap_pose.position.x, bootstrap_pose.position.y),
                )
            rospy.loginfo("Mapping bootstrap displacement: %.3f m", bootstrap_displacement)
            if bootstrap_displacement < self.warmup_bootstrap_min_displacement_m:
                rospy.logerr(
                    "Bootstrap published /cmd_vel but robot moved only %.3f m (required %.3f m). "
                    "Check gazebo_ros_control PID gains and wheel controllers.",
                    bootstrap_displacement, self.warmup_bootstrap_min_displacement_m,
                )
                return False, {
                    "mode": "coverage_waypoints", "passed": False,
                    "failure_reason": "bootstrap_robot_not_moving",
                    "bootstrap_displacement_m": round(bootstrap_displacement, 4),
                }

        completed = 0
        attempted = 0
        skipped_waypoints = []
        for index, waypoint in enumerate(waypoints, start=1):
            if (
                rospy.is_shutdown()
                or self.safety_stop_latched
                or time.time() >= total_deadline
            ):
                break
            attempted += 1
            rospy.loginfo(
                "Warmup waypoint %d/%d [%.2f, %.2f]",
                index, len(waypoints), waypoint[0], waypoint[1],
            )
            reached = False
            attempts_used = 0
            for retry in range(self.warmup_waypoint_retries + 1):
                remaining = total_deadline - time.time()
                if rospy.is_shutdown() or self.safety_stop_latched or remaining <= 0.0:
                    break
                if retry > 0:
                    rospy.logwarn(
                        "Retrying warmup waypoint %d/%d (%d/%d)",
                        index, len(waypoints), retry, self.warmup_waypoint_retries,
                    )
                attempts_used += 1
                waypoint_tolerance = self.warmup_waypoint_tolerance_m
                is_final_return_waypoint = (
                    index == len(waypoints)
                    and dist2d(waypoint, return_start) <= 1e-3
                )
                if self.warmup_end_requires_start and is_final_return_waypoint:
                    # Reach the formal start tolerance in the existing motion;
                    # stopping at the looser waypoint tolerance and resetting
                    # the active path can strand the robot just outside start.
                    waypoint_tolerance = min(waypoint_tolerance, self.start_tolerance_m)
                reached = self.navigate_to_point(
                    waypoint,
                    waypoint_tolerance,
                    min(self.warmup_waypoint_timeout_s, remaining),
                    "warmup waypoint {}/{}".format(index, len(waypoints)),
                    absolute_deadline=total_deadline,
                )
                if reached:
                    break
                if self.localization_failure_detected:
                    break
            drift_sample = self.localization_drift_sample()
            drift_sample.update({
                "waypoint_index": index,
                "target_x": round(waypoint[0], 3),
                "target_y": round(waypoint[1], 3),
                "reached": reached,
            })
            drift_samples.append(drift_sample)
            rospy.loginfo(
                "Warmup localization %d/%d: dx=%s m dy=%s m dyaw=%s deg "
                "distance=%s m reached=%s",
                index, len(waypoints), drift_sample.get("dx_m", ""),
                drift_sample.get("dy_m", ""), drift_sample.get("dyaw_deg", ""),
                drift_sample.get("distance_m", ""), reached,
            )
            if reached:
                completed += 1
            else:
                skipped_waypoints.append(index)
                rospy.logwarn(
                    "Skipping warmup waypoint %d/%d after %d failed attempt(s)",
                    index, len(waypoints), attempts_used,
                )
                if self.localization_failure_detected:
                    rospy.logerr("Aborting warmup because localization frames diverged")
                    break

        coverage = float(completed) / len(waypoints)
        return_start_control = self.goal_in_control_frame(return_start)
        returned_estimated = False
        if self.pose is not None:
            returned_estimated = is_within_start_tolerance(
                (self.pose.position.x, self.pose.position.y),
                return_start_control,
                self.start_tolerance_m,
            )
        returned_ground_truth = False
        if self.ground_truth_pose is not None:
            returned_ground_truth = is_within_start_tolerance(
                (self.ground_truth_pose.position.x, self.ground_truth_pose.position.y),
                return_start,
                self.start_tolerance_m,
            )
        returned = returned_estimated and returned_ground_truth
        if (
            self.warmup_end_requires_start
            and not returned
            and not self.localization_failure_detected
        ):
            remaining = total_deadline - time.time()
            if remaining > 0.0:
                self.navigate_to_point(
                    return_start,
                    self.start_tolerance_m,
                    min(self.reposition_timeout_s, remaining),
                    "evaluation start after warmup",
                    absolute_deadline=total_deadline,
                )
                return_start_control = self.goal_in_control_frame(return_start)
                returned_estimated = bool(
                    self.pose is not None
                    and is_within_start_tolerance(
                        (self.pose.position.x, self.pose.position.y),
                        return_start_control,
                        self.start_tolerance_m,
                    )
                )
                returned_ground_truth = bool(
                    self.ground_truth_pose is not None
                    and is_within_start_tolerance(
                        (
                            self.ground_truth_pose.position.x,
                            self.ground_truth_pose.position.y,
                        ),
                        return_start,
                        self.start_tolerance_m,
                    )
                )
                returned = returned_estimated and returned_ground_truth
        if returned and self.rrt_settle_s > 0.0:
            # Publishing the current position lets the planner finish the previous goal;
            # the following goal-free interval is used by PF-RRT* for tree expansion.
            self.publish_goal(return_start)
            self.cmd_pub.publish(Twist())
            settle_s = min(self.rrt_settle_s, max(total_deadline - time.time(), 0.0))
            rospy.loginfo("Goal-free PF-RRT* settling for %.1f s", settle_s)
            rospy.sleep(settle_s)

        final_estimated_distance_to_start = ""
        if self.pose is not None:
            return_start_control = self.goal_in_control_frame(return_start)
            final_estimated_distance_to_start = round(
                dist2d(
                    (self.pose.position.x, self.pose.position.y),
                    return_start_control,
                ),
                4,
            )
        final_ground_truth_distance_to_start = ""
        if self.ground_truth_pose is not None:
            final_ground_truth_distance_to_start = round(
                dist2d(
                    (self.ground_truth_pose.position.x, self.ground_truth_pose.position.y),
                    return_start,
                ),
                4,
            )
        timed_out = (
            time.time() >= total_deadline
            and self.warmup_end_requires_start
            and not returned
        )
        return_requirement_passed = returned or not self.warmup_end_requires_start
        passed = return_requirement_passed and coverage >= self.warmup_target_coverage_ratio
        failure_reason = ""
        if not passed:
            if self.safety_stop_latched:
                failure_reason = "robot_overturned"
            elif self.localization_failure_detected:
                failure_reason = "localization_drift"
            elif coverage < self.warmup_target_coverage_ratio:
                failure_reason = "insufficient_coverage"
            elif timed_out:
                failure_reason = "warmup_total_timeout"
            else:
                failure_reason = "return_to_start_failed"

        report = {
            "mode": "coverage_waypoints",
            "bootstrap_enabled": self.warmup_bootstrap_enabled,
            "goal_frame_compensation_enabled": self.compensate_goals_with_ground_truth,
            "abort_on_localization_drift": self.abort_on_localization_drift,
            "duration_s": round(time.time() - started, 3),
            "waypoints_total": len(waypoints),
            "waypoints_attempted": attempted,
            "waypoints_completed": completed,
            "skipped_waypoints": skipped_waypoints,
            "coverage_ratio": round(coverage, 4),
            "returned_to_start": returned,
            "returned_to_start_estimated": returned_estimated,
            "returned_to_start_ground_truth": returned_ground_truth,
            "final_distance_to_start_m": final_ground_truth_distance_to_start,
            "final_estimated_distance_to_start_m": final_estimated_distance_to_start,
            "final_tf_gazebo_offset_m": (
                round(self.tf_gazebo_offset_m, 4)
                if self.tf_gazebo_offset_m is not None else ""
            ),
            "localization_drift_samples": drift_samples,
            "timed_out": timed_out,
            "failure_reason": failure_reason,
            "passed": passed,
        }
        rospy.loginfo("Coverage warmup report: %s", report)
        return report["passed"], report

    @staticmethod
    def path_points_are_usable(points):
        return len(points) >= 2 and path_length(points) >= 0.05

    def path_is_recent(self, max_age_s):
        return bool(
            self.path_points_are_usable(self.latest_path_points)
            and self.last_valid_path_wall is not None
            and time.time() - self.last_valid_path_wall <= max_age_s
        )

    def active_path_is_recent(self):
        return bool(
            self.path_points_are_usable(self.active_path_points)
            and self.active_path_wall is not None
            and time.time() - self.active_path_wall <= self.path_max_hold_s
        )

    def has_new_usable_path(self, after_wall):
        """Accept global and rolling PF-RRT* paths published after an action began."""
        if (
            not self.path_points_are_usable(self.latest_path_points)
            or self.last_valid_path_wall is None
            or self.last_valid_path_wall <= after_wall
        ):
            return False
        # A rolling path terminates at a temporary local target rather than the
        # final goal, so endpoint-to-final-goal distance is intentionally not used.
        return True

    def wait_interruptibly(self, duration, stop_when):
        deadline = time.time() + max(duration, 0.0)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and time.time() < deadline:
            if self.safety_stop_latched:
                self.cmd_pub.publish(Twist())
                return False
            if stop_when is not None and stop_when():
                self.cmd_pub.publish(Twist())
                return True
            rate.sleep()
        return False

    def execute_cmd_sequence(self, sequence, label, settle_s=None, stop_when=None):
        rate = rospy.Rate(10)
        for index, (linear, angular, duration) in enumerate(sequence, start=1):
            rospy.loginfo(
                "%s phase %d/%d: linear=%.3f angular=%.3f duration=%.1f",
                label, index, len(sequence), linear, angular, duration,
            )
            cmd = Twist()
            cmd.linear.x = linear
            cmd.angular.z = angular
            deadline = time.time() + duration
            while not rospy.is_shutdown() and time.time() < deadline:
                if self.safety_stop_latched:
                    self.cmd_pub.publish(Twist())
                    return False
                if stop_when is not None and stop_when():
                    self.cmd_pub.publish(Twist())
                    return True
                self.cmd_pub.publish(cmd)
                rate.sleep()

        self.cmd_pub.publish(Twist())
        interrupted = self.wait_interruptibly(
            self.reset_settle_s if settle_s is None else settle_s,
            stop_when,
        )
        return interrupted

    def run_fixed_motion_warmup(self, start):
        if not self.warmup_cmd_sequence:
            rospy.logwarn("warmup_mode:=fixed_motion requested but warmup_cmd_sequence is empty")
            return True

        rospy.loginfo(
            "Running fixed-motion warmup with %d command phases before evaluation",
            len(self.warmup_cmd_sequence),
        )
        self.execute_cmd_sequence(self.warmup_cmd_sequence, "Fixed warmup")
        if self.safety_stop_latched:
            return False

        if self.warmup_reset_to_start:
            if self.start_mode == "gazebo":
                rospy.loginfo("Resetting robot to fixed episode start after warmup")
                self.reset_robot_pose_gazebo(start)
            else:
                rospy.logwarn(
                    "warmup_reset_to_start is true, but start_mode is '%s'. "
                    "For fixed-start simulation tests use start_mode:=gazebo.",
                    self.start_mode,
                )

        if self.warmup_end_requires_start:
            if not self.wait_for_pose():
                rospy.logwarn("No robot pose received after warmup")
                return False
            metric_pose = self.ground_truth_pose or self.pose
            pos = (metric_pose.position.x, metric_pose.position.y)
            distance_to_start = dist2d(pos, start)
            if not is_within_start_tolerance(pos, start, self.start_tolerance_m):
                rospy.logwarn(
                    "Warmup ended %.2f m from the configured start; tolerance is %.2f m",
                    distance_to_start,
                    self.start_tolerance_m,
                )
                return False

        return True

    def warm_up_planner(self, start):
        if self.warmup_mode == "none":
            return True
        if self.warmup_mode == "coverage_waypoints":
            passed, _ = self.run_coverage_warmup(start)
            return passed
        if self.pre_goal_warmup_s <= 0.0:
            return True
        if self.warmup_mode == "fixed_motion":
            if not self.run_fixed_motion_warmup(start):
                return False
            self.reset_episode_state()
            return True
        if self.warmup_mode != "stationary":
            raise ValueError("Unsupported warmup_mode: {}".format(self.warmup_mode))

        rospy.loginfo(
            "Warming up PUTN planner at the episode start for %.1f seconds before sending the evaluation goal",
            self.pre_goal_warmup_s,
        )
        self.publish_goal(start)
        rospy.sleep(self.hold_goal_clear_s)
        self.cmd_pub.publish(Twist())

        deadline = time.time() + self.pre_goal_warmup_s
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and time.time() < deadline:
            if self.safety_stop_latched:
                self.cmd_pub.publish(Twist())
                return False
            self.cmd_pub.publish(Twist())
            rate.sleep()
        self.reset_episode_state()
        return True

    def navigate_to_start(self, start):
        self.last_start_setup_failure_reason = ""
        if not self.wait_for_pose():
            rospy.logwarn("No robot pose received before repositioning to start")
            self.last_start_setup_failure_reason = "start_setup_no_pose"
            return False

        rospy.loginfo(
            "Navigating to episode start [%.2f, %.2f] without Gazebo teleport",
            start[0],
            start[1],
        )
        self.reset_episode_state()
        self.publish_goal(start)

        deadline = time.time() + self.reposition_timeout_s
        mismatch_started_wall = None
        mismatch_reference_distance = None
        mismatch_forced_without_progress = 0
        physical_anchor = None
        physical_anchor_wall = time.time()
        physical_stuck_recovery_count = 0
        no_path_since = time.time()
        heading_recovery_count = 0
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and time.time() < deadline:
            if self.safety_stop_latched:
                self.cmd_pub.publish(Twist())
                self.last_start_setup_failure_reason = (
                    "start_setup_robot_overturned"
                )
                return False
            self.update_control_pose()
            if self.pose is None:
                rate.sleep()
                continue
            if self.localization_drift_exceeded():
                self.cmd_pub.publish(Twist())
                rospy.logerr(
                    "Unable to prepare episode start: TF/Gazebo offset is %.2f m",
                    self.tf_gazebo_offset_m,
                )
                self.last_start_setup_failure_reason = (
                    "start_setup_localization_drift"
                )
                return False

            control_pos = (self.pose.position.x, self.pose.position.y)
            if self.ground_truth_pose is None:
                rate.sleep()
                continue
            ground_truth_pos = (
                self.ground_truth_pose.position.x,
                self.ground_truth_pose.position.y,
            )
            physical_distance = dist2d(ground_truth_pos, start)
            if physical_distance <= self.start_tolerance_m:
                self.cmd_pub.publish(Twist())
                rospy.sleep(self.reset_settle_s)
                self.reset_episode_state()
                return True

            refresh_status = self.refresh_compensated_goal(
                start, "frame drift while preparing episode start"
            )
            if refresh_status == "refreshed":
                mismatch_started_wall = None
                physical_anchor = ground_truth_pos
                physical_anchor_wall = time.time()
                no_path_since = time.time()
                rate.sleep()
                continue

            control_distance = dist2d(
                control_pos, self.last_published_goal or start
            )
            mismatch = (
                self.compensate_goals_with_ground_truth
                and control_distance <= self.control_goal_tolerance_m
                and physical_distance
                >= self.control_goal_mismatch_physical_error_m
            )
            if mismatch:
                self.cmd_pub.publish(Twist())
                if (
                    mismatch_reference_distance is None
                    or physical_distance
                    <= mismatch_reference_distance
                    - self.goal_compensation_forced_min_progress_m
                ):
                    mismatch_reference_distance = physical_distance
                    mismatch_forced_without_progress = 0
                if mismatch_started_wall is None:
                    mismatch_started_wall = time.time()
                    self.control_goal_mismatch_count += 1
                    rospy.logwarn(
                        "Episode-start goal mismatch: PUTN distance %.2f m but "
                        "physical distance %.2f m; waiting %.1f s before refresh",
                        control_distance,
                        physical_distance,
                        self.control_goal_mismatch_hold_s,
                    )
                if (
                    time.time() - mismatch_started_wall
                    >= self.control_goal_mismatch_hold_s
                ):
                    if (
                        mismatch_forced_without_progress
                        >= self.goal_compensation_max_forced_refreshes
                    ):
                        self.last_start_setup_failure_reason = (
                            "start_setup_goal_frame_divergence"
                        )
                        rospy.logerr(
                            "Unable to prepare episode start: mismatch persisted "
                            "through %d forced refreshes without %.2f m physical progress",
                            mismatch_forced_without_progress,
                            self.goal_compensation_forced_min_progress_m,
                        )
                        return False
                    refresh_status = self.refresh_compensated_goal(
                        start, "control goal reached but physical start not reached", force=True
                    )
                    mismatch_started_wall = None
                    if refresh_status == "refreshed":
                        mismatch_forced_without_progress += 1
                        physical_anchor = ground_truth_pos
                        physical_anchor_wall = time.time()
                        no_path_since = time.time()
                        rate.sleep()
                        continue
                rate.sleep()
                continue
            mismatch_started_wall = None

            if physical_anchor is None:
                physical_anchor = ground_truth_pos
                physical_anchor_wall = time.time()
            elif (
                dist2d(ground_truth_pos, physical_anchor)
                >= self.physical_stuck_displacement_m
            ):
                physical_anchor = ground_truth_pos
                physical_anchor_wall = time.time()
            elif (
                (self.path_is_recent(self.warmup_path_stale_s) or self.active_path_is_recent())
                and time.time() - physical_anchor_wall >= self.physical_stuck_hold_s
            ):
                if (
                    physical_stuck_recovery_count
                    >= self.physical_stuck_max_recoveries
                ):
                    refresh_status = "limit"
                else:
                    refresh_status = self.refresh_compensated_goal(
                        start,
                        "no physical displacement while preparing episode start",
                        force=True,
                    )
                    if refresh_status == "refreshed":
                        physical_stuck_recovery_count += 1
                if refresh_status == "refreshed":
                    physical_anchor = ground_truth_pos
                    physical_anchor_wall = time.time()
                    rate.sleep()
                    continue
                self.last_start_setup_failure_reason = (
                    "start_setup_no_physical_progress"
                )
                self.cmd_pub.publish(Twist())
                rospy.logerr(
                    "Unable to prepare episode start: physical displacement stayed "
                    "below %.2f m for %.1f s despite a followable path",
                    self.physical_stuck_displacement_m,
                    self.physical_stuck_hold_s,
                )
                return False

            path_is_fresh = self.path_is_recent(self.warmup_path_stale_s)
            has_followable_path = path_is_fresh or self.active_path_is_recent()
            if has_followable_path:
                no_path_since = None
            elif no_path_since is None:
                no_path_since = time.time()

            if (
                no_path_since is not None
                and time.time() - no_path_since >= self.heading_recovery_trigger_s
            ):
                can_recover = (
                    self.enable_heading_recovery
                    and heading_recovery_count < len(self.heading_recovery_angles_deg)
                )
                if not can_recover:
                    self.cmd_pub.publish(Twist())
                    self.last_start_setup_failure_reason = (
                        "start_setup_no_path_after_heading_recovery"
                    )
                    rospy.logerr(
                        "Unable to prepare episode start: no usable path after %d "
                        "heading-recovery attempts",
                        heading_recovery_count,
                    )
                    return False
                _, completed, path_found = self.recover_heading_for_projection(
                    start, heading_recovery_count
                )
                heading_recovery_count += 1
                no_path_since = None if path_found else time.time()
                physical_anchor = ground_truth_pos
                physical_anchor_wall = time.time()
                if not completed and not path_found:
                    rospy.logwarn(
                        "Episode-start heading recovery timed out; trying the next angle"
                    )
                continue

            _, _, yaw = quaternion_to_rpy(self.pose.orientation)
            cmd = self.publish_path_follow_cmd(
                control_pos, yaw, self.last_published_goal or start
            )
            translating = cmd is not None and abs(cmd.linear.x) >= float(
                self.criteria.get("stuck_speed_threshold_mps", 0.03)
            )
            if not translating:
                # Intentional turn-in-place is not physical translation failure.
                physical_anchor = ground_truth_pos
                physical_anchor_wall = time.time()
            rate.sleep()

        self.cmd_pub.publish(Twist())
        rospy.logwarn("Timed out while navigating to episode start")
        self.last_start_setup_failure_reason = "start_setup_timeout"
        return False

    def prepare_episode_start(self, start):
        self.last_start_setup_failure_reason = ""
        if self.start_mode == "navigate":
            return self.navigate_to_start(start)
        if self.start_mode == "gazebo":
            self.reset_episode_state()
            self.reset_robot_pose_gazebo(start)
            return True
        if self.start_mode == "none":
            if not self.wait_for_pose():
                self.last_start_setup_failure_reason = "start_setup_no_pose"
                return False
            pos = (self.pose.position.x, self.pose.position.y)
            ground_truth_pos = None
            if self.ground_truth_pose is not None:
                ground_truth_pos = (
                    self.ground_truth_pose.position.x,
                    self.ground_truth_pose.position.y,
                )
            estimated_at_start = dist2d(pos, start) <= self.start_tolerance_m
            ground_truth_at_start = (
                ground_truth_pos is not None
                and dist2d(ground_truth_pos, start) <= self.start_tolerance_m
            )
            if ground_truth_at_start and (
                self.compensate_goals_with_ground_truth or estimated_at_start
            ):
                self.reset_episode_state()
                return True
            rospy.logwarn(
                "Robot is %.2f m from requested start. Move it near the start or use start_mode:=navigate.",
                dist2d(pos, start),
            )
            self.last_start_setup_failure_reason = "start_setup_not_at_start"
            return False
        raise ValueError("Unsupported start_mode: {}".format(self.start_mode))

    def distances_to_hazards(self, point):
        min_pit = (
            min(rect_distance(point, pit["center"], pit["size"]) for pit in self.pits)
            if self.pits
            else float("inf")
        )
        min_column = (
            min(max(dist2d(point, col["center"]) - col["radius_m"], 0.0) for col in self.columns)
            if self.columns
            else float("inf")
        )
        return min_pit, min_column

    def nearby_risk_values(self, point):
        if not self.latest_cloud:
            return []
        radius = self.risk_sample_radius_m
        return [
            risk
            for x, y, risk in self.latest_cloud
            if math.hypot(point[0] - x, point[1] - y) <= radius
        ]

    @staticmethod
    def orient_path_from_robot(points, pos):
        oriented = list(points)
        if len(oriented) >= 2 and dist2d(oriented[0], pos) > dist2d(oriented[-1], pos):
            oriented.reverse()
        return oriented

    def path_tracking_target(self, points, pos):
        nearest = min(range(len(points)), key=lambda i: dist2d(pos, points[i]))
        if self.lookahead_distance_m <= 0.0:
            index = min(nearest + self.lookahead_points, len(points) - 1)
            return points[index], nearest

        travelled = 0.0
        index = nearest
        while index + 1 < len(points) and travelled < self.lookahead_distance_m:
            travelled += dist2d(points[index], points[index + 1])
            index += 1
        return points[index], nearest

    @staticmethod
    def remaining_path_length(points, start_index):
        return sum(
            dist2d(points[index], points[index + 1])
            for index in range(start_index, len(points) - 1)
        )

    def update_active_path(self, pos):
        """Adopt PF-RRT* paths at a controlled rate instead of every callback."""
        if (
            not self.path_points_are_usable(self.latest_path_points)
            or self.last_valid_path_wall is None
        ):
            return
        if (
            self.active_path_source_wall is not None
            and self.last_valid_path_wall <= self.active_path_source_wall
        ):
            return

        candidate = self.orient_path_from_robot(self.latest_path_points, pos)
        candidate_span = self.remaining_path_length(candidate, 0)
        if candidate_span < 0.05:
            return
        if dist2d(candidate[0], pos) > self.path_origin_max_error_m:
            self.path_origin_rejection_count += 1
            rospy.logwarn_throttle(
                2.0,
                "Rejecting PF-RRT* path: origin is %.2f m from robot (limit %.2f m)",
                dist2d(candidate[0], pos), self.path_origin_max_error_m,
            )
            return

        now = time.time()
        reason = "initial"
        if self.active_path_points and self.active_path_wall is not None:
            age = now - self.active_path_wall
            if age < self.path_switch_min_interval_s:
                return
            active_target, active_nearest = self.path_tracking_target(
                self.active_path_points, pos
            )
            candidate_target, _ = self.path_tracking_target(candidate, pos)
            active_remaining = self.remaining_path_length(
                self.active_path_points, active_nearest
            )
            cross_track = min(dist2d(pos, point) for point in self.active_path_points)
            active_heading = math.atan2(
                active_target[1] - pos[1], active_target[0] - pos[0]
            )
            candidate_heading = math.atan2(
                candidate_target[1] - pos[1], candidate_target[0] - pos[0]
            )
            heading_delta = abs(math.atan2(
                math.sin(candidate_heading - active_heading),
                math.cos(candidate_heading - active_heading),
            ))

            forced = False
            if active_remaining <= self.active_path_end_distance_m:
                reason = "active_path_end"
                forced = True
            elif cross_track >= self.active_path_max_cross_track_m:
                reason = "cross_track"
                forced = True
            elif age >= self.path_max_hold_s:
                reason = "max_hold"
                forced = True
            else:
                return

            if (
                not forced
                and heading_delta > math.radians(self.path_switch_max_heading_deg)
            ):
                return
            if (
                reason == "max_hold"
                and heading_delta > math.radians(self.path_switch_max_heading_deg)
                and cross_track < self.active_path_max_cross_track_m
                and active_remaining > self.active_path_end_distance_m
            ):
                rospy.logwarn_throttle(
                    2.0,
                    "Deferring discontinuous path switch: heading jump %.1f deg",
                    math.degrees(heading_delta),
                )
                return

        self.active_path_points = candidate
        self.active_path_wall = now
        self.active_path_source_wall = self.last_valid_path_wall
        self.accepted_path_count += 1
        rospy.loginfo(
            "Accepted stabilized path (%s): %d points, length %.2f m",
            reason, len(candidate), candidate_span,
        )

    def publish_path_follow_cmd(self, pos, yaw, goal):
        if self.safety_stop_latched:
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            return cmd
        if not self.enable_cmd_follower:
            return None

        cmd = Twist()
        self.update_active_path(pos)
        if not self.active_path_points:
            self.cmd_pub.publish(cmd)
            return cmd

        points = self.active_path_points
        target, _ = self.path_tracking_target(points, pos)
        if dist2d(pos, goal) < self.goal_slowdown_radius_m:
            target = goal

        target_yaw = math.atan2(target[1] - pos[1], target[0] - pos[0])
        forward_yaw_error = math.atan2(
            math.sin(target_yaw - yaw), math.cos(target_yaw - yaw)
        )
        # Hysteresis prevents rapidly replanned rolling paths from alternating
        # between forward and reverse on every callback.
        if self.allow_reverse_following:
            if (
                self.follow_drive_direction > 0.0
                and abs(forward_yaw_error) >= math.radians(self.reverse_heading_threshold_deg)
            ):
                self.follow_drive_direction = -1.0
            elif (
                self.follow_drive_direction < 0.0
                and abs(forward_yaw_error) <= math.radians(self.reverse_exit_threshold_deg)
            ):
                self.follow_drive_direction = 1.0
        else:
            self.follow_drive_direction = 1.0

        drive_direction = self.follow_drive_direction
        yaw_error = forward_yaw_error
        if drive_direction < 0.0:
            reverse_yaw = target_yaw + math.pi
            yaw_error = math.atan2(math.sin(reverse_yaw - yaw), math.cos(reverse_yaw - yaw))

        abs_yaw_error = abs(yaw_error)
        goal_scale = min(1.0, max(0.25, dist2d(pos, goal) / self.goal_slowdown_radius_m))

        # Enter/exit hysteresis makes turning a state rather than a one-cycle
        # decision. While turning, keep linear velocity at zero so the rolling
        # lookahead bearing cannot jump across the forward/reverse boundary.
        if self.follow_turning_in_place:
            if abs_yaw_error <= math.radians(self.rotate_in_place_exit_threshold_deg):
                self.follow_turning_in_place = False
                rospy.loginfo(
                    "Heading aligned to %.1f deg; resuming path translation",
                    math.degrees(yaw_error),
                )
        elif abs_yaw_error >= math.radians(self.rotate_in_place_threshold_deg):
            self.follow_turning_in_place = True
            rospy.loginfo(
                "Large heading error %.1f deg; rotating in place before translation",
                math.degrees(yaw_error),
            )

        if self.follow_turning_in_place:
            cmd.linear.x = 0.0
        else:
            speed_scale = max(0.25, math.cos(yaw_error))
            linear_speed = max(
                self.min_follow_linear_speed,
                self.max_linear_speed * speed_scale * goal_scale,
            )
            cmd.linear.x = drive_direction * min(linear_speed, self.max_linear_speed)

        angular_cmd = self.heading_gain * yaw_error
        if abs_yaw_error > math.radians(3.0):
            minimum_angular_speed = (
                self.min_turn_in_place_angular_speed
                if self.follow_turning_in_place
                else self.min_follow_angular_speed
            )
            angular_cmd = math.copysign(
                max(abs(angular_cmd), minimum_angular_speed), angular_cmd
            )
        angular_limit = (
            self.max_moving_angular_speed if abs(cmd.linear.x) > 1e-3
            else self.max_angular_speed
        )
        cmd.angular.z = max(-angular_limit, min(angular_limit, angular_cmd))
        self.cmd_pub.publish(cmd)
        rospy.loginfo_throttle(
            2.0,
            "Path follower: goal_dist=%.2f target=[%.2f, %.2f] "
            "yaw_error=%.1fdeg raw_error=%.1fdeg direction=%s motion=%s "
            "cmd=[%.2f, %.2f]",
            dist2d(pos, goal), target[0], target[1], math.degrees(yaw_error),
            math.degrees(forward_yaw_error),
            "reverse" if drive_direction < 0.0 else "forward",
            "turn_in_place" if self.follow_turning_in_place else "tracking",
            cmd.linear.x, cmd.angular.z,
        )
        return cmd

    def rotate_relative(self, angle_deg, stop_when=None):
        """Rotate in place using measured yaw, robustly accumulating across +/-pi."""
        if self.pose is None or abs(angle_deg) < self.heading_recovery_tolerance_deg:
            return True, False

        direction = 1.0 if angle_deg > 0.0 else -1.0
        target = math.radians(abs(angle_deg))
        tolerance = math.radians(self.heading_recovery_tolerance_deg)
        _, _, previous_yaw = quaternion_to_rpy(self.pose.orientation)
        accumulated = 0.0
        deadline = time.time() + self.heading_recovery_rotation_timeout_s
        rate = rospy.Rate(20)
        cmd = Twist()
        cmd.angular.z = direction * min(self.heading_recovery_speed, self.max_angular_speed)

        while not rospy.is_shutdown() and time.time() < deadline:
            if self.safety_stop_latched:
                self.cmd_pub.publish(Twist())
                return False, False
            if stop_when is not None and stop_when():
                self.cmd_pub.publish(Twist())
                return False, True
            if self.pose is None:
                rate.sleep()
                continue
            _, _, yaw = quaternion_to_rpy(self.pose.orientation)
            delta = math.atan2(math.sin(yaw - previous_yaw), math.cos(yaw - previous_yaw))
            accumulated += direction * delta
            previous_yaw = yaw
            if accumulated >= target - tolerance:
                self.cmd_pub.publish(Twist())
                return True, False
            self.cmd_pub.publish(cmd)
            rate.sleep()

        self.cmd_pub.publish(Twist())
        return False, False

    def recover_heading_for_projection(self, goal, attempt):
        angle_deg = self.heading_recovery_angles_deg[attempt]
        rospy.logwarn(
            "No valid path for %.1f s; heading recovery %d/%d: rotate %+.1f deg",
            self.heading_recovery_trigger_s,
            attempt + 1,
            len(self.heading_recovery_angles_deg),
            angle_deg,
        )
        started = time.time()
        recovery_started_wall = started

        def recovery_path_ready():
            return self.has_new_usable_path(recovery_started_wall)

        completed, path_found = self.rotate_relative(
            angle_deg, stop_when=recovery_path_ready
        )
        if not path_found:
            path_found = self.wait_interruptibly(
                self.heading_recovery_settle_s, recovery_path_ready
            )
        if path_found:
            rospy.loginfo(
                "Planning succeeded during heading recovery; stopping rotation "
                "and following the new global/rolling path"
            )
        else:
            self.latest_path_points = []
            self.publish_goal(goal)
        return time.time() - started, completed, path_found

    def run_episode(self, episode_id, task, run_id, do_warmup=True):
        self.formal_episode_metric_start = None
        if not self.prepare_episode_start(task["start"]):
            row = {
                "episode_id": episode_id,
                "task_id": task["id"],
                "direction": task.get("direction", "forward"),
                "run_id": run_id,
                "method": self.method,
                "start_x": task["start"][0],
                "start_y": task["start"][1],
                "goal_x": task["goal"][0],
                "goal_y": task["goal"][1],
                "success": 0,
                "failure_reason": (
                    self.last_start_setup_failure_reason
                    or "start_setup_failure"
                ),
                "failure_stage": "start_setup",
                "eligible_for_success_rate": 0,
                "time_to_goal": "",
                "path_length": "0.000",
                "planned_path_length": "",
                "min_distance_to_pit": "",
                "min_distance_to_column": "",
                "mean_fused_risk": "",
                "max_fused_risk": "",
                "replan_count": 0,
                "path_origin_rejection_count": self.path_origin_rejection_count,
                "goal_compensation_refresh_count": self.goal_compensation_refresh_count,
                "control_goal_mismatch_count": self.control_goal_mismatch_count,
                "max_tilt_violation_duration_s": "",
            }
            return self.add_start_setup_failure_evidence(row)
        if do_warmup and not self.warm_up_planner(task["start"]):
            return {
                "episode_id": episode_id,
                "task_id": task["id"],
                "direction": task.get("direction", "forward"),
                "run_id": run_id,
                "method": self.method,
                "start_x": task["start"][0],
                "start_y": task["start"][1],
                "goal_x": task["goal"][0],
                "goal_y": task["goal"][1],
                "success": 0,
                "failure_reason": "warmup_not_at_start",
                "failure_stage": "warmup",
                "eligible_for_success_rate": 0,
                "time_to_goal": "",
                "path_length": "0.000",
                "planned_path_length": "",
                "min_distance_to_pit": "",
                "min_distance_to_column": "",
                "mean_fused_risk": "",
                "max_fused_risk": "",
                "replan_count": 0,
                "path_origin_rejection_count": self.path_origin_rejection_count,
            }
        if not self.check_pre_episode_health():
            return {
                "episode_id": episode_id,
                "task_id": task["id"],
                "direction": task.get("direction", "forward"),
                "run_id": run_id,
                "method": self.method,
                "start_x": task["start"][0],
                "start_y": task["start"][1],
                "goal_x": task["goal"][0],
                "goal_y": task["goal"][1],
                "success": 0,
                "failure_reason": "session_health_unfit",
                "failure_stage": "session_health",
                "eligible_for_success_rate": 0,
                "time_to_goal": "",
                "path_length": "0.000",
                "planned_path_length": "",
                "min_distance_to_pit": "",
                "min_distance_to_column": "",
                "mean_fused_risk": "",
                "max_fused_risk": "",
                "replan_count": 0,
                "path_origin_rejection_count": self.path_origin_rejection_count,
            }
        # Do not let the final path from repositioning/warmup drive the formal
        # episode before PF-RRT* responds to the evaluation goal.
        self.reset_episode_state()
        self.episode_goal = list(task["goal"])
        self.formal_episode_metric_start = self.runtime_metric_snapshot()
        start_wall = self.publish_goal(task["goal"], begin_episode=True)
        no_path_since = start_wall
        recovery_count = 0
        heading_recovery_time = 0.0
        goal_compensation_refresh_time = 0.0
        actual_points = []
        actual_path_length = 0.0
        risk_values = []
        min_pit = float("inf")
        min_column = float("inf")
        failure_reason = ""
        success = False
        goal_entered_wall = None
        goal_hold_s = float(self.criteria.get("goal_hold_s", 1.0))
        max_abs_roll_deg = 0.0
        max_abs_pitch_deg = 0.0
        max_body_up_tilt_deg = 0.0
        max_tilt_sample = {}
        tilt_violation_started_wall = None
        max_tilt_violation_duration_s = 0.0
        mismatch_started_wall = None
        mismatch_reference_distance = None
        mismatch_forced_without_progress = 0
        physical_anchor = None
        physical_anchor_wall = start_wall
        physical_stuck_recovery_count = 0
        offset_start = self.tf_gazebo_offset_m
        offset_max = self.tf_gazebo_offset_m
        offset_start_sample = self.localization_drift_sample()
        severe_drift_started_wall = None

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.update_control_pose()
            wall_elapsed = time.time() - start_wall
            elapsed = wall_elapsed - heading_recovery_time - goal_compensation_refresh_time
            if elapsed >= self.criteria["timeout_s"]:
                failure_reason = "timeout"
                break
            metric_pose = self.ground_truth_pose
            if self.pose is None or metric_pose is None:
                rate.sleep()
                continue

            control_pos = (self.pose.position.x, self.pose.position.y)
            ground_truth_pos = (metric_pose.position.x, metric_pose.position.y)
            if actual_points:
                actual_path_length += dist2d(actual_points[-1], ground_truth_pos)
            actual_points.append(ground_truth_pos)
            pit_d, column_d = self.distances_to_hazards(ground_truth_pos)
            min_pit = min(min_pit, pit_d)
            min_column = min(min_column, column_d)
            risk_values.extend(self.nearby_risk_values(control_pos))

            if self.tf_gazebo_offset_m is not None:
                offset_max = max(
                    self.tf_gazebo_offset_m,
                    offset_max if offset_max is not None else 0.0,
                )
            severe_drift_active = (
                self.abort_session_on_severe_localization_drift
                and self.control_pose_source == "tf"
                and self.severe_localization_drift_fail_m > 0.0
                and self.tf_gazebo_offset_m is not None
                and self.tf_gazebo_offset_m
                > self.severe_localization_drift_fail_m
            )
            if severe_drift_active:
                if severe_drift_started_wall is None:
                    severe_drift_started_wall = time.time()
                    rospy.logwarn(
                        "Severe TF/Gazebo offset %.2f m exceeds %.2f m; "
                        "requiring %.1f s persistence before ending the session",
                        self.tf_gazebo_offset_m,
                        self.severe_localization_drift_fail_m,
                        self.severe_localization_drift_hold_s,
                    )
                severe_drift_duration = time.time() - severe_drift_started_wall
                if severe_drift_duration >= self.severe_localization_drift_hold_s:
                    failure_reason = "session_localization_failure"
                    self.request_session_restart(
                        "formal-episode TF/Gazebo offset remained above {:.2f} m "
                        "for {:.1f} s (current {:.2f} m)".format(
                            self.severe_localization_drift_fail_m,
                            severe_drift_duration,
                            self.tf_gazebo_offset_m,
                        )
                    )
                    break
            elif severe_drift_started_wall is not None:
                rospy.loginfo(
                    "Severe TF/Gazebo offset recovered below %.2f m after %.2f s",
                    self.severe_localization_drift_fail_m,
                    time.time() - severe_drift_started_wall,
                )
                severe_drift_started_wall = None
            if self.localization_drift_exceeded():
                failure_reason = "localization_drift"
                rospy.logerr(
                    "Ending episode: TF/Gazebo offset %.2f m exceeded %.2f m",
                    self.tf_gazebo_offset_m, self.localization_drift_fail_m,
                )
                break

            roll, pitch, yaw = quaternion_to_rpy(metric_pose.orientation)
            roll_signed_deg = math.degrees(roll)
            pitch_signed_deg = math.degrees(pitch)
            roll_deg = abs(roll_signed_deg)
            pitch_deg = abs(pitch_signed_deg)
            body_tilt_deg = body_up_tilt_deg(metric_pose.orientation)
            tilt_deg = max(roll_deg, pitch_deg)
            if not max_tilt_sample or tilt_deg > max_tilt_sample["tilt_deg"]:
                max_tilt_sample = {
                    "tilt_deg": tilt_deg,
                    "x_m": metric_pose.position.x,
                    "y_m": metric_pose.position.y,
                    "z_m": metric_pose.position.z,
                    "roll_deg_signed": roll_signed_deg,
                    "pitch_deg_signed": pitch_signed_deg,
                    "body_up_tilt_deg": body_tilt_deg,
                    "episode_distance_m": actual_path_length,
                    "elapsed_s": wall_elapsed,
                }
            max_abs_roll_deg = max(max_abs_roll_deg, roll_deg)
            max_abs_pitch_deg = max(max_abs_pitch_deg, pitch_deg)
            max_body_up_tilt_deg = max(max_body_up_tilt_deg, body_tilt_deg)
            if self.safety_stop_latched or body_tilt_deg >= self.severe_tilt_fail_deg:
                self.latch_severe_tilt(metric_pose, "formal navigation")
                roll_deg = max(roll_deg, self.safety_stop_roll_deg)
                pitch_deg = max(pitch_deg, self.safety_stop_pitch_deg)
                body_tilt_deg = max(
                    body_tilt_deg, self.safety_stop_body_up_tilt_deg
                )
                max_abs_roll_deg = max(max_abs_roll_deg, roll_deg)
                max_abs_pitch_deg = max(max_abs_pitch_deg, pitch_deg)
                max_body_up_tilt_deg = max(max_body_up_tilt_deg, body_tilt_deg)
                safety_sample = self.safety_stop_sample
                safety_tilt_deg = max(
                    abs(float(safety_sample.get("roll_deg_signed", 0.0))),
                    abs(float(safety_sample.get("pitch_deg_signed", 0.0))),
                )
                if safety_sample and (
                    not max_tilt_sample
                    or safety_tilt_deg >= max_tilt_sample["tilt_deg"]
                ):
                    max_tilt_sample = dict(safety_sample)
                    max_tilt_sample.update({
                        "tilt_deg": safety_tilt_deg,
                        "episode_distance_m": actual_path_length,
                        "elapsed_s": wall_elapsed,
                    })
                failure_reason = "robot_overturned"
                rospy.logerr(
                    "Ending episode and batch immediately: body-up tilt %.1f deg "
                    "(roll=%.1f pitch=%.1f) reached %.1f deg emergency limit",
                    body_tilt_deg,
                    roll_deg,
                    pitch_deg,
                    self.severe_tilt_fail_deg,
                )
                break

            physical_distance = dist2d(ground_truth_pos, task["goal"])
            tilt_unsafe = tilt_deg > self.criteria["roll_pitch_fail_deg"]
            pit_unsafe = self.pits and pit_d <= self.criteria["pit_margin_m"]
            column_unsafe = (
                self.columns and column_d <= self.criteria["column_margin_m"]
            )
            if physical_distance <= self.criteria["goal_tolerance_m"]:
                self.cmd_pub.publish(Twist())
                if goal_entered_wall is None:
                    goal_entered_wall = time.time()
                if tilt_unsafe:
                    if tilt_violation_started_wall is None:
                        tilt_violation_started_wall = time.time()
                    max_tilt_violation_duration_s = max(
                        max_tilt_violation_duration_s,
                        time.time() - tilt_violation_started_wall,
                    )
                if time.time() - goal_entered_wall >= goal_hold_s:
                    if tilt_unsafe or pit_unsafe or column_unsafe:
                        failure_reason = "goal_reached_unsafe"
                        rospy.logerr(
                            "Physical goal reached but safety criterion failed: "
                            "tilt=%.1f deg pit=%.2f m column=%.2f m",
                            tilt_deg,
                            pit_d,
                            column_d,
                        )
                    else:
                        success = True
                    break
                rate.sleep()
                continue
            goal_entered_wall = None

            if tilt_deg > self.criteria["roll_pitch_fail_deg"]:
                if tilt_violation_started_wall is None:
                    tilt_violation_started_wall = time.time()
                    rospy.logwarn(
                        "Tilt %.1f deg exceeds %.1f deg limit; stopping and "
                        "requiring %.1f s sustained violation before failure",
                        tilt_deg,
                        self.criteria["roll_pitch_fail_deg"],
                        self.tilt_fail_hold_s,
                    )
                violation_duration = time.time() - tilt_violation_started_wall
                max_tilt_violation_duration_s = max(
                    max_tilt_violation_duration_s, violation_duration
                )
                self.cmd_pub.publish(Twist())
                if violation_duration >= self.tilt_fail_hold_s:
                    failure_reason = "excessive_tilt"
                    rospy.logerr(
                        "Ending episode: tilt remained excessive for %.2f s "
                        "(roll=%.1f deg pitch=%.1f deg, limit %.1f deg)",
                        violation_duration,
                        roll_deg,
                        pitch_deg,
                        self.criteria["roll_pitch_fail_deg"],
                    )
                    break
                rate.sleep()
                continue
            if tilt_violation_started_wall is not None:
                violation_duration = time.time() - tilt_violation_started_wall
                max_tilt_violation_duration_s = max(
                    max_tilt_violation_duration_s, violation_duration
                )
                rospy.loginfo(
                    "Tilt recovered below %.1f deg after %.2f s; continuing episode",
                    self.criteria["roll_pitch_fail_deg"],
                    violation_duration,
                )
                tilt_violation_started_wall = None
            if self.pits and pit_d <= self.criteria["pit_margin_m"]:
                failure_reason = "pit_zone"
                rospy.logerr(
                    "Ending episode: pit clearance %.2f m reached %.2f m margin",
                    pit_d, self.criteria["pit_margin_m"],
                )
                break
            if self.columns and column_d <= self.criteria["column_margin_m"]:
                failure_reason = "column_zone"
                rospy.logerr(
                    "Ending episode: column clearance %.2f m reached %.2f m margin",
                    column_d, self.criteria["column_margin_m"],
                )
                break

            refresh_started_wall = time.time()
            refresh_status = self.refresh_compensated_goal(
                task["goal"], "frame drift during formal navigation"
            )
            if refresh_status == "refreshed":
                goal_compensation_refresh_time += time.time() - refresh_started_wall
                mismatch_started_wall = None
                physical_anchor = ground_truth_pos
                physical_anchor_wall = time.time()
                no_path_since = time.time()
                rate.sleep()
                continue

            control_distance = dist2d(
                control_pos, self.last_published_goal or task["goal"]
            )
            mismatch = (
                self.compensate_goals_with_ground_truth
                and control_distance <= self.control_goal_tolerance_m
                and physical_distance
                >= self.control_goal_mismatch_physical_error_m
            )
            if mismatch:
                self.cmd_pub.publish(Twist())
                if (
                    mismatch_reference_distance is None
                    or physical_distance
                    <= mismatch_reference_distance
                    - self.goal_compensation_forced_min_progress_m
                ):
                    mismatch_reference_distance = physical_distance
                    mismatch_forced_without_progress = 0
                if mismatch_started_wall is None:
                    mismatch_started_wall = time.time()
                    self.control_goal_mismatch_count += 1
                    rospy.logwarn(
                        "Control/physical goal mismatch: PUTN distance %.2f m, "
                        "physical distance %.2f m; waiting %.1f s before refresh",
                        control_distance,
                        physical_distance,
                        self.control_goal_mismatch_hold_s,
                    )
                if (
                    time.time() - mismatch_started_wall
                    >= self.control_goal_mismatch_hold_s
                ):
                    if (
                        mismatch_forced_without_progress
                        >= self.goal_compensation_max_forced_refreshes
                    ):
                        failure_reason = "goal_frame_divergence"
                        rospy.logerr(
                            "Ending episode: control/physical mismatch persisted "
                            "through %d forced refreshes without %.2f m physical progress",
                            mismatch_forced_without_progress,
                            self.goal_compensation_forced_min_progress_m,
                        )
                        break
                    refresh_started_wall = time.time()
                    refresh_status = self.refresh_compensated_goal(
                        task["goal"],
                        "control goal reached but physical goal not reached",
                        force=True,
                    )
                    goal_compensation_refresh_time += (
                        time.time() - refresh_started_wall
                    )
                    mismatch_started_wall = None
                    if refresh_status == "refreshed":
                        mismatch_forced_without_progress += 1
                        physical_anchor = ground_truth_pos
                        physical_anchor_wall = time.time()
                        no_path_since = time.time()
                        rate.sleep()
                        continue
                rate.sleep()
                continue
            mismatch_started_wall = None

            if physical_anchor is None:
                physical_anchor = ground_truth_pos
                physical_anchor_wall = time.time()
            elif (
                dist2d(ground_truth_pos, physical_anchor)
                >= self.physical_stuck_displacement_m
            ):
                physical_anchor = ground_truth_pos
                physical_anchor_wall = time.time()

            path_is_fresh = self.path_is_recent(self.warmup_path_stale_s)
            has_followable_path = path_is_fresh or self.active_path_is_recent()
            if has_followable_path:
                no_path_since = None
            elif no_path_since is None:
                no_path_since = time.time()

            if no_path_since is not None and time.time() - no_path_since >= self.heading_recovery_trigger_s:
                can_recover = (
                    self.enable_heading_recovery
                    and recovery_count < len(self.heading_recovery_angles_deg)
                )
                if not can_recover:
                    failure_reason = "no_path_after_heading_recovery"
                    break
                spent, completed, path_found = self.recover_heading_for_projection(
                    task["goal"], recovery_count
                )
                heading_recovery_time += spent
                recovery_count += 1
                no_path_since = None if path_found else time.time()
                physical_anchor = ground_truth_pos
                physical_anchor_wall = time.time()
                if not completed:
                    if not path_found:
                        rospy.logwarn(
                            "Heading recovery rotation timed out; continuing with next planning attempt"
                        )
                continue
            _, _, yaw = quaternion_to_rpy(self.pose.orientation)
            cmd = self.publish_path_follow_cmd(
                control_pos, yaw, self.last_published_goal or task["goal"]
            )
            has_followable_path = path_is_fresh or self.active_path_is_recent()
            translating = cmd is not None and abs(cmd.linear.x) >= float(
                self.criteria.get("stuck_speed_threshold_mps", 0.03)
            )
            if not translating:
                # Turning in place (or waiting for a path) is intentional and
                # must not consume the physical-translation stuck timer.
                physical_anchor = ground_truth_pos
                physical_anchor_wall = time.time()
            if (
                has_followable_path
                and translating
                and physical_anchor is not None
                and time.time() - physical_anchor_wall >= self.physical_stuck_hold_s
            ):
                refresh_started_wall = time.time()
                if (
                    physical_stuck_recovery_count
                    >= self.physical_stuck_max_recoveries
                ):
                    refresh_status = "limit"
                else:
                    refresh_status = self.refresh_compensated_goal(
                        task["goal"],
                        "no physical displacement despite followable path",
                        force=True,
                    )
                    if refresh_status == "refreshed":
                        physical_stuck_recovery_count += 1
                goal_compensation_refresh_time += (
                    time.time() - refresh_started_wall
                )
                if refresh_status == "refreshed":
                    physical_anchor = ground_truth_pos
                    physical_anchor_wall = time.time()
                    no_path_since = time.time()
                    rate.sleep()
                    continue
                failure_reason = "stuck"
                rospy.logerr(
                    "Ending episode: physical displacement stayed below %.2f m "
                    "for %.1f s despite a followable path",
                    self.physical_stuck_displacement_m,
                    self.physical_stuck_hold_s,
                )
                break
            rate.sleep()

        self.cmd_pub.publish(Twist())
        if not success and not failure_reason:
            failure_reason = "interrupted" if rospy.is_shutdown() else "unknown_failure"
        duration = time.time() - start_wall
        online_route_length_estimate = None
        if self.pose is not None and self.active_path_points:
            control_pos = (self.pose.position.x, self.pose.position.y)
            _, nearest = self.path_tracking_target(self.active_path_points, control_pos)
            remaining = (
                dist2d(control_pos, self.active_path_points[nearest])
                + self.remaining_path_length(self.active_path_points, nearest)
            )
            online_route_length_estimate = self.episode_distance_m + remaining
        final_error = ""
        final_ground_truth_sample = {}
        offset_final_sample = self.localization_drift_sample()
        if self.ground_truth_pose is not None:
            final_roll, final_pitch, final_yaw = quaternion_to_rpy(
                self.ground_truth_pose.orientation
            )
            final_ground_truth_sample = {
                "x_m": self.ground_truth_pose.position.x,
                "y_m": self.ground_truth_pose.position.y,
                "z_m": self.ground_truth_pose.position.z,
                "roll_deg_signed": math.degrees(final_roll),
                "pitch_deg_signed": math.degrees(final_pitch),
                "yaw_deg": math.degrees(final_yaw),
                "body_up_tilt_deg": body_up_tilt_deg(
                    self.ground_truth_pose.orientation
                ),
            }
            final_error = "{:.3f}".format(
                dist2d(
                    (
                        self.ground_truth_pose.position.x,
                        self.ground_truth_pose.position.y,
                    ),
                    task["goal"],
                )
            )
        failure_stage = "navigation"
        eligible_for_success_rate = 1
        if (
            not success
            and failure_reason in ("goal_frame_divergence", "localization_drift")
            and self.compensate_goals_with_ground_truth
        ):
            # In algorithm-isolated mode this is a localization/evaluator frame
            # failure, not evidence that PUTN failed to find or follow a route.
            failure_stage = "localization_compensation"
            eligible_for_success_rate = 0
        elif not success and failure_reason == "session_localization_failure":
            # This is an eligible end-to-end failure: the requested navigation
            # had already started and the localization stack degraded while it
            # was running.  It must remain in the success-rate denominator,
            # after which the campaign resumes from a clean simulator session.
            failure_stage = "localization"
        elif not success and failure_reason in (
            "goal_reached_unsafe",
            "excessive_tilt",
            "pit_zone",
            "column_zone",
            "robot_overturned",
        ):
            failure_stage = "safety"
        if not success and final_ground_truth_sample:
            rospy.logerr(
                "Failure evidence: reason=%s Gazebo=(%.2f, %.2f, %.2f) "
                "roll=%.1f pitch=%.1f body_tilt=%.1f deg; max tilt at "
                "Gazebo=(%.2f, %.2f, %.2f) after %.2f m / %.2f s",
                failure_reason,
                final_ground_truth_sample["x_m"],
                final_ground_truth_sample["y_m"],
                final_ground_truth_sample["z_m"],
                final_ground_truth_sample["roll_deg_signed"],
                final_ground_truth_sample["pitch_deg_signed"],
                final_ground_truth_sample["body_up_tilt_deg"],
                max_tilt_sample.get("x_m", float("nan")),
                max_tilt_sample.get("y_m", float("nan")),
                max_tilt_sample.get("z_m", float("nan")),
                max_tilt_sample.get("episode_distance_m", float("nan")),
                max_tilt_sample.get("elapsed_s", float("nan")),
            )
        return {
            "episode_id": episode_id,
            "task_id": task["id"],
            "direction": task.get("direction", "forward"),
            "run_id": run_id,
            "method": self.method,
            "start_x": task["start"][0],
            "start_y": task["start"][1],
            "goal_x": task["goal"][0],
            "goal_y": task["goal"][1],
            "success": int(success),
            "failure_reason": "" if success else failure_reason,
            "failure_stage": failure_stage,
            "eligible_for_success_rate": eligible_for_success_rate,
            "time_to_goal": "{:.3f}".format(duration) if success else "",
            "navigation_time_excluding_recovery": "{:.3f}".format(
                max(
                    duration
                    - heading_recovery_time
                    - goal_compensation_refresh_time,
                    0.0,
                )
            ),
            "elapsed_s": "{:.3f}".format(duration),
            "planning_latency_s": (
                "{:.3f}".format(max(self.first_valid_path_wall - start_wall, 0.0))
                if self.first_valid_path_wall is not None else ""
            ),
            "final_position_error_m": final_error,
            "heading_recovery_count": recovery_count,
            "heading_recovery_time": "{:.3f}".format(heading_recovery_time),
            "goal_compensation_refresh_time": "{:.3f}".format(
                goal_compensation_refresh_time
            ),
            "max_abs_roll_deg": "{:.3f}".format(max_abs_roll_deg),
            "max_abs_pitch_deg": "{:.3f}".format(max_abs_pitch_deg),
            "max_tilt_deg": "{:.3f}".format(max(max_abs_roll_deg, max_abs_pitch_deg)),
            "max_body_up_tilt_deg": "{:.3f}".format(max_body_up_tilt_deg),
            "max_tilt_violation_duration_s": "{:.3f}".format(
                max_tilt_violation_duration_s
            ),
            "final_ground_truth_x_m": (
                "{:.3f}".format(final_ground_truth_sample["x_m"])
                if final_ground_truth_sample else ""
            ),
            "final_ground_truth_y_m": (
                "{:.3f}".format(final_ground_truth_sample["y_m"])
                if final_ground_truth_sample else ""
            ),
            "final_ground_truth_z_m": (
                "{:.3f}".format(final_ground_truth_sample["z_m"])
                if final_ground_truth_sample else ""
            ),
            "failure_ground_truth_x_m": (
                "{:.3f}".format(final_ground_truth_sample["x_m"])
                if not success and final_ground_truth_sample else ""
            ),
            "failure_ground_truth_y_m": (
                "{:.3f}".format(final_ground_truth_sample["y_m"])
                if not success and final_ground_truth_sample else ""
            ),
            "failure_ground_truth_z_m": (
                "{:.3f}".format(final_ground_truth_sample["z_m"])
                if not success and final_ground_truth_sample else ""
            ),
            "failure_ground_truth_yaw_deg": (
                "{:.3f}".format(final_ground_truth_sample["yaw_deg"])
                if not success and final_ground_truth_sample else ""
            ),
            "failure_roll_deg_signed": (
                "{:.3f}".format(final_ground_truth_sample["roll_deg_signed"])
                if not success and final_ground_truth_sample else ""
            ),
            "failure_pitch_deg_signed": (
                "{:.3f}".format(final_ground_truth_sample["pitch_deg_signed"])
                if not success and final_ground_truth_sample else ""
            ),
            "failure_body_up_tilt_deg": (
                "{:.3f}".format(final_ground_truth_sample["body_up_tilt_deg"])
                if not success and final_ground_truth_sample else ""
            ),
            "failure_episode_distance_m": (
                "{:.3f}".format(actual_path_length) if not success else ""
            ),
            "max_tilt_ground_truth_x_m": (
                "{:.3f}".format(max_tilt_sample["x_m"])
                if max_tilt_sample else ""
            ),
            "max_tilt_ground_truth_y_m": (
                "{:.3f}".format(max_tilt_sample["y_m"])
                if max_tilt_sample else ""
            ),
            "max_tilt_ground_truth_z_m": (
                "{:.3f}".format(max_tilt_sample["z_m"])
                if max_tilt_sample else ""
            ),
            "max_tilt_roll_deg_signed": (
                "{:.3f}".format(max_tilt_sample["roll_deg_signed"])
                if max_tilt_sample else ""
            ),
            "max_tilt_pitch_deg_signed": (
                "{:.3f}".format(max_tilt_sample["pitch_deg_signed"])
                if max_tilt_sample else ""
            ),
            "max_tilt_body_up_tilt_deg_at_pose": (
                "{:.3f}".format(max_tilt_sample["body_up_tilt_deg"])
                if max_tilt_sample else ""
            ),
            "max_tilt_episode_distance_m": (
                "{:.3f}".format(max_tilt_sample["episode_distance_m"])
                if max_tilt_sample else ""
            ),
            "max_tilt_elapsed_s": (
                "{:.3f}".format(max_tilt_sample["elapsed_s"])
                if max_tilt_sample else ""
            ),
            "tf_gazebo_offset_start_m": (
                "{:.3f}".format(offset_start) if offset_start is not None else ""
            ),
            "tf_gazebo_offset_max_m": (
                "{:.3f}".format(offset_max) if offset_max is not None else ""
            ),
            "tf_gazebo_offset_final_m": (
                "{:.3f}".format(self.tf_gazebo_offset_m)
                if self.tf_gazebo_offset_m is not None else ""
            ),
            "tf_gazebo_dx_start_m": offset_start_sample.get("dx_m", ""),
            "tf_gazebo_dy_start_m": offset_start_sample.get("dy_m", ""),
            "tf_gazebo_dyaw_start_deg": offset_start_sample.get("dyaw_deg", ""),
            "tf_gazebo_dx_final_m": offset_final_sample.get("dx_m", ""),
            "tf_gazebo_dy_final_m": offset_final_sample.get("dy_m", ""),
            "tf_gazebo_dyaw_final_deg": offset_final_sample.get("dyaw_deg", ""),
            "path_length": "{:.3f}".format(actual_path_length),
            "planned_path_length": (
                "{:.3f}".format(self.first_goal_path_length)
                if self.first_goal_path_length is not None else ""
            ),
            "online_route_length_estimate_m": (
                "{:.3f}".format(online_route_length_estimate)
                if online_route_length_estimate is not None else ""
            ),
            "min_distance_to_pit": "{:.3f}".format(min_pit) if self.pits else "",
            "min_distance_to_column": "{:.3f}".format(min_column) if self.columns else "",
            "mean_fused_risk": "{:.3f}".format(sum(risk_values) / len(risk_values)) if risk_values else "",
            "max_fused_risk": "{:.3f}".format(max(risk_values)) if risk_values else "",
            "valid_path_update_count": self.valid_path_messages,
            "accepted_path_count": self.accepted_path_count,
            "replan_count": max(self.accepted_path_count - 1, 0),
            "path_origin_rejection_count": self.path_origin_rejection_count,
            "goal_compensation_refresh_count": self.goal_compensation_refresh_count,
            "goal_compensation_proactive_refresh_count": (
                self.goal_compensation_proactive_refresh_count
            ),
            "goal_compensation_forced_refresh_count": (
                self.goal_compensation_forced_refresh_count
            ),
            "control_goal_mismatch_count": self.control_goal_mismatch_count,
        }

    @staticmethod
    def format_metric(value):
        return "{:.3f}".format(max(float(value), 0.0))

    def add_runtime_metrics_to_row(self, row, attempt_start, attempt_end):
        """Attach session/campaign exposure fields to every attempt row."""
        formal_start = self.formal_episode_metric_start
        metric_start = formal_start if formal_start is not None else attempt_start
        metric_end = attempt_end
        campaign_start = self.campaign_metric_snapshot(metric_start)
        campaign_end = self.campaign_metric_snapshot(metric_end)
        row.update({
            "campaign_session_id": self.campaign_session_count,
            "session_attempt_id": self.session_attempt_count,
            "session_elapsed_start_s": self.format_metric(
                metric_start["active_elapsed_s"]
            ),
            "session_elapsed_end_s": self.format_metric(
                metric_end["active_elapsed_s"]
            ),
            "campaign_active_elapsed_start_s": self.format_metric(
                campaign_start["active_elapsed_s"]
            ),
            "campaign_active_elapsed_end_s": self.format_metric(
                campaign_end["active_elapsed_s"]
            ),
            "session_ground_truth_distance_start_m": self.format_metric(
                metric_start["ground_truth_distance_m"]
            ),
            "session_ground_truth_distance_end_m": self.format_metric(
                metric_end["ground_truth_distance_m"]
            ),
            "campaign_ground_truth_distance_start_m": self.format_metric(
                campaign_start["ground_truth_distance_m"]
            ),
            "campaign_ground_truth_distance_end_m": self.format_metric(
                campaign_end["ground_truth_distance_m"]
            ),
            "session_elevation_gain_start_m": self.format_metric(
                metric_start["elevation_gain_m"]
            ),
            "session_elevation_gain_end_m": self.format_metric(
                metric_end["elevation_gain_m"]
            ),
            "campaign_elevation_gain_start_m": self.format_metric(
                campaign_start["elevation_gain_m"]
            ),
            "campaign_elevation_gain_end_m": self.format_metric(
                campaign_end["elevation_gain_m"]
            ),
        })
        if formal_start is None:
            row.update({
                "episode_ground_truth_distance_m": "",
                "episode_elevation_gain_m": "",
                "episode_elevation_loss_m": "",
                "episode_vertical_travel_m": "",
                "tf_gazebo_offset_time_mean_m": "",
                "tf_gazebo_offset_time_rms_m": "",
                "tf_gazebo_offset_above_threshold_s": "",
            })
            return

        row.update({
            "episode_ground_truth_distance_m": self.format_metric(
                metric_end["ground_truth_distance_m"]
                - formal_start["ground_truth_distance_m"]
            ),
            "episode_elevation_gain_m": self.format_metric(
                metric_end["elevation_gain_m"] - formal_start["elevation_gain_m"]
            ),
            "episode_elevation_loss_m": self.format_metric(
                metric_end["elevation_loss_m"] - formal_start["elevation_loss_m"]
            ),
            "episode_vertical_travel_m": self.format_metric(
                metric_end["vertical_travel_m"] - formal_start["vertical_travel_m"]
            ),
            "tf_gazebo_offset_above_threshold_s": self.format_metric(
                metric_end["drift_above_threshold_s"]
                - formal_start["drift_above_threshold_s"]
            ),
        })
        observed_s = (
            metric_end["drift_observed_s"] - formal_start["drift_observed_s"]
        )
        if observed_s > 1e-6:
            integral = (
                metric_end["drift_integral_m_s"]
                - formal_start["drift_integral_m_s"]
            )
            squared_integral = (
                metric_end["drift_squared_integral_m2_s"]
                - formal_start["drift_squared_integral_m2_s"]
            )
            row["tf_gazebo_offset_time_mean_m"] = self.format_metric(
                integral / observed_s
            )
            row["tf_gazebo_offset_time_rms_m"] = self.format_metric(
                math.sqrt(max(squared_integral / observed_s, 0.0))
            )
        else:
            row["tf_gazebo_offset_time_mean_m"] = ""
            row["tf_gazebo_offset_time_rms_m"] = ""

    @staticmethod
    def percentile(values, fraction):
        if not values:
            return None
        ordered = sorted(values)
        index = (len(ordered) - 1) * fraction
        low = int(math.floor(index))
        high = int(math.ceil(index))
        if low == high:
            return ordered[low]
        return ordered[low] + (ordered[high] - ordered[low]) * (index - low)

    def summarize_rows(self, rows):
        target_by_group = {
            item["group"]: int(item["target_eligible_samples"])
            for item in self.sampling_report.get("directions", [])
        }
        overall_target = sum(target_by_group.values())
        groups = {"overall": rows}
        for row in rows:
            key = "{}:{}".format(row["task_id"], row.get("direction", "forward"))
            groups.setdefault(key, []).append(row)
            session_id = row.get("campaign_session_id", "")
            if session_id not in ("", None):
                groups.setdefault("session:{}".format(session_id), []).append(row)

        summaries = []
        for group, records in groups.items():
            items = [
                item for item in records
                if int(item.get("eligible_for_success_rate", 1)) == 1
            ]
            excluded_items = [
                item for item in records
                if int(item.get("eligible_for_success_rate", 1)) != 1
            ]
            excluded_failure_counts = {}
            setup_failure_counts = {}
            for item in excluded_items:
                reason = item.get("failure_reason") or "unknown_excluded_failure"
                excluded_failure_counts[reason] = (
                    excluded_failure_counts.get(reason, 0) + 1
                )
                if item.get("failure_stage") in ("start_setup", "warmup"):
                    setup_failure_counts[reason] = (
                        setup_failure_counts.get(reason, 0) + 1
                    )
            successes = [item for item in items if int(item["success"]) == 1]
            failures = [item for item in items if int(item["success"]) != 1]
            half_index = len(items) // 2
            first_half = items[:half_index]
            second_half = items[half_index:]

            def failure_rate(records_subset):
                if not records_subset:
                    return ""
                failed = sum(
                    1 for record in records_subset
                    if int(record.get("success", 0)) != 1
                )
                return round(float(failed) / len(records_subset), 4)

            def numeric_mean(records_subset, field):
                values = []
                for record in records_subset:
                    value = record.get(field, "")
                    if value in ("", None):
                        continue
                    try:
                        values.append(float(value))
                    except (TypeError, ValueError):
                        continue
                return round(statistics.mean(values), 3) if values else ""
            target_episodes = (
                overall_target if group == "overall" else target_by_group.get(group, "")
            )
            sample_shortfall = (
                max(int(target_episodes) - len(items), 0)
                if target_episodes != "" else ""
            )
            wall_times = [
                float(item["time_to_goal"])
                for item in successes if item["time_to_goal"] != ""
            ]
            nav_times = [
                float(item["navigation_time_excluding_recovery"])
                for item in successes
                if item.get("navigation_time_excluding_recovery", "") != ""
            ]
            planning_times = [
                float(item["planning_latency_s"])
                for item in items if item.get("planning_latency_s", "") != ""
            ]
            failure_counts = {}
            for item in items:
                reason = (
                    "success" if int(item["success"]) == 1
                    else item["failure_reason"] or "unknown_failure"
                )
                failure_counts[reason] = failure_counts.get(reason, 0) + 1
            summaries.append({
                "group": group,
                "records": len(records),
                "target_episodes": target_episodes,
                "episodes": len(items),
                "sample_shortfall": sample_shortfall,
                "quota_complete": (
                    int(sample_shortfall == 0) if sample_shortfall != "" else ""
                ),
                "excluded_records": len(excluded_items),
                "excluded_setup_records": sum(setup_failure_counts.values()),
                "excluded_failure_counts": excluded_failure_counts,
                "setup_failure_counts": setup_failure_counts,
                "successes": len(successes),
                "success_rate": round(float(len(successes)) / len(items), 4) if items else "",
                "first_half_episodes": len(first_half),
                "first_half_failure_rate": failure_rate(first_half),
                "second_half_episodes": len(second_half),
                "second_half_failure_rate": failure_rate(second_half),
                "start_drift_mean_success_m": numeric_mean(
                    successes, "tf_gazebo_offset_start_m"
                ),
                "start_drift_mean_failure_m": numeric_mean(
                    failures, "tf_gazebo_offset_start_m"
                ),
                "episode_distance_mean_success_m": numeric_mean(
                    successes, "episode_ground_truth_distance_m"
                ),
                "episode_distance_mean_failure_m": numeric_mean(
                    failures, "episode_ground_truth_distance_m"
                ),
                "elevation_gain_mean_success_m": numeric_mean(
                    successes, "episode_elevation_gain_m"
                ),
                "elevation_gain_mean_failure_m": numeric_mean(
                    failures, "episode_elevation_gain_m"
                ),
                "wall_time_mean_s": round(statistics.mean(wall_times), 3) if wall_times else "",
                "navigation_time_mean_s": round(statistics.mean(nav_times), 3) if nav_times else "",
                "navigation_time_median_s": round(statistics.median(nav_times), 3) if nav_times else "",
                "navigation_time_p90_s": round(self.percentile(nav_times, 0.90), 3) if nav_times else "",
                "planning_latency_mean_s": round(statistics.mean(planning_times), 3) if planning_times else "",
                "failure_counts": failure_counts,
            })
        return summaries

    def write_reports(self, output_dir, stem, rows, warmup_report):
        summary = self.summarize_rows(rows)
        summary_fields = [
            "group", "records", "target_episodes", "episodes",
            "sample_shortfall", "quota_complete", "excluded_records",
            "excluded_setup_records", "excluded_failure_counts", "setup_failure_counts",
            "successes", "success_rate",
            "first_half_episodes", "first_half_failure_rate",
            "second_half_episodes", "second_half_failure_rate",
            "start_drift_mean_success_m", "start_drift_mean_failure_m",
            "episode_distance_mean_success_m", "episode_distance_mean_failure_m",
            "elevation_gain_mean_success_m", "elevation_gain_mean_failure_m",
            "wall_time_mean_s",
            "navigation_time_mean_s", "navigation_time_median_s",
            "navigation_time_p90_s", "planning_latency_mean_s", "failure_counts",
        ]
        summary_path = os.path.join(output_dir, stem + "_summary.csv")
        with open(summary_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=summary_fields)
            writer.writeheader()
            for item in summary:
                row = dict(item)
                row["failure_counts"] = json.dumps(row["failure_counts"], sort_keys=True)
                row["excluded_failure_counts"] = json.dumps(
                    row["excluded_failure_counts"], sort_keys=True
                )
                row["setup_failure_counts"] = json.dumps(
                    row["setup_failure_counts"], sort_keys=True
                )
                writer.writerow(row)

        json_path = os.path.join(output_dir, stem + "_summary.json")
        with open(json_path, "w") as f:
            json.dump(
                {
                    "scene": self.scene,
                    "method": self.method,
                    "use_lesta": self.use_lesta,
                    "lesta_fusion": {
                        "model_path": self.lesta_model_path,
                        "model_sha256": self.lesta_model_sha256,
                        "map_source": self.lesta_map_source,
                        "map_topic": self.lesta_map_topic,
                        "probability_layer": self.lesta_probability_layer,
                        "binary_layer": self.lesta_binary_layer,
                        "putn_weight": self.lesta_putn_weight,
                        "lesta_weight": self.lesta_weight,
                        "unknown_penalty": self.lesta_unknown_penalty,
                        "risk_threshold": self.lesta_risk_threshold,
                        "fusion_policy": self.lesta_fusion_policy,
                        "readiness_required": self.require_lesta_ready,
                        "readiness_confirmed": self.lesta_map_ready,
                        "readiness_layer": self.lesta_ready_layer,
                    },
                    "goal_frame_compensation_enabled": self.compensate_goals_with_ground_truth,
                    "abort_on_localization_drift": self.abort_on_localization_drift,
                    "campaign": {
                        "campaign_id": getattr(self, "campaign_id", ""),
                        "checkpoint_path": getattr(
                            self, "campaign_checkpoint_path", ""
                        ),
                        "session_count": getattr(
                            self, "campaign_session_count", 1
                        ),
                        "resumed": getattr(self, "campaign_resumed", False),
                        "seed_csv": self.campaign_seed_csv,
                        "session_direction_order_policy": (
                            self.session_direction_order_policy
                        ),
                        "session_first_direction_group": getattr(
                            self, "session_first_direction_group", ""
                        ),
                        "max_episode_attempts_per_session": (
                            self.max_episode_attempts_per_session
                        ),
                        "safety_stop_latched": self.safety_stop_latched,
                        "session_restart_requested": self.session_restart_requested,
                        "terminal_reason": (
                            self.safety_stop_reason
                            if self.safety_stop_latched
                            else self.session_restart_reason
                        ),
                        "cumulative_metrics": self.campaign_metric_snapshot(),
                    },
                    "sampling": self.sampling_report,
                    "failure_policy": {
                        "roll_pitch_fail_deg": self.criteria["roll_pitch_fail_deg"],
                        "navigation_timeout_s": self.navigation_timeout_s,
                        "tilt_fail_hold_s": self.tilt_fail_hold_s,
                        "severe_tilt_fail_deg": self.severe_tilt_fail_deg,
                        "severe_tilt_policy": (
                            "latch robot_overturned, save the campaign checkpoint, and "
                            "exit the launch for a full simulator restart"
                        ),
                        "abort_session_on_severe_localization_drift": (
                            self.abort_session_on_severe_localization_drift
                        ),
                        "severe_localization_drift_fail_m": (
                            self.severe_localization_drift_fail_m
                        ),
                        "severe_localization_drift_hold_s": (
                            self.severe_localization_drift_hold_s
                        ),
                        "severe_localization_drift_policy": (
                            "count the active navigation as an eligible end-to-end "
                            "failure, checkpoint, and restart the simulator"
                        ),
                        "control_goal_tolerance_m": self.control_goal_tolerance_m,
                        "control_goal_mismatch_physical_error_m": (
                            self.control_goal_mismatch_physical_error_m
                        ),
                        "control_goal_mismatch_hold_s": (
                            self.control_goal_mismatch_hold_s
                        ),
                        "physical_stuck_displacement_m": (
                            self.physical_stuck_displacement_m
                        ),
                        "physical_stuck_hold_s": self.physical_stuck_hold_s,
                        "physical_stuck_max_recoveries": (
                            self.physical_stuck_max_recoveries
                        ),
                        "goal_compensation_refresh_distance_m": (
                            self.goal_compensation_refresh_distance_m
                        ),
                        "goal_compensation_refresh_yaw_deg": (
                            self.goal_compensation_refresh_yaw_deg
                        ),
                        "goal_compensation_refresh_yaw_policy": (
                            "diagnostic_only_not_a_refresh_trigger"
                        ),
                        "goal_compensation_refresh_min_interval_s": (
                            self.goal_compensation_refresh_min_interval_s
                        ),
                        "goal_compensation_max_refreshes": (
                            self.goal_compensation_max_refreshes
                        ),
                        "goal_compensation_max_forced_refreshes": (
                            self.goal_compensation_max_forced_refreshes
                        ),
                        "goal_compensation_forced_min_progress_m": (
                            self.goal_compensation_forced_min_progress_m
                        ),
                        "pre_episode_health_timeout_s": (
                            self.pre_episode_health_timeout_s
                        ),
                        "pre_episode_health_hold_s": self.pre_episode_health_hold_s,
                        "pre_episode_max_tilt_deg": self.pre_episode_max_tilt_deg,
                        "pre_episode_max_tf_gazebo_offset_m": (
                            self.pre_episode_max_tf_gazebo_offset_m
                        ),
                        "pre_episode_max_sensor_age_s": (
                            self.pre_episode_max_sensor_age_s
                        ),
                        "post_episode_health_check": (
                            self.post_episode_health_check
                        ),
                        "abort_on_start_setup_failure": (
                            self.abort_on_start_setup_failure
                        ),
                        "lesta_map_max_age_s": self.lesta_map_max_age_s,
                        "max_consecutive_planning_failures_before_restart": (
                            self.max_consecutive_planning_failures_before_restart
                        ),
                    },
                    "warmup": warmup_report,
                    "metric_definitions": {
                        "planned_path_length": (
                            "distance already travelled plus the first published PF-RRT* "
                            "path whose endpoint reaches the requested goal"
                        ),
                        "online_route_length_estimate_m": (
                            "distance travelled plus remaining length of the final accepted path"
                        ),
                        "navigation_time_mean_s": (
                            "mean successful navigation_time_excluding_recovery; excludes "
                            "heading recovery and compensated-goal publish time"
                        ),
                        "wall_time_mean_s": (
                            "mean successful time_to_goal including evaluator recovery overhead"
                        ),
                        "valid_path_update_count": "non-empty PF-RRT* path messages received",
                        "accepted_path_count": "stabilized paths adopted by the command follower",
                        "replan_count": "accepted_path_count minus the initial accepted path",
                        "eligible_for_success_rate": (
                            "1 only after the requested episode start was physically verified; "
                            "setup/warmup failures and compensated-mode localization/frame "
                            "failures are excluded from algorithm success rate, except a "
                            "persistent severe in-episode localization failure which remains "
                            "an eligible end-to-end failure"
                        ),
                        "runs_sampling_policy": (
                            "runs is the eligible-sample quota per direction; both successful "
                            "and failed eligible navigation episodes fill the quota, while "
                            "eligible_for_success_rate=0 records trigger a bounded replacement"
                        ),
                        "campaign_resume_policy": (
                            "an incomplete matching campaign reloads prior rows, repeats "
                            "warmup after restart, and runs only missing eligible samples"
                        ),
                        "session_isolation_policy": (
                            "max_episode_attempts_per_session=1 checkpoints and exits after "
                            "one warmed attempt; 0 keeps continuous endurance behavior"
                        ),
                        "attempt_id": (
                            "all attempts in one task direction, including excluded replacements"
                        ),
                        "run_id": (
                            "eligible sample slot within one direction; it is repeated when an "
                            "excluded attempt is automatically replaced"
                        ),
                        "tf_gazebo_offset_m": (
                            "2-D difference between PUTN's /world->/base_link TF pose and "
                            "Gazebo ground truth"
                        ),
                        "tf_gazebo_offset_time_mean_m": (
                            "time-weighted mean TF/Gazebo offset during formal navigation"
                        ),
                        "tf_gazebo_offset_time_rms_m": (
                            "time-weighted RMS TF/Gazebo offset during formal navigation"
                        ),
                        "episode_elevation_gain_m": (
                            "positive Gazebo-Z travel accumulated during formal navigation"
                        ),
                        "campaign_session_id": (
                            "full-simulator session number within a resumed campaign"
                        ),
                        "campaign_active_elapsed_s": (
                            "sum of evaluator wall-active time across resumed sessions; "
                            "downtime between launches is excluded"
                        ),
                        "session_health_policy": (
                            "stale pose/TF/LESTA data, excessive boundary tilt or excessive "
                            "boundary TF/Gazebo offset is excluded, checkpointed, and requests "
                            "a full simulator restart"
                        ),
                        "max_tilt_violation_duration_s": (
                            "longest continuous roll/pitch threshold violation; brief "
                            "spikes shorter than tilt_fail_hold_s do not fail an episode"
                        ),
                        "max_body_up_tilt_deg": (
                            "maximum angle between robot body Z and world Z; used for the "
                            "Euler-singularity-safe irreversible rollover decision"
                        ),
                        "failure_ground_truth_pose": (
                            "Gazebo ground-truth XYZ, yaw, signed roll/pitch and body-up "
                            "tilt sampled when an unsuccessful formal episode ends"
                        ),
                        "failure_episode_distance_m": (
                            "10 Hz Gazebo-ground-truth planar distance accumulated before "
                            "an unsuccessful formal episode ends"
                        ),
                        "max_tilt_ground_truth_pose": (
                            "Gazebo ground-truth XYZ and signed roll/pitch at the largest "
                            "absolute roll-or-pitch sample in the formal episode"
                        ),
                        "max_tilt_episode_distance_m": (
                            "10 Hz Gazebo-ground-truth planar distance accumulated when "
                            "the maximum tilt sample occurred"
                        ),
                        "max_tilt_elapsed_s": (
                            "formal-episode wall time when the maximum tilt sample occurred"
                        ),
                        "goal_compensation_refresh_count": (
                            "total proactive plus forced compensated-goal republishes"
                        ),
                        "goal_compensation_proactive_refresh_count": (
                            "position-shift-triggered republishes subject to interval and budget"
                        ),
                        "goal_compensation_forced_refresh_count": (
                            "republishes forced by verified mismatch or physical-stuck recovery"
                        ),
                        "goal_compensation_refresh_time": (
                            "wall time spent publishing compensated goals, reported separately "
                            "from heading recovery"
                        ),
                        "control_goal_mismatch_count": (
                            "number of times PUTN reached its control goal while Gazebo "
                            "ground truth remained far from the physical goal"
                        ),
                    },
                    "summary": summary,
                },
                f, indent=2, sort_keys=True,
            )
        return summary_path, json_path

    def directional_tasks(self, task):
        start_label = str(task.get("start_label", "start"))
        goal_label = str(task.get("goal_label", "goal"))
        forward = dict(task)
        forward["direction"] = "{}_to_{}".format(start_label, goal_label)
        yield forward
        if self.bidirectional:
            reverse = dict(task)
            reverse["start"], reverse["goal"] = list(task["goal"]), list(task["start"])
            reverse["start_label"], reverse["goal_label"] = goal_label, start_label
            reverse["direction"] = "{}_to_{}".format(goal_label, start_label)
            yield reverse

    @staticmethod
    def sampling_state_group(state):
        task = state["directional_task"]
        return "{}:{}".format(task["id"], task["direction"])

    def order_sampling_states_for_session(self, states, rows, runs_per_direction):
        """Choose a fair first direction for this simulator session.

        The original fixed ordering made the forward direction the first formal
        episode after every warmup, while the reverse direction always inherited
        the extra distance and localization drift.  The balanced policy counts
        which direction was first in earlier sessions and rotates this session's
        order toward the least-used first direction.  Existing checkpoint rows
        are enough to apply the policy when an older campaign is resumed.
        """
        ordered = list(states)
        pending = [
            state for state in ordered
            if state["eligible_count"] < runs_per_direction
            and not state["replacement_limit_reached"]
        ]
        if not pending:
            self.session_first_direction_group = ""
            return ordered

        if self.session_direction_order_policy == "configured":
            first_state = pending[0]
            first_counts = {}
        elif self.session_direction_order_policy == "reverse_first":
            first_state = pending[-1]
            first_counts = {}
        else:
            first_counts = {}
            seen_sessions = set()
            for row_index, row in enumerate(rows):
                session_id = str(row.get("campaign_session_id", "")).strip()
                if not session_id:
                    # Historical seed rows may not carry a session id. Treat
                    # each row marked as first attempt as its own session.
                    if str(row.get("session_attempt_id", "")) != "1":
                        continue
                    session_id = "legacy-{}".format(row_index)
                if session_id in seen_sessions:
                    continue
                seen_sessions.add(session_id)
                group = "{}:{}".format(
                    row.get("task_id", ""), row.get("direction", "forward")
                )
                first_counts[group] = first_counts.get(group, 0) + 1

            original_index = {id(state): index for index, state in enumerate(ordered)}
            first_state = min(
                pending,
                key=lambda state: (
                    first_counts.get(self.sampling_state_group(state), 0),
                    state["eligible_count"],
                    original_index[id(state)],
                ),
            )

        first_index = ordered.index(first_state)
        ordered = ordered[first_index:] + ordered[:first_index]
        self.session_first_direction_group = self.sampling_state_group(first_state)
        rospy.loginfo(
            "Session direction order policy=%s first=%s historical_first_counts=%s",
            self.session_direction_order_policy,
            self.session_first_direction_group,
            first_counts,
        )
        return ordered

    def update_sampling_report(self, states, runs_per_direction):
        directions = []
        for state in states:
            task = state["directional_task"]
            eligible = state["eligible_count"]
            directions.append({
                "group": "{}:{}".format(task["id"], task["direction"]),
                "task_id": task["id"],
                "direction": task["direction"],
                "target_eligible_samples": runs_per_direction,
                "eligible_samples": eligible,
                "successful_samples": state["success_count"],
                "failed_eligible_samples": state["failure_count"],
                "excluded_attempts": state["excluded_count"],
                "total_attempts": state["attempt_count"],
                "sample_shortfall": max(runs_per_direction - eligible, 0),
                "quota_complete": eligible >= runs_per_direction,
                "replacement_limit_reached": state["replacement_limit_reached"],
            })
        self.sampling_report = {
            "policy": "eligible_quota_with_excluded_record_refill",
            "session_direction_order_policy": self.session_direction_order_policy,
            "session_first_direction_group": getattr(
                self, "session_first_direction_group", ""
            ),
            "max_episode_attempts_per_session": (
                self.max_episode_attempts_per_session
            ),
            "target_valid_samples_per_direction": runs_per_direction,
            "max_excluded_retries_per_direction": (
                self.max_excluded_retries_per_direction
            ),
            "eligible_failures_count_toward_quota": True,
            "directions": directions,
        }

    @staticmethod
    def record_sampling_result(state, row):
        """Update one direction's quota; eligible failures are valid samples."""
        eligible = int(row.get("eligible_for_success_rate", 1)) == 1
        if eligible:
            state["eligible_count"] += 1
            if int(row["success"]) == 1:
                state["success_count"] += 1
            else:
                state["failure_count"] += 1
        else:
            state["excluded_count"] += 1
        return eligible

    def configure_campaign(self, runs_per_direction):
        signature_payload = {
            "schema_version": 1,
            "evaluation_name": self.eval_cfg["name"],
            "scene": self.scene,
            "method": self.method,
            "use_lesta": self.use_lesta,
            "bidirectional": self.bidirectional,
            "runs_per_direction": runs_per_direction,
            "tasks": self.tasks,
            "config": self.config,
            "runtime_policy": {
                "warmup_mode": self.warmup_mode,
                "start_mode": self.start_mode,
                "navigation_timeout_s": self.navigation_timeout_s,
                "control_pose_source": self.control_pose_source,
                "compensate_goals_with_ground_truth": (
                    self.compensate_goals_with_ground_truth
                ),
                "abort_on_localization_drift": self.abort_on_localization_drift,
                "abort_on_start_setup_failure": (
                    self.abort_on_start_setup_failure
                ),
                "post_episode_health_check": self.post_episode_health_check,
                "severe_tilt_fail_deg": self.severe_tilt_fail_deg,
                "tilt_fail_hold_s": self.tilt_fail_hold_s,
                "max_linear_speed": self.max_linear_speed,
                "max_angular_speed": self.max_angular_speed,
                "enable_heading_recovery": self.enable_heading_recovery,
                "pre_episode_health_timeout_s": self.pre_episode_health_timeout_s,
                "pre_episode_health_hold_s": self.pre_episode_health_hold_s,
                "pre_episode_max_tilt_deg": self.pre_episode_max_tilt_deg,
                "pre_episode_max_tf_gazebo_offset_m": (
                    self.pre_episode_max_tf_gazebo_offset_m
                ),
                "pre_episode_max_sensor_age_s": self.pre_episode_max_sensor_age_s,
                "lesta_map_max_age_s": self.lesta_map_max_age_s,
                "max_consecutive_planning_failures_before_restart": (
                    self.max_consecutive_planning_failures_before_restart
                ),
            },
        }
        # Preserve compatibility with existing pure-PUTN checkpoints while
        # preventing different LESTA models/weights from sharing samples.
        if self.use_lesta:
            signature_payload["lesta_fusion"] = {
                "model_path": self.lesta_model_path,
                "model_sha256": self.lesta_model_sha256,
                "map_source": self.lesta_map_source,
                "map_topic": self.lesta_map_topic,
                "probability_layer": self.lesta_probability_layer,
                "binary_layer": self.lesta_binary_layer,
                "putn_weight": self.lesta_putn_weight,
                "lesta_weight": self.lesta_weight,
                "unknown_penalty": self.lesta_unknown_penalty,
                "risk_threshold": self.lesta_risk_threshold,
                "fusion_policy": self.lesta_fusion_policy,
                "readiness_required": self.require_lesta_ready,
            }
        # Keep the pre-balancing signature as a one-time compatible value so
        # an incomplete campaign created by the previous evaluator can resume.
        # Once saved again, the checkpoint carries the complete new policy.
        legacy_signature_text = json.dumps(
            signature_payload, sort_keys=True, separators=(",", ":")
        )
        legacy_signature = hashlib.sha256(
            legacy_signature_text.encode("utf-8")
        ).hexdigest()
        self.campaign_compatible_signatures = set()
        signature_payload["runtime_policy"].update({
            "session_direction_order_policy": (
                self.session_direction_order_policy
            ),
            "abort_session_on_severe_localization_drift": (
                self.abort_session_on_severe_localization_drift
            ),
            "severe_localization_drift_fail_m": (
                self.severe_localization_drift_fail_m
            ),
            "severe_localization_drift_hold_s": (
                self.severe_localization_drift_hold_s
            ),
        })
        endurance_signature_text = json.dumps(
            signature_payload, sort_keys=True, separators=(",", ":")
        )
        endurance_signature = hashlib.sha256(
            endurance_signature_text.encode("utf-8")
        ).hexdigest()
        # Only an unlimited/endurance run may migrate older checkpoints.  An
        # isolated one-attempt session must use a new campaign id so that its
        # samples can never be mixed with earlier continuous-run samples.
        if self.max_episode_attempts_per_session == 0:
            self.campaign_compatible_signatures.update({
                legacy_signature,
                endurance_signature,
            })
        signature_payload["runtime_policy"].update({
            "max_episode_attempts_per_session": (
                self.max_episode_attempts_per_session
            ),
        })
        signature_text = json.dumps(
            signature_payload, sort_keys=True, separators=(",", ":")
        )
        signature = hashlib.sha256(signature_text.encode("utf-8")).hexdigest()
        self.campaign_compatible_signatures.add(signature)
        if self.requested_campaign_id:
            safe_id = "".join(
                char if char.isalnum() or char in ("-", "_") else "_"
                for char in self.requested_campaign_id
            ).strip("_")
            if not safe_id:
                raise ValueError("campaign_id must contain a letter or digit")
        else:
            safe_id = signature[:12]
        self.campaign_id = safe_id
        self.campaign_signature = signature
        self.campaign_signature_payload = signature_payload
        self.campaign_checkpoint_path = os.path.join(
            self.output_dir,
            "{}_{}_campaign_checkpoint.json".format(
                self.eval_cfg["name"], self.campaign_id
            ),
        )
        self.campaign_started_at = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        self.campaign_session_count = 1

    def load_campaign_rows(self):
        if (
            not self.resume_incomplete_campaign
            or not os.path.exists(self.campaign_checkpoint_path)
        ):
            return []
        try:
            with open(self.campaign_checkpoint_path, "r") as f:
                checkpoint = json.load(f)
        except (OSError, ValueError) as exc:
            raise RuntimeError(
                "Unable to read campaign checkpoint {}: {}".format(
                    self.campaign_checkpoint_path, exc
                )
            )
        if checkpoint.get("complete", False):
            rospy.loginfo(
                "Previous campaign %s is complete; starting a new campaign",
                self.campaign_id,
            )
            return []
        checkpoint_signature = checkpoint.get("signature")
        if checkpoint_signature not in self.campaign_compatible_signatures:
            raise RuntimeError(
                "Incomplete campaign checkpoint signature does not match the current "
                "test policy. Use a new campaign_id, or restore the matching launch "
                "parameters before resuming: {}".format(
                    self.campaign_checkpoint_path
                )
            )
        if checkpoint_signature != self.campaign_signature:
            rospy.logwarn(
                "Migrating campaign %s checkpoint to the current endurance "
                "policy while preserving its existing samples",
                self.campaign_id,
            )
        rows = checkpoint.get("rows", [])
        if not isinstance(rows, list):
            raise RuntimeError("Campaign checkpoint rows must be a list")
        self.campaign_started_at = checkpoint.get(
            "started_at", self.campaign_started_at
        )
        saved_totals = checkpoint.get("campaign_metrics", {})
        for key in self.campaign_metric_base:
            try:
                self.campaign_metric_base[key] = float(saved_totals.get(key, 0.0))
            except (TypeError, ValueError):
                rospy.logwarn(
                    "Ignoring invalid checkpoint campaign metric %s=%r",
                    key,
                    saved_totals.get(key),
                )
        self.campaign_session_count = int(checkpoint.get("session_count", 1)) + 1
        rospy.logwarn(
            "Resuming incomplete campaign %s from %d recorded attempts; "
            "this session will repeat map warmup and fill only missing eligible samples",
            self.campaign_id,
            len(rows),
        )
        return rows

    def load_campaign_seed_rows(self):
        if not self.campaign_seed_csv:
            return []
        if not os.path.isfile(self.campaign_seed_csv):
            raise RuntimeError(
                "campaign_seed_csv does not exist: {}".format(
                    self.campaign_seed_csv
                )
            )
        with open(self.campaign_seed_csv, "r", newline="") as f:
            rows = list(csv.DictReader(f))
        if not rows:
            raise RuntimeError("campaign_seed_csv contains no episode rows")
        attempts_by_group = {}
        for row in rows:
            if row.get("method") and row["method"] != self.method:
                raise RuntimeError(
                    "Seed CSV method '{}' does not match current method '{}'".format(
                        row["method"], self.method
                    )
                )
            group = "{}:{}".format(
                row.get("task_id", ""), row.get("direction", "forward")
            )
            attempts_by_group[group] = attempts_by_group.get(group, 0) + 1
            if not row.get("attempt_id"):
                row["attempt_id"] = attempts_by_group[group]
        rospy.logwarn(
            "Seeding campaign %s from %d historical records in %s",
            self.campaign_id,
            len(rows),
            self.campaign_seed_csv,
        )
        return rows

    def save_campaign_checkpoint(self, rows, complete=False):
        campaign_metrics = self.campaign_metric_snapshot()
        checkpoint = {
            "schema_version": 2,
            "campaign_id": self.campaign_id,
            "signature": self.campaign_signature,
            "signature_payload": self.campaign_signature_payload,
            "started_at": self.campaign_started_at,
            "updated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "session_count": self.campaign_session_count,
            "session_direction_order_policy": self.session_direction_order_policy,
            "session_first_direction_group": getattr(
                self, "session_first_direction_group", ""
            ),
            "max_episode_attempts_per_session": (
                self.max_episode_attempts_per_session
            ),
            "complete": bool(complete),
            "terminal_reason": (
                self.safety_stop_reason
                if self.safety_stop_latched
                else self.session_restart_reason
            ),
            "session_restart_requested": self.session_restart_requested,
            "campaign_metrics": campaign_metrics,
            "sampling": self.sampling_report,
            "rows": rows,
        }
        temporary_path = self.campaign_checkpoint_path + ".tmp"
        with open(temporary_path, "w") as f:
            json.dump(checkpoint, f, indent=2, sort_keys=True)
            f.flush()
            os.fsync(f.fileno())
        os.replace(temporary_path, self.campaign_checkpoint_path)

    def run(self):
        if self.initial_delay_s > 0.0:
            rospy.loginfo("Waiting %.1f seconds before starting navigation evaluation", self.initial_delay_s)
            rospy.sleep(self.initial_delay_s)

        if not self.wait_for_lesta_ready():
            self.cmd_pub.publish(Twist())
            return

        if not self.verify_cmd_vel_ownership():
            self.cmd_pub.publish(Twist())
            return

        os.makedirs(self.output_dir, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        stem = "{}_{}".format(self.eval_cfg["name"], stamp)
        out_path = os.path.join(self.output_dir, stem + "_episodes.csv")
        runs_per_task = int(
            rospy.get_param("~runs", self.eval_cfg.get("runs_per_task", 1))
        )
        if runs_per_task <= 0:
            raise ValueError("runs must be a positive eligible-sample quota")

        self.configure_campaign(runs_per_task)
        campaign_checkpoint_existed = os.path.exists(
            self.campaign_checkpoint_path
        )
        rows = self.load_campaign_rows()
        if not campaign_checkpoint_existed and not rows:
            rows = self.load_campaign_seed_rows()
        self.campaign_resumed = bool(rows)

        sampling_states = []
        for task in self.tasks:
            for directional_task in self.directional_tasks(task):
                sampling_states.append({
                    "directional_task": directional_task,
                    "eligible_count": 0,
                    "success_count": 0,
                    "failure_count": 0,
                    "excluded_count": 0,
                    "attempt_count": 0,
                    "replacement_limit_reached": False,
                })
        states_by_group = {
            "{}:{}".format(
                state["directional_task"]["id"],
                state["directional_task"]["direction"],
            ): state
            for state in sampling_states
        }
        for saved_row in rows:
            group = "{}:{}".format(
                saved_row["task_id"], saved_row.get("direction", "forward")
            )
            if group not in states_by_group:
                raise RuntimeError(
                    "Campaign checkpoint contains unknown task direction '{}'".format(
                        group
                    )
                )
            state = states_by_group[group]
            state["attempt_count"] = max(
                state["attempt_count"], int(saved_row.get("attempt_id", 0))
            )
            self.record_sampling_result(state, saved_row)
        for state in sampling_states:
            state["replacement_limit_reached"] = (
                state["excluded_count"]
                > self.max_excluded_retries_per_direction
            )
        sampling_states = self.order_sampling_states_for_session(
            sampling_states, rows, runs_per_task
        )
        self.update_sampling_report(sampling_states, runs_per_task)
        pending_before_warmup = any(
            state["eligible_count"] < runs_per_task
            and not state["replacement_limit_reached"]
            for state in sampling_states
        )

        warmup_report = {"mode": self.warmup_mode, "passed": True}
        batch_warmup = (
            pending_before_warmup
            and self.warmup_once
            and self.warmup_mode != "none"
        )
        if batch_warmup:
            # Return from map coverage to the direction selected as this
            # session's first formal sample.  Otherwise a reverse-first test
            # would secretly include an unmeasured cross-map reposition first.
            first_start = next(
                state["directional_task"]["start"]
                for state in sampling_states
                if state["eligible_count"] < runs_per_task
                and not state["replacement_limit_reached"]
            )
            if self.warmup_mode == "coverage_waypoints":
                passed, warmup_report = self.run_coverage_warmup(first_start)
            else:
                started = time.time()
                if not self.prepare_episode_start(first_start):
                    passed = False
                else:
                    passed = self.warm_up_planner(first_start)
                warmup_report = {
                    "mode": self.warmup_mode,
                    "duration_s": round(time.time() - started, 3),
                    "passed": passed,
                }
            if not passed:
                rospy.logerr("Batch warmup failed; refusing to mix a cold map into evaluation results")
                self.save_campaign_checkpoint(rows, complete=False)
                self.write_reports(self.output_dir, stem, rows, warmup_report)
                return

        rospy.loginfo("Writing navigation evaluation CSV to %s", out_path)
        rospy.loginfo(
            "Evaluation scene=%s method=%s use_lesta=%s tasks=%d "
            "eligible_samples_per_direction=%d max_excluded_retries=%d "
            "max_episode_attempts_per_session=%d",
            self.scene, self.method, self.use_lesta, len(self.tasks), runs_per_task,
            self.max_excluded_retries_per_direction,
            self.max_episode_attempts_per_session,
        )
        if self.use_lesta:
            rospy.loginfo(
                "LESTA fusion source=%s model=%s sha256=%s PUTN_weight=%.3f "
                "LESTA_weight=%.3f unknown_penalty=%.3f risk_threshold=%.3f "
                "policy=%s probability_layer=%s binary_layer=%s",
                self.lesta_map_source,
                self.lesta_model_path,
                self.lesta_model_sha256[:12],
                self.lesta_putn_weight,
                self.lesta_weight,
                self.lesta_unknown_penalty,
                self.lesta_risk_threshold,
                self.lesta_fusion_policy,
                self.lesta_probability_layer,
                self.lesta_binary_layer,
            )
        abort_batch = False
        with open(out_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=self.output_fields)
            writer.writeheader()
            for saved_row in rows:
                writer.writerow({
                    field: saved_row.get(field, "") for field in self.output_fields
                })
            f.flush()
            episode_id = max(
                (int(row.get("episode_id", 0)) for row in rows), default=0
            )
            while not abort_batch:
                pending_states = [
                    state for state in sampling_states
                    if state["eligible_count"] < runs_per_task
                    and not state["replacement_limit_reached"]
                ]
                if not pending_states:
                    break

                for state in pending_states:
                    if rospy.is_shutdown():
                        abort_batch = True
                        break
                    directional_task = state["directional_task"]
                    run_id = state["eligible_count"] + 1
                    state["attempt_count"] += 1
                    attempt_id = state["attempt_count"]
                    episode_id += 1
                    self.session_attempt_count += 1
                    rospy.loginfo(
                        "Starting episode %d: %s %s sample %d/%d attempt %d",
                        episode_id,
                        directional_task["id"],
                        directional_task["direction"],
                        run_id,
                        runs_per_task,
                        attempt_id,
                    )
                    attempt_metric_start = self.runtime_metric_snapshot()
                    row = self.run_episode(
                        episode_id,
                        directional_task,
                        run_id,
                        do_warmup=not batch_warmup,
                    )
                    attempt_metric_end = self.runtime_metric_snapshot()
                    self.add_runtime_metrics_to_row(
                        row, attempt_metric_start, attempt_metric_end
                    )
                    row["attempt_id"] = attempt_id
                    writer.writerow(row)
                    rows.append(row)
                    f.flush()

                    eligible = self.record_sampling_result(state, row)
                    if row.get("failure_stage") in (
                        "navigation", "safety", "localization",
                        "localization_compensation"
                    ):
                        self.check_post_episode_health(row)
                    planning_failure_reasons = {
                        "no_path_after_heading_recovery",
                        "stuck",
                        "goal_frame_divergence",
                    }
                    if (
                        int(row.get("success", 0)) == 0
                        and row.get("failure_reason") in planning_failure_reasons
                    ):
                        self.consecutive_planning_failures += 1
                    elif eligible:
                        self.consecutive_planning_failures = 0
                    if (
                        self.max_consecutive_planning_failures_before_restart > 0
                        and self.consecutive_planning_failures
                        >= self.max_consecutive_planning_failures_before_restart
                    ):
                        self.request_session_restart(
                            "{} consecutive planning/control failures".format(
                                self.consecutive_planning_failures
                            )
                        )
                    if eligible:
                        rospy.loginfo(
                            "Episode %d counted as eligible sample %d/%d: "
                            "success=%s failure=%s path=%.3f",
                            episode_id,
                            state["eligible_count"],
                            runs_per_task,
                            row["success"],
                            row["failure_reason"],
                            float(row["path_length"]),
                        )
                    else:
                        if self.safety_stop_latched:
                            abort_batch = True
                        elif (
                            self.abort_on_start_setup_failure
                            and row.get("failure_stage") in ("start_setup", "warmup")
                        ):
                            self.request_session_restart(
                                "excluded {} failure {}; refusing another long "
                                "start reposition in the degraded session".format(
                                    row.get("failure_stage", "setup"),
                                    row.get("failure_reason", "unknown"),
                                )
                            )
                            abort_batch = True
                        elif (
                            state["excluded_count"]
                            > self.max_excluded_retries_per_direction
                        ):
                            state["replacement_limit_reached"] = True
                            rospy.logerr(
                                "%s %s remains short of its eligible-sample quota: "
                                "all %d replacement retries were also excluded "
                                "(%d excluded attempts total)",
                                directional_task["id"],
                                directional_task["direction"],
                                self.max_excluded_retries_per_direction,
                                state["excluded_count"],
                            )
                        else:
                            rospy.logwarn(
                                "Episode %d excluded (%s); automatically retrying "
                                "sample %d/%d for %s (%d/%d excluded attempts used)",
                                episode_id,
                                row["failure_reason"],
                                run_id,
                                runs_per_task,
                                directional_task["direction"],
                                state["excluded_count"],
                                self.max_excluded_retries_per_direction,
                            )
                    remaining_after_attempt = any(
                        candidate["eligible_count"] < runs_per_task
                        and not candidate["replacement_limit_reached"]
                        for candidate in sampling_states
                    )
                    if (
                        not self.safety_stop_latched
                        and not self.session_restart_requested
                        and self.max_episode_attempts_per_session > 0
                        and self.session_attempt_count
                        >= self.max_episode_attempts_per_session
                        and remaining_after_attempt
                    ):
                        self.request_session_restart(
                            "isolated evaluation completed {} episode attempt(s); "
                            "re-warm before the next sample".format(
                                self.session_attempt_count
                            ),
                            health_related=False,
                        )
                    self.update_sampling_report(sampling_states, runs_per_task)
                    self.save_campaign_checkpoint(rows, complete=False)
                    if self.safety_stop_latched:
                        self.cmd_pub.publish(Twist())
                        rospy.logfatal(
                            "Safety stop saved after episode %d; ending this launch. "
                            "Restart the simulator with the same arguments to re-warm "
                            "the map and resume the missing sample quota.",
                            episode_id,
                        )
                        abort_batch = True
                    elif self.session_restart_requested:
                        rospy.logerr(
                            "Session restart checkpoint saved after episode %d; "
                            "restart the same roslaunch command to re-warm and continue "
                            "the remaining eligible quota",
                            episode_id,
                        )
                        abort_batch = True
                    if abort_batch:
                        break
        incomplete = [
            item for item in self.sampling_report["directions"]
            if not item["quota_complete"]
        ]
        if incomplete:
            rospy.logerr(
                "Evaluation ended with eligible-sample shortfalls: %s",
                ", ".join(
                    "{}={}/{}".format(
                        item["group"],
                        item["eligible_samples"],
                        item["target_eligible_samples"],
                    )
                    for item in incomplete
                ),
            )
        else:
            rospy.loginfo(
                "Eligible-sample quota complete for all directions; formal "
                "navigation failures were retained as valid samples"
            )
        campaign_complete = not incomplete
        self.save_campaign_checkpoint(rows, complete=campaign_complete)
        summary_path, json_path = self.write_reports(self.output_dir, stem, rows, warmup_report)
        rospy.loginfo(
            "Navigation evaluation complete: episodes=%s summary=%s json=%s "
            "campaign_checkpoint=%s complete=%s",
            out_path,
            summary_path,
            json_path,
            self.campaign_checkpoint_path,
            campaign_complete,
        )


if __name__ == "__main__":
    rospy.init_node("map_test_nav_eval")
    try:
        MapTestNavEval().run()
    except rospy.ROSInterruptException:
        # Ctrl+C and roslaunch shutdown commonly interrupt a simulated-time
        # sleep.  This is an expected stop, not an evaluator failure.
        pass
    except Exception:
        # Constructor failures used to appear only as an exit-code-1 REQUIRED
        # node death in roslaunch.  Preserve the traceback in ROS logs so an
        # invalid campaign argument is diagnosable after an unattended run.
        rospy.logfatal("Navigation evaluator terminated:\n%s", traceback.format_exc())
        raise
