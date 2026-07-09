#!/usr/bin/env python3
import csv
import math
import os
import time

import rospy
import rospkg
import sensor_msgs.point_cloud2 as pc2
import tf
import yaml
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float32MultiArray


def yaw_to_quaternion(yaw):
    q = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
    return q


def quaternion_to_rpy(q):
    return tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])


def dist2d(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def rect_distance(point, center, size):
    dx = max(abs(point[0] - center[0]) - size[0] * 0.5, 0.0)
    dy = max(abs(point[1] - center[1]) - size[1] * 0.5, 0.0)
    return math.hypot(dx, dy)


def path_length(points):
    if len(points) < 2:
        return 0.0
    return sum(dist2d(points[i - 1], points[i]) for i in range(1, len(points)))


class MapTestNavEval:
    def __init__(self):
        self.config_path = rospy.get_param("~config_path", self.default_config_path())
        self.output_dir = os.path.expanduser(rospy.get_param("~output_dir", "~/.ros/putn_nav_eval"))
        self.method = rospy.get_param("~method", "lesta_putn")
        self.robot_model_name = rospy.get_param("~robot_model_name", "scout/")
        self.goal_topic = rospy.get_param("~goal_topic", "/goal")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.model_states_topic = rospy.get_param("~model_states_topic", "/gazebo/model_states")
        self.global_path_topic = rospy.get_param("~global_path_topic", "/global_planning_node/global_path")
        self.fused_cloud_topic = rospy.get_param(
            "~fused_cloud_topic", "/global_planning_node/fused_traversability_cloud"
        )
        self.start_mode = rospy.get_param("~start_mode", "navigate")
        self.reposition_timeout_s = rospy.get_param("~reposition_timeout_s", 180.0)
        self.start_tolerance_m = rospy.get_param("~start_tolerance_m", 0.6)
        self.reset_settle_s = rospy.get_param("~reset_settle_s", 3.0)
        self.goal_publish_count = rospy.get_param("~goal_publish_count", 5)
        self.pre_goal_warmup_s = rospy.get_param("~pre_goal_warmup_s", 25.0)
        self.hold_goal_clear_s = rospy.get_param("~hold_goal_clear_s", 3.0)
        self.path_grace_s = rospy.get_param("~path_grace_s", 20.0)
        self.risk_sample_radius_m = rospy.get_param("~risk_sample_radius_m", 0.75)
        self.enable_cmd_follower = rospy.get_param("~enable_cmd_follower", True)
        self.lookahead_points = int(rospy.get_param("~lookahead_points", 8))
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.45)
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 0.8)
        self.heading_gain = rospy.get_param("~heading_gain", 1.4)
        self.goal_slowdown_radius_m = rospy.get_param("~goal_slowdown_radius_m", 1.5)
        self.initial_delay_s = rospy.get_param("~initial_delay_s", 0.0)

        with open(self.config_path, "r") as f:
            self.config = yaml.safe_load(f)

        self.eval_cfg = self.config["evaluation"]
        self.criteria = self.eval_cfg["success_criteria"]
        self.tasks = self.config["tasks"]
        self.pits = self.config["map"]["pits"]
        self.columns = self.config["map"]["columns"]
        self.output_fields = self.eval_cfg["output_fields"]

        self.pose = None
        self.last_pose_time = None
        self.current_speed = 0.0
        self.latest_path_points = []
        self.path_messages = 0
        self.latest_cloud = []

        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1, latch=True)
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        rospy.Subscriber(self.model_states_topic, ModelStates, self.model_states_cb, queue_size=1)
        rospy.Subscriber(self.global_path_topic, Float32MultiArray, self.global_path_cb, queue_size=10)
        rospy.Subscriber(self.fused_cloud_topic, rospy.AnyMsg, self.fused_cloud_any_cb, queue_size=1)

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
        if self.pose is not None and self.last_pose_time is not None:
            dt = max((now - self.last_pose_time).to_sec(), 1e-3)
            self.current_speed = dist2d(
                (pose.position.x, pose.position.y), (self.pose.position.x, self.pose.position.y)
            ) / dt
        self.pose = pose
        self.last_pose_time = now

    def global_path_cb(self, msg):
        if not msg.data:
            return
        stride = 5 if len(msg.data) % 5 == 0 else 3
        points = []
        for i in range(0, len(msg.data) - stride + 1, stride):
            points.append((msg.data[i], msg.data[i + 1]))
        self.latest_path_points = points
        self.path_messages += 1

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

    def reset_episode_state(self):
        self.pose = None
        self.last_pose_time = None
        self.latest_path_points = []
        self.path_messages = 0
        self.latest_cloud = []
        self.current_speed = 0.0

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
            if self.pose is not None:
                return True
            rate.sleep()
        return False

    def publish_goal(self, goal):
        msg = PoseStamped()
        msg.header.frame_id = self.eval_cfg.get("fixed_frame", "world")
        msg.pose.position.x = goal[0]
        msg.pose.position.y = goal[1]
        msg.pose.position.z = max(goal[2], 0.1)
        q = yaw_to_quaternion(0.0)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        for _ in range(self.goal_publish_count):
            msg.header.stamp = rospy.Time.now()
            self.goal_pub.publish(msg)
            rospy.sleep(0.2)

    def warm_up_planner(self, start):
        if self.pre_goal_warmup_s <= 0.0:
            return

        rospy.loginfo(
            "Warming up PUTN planner for %.1f seconds before sending the evaluation goal",
            self.pre_goal_warmup_s,
        )
        self.publish_goal(start)
        rospy.sleep(self.hold_goal_clear_s)
        self.cmd_pub.publish(Twist())

        deadline = time.time() + self.pre_goal_warmup_s
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and time.time() < deadline:
            self.cmd_pub.publish(Twist())
            rate.sleep()
        self.reset_episode_state()

    def navigate_to_start(self, start):
        if not self.wait_for_pose():
            rospy.logwarn("No robot pose received before repositioning to start")
            return False

        rospy.loginfo(
            "Navigating to episode start [%.2f, %.2f] without Gazebo teleport",
            start[0],
            start[1],
        )
        self.reset_episode_state()
        self.publish_goal(start)

        deadline = time.time() + self.reposition_timeout_s
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and time.time() < deadline:
            if self.pose is None:
                rate.sleep()
                continue

            pos = (self.pose.position.x, self.pose.position.y)
            if dist2d(pos, start) <= self.start_tolerance_m:
                self.cmd_pub.publish(Twist())
                rospy.sleep(self.reset_settle_s)
                self.reset_episode_state()
                return True

            _, _, yaw = quaternion_to_rpy(self.pose.orientation)
            self.publish_path_follow_cmd(pos, yaw, start)
            rate.sleep()

        self.cmd_pub.publish(Twist())
        rospy.logwarn("Timed out while navigating to episode start")
        return False

    def prepare_episode_start(self, start):
        if self.start_mode == "navigate":
            return self.navigate_to_start(start)
        if self.start_mode == "gazebo":
            self.reset_episode_state()
            self.reset_robot_pose_gazebo(start)
            return True
        if self.start_mode == "none":
            if not self.wait_for_pose():
                return False
            pos = (self.pose.position.x, self.pose.position.y)
            if dist2d(pos, start) <= self.start_tolerance_m:
                self.reset_episode_state()
                return True
            rospy.logwarn(
                "Robot is %.2f m from requested start. Move it near the start or use start_mode:=navigate.",
                dist2d(pos, start),
            )
            return False
        raise ValueError("Unsupported start_mode: {}".format(self.start_mode))

    def distances_to_hazards(self, point):
        min_pit = min(rect_distance(point, pit["center"], pit["size"]) for pit in self.pits)
        min_column = min(
            max(dist2d(point, col["center"]) - col["radius_m"], 0.0) for col in self.columns
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

    def publish_path_follow_cmd(self, pos, yaw, goal):
        if not self.enable_cmd_follower:
            return

        cmd = Twist()
        if not self.latest_path_points:
            self.cmd_pub.publish(cmd)
            return

        points = self.latest_path_points
        if len(points) >= 2 and dist2d(points[0], goal) < dist2d(points[-1], goal):
            points = list(reversed(points))

        nearest = min(range(len(points)), key=lambda i: dist2d(pos, points[i]))
        target_index = min(nearest + self.lookahead_points, len(points) - 1)
        target = points[target_index]
        if dist2d(pos, goal) < self.goal_slowdown_radius_m:
            target = goal

        target_yaw = math.atan2(target[1] - pos[1], target[0] - pos[0])
        yaw_error = math.atan2(math.sin(target_yaw - yaw), math.cos(target_yaw - yaw))
        speed_scale = max(0.0, math.cos(yaw_error))
        goal_scale = min(1.0, max(0.25, dist2d(pos, goal) / self.goal_slowdown_radius_m))

        cmd.linear.x = self.max_linear_speed * speed_scale * goal_scale
        cmd.angular.z = max(
            -self.max_angular_speed,
            min(self.max_angular_speed, self.heading_gain * yaw_error),
        )
        self.cmd_pub.publish(cmd)

    def run_episode(self, episode_id, task, run_id):
        if not self.prepare_episode_start(task["start"]):
            return {
                "episode_id": episode_id,
                "task_id": task["id"],
                "run_id": run_id,
                "method": self.method,
                "success": 0,
                "failure_reason": "timeout",
                "time_to_goal": "0.000",
                "path_length": "0.000",
                "planned_path_length": "0.000",
                "min_distance_to_pit": "",
                "min_distance_to_column": "",
                "mean_fused_risk": "",
                "max_fused_risk": "",
                "replan_count": 0,
            }
        self.warm_up_planner(task["start"])
        self.publish_goal(task["goal"])

        start_wall = time.time()
        last_moving_wall = start_wall
        actual_points = []
        risk_values = []
        min_pit = float("inf")
        min_column = float("inf")
        failure_reason = ""
        success = False

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            elapsed = time.time() - start_wall
            if self.pose is None:
                if elapsed > self.criteria["timeout_s"]:
                    failure_reason = "timeout"
                    break
                rate.sleep()
                continue

            pos = (self.pose.position.x, self.pose.position.y)
            actual_points.append(pos)
            pit_d, column_d = self.distances_to_hazards(pos)
            min_pit = min(min_pit, pit_d)
            min_column = min(min_column, column_d)
            risk_values.extend(self.nearby_risk_values(pos))

            roll, pitch, _ = quaternion_to_rpy(self.pose.orientation)
            if math.degrees(max(abs(roll), abs(pitch))) > self.criteria["roll_pitch_fail_deg"]:
                failure_reason = "collision"
                break
            if pit_d <= self.criteria["pit_margin_m"]:
                failure_reason = "pit"
                break
            if column_d <= self.criteria["column_margin_m"]:
                failure_reason = "collision"
                break
            if dist2d(pos, task["goal"]) <= self.criteria["goal_tolerance_m"]:
                success = True
                break
            if elapsed > self.path_grace_s and self.path_messages == 0:
                failure_reason = "no_path"
                break
            _, _, yaw = quaternion_to_rpy(self.pose.orientation)
            self.publish_path_follow_cmd(pos, yaw, task["goal"])
            if self.current_speed >= self.criteria["stuck_speed_threshold_mps"]:
                last_moving_wall = time.time()
            if (
                self.path_messages > 0
                and time.time() - last_moving_wall >= self.criteria["stuck_duration_s"]
            ):
                failure_reason = "stuck"
                break
            if elapsed >= self.criteria["timeout_s"]:
                failure_reason = "timeout"
                break
            rate.sleep()

        self.cmd_pub.publish(Twist())
        duration = time.time() - start_wall
        return {
            "episode_id": episode_id,
            "task_id": task["id"],
            "run_id": run_id,
            "method": self.method,
            "success": int(success),
            "failure_reason": "" if success else failure_reason,
            "time_to_goal": "{:.3f}".format(duration if success else 0.0),
            "path_length": "{:.3f}".format(path_length(actual_points)),
            "planned_path_length": "{:.3f}".format(path_length(self.latest_path_points)),
            "min_distance_to_pit": "{:.3f}".format(min_pit),
            "min_distance_to_column": "{:.3f}".format(min_column),
            "mean_fused_risk": "{:.3f}".format(sum(risk_values) / len(risk_values)) if risk_values else "",
            "max_fused_risk": "{:.3f}".format(max(risk_values)) if risk_values else "",
            "replan_count": max(self.path_messages - 1, 0),
        }

    def run(self):
        if self.initial_delay_s > 0.0:
            rospy.loginfo("Waiting %.1f seconds before starting navigation evaluation", self.initial_delay_s)
            rospy.sleep(self.initial_delay_s)

        os.makedirs(self.output_dir, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        out_path = os.path.join(self.output_dir, "{}_{}.csv".format(self.eval_cfg["name"], stamp))
        runs_per_task = int(self.eval_cfg["runs_per_task"])

        rospy.loginfo("Writing navigation evaluation CSV to %s", out_path)
        with open(out_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=self.output_fields)
            writer.writeheader()
            episode_id = 0
            for task in self.tasks:
                for run_id in range(1, runs_per_task + 1):
                    if rospy.is_shutdown():
                        return
                    episode_id += 1
                    rospy.loginfo("Starting episode %d: %s run %d", episode_id, task["id"], run_id)
                    row = self.run_episode(episode_id, task, run_id)
                    writer.writerow(row)
                    f.flush()
                    rospy.loginfo(
                        "Episode %d done: success=%s failure=%s path=%.3f",
                        episode_id,
                        row["success"],
                        row["failure_reason"],
                        float(row["path_length"]),
                    )
        rospy.loginfo("Navigation evaluation complete: %s", out_path)


if __name__ == "__main__":
    rospy.init_node("map_test_nav_eval")
    MapTestNavEval().run()
