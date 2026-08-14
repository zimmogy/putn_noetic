#!/usr/bin/env python3
import csv
import math
import os
import time

import rospy
import tf
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


def dist2d(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def quaternion_to_rpy(q):
    return tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])


class ManualGoalEval:
    def __init__(self):
        self.goal = (
            float(rospy.get_param("~goal_x")),
            float(rospy.get_param("~goal_y")),
            float(rospy.get_param("~goal_z", 0.0)),
        )
        self.goal_topic = rospy.get_param("~goal_topic", "/goal")
        self.model_states_topic = rospy.get_param("~model_states_topic", "/gazebo/model_states")
        self.robot_model_name = rospy.get_param("~robot_model_name", "scout/")
        self.fixed_frame = rospy.get_param("~fixed_frame", "world")
        self.goal_tolerance_m = float(rospy.get_param("~goal_tolerance_m", 0.6))
        self.roll_pitch_fail_deg = float(rospy.get_param("~roll_pitch_fail_deg", 45.0))
        self.timeout_s = float(rospy.get_param("~timeout_s", 180.0))
        self.goal_publish_count = int(rospy.get_param("~goal_publish_count", 5))
        self.auto_manage_controller = rospy.get_param("~auto_manage_controller", True)
        self.controller_mode_topic = rospy.get_param("~controller_mode_topic", "/controller/mode")
        self.controller_mode_cmd_topic = rospy.get_param(
            "~controller_mode_cmd_topic", "/controller/mode_cmd"
        )
        self.controller_mode_wait_s = float(rospy.get_param("~controller_mode_wait_s", 3.0))
        self.require_manual_before_exit = rospy.get_param("~require_manual_before_exit", True)
        self.output_dir = os.path.expanduser(
            rospy.get_param("~output_dir", "~/.ros/putn_manual_goal_eval")
        )
        self.task_id = rospy.get_param("~task_id", "manual_goal")
        self.method = rospy.get_param("~method", "manual")

        self.pose = None
        self.start_pos = None
        self.min_goal_distance = float("inf")
        self.max_tilt_deg = 0.0
        self.controller_mode = ""
        self.initial_controller_mode = ""
        self.final_controller_mode = ""

        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1, latch=True)
        self.controller_mode_cmd_pub = rospy.Publisher(
            self.controller_mode_cmd_topic, String, queue_size=1, latch=True
        )
        rospy.Subscriber(self.model_states_topic, ModelStates, self.model_states_cb, queue_size=1)
        rospy.Subscriber(self.controller_mode_topic, String, self.controller_mode_cb, queue_size=1)

    def model_states_cb(self, msg):
        names_to_try = [self.robot_model_name, self.robot_model_name.rstrip("/"), "scout", "scout/"]
        index = None
        for name in names_to_try:
            if name in msg.name:
                index = msg.name.index(name)
                break
        if index is None:
            return
        self.pose = msg.pose[index]

    def controller_mode_cb(self, msg):
        self.controller_mode = msg.data.strip().lower()

    def wait_for_pose(self, timeout_s=10.0):
        deadline = time.time() + timeout_s
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and time.time() < deadline:
            if self.pose is not None:
                return True
            rate.sleep()
        return False

    def wait_for_controller_mode(self, expected_mode, timeout_s=None):
        timeout = self.controller_mode_wait_s if timeout_s is None else timeout_s
        deadline = time.time() + timeout
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and time.time() < deadline:
            if self.controller_mode == expected_mode:
                return True
            rate.sleep()
        return False

    def request_controller_mode(self, mode):
        if not self.auto_manage_controller:
            return False
        rospy.loginfo("Requesting controller mode: %s", mode)
        for _ in range(5):
            self.controller_mode_cmd_pub.publish(String(data=mode))
            rospy.sleep(0.1)
        if self.wait_for_controller_mode(mode):
            rospy.loginfo("Controller mode is now %s", mode)
            return True
        rospy.logwarn(
            "Controller mode did not report '%s' within %.1f seconds. "
            "If the controller node is not running or is an older version, switch mode manually.",
            mode,
            self.controller_mode_wait_s,
        )
        return False

    def ensure_manual_before_exit(self):
        if not self.auto_manage_controller:
            rospy.logwarn(
                "auto_manage_controller is false; manual_goal_eval will not switch controller mode before exit"
            )
            return False

        if not self.require_manual_before_exit:
            return self.request_controller_mode("manual")

        rospy.loginfo("Switching controller to manual mode before finishing evaluation")
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.controller_mode_cmd_pub.publish(String(data="manual"))
            if self.wait_for_controller_mode("manual", timeout_s=0.5):
                rospy.loginfo("Controller confirmed manual mode; evaluation can finish")
                return True
            rospy.logwarn("Waiting for controller to switch to manual mode before exit...")
            rate.sleep()
        return False

    def publish_goal(self):
        msg = PoseStamped()
        msg.header.frame_id = self.fixed_frame
        msg.pose.position.x = self.goal[0]
        msg.pose.position.y = self.goal[1]
        msg.pose.position.z = max(self.goal[2], 0.1)
        msg.pose.orientation.w = 1.0

        for _ in range(self.goal_publish_count):
            msg.header.stamp = rospy.Time.now()
            self.goal_pub.publish(msg)
            rospy.sleep(0.2)

    def write_result(self, row):
        os.makedirs(self.output_dir, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        out_path = os.path.join(self.output_dir, "manual_goal_eval_{}.csv".format(stamp))
        fields = [
            "task_id",
            "method",
            "goal_x",
            "goal_y",
            "goal_z",
            "start_x",
            "start_y",
            "start_z",
            "success",
            "failure_reason",
            "time_to_goal",
            "min_goal_distance",
            "tipped",
            "max_tilt_deg",
            "initial_controller_mode",
            "final_controller_mode",
        ]
        with open(out_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            writer.writerow(row)
        rospy.loginfo("Manual goal evaluation CSV written to %s", out_path)

    def run(self):
        if not self.wait_for_pose():
            raise RuntimeError("No robot pose received from {}".format(self.model_states_topic))

        self.start_pos = (
            self.pose.position.x,
            self.pose.position.y,
            self.pose.position.z,
        )
        self.initial_controller_mode = self.controller_mode
        rospy.loginfo(
            "Publishing manual goal [%.2f, %.2f, %.2f] on %s",
            self.goal[0],
            self.goal[1],
            self.goal[2],
            self.goal_topic,
        )
        self.publish_goal()
        self.request_controller_mode("auto")

        start_wall = time.time()
        success = False
        tipped = False
        failure_reason = "timeout"
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            elapsed = time.time() - start_wall
            if self.pose is None:
                rate.sleep()
                continue

            pos = (self.pose.position.x, self.pose.position.y)
            goal_dist = dist2d(pos, self.goal)
            self.min_goal_distance = min(self.min_goal_distance, goal_dist)

            roll, pitch, _ = quaternion_to_rpy(self.pose.orientation)
            tilt_deg = math.degrees(max(abs(roll), abs(pitch)))
            self.max_tilt_deg = max(self.max_tilt_deg, tilt_deg)

            if tilt_deg >= self.roll_pitch_fail_deg:
                tipped = True
                failure_reason = "tipped"
                break
            if goal_dist <= self.goal_tolerance_m:
                success = True
                failure_reason = ""
                break
            if elapsed >= self.timeout_s:
                break
            rate.sleep()

        duration = time.time() - start_wall
        self.ensure_manual_before_exit()
        self.final_controller_mode = self.controller_mode
        row = {
            "task_id": self.task_id,
            "method": self.method,
            "goal_x": "{:.3f}".format(self.goal[0]),
            "goal_y": "{:.3f}".format(self.goal[1]),
            "goal_z": "{:.3f}".format(self.goal[2]),
            "start_x": "{:.3f}".format(self.start_pos[0]),
            "start_y": "{:.3f}".format(self.start_pos[1]),
            "start_z": "{:.3f}".format(self.start_pos[2]),
            "success": int(success),
            "failure_reason": failure_reason,
            "time_to_goal": "{:.3f}".format(duration if success else 0.0),
            "min_goal_distance": "{:.3f}".format(self.min_goal_distance),
            "tipped": int(tipped),
            "max_tilt_deg": "{:.3f}".format(self.max_tilt_deg),
            "initial_controller_mode": self.initial_controller_mode,
            "final_controller_mode": self.final_controller_mode,
        }
        self.write_result(row)
        rospy.loginfo(
            "Manual goal evaluation done: success=%s failure=%s time_to_goal=%s tipped=%s",
            row["success"],
            row["failure_reason"],
            row["time_to_goal"],
            row["tipped"],
        )


if __name__ == "__main__":
    rospy.init_node("manual_goal_eval")
    ManualGoalEval().run()
