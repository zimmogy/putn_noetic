#!/usr/bin/env python3
import copy
import math
import statistics
import struct

import rospy
import sensor_msgs.point_cloud2 as pc2
import tf
from sensor_msgs.msg import PointCloud2, PointField


class LidarBlindSpotPatch:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/laser_cloud_map")
        self.output_topic = rospy.get_param("~output_topic", "/laser_cloud_map_patched")
        self.robot_frame = rospy.get_param("~robot_frame", "base_link")
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 5.0))
        self.sample_inner_radius_m = float(rospy.get_param("~sample_inner_radius_m", 0.30))
        self.sample_outer_radius_m = float(rospy.get_param("~sample_outer_radius_m", 1.20))
        self.patch_half_width_m = float(rospy.get_param("~patch_half_width_m", 0.30))
        self.patch_resolution_m = float(rospy.get_param("~patch_resolution_m", 0.10))
        self.max_ground_samples = int(rospy.get_param("~max_ground_samples", 40))
        self.listener = tf.TransformListener()
        self.publisher = rospy.Publisher(self.output_topic, PointCloud2, queue_size=1)
        self.last_publish = rospy.Time(0)
        rospy.Subscriber(self.input_topic, PointCloud2, self.cloud_cb, queue_size=1)

    def cloud_cb(self, msg):
        now = rospy.Time.now()
        if self.publish_rate_hz > 0.0 and (now - self.last_publish).to_sec() < 1.0 / self.publish_rate_hz:
            return
        frame = msg.header.frame_id or "world"
        try:
            translation, _ = self.listener.lookupTransform(frame, self.robot_frame, rospy.Time(0))
        except tf.Exception as exc:
            rospy.logwarn_throttle(5.0, "Blind-spot patch waiting for TF %s -> %s: %s", frame, self.robot_frame, exc)
            return

        robot_x, robot_y, robot_z = translation
        candidates = []
        inner_sq = self.sample_inner_radius_m ** 2
        outer_sq = self.sample_outer_radius_m ** 2
        for x, y, z in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            distance_sq = (x - robot_x) ** 2 + (y - robot_y) ** 2
            if inner_sq <= distance_sq <= outer_sq and z < robot_z:
                candidates.append((distance_sq, z))

        if not candidates:
            rospy.logwarn_throttle(5.0, "No observed ground around robot; publishing unmodified planner map")
            self.publisher.publish(msg)
            self.last_publish = now
            return

        candidates.sort(key=lambda item: item[0])
        ground_z = statistics.median(item[1] for item in candidates[:self.max_ground_samples])
        patched = self.append_patch(msg, robot_x, robot_y, ground_z)
        self.publisher.publish(patched)
        self.last_publish = now
        rospy.logdebug("Patched planner map under robot at [%.2f, %.2f, %.2f]", robot_x, robot_y, ground_z)

    def append_patch(self, msg, center_x, center_y, ground_z):
        fields = {field.name: field for field in msg.fields}
        if not all(name in fields for name in ("x", "y", "z")):
            rospy.logerr_throttle(5.0, "Planner cloud has no x/y/z fields")
            return msg
        if any(fields[name].datatype != PointField.FLOAT32 for name in ("x", "y", "z")):
            rospy.logerr_throttle(5.0, "Blind-spot patch currently requires FLOAT32 x/y/z fields")
            return msg

        steps = max(0, int(round(self.patch_half_width_m / self.patch_resolution_m)))
        endian = ">" if msg.is_bigendian else "<"
        appended = bytearray()
        for ix in range(-steps, steps + 1):
            for iy in range(-steps, steps + 1):
                record = bytearray(msg.point_step)
                struct.pack_into(endian + "f", record, fields["x"].offset,
                                 center_x + ix * self.patch_resolution_m)
                struct.pack_into(endian + "f", record, fields["y"].offset,
                                 center_y + iy * self.patch_resolution_m)
                struct.pack_into(endian + "f", record, fields["z"].offset, ground_z)
                appended.extend(record)

        output = copy.copy(msg)
        output.height = 1
        output.width = msg.width * msg.height + len(appended) // msg.point_step
        output.row_step = output.width * msg.point_step
        output.data = bytes(msg.data) + bytes(appended)
        return output


if __name__ == "__main__":
    rospy.init_node("lidar_blind_spot_patch")
    LidarBlindSpotPatch()
    rospy.spin()
