#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
# 带有详细 Debug 日志的视觉数据提取脚本
"""
import rospy
import cv2
import numpy as np
import os
import math
from collections import deque
import tf2_ros
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge

class VisionDatasetExtractor:
    def __init__(self):
        rospy.init_node('vision_dataset_extractor', anonymous=True)
        
        # 配置参数
        self.delay_time = rospy.get_param('~delay_time', 3.0) 
        self.output_dir = rospy.get_param('~output_dir', '/home/whr/Data/dataset/rellis')
        self.image_dir = os.path.join(self.output_dir, 'train/images')
        self.mask_dir = os.path.join(self.output_dir, 'train/sparse_masks')
        
        os.makedirs(self.image_dir, exist_ok=True)
        os.makedirs(self.mask_dir, exist_ok=True)

        self.K = np.array([
            [2813.643275, 0.0,         969.285772],
            [0.0,         2808.326079, 624.049972],
            [0.0,         0.0,         1.0       ]
        ])
        
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(15.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.bridge = CvBridge()
        
        self.image_queue = deque()
        self.imu_queue = deque()
        
        self.lambda_decay = 0.5
        self.min_soft_label = 0.2
        self.imu_window_size = 0.5 

        # Debug 统计变量
        self.received_images_count = 0
        self.received_imu_count = 0
        
        rospy.Subscriber('/pylon_camera_node/image_raw', Image, self.image_callback)
        rospy.Subscriber('/vectornav/IMU', Imu, self.imu_callback)
        rospy.Timer(rospy.Duration(0.1), self.process_queue)
        
        rospy.loginfo(f"====== Vision Dataset Extractor 初始化成功 ======")
        rospy.loginfo(f"输出目录: {self.output_dir}")
        rospy.loginfo("正在等待接收图像和 IMU 数据...")

    def imu_callback(self, msg):
        self.imu_queue.append(msg)
        self.received_imu_count += 1
        if self.received_imu_count % 1000 == 0:
            rospy.loginfo(f"[监控] 已接收 {self.received_imu_count} 条 IMU 数据")
            
        while self.imu_queue and (msg.header.stamp - self.imu_queue[0].header.stamp).to_sec() > 10.0:
            self.imu_queue.popleft()

    def image_callback(self, msg):
        self.image_queue.append(msg)
        self.received_images_count += 1
        if self.received_images_count % 100 == 0:
            rospy.loginfo(f"[监控] 已接收 {self.received_images_count} 张图像, 当前队列堆积: {len(self.image_queue)} 张")

    def calculate_soft_label(self, target_time):
        valid_imus = [m.linear_acceleration.z for m in self.imu_queue 
                      if 0 <= (target_time - m.header.stamp).to_sec() <= self.imu_window_size]
        if not valid_imus:
            return 1.0
        mean_z = sum(valid_imus) / len(valid_imus)
        variance_z = sum((z - mean_z) ** 2 for z in valid_imus) / len(valid_imus)
        return max(self.min_soft_label, float(math.exp(-self.lambda_decay * variance_z)))

    def process_queue(self, event):
        now = rospy.Time.now()
        
        if not self.image_queue:
            return
            
        while self.image_queue:
            img_msg = self.image_queue[0]
            img_time = img_msg.header.stamp
            time_diff = (now - img_time).to_sec()
            
            # [Debug] 检查是否忘记开仿真时间
            if time_diff > 100000:
                rospy.logerr_throttle(5.0, f"[致命错误] 时间差巨大 ({time_diff}秒)！你绝对忘记了设置 use_sim_time 或没有带 --clock 播放！")
                return

            # 如果还没攒够未来的时间，就继续等
            if time_diff < self.delay_time:
                break
                
            # 满足延迟条件，弹出图像进行处理
            self.image_queue.popleft()
            
            try:
                cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
            except Exception as e:
                rospy.logwarn(f"CV Bridge 转换失败: {e}")
                continue
                
            image_shape = cv_image.shape[:2]
            mask = np.full(image_shape, -1.0, dtype=np.float32) 
            has_trajectory = False
            
            # [Debug] 诊断信息统计
            tf_error_count = 0
            behind_camera_count = 0
            out_of_bounds_count = 0
            valid_points_count = 0
            last_tf_error = ""
            
            for dt in np.arange(0.0, self.delay_time, 0.1):
                future_time = img_time + rospy.Duration(dt)
                
                try:
                    trans = self.tf_buffer.lookup_transform_full(
                        target_frame='pylon_camera', 
                        target_time=img_time,
                        source_frame='ouster1/os1_lidar',
                        source_time=future_time,
                        fixed_frame='odom',
                        timeout=rospy.Duration(0.05)
                    )
                    
                    x, y, z = trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z
                    
                    if z <= 0.1:
                        behind_camera_count += 1
                        continue
                        
                    u = int((self.K[0,0] * x) / z + self.K[0,2])
                    v = int((self.K[1,1] * y) / z + self.K[1,2])
                    
                    if 0 <= u < image_shape[1] and 0 <= v < image_shape[0]:
                        cost = self.calculate_soft_label(future_time)
                        cv2.circle(mask, (u, v), radius=25, color=float(cost), thickness=-1)
                        has_trajectory = True
                        valid_points_count += 1
                    else:
                        out_of_bounds_count += 1
                        
                except Exception as e:
                    tf_error_count += 1
                    last_tf_error = str(e)
                    continue
            
            timestamp_str = f"{img_time.to_sec():.6f}"
            
            # 如果没有提取到任何轨迹点，打印出具体是死在哪个环节了
            if not has_trajectory:
                rospy.logwarn_throttle(2.0, f"[丢弃图像] {timestamp_str} 没有有效轨迹点。诊断: "
                                            f"TF报错: {tf_error_count}次 (最近错误: {last_tf_error}), "
                                            f"相机后方: {behind_camera_count}个, "
                                            f"越界(飞出屏幕): {out_of_bounds_count}个")
                continue
                
            # 成功保存！
            cv2.imwrite(os.path.join(self.image_dir, f"{timestamp_str}.jpg"), cv_image)
            np.save(os.path.join(self.mask_dir, f"{timestamp_str}.npy"), mask)
            rospy.loginfo(f"[成功] 保存数据: {timestamp_str} | 轨迹点数: {valid_points_count}")

if __name__ == '__main__':
    try:
        VisionDatasetExtractor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass