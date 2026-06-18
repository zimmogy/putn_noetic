#!/usr/bin/env python3
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../pylesta')))
import rospy
import cv2
import torch
import numpy as np
from sensor_msgs.msg import Image

from lesta.core.models.vision_cost_net import MobileNetV3CostNet

class VisualCostInferNode:
    def __init__(self):
        rospy.init_node('visual_cost_mobilenet_node', anonymous=True)
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        # 1. 加载定制的 MobileNetV3 模型
        weight_path = rospy.get_param('~model_path', 'mobilenetv3_cost_epoch_30.pth')
        self.model = MobileNetV3CostNet(pretrained=False).to(self.device)
        self.model.load_state_dict(torch.load(weight_path, map_location=self.device))
        self.model.eval()
        
        rospy.loginfo(f"MobileNetV3 Cost Model loaded from {weight_path}")
        
        # 2. 订阅 RELLIS-3D 图像与发布 32FC1 代价图
        camera_topic = rospy.get_param('~camera_topic', '/pylon_camera_node/image_raw')
        
        # 【修改 3】：扩大队列并增加 buff_size，防止 Socket 阻塞导致大幅丢帧饿死同步器
        self.sub_img = rospy.Subscriber(camera_topic, Image, self.image_callback, 
                                        queue_size=5, buff_size=2**24)
        self.pub_cost = rospy.Publisher("/visual_cost_map", Image, queue_size=5)

    def image_callback(self, msg):
        # ==========================================
        # 1. 动态通道探测与 Bayer 图像解码
        # ==========================================
        try:
            # 拿到原始字节流，根据图像尺寸反推它的通道数
            img_1d = np.frombuffer(msg.data, dtype=np.uint8)
            channels = len(img_1d) // (msg.height * msg.width)

            if channels == 3:
                # 原生 3 通道彩色图
                img_np = img_1d.reshape((msg.height, msg.width, 3))
                if 'bgr' in msg.encoding.lower():
                    img_rgb = img_np[..., [2, 1, 0]]
                else:
                    img_rgb = img_np
            elif channels == 1:
                # 单通道图：大概率是 Bayer 格式 或 Mono8 灰度图
                img_np = img_1d.reshape((msg.height, msg.width)) # 转为纯 2D 矩阵
                encoding = msg.encoding.lower()
                
                if 'rggb' in encoding:
                    img_rgb = cv2.cvtColor(img_np, cv2.COLOR_BayerRG2RGB)
                elif 'gbrg' in encoding:
                    img_rgb = cv2.cvtColor(img_np, cv2.COLOR_BayerGB2RGB)
                elif 'grbg' in encoding:
                    img_rgb = cv2.cvtColor(img_np, cv2.COLOR_BayerGR2RGB)
                elif 'bayer' in encoding: # 默认后备为 bggr (Basler相机常见)
                    img_rgb = cv2.cvtColor(img_np, cv2.COLOR_BayerBG2RGB)
                else: # 纯灰度 mono8
                    img_rgb = cv2.cvtColor(img_np, cv2.COLOR_GRAY2RGB)
            else:
                rospy.logerr_throttle(1.0, f"[VisualCostNode] 无法解析的图像通道数: {channels}")
                return
                
            # 强制转换为物理连续内存！(彻底消除上一版的 Warning 和底层崩溃隐患)
            img_rgb = np.ascontiguousarray(img_rgb)
            
        except Exception as e:
            rospy.logerr_throttle(1.0, f"[VisualCostNode] 内存解析图像失败: {e}")
            return

        # ==========================================
        # 2. PyTorch 推理与显存溢出 (OOM) 保护
        # ==========================================
        try:
            # 缩小图像进行推理，防止 1920x1200 超大尺寸把显存撑爆 (极其重要)
            inference_h, inference_w = msg.height // 2, msg.width // 2
            img_resized = cv2.resize(img_rgb, (inference_w, inference_h), interpolation=cv2.INTER_LINEAR)
            
            # 转为 Tensor，此时必定是严格的 3 通道 -> shape: [1, 3, 600, 960]
            img_tensor = torch.from_numpy(img_resized.transpose(2, 0, 1)).float() / 255.0
            img_tensor = img_tensor.unsqueeze(0).to(self.device)

            with torch.no_grad():
                cost_map_tensor = self.model(img_tensor)
                
            cost_map_small = cost_map_tensor.squeeze().cpu().numpy().astype(np.float32)
            
            # 将输出代价图放大回原尺寸 (1920x1200)，为了让 C++ 底层的相机投影矩阵能完美对齐！
            cost_map_np = cv2.resize(cost_map_small, (msg.width, msg.height), interpolation=cv2.INTER_LINEAR)

        except RuntimeError as e:
            rospy.logerr_throttle(1.0, f"[VisualCostNode] PyTorch 推理异常: {e}")
            return

        # ==========================================
        # 3. 手动封装发布 (绕开 cv_bridge 冲突)
        # ==========================================
        try:
            cost_msg = Image()
            cost_msg.header = msg.header 
            cost_msg.height = cost_map_np.shape[0]
            cost_msg.width = cost_map_np.shape[1]
            cost_msg.encoding = "32FC1"
            cost_msg.is_bigendian = 0
            cost_msg.step = cost_msg.width * 4 
            cost_msg.data = cost_map_np.tobytes() 
            
            self.pub_cost.publish(cost_msg)
        except Exception as e:
            rospy.logerr(f"[VisualCostNode] 代价图发布失败: {e}")
            
if __name__ == '__main__':
    try:
        node = VisualCostInferNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass