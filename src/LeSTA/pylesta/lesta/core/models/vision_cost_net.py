import torch
import torch.nn as nn
import torchvision.models as models

class MobileNetV3CostNet(nn.Module):
    """
    轻量级视觉代价估计网络 (纯 MobileNetV3 Encoder + 简易 Decoder)
    输入: [B, 3, H, W] 的 RGB 图像
    输出: [B, H, W] 的单通道连续代价图 (0.0 ~ 1.0)
    """
    def __init__(self, pretrained=True):
        super(MobileNetV3CostNet, self).__init__()
        
        # 1. 编码器 (Encoder): 使用 MobileNetV3 Large 的特征提取部分
        mobilenet = models.mobilenet_v3_large(pretrained=pretrained)
        self.backbone = mobilenet.features
        
        # MobileNetV3 Large 的最终特征通道数为 960，分辨率为输入图像的 1/32
        
        # 2. 解码器 (Decoder): 渐进式上采样恢复空间分辨率
        self.decoder = nn.Sequential(
            # 第一阶段: 通道降维与初步特征融合 (1/32)
            nn.Conv2d(960, 256, kernel_size=3, padding=1, bias=False),
            nn.BatchNorm2d(256),
            nn.ReLU(inplace=True),
            
            # 放大 4 倍 -> 1/8 分辨率
            nn.Upsample(scale_factor=4, mode='bilinear', align_corners=False),
            nn.Conv2d(256, 64, kernel_size=3, padding=1, bias=False),
            nn.BatchNorm2d(64),
            nn.ReLU(inplace=True),
            
            # 再放大 8 倍 -> 1/1 (恢复原图分辨率)
            nn.Upsample(scale_factor=8, mode='bilinear', align_corners=False),
            
            # 最终输出层: 压缩为单通道
            nn.Conv2d(64, 1, kernel_size=1)
        )
        
        # 3. 激活函数: 将代价限制在 0~1 之间
        self.sigmoid = nn.Sigmoid()

    def forward(self, x):
        # 特征提取: x shape -> [B, 960, H/32, W/32]
        features = self.backbone(x) 
        
        # 解码还原: features shape -> [B, 1, H, W]
        out = self.decoder(features) 
        
        # 去除通道维度，输出 [B, H, W]
        return self.sigmoid(out).squeeze(1)

if __name__ == "__main__":
    # 测试网络连通性与参数量
    model = MobileNetV3CostNet()
    dummy_input = torch.randn(2, 3, 520, 520) # 假设 RELLIS-3D 预处理后的分辨率
    output = model(dummy_input)
    print(f"Output shape: {output.shape}")    # 预期: torch.Size([2, 520, 520])
    
    # 打印参数量，您会发现它极度轻量
    total_params = sum(p.numel() for p in model.parameters())
    print(f"Total parameters: {total_params / 1e6:.2f} M")