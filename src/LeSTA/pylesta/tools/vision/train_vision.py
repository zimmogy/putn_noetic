import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../'))

# ==========================================
# 【新增】：解决 OpenBLAS 和 DataLoader 的多进程冲突
# 必须放在 import torch 和其他科学计算库之前！
# ==========================================
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["OMP_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"      # 针对 Intel MKL
os.environ["NUMEXPR_NUM_THREADS"] = "1"  # 针对 NumExpr

import torch
import torch.nn.functional as F
import torch.optim as optim
from torch.utils.data import DataLoader

import cv2
cv2.setNumThreads(0)  # 禁止 OpenCV 使用多线程，避免与 PyTorch DataLoader 冲突

from pylesta.lesta.core.models.vision_cost_net import MobileNetV3CostNet
from pylesta.lesta.core.loss_fns.masked_loss import MaskedSmoothL1Loss
from pylesta.lesta.core.datasets.pcd_dataset.vision_dataset.dataset import RellisVisionDataset

def main():
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Start training on: {device}")

    # 1. 实例化数据集
    train_dataset = RellisVisionDataset(
        image_dir='/home/whr/Data/dataset/00000/rellis/train/images',       # 【修改这里】
        mask_dir='/home/whr/Data/dataset/00000/rellis/train/sparse_masks'   # 【修改这里】
    )
    # 增加 num_workers 提升读取速度
    train_loader = DataLoader(train_dataset, batch_size=2, shuffle=True, num_workers=4)

    # 2. 实例化 MobileNetV3 骨干网络
    model = MobileNetV3CostNet(pretrained=True).to(device)
    criterion = MaskedSmoothL1Loss(ignore_index=-1.0)
    optimizer = optim.Adam(model.parameters(), lr=2e-4, weight_decay=1e-5)

    num_epochs = 30
    
    # 3. 训练循环
    for epoch in range(num_epochs):
        model.train()
        epoch_loss = 0.0
        
        for batch_idx, (images, masks) in enumerate(train_loader):
            images, masks = images.to(device), masks.to(device)
            
            # ===================================
            # 【新增修改】：将“通行度”标签反转为“风险代价”
            # 注意：必须跳过背景空白区域的 ignore_index(-1.0)
            # ===================================
            valid_pixels = (masks != -1.0)
            masks[valid_pixels] = 1.0 - masks[valid_pixels]
            # ===================================

            optimizer.zero_grad()
            preds = model(images)

            # ====================================
            # [修复] 强制将preds的尺寸对其到masks的村吃
            # interpolate 需要四维，所以需要先升维再降维
            # ====================================
            if preds.shape != masks.shape:
                # 升维: [8, 1216, 1920] -> [8, 1, 1216, 1920]
                preds = preds.unsqueeze(1) 
                # 双线性插值缩放到 1200x1920
                preds = F.interpolate(preds, size=(masks.shape[1], masks.shape[2]), mode='bilinear', align_corners=False)
                # 降维恢复: [8, 1, 1200, 1920] -> [8, 1200, 1920]
                preds = preds.squeeze(1)
            # ==========================================
            loss = criterion(preds, masks)
            
            # 跳过全图无轨迹的无效批次
            if loss.item() == 0.0:
                continue
                
            loss.backward()
            optimizer.step()
            
            epoch_loss += loss.item()
            
            if batch_idx % 20 == 0:
                print(f"Epoch [{epoch+1}/{num_epochs}], Batch [{batch_idx}/{len(train_loader)}], Loss: {loss.item():.4f}")
                
        print(f"--> Epoch {epoch+1} Average Loss: {epoch_loss/len(train_loader):.4f}")
        torch.save(model.state_dict(), f"mobilenetv3_cost_epoch_{epoch+1}.pth")

if __name__ == "__main__":
    main()