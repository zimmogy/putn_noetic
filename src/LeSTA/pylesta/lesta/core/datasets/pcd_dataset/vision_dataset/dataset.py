import os
import cv2
import torch
import numpy as np
from torch.utils.data import Dataset

class RellisVisionDataset(Dataset):
    def __init__(self, image_dir, mask_dir):
        self.image_dir = image_dir
        self.mask_dir = mask_dir
        self.image_files = sorted([f for f in os.listdir(image_dir) if f.endswith('.jpg')])

    def __len__(self):
        return len(self.image_files)

    def __getitem__(self, idx):
        img_name = self.image_files[idx]
        img_path = os.path.join(self.image_dir, img_name)
        mask_path = os.path.join(self.mask_dir, img_name.replace('.jpg', '.npy'))

        # OpenCV 读取并转为 RGB
        image = cv2.imread(img_path)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # 读取带有 -1.0 (忽略) 和 0.0~1.0 (代价) 的 Numpy Mask
        mask = np.load(mask_path).astype(np.float32)

        # 图像归一化与维度转换 (H,W,C -> C,H,W)
        image_tensor = torch.from_numpy(image.transpose(2, 0, 1)).float() / 255.0
        mask_tensor = torch.from_numpy(mask).float()

        return image_tensor, mask_tensor