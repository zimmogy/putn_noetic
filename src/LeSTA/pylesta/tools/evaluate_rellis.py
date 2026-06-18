#!/usr/bin/env python
# -*- coding: utf-8 -*-

import torch
import numpy as np
from torch.utils.data import DataLoader

# 根据实际路径调整导入
from lesta.core.datasets.pcd_dataset.dataset import PCDDataset
from lesta.core.models.mlp_classifier import MLPClassifier
from utils.param import yaml
from utils.pytorch.evaluation import ConfusionMatrixTracker

def evaluate_model(config_path, model_ckpt_path, val_pcd_path):
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"🚀 使用设备: {device}")

    # ==========================================
    # 1. 加载配置与模型
    # ==========================================

    cfg = yaml.load(config_path)
    
    print(f"📦 正在加载模型权重: {model_ckpt_path}")
    try:
        # 针对 yaml 中 save_for_libtorch: true 生成的 TorchScript 模型
        model = torch.jit.load(model_ckpt_path, map_location=device)
        print("✅ 成功作为 TorchScript (JIT) 模型加载！")
    except RuntimeError:
        # 向下兼容：如果未来你关闭了 save_for_libtorch，使用传统 state_dict 加载
        print("⚠️ 检测到非 TorchScript 格式，尝试作为传统 state_dict 加载...")
        model = MLPClassifier(cfg['MODEL'])
        # 增加 weights_only=True 消除安全警告
        checkpoint = torch.load(model_ckpt_path, map_location=device, weights_only=True)
        if 'model_state_dict' in checkpoint:
            model.load_state_dict(checkpoint['model_state_dict'])
        else:
            model.load_state_dict(checkpoint)
        print("✅ 成功作为 State Dict 加载！")
        
    model.to(device)
    model.eval()
    print(f"✅ 模型加载成功: {model_ckpt_path}")

    # ==========================================
    # 2. 构建验证集 DataLoader
    # ==========================================
    val_cfg = cfg['DATASET'].copy()
    val_cfg['training_data'] = val_pcd_path 
    
    print(f"📦 正在加载验证集 PCD: {val_pcd_path}")
    val_dataset = PCDDataset(val_pcd_path, val_cfg)
    val_loader = DataLoader(val_dataset, batch_size=2048, shuffle=False, num_workers=4)

    # ==========================================
    # 3. 运行推理与收集数据
    # ==========================================
    all_preds = []
    all_targets = []

    print("🧠 开始前向推理计算...")
    with torch.no_grad():
        for batch in val_loader:
            feats = batch["feats"].to(device)
            labels = batch["label"].to(device)

            preds_prob = model(feats)

            all_preds.append(preds_prob.cpu().numpy())
            all_targets.append(labels.cpu().numpy())

    # 拼接所有的 batch
    preds_np = np.concatenate(all_preds, axis=0).squeeze()
    targets_np = np.concatenate(all_targets, axis=0).squeeze()

    # ==========================================
    # 4. 数据清洗：核心的物理掩码 (Masking)
    # ==========================================
    valid_mask = (targets_np != -1.0)
    
    clean_targets = targets_np[valid_mask]
    clean_preds_prob = preds_np[valid_mask]
    
    clean_preds_hard = (clean_preds_prob >= 0.5).astype(np.float32)

    print(f"🧹 清洗完毕! 有效评估网格数: {len(clean_targets)} (剔除了 {len(targets_np) - len(clean_targets)} 个未知网格)")

    # ==========================================
    # 5. 计算并打印评估指标
    # ==========================================
    preds_tensor = torch.from_numpy(clean_preds_hard)
    targets_tensor = torch.from_numpy(clean_targets)

    metrics_calculator = ConfusionMatrixTracker(num_classes=2)
    
    preds_dict = {"eval": preds_tensor}
    targets_dict = {"eval": targets_tensor}
    
    metrics_calculator.add_batch(preds_dict, targets_dict)
    metrics = metrics_calculator.get_metrics("eval")
    
    cm = metrics_calculator.confusion_matrices["eval"]
    tn, fp = cm[0, 0].item(), cm[0, 1].item()
    fn, tp = cm[1, 0].item(), cm[1, 1].item()
    
    total = tp + tn + fp + fn
    accuracy = (tp + tn) / total if total > 0 else 0.0
    iou = tp / (tp + fp + fn) if (tp + fp + fn) > 0 else 0.0

    print("\n" + "="*40)
    print("📈 越野环境通过性 (Traversability) 评估报告")
    print("="*40)
    print(f"Accuracy (整体准确率): \t{accuracy * 100:.2f}%")
    print(f"Precision (精确率): \t{metrics['Precision'] * 100:.2f}%")
    print(f"Recall (召回率): \t{metrics['Recall'] * 100:.2f}%")
    print(f"Specificity (特异度): \t{metrics['Specificity'] * 100:.2f}%")
    print(f"F1-Score (F1分数): \t{metrics['F1'] * 100:.2f}%")
    print(f"IoU (交并比): \t\t{iou * 100:.2f}%")
    print("="*40)
    
    metrics['Accuracy'] = accuracy
    metrics['IoU'] = iou
    
    return metrics

if __name__ == "__main__":
    CONFIG_YAML = "/home/whr/rellis_ws/src/LeSTA/pylesta/configs/lesta.yaml"
    BEST_MODEL_CKPT = "/home/whr/rellis_ws/src/LeSTA/pylesta/checkpoints/epoch_best.pt" 
    EVAL_PCD = "/home/whr/Data/dataset/00000/val_set_with_gt/rellis_00000_global_eval_ascii.pcd" 
    
    evaluate_model(CONFIG_YAML, BEST_MODEL_CKPT, EVAL_PCD)