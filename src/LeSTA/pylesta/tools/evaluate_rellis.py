#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Modified by: Haoran Wang
Revision date: 2026-08-12
"""


import torch
import numpy as np
from torch.utils.data import DataLoader

from lesta.core.datasets.pcd_dataset.dataset import PCDDataset
from lesta.core.models.mlp_classifier import MLPClassifier
from utils.param import yaml
from utils.pytorch.evaluation import ConfusionMatrixTracker

def evaluate_model(config_path, model_ckpt_path, val_pcd_path):
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    cfg = yaml.load(config_path)
    
    print(f"Loading model checkpoint: {model_ckpt_path}")
    try:
        model = torch.jit.load(model_ckpt_path, map_location=device)
        print("Loaded checkpoint as a TorchScript model.")
    except RuntimeError:
        print("Checkpoint is not TorchScript. Loading as a PyTorch state_dict.")
        model = MLPClassifier(cfg['MODEL'])
        checkpoint = torch.load(model_ckpt_path, map_location=device, weights_only=True)
        if 'model_state_dict' in checkpoint:
            model.load_state_dict(checkpoint['model_state_dict'])
        else:
            model.load_state_dict(checkpoint)
        print("Loaded checkpoint as a state_dict.")
        
    model.to(device)
    model.eval()
    print(f"Model loaded: {model_ckpt_path}")

    val_cfg = cfg['DATASET'].copy()
    val_cfg['training_data'] = val_pcd_path 
    
    print(f"Loading validation PCD: {val_pcd_path}")
    val_dataset = PCDDataset(val_pcd_path, val_cfg)
    val_loader = DataLoader(val_dataset, batch_size=2048, shuffle=False, num_workers=4)

    all_preds = []
    all_targets = []

    print("Running inference...")
    with torch.no_grad():
        for batch in val_loader:
            feats = batch["feats"].to(device)
            labels = batch["label"].to(device)

            preds_prob = model(feats)

            all_preds.append(preds_prob.cpu().numpy())
            all_targets.append(labels.cpu().numpy())

    preds_np = np.concatenate(all_preds, axis=0).squeeze()
    targets_np = np.concatenate(all_targets, axis=0).squeeze()

    valid_mask = (targets_np != -1.0)
    
    clean_targets = targets_np[valid_mask]
    clean_preds_prob = preds_np[valid_mask]
    
    clean_preds_hard = (clean_preds_prob >= 0.5).astype(np.float32)

    print(f"Valid evaluation cells: {len(clean_targets)}; ignored cells: {len(targets_np) - len(clean_targets)}")

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
    print("Traversability evaluation report")
    print("="*40)
    print(f"Accuracy: \t{accuracy * 100:.2f}%")
    print(f"Precision: \t{metrics['Precision'] * 100:.2f}%")
    print(f"Recall: \t{metrics['Recall'] * 100:.2f}%")
    print(f"Specificity: \t{metrics['Specificity'] * 100:.2f}%")
    print(f"F1-Score: \t{metrics['F1'] * 100:.2f}%")
    print(f"IoU: \t\t{iou * 100:.2f}%")
    print("="*40)
    
    metrics['Accuracy'] = accuracy
    metrics['IoU'] = iou
    
    return metrics

if __name__ == "__main__":
    CONFIG_YAML = "/home/whr/rellis_ws/src/LeSTA/pylesta/configs/lesta.yaml"
    BEST_MODEL_CKPT = "/home/whr/rellis_ws/src/LeSTA/pylesta/checkpoints/epoch_best.pt" 
    EVAL_PCD = "/home/whr/Data/dataset/00000/val_set_with_gt/rellis_00000_global_eval_ascii.pcd" 
    
    evaluate_model(CONFIG_YAML, BEST_MODEL_CKPT, EVAL_PCD)
