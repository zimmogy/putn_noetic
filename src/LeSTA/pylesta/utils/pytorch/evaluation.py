"""
Author: Ikhyeon Cho
Link: https://github.com/Ikhyeon-Cho
File: utils/pytorch/evaluation.py
"""

import torch

class ConfusionMatrixTracker:
    def __init__(self, num_classes: int):
        self.num_classes = num_classes
        self.confusion_matrices = {}

    def add_batch(self, preds: dict, targets: dict):
        if not isinstance(preds, dict) or not isinstance(targets, dict):
            raise ValueError(
                'ConfusionMatrixTracker: Preds and targets must be dictionaries')
        # Add batch to confusion matrix
        for pred_key in preds.keys():
            if pred_key not in self.confusion_matrices:
                self.confusion_matrices[pred_key] = torch.zeros(
                    self.num_classes, self.num_classes)
            # Ensure tensors are on the same device before adding
            conf_mat = self._update_confMat(preds[pred_key], targets[pred_key])
            self.confusion_matrices[pred_key] += conf_mat.to(self.confusion_matrices[pred_key].device)

    def _update_confMat(self, preds: torch.Tensor, targets: torch.Tensor):
        # Validity check
        assert preds.shape == targets.shape
        x_row = preds.view(-1).long()
        y_row = targets.view(-1).long()
        assert x_row.shape == y_row.shape
        
        # Mask out invalid targets (e.g. 忽略标签 -1 的点)
        valid_mask = (y_row >= 0) & (y_row < self.num_classes)
        x_row = x_row[valid_mask]
        y_row = y_row[valid_mask]
        
        # Calculate confusion matrix using bincount
        indices = self.num_classes * y_row + x_row
        m = torch.bincount(indices, minlength=self.num_classes ** 2)
        return m.reshape(self.num_classes, self.num_classes).float()

    def get_metrics(self, pred_key: str):
        """基于混淆矩阵计算 Pr, Re, Spe, F1 指标"""
        if pred_key not in self.confusion_matrices:
            return None
            
        cm = self.confusion_matrices[pred_key]
        # 混淆矩阵结构: 行是 True Label (0: Non-trav, 1: Trav), 列是 Predicted Label
        tn = cm[0, 0].item()
        fp = cm[0, 1].item()
        fn = cm[1, 0].item()
        tp = cm[1, 1].item()
        
        precision = tp / (tp + fp) if (tp + fp) > 0 else 0.0
        recall = tp / (tp + fn) if (tp + fn) > 0 else 0.0
        specificity = tn / (tn + fp) if (tn + fp) > 0 else 0.0
        f1_score = 2 * (precision * recall) / (precision + recall) if (precision + recall) > 0 else 0.0
        
        return {
            "Precision": precision,
            "Recall": recall,
            "Specificity": specificity,
            "F1": f1_score
        }

class LossTracker:
    def __init__(self):
        # tracked_losses: {loss_name: [loss_values]}
        # steps: [steps_corresponding_to_loss_values]
        self.losses_tracked: dict[str, list] = {}
        self.steps: list[int] = []

    def append(self, loss_to_add: dict[str, torch.Tensor], step: int = None):
        """Add losses for later usage like epoch loss calculation, logging, etc.

        Args:
            loss: Losses as dictionary format {loss_name: loss_value}
            step: Global batch step number
        """
        if not isinstance(loss_to_add, dict):
            raise ValueError('LossTrackerTB: Loss must be a dictionary')
        # Append batch losses
        for loss_name, loss_val in loss_to_add.items():
            if loss_name not in self.losses_tracked:
                self.losses_tracked[loss_name] = []
            # Store tensor values as list
            self.losses_tracked[loss_name].append(loss_val.detach())
        # Append batch steps
        if step is not None:
            self.steps.append(step)

    def batch_loss(self) -> dict[str, list]:
        """Get all stored batch losses."""
        return self.losses_tracked

    def batch_steps(self) -> list[int]:
        """Get all stored steps."""
        return self.steps

    def epoch_loss(self) -> dict[str, float]:
        """Calculate epoch loss from stored batch losses."""
        epoch_losses = {}
        for loss_name, loss_vals in self.losses_tracked.items():
            mean_loss = torch.stack(loss_vals).mean().cpu().item()
            epoch_losses[loss_name] = mean_loss
        return epoch_losses

    def reset(self):
        """Reset tracked losses and steps."""
        self.losses_tracked = {}
        self.steps = []
