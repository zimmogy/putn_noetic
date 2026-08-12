"""
Modified by: Haoran Wang
Revision date: 2026-08-12
"""

import torch
from torch.nn import Module, BCEWithLogitsLoss
from .entropy_regularization import EntropyRegularization

class UncertaintyAwareBCELoss(Module):
    """
    Uncertainty-aware BCE loss for traversability learning.

    Variance, intensity variance, and sparsity are used to reduce the penalty
    assigned to uncertain negative samples.
    """
    def __init__(self, reduction='mean', pos_weight=None, 
                 alpha=1.0, beta=1.0, gamma=0.5, min_weight=0.1):
        super().__init__()
        self.reduction = reduction
        self.criterion = BCEWithLogitsLoss(reduction='none', pos_weight=pos_weight)
        
        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma
        self.min_weight = min_weight

    def set_pos_weight(self, pos_weight):
        self.criterion.pos_weight = pos_weight

    def compute_certainty_weights(self, targets, variance, intensity_var, sparsity):
        epsilon = 1e-8

        log_var = torch.log(variance + epsilon)
        var_min, var_max = log_var.min(), log_var.max()
        norm_var = (log_var - var_min) / (var_max - var_min + epsilon)

        int_min, int_max = intensity_var.min(), intensity_var.max()
        norm_int_var = (intensity_var - int_min) / (int_max - int_min + epsilon)

        spa_min, spa_max = sparsity.min(), sparsity.max()
        norm_sparsity = (sparsity - spa_min) / (spa_max - spa_min + epsilon)
        
        penalty = self.alpha * norm_var + self.beta * norm_int_var + self.gamma * norm_sparsity
        
        certainty = torch.exp(-penalty)
        
        weights = torch.where(
            targets == 1.0,
            torch.ones_like(targets),
            certainty
        )
        
        weights = torch.clamp(weights, min=self.min_weight, max=1.0)
        return weights

    def forward(self, logits, targets, variance, intensity_var, sparsity):
        if logits.dim() > 1:
            logits = logits.squeeze(-1)

        instance_weights = self.compute_certainty_weights(targets, variance, intensity_var, sparsity)
        
        smoothed_targets = targets.float() * 0.90 + 0.05

        loss = self.criterion(logits, smoothed_targets)

        if instance_weights.dim() == 1 and loss.dim() > 1:
            instance_weights = instance_weights.view(-1, *([1] * (loss.dim() - 1)))

        weighted_loss = loss * instance_weights

        if self.reduction == 'mean':
            return weighted_loss.mean()
        elif self.reduction == 'sum':
            return weighted_loss.sum()
        else:
            return weighted_loss


class UncertaintyAwareLossWithEntropy(Module):
    """
    Uncertainty-aware loss with entropy regularization.
    """
    def __init__(self, entropy_coefficient=0.1, reduction='mean', pos_weight=None,
                 alpha=1.0, beta=1.0, gamma=0.5, min_weight=0.1):
        super().__init__()
        self.bce_uncertainty = UncertaintyAwareBCELoss(
            reduction=reduction, pos_weight=pos_weight, 
            alpha=alpha, beta=beta, gamma=gamma, min_weight=min_weight
        )
        self.entropy_reg = EntropyRegularization(coefficient=entropy_coefficient)

    def set_pos_weight(self, pos_weight):
        self.bce_uncertainty.set_pos_weight(pos_weight)

    def forward(self, logits, targets, variance, intensity_var, sparsity):
        bce_loss = self.bce_uncertainty(logits, targets, variance, intensity_var, sparsity)
        
        if logits.dim() > 1:
            logits = logits.squeeze(-1)
        entropy_term = self.entropy_reg(logits)

        return bce_loss + entropy_term
