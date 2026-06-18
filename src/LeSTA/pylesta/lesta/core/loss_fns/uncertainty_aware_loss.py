import torch
from torch.nn import Module, BCEWithLogitsLoss
from .entropy_regularization import EntropyRegularization

class UncertaintyAwareBCELoss(Module):
    """
    结合几何与材质特征的不确定性感知损失函数。
    利用 variance, intensity_var 和 sparsity 动态降低模糊负样本的惩罚。
    """
    def __init__(self, reduction='mean', pos_weight=None, 
                 alpha=1.0, beta=1.0, gamma=0.5, min_weight=0.1):
        super().__init__()
        self.reduction = reduction
        self.criterion = BCEWithLogitsLoss(reduction='none', pos_weight=pos_weight)
        
        # 不确定性超参数：分别对应 variance, intensity_var, sparsity 的惩罚力度
        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma
        self.min_weight = min_weight

    def set_pos_weight(self, pos_weight):
        self.criterion.pos_weight = pos_weight

    def compute_certainty_weights(self, targets, variance, intensity_var, sparsity):
        epsilon = 1e-8  # 防止除以零或 log(0)

        # 1. 解决 Variance 极小值问题：先取对数放大差异，再做 Min-Max 归一化
        # 取对数能很好地拉开 10^-6 和 10^-4 之间的差距
        log_var = torch.log(variance + epsilon)
        var_min, var_max = log_var.min(), log_var.max()
        # 将其拉伸到 0 ~ 1
        norm_var = (log_var - var_min) / (var_max - var_min + epsilon)

        # 2. 解决 Intensity Var 可能极大的问题：同样使用 Min-Max 归一化
        int_min, int_max = intensity_var.min(), intensity_var.max()
        norm_int_var = (intensity_var - int_min) / (int_max - int_min + epsilon)

        # 3. Sparsity 归一化
        spa_min, spa_max = sparsity.min(), sparsity.max()
        norm_sparsity = (sparsity - spa_min) / (spa_max - spa_min + epsilon)
        
        # 4. 计算不确定性惩罚项 (现在 penalty 的值域有明确的界限)
        # norm_* 的值全都在 0~1 之间。值越接近1，代表该样本在当前 Batch 中特征越发散（越像草丛/越不确定）
        penalty = self.alpha * norm_var + self.beta * norm_int_var + self.gamma * norm_sparsity
        
        # 5. 将惩罚映射为 0~1 的置信度权重 (指数衰减)
        certainty = torch.exp(-penalty)
        
        # 6. 非对称加权 (Asymmetric Weighting)
        weights = torch.where(
            targets == 1.0,
            torch.ones_like(targets),
            certainty
        )
        
        # 7. 截断保底
        weights = torch.clamp(weights, min=self.min_weight, max=1.0)
        return weights

    def forward(self, logits, targets, variance, intensity_var, sparsity):
        if logits.dim() > 1:
            logits = logits.squeeze(-1)

        # 动态计算每个样本的权重
        instance_weights = self.compute_certainty_weights(targets, variance, intensity_var, sparsity)
        
        # ======== [新增：标签平滑 (Label Smoothing)] ========
        # 0.0 变成 0.05, 1.0 变成 0.95
        # 这能彻底防止网络输出无穷大的 Logit，从而极大地压低那 85.2% 的伪标签率
        smoothed_targets = targets.float() * 0.90 + 0.05
        # ====================================================

        # 计算基础 BCE Loss
        loss = self.criterion(logits, smoothed_targets)

        # 广播机制以对齐维度
        if instance_weights.dim() == 1 and loss.dim() > 1:
            instance_weights = instance_weights.view(-1, *([1] * (loss.dim() - 1)))

        # 应用动态权重
        weighted_loss = loss * instance_weights

        if self.reduction == 'mean':
            return weighted_loss.mean()
        elif self.reduction == 'sum':
            return weighted_loss.sum()
        else:
            return weighted_loss


class UncertaintyAwareLossWithEntropy(Module):
    """
    包含全局熵正则化的不确定性感知损失函数。
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
        # 1. 计算带不确定性权重的 BCE
        bce_loss = self.bce_uncertainty(logits, targets, variance, intensity_var, sparsity)
        
        # 2. 计算全局熵正则化
        if logits.dim() > 1:
            logits = logits.squeeze(-1)
        entropy_term = self.entropy_reg(logits)

        return bce_loss + entropy_term