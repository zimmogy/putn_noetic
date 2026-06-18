import torch
import torch.nn as nn

class MaskedSmoothL1Loss(nn.Module):
    def __init__(self, ignore_index=-1.0):
        super(MaskedSmoothL1Loss, self).__init__()
        self.ignore_index = ignore_index
        # reduction='none' 保留每个像素的 loss，方便我们根据 mask 过滤
        self.criterion = nn.SmoothL1Loss(reduction='none')

    def forward(self, pred, target):
        valid_mask = (target != self.ignore_index)
        
        if valid_mask.sum() == 0:
            return torch.tensor(0.0, device=pred.device, requires_grad=True)

        loss = self.criterion(pred[valid_mask], target[valid_mask])
        return loss.mean()