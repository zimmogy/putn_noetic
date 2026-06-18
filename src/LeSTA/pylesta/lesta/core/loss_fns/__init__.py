from .bce_loss import BCELoss
from .instance_weighted_loss import InstanceWeightedLoss
from .instance_weighted_loss import InstanceWeightedBCEWithLogitLossAndEntropy

# [new] import new loss functions here
from .masked_loss import MaskedSmoothL1Loss

# update __all__ to include new loss functions
__all__ = [
    "BCELoss",
    "InstanceWeightedLoss",
    "InstanceWeightedBCEWithLogitLossAndEntropy",
    "MaskedSmoothL1Loss",  # [new] add new loss function to __all__
]
