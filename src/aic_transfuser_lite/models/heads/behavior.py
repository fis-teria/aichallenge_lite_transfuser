from __future__ import annotations

import torch
from torch import nn


class BehaviorHeadV1(nn.Module):
    """Factorized behavior and maneuver-side classifier."""

    def __init__(self, input_dim: int, *, classes: int = 5, sides: int = 3) -> None:
        super().__init__()
        if input_dim <= 0 or classes <= 1 or sides <= 1:
            raise ValueError("behavior head dimensions must be positive")
        self.shared = nn.Sequential(
            nn.Linear(input_dim, input_dim), nn.ReLU(inplace=True), nn.Dropout(0.1)
        )
        self.behavior = nn.Linear(input_dim, classes)
        self.side = nn.Linear(input_dim, sides)

    def forward(self, feature: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        shared = self.shared(feature)
        return self.behavior(shared), self.side(shared)
