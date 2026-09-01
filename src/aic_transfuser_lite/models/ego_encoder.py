from __future__ import annotations

import torch
from torch import nn


class EgoEncoder(nn.Module):
    """Encode ego state [B,D] into one token [B,1,C]."""

    def __init__(self, input_dim: int = 5, hidden_dim: int = 64, output_dim: int = 128) -> None:
        super().__init__()
        self.network = nn.Sequential(
            nn.Linear(input_dim, hidden_dim),
            nn.LayerNorm(hidden_dim),
            nn.ReLU(inplace=True),
            nn.Linear(hidden_dim, output_dim),
        )

    def forward(self, ego: torch.Tensor) -> torch.Tensor:
        if ego.ndim != 2:
            raise ValueError(f"Expected ego [B,D], got {tuple(ego.shape)}")
        return self.network(ego).unsqueeze(1)
