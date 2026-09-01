from __future__ import annotations

import torch
from torch import nn
from torch.nn import functional as F


class SpeedProfileHead(nn.Module):
    """Predict non-negative speed in m/s for every trajectory point."""

    def __init__(self, input_dim: int, *, candidates: int = 1, steps: int = 15) -> None:
        super().__init__()
        if input_dim <= 0 or candidates <= 0 or steps <= 0:
            raise ValueError("input_dim, candidates and steps must be positive")
        self.candidates = candidates
        self.steps = steps
        self.projection = nn.Linear(input_dim, candidates * steps)

    def forward(self, feature: torch.Tensor) -> torch.Tensor:
        """Return speed profile ``[B,K,N]`` in m/s."""
        if feature.ndim != 2:
            raise ValueError("speed feature must be [B,D]")
        speed = F.softplus(self.projection(feature)).view(
            feature.shape[0], self.candidates, self.steps
        )
        if not torch.isfinite(speed).all():
            raise ValueError("speed head produced non-finite output")
        return speed
