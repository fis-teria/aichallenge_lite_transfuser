from __future__ import annotations

import torch
from torch import nn


class TrajectoryHead(nn.Module):
    """Predict K ego-frame trajectories as cumulative XY increments in metres."""

    def __init__(self, input_dim: int, *, candidates: int = 1, steps: int = 15) -> None:
        super().__init__()
        if input_dim <= 0 or candidates <= 0 or steps <= 0:
            raise ValueError("input_dim, candidates and steps must be positive")
        self.candidates = candidates
        self.steps = steps
        self.delta_xy = nn.Linear(input_dim, candidates * steps * 2)
        self.candidate_logits = nn.Linear(input_dim, candidates)

    def forward(self, feature: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """Return ``trajectory_xy [B,K,N,2]`` and logits ``[B,K]``."""
        if feature.ndim != 2:
            raise ValueError("trajectory feature must be [B,D]")
        batch = feature.shape[0]
        delta = self.delta_xy(feature).view(batch, self.candidates, self.steps, 2)
        trajectory = torch.cumsum(delta, dim=2)
        logits = self.candidate_logits(feature)
        if not torch.isfinite(trajectory).all() or not torch.isfinite(logits).all():
            raise ValueError("trajectory head produced non-finite output")
        return trajectory, logits
