from __future__ import annotations

import torch
from torch import nn
from torch.nn import functional as F


class MultiTaskHeads(nn.Module):
    """Prediction heads sharing a fused feature vector."""

    def __init__(
        self,
        input_dim: int,
        num_waypoints: int,
        num_modes: int,
        stop_enabled: bool = True,
        mode_enabled: bool = True,
        direct_control_aux: bool = True,
    ) -> None:
        super().__init__()
        hidden = input_dim
        self.num_waypoints = num_waypoints
        self.shared = nn.Sequential(
            nn.Linear(input_dim, hidden),
            nn.ReLU(inplace=True),
            nn.Dropout(0.1),
        )
        self.waypoint = nn.Linear(hidden, num_waypoints * 2)
        self.speed = nn.Linear(hidden, 1)
        self.stop = nn.Linear(hidden, 1) if stop_enabled else None
        self.mode = nn.Linear(hidden, num_modes) if mode_enabled else None
        self.direct_control = nn.Linear(hidden, 2) if direct_control_aux else None

    def forward(self, feature: torch.Tensor) -> dict[str, torch.Tensor]:
        shared = self.shared(feature)
        output: dict[str, torch.Tensor] = {
            "waypoints": self.waypoint(shared).view(-1, self.num_waypoints, 2),
            "target_speed": F.softplus(self.speed(shared)),
        }
        if self.stop is not None:
            output["stop_logit"] = self.stop(shared)
        if self.mode is not None:
            output["mode_logits"] = self.mode(shared)
        if self.direct_control is not None:
            output["direct_control"] = self.direct_control(shared)
        return output
