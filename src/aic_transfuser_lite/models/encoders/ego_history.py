from __future__ import annotations

import torch
from torch import nn

from ..temporal.gru import MaskedGRUTemporalEncoder


class EgoHistoryEncoder(nn.Module):
    """Encode ego SI features and command history with explicit validity masks."""

    def __init__(self, ego_dim: int, hidden_dim: int) -> None:
        super().__init__()
        if ego_dim <= 0:
            raise ValueError("ego_dim must be positive")
        self.ego_dim = ego_dim
        self.ego_projection = nn.Linear(ego_dim, hidden_dim)
        self.command_projection = nn.Linear(3, hidden_dim)
        self.ego_temporal = MaskedGRUTemporalEncoder(hidden_dim, hidden_dim)
        self.command_temporal = MaskedGRUTemporalEncoder(
            hidden_dim, hidden_dim, allow_empty=True
        )
        self.output = nn.Linear(hidden_dim * 2, hidden_dim)

    def forward(
        self,
        ego: torch.Tensor,
        ego_feature_mask: torch.Tensor,
        command: torch.Tensor,
        command_mask: torch.Tensor,
    ) -> torch.Tensor:
        if ego.ndim != 3 or ego.shape[-1] != self.ego_dim:
            raise ValueError("ego history must be [B,T,ego_dim]")
        if ego_feature_mask.shape != ego.shape or ego_feature_mask.dtype != torch.bool:
            raise ValueError("ego feature mask must be bool and match ego history")
        if command.ndim != 3 or command.shape[-1] != 3:
            raise ValueError("command history must be [B,T,3]")
        ego_step_mask = ego_feature_mask.all(dim=-1)
        ego_values = self.ego_projection(ego * ego_feature_mask.to(ego.dtype))
        command_values = self.command_projection(command)
        ego_hidden = self.ego_temporal(ego_values, ego_step_mask)
        command_hidden = self.command_temporal(command_values, command_mask)
        return self.output(torch.cat((ego_hidden, command_hidden), dim=-1))
