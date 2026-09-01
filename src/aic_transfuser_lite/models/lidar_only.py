from __future__ import annotations

from typing import Any

import torch
from torch import nn

from .ego_encoder import EgoEncoder
from .heads import MultiTaskHeads
from .lidar_encoder import Lidar1DEncoder


class LidarOnlyModel(nn.Module):
    """Lightweight LiDAR + ego baseline."""

    def __init__(self, config: dict[str, Any]) -> None:
        super().__init__()
        data_cfg = config["data"]
        model_cfg = config["model"]
        hidden = int(model_cfg.get("hidden_dim", 128))
        lidar_cfg = model_cfg.get("lidar", {})
        self.lidar = Lidar1DEncoder(
            output_dim=hidden,
            token_count=int(lidar_cfg.get("token_count", 64)),
        )
        self.ego = EgoEncoder(
            input_dim=int(data_cfg.get("ego_dim", 5)),
            hidden_dim=64,
            output_dim=hidden,
        )
        self.fuse = nn.Sequential(
            nn.Linear(hidden * 2, hidden),
            nn.ReLU(inplace=True),
            nn.Dropout(0.1),
        )
        heads_cfg = model_cfg.get("heads", {})
        self.heads = MultiTaskHeads(
            input_dim=hidden,
            num_waypoints=int(data_cfg.get("num_waypoints", 6)),
            num_modes=len(data_cfg.get("mode_classes", {"follow": 0})),
            stop_enabled=bool(heads_cfg.get("stop", True)),
            mode_enabled=bool(heads_cfg.get("behavior_mode", True)),
            direct_control_aux=bool(heads_cfg.get("direct_control_aux", True)),
        )

    def forward(self, lidar: torch.Tensor, ego: torch.Tensor) -> dict[str, torch.Tensor]:
        lidar_feature = self.lidar(lidar).mean(dim=1)
        ego_feature = self.ego(ego).squeeze(1)
        return self.heads(self.fuse(torch.cat([lidar_feature, ego_feature], dim=1)))
