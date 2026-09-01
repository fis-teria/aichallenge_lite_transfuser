from __future__ import annotations

from typing import Any

import torch
from torch import nn

from .camera_encoder import CameraEncoder
from .ego_encoder import EgoEncoder
from .heads import MultiTaskHeads
from .lidar_encoder import Lidar1DEncoder


class LateFusionModel(nn.Module):
    """Late-fusion baseline for Camera, 2D LiDAR and ego state.

    Inputs are image ``[B, 3, H, W]``, normalized LaserScan ``[B, P]`` and
    ego state ``[B, D]``. Each modality is independently encoded and globally
    pooled before fusion, so no cross-modal token attention is performed.
    """

    def __init__(self, config: dict[str, Any]) -> None:
        super().__init__()
        data_cfg = config["data"]
        model_cfg = config["model"]
        hidden = int(model_cfg.get("hidden_dim", 128))
        camera_cfg = model_cfg.get("camera", {})
        lidar_cfg = model_cfg.get("lidar", {})
        heads_cfg = model_cfg.get("heads", {})

        self.camera = CameraEncoder(
            output_dim=hidden,
            token_h=int(camera_cfg.get("token_h", 4)),
            token_w=int(camera_cfg.get("token_w", 4)),
            pretrained=bool(camera_cfg.get("pretrained", False)),
        )
        self.lidar = Lidar1DEncoder(
            output_dim=hidden,
            token_count=int(lidar_cfg.get("token_count", 32)),
        )
        self.ego = EgoEncoder(
            input_dim=int(data_cfg.get("ego_dim", 5)),
            hidden_dim=int(model_cfg.get("ego", {}).get("hidden_dim", 64)),
            output_dim=hidden,
        )
        self.fuse = nn.Sequential(
            nn.Linear(hidden * 3, hidden),
            nn.LayerNorm(hidden),
            nn.ReLU(inplace=True),
            nn.Dropout(float(model_cfg.get("dropout", 0.1))),
        )
        self.heads = MultiTaskHeads(
            input_dim=hidden,
            num_waypoints=int(data_cfg.get("num_waypoints", 6)),
            num_modes=len(data_cfg.get("mode_classes", {"follow": 0})),
            stop_enabled=bool(heads_cfg.get("stop", True)),
            mode_enabled=bool(heads_cfg.get("behavior_mode", True)),
            direct_control_aux=bool(heads_cfg.get("direct_control_aux", True)),
        )

    def forward(
        self,
        image: torch.Tensor,
        lidar: torch.Tensor,
        ego: torch.Tensor,
    ) -> dict[str, torch.Tensor]:
        batch = image.shape[0]
        if lidar.shape[0] != batch or ego.shape[0] != batch:
            raise ValueError(
                "Camera, LiDAR and ego batch sizes must match: "
                f"image={image.shape[0]}, lidar={lidar.shape[0]}, ego={ego.shape[0]}"
            )
        image_feature = self.camera(image).mean(dim=1)
        lidar_feature = self.lidar(lidar).mean(dim=1)
        ego_feature = self.ego(ego).squeeze(1)
        fused = self.fuse(
            torch.cat([image_feature, lidar_feature, ego_feature], dim=1)
        )
        return self.heads(fused)
