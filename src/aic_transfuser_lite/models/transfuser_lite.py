from __future__ import annotations

from collections.abc import Callable
from typing import Any

import torch
from torch import nn


def _build_component(
    strategy: str, seed: int, factory: Callable[[], nn.Module]
) -> nn.Module:
    if strategy == "legacy_global":
        return factory()
    if strategy != "component_seeded_v1":
        raise ValueError(f"Unsupported model.initialization={strategy!r}")
    with torch.random.fork_rng(devices=[]):
        torch.manual_seed(seed)
        return factory()

from .camera_encoder import CameraEncoder
from .ego_encoder import EgoEncoder
from .fusion import TokenFusionTransformer
from .heads import MultiTaskHeads
from .lidar_encoder import Lidar1DEncoder


class AICTransFuserLite(nn.Module):
    """Camera + 2D LiDAR + ego-state TransFuser-style model."""

    def __init__(self, config: dict[str, Any]) -> None:
        super().__init__()
        data_cfg = config["data"]
        model_cfg = config["model"]
        hidden = int(model_cfg.get("hidden_dim", 128))
        camera_cfg = model_cfg.get("camera", {})
        lidar_cfg = model_cfg.get("lidar", {})
        fusion_cfg = model_cfg.get("fusion", {})
        heads_cfg = model_cfg.get("heads", {})
        initialization = str(model_cfg.get("initialization", "legacy_global"))
        base_seed = int(config.get("project", {}).get("seed", 42))

        self.camera = _build_component(
            initialization,
            base_seed + 101,
            lambda: CameraEncoder(
                output_dim=hidden,
                token_h=int(camera_cfg.get("token_h", 8)),
                token_w=int(camera_cfg.get("token_w", 8)),
                pretrained=bool(camera_cfg.get("pretrained", False)),
            ),
        )
        self.lidar = _build_component(
            initialization,
            base_seed + 102,
            lambda: Lidar1DEncoder(
                output_dim=hidden,
                token_count=int(lidar_cfg.get("token_count", 64)),
            ),
        )
        self.ego = _build_component(
            initialization,
            base_seed + 103,
            lambda: EgoEncoder(
                input_dim=int(data_cfg.get("ego_dim", 5)),
                hidden_dim=int(model_cfg.get("ego", {}).get("hidden_dim", 64)),
                output_dim=hidden,
            ),
        )
        if initialization == "component_seeded_v1":
            with torch.random.fork_rng(devices=[]):
                torch.manual_seed(base_seed + 104)
                self.ego.network[-1].reset_parameters()
        self.fusion = _build_component(
            initialization,
            base_seed + 105,
            lambda: TokenFusionTransformer(
                dim=hidden,
                image_tokens=self.camera.token_count,
                lidar_tokens=self.lidar.token_count,
                depth=int(fusion_cfg.get("depth", 3)),
                heads=int(fusion_cfg.get("heads", 4)),
                mlp_ratio=int(fusion_cfg.get("mlp_ratio", 4)),
                dropout=float(fusion_cfg.get("dropout", 0.1)),
                pooling=str(fusion_cfg.get("pooling", "ego")),
            ),
        )
        self.heads = _build_component(
            initialization,
            base_seed + 106,
            lambda: MultiTaskHeads(
                input_dim=hidden,
                num_waypoints=int(data_cfg.get("num_waypoints", 6)),
                num_modes=len(data_cfg.get("mode_classes", {"follow": 0})),
                stop_enabled=bool(heads_cfg.get("stop", True)),
                mode_enabled=bool(heads_cfg.get("behavior_mode", True)),
                direct_control_aux=bool(heads_cfg.get("direct_control_aux", True)),
            ),
        )

    def forward(
        self,
        image: torch.Tensor,
        lidar: torch.Tensor,
        ego: torch.Tensor,
    ) -> dict[str, torch.Tensor]:
        image_tokens = self.camera(image)
        lidar_tokens = self.lidar(lidar)
        ego_token = self.ego(ego)
        _, pooled = self.fusion(image_tokens, lidar_tokens, ego_token)
        return self.heads(pooled)
