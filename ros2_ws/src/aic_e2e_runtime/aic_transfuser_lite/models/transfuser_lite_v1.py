from __future__ import annotations

from typing import Any

import torch
from torch import nn

from .camera_encoder import CameraEncoder
from .ego_encoder import EgoEncoder
from .fusion import TokenFusionTransformer
from .heads import MultiTaskHeads
from .lidar_encoder import Lidar1DEncoder
from .transfuser_lite import _build_component


class AICTransFuserLiteV1(nn.Module):
    """Strict static Dataset-v2 TransFuser Lite model boundary."""

    def __init__(self, config: dict[str, Any]) -> None:
        super().__init__()
        data_cfg = config["data"]
        model_cfg = config["model"]
        camera_cfg = model_cfg["camera"]
        lidar_cfg = model_cfg["lidar"]
        fusion_cfg = model_cfg["fusion"]
        heads_cfg = model_cfg["heads"]
        hidden = int(model_cfg["hidden_dim"])
        initialization = str(model_cfg["initialization"])
        base_seed = int(config["project"]["seed"])

        self.image_height = int(data_cfg["image_height"])
        self.image_width = int(data_cfg["image_width"])
        self.lidar_points = int(data_cfg["lidar_points"])
        self.ego_dim = int(data_cfg["ego_dim"])
        self.num_waypoints = int(data_cfg["num_waypoints"])

        self.camera = _build_component(
            initialization,
            base_seed + 101,
            lambda: CameraEncoder(
                output_dim=hidden,
                token_h=int(camera_cfg["token_h"]),
                token_w=int(camera_cfg["token_w"]),
                pretrained=bool(camera_cfg["pretrained"]),
            ),
        )
        self.lidar = _build_component(
            initialization,
            base_seed + 102,
            lambda: Lidar1DEncoder(
                output_dim=hidden,
                token_count=int(lidar_cfg["token_count"]),
                input_channels=2,
                lidar_points=self.lidar_points,
                angle_min_rad=float(data_cfg["lidar_angle_min_rad"]),
                angle_increment_rad=float(data_cfg["lidar_angle_increment_rad"]),
                use_angle_encoding=True,
            ),
        )
        self.ego = _build_component(
            initialization,
            base_seed + 103,
            lambda: EgoEncoder(
                input_dim=self.ego_dim,
                hidden_dim=int(model_cfg["ego"]["hidden_dim"]),
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
                depth=int(fusion_cfg["depth"]),
                heads=int(fusion_cfg["heads"]),
                mlp_ratio=int(fusion_cfg["mlp_ratio"]),
                dropout=float(fusion_cfg["dropout"]),
                pooling=str(fusion_cfg["pooling"]),
            ),
        )
        self.heads = _build_component(
            initialization,
            base_seed + 106,
            lambda: MultiTaskHeads(
                input_dim=hidden,
                num_waypoints=self.num_waypoints,
                num_modes=len(data_cfg["mode_classes"]),
                stop_enabled=bool(heads_cfg["stop"]),
                mode_enabled=bool(heads_cfg["behavior_mode"]),
                direct_control_aux=bool(heads_cfg["direct_control_aux"]),
            ),
        )

    def forward(
        self,
        image: torch.Tensor,
        lidar: torch.Tensor,
        ego: torch.Tensor,
    ) -> dict[str, torch.Tensor]:
        if image.ndim != 4 or tuple(image.shape[1:]) != (
            3,
            self.image_height,
            self.image_width,
        ):
            raise ValueError(
                f"Expected image [B,3,{self.image_height},{self.image_width}], "
                f"got {tuple(image.shape)}"
            )
        if lidar.ndim != 3 or tuple(lidar.shape[1:]) != (2, self.lidar_points):
            raise ValueError(
                f"Expected lidar [B,2,{self.lidar_points}], got {tuple(lidar.shape)}"
            )
        if ego.ndim != 2 or ego.shape[1] != self.ego_dim:
            raise ValueError(
                f"Expected ego [B,{self.ego_dim}], got {tuple(ego.shape)}"
            )
        if not (image.shape[0] == lidar.shape[0] == ego.shape[0]):
            raise ValueError("Image, LiDAR, and ego batch sizes differ")

        image_tokens = self.camera(image)
        lidar_tokens = self.lidar(lidar)
        ego_token = self.ego(ego)
        _, pooled = self.fusion(image_tokens, lidar_tokens, ego_token)
        output = self.heads(pooled)
        expected_keys = {"waypoints", "target_speed"}
        if set(output) != expected_keys:
            raise RuntimeError(
                f"Static v1 output contract drifted: expected {expected_keys}, got {set(output)}"
            )
        if output["waypoints"].shape != (image.shape[0], self.num_waypoints, 2):
            raise RuntimeError("Static v1 waypoint output shape drifted")
        if output["target_speed"].shape != (image.shape[0], 1):
            raise RuntimeError("Static v1 target-speed output shape drifted")
        return output
