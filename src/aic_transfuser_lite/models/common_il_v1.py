"""Common XY/speed/stop heads over the existing causal sensor fusion."""
from __future__ import annotations

from dataclasses import replace
from typing import Any

import torch
from torch import nn

from aic_transfuser_lite.contracts.common_il_v1 import CommonILPrediction, common_il_contract
from aic_transfuser_lite.contracts.model_batch_v3 import ModelBatchV3
from aic_transfuser_lite.training.time_config_v1 import TimeModelConfig
from .dinov3_camera_encoder import build_camera_encoder, resolve_camera_encoder_config
from .time_backbone_v1 import TimeBackboneV1


class CommonILHeadsV1(nn.Module):
    def __init__(self, hidden_dim: int) -> None:
        super().__init__()
        if type(hidden_dim) is not int or hidden_dim <= 0:
            raise ValueError("positive integer hidden_dim required")
        self.hidden_dim = hidden_dim
        self.decoder = nn.GRUCell(2, hidden_dim)
        self.delta_head = nn.Linear(hidden_dim, 2)
        self.speed_head = nn.Linear(hidden_dim, 1)
        self.stop_head = nn.Linear(hidden_dim, 1)

    def forward(self, features: torch.Tensor) -> CommonILPrediction:
        """Decode input-only fused features [B,D]; no future action/observation."""
        if features.ndim != 2 or features.shape[1] != self.hidden_dim:
            raise ValueError("common IL features must be [B,hidden_dim]")
        hidden = features
        xy = hidden.new_zeros((features.shape[0], 2))
        points = []
        for _ in range(6):
            hidden = self.decoder(xy, hidden)
            xy = xy + self.delta_head(hidden)
            points.append(xy)
        prediction = CommonILPrediction(torch.stack(points, dim=1), self.speed_head(features), self.stop_head(features))
        prediction.validate()
        return prediction


class CommonILV1(nn.Module):
    """New model/checkpoint family; legacy TimePathV1 remains unchanged.

    Construct the shared fusion and heads with independent fixed RNG streams.
    Replacing the camera cannot alter initialization of LiDAR/fusion/IL heads.
    """

    def __init__(self, config: TimeModelConfig, *, initialization_seed: int = 42) -> None:
        super().__init__()
        config.validate()
        if type(initialization_seed) is not int or initialization_seed < 0:
            raise ValueError("initialization_seed must be nonnegative integer")
        self.config = TimeModelConfig.from_dict(config.to_dict())
        self.initialization_seed = initialization_seed
        kwargs = config.model_kwargs()
        kwargs.pop("camera_encoder", None)
        # The reused V3 constructor has an unused 15-point baseline head. As
        # with TimePathV1, remove that head after construction and use our own.
        kwargs.pop("trajectory_steps")
        with torch.random.fork_rng(devices=[]):
            torch.set_rng_state(torch.Generator().manual_seed(initialization_seed).get_state())
            self.backbone = TimeBackboneV1(**kwargs)
            self.backbone.trajectory_head = None
            self.backbone.speed_profile_head = None
            if config.camera_encoder is not None:
                camera = resolve_camera_encoder_config(config.camera_encoder, output_dim=config.hidden_dim,
                    token_h=config.camera_tokens_hw[0], token_w=config.camera_tokens_hw[1],
                    image_height=config.image_height, image_width=config.image_width)
                if camera["backbone"] != "resnet18" or camera.get("pretrained", False):
                    torch.set_rng_state(torch.Generator().manual_seed(initialization_seed + 1).get_state())
                    self.backbone.camera = build_camera_encoder(camera)
            torch.set_rng_state(torch.Generator().manual_seed(initialization_seed + 2).get_state())
            self.heads = CommonILHeadsV1(config.hidden_dim)

    def get_extra_state(self) -> dict[str, Any]:
        return {"contract": common_il_contract(), "model_config": self.config.to_dict(),
                "initialization_seed": self.initialization_seed}

    def set_extra_state(self, state: dict[str, Any]) -> None:
        if state != self.get_extra_state():
            raise ValueError("common IL model identity mismatch")

    def forward_features(self, inputs: ModelBatchV3) -> torch.Tensor:
        """Causal input-only [B,D] features; training labels are rejected."""
        if not isinstance(inputs, ModelBatchV3) or inputs.targets is not None:
            raise ValueError("common IL policy accepts input-only ModelBatchV3")
        inputs = replace(inputs, requested_outputs=frozenset({"trajectory"}))
        if not self.config.use_command_history:
            inputs = replace(inputs, command_history=torch.zeros_like(inputs.command_history),
                             command_mask=torch.zeros_like(inputs.command_mask))
        return self.backbone.forward_features(inputs)

    def forward(self, inputs: ModelBatchV3) -> CommonILPrediction:
        return self.heads(self.forward_features(inputs))
