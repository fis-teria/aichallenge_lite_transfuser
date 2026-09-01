from __future__ import annotations

from typing import Mapping, overload

import torch
from torch import nn

from aic_transfuser_lite.contracts.model_batch_v3 import ModelBatchV3
from aic_transfuser_lite.contracts.model_output_v3 import ModelOutputV3

from .transfuser_lite_v1 import AICTransFuserLiteV1


class TransFuserLiteV1Adapter(nn.Module):
    """Run the frozen V1 model from a current or temporal V3 tensor mapping.

    Accepted image shapes are ``[B,3,H,W]`` or ``[B,T,3,H,W]``. LiDAR accepts
    ``[B,2,P]`` or ``[B,T,2,P]`` and ego accepts ``[B,1]`` or ``[B,T,F]``.
    Temporal inputs use the last step and only the first ego feature (speed).
    The wrapped V1 module and state-dict keys are never mutated.
    """

    def __init__(self, v1_model: AICTransFuserLiteV1) -> None:
        super().__init__()
        self.v1_model = v1_model

    def forward(
        self, batch: ModelBatchV3 | Mapping[str, torch.Tensor]
    ) -> ModelOutputV3 | dict[str, torch.Tensor]:
        typed = isinstance(batch, ModelBatchV3)
        if typed:
            batch.validate()
            values: Mapping[str, torch.Tensor] = {
                "image": batch.image,
                "lidar": batch.lidar,
                "ego": batch.ego,
            }
        else:
            values = batch
        required = {"image", "lidar", "ego"}
        missing = sorted(required.difference(values))
        if missing:
            raise KeyError(f"V1 adapter batch is missing keys: {missing}")
        image = values["image"]
        lidar = values["lidar"]
        ego = values["ego"]
        if image.ndim == 5:
            image = image[:, -1]
        if lidar.ndim == 4:
            lidar = lidar[:, -1]
        if ego.ndim == 3:
            ego = ego[:, -1, :1]
        elif ego.ndim == 2:
            ego = ego[:, :1]
        else:
            raise ValueError(f"V1 adapter ego must be [B,F] or [B,T,F], got {tuple(ego.shape)}")
        output = self.v1_model(image, lidar, ego)
        if not typed:
            return output
        waypoints = output["waypoints"].unsqueeze(1)
        target_speed = output["target_speed"].unsqueeze(1).expand(
            -1, 1, waypoints.shape[2]
        )
        converted = ModelOutputV3(
            trajectory_xy=waypoints,
            trajectory_speed_mps=target_speed,
            candidate_logits=waypoints.new_zeros((waypoints.shape[0], 1)),
        )
        converted.validate(
            batch_size=waypoints.shape[0],
            candidates=1,
            trajectory_steps=waypoints.shape[2],
            requested_outputs=frozenset({"trajectory", "speed_profile"}),
        )
        return converted
