"""Proposed time-waypoint baseline; offline only, not an existing TransFuser model."""
from __future__ import annotations

from dataclasses import replace
from typing import Any

import torch
from torch import nn
from torch.nn import functional as F

from aic_transfuser_lite.contracts.model_batch_v3 import ModelBatchV3
from .full_control_lite_v3 import FullControlLiteV3

STEPS = 30
DT_SEC = 0.1


def time_contract() -> dict[str, Any]:
    """Serializable output identity; point i is (i+1)*0.1 s after observation."""
    return {"format": "proposed_time_path_v1", "time_sec": [i / 10 for i in range(1, 31)],
            "frame": "base_link_at_observation", "xy_unit": "m",
            "speed_unit": "m/s", "speed_meaning": "interval_chord_speed_norm",
            "runtime_ready": False}


def validate_time_contract(value: dict[str, Any]) -> None:
    if value != time_contract():
        raise ValueError("time trajectory contract mismatch")


def interval_speed(xy: torch.Tensor) -> torch.Tensor:
    """[B,30,2] metres -> [B,30] interval chord speed norm in m/s.

    This is not signed longitudinal velocity, a learned speed head or a stop probability.
    """
    if xy.ndim != 3 or xy.shape[1:] != (STEPS, 2) or not xy.is_floating_point():
        raise ValueError("time XY must be floating [B,30,2]")
    if not torch.isfinite(xy).all():
        raise ValueError("time XY must be finite")
    delta = torch.diff(torch.cat((torch.zeros_like(xy[:, :1]), xy), dim=1), dim=1)
    return torch.linalg.vector_norm(delta, dim=-1) / DT_SEC


class TimePathV1(nn.Module):
    """Existing V3 fusion + new autoregressive GRU XY decoder, output [B,30,2] m.

    No target point input is invented. Unlike original TransFuser this decoder is
    not goal-conditioned. No trained checkpoint or runtime adapter is provided.
    """
    def __init__(self, *, use_command_history: bool = True, **kwargs: Any) -> None:
        super().__init__()
        self.use_command_history = use_command_history
        self.backbone = FullControlLiteV3(**{**kwargs, "control_head_enabled": False,
            "control_sequence_head_enabled": False, "behavior_head_enabled": False})
        self.backbone.trajectory_head = None
        self.backbone.speed_profile_head = None
        hidden = int(kwargs.get("hidden_dim", 128))
        self.decoder = nn.GRUCell(2, hidden)
        self.delta_head = nn.Linear(hidden, 2)

    def forward(self, batch: ModelBatchV3) -> torch.Tensor:
        """Input-only features; teacher tensors never reach backbone or decoder."""
        inputs = replace(batch, targets=None, requested_outputs=frozenset({"trajectory"}))
        if not self.use_command_history:
            inputs = replace(inputs, command_history=torch.zeros_like(inputs.command_history),
                             command_mask=torch.zeros_like(inputs.command_mask))
        hidden = self.backbone.forward_features(inputs)
        xy = hidden.new_zeros((batch.batch_size, 2))
        points = []
        for _ in range(STEPS):
            hidden = self.decoder(xy, hidden)
            xy = xy + self.delta_head(hidden)
            points.append(xy)
        result = torch.stack(points, dim=1)
        if result.shape != (batch.batch_size, STEPS, 2) or not torch.isfinite(result).all():
            raise ValueError("invalid time trajectory output")
        return result


def masked_time_loss(prediction: torch.Tensor, target: torch.Tensor,
                     mask: torch.Tensor) -> torch.Tensor | None:
    """Mean L1 XY loss per supported anchor, then mean anchors; metres.

    Invalid targets may be NaN; sanitize BEFORE arithmetic for zero invalid gradient.
    Returns None when no anchor has support. No synthetic future is filled in.
    """
    if (prediction.ndim != 3 or prediction.shape[1:] != (STEPS, 2)
            or target.shape != prediction.shape or mask.shape != prediction.shape[:2]
            or mask.dtype != torch.bool):
        raise ValueError("time loss shape/mask mismatch")
    if not torch.isfinite(prediction).all() or not torch.isfinite(target[mask]).all():
        raise ValueError("nonfinite supported time loss value")
    supported = mask.any(dim=1)
    if not supported.any():
        return None
    safe_pred = torch.where(mask[..., None], prediction, 0.0)
    safe_target = torch.where(mask[..., None], target, 0.0)
    error = F.l1_loss(safe_pred, safe_target, reduction="none").mean(-1)
    return (error.sum(1) / mask.sum(1).clamp_min(1))[supported].mean()
