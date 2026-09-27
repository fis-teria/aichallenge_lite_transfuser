"""Separate inference outputs and supervised labels for the common IL policy."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import torch


def common_il_contract() -> dict[str, Any]:
    return {
        "format": "common_il_v1", "waypoint_time_sec": [i / 2 for i in range(1, 7)],
        "waypoint_frame": "base_link_at_observation", "waypoint_unit": "m",
        "target_speed_unit": "m/s", "target_speed_meaning": "measured_signed_longitudinal_at_t_plus_0.5s",
        "stop_meaning": "explicit_environment_stop_intent_at_observation",
        "stop_probability": "sigmoid_of_stop_logit", "runtime_ready": False,
    }


@dataclass(frozen=True)
class CommonILPrediction:
    """XY [B,6,2] m at 0.5..3.0 s, speed [B,1] m/s, stop logit [B,1]."""

    waypoints_m: torch.Tensor
    target_speed_mps: torch.Tensor
    stop_logit: torch.Tensor

    @property
    def stop_probability(self) -> torch.Tensor:
        return self.stop_logit.sigmoid()

    def validate(self) -> None:
        if self.waypoints_m.ndim != 3 or self.waypoints_m.shape[1:] != (6, 2):
            raise ValueError("waypoints must be [B,6,2] metres")
        batch = self.waypoints_m.shape[0]
        if batch < 1:
            raise ValueError("empty common IL prediction")
        for name, shape in (("waypoints_m", (batch, 6, 2)), ("target_speed_mps", (batch, 1)),
                            ("stop_logit", (batch, 1))):
            value = getattr(self, name)
            if (value.shape != shape or not value.is_floating_point()
                    or value.device != self.waypoints_m.device or not torch.isfinite(value).all()):
                raise ValueError(f"invalid common IL output {name}")


@dataclass(frozen=True)
class CommonILTargets:
    """Training-only labels; masked values may be NaN and never enter arithmetic."""

    waypoints_m: torch.Tensor
    waypoint_mask: torch.Tensor
    target_speed_mps: torch.Tensor
    target_speed_mask: torch.Tensor
    stop_target: torch.Tensor
    stop_mask: torch.Tensor

    def validate(self) -> None:
        if self.waypoints_m.ndim != 3 or self.waypoints_m.shape[1:] != (6, 2):
            raise ValueError("waypoint labels must be [B,6,2]")
        batch = self.waypoints_m.shape[0]
        if batch < 1:
            raise ValueError("empty common IL targets")
        for name, mask_name, shape in (("waypoints_m", "waypoint_mask", (batch, 6, 2)),
                ("target_speed_mps", "target_speed_mask", (batch, 1)),
                ("stop_target", "stop_mask", (batch, 1))):
            value, mask = getattr(self, name), getattr(self, mask_name)
            mask_shape = shape[:2]
            if (value.shape != shape or not value.is_floating_point()
                    or mask.shape != mask_shape or mask.dtype != torch.bool
                    or value.device != self.waypoints_m.device or mask.device != value.device):
                raise ValueError(f"common IL target shape/dtype/device mismatch: {name}")
            if not torch.isfinite(value[mask]).all():
                raise ValueError(f"nonfinite supported common IL target: {name}")
        stop = self.stop_target[self.stop_mask]
        if not ((stop == 0) | (stop == 1)).all():
            raise ValueError("stop target requires explicit binary intent")
