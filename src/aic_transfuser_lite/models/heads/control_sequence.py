from __future__ import annotations

from dataclasses import dataclass

import torch
from torch import nn


@dataclass(frozen=True)
class ControlTargetSelection:
    values: torch.Tensor
    mask: torch.Tensor
    provenance: tuple[str, ...]


def select_current_control_targets(
    *,
    nominal_values: torch.Tensor,
    nominal_mask: torch.Tensor,
    final_values: torch.Tensor,
    final_mask: torch.Tensor,
) -> ControlTargetSelection:
    """Prefer complete nominal labels, explicitly marking final-command fallback."""
    expected = nominal_values.shape
    if nominal_values.ndim != 2 or expected[1:] != (3,):
        raise ValueError("control values must be [B,3] as steering rad, speed m/s, acceleration m/s^2")
    if final_values.shape != expected or nominal_mask.shape != expected or final_mask.shape != expected:
        raise ValueError("nominal/final control target shapes differ")
    if nominal_mask.dtype != torch.bool or final_mask.dtype != torch.bool:
        raise ValueError("control target masks must be boolean")
    values = torch.zeros_like(nominal_values)
    mask = torch.zeros_like(nominal_mask)
    provenance: list[str] = []
    for row in range(expected[0]):
        if bool(nominal_mask[row].all()):
            values[row] = nominal_values[row]
            mask[row] = True
            provenance.append("nominal")
        elif bool(final_mask[row].all()):
            values[row] = final_values[row]
            mask[row] = True
            provenance.append("final_fallback")
        else:
            provenance.append("missing")
    return ControlTargetSelection(values, mask, tuple(provenance))


class ControlSequenceHead(nn.Module):
    """Bound the current [steering rad, speed m/s, acceleration m/s^2] proposal."""

    def __init__(
        self,
        input_dim: int,
        *,
        candidates: int = 1,
        max_steering_rad: float = 0.6,
        max_speed_mps: float = 12.0,
        min_acceleration_mps2: float = -4.0,
        max_acceleration_mps2: float = 2.0,
    ) -> None:
        super().__init__()
        if input_dim <= 0 or candidates <= 0:
            raise ValueError("control head dimensions must be positive")
        if max_steering_rad <= 0.0 or max_speed_mps <= 0.0:
            raise ValueError("control steering/speed bounds must be positive")
        if max_acceleration_mps2 <= min_acceleration_mps2:
            raise ValueError("acceleration bounds are invalid")
        self.candidates = candidates
        self.max_steering_rad = max_steering_rad
        self.max_speed_mps = max_speed_mps
        self.min_acceleration_mps2 = min_acceleration_mps2
        self.max_acceleration_mps2 = max_acceleration_mps2
        self.projection = nn.Linear(input_dim, candidates * 3)

    def forward(self, feature: torch.Tensor) -> torch.Tensor:
        if feature.ndim != 2:
            raise ValueError("control feature must be [B,D]")
        raw = self.projection(feature).view(feature.shape[0], self.candidates, 3)
        steering = torch.tanh(raw[..., 0]) * self.max_steering_rad
        speed = torch.sigmoid(raw[..., 1]) * self.max_speed_mps
        acceleration_unit = torch.sigmoid(raw[..., 2])
        acceleration = self.min_acceleration_mps2 + acceleration_unit * (
            self.max_acceleration_mps2 - self.min_acceleration_mps2
        )
        return torch.stack((steering, speed, acceleration), dim=-1)
