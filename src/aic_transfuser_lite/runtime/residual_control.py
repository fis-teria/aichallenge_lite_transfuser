from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .authority import DebugModelControl


@dataclass(frozen=True)
class ExternalControllerCommand:
    """Primary external-controller proposal in rad, m/s, and m/s^2."""

    steering_rad: float
    speed_mps: float
    acceleration_mps2: float

    def validate(self) -> None:
        values = (self.steering_rad, self.speed_mps, self.acceleration_mps2)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("external controller command must be finite")
        if self.speed_mps < 0.0:
            raise ValueError("external controller speed must be non-negative")


@dataclass(frozen=True)
class ResidualLimits:
    max_abs_steering_residual_rad: float
    max_abs_speed_residual_mps: float
    max_abs_acceleration_residual_mps2: float
    authoritative: bool
    source: str

    def validate(self) -> None:
        values = (
            self.max_abs_steering_residual_rad,
            self.max_abs_speed_residual_mps,
            self.max_abs_acceleration_residual_mps2,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError("residual hard limits must be finite and positive")
        if not self.authoritative or not self.source.strip():
            raise ValueError("authoritative residual hard limits are required")


@dataclass(frozen=True)
class AppliedResidual:
    steering_rad: float
    speed_mps: float
    acceleration_mps2: float


@dataclass(frozen=True)
class ResidualBlendResult:
    command: ExternalControllerCommand
    applied_residual: AppliedResidual
    enabled: bool
    external_controller_primary: bool = True
    requires_safety_supervisor: bool = True


def blend_bounded_residual(
    external: ExternalControllerCommand,
    model: DebugModelControl | None,
    *,
    enabled: bool,
    limits: ResidualLimits | None,
) -> ResidualBlendResult:
    """Blend a clipped model-minus-external residual without replacing primary."""

    external.validate()
    if not enabled:
        return ResidualBlendResult(
            command=external,
            applied_residual=AppliedResidual(0.0, 0.0, 0.0),
            enabled=False,
        )
    if limits is None:
        raise ValueError("enabled residual requires hard limits")
    limits.validate()
    if model is None:
        raise ValueError("enabled residual requires a model proposal")
    model_values = (model.steering_rad, model.speed_mps, model.acceleration_mps2)
    if model.authoritative:
        raise ValueError("model proposal must remain non-authoritative")
    if not all(math.isfinite(value) for value in model_values):
        raise ValueError("model residual proposal must be finite")
    if model.speed_mps < 0.0:
        raise ValueError("model residual speed must be non-negative")

    raw = np.asarray(model_values, dtype=np.float64) - np.asarray(
        (external.steering_rad, external.speed_mps, external.acceleration_mps2),
        dtype=np.float64,
    )
    maxima = np.asarray(
        (
            limits.max_abs_steering_residual_rad,
            limits.max_abs_speed_residual_mps,
            limits.max_abs_acceleration_residual_mps2,
        ),
        dtype=np.float64,
    )
    applied = np.clip(raw, -maxima, maxima)
    blended = np.asarray(
        (external.steering_rad, external.speed_mps, external.acceleration_mps2),
        dtype=np.float64,
    ) + applied
    return ResidualBlendResult(
        command=ExternalControllerCommand(
            steering_rad=float(blended[0]),
            speed_mps=float(blended[1]),
            acceleration_mps2=float(blended[2]),
        ),
        applied_residual=AppliedResidual(
            steering_rad=float(applied[0]),
            speed_mps=float(applied[1]),
            acceleration_mps2=float(applied[2]),
        ),
        enabled=True,
    )
