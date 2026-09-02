from __future__ import annotations

import math

import pytest

from aic_transfuser_lite.runtime.authority import DebugModelControl
from aic_transfuser_lite.runtime.residual_control import (
    ExternalControllerCommand,
    ResidualLimits,
    blend_bounded_residual,
)


def _limits(**overrides: object) -> ResidualLimits:
    values: dict[str, object] = {
        "max_abs_steering_residual_rad": 0.05,
        "max_abs_speed_residual_mps": 0.5,
        "max_abs_acceleration_residual_mps2": 0.25,
        "authoritative": True,
        "source": "reviewed_residual_limits_v1",
    }
    values.update(overrides)
    return ResidualLimits(**values)  # type: ignore[arg-type]


def test_disabled_residual_returns_exact_external_baseline_object() -> None:
    baseline = ExternalControllerCommand(0.123456789, 3.25, -0.75)
    result = blend_bounded_residual(
        baseline,
        None,
        enabled=False,
        limits=None,
    )
    assert result.command is baseline
    assert result.applied_residual.steering_rad == 0.0
    assert result.applied_residual.speed_mps == 0.0
    assert result.applied_residual.acceleration_mps2 == 0.0
    assert result.external_controller_primary
    assert result.requires_safety_supervisor


def test_enabled_residual_is_hard_clipped_per_si_field() -> None:
    baseline = ExternalControllerCommand(0.1, 3.0, 0.5)
    model = DebugModelControl(-0.5, 8.0, -2.0)
    result = blend_bounded_residual(
        baseline,
        model,
        enabled=True,
        limits=_limits(),
    )
    assert result.command.steering_rad == pytest.approx(0.05)
    assert result.command.speed_mps == pytest.approx(3.5)
    assert result.command.acceleration_mps2 == pytest.approx(0.25)
    assert result.applied_residual.steering_rad == pytest.approx(-0.05)
    assert result.applied_residual.speed_mps == pytest.approx(0.5)
    assert result.applied_residual.acceleration_mps2 == pytest.approx(-0.25)
    assert result.external_controller_primary
    assert result.requires_safety_supervisor


@pytest.mark.parametrize(
    ("limits", "message"),
    [
        (None, "requires hard limits"),
        (_limits(authoritative=False), "authoritative"),
        (_limits(source=""), "authoritative"),
        (_limits(max_abs_speed_residual_mps=0.0), "finite and positive"),
        (_limits(max_abs_steering_residual_rad=float("inf")), "finite and positive"),
    ],
)
def test_enabled_residual_rejects_missing_or_untrusted_limits(
    limits: ResidualLimits | None, message: str
) -> None:
    with pytest.raises(ValueError, match=message):
        blend_bounded_residual(
            ExternalControllerCommand(0.0, 1.0, 0.0),
            DebugModelControl(0.0, 1.0, 0.0),
            enabled=True,
            limits=limits,
        )


@pytest.mark.parametrize(
    ("external", "model", "message"),
    [
        (
            ExternalControllerCommand(math.nan, 1.0, 0.0),
            DebugModelControl(0.0, 1.0, 0.0),
            "external controller command must be finite",
        ),
        (
            ExternalControllerCommand(0.0, -1.0, 0.0),
            DebugModelControl(0.0, 1.0, 0.0),
            "external controller speed",
        ),
        (
            ExternalControllerCommand(0.0, 1.0, 0.0),
            None,
            "requires a model proposal",
        ),
        (
            ExternalControllerCommand(0.0, 1.0, 0.0),
            DebugModelControl(0.0, math.inf, 0.0),
            "must be finite",
        ),
        (
            ExternalControllerCommand(0.0, 1.0, 0.0),
            DebugModelControl(0.0, 1.0, 0.0, authoritative=True),
            "non-authoritative",
        ),
    ],
)
def test_residual_rejects_invalid_primary_or_model(
    external: ExternalControllerCommand,
    model: DebugModelControl | None,
    message: str,
) -> None:
    with pytest.raises(ValueError, match=message):
        blend_bounded_residual(external, model, enabled=True, limits=_limits())
