from dataclasses import replace

import pytest

from aic_transfuser_lite.data.calibration.lateral import LateralCalibration
from aic_transfuser_lite.data.calibration.longitudinal import LongitudinalModeFit
from aic_transfuser_lite.data.calibration.stability import (
    CalibrationStabilityLimits,
    evaluate_lateral_stability,
    evaluate_longitudinal_stability,
)


def _limits(*, yaw: float | None = None) -> CalibrationStabilityLimits:
    return CalibrationStabilityLimits(0.02, 0.05, 0.20, 0.10, 0.8, 0.7, 50, yaw)


def _lateral(**changes: object) -> LateralCalibration:
    value = LateralCalibration(
        pure_delay_sec=0.0, time_constant_sec=0.5, gain=1.0, bias_rad=0.0,
        valid_speed_range_mps=(0.1, 1.0), nrmse=0.65, yaw_rate_nrmse=0.78,
        correlation_peak=0.98, dynamic_sample_count=250, total_sample_count=400,
        excluded_sample_count=10, individually_valid=True, validity_reasons=(),
        source_method="test",
    )
    return replace(value, **changes)


def _longitudinal(mode: str = "brake", **changes: object) -> LongitudinalModeFit:
    value = LongitudinalModeFit(
        mode=mode, pure_delay_sec=0.0, time_constant_sec=0.12, gain=0.72,
        bias_mps2=-0.33, valid_speed_range_mps=(0.1, 0.8),
        command_range_mps2=(-1.0, -0.2), rmse_mps2=0.1, nrmse=0.47,
        correlation_peak=0.88, mode_sample_count=56, total_sample_count=500,
        excluded_actual_accel_count=0, individually_valid=True, validity_reasons=(),
    )
    return replace(value, **changes)


def test_measured_steering_and_brake_cohorts_pass_declared_gates() -> None:
    steering = evaluate_lateral_stability(
        [_lateral(), _lateral(time_constant_sec=0.5, nrmse=0.66, yaw_rate_nrmse=0.79)],
        _limits(yaw=0.8),
    )
    brake = evaluate_longitudinal_stability(
        [
            _longitudinal(),
            _longitudinal(time_constant_sec=0.14, gain=0.76, bias_mps2=-0.30),
            _longitudinal(time_constant_sec=0.14, gain=0.64, bias_mps2=-0.37, nrmse=0.56),
        ],
        _limits(),
        mode="brake",
    )
    assert steering.passed and steering.reasons == ()
    assert brake.passed and brake.relative_gain_span == pytest.approx((0.76 - 0.64) / 0.72)


def test_unstable_gain_returns_structured_reason() -> None:
    result = evaluate_longitudinal_stability(
        [_longitudinal(), _longitudinal(gain=0.4)], _limits(), mode="brake"
    )
    assert not result.passed
    assert "relative_gain_span" in result.reasons


@pytest.mark.parametrize("failure", ["one", "invalid", "mixed"])
def test_stability_rejects_non_independent_or_invalid_inputs(failure: str) -> None:
    cohorts = [_longitudinal(), _longitudinal()]
    if failure == "one":
        cohorts.pop()
    elif failure == "invalid":
        cohorts[1] = _longitudinal(individually_valid=False)
    else:
        cohorts[1] = _longitudinal(mode="drive")
    with pytest.raises(ValueError):
        evaluate_longitudinal_stability(cohorts, _limits(), mode="brake")
