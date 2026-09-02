from __future__ import annotations

from dataclasses import replace
import json

import numpy as np
import pytest

from aic_transfuser_lite.data.delay_estimation import DelayEstimationConfig
from aic_transfuser_lite.data.calibration.artifact import (
    CalibrationPromotion,
    build_calibration_artifact,
    load_calibration_artifact,
    write_calibration_artifact,
)
from aic_transfuser_lite.data.calibration.lateral import (
    LateralCalibration,
    fit_lateral_calibration,
)
from aic_transfuser_lite.data.calibration.longitudinal import (
    LongitudinalFitConfig,
    derive_actual_acceleration,
    fit_longitudinal_mode,
)


def _simulate(
    timestamps: np.ndarray,
    command: np.ndarray,
    *,
    delay_sec: float,
    time_constant_sec: float,
    gain: float,
    bias: float,
) -> np.ndarray:
    indices = np.searchsorted(timestamps, timestamps - delay_sec, side="right") - 1
    delayed = command[np.clip(indices, 0, len(command) - 1)]
    filtered = np.empty_like(delayed)
    filtered[0] = delayed[0]
    for index in range(1, len(filtered)):
        alpha = 1.0 - np.exp(
            -(timestamps[index] - timestamps[index - 1]) / time_constant_sec
        )
        filtered[index] = filtered[index - 1] + alpha * (
            delayed[index] - filtered[index - 1]
        )
    return gain * filtered + bias


@pytest.mark.parametrize(
    ("mode", "offset", "amplitude"),
    [("drive", 1.0, 0.6), ("brake", -1.0, 0.6)],
)
def test_longitudinal_fit_separates_delay_lag_gain_and_bias(
    mode: str, offset: float, amplitude: float
) -> None:
    timestamps = np.arange(0.0, 60.0, 0.1)
    command = offset + amplitude * np.sin(timestamps * 0.7)
    if mode == "brake":
        command = np.minimum(command, -0.2)
    actual = _simulate(
        timestamps,
        command,
        delay_sec=0.2,
        time_constant_sec=0.3,
        gain=0.8,
        bias=0.05,
    )
    fit = fit_longitudinal_mode(
        timestamps,
        command,
        actual,
        np.linspace(1.0, 8.0, len(timestamps)),
        mode=mode,  # type: ignore[arg-type]
        config=LongitudinalFitConfig(
            delay_max_sec=0.4,
            delay_step_sec=0.1,
            time_constant_min_sec=0.1,
            time_constant_max_sec=0.6,
            time_constant_step_sec=0.1,
            minimum_mode_samples=100,
            minimum_correlation=0.8,
        ),
    )

    assert fit.pure_delay_sec == pytest.approx(0.2, abs=0.11)
    assert fit.time_constant_sec == pytest.approx(0.3, abs=0.11)
    assert fit.gain == pytest.approx(0.8, abs=0.08)
    assert fit.bias_mps2 == pytest.approx(0.05, abs=0.08)
    assert fit.individually_valid


def test_acceleration_derivative_contract_and_negative_inputs() -> None:
    timestamps = np.arange(0.0, 1.0, 0.1)
    acceleration = derive_actual_acceleration(timestamps, timestamps**2, smoothing_samples=1)
    np.testing.assert_allclose(acceleration[1:-1], 2.0 * timestamps[1:-1], atol=1e-10)
    with pytest.raises(ValueError, match="positive odd"):
        derive_actual_acceleration(timestamps, timestamps, smoothing_samples=2)
    with pytest.raises(ValueError, match="strictly increasing"):
        derive_actual_acceleration(timestamps[::-1], timestamps)


def test_lateral_wrapper_preserves_delay_fitter_and_reports_exclusions() -> None:
    timestamps = np.arange(0.0, 60.0, 0.1)
    command = np.repeat(np.asarray([-0.25, 0.2, 0.0]), 200)
    actual = _simulate(
        timestamps,
        command,
        delay_sec=0.2,
        time_constant_sec=0.3,
        gain=1.0,
        bias=0.0,
    )
    speed = np.full_like(timestamps, 4.0)
    yaw_rate = speed * np.tan(actual) / 1.087
    speed[0] = -1.0
    yaw_rate[1] = 100.0
    fit = fit_lateral_calibration(
        timestamps,
        command,
        actual,
        speed,
        yaw_rate,
        wheelbase_m=1.087,
        config=DelayEstimationConfig(
            tau_max_sec=0.4,
            tau_step_sec=0.1,
            time_constant_min_sec=0.1,
            time_constant_max_sec=0.5,
            time_constant_step_sec=0.1,
            minimum_dynamic_samples=100,
        ),
    )

    assert fit.source_method == "command_to_actual_first_order_and_yaw"
    assert fit.pure_delay_sec == pytest.approx(0.2, abs=0.11)
    assert fit.time_constant_sec == pytest.approx(0.3, abs=0.11)
    assert fit.excluded_sample_count == 2
    assert fit.individually_valid


def test_longitudinal_quality_gate_rejects_uncorrelated_fit() -> None:
    timestamps = np.arange(0.0, 40.0, 0.1)
    command = 1.0 + 0.6 * np.sin(timestamps)
    actual = np.random.default_rng(7).normal(0.0, 1.0, len(timestamps))
    fit = fit_longitudinal_mode(
        timestamps,
        command,
        actual,
        np.full_like(timestamps, 3.0),
        mode="drive",
        config=LongitudinalFitConfig(
            delay_max_sec=0.2,
            delay_step_sec=0.1,
            time_constant_min_sec=0.1,
            time_constant_max_sec=0.3,
            time_constant_step_sec=0.1,
            minimum_mode_samples=100,
        ),
    )

    assert not fit.individually_valid
    assert any("correlation_peak" in reason or "nrmse" in reason for reason in fit.validity_reasons)


def _fit(mode: str):
    timestamps = np.arange(0.0, 20.0, 0.1)
    command = (1.0 if mode == "drive" else -1.0) + 0.5 * np.sin(timestamps)
    actual = _simulate(
        timestamps,
        command,
        delay_sec=0.1,
        time_constant_sec=0.2,
        gain=0.7,
        bias=0.0,
    )
    return fit_longitudinal_mode(
        timestamps,
        command,
        actual,
        np.linspace(0.0, 5.0, len(timestamps)),
        mode=mode,  # type: ignore[arg-type]
        config=LongitudinalFitConfig(
            delay_max_sec=0.2,
            delay_step_sec=0.1,
            time_constant_min_sec=0.1,
            time_constant_max_sec=0.3,
            time_constant_step_sec=0.1,
            minimum_mode_samples=50,
        ),
    )


def test_artifact_hash_roundtrip_and_tamper_rejection(tmp_path) -> None:
    lateral = LateralCalibration(
        pure_delay_sec=0.1,
        time_constant_sec=0.2,
        gain=1.0,
        bias_rad=0.0,
        valid_speed_range_mps=(0.0, 8.0),
        nrmse=0.1,
        yaw_rate_nrmse=0.2,
        correlation_peak=0.9,
        dynamic_sample_count=1000,
        total_sample_count=1200,
        excluded_sample_count=2,
        individually_valid=True,
        validity_reasons=(),
        source_method="command_to_actual_first_order_and_yaw",
    )
    artifact = build_calibration_artifact(
        source_run_hashes={"run_a": "a" * 64},
        vehicle_profile_sha256="b" * 64,
        steering=lateral,
        drive=_fit("drive"),
        brake=_fit("brake"),
    )
    path = tmp_path / "calibration.json"
    write_calibration_artifact(artifact, path)
    assert load_calibration_artifact(path) == artifact
    raw = json.loads(path.read_text(encoding="utf-8"))
    raw["drive"]["gain"] += 0.1
    path.write_text(json.dumps(raw), encoding="utf-8")
    with pytest.raises(ValueError, match="hash mismatch"):
        load_calibration_artifact(path)


def test_artifact_rejects_mode_swap_and_non_candidate_state() -> None:
    drive = _fit("drive")
    brake = _fit("brake")
    lateral = LateralCalibration(
        0.1, 0.2, 1.0, 0.0, (0.0, 5.0), 0.1, 0.2, 0.9, 100, 100, 0, True, (), "test"
    )
    with pytest.raises(ValueError, match="distinct modes"):
        build_calibration_artifact(
            source_run_hashes={"run": "a" * 64},
            vehicle_profile_sha256="b" * 64,
            steering=lateral,
            drive=brake,
            brake=drive,
        )
    with pytest.raises(ValueError, match="unsupported"):
        CalibrationPromotion(state="invalid").validate()
