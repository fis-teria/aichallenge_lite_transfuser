from __future__ import annotations

import numpy as np
import pytest

from aic_transfuser_lite.data.delay_estimation import (
    DelayEstimationConfig,
    assess_delay_consistency,
    estimate_combined_yaw_delay,
    estimate_steering_delay,
)


def _delayed_first_order(
    timestamps_sec: np.ndarray,
    command_rad: np.ndarray,
    *,
    delay_sec: float,
    time_constant_sec: float,
) -> np.ndarray:
    delayed = np.interp(
        timestamps_sec - delay_sec,
        timestamps_sec,
        command_rad,
        left=command_rad[0],
        right=command_rad[-1],
    )
    output = np.empty_like(delayed)
    output[0] = delayed[0]
    for index in range(1, len(output)):
        dt_sec = timestamps_sec[index] - timestamps_sec[index - 1]
        alpha = 1.0 - np.exp(-dt_sec / time_constant_sec)
        output[index] = output[index - 1] + alpha * (delayed[index] - output[index - 1])
    return output


def test_known_delay_and_time_constant_are_recovered_within_twenty_ms() -> None:
    rng = np.random.default_rng(42)
    timestamps_sec = np.arange(0.0, 80.0, 0.02)
    held = rng.uniform(-0.35, 0.35, size=200)
    command_rad = np.repeat(held, 20)[: len(timestamps_sec)]
    actual_rad = _delayed_first_order(
        timestamps_sec,
        command_rad,
        delay_sec=0.12,
        time_constant_sec=0.18,
    )
    speed_mps = np.full_like(timestamps_sec, 4.0)
    yaw_rate_rps = speed_mps * np.tan(actual_rad) / 1.087
    config = DelayEstimationConfig(
        tau_min_sec=0.0,
        tau_max_sec=0.5,
        tau_step_sec=0.01,
        time_constant_min_sec=0.02,
        time_constant_max_sec=0.5,
        time_constant_step_sec=0.01,
        minimum_dynamic_samples=500,
        minimum_correlation=0.7,
    )

    result = estimate_steering_delay(
        timestamps_sec,
        command_rad,
        actual_rad,
        speed_mps,
        yaw_rate_rps,
        wheelbase_m=1.087,
        config=config,
    )

    assert result.delay_sec == pytest.approx(0.12, abs=0.02)
    assert result.time_constant_sec == pytest.approx(0.18, abs=0.02)
    assert result.dynamic_sample_count >= 500
    assert result.correlation_peak > 0.7
    assert result.individual_valid


def test_yaw_only_fallback_marks_combined_delay_explicitly() -> None:
    timestamps_sec = np.arange(0.0, 80.0, 0.02)
    command_rad = 0.25 * np.sin(2.0 * np.pi * timestamps_sec / 3.0)
    delayed_command = np.interp(
        timestamps_sec - 0.20,
        timestamps_sec,
        command_rad,
        left=command_rad[0],
    )
    speed_mps = np.full_like(timestamps_sec, 3.0)
    yaw_rate_rps = speed_mps * np.tan(delayed_command) / 1.087

    result = estimate_combined_yaw_delay(
        timestamps_sec,
        command_rad,
        speed_mps,
        yaw_rate_rps,
        wheelbase_m=1.087,
        config=DelayEstimationConfig(minimum_dynamic_samples=500),
    )

    assert result.method == "command_to_yaw_combined_delay"
    assert result.delay_sec == pytest.approx(0.20, abs=0.02)
    assert result.time_constant_sec is None
    assert result.individual_valid


def test_cross_run_consistency_requires_five_runs_and_hundred_ms_band() -> None:
    timestamps_sec = np.arange(0.0, 30.0, 0.02)
    command_rad = 0.3 * np.sin(timestamps_sec)
    speed_mps = np.full_like(timestamps_sec, 3.0)
    base_results = []
    for delay_sec in (0.10, 0.11, 0.12, 0.13, 0.25):
        delayed = np.interp(
            timestamps_sec - delay_sec,
            timestamps_sec,
            command_rad,
            left=command_rad[0],
        )
        yaw = speed_mps * np.tan(delayed) / 1.087
        base_results.append(
            estimate_combined_yaw_delay(
                timestamps_sec,
                command_rad,
                speed_mps,
                yaw,
                wheelbase_m=1.087,
                config=DelayEstimationConfig(minimum_dynamic_samples=500),
            )
        )

    assessment = assess_delay_consistency(base_results, minimum_runs=5, max_deviation_sec=0.10)

    assert assessment.run_count == 5
    assert assessment.median_delay_sec == pytest.approx(0.12, abs=0.02)
    assert assessment.dataset_valid is False
    assert assessment.run_consistent == (True, True, True, True, False)
