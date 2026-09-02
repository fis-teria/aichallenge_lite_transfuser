from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest

from aic_transfuser_lite.data.calibration.artifact import (
    CalibrationPromotion,
    build_calibration_artifact,
)
from aic_transfuser_lite.data.calibration.lateral import LateralCalibration
from aic_transfuser_lite.data.calibration.longitudinal import LongitudinalModeFit
from aic_transfuser_lite.runtime.control_projection import (
    ProjectedControlSequence,
    PreviousControlState,
)
from aic_transfuser_lite.runtime.rollout_consistency import (
    ActuatorBicycleRollout,
    ConsistencyThresholds,
    RolloutInitialState,
    evaluate_rollout_consistency,
    rollout_actuator_bicycle,
)


def _fit(mode: str) -> LongitudinalModeFit:
    return LongitudinalModeFit(
        mode=mode,  # type: ignore[arg-type]
        pure_delay_sec=0.0,
        time_constant_sec=0.2,
        gain=1.0,
        bias_mps2=0.0,
        valid_speed_range_mps=(0.0, 20.0),
        command_range_mps2=(-4.0, 2.0),
        rmse_mps2=0.01,
        nrmse=0.01,
        correlation_peak=0.99,
        mode_sample_count=500,
        total_sample_count=1000,
        excluded_actual_accel_count=0,
        individually_valid=True,
        validity_reasons=(),
    )


def _calibration(*, steering_valid: bool = True):
    steering = LateralCalibration(
        pure_delay_sec=0.0,
        time_constant_sec=0.2,
        gain=1.0,
        bias_rad=0.0,
        valid_speed_range_mps=(0.0, 20.0),
        nrmse=0.1,
        yaw_rate_nrmse=0.1,
        correlation_peak=0.99,
        dynamic_sample_count=500,
        total_sample_count=1000,
        excluded_sample_count=0,
        individually_valid=steering_valid,
        validity_reasons=() if steering_valid else ("steering_nrmse>=0.7",),
        source_method="synthetic_test",
    )
    return build_calibration_artifact(
        source_run_hashes={"synthetic": "1" * 64},
        vehicle_profile_sha256="2" * 64,
        steering=steering,
        drive=_fit("drive"),
        brake=_fit("brake"),
        promotion=CalibrationPromotion(state="candidate"),
    )


def _sequence(commands: np.ndarray, *, speed_mps: float = 2.0) -> ProjectedControlSequence:
    commands = np.asarray(commands, dtype=np.float64)
    return ProjectedControlSequence(
        commands=commands,
        steering_rate_radps=np.zeros(commands.shape[0]),
        jerk_mps3=np.zeros(commands.shape[0]),
        source_stamp_sec=10.0,
        valid_until_sec=10.2,
        limits_source="synthetic_authoritative_limits",
        dt_sec=0.1,
        initial_state=PreviousControlState(0.0, speed_mps, 0.0),
    )


def test_actuator_bicycle_rollout_straight_constant_speed_has_si_shapes() -> None:
    rollout = rollout_actuator_bicycle(
        _sequence(np.array([[0.0, 2.0, 0.0]] * 4)),
        calibration=_calibration(),
        wheelbase_m=1.087,
        initial=RolloutInitialState(0.0, 0.0),
    )

    assert rollout.trajectory_xy_m.shape == (4, 2)
    np.testing.assert_allclose(rollout.trajectory_xy_m[:, 0], [0.2, 0.4, 0.6, 0.8])
    np.testing.assert_allclose(rollout.trajectory_xy_m[:, 1], 0.0)
    np.testing.assert_allclose(rollout.speed_mps, 2.0)
    np.testing.assert_allclose(rollout.heading_rad, 0.0)


def test_longitudinal_mode_switch_uses_hysteresis() -> None:
    commands = np.array(
        [[0.0, 2.0, 0.2], [0.0, 2.0, 0.05], [0.0, 2.0, -0.2]]
    )
    rollout = rollout_actuator_bicycle(
        _sequence(commands),
        calibration=_calibration(),
        wheelbase_m=1.087,
        initial=RolloutInitialState(0.0, 0.0, longitudinal_mode="drive"),
        mode_hysteresis_mps2=0.1,
    )
    assert rollout.longitudinal_modes == ("drive", "drive", "brake")


def test_rollout_rejects_failed_calibration_and_outside_applicability() -> None:
    with pytest.raises(ValueError, match="steering calibration is not individually valid"):
        rollout_actuator_bicycle(
            _sequence(np.array([[0.0, 2.0, 0.0]] * 2)),
            calibration=_calibration(steering_valid=False),
            wheelbase_m=1.087,
            initial=RolloutInitialState(0.0, 0.0),
        )

    narrow = _calibration()
    narrow = replace(
        narrow,
        steering=replace(narrow.steering, valid_speed_range_mps=(0.0, 1.0)),
    )
    with pytest.raises(ValueError, match="outside calibration applicability"):
        rollout_actuator_bicycle(
            _sequence(np.array([[0.0, 2.0, 0.0]] * 2)),
            calibration=narrow,
            wheelbase_m=1.087,
            initial=RolloutInitialState(0.0, 0.0),
        )


def _rollout(
    trajectory: np.ndarray, headings: np.ndarray, speeds: np.ndarray
) -> ActuatorBicycleRollout:
    count = len(trajectory)
    return ActuatorBicycleRollout(
        trajectory_xy_m=np.asarray(trajectory, dtype=np.float64),
        heading_rad=np.asarray(headings, dtype=np.float64),
        speed_mps=np.asarray(speeds, dtype=np.float64),
        actual_steering_rad=np.zeros(count),
        actual_acceleration_mps2=np.zeros(count),
        longitudinal_modes=("drive",) * count,
    )


def _thresholds(**overrides: float) -> ConsistencyThresholds:
    values = {
        "max_position_error_m": 0.5,
        "max_lateral_error_m": 0.25,
        "max_heading_error_rad": 0.2,
        "max_speed_error_mps": 0.5,
        "max_endpoint_error_m": 0.4,
    }
    values.update(overrides)
    return ConsistencyThresholds(**values)


def test_matching_same_candidate_rollout_is_consistent() -> None:
    trajectory = np.array([[1.0, 0.0], [2.0, 0.0], [3.0, 0.0]])
    result = evaluate_rollout_consistency(
        trajectory,
        np.array([2.0, 2.0, 2.0]),
        _rollout(trajectory, np.zeros(3), np.array([2.0, 2.0, 2.0])),
        thresholds=_thresholds(),
    )
    assert result.consistent
    assert result.reasons == ()
    assert result.endpoint_error_m == 0.0


def test_inconsistency_returns_structured_metric_reasons() -> None:
    trajectory = np.array([[1.0, 0.0], [2.0, 0.0], [3.0, 0.0]])
    shifted = trajectory + np.array([0.6, 1.0])
    result = evaluate_rollout_consistency(
        trajectory,
        np.array([2.0, 2.0, 2.0]),
        _rollout(shifted, np.ones(3), np.array([3.0, 3.0, 3.0])),
        thresholds=_thresholds(),
    )
    assert not result.consistent
    assert result.reasons == (
        "max_position_error_m>0.500000",
        "max_lateral_error_m>0.250000",
        "max_heading_error_rad>0.200000",
        "max_speed_error_mps>0.500000",
        "endpoint_error_m>0.400000",
    )


@pytest.mark.parametrize(
    ("trajectory", "speeds", "message"),
    [
        (np.zeros((1, 2)), np.zeros(1), r"N>=2"),
        (np.zeros((2, 2)), np.zeros(3), r"\[N\]"),
        (np.array([[0.0, 0.0], [np.nan, 0.0]]), np.zeros(2), "finite"),
        (np.zeros((2, 2)), np.array([0.0, -1.0]), "non-negative"),
    ],
)
def test_consistency_rejects_invalid_shapes_and_values(
    trajectory: np.ndarray, speeds: np.ndarray, message: str
) -> None:
    with pytest.raises(ValueError, match=message):
        evaluate_rollout_consistency(
            trajectory,
            speeds,
            _rollout(np.zeros((2, 2)), np.zeros(2), np.zeros(2)),
            thresholds=_thresholds(),
        )


def test_consistency_requires_explicit_positive_thresholds() -> None:
    with pytest.raises(ValueError, match="finite and positive"):
        evaluate_rollout_consistency(
            np.zeros((2, 2)),
            np.zeros(2),
            _rollout(np.zeros((2, 2)), np.zeros(2), np.zeros(2)),
            thresholds=_thresholds(max_heading_error_rad=0.0),
        )
