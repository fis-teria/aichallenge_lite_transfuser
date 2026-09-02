from __future__ import annotations

import numpy as np
import pytest

from aic_transfuser_lite.runtime.control_projection import (
    ControlLimits,
    PreviousControlState,
    ProjectionTiming,
    project_control_sequence,
)


def _limits(**overrides: object) -> ControlLimits:
    values: dict[str, object] = {
        "max_abs_steering_rad": 0.6,
        "max_steering_rate_radps": 0.5,
        "min_acceleration_mps2": -4.0,
        "max_acceleration_mps2": 2.0,
        "min_jerk_mps3": -6.0,
        "max_jerk_mps3": 3.0,
        "max_speed_mps": 12.0,
        "dt_sec": 0.1,
        "authoritative": True,
        "source": "measured_vehicle_limits_v1",
    }
    values.update(overrides)
    return ControlLimits(**values)  # type: ignore[arg-type]


def _timing(**overrides: float) -> ProjectionTiming:
    values = {
        "observation_stamp_sec": 10.0,
        "now_sec": 10.05,
        "valid_for_sec": 0.2,
        "max_observation_age_sec": 0.1,
    }
    values.update(overrides)
    return ProjectionTiming(**values)


def test_projection_integrates_bounded_steering_rate_and_asymmetric_jerk() -> None:
    raw = np.array([[100.0, 100.0], [-100.0, -100.0]] * 10)
    result = project_control_sequence(
        raw,
        previous=PreviousControlState(0.58, 2.0, 1.9),
        limits=_limits(),
        timing=_timing(),
    )

    assert result.commands.shape == (20, 3)
    assert np.max(np.abs(result.commands[:, 0])) <= 0.6
    assert np.max(np.abs(result.steering_rate_radps)) <= 0.5 + 1e-12
    assert np.min(result.commands[:, 1]) >= 0.0
    assert np.max(result.commands[:, 1]) <= 12.0
    assert np.min(result.commands[:, 2]) >= -4.0
    assert np.max(result.commands[:, 2]) <= 2.0
    assert np.min(result.jerk_mps3) >= -6.0 - 1e-12
    assert np.max(result.jerk_mps3) <= 3.0 + 1e-12
    assert result.valid_until_sec == pytest.approx(10.2)
    assert result.source_stamp_sec == 10.0
    assert result.dt_sec == 0.1


def test_zero_raw_sequence_preserves_stationary_control_state() -> None:
    result = project_control_sequence(
        np.zeros((3, 2)),
        previous=PreviousControlState(0.1, 2.0, 0.0),
        limits=_limits(),
        timing=_timing(),
    )
    np.testing.assert_allclose(result.commands, [[0.1, 2.0, 0.0]] * 3)
    np.testing.assert_allclose(result.steering_rate_radps, 0.0)
    np.testing.assert_allclose(result.jerk_mps3, 0.0)


@pytest.mark.parametrize(
    ("overrides", "message"),
    [
        ({"authoritative": False}, "authoritative"),
        ({"source": ""}, "authoritative"),
        ({"max_steering_rate_radps": 0.0}, "steering_rate"),
        ({"max_speed_mps": float("nan")}, "finite"),
        ({"min_jerk_mps3": 0.0}, "jerk limits"),
        ({"min_acceleration_mps2": 0.0}, "acceleration limits"),
    ],
)
def test_projection_rejects_missing_or_invalid_authoritative_limits(
    overrides: dict[str, object], message: str
) -> None:
    with pytest.raises(ValueError, match=message):
        project_control_sequence(
            np.zeros((1, 2)),
            previous=PreviousControlState(0.0, 0.0, 0.0),
            limits=_limits(**overrides),
            timing=_timing(),
        )


@pytest.mark.parametrize(
    ("timing", "message"),
    [
        (_timing(observation_stamp_sec=10.1, now_sec=10.0), "future"),
        (_timing(observation_stamp_sec=9.8), "stale"),
        (_timing(observation_stamp_sec=9.9, valid_for_sec=0.05), "expired"),
        (_timing(valid_for_sec=0.0), "lifetimes"),
    ],
)
def test_projection_rejects_invalid_timing(
    timing: ProjectionTiming, message: str
) -> None:
    with pytest.raises(ValueError, match=message):
        project_control_sequence(
            np.zeros((1, 2)),
            previous=PreviousControlState(0.0, 0.0, 0.0),
            limits=_limits(),
            timing=timing,
        )


@pytest.mark.parametrize(
    ("raw", "previous", "message"),
    [
        (np.zeros((2, 3)), PreviousControlState(0.0, 0.0, 0.0), r"\[H,2\]"),
        (np.array([[np.inf, 0.0]]), PreviousControlState(0.0, 0.0, 0.0), "finite"),
        (np.zeros((1, 2)), PreviousControlState(0.7, 0.0, 0.0), "steering"),
        (np.zeros((1, 2)), PreviousControlState(0.0, -0.1, 0.0), "speed"),
    ],
)
def test_projection_rejects_invalid_shape_values_and_previous_state(
    raw: np.ndarray, previous: PreviousControlState, message: str
) -> None:
    with pytest.raises(ValueError, match=message):
        project_control_sequence(raw, previous=previous, limits=_limits(), timing=_timing())
