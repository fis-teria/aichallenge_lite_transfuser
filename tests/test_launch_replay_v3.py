from pathlib import Path

import numpy as np
import pytest

from aic_transfuser_lite.evaluation.launch_replay_v3 import (
    load_path_only_replay_config_v3,
    replay_path_only_launch_v3,
)


ROOT = Path(__file__).parents[1]
RUNTIME_CONFIG = (
    ROOT
    / "ros2_ws/src/aic_e2e_runtime/config/"
    "runtime.v3.trajectory_authoritative.param.yaml"
)


def _config():
    return load_path_only_replay_config_v3(
        RUNTIME_CONFIG,
        trajectory_steps=15,
        minimum_endpoint_forward_m=0.1,
    )


def test_launch_replay_uses_runtime_path_only_speed_and_controller() -> None:
    x = np.linspace(0.1, 1.5, 15)
    result = replay_path_only_launch_v3(
        np.stack((x, np.zeros_like(x)), axis=1),
        np.zeros(15),
        current_speed_mps=0.0,
        yaw_rate_rps=0.0,
        actual_steering_rad=0.0,
        config=_config(),
    )

    assert result.ready is True
    assert result.reference_accepted is True
    assert result.controller_requested_speed_mps == pytest.approx(0.75)
    assert result.controller_state == "launching"
    assert result.lookahead_distance_m == pytest.approx(1.0)
    assert result.path_length_m == pytest.approx(1.5)
    assert "model_speed_ignored" in result.transformations
    assert "stop_probability_unavailable" in result.transformations
    assert result.stop_probability_connected is False


def test_maximum_x_alone_cannot_pass_runtime_launch_replay() -> None:
    x = np.concatenate((np.linspace(0.1, 0.3, 8), np.linspace(0.25, 0.05, 7)))
    result = replay_path_only_launch_v3(
        np.stack((x, np.zeros_like(x)), axis=1),
        np.ones(15),
        current_speed_mps=0.0,
        yaw_rate_rps=0.0,
        actual_steering_rad=0.0,
        config=_config(),
    )

    assert result.maximum_forward_m >= 0.1
    assert result.endpoint_forward_m < 0.1
    assert result.reference_accepted is True
    assert result.ready is False
    assert "endpoint_forward_too_short" in result.reasons


def test_nonforward_initial_path_is_rejected_fail_closed() -> None:
    trajectory = np.stack(
        (np.linspace(-0.1, 1.0, 15), np.zeros(15)), axis=1
    )
    result = replay_path_only_launch_v3(
        trajectory,
        np.ones(15),
        current_speed_mps=0.0,
        yaw_rate_rps=0.0,
        actual_steering_rad=0.0,
        config=_config(),
    )
    assert result.reference_accepted is False
    assert result.ready is False
    assert result.reasons == ("initial_waypoint_not_forward",)


def test_launch_replay_preserves_curvature_cap_and_rejects_nonfinite_path() -> None:
    theta = np.linspace(0.05, np.pi / 2.0, 15)
    radius = 0.2
    trajectory = np.stack(
        (radius * np.sin(theta), radius * (1.0 - np.cos(theta))), axis=1
    )
    result = replay_path_only_launch_v3(
        trajectory,
        np.zeros(15),
        current_speed_mps=0.0,
        yaw_rate_rps=0.0,
        actual_steering_rad=0.0,
        config=_config(),
    )
    assert result.reference_accepted is True
    assert "curvature_speed_cap" in result.transformations
    assert result.maximum_abs_curvature_per_m is not None
    assert result.maximum_abs_curvature_per_m > 1.0
    assert result.controller_requested_speed_mps is not None
    assert result.controller_requested_speed_mps < 0.75

    trajectory[3, 0] = np.nan
    with pytest.raises(ValueError, match="must be finite"):
        replay_path_only_launch_v3(
            trajectory,
            np.zeros(15),
            current_speed_mps=0.0,
            yaw_rate_rps=0.0,
            actual_steering_rad=0.0,
            config=_config(),
        )


def test_reverse_stationary_noise_is_counted_as_fail_closed_launch_rejection() -> None:
    x = np.linspace(0.1, 1.5, 15)
    result = replay_path_only_launch_v3(
        np.stack((x, np.zeros_like(x)), axis=1),
        np.zeros(15),
        current_speed_mps=-0.04,
        yaw_rate_rps=0.0,
        actual_steering_rad=0.0,
        config=_config(),
    )
    assert result.ready is False
    assert result.reference_accepted is False
    assert result.reasons[0].startswith("measured_speed_rejected:")
