from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from aic_transfuser_lite.data.mcap_converter import (
    ConverterConfig,
    TimedControl,
    assign_run_splits,
    integrate_command_waypoints,
    message_image_to_rgb,
    nearest_index,
    previous_index,
    resample_lidar_nearest,
)


def test_message_image_to_rgb_decodes_bgr8_with_row_padding() -> None:
    # Two BGR pixels followed by two bytes of row padding.
    message = SimpleNamespace(
        height=1,
        width=2,
        encoding="bgr8",
        step=8,
        data=np.array([3, 2, 1, 30, 20, 10, 255, 255], dtype=np.uint8),
    )

    image = message_image_to_rgb(message)

    assert image.shape == (1, 2, 3)
    assert image.dtype == np.uint8
    np.testing.assert_array_equal(image[0], [[1, 2, 3], [10, 20, 30]])


def test_message_image_to_rgb_rejects_unsupported_encoding() -> None:
    message = SimpleNamespace(
        height=1,
        width=1,
        encoding="16UC1",
        step=2,
        data=np.zeros(2, dtype=np.uint8),
    )

    with pytest.raises(ValueError, match="Unsupported"):
        message_image_to_rgb(message)


def test_resample_lidar_has_contract_shape_and_preserves_invalid_values() -> None:
    source = np.array([1.0, np.inf, 3.0], dtype=np.float32)

    result = resample_lidar_nearest(source, 5)

    assert result.shape == (5,)
    assert result.dtype == np.float32
    assert np.isinf(result[2])


def test_timestamp_matching_obeys_direction_and_tolerance() -> None:
    timestamps = [100, 200, 300]

    assert nearest_index(timestamps, 260, 50) == (2, 40)
    assert nearest_index(timestamps, 260, 20) is None
    assert previous_index(timestamps, 260, 100) == (1, -60)
    assert previous_index(timestamps, 90, 100) is None


def test_integrate_command_waypoints_straight_line_units_and_shape() -> None:
    controls = [
        TimedControl(0, speed_mps=2.0, acceleration_mps2=0.0, steering_rad=0.0),
        TimedControl(
            4_000_000_000,
            speed_mps=2.0,
            acceleration_mps2=0.0,
            steering_rad=0.0,
        ),
    ]

    waypoints = integrate_command_waypoints(
        controls,
        observation_ns=0,
        waypoint_times_sec=(0.5, 1.0, 2.0),
        wheelbase_m=1.0,
    )

    assert waypoints.shape == (3, 2)
    np.testing.assert_allclose(waypoints[:, 0], [1.0, 2.0, 4.0], atol=1e-6)
    np.testing.assert_allclose(waypoints[:, 1], 0.0, atol=1e-6)


def test_integrate_command_waypoints_turns_left_for_positive_steering() -> None:
    controls = [
        TimedControl(0, speed_mps=1.0, acceleration_mps2=0.0, steering_rad=0.2),
        TimedControl(
            2_000_000_000,
            speed_mps=1.0,
            acceleration_mps2=0.0,
            steering_rad=0.2,
        ),
    ]

    waypoints = integrate_command_waypoints(
        controls,
        observation_ns=0,
        waypoint_times_sec=(0.5, 1.0),
        wheelbase_m=1.0,
    )

    assert np.all(waypoints[:, 0] > 0.0)
    assert np.all(waypoints[:, 1] > 0.0)


def test_converter_config_rejects_implicit_invalid_units() -> None:
    config = ConverterConfig(
        sample_rate_hz=10.0,
        sync_tolerance_ms=100.0,
        lidar_points=1080,
        waypoint_times_sec=(0.5, 1.0),
        wheelbase_m=0.0,
    )

    with pytest.raises(ValueError, match="wheelbase_m"):
        config.validate()

    invalid_speed_threshold = ConverterConfig(
        sample_rate_hz=10.0,
        sync_tolerance_ms=100.0,
        lidar_points=1080,
        waypoint_times_sec=(0.5, 1.0),
        wheelbase_m=1.0,
        min_usable_commanded_speed_mps=-0.1,
    )
    with pytest.raises(ValueError, match="min_usable_commanded_speed_mps"):
        invalid_speed_threshold.validate()


def test_assign_run_splits_never_splits_a_run() -> None:
    run_ids = ["run_a", "run_b", "run_c", "run_d"] * 2

    mapping = assign_run_splits(
        run_ids, val_ratio=0.25, test_ratio=0.25, seed=42
    )

    assert set(mapping) == {"run_a", "run_b", "run_c", "run_d"}
    assert set(mapping.values()) == {"train", "val", "test"}
