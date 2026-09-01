from types import SimpleNamespace

import numpy as np
import pytest

from aic_e2e_runtime.runtime_adapter import (
    image_message_to_rgb,
    odometry_to_ego,
    resample_laser_ranges,
    stamp_to_seconds,
)


def test_bgr_image_with_row_padding_converts_to_rgb() -> None:
    message = SimpleNamespace(
        height=1,
        width=2,
        encoding="bgr8",
        step=8,
        data=bytes([1, 2, 3, 4, 5, 6, 99, 99]),
    )
    result = image_message_to_rgb(message)
    np.testing.assert_array_equal(result, [[[3, 2, 1], [6, 5, 4]]])


def test_laser_resample_preserves_endpoints_and_shape() -> None:
    result = resample_laser_ranges(np.asarray([1.0, 2.0, 3.0]), 5)
    assert result.shape == (5,)
    assert result[0] == pytest.approx(1.0)
    assert result[-1] == pytest.approx(3.0)


def test_odometry_ego_shape_units_and_speed() -> None:
    vector = SimpleNamespace(x=3.0, y=4.0, z=0.0)
    angular = SimpleNamespace(x=0.0, y=0.0, z=0.2)
    odometry = SimpleNamespace(
        twist=SimpleNamespace(twist=SimpleNamespace(linear=vector, angular=angular))
    )
    ego, speed = odometry_to_ego(odometry, previous_steering_rad=0.1)
    np.testing.assert_allclose(ego, [3.0, 4.0, 0.2, 0.1, 1.0])
    assert speed == pytest.approx(5.0)


def test_odometry_speed_only_contract_uses_speed_magnitude() -> None:
    vector = SimpleNamespace(x=3.0, y=4.0, z=0.0)
    angular = SimpleNamespace(x=0.0, y=0.0, z=0.2)
    odometry = SimpleNamespace(
        twist=SimpleNamespace(twist=SimpleNamespace(linear=vector, angular=angular))
    )
    ego, speed = odometry_to_ego(
        odometry, previous_steering_rad=0.1, ego_features=("speed_mps",)
    )
    np.testing.assert_allclose(ego, [5.0])
    assert speed == pytest.approx(5.0)


def test_zero_stamp_uses_fallback() -> None:
    assert stamp_to_seconds(SimpleNamespace(sec=0, nanosec=0), 12.5) == 12.5
