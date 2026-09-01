from __future__ import annotations

import numpy as np
import pytest

from aic_transfuser_lite.runtime.preprocessing_v1 import (
    V1LidarContract,
    prepare_native_lidar_input,
    prepare_speed_only_ego,
)


def contract(points: int = 750) -> V1LidarContract:
    return V1LidarContract(
        points=points,
        angle_min_rad=-1.5666074752807617,
        angle_increment_rad=0.004188789986073971,
        range_min_m=0.0,
        range_max_m=25.0,
        frame_id="lidar",
    )


def test_native_lidar_builds_range_and_explicit_validity_channels() -> None:
    ranges = np.linspace(0.0, 25.0, 750, dtype=np.float32)
    ranges[1] = np.inf
    ranges[2] = np.nan
    ranges[3] = 26.0
    result = prepare_native_lidar_input(
        ranges,
        angle_min_rad=-1.5666074752807617,
        angle_increment_rad=0.004188789986073971,
        range_min_m=0.0,
        range_max_m=25.0,
        frame_id="lidar",
        contract=contract(),
    )
    assert result.shape == (2, 750)
    assert result.dtype == np.float32
    np.testing.assert_array_equal(result[1, 1:4], [0.0, 0.0, 0.0])
    np.testing.assert_array_equal(result[0, 1:4], [1.0, 1.0, 1.0])
    assert result[1, 4] == 1.0
    assert result[0, -1] == pytest.approx(1.0)


@pytest.mark.parametrize(
    ("override", "match"),
    [
        ({"ranges_m": np.ones(749, dtype=np.float32)}, "beam count"),
        ({"angle_min_rad": -1.5}, "angle_min"),
        ({"angle_increment_rad": 0.0042}, "angle_increment"),
        ({"range_max_m": 30.0}, "range_max"),
        ({"frame_id": "base_link"}, "frame"),
    ],
)
def test_native_lidar_rejects_geometry_drift(override: dict[str, object], match: str) -> None:
    kwargs: dict[str, object] = {
        "ranges_m": np.ones(750, dtype=np.float32),
        "angle_min_rad": -1.5666074752807617,
        "angle_increment_rad": 0.004188789986073971,
        "range_min_m": 0.0,
        "range_max_m": 25.0,
        "frame_id": "lidar",
        "contract": contract(),
    }
    kwargs.update(override)
    with pytest.raises(ValueError, match=match):
        prepare_native_lidar_input(**kwargs)


def test_speed_only_ego_uses_signed_longitudinal_speed_and_training_scale() -> None:
    np.testing.assert_allclose(prepare_speed_only_ego(5.0, scale_mps=10.0), [0.5])
    np.testing.assert_allclose(prepare_speed_only_ego(-2.0, scale_mps=10.0), [-0.2])
    with pytest.raises(ValueError, match="finite"):
        prepare_speed_only_ego(float("nan"), scale_mps=10.0)
