import numpy as np

from aic_transfuser_lite.data.lidar_preprocess import (
    LidarPreprocessConfig,
    build_occupancy_bev,
    sanitize_lidar,
)


def test_sanitize_lidar_returns_finite_normalized_values() -> None:
    raw = np.array([np.nan, np.inf, 0.0, 0.05, 1.0, 30.0, 31.0], dtype=np.float32)
    normalized, valid = sanitize_lidar(raw, LidarPreprocessConfig(0.05, 30.0))
    assert normalized.shape == raw.shape
    assert valid.shape == raw.shape
    assert np.isfinite(normalized).all()
    assert ((normalized >= 0.0) & (normalized <= 1.0)).all()
    assert valid.tolist() == [0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0]


def test_bev_shape_and_endpoint() -> None:
    scan = np.array([1.0], dtype=np.float32)
    bev = build_occupancy_bev(
        scan,
        angle_min_rad=0.0,
        angle_increment_rad=0.1,
        x_min_m=0.0,
        x_max_m=2.0,
        y_min_m=-1.0,
        y_max_m=1.0,
        resolution_m=0.5,
    )
    assert bev.shape == (1, 4, 4)
    assert bev.sum() == 1.0
