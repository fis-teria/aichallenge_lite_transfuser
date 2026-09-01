from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class LidarPreprocessConfig:
    min_range_m: float = 0.05
    max_range_m: float = 30.0


def sanitize_lidar(
    ranges: np.ndarray,
    config: LidarPreprocessConfig = LidarPreprocessConfig(),
) -> tuple[np.ndarray, np.ndarray]:
    """Sanitize and normalize LaserScan ranges.

    Args:
        ranges: One-dimensional distance array in metres.
        config: Valid range and normalization limits.

    Returns:
        normalized: float32 values clipped to [0, 1]. Invalid beams are set to 1.
        valid_mask: float32 mask where valid beam is 1 and invalid beam is 0.
    """
    raw = np.asarray(ranges, dtype=np.float32)
    if raw.ndim != 1:
        raise ValueError(f"Expected 1D LaserScan array, got shape={raw.shape}")
    if config.max_range_m <= config.min_range_m:
        raise ValueError("max_range_m must be greater than min_range_m")

    valid = (
        np.isfinite(raw)
        & (raw >= config.min_range_m)
        & (raw <= config.max_range_m)
    )
    cleaned = np.where(valid, raw, config.max_range_m)
    cleaned = np.clip(cleaned, config.min_range_m, config.max_range_m)
    normalized = (cleaned - config.min_range_m) / (
        config.max_range_m - config.min_range_m
    )
    return normalized.astype(np.float32), valid.astype(np.float32)


def laser_to_xy(
    ranges_m: np.ndarray,
    angle_min_rad: float,
    angle_increment_rad: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Convert a 1D scan to x/y points in the sensor frame.

    The caller is responsible for confirming the frame convention. This function
    assumes x=range*cos(angle), y=range*sin(angle).
    """
    ranges = np.asarray(ranges_m, dtype=np.float32)
    if ranges.ndim != 1:
        raise ValueError(f"Expected 1D ranges, got shape={ranges.shape}")
    angles = angle_min_rad + np.arange(ranges.size, dtype=np.float32) * angle_increment_rad
    valid = np.isfinite(ranges) & (ranges > 0.0)
    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)
    return x.astype(np.float32), y.astype(np.float32), valid.astype(np.float32)


def build_occupancy_bev(
    ranges_m: np.ndarray,
    angle_min_rad: float,
    angle_increment_rad: float,
    *,
    x_min_m: float = -5.0,
    x_max_m: float = 30.0,
    y_min_m: float = -10.0,
    y_max_m: float = 10.0,
    resolution_m: float = 0.2,
) -> np.ndarray:
    """Rasterize valid scan endpoints into a simple occupancy BEV.

    Returns:
        Array of shape [1, H, W] with occupied endpoints set to 1.

    This starter implementation does not ray-trace free space. Add occupied/free/
    unknown channels after the sensor frame and vehicle footprint are verified.
    """
    if resolution_m <= 0:
        raise ValueError("resolution_m must be positive")
    if not (x_max_m > x_min_m and y_max_m > y_min_m):
        raise ValueError("Invalid BEV bounds")

    width = int(np.ceil((x_max_m - x_min_m) / resolution_m))
    height = int(np.ceil((y_max_m - y_min_m) / resolution_m))
    bev = np.zeros((1, height, width), dtype=np.float32)

    x, y, valid = laser_to_xy(ranges_m, angle_min_rad, angle_increment_rad)
    valid_bool = valid.astype(bool)
    x = x[valid_bool]
    y = y[valid_bool]
    inside = (x >= x_min_m) & (x < x_max_m) & (y >= y_min_m) & (y < y_max_m)
    x = x[inside]
    y = y[inside]

    col = ((x - x_min_m) / resolution_m).astype(np.int64)
    row = ((y_max_m - y) / resolution_m).astype(np.int64)
    row = np.clip(row, 0, height - 1)
    col = np.clip(col, 0, width - 1)
    bev[0, row, col] = 1.0
    return bev
