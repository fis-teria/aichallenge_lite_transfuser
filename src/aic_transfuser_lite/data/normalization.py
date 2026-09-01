from __future__ import annotations

import math

import numpy as np


IMAGENET_RGB_MEAN = (0.485, 0.456, 0.406)
IMAGENET_RGB_STD = (0.229, 0.224, 0.225)


def normalize_longitudinal_speed(
    speed_mps: float,
    *,
    scale_mps: float = 10.0,
) -> np.float32:
    """Normalize measured longitudinal speed without changing its sign."""
    speed = float(speed_mps)
    scale = float(scale_mps)
    if not math.isfinite(speed):
        raise ValueError("longitudinal speed must be finite")
    if not math.isfinite(scale) or scale <= 0.0:
        raise ValueError("speed normalization scale must be finite and positive")
    return np.float32(np.clip(speed / scale, -1.0, 1.0))


def normalize_lidar_range_and_validity(
    ranges_m: np.ndarray,
    valid_mask: np.ndarray,
    *,
    min_range_m: float,
    max_range_m: float,
) -> np.ndarray:
    """Build the v2 LiDAR input ``[normalized_range, explicit_validity]``.

    Validity is never inferred from a command or silently repaired. A beam marked
    valid must contain a finite measured range; invalid beams are represented by
    maximum range in channel 0 and zero in channel 1.
    """
    minimum = float(min_range_m)
    maximum = float(max_range_m)
    if not math.isfinite(minimum) or not math.isfinite(maximum) or maximum <= minimum:
        raise ValueError("LiDAR max_range_m must be finite and greater than min_range_m")

    ranges = np.asarray(ranges_m, dtype=np.float32)
    mask_raw = np.asarray(valid_mask)
    if ranges.ndim != 1 or mask_raw.ndim != 1:
        raise ValueError(
            f"Expected one-dimensional LiDAR range/mask, got {ranges.shape} and {mask_raw.shape}"
        )
    if ranges.shape != mask_raw.shape:
        raise ValueError(
            f"LiDAR range/mask shapes differ: {ranges.shape} vs {mask_raw.shape}"
        )
    if not np.all((mask_raw == 0) | (mask_raw == 1)):
        raise ValueError("LiDAR validity mask must contain only 0 or 1")

    valid = mask_raw.astype(bool, copy=False)
    if not np.all(np.isfinite(ranges[valid])):
        raise ValueError("A LiDAR beam marked valid contains a non-finite range")

    cleaned = np.where(valid, ranges, maximum)
    cleaned = np.clip(cleaned, minimum, maximum)
    normalized = (cleaned - minimum) / (maximum - minimum)
    return np.stack(
        (normalized.astype(np.float32), valid.astype(np.float32)),
        axis=0,
    )
