from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from aic_transfuser_lite.data.normalization import (
    normalize_lidar_range_and_validity,
    normalize_longitudinal_speed,
)


@dataclass(frozen=True)
class V1LidarContract:
    """Native LaserScan geometry and normalization contract for static v1."""

    points: int
    angle_min_rad: float
    angle_increment_rad: float
    range_min_m: float
    range_max_m: float
    frame_id: str
    angle_min_tolerance_rad: float = 1e-7
    angle_increment_tolerance_rad: float = 1e-9
    range_tolerance_m: float = 1e-7

    def __post_init__(self) -> None:
        numeric = (
            self.angle_min_rad,
            self.angle_increment_rad,
            self.range_min_m,
            self.range_max_m,
            self.angle_min_tolerance_rad,
            self.angle_increment_tolerance_rad,
            self.range_tolerance_m,
        )
        if self.points < 2:
            raise ValueError("LiDAR points must be at least two")
        if not all(math.isfinite(float(value)) for value in numeric):
            raise ValueError("LiDAR geometry values must be finite")
        if self.angle_increment_rad <= 0.0:
            raise ValueError("LiDAR angle_increment must be positive")
        if self.range_max_m <= self.range_min_m:
            raise ValueError("LiDAR range_max must exceed range_min")
        if not self.frame_id:
            raise ValueError("LiDAR frame must be explicit")
        if min(
            self.angle_min_tolerance_rad,
            self.angle_increment_tolerance_rad,
            self.range_tolerance_m,
        ) < 0.0:
            raise ValueError("LiDAR geometry tolerances must be non-negative")


def _require_close(
    name: str,
    actual: float,
    expected: float,
    tolerance: float,
) -> None:
    if not math.isfinite(float(actual)) or not math.isclose(
        float(actual), float(expected), rel_tol=0.0, abs_tol=float(tolerance)
    ):
        raise ValueError(
            f"LiDAR {name} drifted: actual={actual!r}, expected={expected!r}, "
            f"tolerance={tolerance!r}"
        )


def prepare_native_lidar_input(
    ranges_m: np.ndarray,
    *,
    angle_min_rad: float,
    angle_increment_rad: float,
    range_min_m: float,
    range_max_m: float,
    frame_id: str,
    contract: V1LidarContract,
) -> np.ndarray:
    """Build v1 LiDAR ``[2,P]`` without resampling or hidden repair."""

    ranges = np.asarray(ranges_m, dtype=np.float32)
    if ranges.ndim != 1 or ranges.size != contract.points:
        raise ValueError(
            f"LiDAR beam count/shape drifted: expected ({contract.points},), "
            f"got {ranges.shape}"
        )
    if str(frame_id) != contract.frame_id:
        raise ValueError(
            f"LiDAR frame drifted: actual={frame_id!r}, expected={contract.frame_id!r}"
        )
    _require_close(
        "angle_min",
        angle_min_rad,
        contract.angle_min_rad,
        contract.angle_min_tolerance_rad,
    )
    _require_close(
        "angle_increment",
        angle_increment_rad,
        contract.angle_increment_rad,
        contract.angle_increment_tolerance_rad,
    )
    _require_close(
        "range_min", range_min_m, contract.range_min_m, contract.range_tolerance_m
    )
    _require_close(
        "range_max", range_max_m, contract.range_max_m, contract.range_tolerance_m
    )

    valid = (
        np.isfinite(ranges)
        & (ranges >= float(range_min_m))
        & (ranges <= float(range_max_m))
    ).astype(np.uint8)
    return normalize_lidar_range_and_validity(
        ranges,
        valid,
        min_range_m=contract.range_min_m,
        max_range_m=contract.range_max_m,
    )


def prepare_speed_only_ego(
    longitudinal_speed_mps: float,
    *,
    scale_mps: float,
) -> np.ndarray:
    """Build static-v1 ego ``[1]`` from measured longitudinal velocity."""

    normalized = normalize_longitudinal_speed(
        longitudinal_speed_mps,
        scale_mps=scale_mps,
    )
    return np.asarray([normalized], dtype=np.float32)
