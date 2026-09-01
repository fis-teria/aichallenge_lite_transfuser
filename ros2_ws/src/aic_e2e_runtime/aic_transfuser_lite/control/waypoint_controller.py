from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class ControllerConfig:
    wheelbase_m: float = 1.0
    min_lookahead_m: float = 1.0
    max_steer_rad: float = 0.6
    min_accel_mps2: float = -4.0
    max_accel_mps2: float = 2.0
    speed_kp: float = 1.0


@dataclass(frozen=True)
class ControlCommand:
    steering_rad: float
    acceleration_mps2: float


def select_lookahead(waypoints: np.ndarray, min_distance_m: float) -> np.ndarray:
    points = np.asarray(waypoints, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) == 0:
        raise ValueError(f"Expected waypoints [N,2], got {points.shape}")
    distances = np.linalg.norm(points, axis=1)
    indices = np.flatnonzero(distances >= min_distance_m)
    return points[int(indices[0])] if len(indices) else points[-1]


def control_from_waypoints(
    waypoints: np.ndarray,
    target_speed_mps: float,
    current_speed_mps: float,
    config: ControllerConfig = ControllerConfig(),
) -> ControlCommand:
    target = select_lookahead(waypoints, config.min_lookahead_m)
    x, y = float(target[0]), float(target[1])
    lookahead_sq = max(x * x + y * y, 1e-6)
    curvature = 2.0 * y / lookahead_sq
    steering = math.atan(config.wheelbase_m * curvature)
    steering = float(np.clip(steering, -config.max_steer_rad, config.max_steer_rad))
    acceleration = config.speed_kp * (float(target_speed_mps) - float(current_speed_mps))
    acceleration = float(
        np.clip(acceleration, config.min_accel_mps2, config.max_accel_mps2)
    )
    return ControlCommand(steering_rad=steering, acceleration_mps2=acceleration)
