from __future__ import annotations

"""ROS-message-like adapters kept free of ROS imports for unit testing."""

from collections.abc import Sequence
import math
from typing import Any

import numpy as np

from aic_transfuser_lite.data.ego_features import LEGACY_EGO_FEATURES, select_ego_features


def stamp_to_seconds(stamp: Any, fallback_sec: float) -> float:
    sec = float(getattr(stamp, "sec", 0))
    nanosec = float(getattr(stamp, "nanosec", 0))
    value = sec + nanosec * 1e-9
    return value if value > 0.0 and math.isfinite(value) else float(fallback_sec)


def strict_message_stamp_to_seconds(message: Any) -> float:
    """Read ``header.stamp`` or ``stamp`` without a wall/sim-time fallback."""

    for path in (("header", "stamp"), ("stamp",)):
        value = message
        try:
            for name in path:
                value = getattr(value, name)
            stamp = float(value.sec) + float(value.nanosec) * 1e-9
        except (AttributeError, TypeError, ValueError):
            continue
        if math.isfinite(stamp) and stamp > 0.0:
            return stamp
    raise ValueError("message has no finite positive header.stamp or stamp")


def velocity_report_to_state(message: Any) -> tuple[float, float]:
    """Return measured longitudinal speed [m/s] and heading rate [rad/s]."""

    longitudinal = float(message.longitudinal_velocity)
    yaw_rate = float(message.heading_rate)
    if not math.isfinite(longitudinal) or not math.isfinite(yaw_rate):
        raise ValueError("velocity report contains NaN or infinity")
    return longitudinal, yaw_rate


def steering_report_to_angle(message: Any) -> float:
    """Return measured steering tire angle in radians."""

    steering = float(message.steering_tire_angle)
    if not math.isfinite(steering):
        raise ValueError("steering report contains NaN or infinity")
    return steering


def image_message_to_rgb(message: Any) -> np.ndarray:
    """Convert sensor_msgs/Image-compatible data to RGB uint8 ``[H,W,3]``."""
    height = int(message.height)
    width = int(message.width)
    encoding = str(message.encoding).lower()
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(encoding)
    if height <= 0 or width <= 0 or channels is None:
        raise ValueError(f"unsupported image geometry/encoding: {width}x{height} {encoding}")
    row_bytes = width * channels
    step = int(message.step)
    if step < row_bytes:
        raise ValueError(f"image step={step} is smaller than row bytes={row_bytes}")
    raw = np.frombuffer(message.data, dtype=np.uint8)
    if raw.size < height * step:
        raise ValueError("image data is shorter than height*step")
    rows = raw[: height * step].reshape(height, step)[:, :row_bytes]
    image = rows.reshape(height, width, channels)
    if encoding in {"bgr8", "bgra8"}:
        image = image[:, :, [2, 1, 0, 3] if channels == 4 else [2, 1, 0]]
    return np.ascontiguousarray(image[:, :, :3])


def resample_laser_ranges(ranges_m: np.ndarray, output_points: int) -> np.ndarray:
    """Nearest-neighbour resample a one-dimensional LaserScan in beam order."""
    source = np.asarray(ranges_m, dtype=np.float32)
    if source.ndim != 1 or source.size < 2:
        raise ValueError(f"expected at least two laser beams, got {source.shape}")
    if output_points < 2:
        raise ValueError("output_points must be at least two")
    indices = np.rint(np.linspace(0, source.size - 1, output_points)).astype(np.int64)
    return source[indices].astype(np.float32, copy=True)


def odometry_to_ego(
    odometry: Any,
    *,
    previous_steering_rad: float,
    forward_gear_value: float = 1.0,
    ego_features: Sequence[str] = LEGACY_EGO_FEATURES,
) -> tuple[np.ndarray, float]:
    """Return the configured model ego vector and scalar speed in m/s."""
    linear = odometry.twist.twist.linear
    angular = odometry.twist.twist.angular
    vx = float(linear.x)
    vy = float(linear.y)
    yaw_rate = float(angular.z)
    speed_mps = math.hypot(vx, vy)
    values = np.asarray(
        select_ego_features(
            ego_features,
            {
                "speed_mps": speed_mps,
                "longitudinal_speed_mps": vx,
                "lateral_speed_mps": vy,
                "yaw_rate_rps": yaw_rate,
                "steering_rad": float(previous_steering_rad),
                "gear": float(forward_gear_value),
            },
        ),
        dtype=np.float32,
    )
    if not np.isfinite(values).all():
        raise ValueError("odometry ego values contain NaN or infinity")
    return values, speed_mps
