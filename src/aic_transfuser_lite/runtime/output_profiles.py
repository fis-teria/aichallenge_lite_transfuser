from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Mapping

import numpy as np


class RuntimeProfile(str, Enum):
    TRAJECTORY_ONLY = "trajectory_only"
    EXTERNAL_CONTROLLER = "external_controller"
    SHADOW_CONTROL = "shadow_control"
    BOUNDED_RESIDUAL = "bounded_residual"
    FULL_CONTROL = "full_control"
    BEHAVIOR_DIAGNOSTIC = "behavior_diagnostic"


@dataclass(frozen=True)
class OutputProfile:
    requested_outputs: frozenset[str]
    publisher_topics: frozenset[str]
    nominal_control_authority: bool


@dataclass(frozen=True)
class TrajectorySpeedPublication:
    """Flattened candidate-zero outputs for ROS publication in SI units."""

    trajectory_xy_m: tuple[float, ...]
    speed_profile_mps: tuple[float, ...]
    point_count: int


@dataclass(frozen=True)
class TrajectoryPathPublication:
    """ROS-independent path payload in a declared coordinate frame."""

    frame_id: str
    points_xy_m: tuple[tuple[float, float], ...]


_BASE_TOPICS = frozenset({
    "predicted_trajectory",
    "predicted_trajectory_path",
    "predicted_speed_profile",
    "runtime_status",
    "runtime_sync_debug",
})
_PROFILES: Mapping[RuntimeProfile, OutputProfile] = {
    RuntimeProfile.TRAJECTORY_ONLY: OutputProfile(
        frozenset({"trajectory", "speed_profile"}), _BASE_TOPICS, False
    ),
    RuntimeProfile.EXTERNAL_CONTROLLER: OutputProfile(
        frozenset({"trajectory", "speed_profile"}),
        _BASE_TOPICS | {"shadow_external_control"},
        False,
    ),
    RuntimeProfile.SHADOW_CONTROL: OutputProfile(
        frozenset({"trajectory", "speed_profile", "current_control"}),
        _BASE_TOPICS | {"shadow_model_control"},
        False,
    ),
    RuntimeProfile.BOUNDED_RESIDUAL: OutputProfile(
        frozenset({"trajectory", "speed_profile", "current_control"}),
        _BASE_TOPICS | {"bounded_residual_control"},
        True,
    ),
    RuntimeProfile.FULL_CONTROL: OutputProfile(
        frozenset({
            "trajectory", "speed_profile", "current_control", "control_sequence",
            "behavior", "behavior_side",
        }),
        _BASE_TOPICS | {
            "nominal_control_cmd", "behavior_mode", "behavior_label",
            "behavior_confidence", "behavior_side",
        },
        True,
    ),
    RuntimeProfile.BEHAVIOR_DIAGNOSTIC: OutputProfile(
        frozenset({"trajectory", "speed_profile", "behavior", "behavior_side"}),
        _BASE_TOPICS | {
            "behavior_mode", "behavior_label", "behavior_confidence", "behavior_side"
        },
        False,
    ),
}


def output_profile(name: str | RuntimeProfile) -> OutputProfile:
    try:
        return _PROFILES[RuntimeProfile(name)]
    except ValueError as error:
        raise ValueError(f"unknown runtime profile: {name!r}") from error


def trajectory_speed_publication(
    trajectory_xy: np.ndarray,
    trajectory_speed_mps: np.ndarray,
) -> TrajectorySpeedPublication:
    """Validate ``[1,K,N,2]``/``[1,K,N]`` and select candidate zero.

    The returned trajectory is flattened as ``[x0,y0,...]`` in metres and the
    speed profile contains one non-negative value per point in metres/second.
    Both outputs are validated before either ROS message is published.
    """

    points = np.asarray(trajectory_xy)
    speeds = np.asarray(trajectory_speed_mps)
    if points.ndim != 4 or points.shape[0] != 1 or points.shape[1] < 1:
        raise ValueError(f"trajectory_xy must be [1,K,N,2], got {points.shape}")
    if points.shape[2] < 1 or points.shape[3] != 2:
        raise ValueError(f"trajectory_xy must be [1,K,N,2], got {points.shape}")
    if speeds.shape != points.shape[:-1]:
        raise ValueError(
            "trajectory_speed_mps must match trajectory [1,K,N], "
            f"got {speeds.shape} for {points.shape}"
        )
    selected_points = points[0, 0]
    selected_speeds = speeds[0, 0]
    if not np.isfinite(selected_points).all():
        raise ValueError("trajectory_xy must be finite")
    if not np.isfinite(selected_speeds).all():
        raise ValueError("trajectory_speed_mps must be finite")
    if bool((selected_speeds < 0.0).any()):
        raise ValueError("trajectory_speed_mps must be non-negative")
    return TrajectorySpeedPublication(
        trajectory_xy_m=tuple(float(value) for value in selected_points.reshape(-1)),
        speed_profile_mps=tuple(float(value) for value in selected_speeds),
        point_count=int(selected_points.shape[0]),
    )


def trajectory_path_publication(
    trajectory_xy_m: tuple[float, ...],
    *,
    frame_id: str,
) -> TrajectoryPathPublication:
    """Validate flattened ``[N,2]`` metres for a stamped ROS Path message."""

    if not isinstance(frame_id, str) or not frame_id.strip():
        raise ValueError("trajectory frame_id must be a non-empty string")
    points = np.asarray(trajectory_xy_m, dtype=np.float64)
    if points.ndim != 1 or points.size < 2 or points.size % 2 != 0:
        raise ValueError(
            "trajectory_xy_m must be flattened [N,2] with at least one point"
        )
    if not np.isfinite(points).all():
        raise ValueError("trajectory_xy_m must be finite")
    shaped = points.reshape(-1, 2)
    return TrajectoryPathPublication(
        frame_id=frame_id,
        points_xy_m=tuple((float(point[0]), float(point[1])) for point in shaped),
    )


def runtime_clock_has_reached_observation(
    *,
    now_sec: float,
    source_stamps_sec: Mapping[str, float],
    future_tolerance_sec: float = 0.001,
) -> bool:
    """Return whether ROS time has reached every selected sensor stamp."""

    if not math.isfinite(now_sec) or now_sec <= 0.0:
        raise ValueError("invalid_runtime_clock")
    if not math.isfinite(future_tolerance_sec) or future_tolerance_sec < 0.0:
        raise ValueError("invalid_future_tolerance")
    if not source_stamps_sec:
        raise ValueError("source_stamps_empty")
    if any(
        not math.isfinite(stamp) or stamp <= 0.0
        for stamp in source_stamps_sec.values()
    ):
        raise ValueError("invalid_timestamp")
    return max(source_stamps_sec.values()) <= now_sec + future_tolerance_sec


def validate_observation_timing(
    *,
    now_sec: float,
    camera_stamp_sec: float,
    role_stamps_sec: Mapping[str, float],
    timeout_sec: float,
    max_skew_sec: float,
    future_tolerance_sec: float = 0.001,
) -> None:
    values = {"camera": camera_stamp_sec, **role_stamps_sec}
    if any(not math.isfinite(value) or value <= 0.0 for value in values.values()):
        raise ValueError("invalid_timestamp")
    if not math.isfinite(now_sec) or now_sec <= 0.0:
        raise ValueError("invalid_runtime_clock")
    future = [name for name, value in values.items() if value > now_sec + future_tolerance_sec]
    if future:
        raise ValueError("future_timestamp:" + ",".join(sorted(future)))
    stale = [name for name, value in values.items() if now_sec - value > timeout_sec]
    if stale:
        raise ValueError("stale:" + ",".join(sorted(stale)))
    skew = max(values.values()) - min(values.values())
    if skew > max_skew_sec:
        raise ValueError(f"sensor_skew:{skew:.6f}")
