from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Mapping


class RuntimeProfile(str, Enum):
    TRAJECTORY_ONLY = "trajectory_only"
    EXTERNAL_CONTROLLER = "external_controller"
    SHADOW_CONTROL = "shadow_control"
    BOUNDED_RESIDUAL = "bounded_residual"
    FULL_CONTROL = "full_control"


@dataclass(frozen=True)
class OutputProfile:
    requested_outputs: frozenset[str]
    publisher_topics: frozenset[str]
    nominal_control_authority: bool


_BASE_TOPICS = frozenset({"predicted_trajectory", "runtime_status", "runtime_sync_debug"})
_PROFILES: Mapping[RuntimeProfile, OutputProfile] = {
    RuntimeProfile.TRAJECTORY_ONLY: OutputProfile(
        frozenset({"trajectory", "speed_profile"}), _BASE_TOPICS, False
    ),
    RuntimeProfile.EXTERNAL_CONTROLLER: OutputProfile(
        frozenset({"trajectory", "speed_profile"}), _BASE_TOPICS, False
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
        frozenset({"trajectory", "speed_profile", "current_control", "control_sequence"}),
        _BASE_TOPICS | {"nominal_control_cmd"},
        True,
    ),
}


def output_profile(name: str | RuntimeProfile) -> OutputProfile:
    try:
        return _PROFILES[RuntimeProfile(name)]
    except ValueError as error:
        raise ValueError(f"unknown runtime profile: {name!r}") from error


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
