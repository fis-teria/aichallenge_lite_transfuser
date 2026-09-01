from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Mapping, Sequence


DATASET_FORMAT_VERSION_V2 = 2

V2_MODEL_INPUT_COLUMNS = (
    "sample_id",
    "run_id",
    "scenario_id",
    "timestamp_ns",
    "image_path",
    "lidar_path",
    "lidar_valid_path",
    "velocity_longitudinal_mps",
    "gear",
)

V2_STATE_COLUMNS = (
    "velocity_lateral_mps",
    "yaw_rate_rps",
    "actual_steering_rad",
    "actual_steering_valid",
    "nominal_command_steering_rad",
    "final_command_steering_rad",
)

V2_TEACHER_DEBUG_COLUMNS = (
    "target_speed_mps",
    "teacher_command_steering_rad",
    "teacher_command_acceleration_mps2",
    "collision",
    "offtrack",
    "recovery_flag",
    "quality_score",
    "label_provenance",
)

V2_QUALITY_COLUMNS = (
    "grid_timestamp_ns",
    "camera_dt_ms",
    "lidar_dt_ms",
    "pose_dt_ms",
    "velocity_dt_ms",
    "steering_dt_ms",
    "nominal_command_age_ms",
    "final_command_age_ms",
)


@dataclass(frozen=True)
class DatasetContract:
    image_height: int
    image_width: int
    lidar_points: int
    ego_dim: int
    num_waypoints: int


@dataclass(frozen=True)
class DatasetV2Contract:
    """Versioned static-baseline data contract.

    ``lidar_points`` is inferred from one native LaserScan geometry and must be
    constant across the dataset. It is never filled with a design-time beam
    count guess.
    """

    lidar_points: int
    num_waypoints: int = 6
    format_version: int = DATASET_FORMAT_VERSION_V2

    def validate(self) -> None:
        if self.format_version != DATASET_FORMAT_VERSION_V2:
            raise ValueError(
                f"Expected format version {DATASET_FORMAT_VERSION_V2}, got {self.format_version}"
            )
        if self.lidar_points <= 1:
            raise ValueError("lidar_points must be greater than one")
        if self.num_waypoints <= 0:
            raise ValueError("num_waypoints must be positive")


@dataclass(frozen=True)
class SamplePaths:
    image_path: Path
    lidar_path: Path


@dataclass(frozen=True)
class SamplePathsV2:
    image_path: Path
    lidar_path: Path
    lidar_valid_path: Path


def required_v2_columns(num_waypoints: int) -> tuple[str, ...]:
    if num_waypoints <= 0:
        raise ValueError("num_waypoints must be positive")
    waypoint_columns = tuple(
        column
        for index in range(num_waypoints)
        for column in (f"wp_{index}_x", f"wp_{index}_y")
    )
    return tuple(
        dict.fromkeys(
            V2_MODEL_INPUT_COLUMNS
            + V2_STATE_COLUMNS
            + V2_TEACHER_DEBUG_COLUMNS
            + V2_QUALITY_COLUMNS
            + waypoint_columns
        )
    )


def validate_v2_columns(columns: Sequence[str], *, num_waypoints: int) -> None:
    available = set(columns)
    missing = sorted(set(required_v2_columns(num_waypoints)).difference(available))
    if missing:
        raise ValueError(f"Missing required Dataset v2 columns: {missing}")
    shortcut_leaks = {
        "actual_steering_rad",
        "nominal_command_steering_rad",
        "final_command_steering_rad",
        "teacher_command_steering_rad",
    }.intersection(V2_MODEL_INPUT_COLUMNS)
    if shortcut_leaks:
        raise AssertionError(f"Teacher/state columns leaked into model inputs: {shortcut_leaks}")


def validate_v2_row(row: Mapping[str, Any], *, num_waypoints: int) -> None:
    """Validate one row without replacing missing measured state with zero."""

    validate_v2_columns(tuple(row), num_waypoints=num_waypoints)
    if row["label_provenance"] != "measured_pose":
        raise ValueError("Dataset v2 waypoint labels must have measured_pose provenance")
    actual_valid = int(row["actual_steering_valid"])
    if actual_valid not in (0, 1):
        raise ValueError("actual_steering_valid must be 0 or 1")
    actual_value = float(row["actual_steering_rad"])
    if actual_valid == 0 and math.isfinite(actual_value):
        raise ValueError(
            "Missing actual steering must be NaN with actual_steering_valid=0, not zero-filled"
        )
    if actual_valid == 1 and not math.isfinite(actual_value):
        raise ValueError("Valid actual steering must be finite")
    finite_required = (
        "velocity_longitudinal_mps",
        "velocity_lateral_mps",
        "yaw_rate_rps",
        "target_speed_mps",
        "quality_score",
    )
    for name in finite_required:
        if not math.isfinite(float(row[name])):
            raise ValueError(f"{name} must be finite")
    for index in range(num_waypoints):
        for axis in ("x", "y"):
            name = f"wp_{index}_{axis}"
            if not math.isfinite(float(row[name])):
                raise ValueError(f"{name} must be finite")
