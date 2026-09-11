"""Typed configuration and construction boundary for the 30-step time model."""
from __future__ import annotations

from dataclasses import asdict, dataclass, fields
import math
from typing import Any

from torch import nn

from aic_transfuser_lite.models.time_path_v1 import TimePathV1


@dataclass(frozen=True)
class TimeModelConfig:
    """Complete, serializable identity of a TimePathV1 experiment.

    The command history default is deliberately off: it is an explicit
    diagnostic baseline and avoids silently depending on teacher-policy
    commands.  Dataset and runtime code should consume the same values.
    """

    image_height: int = 224
    image_width: int = 384
    lidar_points: int = 750
    lidar_min_range_m: float = 0.0
    lidar_max_range_m: float = 25.0
    lidar_angle_min_rad: float = -math.pi
    ego_dim: int = 4
    hidden_dim: int = 128
    camera_tokens_hw: tuple[int, int] = (4, 4)
    lidar_tokens: int = 32
    fusion_depth: int = 2
    fusion_heads: int = 4
    max_sensor_history: int = 4
    max_ego_history: int = 10
    command_history_length: int = 10
    ego_history_length: int = 10
    history_tolerance_ns: int = 50_000_000
    use_command_history: bool = False
    image_history_length: int = 4
    lidar_history_length: int = 4
    control_dt_sec: float = 0.1
    teacher_tolerance_ms: float = 50.0
    future_steps: int = 30
    future_dt_sec: float = 0.1
    capture_clock: str = "sim"
    available_clock: str = "bag_receipt"
    freeze_policy: str = "available_before_freeze"
    pose_world_frame: str = "map"
    pose_body_frame: str = "base_link"
    preprocess_version: str = "time_preprocess_v1"
    history_version: str = "valid_cnn_fixed_slot_masked_ego_v2"

    def validate(self) -> None:
        ints = (self.image_height, self.image_width, self.lidar_points,
                self.ego_dim, self.hidden_dim, self.lidar_tokens,
                self.fusion_depth, self.fusion_heads, self.max_sensor_history,
                self.max_ego_history, self.command_history_length,
                self.image_history_length, self.lidar_history_length,
                self.future_steps, self.ego_history_length)
        if any(type(value) is not int or value <= 0 for value in ints):
            raise ValueError("time model integer settings must be positive ints")
        if type(self.use_command_history) is not bool:
            raise TypeError("use_command_history must be bool")
        if type(self.camera_tokens_hw) is not tuple or len(self.camera_tokens_hw) != 2:
            raise TypeError("camera_tokens_hw must be a 2-tuple")
        if any(type(value) is not int or value <= 0 for value in self.camera_tokens_hw):
            raise ValueError("camera_tokens_hw values must be positive ints")
        if self.future_steps != 30 or self.future_dt_sec != 0.1:
            raise ValueError("TimePathV1 requires 30 future steps at 0.1 seconds")
        if self.image_history_length > self.max_sensor_history or self.lidar_history_length > self.max_sensor_history:
            raise ValueError("sensor history exceeds model maximum")
        if self.command_history_length > self.max_ego_history or self.ego_history_length > self.max_ego_history:
            raise ValueError("command history exceeds ego maximum")
        if (max(self.max_sensor_history,self.max_ego_history) > 10 or self.ego_dim != 4
                or self.lidar_points < 2 or self.hidden_dim % self.fusion_heads != 0):
            raise ValueError("invalid time backbone shape/history settings")
        if type(self.history_tolerance_ns) is not int or self.history_tolerance_ns < 0:
            raise ValueError("history tolerance must be integer ns")
        for value in (self.control_dt_sec, self.teacher_tolerance_ms, self.future_dt_sec,
                      self.lidar_max_range_m):
            if type(value) is not float or not math.isfinite(value) or value <= 0:
                raise ValueError("time intervals must be positive numbers")
        if (type(self.lidar_min_range_m) is not float
                or not math.isfinite(float(self.lidar_min_range_m))
                or self.lidar_min_range_m < 0):
            raise ValueError("lidar_min_range_m must be non-negative")
        if (self.lidar_min_range_m >= self.lidar_max_range_m
                or type(self.lidar_angle_min_rad) is not float
                or not math.isfinite(float(self.lidar_angle_min_rad))):
            raise ValueError("LiDAR range bounds are invalid")
        if self.control_dt_sec != 0.1:
            raise ValueError("control_dt_sec must be 0.1 seconds for time baseline")
        if (self.pose_body_frame != "base_link" or self.preprocess_version != "time_preprocess_v1"
                or self.history_version != "valid_cnn_fixed_slot_masked_ego_v2"
                or self.freeze_policy != "available_before_freeze"):
            raise ValueError("unsupported preprocessing/history/body contract")
        if not self.preprocess_version or not self.history_version:
            raise ValueError("preprocess/history versions are required")
        if not all(isinstance(value, str) and value for value in
                   (self.preprocess_version, self.history_version,
                    self.capture_clock, self.available_clock, self.freeze_policy,
                    self.pose_world_frame, self.pose_body_frame)):
            raise ValueError("clock/frame identities are required")

    def to_dict(self) -> dict[str, Any]:
        self.validate()
        value = asdict(self)
        value["camera_tokens_hw"] = list(self.camera_tokens_hw)
        return value

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "TimeModelConfig":
        if type(value) is not dict:
            raise TypeError("time model config must be a dict")
        names = {field.name for field in fields(cls)}
        missing = names - set(value)
        unknown = set(value) - names
        if missing or unknown:
            raise ValueError(f"time model config keys mismatch: missing={sorted(missing)}, unknown={sorted(unknown)}")
        data = dict(value)
        if "camera_tokens_hw" in data:
            data["camera_tokens_hw"] = tuple(data["camera_tokens_hw"])
        config = cls(**data)
        config.validate()
        return config

    def model_kwargs(self) -> dict[str, Any]:
        self.validate()
        return {"image_height": self.image_height, "image_width": self.image_width,
                "lidar_points": self.lidar_points, "ego_dim": self.ego_dim,
                "hidden_dim": self.hidden_dim, "camera_tokens_hw": self.camera_tokens_hw,
                "lidar_tokens": self.lidar_tokens, "fusion_depth": self.fusion_depth,
                "fusion_heads": self.fusion_heads, "max_sensor_history": self.max_sensor_history,
                "max_ego_history": self.max_ego_history, "trajectory_steps": 30,
                "control_head_enabled": False, "control_sequence_head_enabled": False,
                "behavior_head_enabled": False,
                "lidar_angle_min_rad": self.lidar_angle_min_rad}

    def dataset_config(self) -> Any:
        from aic_transfuser_lite.data.time_dataset_v1 import TimeDatasetConfig
        self.validate()
        return TimeDatasetConfig(
            image_shape=(3, self.image_height, self.image_width),
            lidar_shape=(2, self.lidar_points), ego_features=self.ego_dim,
            camera_history_length=self.image_history_length,
            lidar_history_length=self.lidar_history_length,
            ego_history_length=self.ego_history_length,
            command_history_length=self.command_history_length,
            tolerance_ns=self.history_tolerance_ns,
            teacher_tolerance_ms=self.teacher_tolerance_ms,
            lidar_min_range_m=self.lidar_min_range_m,
            lidar_max_range_m=self.lidar_max_range_m,
            lidar_angle_min_rad=self.lidar_angle_min_rad,
            capture_clock=self.capture_clock, available_clock=self.available_clock,
            pose_world_frame=self.pose_world_frame, pose_body_frame=self.pose_body_frame)


def build_time_model(config: TimeModelConfig) -> nn.Module:
    """Construct only the explicit 30-step TimePath model."""
    config.validate()
    return TimePathV1(**config.model_kwargs(), use_command_history=config.use_command_history)
