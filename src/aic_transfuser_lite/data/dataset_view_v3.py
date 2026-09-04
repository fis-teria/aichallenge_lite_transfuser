from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import csv
import json
import math
from typing import Mapping, Sequence

import numpy as np
from PIL import Image
import torch
import yaml

from .canonical_schema_v3 import CanonicalSampleV3
from .behavior_view_v1 import load_behavior_view_v1
from aic_transfuser_lite.contracts.model_batch_v3 import ModelBatchV3, TrainingTargetsV3
from aic_transfuser_lite.models.temporal.gru import (
    select_epoch_history,
    select_epoch_history_before_anchor,
)
from .storage_v3 import validate_complete_dataset
from .image_preprocess import preprocess_image
from .normalization import (
    IMAGENET_RGB_MEAN,
    IMAGENET_RGB_STD,
    normalize_lidar_range_and_validity,
    normalize_longitudinal_speed,
)


V1_COMPAT_VIEW_FORMAT = "aic_model_view_v1_compat_v1"


@dataclass(frozen=True)
class ControlTargetBoundsV3:
    """SI-unit bounds used to make teacher controls reachable by the V3 head."""

    max_steering_rad: float
    max_steering_rate_radps: float
    max_speed_mps: float
    min_acceleration_mps2: float
    max_acceleration_mps2: float
    min_jerk_mps3: float
    max_jerk_mps3: float
    control_dt_sec: float

    def validate(self) -> None:
        values = tuple(float(value) for value in self.__dict__.values())
        if not all(math.isfinite(value) for value in values):
            raise ValueError("control target bounds must be finite")
        if self.max_steering_rad <= 0.0 or self.max_steering_rate_radps <= 0.0:
            raise ValueError("control target steering bounds must be positive")
        if self.max_speed_mps <= 0.0 or self.control_dt_sec <= 0.0:
            raise ValueError("control target speed and time step must be positive")
        if not self.min_acceleration_mps2 < 0.0 < self.max_acceleration_mps2:
            raise ValueError("control target acceleration bounds must straddle zero")
        if not self.min_jerk_mps3 < 0.0 < self.max_jerk_mps3:
            raise ValueError("control target jerk bounds must straddle zero")


@dataclass(frozen=True)
class MotionTargetFilterConfigV3:
    """Reject commanded-motion anchors whose measured future stayed stationary.

    All speeds are in m/s, displacement is in metres, and ``horizon_steps`` is
    counted on the Dataset V3 dense future grid. The filter is deliberately
    conditional on a positive teacher command so genuine stop examples remain
    eligible.
    """

    enabled: bool = False
    stopped_speed_max_mps: float = 0.05
    commanded_speed_min_mps: float = 0.5
    minimum_future_speed_mps: float = 0.2
    minimum_future_displacement_m: float = 0.1
    horizon_steps: int = 15

    def validate(self) -> None:
        values = (
            self.stopped_speed_max_mps,
            self.commanded_speed_min_mps,
            self.minimum_future_speed_mps,
            self.minimum_future_displacement_m,
        )
        if any(not math.isfinite(float(value)) or value < 0.0 for value in values):
            raise ValueError("motion target filter thresholds must be finite and non-negative")
        if self.commanded_speed_min_mps <= self.stopped_speed_max_mps:
            raise ValueError("commanded motion threshold must exceed stopped speed threshold")
        if self.minimum_future_speed_mps <= 0.0:
            raise ValueError("minimum future speed must be positive")
        if self.minimum_future_displacement_m <= 0.0:
            raise ValueError("minimum future displacement must be positive")
        if self.horizon_steps <= 0:
            raise ValueError("motion target filter horizon_steps must be positive")


def stationary_commanded_motion_target_v3(
    future_state: np.ndarray,
    *,
    current_speed_mps: float,
    commanded_speed_mps: float,
    config: MotionTargetFilterConfigV3,
) -> bool:
    """Return true only for a contradictory commanded-motion training target."""

    config.validate()
    if not config.enabled:
        return False
    current_speed = abs(float(current_speed_mps))
    commanded_speed = float(commanded_speed_mps)
    if not math.isfinite(current_speed) or not math.isfinite(commanded_speed):
        raise ValueError("motion target filter speeds must be finite")
    if (
        current_speed > config.stopped_speed_max_mps
        or commanded_speed < config.commanded_speed_min_mps
    ):
        return False
    future = np.asarray(future_state)
    if future.ndim != 2 or future.shape[1] != 8:
        raise ValueError("motion target filter requires dense future state [H,8]")
    horizon = future[: config.horizon_steps]
    valid = horizon[:, 7].astype(bool)
    if not bool(valid.any()):
        return False
    selected = horizon[valid]
    if not np.isfinite(selected[:, 1:5]).all():
        raise ValueError("valid motion target filter future values must be finite")
    maximum_displacement_m = float(np.linalg.norm(selected[:, 1:3], axis=1).max())
    maximum_forward_speed_mps = float(np.maximum(selected[:, 4], 0.0).max())
    return (
        maximum_forward_speed_mps < config.minimum_future_speed_mps
        and maximum_displacement_m < config.minimum_future_displacement_m
    )


def clip_control_target_v3(
    control: torch.Tensor, *, bounds: ControlTargetBoundsV3
) -> torch.Tensor:
    """Clip one ``[steering rad, speed m/s, acceleration m/s^2]`` teacher."""

    bounds.validate()
    if control.shape != (3,):
        raise ValueError("current teacher control must be [3]")
    if not torch.isfinite(control).all():
        raise ValueError("current teacher control must be finite")
    return torch.stack(
        (
            control[0].clamp(-bounds.max_steering_rad, bounds.max_steering_rad),
            control[1].clamp(0.0, bounds.max_speed_mps),
            control[2].clamp(
                bounds.min_acceleration_mps2, bounds.max_acceleration_mps2
            ),
        )
    )


def project_teacher_control_sequence_v3(
    commands: torch.Tensor,
    mask: torch.Tensor,
    *,
    initial_steering_rad: float,
    initial_acceleration_mps2: float,
    bounds: ControlTargetBoundsV3,
) -> torch.Tensor:
    """Project teacher ``[H,3]`` controls onto the V3 head's reachable set.

    Steering is in rad, speed in m/s, acceleration in m/s^2, steering rate in
    rad/s, jerk in m/s^3, and ``control_dt_sec`` in seconds. Masked steps remain
    zero and do not advance the projected state.
    """

    bounds.validate()
    if commands.ndim != 2 or commands.shape[1] != 3 or mask.shape != commands.shape:
        raise ValueError("teacher controls and mask must both be [H,3]")
    if mask.dtype != torch.bool:
        raise ValueError("teacher control mask must be boolean")
    row_valid = mask.all(dim=1)
    if not torch.equal(mask, row_valid.unsqueeze(1).expand_as(mask)):
        raise ValueError("teacher control mask rows must be all-valid or all-invalid")
    if not torch.isfinite(commands[mask]).all():
        raise ValueError("valid teacher controls must be finite")
    initial = commands.new_tensor([initial_steering_rad, initial_acceleration_mps2])
    if not torch.isfinite(initial).all():
        raise ValueError("initial teacher control state must be finite")
    projected = torch.zeros_like(commands)
    steering = float(
        max(
            -bounds.max_steering_rad,
            min(bounds.max_steering_rad, initial_steering_rad),
        )
    )
    acceleration = float(
        max(
            bounds.min_acceleration_mps2,
            min(bounds.max_acceleration_mps2, initial_acceleration_mps2),
        )
    )
    steering_step = bounds.max_steering_rate_radps * bounds.control_dt_sec
    minimum_acceleration_step = bounds.min_jerk_mps3 * bounds.control_dt_sec
    maximum_acceleration_step = bounds.max_jerk_mps3 * bounds.control_dt_sec
    for index in range(commands.shape[0]):
        if not bool(mask[index].all()):
            continue
        proposal = commands[index]
        target_steering = float(
            proposal[0].clamp(-bounds.max_steering_rad, bounds.max_steering_rad)
        )
        target_acceleration = float(
            proposal[2].clamp(
                bounds.min_acceleration_mps2, bounds.max_acceleration_mps2
            )
        )
        steering += max(
            -steering_step, min(steering_step, target_steering - steering)
        )
        acceleration += max(
            minimum_acceleration_step,
            min(maximum_acceleration_step, target_acceleration - acceleration),
        )
        projected[index] = commands.new_tensor(
            [
                steering,
                float(proposal[1].clamp(0.0, bounds.max_speed_mps)),
                acceleration,
            ],
        )
    return projected


@dataclass(frozen=True)
class V1CompatibilityViewConfig:
    view_id: str
    image_height: int
    image_width: int
    lidar_points: int
    lidar_min_range_m: float
    lidar_max_range_m: float
    ego_speed_scale_mps: float
    waypoint_times_sec: tuple[float, ...]
    target_speed_offset_sec: float
    require_all_targets_valid: bool
    format_version: str = V1_COMPAT_VIEW_FORMAT

    def validate(self) -> None:
        if self.format_version != V1_COMPAT_VIEW_FORMAT or self.view_id != "v1_compat":
            raise ValueError("unsupported V1 compatibility view identity")
        if self.image_height <= 0 or self.image_width <= 0 or self.lidar_points < 2:
            raise ValueError("image dimensions and lidar_points must be positive")
        if self.lidar_max_range_m <= self.lidar_min_range_m:
            raise ValueError("LiDAR maximum range must exceed minimum range")
        if self.ego_speed_scale_mps <= 0.0:
            raise ValueError("ego_speed_scale_mps must be positive")
        if len(self.waypoint_times_sec) != 6 or any(
            right <= left
            for left, right in zip(self.waypoint_times_sec, self.waypoint_times_sec[1:])
        ):
            raise ValueError("V1 compatibility requires six increasing waypoint times")
        if self.target_speed_offset_sec <= 0.0:
            raise ValueError("target_speed_offset_sec must be positive")


@dataclass(frozen=True)
class V1CompatibilityRecord:
    sample_id: str
    run_id: str
    split: str
    tensors: dict[str, torch.Tensor]


def load_v1_compatibility_view_config(path: str | Path) -> V1CompatibilityViewConfig:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError("V1 compatibility view config root must be a mapping")
    expected = {
        "format_version",
        "view_id",
        "image_height",
        "image_width",
        "lidar_points",
        "lidar_min_range_m",
        "lidar_max_range_m",
        "ego_speed_scale_mps",
        "waypoint_times_sec",
        "target_speed_offset_sec",
        "require_all_targets_valid",
    }
    if set(raw) != expected:
        raise ValueError(
            f"V1 view config fields mismatch: missing={sorted(expected-set(raw))}, "
            f"unknown={sorted(set(raw)-expected)}"
        )
    if not isinstance(raw["waypoint_times_sec"], list) or not isinstance(
        raw["require_all_targets_valid"], bool
    ):
        raise ValueError("waypoint times must be a list and validity policy a boolean")
    config = V1CompatibilityViewConfig(
        format_version=str(raw["format_version"]),
        view_id=str(raw["view_id"]),
        image_height=int(raw["image_height"]),
        image_width=int(raw["image_width"]),
        lidar_points=int(raw["lidar_points"]),
        lidar_min_range_m=float(raw["lidar_min_range_m"]),
        lidar_max_range_m=float(raw["lidar_max_range_m"]),
        ego_speed_scale_mps=float(raw["ego_speed_scale_mps"]),
        waypoint_times_sec=tuple(float(value) for value in raw["waypoint_times_sec"]),
        target_speed_offset_sec=float(raw["target_speed_offset_sec"]),
        require_all_targets_valid=raw["require_all_targets_valid"],
    )
    config.validate()
    return config


class V1CompatibilityViewBuilder:
    """Build the exact static V1 tensor/target contract from one V3 sample."""

    def __init__(self, config: V1CompatibilityViewConfig) -> None:
        config.validate()
        self.config = config

    def build_record(
        self,
        sample: CanonicalSampleV3,
        *,
        image_rgb: np.ndarray | Image.Image,
        lidar_ranges_m: np.ndarray,
        lidar_valid: np.ndarray,
        split: str,
    ) -> V1CompatibilityRecord:
        sample.validate()
        if not split:
            raise ValueError("V1 compatibility record requires an explicit split")
        image = image_rgb if isinstance(image_rgb, Image.Image) else Image.fromarray(np.asarray(image_rgb))
        image_tensor = preprocess_image(
            image.convert("RGB"),
            height=self.config.image_height,
            width=self.config.image_width,
            mean=IMAGENET_RGB_MEAN,
            std=IMAGENET_RGB_STD,
        ).float()
        ranges = np.asarray(lidar_ranges_m, dtype=np.float32)
        valid = np.asarray(lidar_valid)
        if ranges.shape != (self.config.lidar_points,) or valid.shape != ranges.shape:
            raise ValueError(
                f"V1 compatibility requires LiDAR {(self.config.lidar_points,)}, "
                f"got {ranges.shape}/{valid.shape}"
            )
        lidar = torch.from_numpy(
            normalize_lidar_range_and_validity(
                ranges,
                valid,
                min_range_m=self.config.lidar_min_range_m,
                max_range_m=self.config.lidar_max_range_m,
            )
        ).float()
        speed = sample.ego_state.longitudinal_speed_mps
        speed.validate(field_name="ego_state.longitudinal_speed_mps")
        ego = torch.tensor(
            [normalize_longitudinal_speed(speed.value, scale_mps=self.config.ego_speed_scale_mps)],
            dtype=torch.float32,
        )
        future = sample.future_state
        if future is None:
            raise ValueError("V1 compatibility requires dense future state")
        future.validate()
        waypoint_indices = [
            _exact_time_index(future.relative_time_sec, value)
            for value in self.config.waypoint_times_sec
        ]
        speed_index = _exact_time_index(
            future.relative_time_sec, self.config.target_speed_offset_sec
        )
        target_indices = waypoint_indices + [speed_index]
        if self.config.require_all_targets_valid and not bool(future.valid[target_indices].all()):
            raise ValueError("V1 compatibility targets contain invalid future steps")
        waypoints = torch.from_numpy(
            np.stack([future.x_m[waypoint_indices], future.y_m[waypoint_indices]], axis=1).astype(
                np.float32
            )
        )
        target_speed = torch.tensor(
            [float(future.longitudinal_speed_mps[speed_index])], dtype=torch.float32
        )
        tensors = {
            "image": image_tensor,
            "lidar": lidar,
            "ego": ego,
            "waypoints": waypoints,
            "target_speed": target_speed,
        }
        expected = {
            "image": (3, self.config.image_height, self.config.image_width),
            "lidar": (2, self.config.lidar_points),
            "ego": (1,),
            "waypoints": (6, 2),
            "target_speed": (1,),
        }
        for name, shape in expected.items():
            if tuple(tensors[name].shape) != shape:
                raise AssertionError(f"V1 compatibility {name} shape drifted")
            if not torch.isfinite(tensors[name]).all():
                raise ValueError(f"V1 compatibility {name} contains non-finite values")
        return V1CompatibilityRecord(sample.sample_id, sample.run_id, split, tensors)


def _exact_time_index(values: np.ndarray, target_sec: float) -> int:
    array = np.asarray(values, dtype=np.float64)
    matches = np.flatnonzero(np.isclose(array, target_sec, rtol=0.0, atol=1e-6))
    if len(matches) != 1:
        raise ValueError(f"dense future does not contain exactly one {target_sec:.3f} s step")
    return int(matches[0])


@dataclass(frozen=True)
class _LazyTemporalTrainingBatchesV3(Sequence[ModelBatchV3]):
    root: Path
    rows: list[dict[str, str]]
    epoch_keys: tuple[tuple[str, str], ...]
    row_index_by_epoch_stamp: dict[tuple[str, str, int], int]
    usable_anchors: list[int]
    behavior_by_sample: dict[str, dict[str, str]] | None
    image_height: int
    image_width: int
    lidar_points: int
    lidar_min_range_m: float
    lidar_max_range_m: float
    ego_features: tuple[str, ...]
    ego_abs_limits: Mapping[str, float] | None
    trajectory_steps: int
    control_sequence_steps: int
    camera_history_length: int
    ego_history_length: int
    command_history_length: int
    control_target_bounds: ControlTargetBoundsV3
    batch_size: int
    max_batches: int | None
    motion_target_rejected_count: int

    def __len__(self) -> int:
        count = math.ceil(len(self.usable_anchors) / self.batch_size)
        return count if self.max_batches is None else min(count, self.max_batches)

    def class_counts(self, target_name: str, class_count: int) -> torch.Tensor:
        """Count behavior labels without materializing image or LiDAR assets."""

        columns = {
            "behavior_class": ("behavior_class", "behavior_valid"),
            "behavior_side": ("behavior_side", "behavior_side_valid"),
        }
        if target_name not in columns or class_count <= 0:
            raise ValueError("unsupported class-count request")
        counts = torch.zeros(class_count, dtype=torch.long)
        if self.behavior_by_sample is None:
            return counts
        value_column, valid_column = columns[target_name]
        batch_limit = len(self.usable_anchors)
        if self.max_batches is not None:
            batch_limit = min(batch_limit, self.max_batches * self.batch_size)
        for anchor in self.usable_anchors[:batch_limit]:
            annotation = self.behavior_by_sample.get(self.rows[anchor]["sample_id"])
            if annotation is None or not _csv_bool(annotation[valid_column]):
                continue
            value = int(annotation[value_column])
            if value < 0 or value >= class_count:
                raise ValueError(f"{target_name} label is outside configured classes")
            counts[value] += 1
        return counts

    def __getitem__(self, index: int) -> ModelBatchV3:
        count = len(self)
        normalized = index + count if index < 0 else index
        if normalized < 0 or normalized >= count:
            raise IndexError(index)
        start = normalized * self.batch_size
        anchors = self.usable_anchors[start : start + self.batch_size]
        chunk = [self._materialize_sample(anchor) for anchor in anchors]
        stack = lambda key: torch.stack([item[key] for item in chunk])  # noqa: E731
        targets = TrainingTargetsV3(
            trajectory_xy_m=stack("trajectory"),
            trajectory_mask=stack("trajectory_mask"),
            speed_mps=stack("speed"),
            speed_mask=stack("trajectory_mask"),
            current_control=stack("control"),
            current_control_mask=torch.ones(len(chunk), 3, dtype=torch.bool),
            control_provenance=tuple(str(item["provenance"]) for item in chunk),
            control_sequence=stack("control_sequence"),
            control_sequence_mask=stack("control_sequence_mask"),
            control_sequence_provenance=tuple(
                tuple(str(value) for value in item["control_sequence_provenance"])
                for item in chunk
            ),
            control_sequence_time_sec=stack("control_sequence_time_sec"),
            behavior_class=stack("behavior_class"),
            behavior_mask=stack("behavior_mask"),
            behavior_side=stack("behavior_side"),
            behavior_side_mask=stack("behavior_side_mask"),
        )
        requested_outputs = {
            "trajectory", "speed_profile", "current_control", "control_sequence"
        }
        if self.behavior_by_sample is not None:
            requested_outputs.update({"behavior", "behavior_side"})
        return ModelBatchV3(
            image=stack("image"),
            image_mask=stack("image_mask"),
            lidar=stack("lidar"),
            lidar_mask=stack("lidar_mask"),
            ego=stack("ego"),
            ego_feature_mask=stack("ego_feature_mask"),
            command_history=stack("command_history"),
            command_mask=stack("command_mask"),
            sensor_dt_sec=stack("sensor_dt_sec"),
            targets=targets,
            requested_outputs=frozenset(requested_outputs),
        )

    def _materialize_sample(self, anchor: int) -> dict[str, torch.Tensor | str]:
        row = self.rows[anchor]
        sensor_selection = select_epoch_history(
            self.epoch_keys, anchor_index=anchor, length=self.camera_history_length
        )
        ego_selection = select_epoch_history(
            self.epoch_keys, anchor_index=anchor, length=self.ego_history_length
        )
        command_selection = select_epoch_history_before_anchor(
            self.epoch_keys,
            anchor_index=anchor,
            length=self.command_history_length,
        )
        sensor_rows = [self.rows[index] for index in sensor_selection.indices]
        ego_rows = [self.rows[index] for index in ego_selection.indices]
        command_rows = [self.rows[index] for index in command_selection.indices]
        image = torch.stack([self._load_image(item) for item in sensor_rows])
        lidar_values = [self._load_lidar(item) for item in sensor_rows]
        ego_values, ego_masks = [], []
        for item in ego_rows:
            values, mask = _ego_row(
                item,
                self.ego_features,
                abs_limits=self.ego_abs_limits,
            )
            ego_values.append(values)
            ego_masks.append(mask)
        command_values, command_masks = [], []
        for item in command_rows:
            selected = _selected_command(item, bounds=self.control_target_bounds)
            command_values.append(torch.zeros(3) if selected is None else selected[0])
            command_masks.append(selected is not None)
        trajectory = np.load(self.root / row["trajectory_path"], allow_pickle=False)
        if (
            trajectory.ndim != 2
            or trajectory.shape[0] < self.trajectory_steps
            or trajectory.shape[1] != 8
        ):
            raise ValueError("dense trajectory asset shape mismatch")
        future = trajectory[: self.trajectory_steps]
        command = _selected_command(row, bounds=self.control_target_bounds)
        if command is None:
            raise AssertionError("eligible anchor lost its full-control command")
        target_values, provenance = command
        sequence_values: list[torch.Tensor] = []
        sequence_masks: list[torch.Tensor] = []
        sequence_provenance: list[str] = []
        # Step zero is the immediate teacher command. Later unavailable steps
        # on the exact time grid remain masked instead of compressing time.
        sequence_indices = select_control_sequence_row_indices_v3(
            self.rows,
            self.row_index_by_epoch_stamp,
            anchor_index=anchor,
            steps=self.control_sequence_steps,
            control_dt_sec=self.control_target_bounds.control_dt_sec,
        )
        for future_index in sequence_indices:
            future_command = None
            if future_index is not None:
                future_command = _selected_command(
                    self.rows[future_index], bounds=self.control_target_bounds
                )
            sequence_values.append(
                torch.zeros(3) if future_command is None else future_command[0]
            )
            sequence_masks.append(
                torch.full((3,), future_command is not None, dtype=torch.bool)
            )
            sequence_provenance.append(
                "missing_exact_timestamp"
                if future_command is None
                else future_command[1]
            )
        sequence = torch.stack(sequence_values)
        sequence_mask = torch.stack(sequence_masks)
        steering_index = (
            self.ego_features.index("actual_steering_rad")
            if "actual_steering_rad" in self.ego_features
            else None
        )
        initial_steering = (
            float(ego_values[-1][steering_index]) if steering_index is not None else 0.0
        )
        previous_acceleration = (
            float(command_values[-1][2])
            if command_masks[-1] and command_selection.mask[-1]
            else 0.0
        )
        sequence = project_teacher_control_sequence_v3(
            sequence,
            sequence_mask,
            initial_steering_rad=initial_steering,
            initial_acceleration_mps2=previous_acceleration,
            bounds=self.control_target_bounds,
        )
        annotation = (
            None
            if self.behavior_by_sample is None
            else self.behavior_by_sample.get(row["sample_id"])
        )
        behavior_valid = annotation is not None and _csv_bool(annotation["behavior_valid"])
        side_valid = annotation is not None and _csv_bool(annotation["behavior_side_valid"])
        return {
            "image": image,
            "image_mask": torch.tensor(sensor_selection.mask),
            "lidar": torch.stack(lidar_values),
            "lidar_mask": torch.tensor(sensor_selection.mask),
            "ego": torch.stack(ego_values),
            "ego_feature_mask": torch.stack(ego_masks),
            "command_history": torch.stack(command_values),
            "command_mask": torch.tensor(command_masks)
            & torch.tensor(command_selection.mask),
            "sensor_dt_sec": torch.zeros(self.camera_history_length, 2),
            "trajectory": torch.from_numpy(future[:, 1:3]).float(),
            "trajectory_mask": torch.from_numpy(future[:, 7].astype(bool)),
            "speed": torch.from_numpy(future[:, 4]).float(),
            "control": target_values,
            "provenance": provenance,
            "control_sequence": sequence,
            "control_sequence_mask": sequence_mask,
            "control_sequence_provenance": tuple(sequence_provenance),
            "control_sequence_time_sec": torch.arange(
                self.control_sequence_steps, dtype=torch.float32
            )
            * self.control_target_bounds.control_dt_sec,
            "behavior_class": torch.tensor(
                int(annotation["behavior_class"]) if behavior_valid else -1,
                dtype=torch.long,
            ),
            "behavior_mask": torch.tensor(behavior_valid),
            "behavior_side": torch.tensor(
                int(annotation["behavior_side"]) if side_valid else -1,
                dtype=torch.long,
            ),
            "behavior_side_mask": torch.tensor(side_valid),
        }

    def _load_image(self, row: dict[str, str]) -> torch.Tensor:
        with Image.open(self.root / row["image_path"]) as image:
            return preprocess_image(
                image,
                height=self.image_height,
                width=self.image_width,
            )

    def _load_lidar(self, row: dict[str, str]) -> torch.Tensor:
        ranges = np.load(self.root / row["lidar_path"], allow_pickle=False)
        valid = np.load(self.root / row["lidar_valid_path"], allow_pickle=False).astype(bool)
        if ranges.shape != (self.lidar_points,) or valid.shape != ranges.shape:
            raise ValueError("Dataset V3 LiDAR shape differs from model config")
        return torch.from_numpy(
            normalize_lidar_range_and_validity(
                ranges,
                valid,
                min_range_m=self.lidar_min_range_m,
                max_range_m=self.lidar_max_range_m,
            )
        ).float()


def load_temporal_training_batches_v3(
    dataset_root: str | Path,
    split_manifest_path: str | Path,
    *,
    split: str,
    image_height: int,
    image_width: int,
    lidar_points: int,
    lidar_min_range_m: float,
    lidar_max_range_m: float,
    ego_features: tuple[str, ...],
    trajectory_steps: int,
    control_sequence_steps: int,
    camera_history_length: int,
    ego_history_length: int,
    command_history_length: int,
    control_target_bounds: ControlTargetBoundsV3,
    batch_size: int,
    ego_abs_limits: Mapping[str, float] | None = None,
    max_batches: int | None = None,
    behavior_view_root: str | Path | None = None,
    motion_target_filter: MotionTargetFilterConfigV3 | None = None,
) -> Sequence[ModelBatchV3]:
    """Create leakage-safe lazy temporal batches from Dataset V3.

    CSV rows and eligible anchor indices stay in memory, while image/LiDAR and
    trajectory tensors are materialized only for the requested batch. This
    bounds host memory independently of Dataset duration.
    """
    root = Path(dataset_root)
    dataset_manifest = validate_complete_dataset(root)
    target_filter = motion_target_filter or MotionTargetFilterConfigV3()
    target_filter.validate()
    behavior_by_sample = (
        None
        if behavior_view_root is None
        else load_behavior_view_v1(
            behavior_view_root,
            dataset_manifest_sha256=str(dataset_manifest["manifest_sha256"]),
        )
    )
    split_manifest = json.loads(Path(split_manifest_path).read_text(encoding="utf-8"))
    if split_manifest.get("dataset_manifest_sha256") != dataset_manifest["manifest_sha256"]:
        raise ValueError("split manifest targets a different Dataset V3 manifest")
    assigned = {
        item["run_id"] for item in split_manifest.get("assignments", []) if item.get("split") == split
    }
    if not assigned:
        raise ValueError(f"split {split!r} contains no runs")
    if (
        batch_size <= 0
        or trajectory_steps <= 0
        or control_sequence_steps <= 0
        or camera_history_length <= 0
        or ego_history_length <= 0
        or command_history_length <= 0
    ):
        raise ValueError("batch size and history/trajectory/control steps must be positive")
    with (root / "samples.csv").open(newline="", encoding="utf-8") as stream:
        rows = [row for row in csv.DictReader(stream) if row["run_id"] in assigned]
    rows.sort(key=lambda row: (row["run_id"], row["segment_id"], int(row["grid_stamp_ns"])))
    epoch_keys = tuple((item["run_id"], item["segment_id"]) for item in rows)
    row_index_by_epoch_stamp: dict[tuple[str, str, int], int] = {}
    for index, row in enumerate(rows):
        key = (row["run_id"], row["segment_id"], int(row["grid_stamp_ns"]))
        if key in row_index_by_epoch_stamp:
            raise ValueError("duplicate run/segment/grid timestamp in Dataset V3")
        row_index_by_epoch_stamp[key] = index
    if ego_abs_limits is not None:
        unknown_limits = set(ego_abs_limits) - set(ego_features)
        if unknown_limits:
            raise ValueError(
                f"ego_abs_limits has unknown features: {sorted(unknown_limits)}"
            )
        if any(
            not math.isfinite(float(value)) or float(value) <= 0.0
            for value in ego_abs_limits.values()
        ):
            raise ValueError("ego_abs_limits values must be finite and positive")
    usable_anchors: list[int] = []
    motion_target_rejected_count = 0
    for anchor, row in enumerate(rows):
        selected_command = _selected_command(row, bounds=control_target_bounds)
        if selected_command is None:
            continue
        if int(row["future_valid_count"]) <= 0:
            continue
        _, current_ego_mask = _ego_row(
            row,
            ego_features,
            abs_limits=ego_abs_limits,
        )
        if not bool(current_ego_mask.all()):
            continue
        current_speed_mps = float(row["velocity_longitudinal_mps"])
        commanded_speed_mps = float(selected_command[0][1])
        if (
            target_filter.enabled
            and abs(current_speed_mps) <= target_filter.stopped_speed_max_mps
            and commanded_speed_mps >= target_filter.commanded_speed_min_mps
        ):
            future = np.load(root / row["trajectory_path"], allow_pickle=False)
            if stationary_commanded_motion_target_v3(
                future,
                current_speed_mps=current_speed_mps,
                commanded_speed_mps=commanded_speed_mps,
                config=target_filter,
            ):
                motion_target_rejected_count += 1
                continue
        usable_anchors.append(anchor)
    if not usable_anchors:
        raise ValueError("no full-control-capable samples in selected split")
    if behavior_by_sample is not None and not any(
        (annotation := behavior_by_sample.get(rows[anchor]["sample_id"])) is not None
        and _csv_bool(annotation["behavior_valid"])
        for anchor in usable_anchors
    ):
        raise ValueError("behavior view has no valid behavior labels in selected split")
    return _LazyTemporalTrainingBatchesV3(
        root=root,
        rows=rows,
        epoch_keys=epoch_keys,
        row_index_by_epoch_stamp=row_index_by_epoch_stamp,
        usable_anchors=usable_anchors,
        behavior_by_sample=behavior_by_sample,
        image_height=image_height,
        image_width=image_width,
        lidar_points=lidar_points,
        lidar_min_range_m=lidar_min_range_m,
        lidar_max_range_m=lidar_max_range_m,
        ego_features=ego_features,
        ego_abs_limits=ego_abs_limits,
        trajectory_steps=trajectory_steps,
        control_sequence_steps=control_sequence_steps,
        camera_history_length=camera_history_length,
        ego_history_length=ego_history_length,
        command_history_length=command_history_length,
        control_target_bounds=control_target_bounds,
        batch_size=batch_size,
        max_batches=max_batches,
        motion_target_rejected_count=motion_target_rejected_count,
    )


def select_control_sequence_row_indices_v3(
    rows: Sequence[dict[str, str]],
    row_index_by_epoch_stamp: dict[tuple[str, str, int], int],
    *,
    anchor_index: int,
    steps: int,
    control_dt_sec: float,
) -> tuple[int | None, ...]:
    """Select exact ``t + k*dt`` rows without crossing a run/segment epoch."""

    if not 0 <= anchor_index < len(rows):
        raise IndexError("control sequence anchor is outside Dataset V3 rows")
    if steps <= 0 or not math.isfinite(control_dt_sec) or control_dt_sec <= 0.0:
        raise ValueError("control sequence steps and dt must be positive")
    anchor = rows[anchor_index]
    anchor_stamp_ns = int(anchor["grid_stamp_ns"])
    dt_ns = int(round(control_dt_sec * 1_000_000_000.0))
    if dt_ns <= 0:
        raise ValueError("control sequence dt rounds to zero nanoseconds")
    epoch = (anchor["run_id"], anchor["segment_id"])
    return tuple(
        row_index_by_epoch_stamp.get(
            (epoch[0], epoch[1], anchor_stamp_ns + offset * dt_ns)
        )
        for offset in range(steps)
    )


def _selected_command(
    row: dict[str, str], *, bounds: ControlTargetBoundsV3 | None = None
) -> tuple[torch.Tensor, str] | None:
    for field, provenance in (("nominal_command", "nominal"), ("final_command", "final_fallback")):
        value = json.loads(row[field])
        if bool(value.get("valid")):
            tensor = torch.tensor([
                value["steering_rad"], value["speed_mps"], value["acceleration_mps2"]
            ], dtype=torch.float32)
            if torch.isfinite(tensor).all():
                if bounds is not None:
                    tensor = clip_control_target_v3(tensor, bounds=bounds)
                return tensor, provenance
    return None


def _ego_row(
    row: dict[str, str],
    features: tuple[str, ...],
    *,
    abs_limits: Mapping[str, float] | None = None,
) -> tuple[torch.Tensor, torch.Tensor]:
    columns = {
        "longitudinal_speed_mps": "velocity_longitudinal_mps",
        "lateral_speed_mps": "velocity_lateral_mps",
        "yaw_rate_rps": "yaw_rate_rps",
        "actual_steering_rad": "actual_steering_rad",
    }
    values, valid = [], []
    for feature in features:
        if feature not in columns:
            raise ValueError(f"unsupported ego feature: {feature}")
        value = float(row[columns[feature]])
        is_valid = math.isfinite(value) and (
            feature != "actual_steering_rad" or row["actual_steering_valid"].lower() == "true"
        )
        if is_valid and abs_limits is not None and feature in abs_limits:
            is_valid = abs(value) <= float(abs_limits[feature])
        values.append(value if is_valid else 0.0)
        valid.append(is_valid)
    return torch.tensor(values, dtype=torch.float32), torch.tensor(valid, dtype=torch.bool)


def _csv_bool(value: str) -> bool:
    normalized = str(value).strip().lower()
    if normalized not in {"true", "false"}:
        raise ValueError(f"invalid behavior view boolean: {value!r}")
    return normalized == "true"
