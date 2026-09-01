from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any
import csv
import json
import math

import numpy as np
from PIL import Image
import torch
import yaml

from .canonical_schema_v3 import CanonicalSampleV3
from aic_transfuser_lite.contracts.model_batch_v3 import ModelBatchV3, TrainingTargetsV3
from aic_transfuser_lite.models.temporal.gru import select_epoch_history
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
    camera_history_length: int,
    ego_history_length: int,
    batch_size: int,
    max_batches: int | None = None,
) -> list[ModelBatchV3]:
    """Materialize leakage-safe temporal full-control batches from Dataset V3."""
    root = Path(dataset_root)
    dataset_manifest = validate_complete_dataset(root)
    split_manifest = json.loads(Path(split_manifest_path).read_text(encoding="utf-8"))
    if split_manifest.get("dataset_manifest_sha256") != dataset_manifest["manifest_sha256"]:
        raise ValueError("split manifest targets a different Dataset V3 manifest")
    assigned = {
        item["run_id"] for item in split_manifest.get("assignments", []) if item.get("split") == split
    }
    if not assigned:
        raise ValueError(f"split {split!r} contains no runs")
    if batch_size <= 0 or trajectory_steps <= 0:
        raise ValueError("batch_size and trajectory_steps must be positive")
    with (root / "samples.csv").open(newline="", encoding="utf-8") as stream:
        rows = [row for row in csv.DictReader(stream) if row["run_id"] in assigned]
    rows.sort(key=lambda row: (row["run_id"], row["segment_id"], int(row["grid_stamp_ns"])))
    usable: list[dict[str, torch.Tensor | str]] = []
    for anchor, row in enumerate(rows):
        command = _selected_command(row)
        if command is None:
            continue
        sensor_selection = select_epoch_history(
            [(item["run_id"], item["segment_id"]) for item in rows],
            anchor_index=anchor,
            length=camera_history_length,
        )
        ego_selection = select_epoch_history(
            [(item["run_id"], item["segment_id"]) for item in rows],
            anchor_index=anchor,
            length=ego_history_length,
        )
        sensor_rows = [rows[index] for index in sensor_selection.indices]
        ego_rows = [rows[index] for index in ego_selection.indices]
        image = torch.stack([
            preprocess_image(Image.open(root / item["image_path"]), height=image_height, width=image_width)
            for item in sensor_rows
        ])
        lidar_values = []
        for item in sensor_rows:
            ranges = np.load(root / item["lidar_path"], allow_pickle=False)
            valid = np.load(root / item["lidar_valid_path"], allow_pickle=False).astype(bool)
            if ranges.shape != (lidar_points,) or valid.shape != ranges.shape:
                raise ValueError("Dataset V3 LiDAR shape differs from model config")
            lidar_values.append(torch.from_numpy(normalize_lidar_range_and_validity(
                ranges, valid, min_range_m=lidar_min_range_m, max_range_m=lidar_max_range_m
            )).float())
        ego_values, ego_masks, command_values, command_masks = [], [], [], []
        for item in ego_rows:
            values, mask = _ego_row(item, ego_features)
            ego_values.append(values)
            ego_masks.append(mask)
            selected = _selected_command(item)
            command_values.append(torch.zeros(3) if selected is None else selected[0])
            command_masks.append(selected is not None)
        if not bool(ego_masks[-1].all()):
            continue
        trajectory = np.load(root / row["trajectory_path"], allow_pickle=False)
        if trajectory.ndim != 2 or trajectory.shape[0] < trajectory_steps or trajectory.shape[1] != 8:
            raise ValueError("dense trajectory asset shape mismatch")
        future = trajectory[:trajectory_steps]
        target_values, provenance = command
        usable.append({
            "image": image, "image_mask": torch.tensor(sensor_selection.mask),
            "lidar": torch.stack(lidar_values), "lidar_mask": torch.tensor(sensor_selection.mask),
            "ego": torch.stack(ego_values), "ego_feature_mask": torch.stack(ego_masks),
            "command_history": torch.stack(command_values),
            "command_mask": torch.tensor(command_masks) & torch.tensor(ego_selection.mask),
            "sensor_dt_sec": torch.zeros(camera_history_length, 2),
            "trajectory": torch.from_numpy(future[:, 1:3]).float(),
            "trajectory_mask": torch.from_numpy(future[:, 7].astype(bool)),
            "speed": torch.from_numpy(future[:, 4]).float(),
            "control": target_values, "provenance": provenance,
        })
    if not usable:
        raise ValueError("no full-control-capable samples in selected split")
    batches: list[ModelBatchV3] = []
    for start in range(0, len(usable), batch_size):
        chunk = usable[start : start + batch_size]
        stack = lambda key: torch.stack([item[key] for item in chunk])  # noqa: E731
        targets = TrainingTargetsV3(
            trajectory_xy_m=stack("trajectory"), trajectory_mask=stack("trajectory_mask"),
            speed_mps=stack("speed"), speed_mask=stack("trajectory_mask"),
            current_control=stack("control"),
            current_control_mask=torch.ones(len(chunk), 3, dtype=torch.bool),
            control_provenance=tuple(str(item["provenance"]) for item in chunk),
        )
        batches.append(ModelBatchV3(
            image=stack("image"), image_mask=stack("image_mask"),
            lidar=stack("lidar"), lidar_mask=stack("lidar_mask"),
            ego=stack("ego"), ego_feature_mask=stack("ego_feature_mask"),
            command_history=stack("command_history"), command_mask=stack("command_mask"),
            sensor_dt_sec=stack("sensor_dt_sec"), targets=targets,
            requested_outputs=frozenset({"trajectory", "speed_profile", "current_control"}),
        ))
        if max_batches is not None and len(batches) >= max_batches:
            break
    return batches


def _selected_command(row: dict[str, str]) -> tuple[torch.Tensor, str] | None:
    for field, provenance in (("nominal_command", "nominal"), ("final_command", "final_fallback")):
        value = json.loads(row[field])
        if bool(value.get("valid")):
            tensor = torch.tensor([
                value["steering_rad"], value["speed_mps"], value["acceleration_mps2"]
            ], dtype=torch.float32)
            if torch.isfinite(tensor).all():
                return tensor, provenance
    return None


def _ego_row(row: dict[str, str], features: tuple[str, ...]) -> tuple[torch.Tensor, torch.Tensor]:
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
        values.append(value if is_valid else 0.0)
        valid.append(is_valid)
    return torch.tensor(values, dtype=torch.float32), torch.tensor(valid, dtype=torch.bool)
