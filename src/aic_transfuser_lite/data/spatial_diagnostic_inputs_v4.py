"""Selected canonical inputs only, using the existing V3 preprocessing helpers."""
from __future__ import annotations

import io
from typing import Callable
import numpy as np
from PIL import Image
import torch

from aic_transfuser_lite.contracts.model_batch_v3 import ModelBatchV3
from .dataset_view_v3 import ControlTargetBoundsV3, _ego_row, _selected_command
from .image_preprocess import preprocess_image
from .normalization import normalize_lidar_range_and_validity
from aic_transfuser_lite.models.temporal.gru import select_epoch_history, select_epoch_history_before_anchor

FEATURES = ("longitudinal_speed_mps", "lateral_speed_mps", "yaw_rate_rps", "actual_steering_rad")
EGO_LIMITS = dict(zip(FEATURES, (20.0, 10.0, 5.0, 0.7)))
BOUNDS = ControlTargetBoundsV3(0.6, 0.8, 12.0, -4.0, 2.0, -8.0, 4.0, 0.1)
INPUT_FIELDS = ("image", "image_mask", "lidar", "lidar_mask", "ego", "ego_feature_mask",
                "command_history", "command_mask", "sensor_dt_sec")
INPUT_CONTRACT = {"version": "spatial_diagnostic_inputs_v4_v1", "history": [4, 4, 10, 10],
    "image_shape": [4, 3, 224, 384], "lidar_shape": [4, 2, 750], "ego_shape": [10, 4],
    "command_shape": [10, 3], "ego_features": FEATURES, "ego_abs_limits": EGO_LIMITS,
    "ego_units": ["m/s", "m/s", "rad/s", "rad"], "command_units": ["rad", "m/s", "m/s^2"],
    "command_alignment": "causal_previous_only", "command_priority": ["nominal", "final_fallback"],
    "image_normalization": "existing preprocess_image ImageNet RGB mean/std; no augmentation",
    "lidar_normalization": "existing normalize_lidar_range_and_validity 0..25m",
    "lidar_channel_meaning": ["normalized range", "validity"],
    "sensor_dt_meaning_s": ["camera_delta_ms/1000", "lidar_delta_ms/1000"],
    "sensor_dt_model_use": "retained and validated by V3; not fused by current backbone",
    "past_padding": "repeat earliest sensor/ego with false masks; zero commands with false masks",
    "reset_gap_s": 0.2, "teacher_forward_inputs": [], "runtime_connected": False}


def epoch_keys(rows: list[dict]) -> tuple:
    result, epoch = [], 0
    previous = None
    for row in rows:
        stamp = int(row["grid_stamp_ns"])
        key = (row["run_id"], row["segment_id"])
        if previous is not None and (key != previous[0] or not 0 < stamp-previous[1] <= 200_000_000):
            epoch += 1
        result.append((*key, epoch))
        previous = (key, stamp)
    return tuple(result)


def build_inputs(rows: list[dict], anchor: int, read_asset: Callable[[str], bytes],
                 *, height: int = 224, width: int = 384, lidar_points: int = 750,
                 keys: tuple | None = None) -> tuple[ModelBatchV3, dict]:
    keys = epoch_keys(rows) if keys is None else keys
    sensor = select_epoch_history(keys, anchor_index=anchor, length=4)
    ego = select_epoch_history(keys, anchor_index=anchor, length=10)
    command = select_epoch_history_before_anchor(keys, anchor_index=anchor, length=10)
    images, lidars, timing = [], [], []
    for idx in sensor.indices:
        row = rows[idx]
        with Image.open(io.BytesIO(read_asset(row["image_path"]))) as image:
            images.append(preprocess_image(image, height=height, width=width))
        ranges = np.load(io.BytesIO(read_asset(row["lidar_path"])), allow_pickle=False)
        valid_raw = np.load(io.BytesIO(read_asset(row["lidar_valid_path"])), allow_pickle=False)
        if ranges.shape != (lidar_points,) or valid_raw.shape != ranges.shape or not np.isin(valid_raw, [0, 1]).all():
            raise ValueError("LiDAR shape/mask mismatch")
        valid = valid_raw.astype(bool)
        if not np.isfinite(ranges[valid]).all():
            raise ValueError("nonfinite valid LiDAR input")
        lidars.append(torch.from_numpy(normalize_lidar_range_and_validity(ranges, valid, min_range_m=0.0, max_range_m=25.0)).float())
        timing.append([float(row["camera_delta_ms"])*1e-3, float(row["lidar_delta_ms"])*1e-3])
    ev = [_ego_row(rows[i], FEATURES, abs_limits=EGO_LIMITS) for i in ego.indices]
    commands = [_selected_command(rows[i], bounds=BOUNDS) if valid else None for i, valid in zip(command.indices, command.mask)]
    batch = ModelBatchV3(image=torch.stack(images)[None], image_mask=torch.tensor([sensor.mask]),
        lidar=torch.stack(lidars)[None], lidar_mask=torch.tensor([sensor.mask]),
        ego=torch.stack([x[0] for x in ev])[None],
        ego_feature_mask=(torch.stack([x[1] for x in ev]) & torch.tensor(ego.mask)[:, None])[None],
        command_history=torch.stack([c[0] if c else torch.zeros(3) for c in commands])[None],
        command_mask=torch.tensor([[c is not None for c in commands]]),
        sensor_dt_sec=torch.tensor([timing], dtype=torch.float32), targets=None, requested_outputs=frozenset({"trajectory"}))
    batch.validate()
    provenance = {"anchor_id": rows[anchor]["sample_id"], "epoch": keys[anchor],
        "sensor_ids": [rows[i]["sample_id"] for i in sensor.indices],
        "sensor_stamps_ns": [int(rows[i]["grid_stamp_ns"]) for i in sensor.indices],
        "ego_ids": [rows[i]["sample_id"] for i in ego.indices],
        "ego_stamps_ns": [int(rows[i]["grid_stamp_ns"]) for i in ego.indices],
        "command_ids": [rows[i]["sample_id"] if valid else None for i, valid in zip(command.indices, command.mask)],
        "command_stamps_ns": [int(rows[i]["grid_stamp_ns"]) if valid else None for i, valid in zip(command.indices, command.mask)],
        "command_sources": [c[1] if c else "padding_or_missing" for c in commands],
        "sensor_asset_paths": [{k: rows[i][k] for k in ("image_path", "lidar_path", "lidar_valid_path")} for i in sensor.indices]}
    if any(stamp is not None and stamp >= int(rows[anchor]["grid_stamp_ns"]) for stamp in provenance["command_stamps_ns"]):
        raise ValueError("noncausal command history")
    return batch, provenance


def combine_inputs(batches: list[ModelBatchV3], device: str | torch.device = "cpu") -> ModelBatchV3:
    return ModelBatchV3(**{k: torch.cat([getattr(b, k) for b in batches]).to(device) for k in INPUT_FIELDS},
                        targets=None, requested_outputs=frozenset({"trajectory"}))
