from __future__ import annotations

import copy
from pathlib import Path

import numpy as np
import pandas as pd
from PIL import Image
import pytest
import torch
from torch import nn
import yaml

from aic_transfuser_lite.config import load_v1_config
from aic_transfuser_lite.training.checkpoint_v1 import load_v1_checkpoint
from aic_transfuser_lite.training import train_v1 as training_module


ROOT = Path(__file__).resolve().parents[1]


class TinyV2TrainingModel(nn.Module):
    """Small differentiable model for trainer orchestration tests."""

    def __init__(self) -> None:
        super().__init__()
        self.camera = nn.Module()
        self.camera.backbone = nn.Sequential(
            nn.Conv2d(3, 2, kernel_size=1),
            nn.AdaptiveAvgPool2d((1, 1)),
        )
        self.head = nn.Linear(2 + 2 + 1, 13)

    def forward(
        self,
        image: torch.Tensor,
        lidar: torch.Tensor,
        ego: torch.Tensor,
    ) -> dict[str, torch.Tensor]:
        image_features = self.camera.backbone(image).flatten(1)
        lidar_features = lidar.mean(dim=-1)
        values = self.head(torch.cat((image_features, lidar_features, ego), dim=1))
        return {
            "waypoints": values[:, :12].reshape(-1, 6, 2),
            "target_speed": values[:, 12:13],
        }


def circular_path(curvature_per_m: float) -> np.ndarray:
    distances = np.arange(0.5, 3.01, 0.5, dtype=np.float32)
    if curvature_per_m == 0.0:
        return np.stack((distances, np.zeros_like(distances)), axis=-1)
    radius = 1.0 / curvature_per_m
    theta = distances / radius
    return np.stack(
        (radius * np.sin(theta), radius * (1.0 - np.cos(theta))), axis=-1
    )


def sample_row(
    *,
    sample_id: str,
    run_id: str,
    split: str,
    curvature_per_m: float,
    timestamp_ns: int,
) -> dict[str, object]:
    row: dict[str, object] = {
        "sample_id": sample_id,
        "run_id": run_id,
        "scenario_id": "synthetic_training_contract",
        "timestamp_ns": timestamp_ns,
        "grid_timestamp_ns": timestamp_ns,
        "image_path": f"images/{sample_id}.png",
        "lidar_path": f"lidar/{sample_id}.npy",
        "lidar_valid_path": f"lidar_valid/{sample_id}.npy",
        "velocity_longitudinal_mps": 3.0,
        "velocity_lateral_mps": 0.0,
        "yaw_rate_rps": 0.0,
        "gear": 1,
        "actual_steering_rad": np.nan,
        "actual_steering_valid": 0,
        "nominal_command_steering_rad": 0.0,
        "nominal_command_speed_mps": 3.0,
        "nominal_command_acceleration_mps2": 0.0,
        "final_command_steering_rad": 0.0,
        "final_command_speed_mps": 3.0,
        "final_command_acceleration_mps2": 0.0,
        "target_speed_mps": 3.0 + curvature_per_m,
        "teacher_command_steering_rad": 0.0,
        "teacher_command_acceleration_mps2": 0.0,
        "collision": 0,
        "offtrack": 0,
        "recovery_flag": 0,
        "quality_score": 1.0,
        "label_provenance": "measured_pose",
        "pose_frame_id": "map",
        "pose_child_frame_id": "base_link",
        "pose_x_world_m": 0.0,
        "pose_y_world_m": 0.0,
        "pose_yaw_world_rad": 0.0,
        "lidar_points": 750,
        "camera_dt_ms": 0.0,
        "lidar_dt_ms": 5.0,
        "pose_dt_ms": 0.0,
        "velocity_dt_ms": 0.0,
        "steering_dt_ms": np.nan,
        "nominal_command_age_ms": 10.0,
        "final_command_age_ms": 10.0,
        "split": split,
    }
    for index, point in enumerate(circular_path(curvature_per_m)):
        row[f"wp_{index}_x"] = float(point[0])
        row[f"wp_{index}_y"] = float(point[1])
    return row


def write_synthetic_dataset(root: Path) -> tuple[Path, Path]:
    for directory in ("images", "lidar", "lidar_valid"):
        (root / directory).mkdir(parents=True, exist_ok=True)
    metadata = {
        "format_version": 2,
        "vehicle_config_provenance": {
            "path": "synthetic/vehicle_info.param.yaml",
            "sha256": "0" * 64,
            "wheelbase_m": 1.087,
            "max_steering_rad": 0.64,
        },
        "lidar_geometry": {
            "saved_points": 750,
            "source_points": 750,
            "angle_min_rad": -1.5666074752807617,
            "angle_increment_rad": 0.004188789986073971,
            "range_min_m": 0.0,
            "range_max_m": 25.0,
            "resampling": "none_native_beam_order",
        },
    }
    (root / "metadata.yaml").write_text(
        yaml.safe_dump(metadata, sort_keys=False), encoding="utf-8"
    )
    train_rows: list[dict[str, object]] = []
    val_rows: list[dict[str, object]] = []
    curvatures = (0.0, 0.08, 0.18, 0.0, 0.08, 0.18)
    for split, run_id, rows in (
        ("train", "synthetic_train", train_rows),
        ("val", "synthetic_val", val_rows),
    ):
        for index, curvature in enumerate(curvatures):
            sample_id = f"{run_id}_{index:06d}"
            rows.append(
                sample_row(
                    sample_id=sample_id,
                    run_id=run_id,
                    split=split,
                    curvature_per_m=curvature,
                    timestamp_ns=1_000_000_000 + index * 100_000_000,
                )
            )
            Image.new("RGB", (12, 8), color=(64 + index, 128, 192)).save(
                root / f"images/{sample_id}.png"
            )
            np.save(
                root / f"lidar/{sample_id}.npy",
                np.linspace(1.0, 25.0, 750, dtype=np.float32),
            )
            np.save(
                root / f"lidar_valid/{sample_id}.npy",
                np.ones(750, dtype=np.uint8),
            )
    train_index = root / "train_index.csv"
    val_index = root / "val_index.csv"
    pd.DataFrame(train_rows).to_csv(train_index, index=False)
    pd.DataFrame(val_rows).to_csv(val_index, index=False)
    return train_index, val_index


def write_training_config(path: Path) -> None:
    config = copy.deepcopy(load_v1_config(ROOT / "configs/transfuser_lite_v1_static.yaml"))
    config["model"]["camera"]["pretrained"] = False
    config["data"]["augmentation"]["enabled"] = False
    config["training"].update(
        {
            "batch_size": 3,
            "epochs": 2,
            "num_workers": 0,
            "pin_memory": False,
            "persistent_workers": False,
            "prefetch_factor": None,
            "freeze_backbone_epochs": 0,
        }
    )
    path.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")


def test_dataset_v2_amp_checkpoint_pause_and_resume(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    dataset_root = tmp_path / "dataset"
    train_index, val_index = write_synthetic_dataset(dataset_root)
    config_path = tmp_path / "config.yaml"
    write_training_config(config_path)
    output = tmp_path / "run"
    monkeypatch.setattr(
        training_module,
        "build_model",
        lambda config: TinyV2TrainingModel(),
    )
    device = "cuda" if torch.cuda.is_available() else "cpu"

    paused = training_module.train_v1(
        config_path=config_path,
        train_index=train_index,
        val_index=val_index,
        output=output,
        stop_after_epoch=1,
        requested_device=device,
    )
    assert paused["status"] == "PAUSED"
    assert paused["completed_epoch"] == 1
    assert paused["global_step"] == 2
    assert paused["amp_effective"] is torch.cuda.is_available()
    first_checkpoint = load_v1_checkpoint(output / "last.pt")
    assert first_checkpoint["epoch"] == 1
    assert first_checkpoint["global_step"] == 2

    completed = training_module.train_v1(
        config_path=config_path,
        train_index=train_index,
        val_index=val_index,
        output=output,
        resume=output / "last.pt",
        requested_device=device,
    )
    assert completed["status"] == "COMPLETED"
    assert completed["completed_epoch"] == 2
    assert completed["global_step"] == 4
    final_checkpoint = load_v1_checkpoint(output / "last.pt")
    assert final_checkpoint["epoch"] == 2
    assert final_checkpoint["global_step"] == 4
    assert set(completed["selected_checkpoints"]) == {
        "best_ade.pt",
        "best_corner_control.pt",
        "best_speed.pt",
    }
    assert (
        completed["selected_checkpoints"]["best_corner_control.pt"]["metric"]
        == "sharp_controller_proxy_mae_rad"
    )
    history = yaml.safe_load((output / "history.json").read_text(encoding="utf-8"))
    assert [record["epoch"] for record in history] == [1, 2]
    assert history[-1]["validation"]["curvature_buckets"]["sharp"][
        "sample_count"
    ] == 2
