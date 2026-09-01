from __future__ import annotations

import copy
from pathlib import Path

import numpy as np
import pandas as pd
from PIL import Image
import pytest
import torch
import yaml

from aic_transfuser_lite.config import (
    ConfigValidationError,
    load_v1_config,
    validate_v1_config,
    validate_v2_data_config,
)
from aic_transfuser_lite.data.dataset_v2 import DrivingDatasetV2


ROOT = Path(__file__).resolve().parents[1]


def data_config(*, lidar_points: int = 4) -> dict:
    return {
        "sample_rate_hz": 10.0,
        "sync_tolerance_ms": 30.0,
        "image_height": 4,
        "image_width": 6,
        "lidar_points": lidar_points,
        "lidar_min_range_m": 0.0,
        "lidar_max_range_m": 25.0,
        "ego_dim": 1,
        "ego_features": ["speed_mps"],
        "prediction_horizon_sec": 3.0,
        "num_waypoints": 6,
        "mode_classes": {"follow": 0},
        "format_version": 2,
        "lidar_angle_min_rad": -0.75,
        "lidar_angle_increment_rad": 0.5,
        "ego_speed_scale_mps": 10.0,
        "augmentation": {
            "enabled": False,
            "camera": {
                "brightness_delta": 0.15,
                "contrast_delta": 0.15,
                "gamma_min": 0.9,
                "gamma_max": 1.1,
                "blur_probability": 0.1,
                "blur_radius_max_px": 1.0,
                "noise_probability": 0.1,
                "noise_std_fraction": 0.01,
            },
            "lidar": {
                "range_noise_sigma_min_m": 0.01,
                "range_noise_sigma_max_m": 0.03,
                "beam_dropout_max_fraction": 0.02,
                "sector_dropout_probability": 0.1,
                "sector_dropout_max_degrees": 5.0,
            },
        },
    }


def sample_row() -> dict:
    row = {
        "sample_id": "run_a_000000",
        "run_id": "run_a",
        "scenario_id": "normal_course",
        "timestamp_ns": 1_000_000_000,
        "grid_timestamp_ns": 1_000_000_000,
        "image_path": "images/run_a_000000.png",
        "lidar_path": "lidar/run_a_000000.npy",
        "lidar_valid_path": "lidar_valid/run_a_000000.npy",
        "velocity_longitudinal_mps": 5.0,
        "velocity_lateral_mps": 0.1,
        "yaw_rate_rps": 0.2,
        "gear": 1,
        "actual_steering_rad": np.nan,
        "actual_steering_valid": 0,
        "nominal_command_steering_rad": 0.4,
        "nominal_command_speed_mps": 5.5,
        "nominal_command_acceleration_mps2": 0.1,
        "final_command_steering_rad": 0.3,
        "final_command_speed_mps": 5.0,
        "final_command_acceleration_mps2": 0.0,
        "target_speed_mps": 4.5,
        "teacher_command_steering_rad": 0.4,
        "teacher_command_acceleration_mps2": 0.1,
        "collision": 0,
        "offtrack": 0,
        "recovery_flag": 0,
        "quality_score": 1.0,
        "label_provenance": "measured_pose",
        "pose_frame_id": "map",
        "pose_child_frame_id": "base_link",
        "pose_x_world_m": 1.0,
        "pose_y_world_m": 2.0,
        "pose_yaw_world_rad": 0.3,
        "lidar_points": 4,
        "camera_dt_ms": 0.0,
        "lidar_dt_ms": 5.0,
        "pose_dt_ms": 0.0,
        "velocity_dt_ms": 0.0,
        "steering_dt_ms": np.nan,
        "nominal_command_age_ms": 10.0,
        "final_command_age_ms": 10.0,
        "split": "train",
    }
    for index in range(6):
        row[f"wp_{index}_x"] = float(index + 1)
        row[f"wp_{index}_y"] = float(index + 1) * 0.1
    return row


def write_dataset(root: Path, *, row: dict | None = None) -> Path:
    for directory in ("images", "lidar", "lidar_valid"):
        (root / directory).mkdir(parents=True, exist_ok=True)
    Image.new("RGB", (12, 8), color=(64, 128, 192)).save(
        root / "images/run_a_000000.png"
    )
    np.save(
        root / "lidar/run_a_000000.npy",
        np.asarray([0.0, 5.0, 25.0, 20.0], dtype=np.float32),
    )
    np.save(
        root / "lidar_valid/run_a_000000.npy",
        np.asarray([1, 1, 0, 1], dtype=np.uint8),
    )
    metadata = {
        "format_version": 2,
        "lidar_geometry": {
            "saved_points": 4,
            "source_points": 4,
            "angle_min_rad": -0.75,
            "angle_increment_rad": 0.5,
            "range_min_m": 0.0,
            "range_max_m": 25.0,
            "resampling": "none_native_beam_order",
        },
    }
    (root / "metadata.yaml").write_text(
        yaml.safe_dump(metadata, sort_keys=False), encoding="utf-8"
    )
    index_path = root / "train_index.csv"
    pd.DataFrame([row or sample_row()]).to_csv(index_path, index=False)
    return index_path


def test_v2_loader_returns_only_static_model_inputs_and_targets(tmp_path: Path) -> None:
    index_path = write_dataset(tmp_path)
    dataset = DrivingDatasetV2(index_path, {"data": data_config()}, training=False)

    sample = dataset[0]
    assert set(sample) == {"image", "lidar", "ego", "waypoints", "target_speed"}
    assert sample["image"].shape == (3, 4, 6)
    assert sample["lidar"].shape == (2, 4)
    torch.testing.assert_close(
        sample["lidar"],
        torch.tensor([[0.0, 0.2, 1.0, 0.8], [1.0, 1.0, 0.0, 1.0]]),
    )
    torch.testing.assert_close(sample["ego"], torch.tensor([0.5]))
    assert sample["waypoints"].shape == (6, 2)
    torch.testing.assert_close(sample["target_speed"], torch.tensor([4.5]))
    assert "teacher_command_steering_rad" in dataset.frame.columns
    assert "actual_steering_rad" in dataset.frame.columns


def test_full_lidar_dropout_is_training_only_and_uses_invalid_contract(
    tmp_path: Path,
) -> None:
    index_path = write_dataset(tmp_path)
    config = data_config()
    config["augmentation"]["enabled"] = True
    config["augmentation"]["lidar"].update(
        {
            "range_noise_sigma_min_m": 0.0,
            "range_noise_sigma_max_m": 0.0,
            "beam_dropout_max_fraction": 0.0,
            "sector_dropout_probability": 0.0,
            "full_dropout_probability": 1.0,
        }
    )

    training_sample = DrivingDatasetV2(
        index_path, {"data": config}, training=True
    )[0]
    evaluation_sample = DrivingDatasetV2(
        index_path, {"data": config}, training=False
    )[0]

    torch.testing.assert_close(
        training_sample["lidar"],
        torch.tensor([[1.0, 1.0, 1.0, 1.0], [0.0, 0.0, 0.0, 0.0]]),
    )
    torch.testing.assert_close(
        evaluation_sample["lidar"],
        torch.tensor([[0.0, 0.2, 1.0, 0.8], [1.0, 1.0, 0.0, 1.0]]),
    )


def test_full_camera_dropout_is_training_only_and_uses_normalized_zero(
    tmp_path: Path,
) -> None:
    index_path = write_dataset(tmp_path)
    config = data_config()
    config["augmentation"]["enabled"] = True
    config["augmentation"]["camera"]["full_dropout_probability"] = 1.0

    training_sample = DrivingDatasetV2(
        index_path, {"data": config}, training=True
    )[0]
    evaluation_sample = DrivingDatasetV2(
        index_path, {"data": config}, training=False
    )[0]

    torch.testing.assert_close(
        training_sample["image"], torch.zeros_like(training_sample["image"])
    )
    assert int(torch.count_nonzero(evaluation_sample["image"])) > 0


@pytest.mark.parametrize(
    ("draw", "camera_dropped", "lidar_dropped"),
    [
        (0.10, True, False),
        (0.30, False, True),
        (0.90, False, False),
    ],
)
def test_mutually_exclusive_dropout_selects_exactly_one_or_neither(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    draw: float,
    camera_dropped: bool,
    lidar_dropped: bool,
) -> None:
    index_path = write_dataset(tmp_path)
    config = data_config()
    config["augmentation"]["enabled"] = True
    config["augmentation"]["full_dropout_policy"] = "mutually_exclusive"
    config["augmentation"]["camera"]["full_dropout_probability"] = 0.2
    config["augmentation"]["lidar"].update(
        {
            "range_noise_sigma_min_m": 0.0,
            "range_noise_sigma_max_m": 0.0,
            "beam_dropout_max_fraction": 0.0,
            "sector_dropout_probability": 0.0,
            "full_dropout_probability": 0.2,
        }
    )
    monkeypatch.setattr(
        torch,
        "rand",
        lambda *args, **kwargs: torch.tensor(draw, dtype=torch.float32),
    )

    sample = DrivingDatasetV2(index_path, {"data": config}, training=True)[0]
    image_is_zero = int(torch.count_nonzero(sample["image"])) == 0
    lidar_is_invalid = bool(
        torch.equal(
            sample["lidar"],
            torch.tensor([[1.0, 1.0, 1.0, 1.0], [0.0, 0.0, 0.0, 0.0]]),
        )
    )
    assert image_is_zero is camera_dropped
    assert lidar_is_invalid is lidar_dropped
    assert not (image_is_zero and lidar_is_invalid)


def test_mutually_exclusive_dropout_is_disabled_for_evaluation(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    index_path = write_dataset(tmp_path)
    config = data_config()
    config["augmentation"]["enabled"] = True
    config["augmentation"]["full_dropout_policy"] = "mutually_exclusive"
    config["augmentation"]["camera"]["full_dropout_probability"] = 0.2
    config["augmentation"]["lidar"]["full_dropout_probability"] = 0.2
    monkeypatch.setattr(
        torch,
        "rand",
        lambda *args, **kwargs: torch.tensor(0.1, dtype=torch.float32),
    )

    sample = DrivingDatasetV2(index_path, {"data": config}, training=False)[0]
    assert int(torch.count_nonzero(sample["image"])) > 0
    torch.testing.assert_close(
        sample["lidar"],
        torch.tensor([[0.0, 0.2, 1.0, 0.8], [1.0, 1.0, 0.0, 1.0]]),
    )


def test_missing_policy_preserves_independent_both_dropout_contract(
    tmp_path: Path,
) -> None:
    index_path = write_dataset(tmp_path)
    config = data_config()
    config["augmentation"]["enabled"] = True
    config["augmentation"]["camera"]["full_dropout_probability"] = 1.0
    config["augmentation"]["lidar"]["full_dropout_probability"] = 1.0

    sample = DrivingDatasetV2(index_path, {"data": config}, training=True)[0]
    assert int(torch.count_nonzero(sample["image"])) == 0
    torch.testing.assert_close(
        sample["lidar"],
        torch.tensor([[1.0, 1.0, 1.0, 1.0], [0.0, 0.0, 0.0, 0.0]]),
    )


def test_v2_loader_does_not_zero_fill_missing_actual_state(tmp_path: Path) -> None:
    invalid = sample_row()
    invalid["actual_steering_rad"] = 0.0
    index_path = write_dataset(tmp_path, row=invalid)

    with pytest.raises(ValueError, match="must be NaN"):
        DrivingDatasetV2(index_path, {"data": data_config()})


def test_v2_loader_rejects_geometry_drift(tmp_path: Path) -> None:
    index_path = write_dataset(tmp_path)
    config = data_config()
    config["lidar_angle_increment_rad"] = 0.4

    with pytest.raises(ValueError, match="angle_increment_rad"):
        DrivingDatasetV2(index_path, {"data": config})


def test_v2_data_config_requires_complete_explicit_augmentation() -> None:
    config = data_config()
    validate_v2_data_config(config)
    del config["augmentation"]["lidar"]["sector_dropout_max_degrees"]
    with pytest.raises(ConfigValidationError, match="Missing data.augmentation.lidar"):
        validate_v2_data_config(config)


def test_full_config_accepts_v2_only_with_explicit_static_model_contract() -> None:
    config = copy.deepcopy(
        load_v1_config(ROOT / "configs/diagnostics/v1_training_stack_smoke.yaml")
    )
    config["data"] = data_config(lidar_points=750)
    config["model"]["camera"]["token_h"] = 6
    config["model"]["camera"]["token_w"] = 10
    config["model"]["lidar"]["use_valid_mask"] = True
    config["model"]["heads"]["stop"] = False
    config["model"]["heads"]["behavior_mode"] = False
    config["model"]["heads"]["direct_control_aux"] = False
    config["training"]["sampler"] = {
        "type": "capped_inverse_frequency_curvature_recovery",
        "straight_threshold_per_m": 0.03,
        "sharp_threshold_per_m": 0.12,
        "max_weight": 4.0,
        "recovery_weight": 4.0,
    }
    validate_v1_config(config)

    config["model"]["heads"]["direct_control_aux"] = True
    with pytest.raises(ConfigValidationError, match="Dataset v2 static requires"):
        validate_v1_config(config)
