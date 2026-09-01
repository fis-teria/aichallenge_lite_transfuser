from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd
from PIL import Image, ImageEnhance, ImageFilter
import torch
from torch.utils.data import Dataset
import yaml

from aic_transfuser_lite.config import validate_v2_data_config

from .image_preprocess import preprocess_image
from .normalization import (
    IMAGENET_RGB_MEAN,
    IMAGENET_RGB_STD,
    normalize_lidar_range_and_validity,
    normalize_longitudinal_speed,
)
from .schema import DATASET_FORMAT_VERSION_V2, validate_v2_columns, validate_v2_row


class DrivingDatasetV2(Dataset[dict[str, torch.Tensor]]):
    """Versioned measured-pose Dataset v2 loader for the static v1 model.

    Only inference-time model inputs and trained targets are returned. Command,
    steering, collision, and other teacher/debug columns remain available in
    ``frame`` for audits but can never enter the model batch through this loader.
    """

    def __init__(
        self,
        index_path: str | Path,
        config: dict[str, Any],
        *,
        training: bool = False,
    ) -> None:
        self.index_path = Path(index_path)
        if not self.index_path.is_file():
            raise FileNotFoundError(f"Index not found: {self.index_path}")
        self.root = self.index_path.parent
        self.metadata_path = self.root / "metadata.yaml"
        if not self.metadata_path.is_file():
            raise FileNotFoundError(f"Dataset v2 metadata not found: {self.metadata_path}")

        self.data_cfg = config["data"]
        validate_v2_data_config(self.data_cfg)
        self.training = bool(training)
        self.num_waypoints = int(self.data_cfg["num_waypoints"])
        self.image_height = int(self.data_cfg["image_height"])
        self.image_width = int(self.data_cfg["image_width"])
        self.lidar_points = int(self.data_cfg["lidar_points"])
        self.lidar_min_range_m = float(self.data_cfg["lidar_min_range_m"])
        self.lidar_max_range_m = float(self.data_cfg["lidar_max_range_m"])
        self.lidar_angle_increment_rad = float(
            self.data_cfg["lidar_angle_increment_rad"]
        )
        self.speed_scale_mps = float(self.data_cfg["ego_speed_scale_mps"])
        self.augmentation_cfg = self.data_cfg["augmentation"]
        self.full_dropout_policy = str(
            self.augmentation_cfg.get("full_dropout_policy", "independent")
        )

        metadata = yaml.safe_load(self.metadata_path.read_text(encoding="utf-8"))
        if not isinstance(metadata, dict):
            raise ValueError(f"Dataset metadata root must be a mapping: {self.metadata_path}")
        self.metadata = metadata
        self._validate_metadata()

        self.frame = pd.read_csv(self.index_path)
        validate_v2_columns(tuple(self.frame.columns), num_waypoints=self.num_waypoints)
        if self.frame.empty:
            raise ValueError(f"Dataset index is empty: {self.index_path}")
        for row in self.frame.to_dict(orient="records"):
            validate_v2_row(row, num_waypoints=self.num_waypoints)
        lidar_points = self.frame["lidar_points"].astype(int)
        if not bool((lidar_points == self.lidar_points).all()):
            values = sorted(int(value) for value in lidar_points.unique())
            raise ValueError(
                f"Dataset row LiDAR beam counts {values} do not match config {self.lidar_points}"
            )

    def _validate_metadata(self) -> None:
        if int(self.metadata.get("format_version", -1)) != DATASET_FORMAT_VERSION_V2:
            raise ValueError(
                f"Expected Dataset format version {DATASET_FORMAT_VERSION_V2} metadata"
            )
        geometry = self.metadata.get("lidar_geometry")
        if not isinstance(geometry, dict):
            raise ValueError("Dataset v2 metadata lacks lidar_geometry")
        expected = {
            "saved_points": self.lidar_points,
            "source_points": self.lidar_points,
        }
        for name, value in expected.items():
            if int(geometry.get(name, -1)) != value:
                raise ValueError(
                    f"metadata.lidar_geometry.{name}={geometry.get(name)!r} "
                    f"does not match config {value}"
                )
        numeric_contract = {
            "angle_min_rad": float(self.data_cfg["lidar_angle_min_rad"]),
            "angle_increment_rad": self.lidar_angle_increment_rad,
            "range_min_m": self.lidar_min_range_m,
            "range_max_m": self.lidar_max_range_m,
        }
        for name, expected_value in numeric_contract.items():
            actual = geometry.get(name)
            if isinstance(actual, bool) or not isinstance(actual, (int, float)):
                raise ValueError(f"metadata.lidar_geometry.{name} must be numeric")
            if not math.isclose(
                float(actual), expected_value, rel_tol=1e-7, abs_tol=1e-9
            ):
                raise ValueError(
                    f"metadata.lidar_geometry.{name}={actual!r} "
                    f"does not match config {expected_value!r}"
                )
        if geometry.get("resampling") != "none_native_beam_order":
            raise ValueError("Dataset v2 LiDAR must preserve native beam order")

    def __len__(self) -> int:
        return len(self.frame)

    def _resolve(self, value: object) -> Path:
        path = Path(str(value))
        return path if path.is_absolute() else self.root / path

    def _augment_image(self, image: Image.Image) -> Image.Image:
        cfg = self.augmentation_cfg["camera"]
        result = image.convert("RGB")
        brightness_delta = float(cfg["brightness_delta"])
        contrast_delta = float(cfg["contrast_delta"])
        if brightness_delta > 0.0:
            result = ImageEnhance.Brightness(result).enhance(
                float(np.random.uniform(1.0 - brightness_delta, 1.0 + brightness_delta))
            )
        if contrast_delta > 0.0:
            result = ImageEnhance.Contrast(result).enhance(
                float(np.random.uniform(1.0 - contrast_delta, 1.0 + contrast_delta))
            )
        gamma = float(np.random.uniform(cfg["gamma_min"], cfg["gamma_max"]))
        if gamma != 1.0:
            values = np.asarray(result, dtype=np.float32) / 255.0
            values = np.clip(np.power(values, gamma) * 255.0, 0.0, 255.0)
            result = Image.fromarray(values.astype(np.uint8), mode="RGB")
        if np.random.random() < float(cfg["blur_probability"]):
            radius = float(np.random.uniform(0.0, cfg["blur_radius_max_px"]))
            result = result.filter(ImageFilter.GaussianBlur(radius=radius))
        if np.random.random() < float(cfg["noise_probability"]):
            values = np.asarray(result, dtype=np.float32) / 255.0
            noise = np.random.normal(
                0.0,
                float(cfg["noise_std_fraction"]),
                size=values.shape,
            ).astype(np.float32)
            values = np.clip((values + noise) * 255.0, 0.0, 255.0)
            result = Image.fromarray(values.astype(np.uint8), mode="RGB")
        return result

    def _augment_lidar(
        self,
        ranges_m: np.ndarray,
        valid_mask: np.ndarray,
        *,
        force_full_dropout: bool | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        cfg = self.augmentation_cfg["lidar"]
        ranges = np.asarray(ranges_m, dtype=np.float32).copy()
        valid = np.asarray(valid_mask).astype(bool, copy=True)
        if force_full_dropout is None:
            full_dropout_probability = float(
                cfg.get("full_dropout_probability", 0.0)
            )
            force_full_dropout = bool(
                full_dropout_probability > 0.0
                and np.random.random() < full_dropout_probability
            )
        if force_full_dropout:
            ranges.fill(self.lidar_max_range_m)
            valid.fill(False)
            return ranges, valid.astype(np.uint8)

        sigma = float(
            np.random.uniform(
                cfg["range_noise_sigma_min_m"],
                cfg["range_noise_sigma_max_m"],
            )
        )
        if sigma > 0.0 and bool(valid.any()):
            ranges[valid] += np.random.normal(0.0, sigma, int(valid.sum())).astype(
                np.float32
            )

        dropout_fraction = float(
            np.random.uniform(0.0, cfg["beam_dropout_max_fraction"])
        )
        if dropout_fraction > 0.0:
            valid &= np.random.random(valid.shape) >= dropout_fraction

        if np.random.random() < float(cfg["sector_dropout_probability"]):
            width_degrees = float(
                np.random.uniform(0.0, cfg["sector_dropout_max_degrees"])
            )
            width_beams = max(
                1,
                int(
                    math.ceil(
                        math.radians(width_degrees)
                        / abs(self.lidar_angle_increment_rad)
                    )
                ),
            )
            center = int(np.random.randint(0, self.lidar_points))
            start = max(center - width_beams // 2, 0)
            stop = min(start + width_beams, self.lidar_points)
            valid[start:stop] = False

        ranges = np.clip(ranges, self.lidar_min_range_m, self.lidar_max_range_m)
        ranges[~valid] = self.lidar_max_range_m
        return ranges, valid.astype(np.uint8)

    def __getitem__(self, index: int) -> dict[str, torch.Tensor]:
        row = self.frame.iloc[index]
        image_path = self._resolve(row["image_path"])
        lidar_path = self._resolve(row["lidar_path"])
        valid_path = self._resolve(row["lidar_valid_path"])
        for name, path in (
            ("image", image_path),
            ("LiDAR", lidar_path),
            ("LiDAR validity", valid_path),
        ):
            if not path.is_file():
                raise FileNotFoundError(f"{name} asset not found: {path}")

        augmentation_enabled = self.training and bool(
            self.augmentation_cfg["enabled"]
        )
        exclusive_camera_dropout = False
        exclusive_lidar_dropout = False
        if augmentation_enabled and self.full_dropout_policy == "mutually_exclusive":
            camera_probability = float(
                self.augmentation_cfg["camera"].get(
                    "full_dropout_probability", 0.0
                )
            )
            lidar_probability = float(
                self.augmentation_cfg["lidar"].get(
                    "full_dropout_probability", 0.0
                )
            )
            draw = float(torch.rand((), dtype=torch.float32).item())
            exclusive_camera_dropout = draw < camera_probability
            exclusive_lidar_dropout = (
                not exclusive_camera_dropout
                and draw < camera_probability + lidar_probability
            )

        with Image.open(image_path) as image:
            prepared_image = image.convert("RGB")
            if augmentation_enabled:
                prepared_image = self._augment_image(prepared_image)
            image_tensor = preprocess_image(
                prepared_image,
                height=self.image_height,
                width=self.image_width,
                mean=IMAGENET_RGB_MEAN,
                std=IMAGENET_RGB_STD,
            )
            camera_full_dropout_probability = float(
                self.augmentation_cfg["camera"].get(
                    "full_dropout_probability", 0.0
                )
            )
            if self.full_dropout_policy == "mutually_exclusive":
                apply_camera_full_dropout = exclusive_camera_dropout
            else:
                apply_camera_full_dropout = bool(
                    augmentation_enabled
                    and camera_full_dropout_probability > 0.0
                    and (
                        torch.rand((), dtype=torch.float32).item()
                        < camera_full_dropout_probability
                    )
                )
            if apply_camera_full_dropout:
                image_tensor.zero_()

        ranges = np.load(lidar_path, allow_pickle=False)
        valid = np.load(valid_path, allow_pickle=False)
        if ranges.shape != (self.lidar_points,) or valid.shape != (self.lidar_points,):
            raise ValueError(
                f"Expected LiDAR range/mask shape {(self.lidar_points,)}, "
                f"got {ranges.shape} and {valid.shape}"
            )
        if augmentation_enabled:
            if self.full_dropout_policy == "mutually_exclusive":
                ranges, valid = self._augment_lidar(
                    ranges,
                    valid,
                    force_full_dropout=exclusive_lidar_dropout,
                )
            else:
                ranges, valid = self._augment_lidar(ranges, valid)
        lidar = normalize_lidar_range_and_validity(
            ranges,
            valid,
            min_range_m=self.lidar_min_range_m,
            max_range_m=self.lidar_max_range_m,
        )

        waypoints = np.asarray(
            [
                [float(row[f"wp_{point}_x"]), float(row[f"wp_{point}_y"])]
                for point in range(self.num_waypoints)
            ],
            dtype=np.float32,
        )
        ego_speed = normalize_longitudinal_speed(
            float(row["velocity_longitudinal_mps"]),
            scale_mps=self.speed_scale_mps,
        )
        return {
            "image": image_tensor.float(),
            "lidar": torch.from_numpy(lidar).float(),
            "ego": torch.tensor([ego_speed], dtype=torch.float32),
            "waypoints": torch.from_numpy(waypoints).float(),
            "target_speed": torch.tensor(
                [float(row["target_speed_mps"])], dtype=torch.float32
            ),
        }
