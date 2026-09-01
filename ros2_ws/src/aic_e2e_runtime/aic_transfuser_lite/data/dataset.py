from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd
from PIL import Image
import torch
from torch.utils.data import Dataset

from .ego_features import configured_ego_features, select_ego_features
from .image_preprocess import preprocess_image
from .lidar_preprocess import LidarPreprocessConfig, sanitize_lidar


class DrivingDataset(Dataset[dict[str, torch.Tensor]]):
    """Canonical index.csv based dataset.

    Expected waypoint columns are wp_0_x, wp_0_y, ..., wp_{N-1}_x, wp_{N-1}_y.
    """

    def __init__(self, index_path: str | Path, config: dict[str, Any]) -> None:
        self.index_path = Path(index_path)
        if not self.index_path.is_file():
            raise FileNotFoundError(f"Index not found: {self.index_path}")
        self.root = self.index_path.parent
        self.frame = pd.read_csv(self.index_path)
        self.data_cfg = config["data"]
        self.num_waypoints = int(self.data_cfg["num_waypoints"])
        self.image_height = int(self.data_cfg["image_height"])
        self.image_width = int(self.data_cfg["image_width"])
        self.lidar_points = int(self.data_cfg["lidar_points"])
        self.ego_features = configured_ego_features(self.data_cfg)
        self.ego_dim = len(self.ego_features)
        self.lidar_cfg = LidarPreprocessConfig(
            min_range_m=float(self.data_cfg.get("lidar_min_range_m", 0.05)),
            max_range_m=float(self.data_cfg.get("lidar_max_range_m", 30.0)),
        )
        self._validate_columns()

    def _validate_columns(self) -> None:
        required = {
            "image_path",
            "lidar_path",
            "velocity_mps",
            "steering_rad",
            "heading_rate_rps",
            "gear",
            "target_speed_mps",
            "stop_flag",
            "behavior_mode",
        }
        for i in range(self.num_waypoints):
            required.add(f"wp_{i}_x")
            required.add(f"wp_{i}_y")
        missing = sorted(required.difference(self.frame.columns))
        if missing:
            raise ValueError(f"Missing required index columns: {missing}")

    def __len__(self) -> int:
        return len(self.frame)

    def _resolve(self, value: str) -> Path:
        path = Path(value)
        return path if path.is_absolute() else self.root / path

    def __getitem__(self, index: int) -> dict[str, torch.Tensor]:
        row = self.frame.iloc[index]
        image_path = self._resolve(str(row["image_path"]))
        lidar_path = self._resolve(str(row["lidar_path"]))
        if not image_path.is_file():
            raise FileNotFoundError(f"Image not found: {image_path}")
        if not lidar_path.is_file():
            raise FileNotFoundError(f"LiDAR not found: {lidar_path}")

        with Image.open(image_path) as image:
            image_tensor = preprocess_image(
                image,
                height=self.image_height,
                width=self.image_width,
            )

        lidar_raw = np.load(lidar_path).astype(np.float32)
        if lidar_raw.shape != (self.lidar_points,):
            raise ValueError(
                f"Expected lidar shape {(self.lidar_points,)}, got {lidar_raw.shape} at {lidar_path}"
            )
        lidar_norm, lidar_valid = sanitize_lidar(lidar_raw, self.lidar_cfg)

        commanded_speed_mps = float(row["velocity_mps"])
        ego_values = select_ego_features(
            self.ego_features,
            {
                "speed_mps": abs(commanded_speed_mps),
                "longitudinal_speed_mps": commanded_speed_mps,
                "lateral_speed_mps": 0.0,
                "yaw_rate_rps": float(row["heading_rate_rps"]),
                "steering_rad": float(row["steering_rad"]),
                "gear": float(row["gear"]),
            },
        )

        waypoints = [
            [float(row[f"wp_{i}_x"]), float(row[f"wp_{i}_y"])]
            for i in range(self.num_waypoints)
        ]

        direct_control = [
            float(row.get("direct_steering_rad", 0.0)),
            float(row.get("direct_acceleration_mps2", 0.0)),
        ]

        return {
            "image": image_tensor.float(),
            "lidar": torch.from_numpy(lidar_norm).float(),
            "lidar_valid": torch.from_numpy(lidar_valid).float(),
            "ego": torch.tensor(ego_values, dtype=torch.float32),
            "waypoints": torch.tensor(waypoints, dtype=torch.float32),
            "target_speed": torch.tensor([float(row["target_speed_mps"])], dtype=torch.float32),
            "stop": torch.tensor([float(row["stop_flag"])], dtype=torch.float32),
            "mode": torch.tensor(int(row["behavior_mode"]), dtype=torch.long),
            "direct_control": torch.tensor(direct_control, dtype=torch.float32),
        }
