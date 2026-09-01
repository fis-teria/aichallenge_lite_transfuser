#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
from PIL import Image


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--samples", type=int, default=64)
    parser.add_argument("--lidar-points", type=int, default=1080)
    parser.add_argument("--waypoints", type=int, default=6)
    args = parser.parse_args()

    root = Path(args.output)
    image_dir = root / "images"
    lidar_dir = root / "lidar"
    image_dir.mkdir(parents=True, exist_ok=True)
    lidar_dir.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(42)
    rows = []

    for index in range(args.samples):
        image_rel = Path("images") / f"{index:06d}.jpg"
        lidar_rel = Path("lidar") / f"{index:06d}.npy"
        image = (rng.random((180, 320, 3)) * 255).astype(np.uint8)
        Image.fromarray(image).save(root / image_rel, quality=90)
        lidar = rng.uniform(1.0, 30.0, size=(args.lidar_points,)).astype(np.float32)
        np.save(root / lidar_rel, lidar)

        row = {
            "sample_id": f"demo_{index:06d}",
            "run_id": "demo_run",
            "scenario_id": "synthetic",
            "timestamp_ns": 1_000_000_000 + index * 100_000_000,
            "image_path": str(image_rel),
            "lidar_path": str(lidar_rel),
            "velocity_mps": float(rng.uniform(0.0, 5.0)),
            "steering_rad": float(rng.uniform(-0.2, 0.2)),
            "heading_rate_rps": float(rng.uniform(-0.1, 0.1)),
            "gear": 1,
            "target_speed_mps": float(rng.uniform(0.0, 5.0)),
            "stop_flag": int(index % 16 == 0),
            "behavior_mode": int(index % 6),
            "direct_steering_rad": float(rng.uniform(-0.2, 0.2)),
            "direct_acceleration_mps2": float(rng.uniform(-1.0, 1.0)),
        }
        for wp in range(args.waypoints):
            row[f"wp_{wp}_x"] = float(wp + 1)
            row[f"wp_{wp}_y"] = float(0.1 * np.sin(index / 10.0))
        rows.append(row)

    pd.DataFrame(rows).to_csv(root / "index.csv", index=False)
    print(root / "index.csv")


if __name__ == "__main__":
    main()
