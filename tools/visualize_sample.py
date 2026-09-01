#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from PIL import Image


def resolve(root: Path, value: object) -> Path:
    path = Path(str(value))
    return path if path.is_absolute() else root / path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--index", required=True)
    parser.add_argument("--row", type=int, default=0)
    parser.add_argument("--output", required=True)
    parser.add_argument("--angle-min-rad", type=float, default=-2.35619)
    parser.add_argument("--angle-increment-rad", type=float, default=0.00436332)
    args = parser.parse_args()

    index_path = Path(args.index)
    frame = pd.read_csv(index_path)
    row = frame.iloc[args.row]
    root = index_path.parent
    image = Image.open(resolve(root, row["image_path"])).convert("RGB")
    lidar = np.load(resolve(root, row["lidar_path"]))
    angles = args.angle_min_rad + np.arange(len(lidar)) * args.angle_increment_rad
    valid = np.isfinite(lidar) & (lidar > 0.0)
    x = lidar[valid] * np.cos(angles[valid])
    y = lidar[valid] * np.sin(angles[valid])

    fig = plt.figure(figsize=(10, 4))
    ax1 = fig.add_subplot(1, 2, 1)
    ax1.imshow(image)
    ax1.set_title("Camera")
    ax1.axis("off")
    ax2 = fig.add_subplot(1, 2, 2)
    ax2.scatter(x, y, s=2)
    ax2.set_aspect("equal", adjustable="box")
    ax2.set_title("LiDAR endpoints")
    ax2.set_xlabel("x [m]")
    ax2.set_ylabel("y [m]")
    fig.tight_layout()
    fig.savefig(args.output, dpi=150)


if __name__ == "__main__":
    main()
