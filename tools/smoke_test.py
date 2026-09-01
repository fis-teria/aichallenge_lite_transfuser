#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import torch

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src"))

from aic_transfuser_lite.config import load_config
from aic_transfuser_lite.models.factory import build_model


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    args = parser.parse_args()
    config = load_config(args.config)
    model = build_model(config)
    model.eval()
    data = config["data"]
    batch = 2
    image = torch.randn(batch, 3, int(data["image_height"]), int(data["image_width"]))
    lidar = torch.rand(batch, int(data["lidar_points"]))
    ego = torch.randn(batch, int(data["ego_dim"]))
    with torch.inference_mode():
        if config["model"]["name"] == "lidar_only":
            output = model(lidar, ego)
        else:
            output = model(image, lidar, ego)
    for name, tensor in output.items():
        if not torch.isfinite(tensor).all():
            raise RuntimeError(f"Non-finite output: {name}")
        print(f"{name}: {tuple(tensor.shape)}")
    parameters = sum(p.numel() for p in model.parameters())
    print(f"parameters: {parameters:,}")


if __name__ == "__main__":
    main()
