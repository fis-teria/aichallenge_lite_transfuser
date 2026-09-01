from __future__ import annotations

import os
from pathlib import Path

import torch

from aic_transfuser_lite.runtime.inference_core import infer_v1
from aic_transfuser_lite.runtime.model_loader_v1 import load_runtime_model_v1


EXPECTED_CHECKPOINT_SHA256 = (
    "1b82e33aa676ccc433a66781658ba9a919d88de34df6c0bc6948738e130dbb84"
)


def _checkpoint_path() -> Path:
    configured = os.environ.get("AIC_V1_CHECKPOINT_PATH")
    if configured:
        return Path(configured)
    return Path(__file__).resolve().parents[1] / "ckpt" / "transfuser_lite_v1_best_ade.pt"


def test_promoted_checkpoint_strictly_loads_and_runs_batch_one_on_cpu() -> None:
    path = _checkpoint_path()
    assert path.is_file(), f"required promoted v1 checkpoint is missing: {path}"
    loaded = load_runtime_model_v1(
        path,
        device=torch.device("cpu"),
        expected_checkpoint_sha256=EXPECTED_CHECKPOINT_SHA256,
    )
    data = loaded.config["data"]
    output = infer_v1(
        loaded.model,
        image=torch.zeros(
            1, 3, int(data["image_height"]), int(data["image_width"])
        ),
        lidar=torch.ones(1, 2, int(data["lidar_points"])),
        ego=torch.zeros(1, int(data["ego_dim"])),
        use_amp=False,
    )
    assert output["waypoints"].shape == (1, int(data["num_waypoints"]), 2)
    assert output["target_speed"].shape == (1, 1)
    assert all(torch.isfinite(value).all() for value in output.values())
