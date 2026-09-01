from pathlib import Path

import pytest
import torch

from aic_transfuser_lite.models.factory import build_model
from aic_transfuser_lite.runtime.inference_core import infer


def test_deployed_checkpoint_runs_one_inference() -> None:
    checkpoint_path = Path(__file__).parents[1] / "ckpt" / "best.pt"
    if not checkpoint_path.is_file():
        pytest.skip("deployed checkpoint is not present")
    checkpoint = torch.load(checkpoint_path, map_location="cpu")
    config = checkpoint["config"]
    model = build_model(config)
    model.load_state_dict(checkpoint["model"], strict=True)
    data = config["data"]
    output = infer(
        model,
        image=torch.zeros(
            1, 3, int(data["image_height"]), int(data["image_width"])
        ),
        lidar=torch.ones(1, int(data["lidar_points"])),
        ego=torch.zeros(1, int(data["ego_dim"])),
        model_name=str(config["model"]["name"]),
    )
    assert output["waypoints"].shape == (1, int(data["num_waypoints"]), 2)
    assert output["target_speed"].shape == (1, 1)
    assert output["stop_probability"].shape == (1, 1)
    assert all(torch.isfinite(value).all() for value in output.values())
