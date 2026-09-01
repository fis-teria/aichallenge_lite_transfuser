from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import numpy as np
from PIL import Image
import torch

from aic_transfuser_lite.config import load_v1_config
from aic_transfuser_lite.data.dataset_v2 import DrivingDatasetV2
from aic_transfuser_lite.data.dataset_view_v3 import (
    V1CompatibilityViewBuilder,
    V1CompatibilityViewConfig,
)
from aic_transfuser_lite.models.factory import build_model
from aic_transfuser_lite.models.transfuser_lite_v1_adapter import (
    TransFuserLiteV1Adapter,
)
from test_dataset_v2 import data_config, write_dataset
from test_dataset_v3_converter import _convert


ROOT = Path(__file__).parents[1]


def test_v3_view_matches_current_dataset_v2_tensors_targets_and_identity(tmp_path: Path) -> None:
    v2_index = write_dataset(tmp_path / "v2")
    v2 = DrivingDatasetV2(v2_index, {"data": data_config()}, training=False)[0]
    prepared = _convert().samples[0]
    future = prepared.sample.future_state
    assert future is not None
    indices = [4, 9, 14, 19, 24, 29]
    x = future.x_m.copy()
    y = future.y_m.copy()
    speed = future.longitudinal_speed_mps.copy()
    for point, index in enumerate(indices):
        x[index] = point + 1.0
        y[index] = (point + 1.0) * 0.1
    speed[4] = 4.5
    canonical = replace(
        prepared.sample,
        future_state=replace(future, x_m=x, y_m=y, longitudinal_speed_mps=speed),
        ego_state=replace(
            prepared.sample.ego_state,
            longitudinal_speed_mps=replace(
                prepared.sample.ego_state.longitudinal_speed_mps, value=5.0
            ),
        ),
    )
    config = V1CompatibilityViewConfig(
        view_id="v1_compat",
        image_height=4,
        image_width=6,
        lidar_points=4,
        lidar_min_range_m=0.0,
        lidar_max_range_m=25.0,
        ego_speed_scale_mps=10.0,
        waypoint_times_sec=(0.5, 1.0, 1.5, 2.0, 2.5, 3.0),
        target_speed_offset_sec=0.5,
        require_all_targets_valid=True,
    )
    image = np.asarray(Image.new("RGB", (12, 8), color=(64, 128, 192)))
    record = V1CompatibilityViewBuilder(config).build_record(
        canonical,
        image_rgb=image,
        lidar_ranges_m=np.array([0.0, 5.0, 25.0, 20.0], dtype=np.float32),
        lidar_valid=np.array([1, 1, 0, 1], dtype=np.uint8),
        split="train",
    )
    assert record.sample_id == canonical.sample_id and record.split == "train"
    for key in ("image", "lidar", "ego", "waypoints", "target_speed"):
        torch.testing.assert_close(record.tensors[key], v2[key])


def test_view_shapes_are_exact_v1_contract() -> None:
    prepared = _convert().samples[0]
    config = V1CompatibilityViewConfig(
        "v1_compat", 180, 320, 4, 0.1, 10.0, 10.0,
        (0.5, 1.0, 1.5, 2.0, 2.5, 3.0), 0.5, True
    )
    record = V1CompatibilityViewBuilder(config).build_record(
        prepared.sample,
        image_rgb=prepared.image_rgb,
        lidar_ranges_m=prepared.lidar_ranges_m,
        lidar_valid=prepared.lidar_valid,
        split="validation",
    )
    assert record.tensors["image"].shape == (3, 180, 320)
    assert record.tensors["lidar"].shape == (2, 4)
    assert record.tensors["ego"].shape == (1,)
    assert record.tensors["waypoints"].shape == (6, 2)
    assert record.tensors["target_speed"].shape == (1,)


def test_v1_adapter_strict_forward_and_state_dict_keys_remain_unchanged() -> None:
    config = load_v1_config(ROOT / "configs/transfuser_lite_v1_static.yaml")
    config["model"]["camera"]["pretrained"] = False
    model = build_model(config)
    keys_before = tuple(model.state_dict())
    adapter = TransFuserLiteV1Adapter(model)
    batch = {
        "image": torch.zeros(1, 2, 3, 180, 320),
        "lidar": torch.ones(1, 2, 2, 750),
        "ego": torch.zeros(1, 3, 5),
    }
    with torch.no_grad():
        output = adapter(batch)
    assert set(output) == {"waypoints", "target_speed"}
    assert output["waypoints"].shape == (1, 6, 2)
    assert output["target_speed"].shape == (1, 1)
    assert tuple(model.state_dict()) == keys_before


def test_view_rejects_invalid_target_or_lidar_shape() -> None:
    prepared = _convert().samples[-1]
    config = V1CompatibilityViewConfig(
        "v1_compat", 4, 6, 4, 0.1, 10.0, 10.0,
        (0.5, 1.0, 1.5, 2.0, 2.5, 3.0), 0.5, True
    )
    builder = V1CompatibilityViewBuilder(config)
    try:
        builder.build_record(
            prepared.sample,
            image_rgb=prepared.image_rgb,
            lidar_ranges_m=np.ones(3, dtype=np.float32),
            lidar_valid=np.ones(3, dtype=np.uint8),
            split="test",
        )
    except ValueError as error:
        assert "LiDAR" in str(error) or "invalid future" in str(error)
    else:
        raise AssertionError("invalid V1 compatibility sample was accepted")
