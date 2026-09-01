import pytest
import torch

from aic_transfuser_lite.contracts.model_batch_v3 import ModelBatchV3
from aic_transfuser_lite.models.full_control_lite_v3 import FullControlLiteV3


def _batch(*, temporal: int = 1) -> ModelBatchV3:
    batch = 2
    return ModelBatchV3(
        image=torch.randn(batch, temporal, 3, 64, 64),
        image_mask=torch.ones(batch, temporal, dtype=torch.bool),
        lidar=torch.rand(batch, temporal, 2, 64),
        lidar_mask=torch.ones(batch, temporal, dtype=torch.bool),
        ego=torch.randn(batch, 1, 8),
        ego_feature_mask=torch.ones(batch, 1, 8, dtype=torch.bool),
        command_history=torch.zeros(batch, 1, 3),
        command_mask=torch.ones(batch, 1, dtype=torch.bool),
        sensor_dt_sec=torch.zeros(batch, temporal, 2),
    )


def _model() -> FullControlLiteV3:
    return FullControlLiteV3(
        image_height=64,
        image_width=64,
        lidar_points=64,
        ego_dim=8,
        hidden_dim=32,
        camera_tokens_hw=(2, 2),
        lidar_tokens=4,
        fusion_depth=1,
        fusion_heads=4,
    )


def test_t1_k1_n15_model_shape_and_units() -> None:
    model = _model().eval()
    with torch.no_grad():
        output = model(_batch())
    assert output.trajectory_xy.shape == (2, 1, 15, 2)
    assert output.trajectory_speed_mps.shape == (2, 1, 15)
    assert (output.trajectory_speed_mps >= 0.0).all()


def test_history_longer_than_configured_is_rejected() -> None:
    with pytest.raises(ValueError, match="history exceeds"):
        _model()(_batch(temporal=5))


def test_v1_migration_is_explicit() -> None:
    model = _model()
    state = model.state_dict()
    report = model.migrate_v1_weights(
        {
            "camera.projection.weight": state["camera.projection.weight"].clone(),
            "camera.projection.bias": torch.zeros(99),
            "heads.waypoint.weight": torch.zeros(1),
        }
    )
    assert report.loaded == ("camera.projection.weight",)
    assert report.shape_mismatch == ("camera.projection.bias",)
    assert report.unmapped_v1 == ("heads.waypoint.weight",)
    assert "trajectory_head.delta_xy.weight" in report.new_v3
