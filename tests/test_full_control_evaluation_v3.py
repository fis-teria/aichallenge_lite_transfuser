import pytest
import torch

from aic_transfuser_lite.contracts.model_batch_v3 import ModelBatchV3, TrainingTargetsV3
from aic_transfuser_lite.contracts.model_output_v3 import ModelOutputV3
from aic_transfuser_lite.training.evaluate_v3 import evaluate_offline_v3


class _FixedModel(torch.nn.Module):
    def __init__(self) -> None:
        super().__init__()
        self.anchor = torch.nn.Parameter(torch.tensor(0.0))

    def forward(self, batch: ModelBatchV3) -> ModelOutputV3:
        size = batch.batch_size
        return ModelOutputV3(
            trajectory_xy=torch.zeros(size, 1, 2, 2, device=self.anchor.device),
            trajectory_speed_mps=torch.ones(size, 1, 2, device=self.anchor.device) * 2.0,
            candidate_logits=torch.zeros(size, 1, device=self.anchor.device),
            current_control=torch.zeros(size, 1, 3, device=self.anchor.device),
            control_sequence=torch.zeros(size, 1, 2, 3, device=self.anchor.device),
        )


def _batch() -> ModelBatchV3:
    targets = TrainingTargetsV3(
        trajectory_xy_m=torch.tensor([[[3.0, 4.0], [0.0, 0.0]]]),
        trajectory_mask=torch.tensor([[True, False]]),
        speed_mps=torch.tensor([[1.0, 20.0]]),
        speed_mask=torch.tensor([[True, False]]),
        current_control=torch.tensor([[0.1, 2.0, -0.3]]),
        current_control_mask=torch.ones(1, 3, dtype=torch.bool),
        control_provenance=("nominal",),
        control_sequence=torch.tensor([[[0.2, 1.0, -0.5], [9.0, 9.0, 9.0]]]),
        control_sequence_mask=torch.tensor([[[True, True, True], [False, False, False]]]),
        control_sequence_provenance=(("nominal", "missing_exact_timestamp"),),
        control_sequence_time_sec=torch.tensor([[0.0, 0.1]]),
    )
    return ModelBatchV3(
        image=torch.zeros(1, 1, 3, 2, 2), image_mask=torch.ones(1, 1, dtype=torch.bool),
        lidar=torch.zeros(1, 1, 2, 2), lidar_mask=torch.ones(1, 1, dtype=torch.bool),
        ego=torch.zeros(1, 1, 4), ego_feature_mask=torch.ones(1, 1, 4, dtype=torch.bool),
        command_history=torch.zeros(1, 1, 3), command_mask=torch.ones(1, 1, dtype=torch.bool),
        sensor_dt_sec=torch.zeros(1, 1, 2), targets=targets,
        requested_outputs=frozenset({"trajectory", "speed_profile", "current_control", "control_sequence"}),
    )


def test_offline_metrics_use_masks_and_keep_si_dimensions_separate() -> None:
    model = _FixedModel().train()
    result = evaluate_offline_v3(model, [_batch()], device=torch.device("cpu"))
    assert result.trajectory_ade_m == pytest.approx(5.0)
    assert result.speed_mae_mps == pytest.approx(1.0)
    assert result.current_control_mae == pytest.approx((0.1, 2.0, 0.3))
    assert result.control_sequence_mae == pytest.approx((0.2, 1.0, 0.5))
    assert model.training


def test_offline_metrics_require_targets() -> None:
    batch = ModelBatchV3(**{**_batch().__dict__, "targets": None})
    with pytest.raises(ValueError, match="no targets"):
        evaluate_offline_v3(_FixedModel(), [batch], device=torch.device("cpu"))
