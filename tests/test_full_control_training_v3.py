from pathlib import Path

import pytest
import torch

from aic_transfuser_lite.contracts.model_batch_v3 import ModelBatchV3, TrainingTargetsV3
from aic_transfuser_lite.models.full_control_lite_v3 import FullControlLiteV3
from aic_transfuser_lite.models.heads.control_sequence import select_current_control_targets
from aic_transfuser_lite.training.checkpoint_v3 import ExperimentIdentityV3
from aic_transfuser_lite.training.losses_v3 import (
    LossWeightsV3, compute_losses_v3, enforce_trajectory_regression_gate,
)
from aic_transfuser_lite.training.train_v3 import TrainerV3


def _full_batch() -> ModelBatchV3:
    batch = 2
    nominal = torch.tensor([[0.1, 2.0, 0.3], [0.0, 0.0, 0.0]])
    final = torch.tensor([[0.2, 1.0, -0.2], [-0.2, 1.5, -0.4]])
    selected = select_current_control_targets(
        nominal_values=nominal,
        nominal_mask=torch.tensor([[True, True, True], [False, False, False]]),
        final_values=final,
        final_mask=torch.ones(batch, 3, dtype=torch.bool),
    )
    targets = TrainingTargetsV3(
        trajectory_xy_m=torch.zeros(batch, 15, 2),
        trajectory_mask=torch.ones(batch, 15, dtype=torch.bool),
        speed_mps=torch.ones(batch, 15), speed_mask=torch.ones(batch, 15, dtype=torch.bool),
        current_control=selected.values, current_control_mask=selected.mask,
        control_provenance=selected.provenance,
        behavior_class=torch.tensor([2, 4]), behavior_mask=torch.ones(batch, dtype=torch.bool),
        behavior_side=torch.tensor([2, 0]), behavior_side_mask=torch.ones(batch, dtype=torch.bool),
    )
    return ModelBatchV3(
        image=torch.randn(batch, 1, 3, 32, 32), image_mask=torch.ones(batch, 1, dtype=torch.bool),
        lidar=torch.rand(batch, 1, 2, 16), lidar_mask=torch.ones(batch, 1, dtype=torch.bool),
        ego=torch.randn(batch, 1, 4), ego_feature_mask=torch.ones(batch, 1, 4, dtype=torch.bool),
        command_history=torch.zeros(batch, 1, 3), command_mask=torch.zeros(batch, 1, dtype=torch.bool),
        sensor_dt_sec=torch.zeros(batch, 1, 2), targets=targets,
        requested_outputs=frozenset({
            "trajectory", "speed_profile", "current_control", "behavior", "behavior_side"
        }),
    )


def _model() -> FullControlLiteV3:
    return FullControlLiteV3(
        image_height=32, image_width=32, lidar_points=16, ego_dim=4,
        hidden_dim=16, camera_tokens_hw=(1, 1), lidar_tokens=2,
        fusion_depth=1, fusion_heads=4, control_head_enabled=True,
        behavior_head_enabled=True,
    )


def test_full_control_outputs_current_steering_speed_acceleration() -> None:
    model = _model().eval()
    with torch.no_grad():
        output = model(_full_batch())
    assert output.current_control is not None
    assert output.current_control.shape == (2, 1, 3)
    assert (output.current_control[..., 1] >= 0.0).all()
    assert output.behavior_logits is not None and output.behavior_logits.shape == (2, 5)
    assert output.behavior_side_logits is not None and output.behavior_side_logits.shape == (2, 3)


def test_nominal_teacher_preferred_and_final_is_explicit_fallback() -> None:
    batch = _full_batch()
    assert batch.targets is not None
    assert batch.targets.control_provenance == ("nominal", "final_fallback")
    torch.testing.assert_close(batch.targets.current_control[0], torch.tensor([0.1, 2.0, 0.3]))
    torch.testing.assert_close(batch.targets.current_control[1], torch.tensor([-0.2, 1.5, -0.4]))


def test_control_loss_requires_head_and_logs_provenance() -> None:
    batch = _full_batch()
    output = _model()(batch)
    report = compute_losses_v3(output, batch.targets, LossWeightsV3(1.0, 0.5, 0.2, 0.2, 0.1))
    assert "current_control_nominal" in report.raw
    assert "current_control_final_fallback" in report.raw
    missing = output.__class__(output.trajectory_xy, output.trajectory_speed_mps, output.candidate_logits)
    with pytest.raises(ValueError, match="head output is absent"):
        compute_losses_v3(missing, batch.targets, LossWeightsV3(current_control=0.2))


def test_trajectory_regression_gate() -> None:
    enforce_trajectory_regression_gate(candidate_ade_m=1.01, baseline_ade_m=1.0, max_relative_regression=0.02)
    with pytest.raises(RuntimeError, match="gate failed"):
        enforce_trajectory_regression_gate(candidate_ade_m=1.03, baseline_ade_m=1.0, max_relative_regression=0.02)


def test_one_epoch_smoke_and_resume(tmp_path: Path) -> None:
    model = _model()
    trainer = TrainerV3(
        model=model, batches=[_full_batch()], optimizer=torch.optim.Adam(model.parameters(), lr=1e-4),
        identity=ExperimentIdentityV3("dataset", "split", "view", "contract", 1),
        loss_weights=LossWeightsV3(1.0, 0.5, 0.2, 0.2, 0.1),
    )
    logs = trainer.train_steps(1)
    assert trainer.sampler.epoch == 0 and trainer.sampler.offset == 1
    assert "loss_raw/current_control" in logs[0]
    assert "metric/behavior_accuracy" in logs[0]
    assert "metric/behavior_side_accuracy" in logs[0]
    checkpoint = tmp_path / "full_control_last.pt"
    trainer.save(checkpoint)
    resumed_model = _model()
    resumed = TrainerV3(
        model=resumed_model, batches=[_full_batch()],
        optimizer=torch.optim.Adam(resumed_model.parameters(), lr=1e-4),
        identity=trainer.identity, loss_weights=trainer.loss_weights,
    )
    resumed.resume(checkpoint)
    assert resumed.global_step == 1
    resumed.train_steps(1)
    assert resumed.global_step == 2
