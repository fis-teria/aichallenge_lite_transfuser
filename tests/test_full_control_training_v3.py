from dataclasses import replace
from pathlib import Path

import pytest
import torch

from aic_transfuser_lite.contracts.model_batch_v3 import ModelBatchV3, TrainingTargetsV3
from aic_transfuser_lite.contracts.model_output_v3 import ModelOutputV3
from aic_transfuser_lite.models.full_control_lite_v3 import FullControlLiteV3
from aic_transfuser_lite.models.heads.control_sequence import select_current_control_targets
from aic_transfuser_lite.training.checkpoint_v3 import ExperimentIdentityV3
from aic_transfuser_lite.training.losses_v3 import (
    LossWeightsV3, compute_losses_v3, enforce_trajectory_regression_gate,
)
from aic_transfuser_lite.training.train_v3 import (
    LaunchReadinessGateConfigV3,
    TrainerV3,
    evaluate_trajectory_speed_v3,
    is_better_trajectory_checkpoint_v3,
)


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
        control_sequence=torch.zeros(batch, 10, 3),
        control_sequence_mask=torch.ones(batch, 10, 3, dtype=torch.bool),
        control_sequence_provenance=tuple(("nominal",) * 10 for _ in range(batch)),
        control_sequence_time_sec=torch.arange(10).repeat(batch, 1).float() * 0.1,
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
            "trajectory", "speed_profile", "current_control", "control_sequence",
            "behavior", "behavior_side"
        }),
    )


def _model() -> FullControlLiteV3:
    return FullControlLiteV3(
        image_height=32, image_width=32, lidar_points=16, ego_dim=4,
        hidden_dim=16, camera_tokens_hw=(1, 1), lidar_tokens=2,
        fusion_depth=1, fusion_heads=4, control_head_enabled=True,
        control_sequence_head_enabled=True, control_sequence_steps=10,
        behavior_head_enabled=True,
    )


def test_full_control_outputs_current_steering_speed_acceleration() -> None:
    model = _model().eval()
    with torch.no_grad():
        output = model(_full_batch())
    assert output.current_control is not None
    assert output.current_control.shape == (2, 1, 3)
    assert output.control_sequence is not None
    assert output.control_sequence.shape == (2, 1, 10, 3)
    steering_delta = output.control_sequence[:, :, 1:, 0] - output.control_sequence[:, :, :-1, 0]
    acceleration_delta = output.control_sequence[:, :, 1:, 2] - output.control_sequence[:, :, :-1, 2]
    assert (steering_delta.abs() <= 0.080001).all()
    assert (acceleration_delta <= 0.400001).all()
    assert (acceleration_delta >= -0.800001).all()
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
    sequence_report = compute_losses_v3(
        output, batch.targets, LossWeightsV3(control_sequence=0.4)
    )
    assert "control_sequence" in sequence_report.raw
    missing = output.__class__(output.trajectory_xy, output.trajectory_speed_mps, output.candidate_logits)
    with pytest.raises(ValueError, match="head output is absent"):
        compute_losses_v3(missing, batch.targets, LossWeightsV3(current_control=0.2))
    with pytest.raises(ValueError, match="control_sequence"):
        compute_losses_v3(missing, batch.targets, LossWeightsV3(control_sequence=0.4))


def test_trajectory_regression_gate() -> None:
    enforce_trajectory_regression_gate(candidate_ade_m=1.01, baseline_ade_m=1.0, max_relative_regression=0.02)
    with pytest.raises(RuntimeError, match="gate failed"):
        enforce_trajectory_regression_gate(candidate_ade_m=1.03, baseline_ade_m=1.0, max_relative_regression=0.02)


def test_plan_consistency_loss_matches_trajectory_geometric_speed() -> None:
    batch = _full_batch()
    assert batch.targets is not None
    x = torch.arange(1, 16, dtype=torch.float32) * 0.1
    trajectory = torch.stack((x, torch.zeros_like(x)), dim=-1)
    trajectory = trajectory.unsqueeze(0).unsqueeze(0).repeat(2, 1, 1, 1)
    output = ModelOutputV3(
        trajectory_xy=trajectory,
        trajectory_speed_mps=torch.ones(2, 1, 15),
        candidate_logits=torch.zeros(2, 1),
    )
    report = compute_losses_v3(
        output,
        batch.targets,
        LossWeightsV3(plan_consistency=1.0, plan_step_sec=0.1),
    )
    assert report.raw["plan_consistency"].item() == pytest.approx(0.0, abs=1e-6)

    inconsistent = ModelOutputV3(
        trajectory_xy=trajectory,
        trajectory_speed_mps=torch.zeros(2, 1, 15),
        candidate_logits=torch.zeros(2, 1),
    )
    bad = compute_losses_v3(
        inconsistent,
        batch.targets,
        LossWeightsV3(plan_consistency=1.0, plan_step_sec=0.1),
    )
    assert bad.raw["plan_consistency"].item() == pytest.approx(0.5, abs=1e-6)


def test_plan_consistency_rejects_invalid_step() -> None:
    with pytest.raises(ValueError, match="plan_step_sec"):
        LossWeightsV3(plan_consistency=1.0, plan_step_sec=0.0).validate()


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


def test_gradient_accumulation_advances_micro_batches_per_optimizer_step() -> None:
    model = _model()
    trainer = TrainerV3(
        model=model,
        batches=[_full_batch(), _full_batch()],
        optimizer=torch.optim.Adam(model.parameters(), lr=1e-4),
        identity=ExperimentIdentityV3("dataset", "split", "view", "contract", 1),
        loss_weights=LossWeightsV3(1.0, 0.5, 0.2, 0.2, 0.1),
        gradient_accumulation_steps=2,
    )
    trainer.train_steps(1)
    assert trainer.global_step == 1
    assert trainer.sampler.offset == 2
    assert len(trainer.logs) == 1

    with pytest.raises(ValueError, match="gradient_accumulation_steps"):
        TrainerV3(
            model=model,
            batches=[_full_batch()],
            optimizer=torch.optim.Adam(model.parameters(), lr=1e-4),
            identity=trainer.identity,
            gradient_accumulation_steps=0,
        )
    trainer.train_steps(1, micro_batches_per_optimizer_step=1)
    assert trainer.sampler.epoch == 1
    assert trainer.sampler.offset == 1
    with pytest.raises(ValueError, match="within gradient accumulation"):
        trainer.train_steps(1, micro_batches_per_optimizer_step=3)


class _FixedValidationModel(torch.nn.Module):
    def __init__(self, *, xy_offset: tuple[float, float], speed_offset: float) -> None:
        super().__init__()
        self.anchor = torch.nn.Parameter(torch.zeros(()))
        self.xy_offset = xy_offset
        self.speed_offset = speed_offset

    def forward(self, batch: ModelBatchV3) -> ModelOutputV3:
        assert batch.targets is not None
        xy = batch.targets.trajectory_xy_m + batch.targets.trajectory_xy_m.new_tensor(
            self.xy_offset
        )
        speed = batch.targets.speed_mps + self.speed_offset
        return ModelOutputV3(
            trajectory_xy=xy.unsqueeze(1),
            trajectory_speed_mps=speed.unsqueeze(1),
            candidate_logits=torch.zeros(
                batch.batch_size, 1, device=batch.image.device
            ),
        )


def test_validation_metrics_use_si_units_and_restore_train_state() -> None:
    model = _FixedValidationModel(xy_offset=(3.0, 4.0), speed_offset=0.25)
    model.train()
    metrics = evaluate_trajectory_speed_v3(model, [_full_batch()])
    assert metrics["trajectory_ade_m"] == pytest.approx(5.0)
    assert metrics["speed_profile_mae_mps"] == pytest.approx(0.25)
    assert metrics["trajectory_valid_waypoints"] == 30
    assert metrics["speed_valid_waypoints"] == 30
    assert model.training is True


def test_launch_readiness_gate_is_separate_from_average_ade() -> None:
    source = _full_batch()
    stopped = replace(source, ego=torch.zeros_like(source.ego))
    gate = LaunchReadinessGateConfigV3(
        minimum_samples=2,
        minimum_ready_fraction=1.0,
        minimum_forward_progress_m=0.1,
    )
    ready = evaluate_trajectory_speed_v3(
        _FixedValidationModel(xy_offset=(0.2, 0.0), speed_offset=0.0),
        [stopped],
        launch_gate=gate,
    )
    blocked = evaluate_trajectory_speed_v3(
        _FixedValidationModel(xy_offset=(0.05, 0.0), speed_offset=0.0),
        [stopped],
        launch_gate=gate,
    )

    assert ready["launch_sample_count"] == 2
    assert ready["launch_path_ready_fraction"] == pytest.approx(1.0)
    assert ready["launch_gate_pass"] is True
    assert blocked["trajectory_ade_m"] < ready["trajectory_ade_m"]
    assert blocked["launch_gate_pass"] is False


def test_validation_and_checkpoint_order_fail_closed_on_invalid_input() -> None:
    model = _FixedValidationModel(xy_offset=(0.0, 0.0), speed_offset=0.0)
    with pytest.raises(ValueError, match="missing targets"):
        evaluate_trajectory_speed_v3(model, [replace(_full_batch(), targets=None)])
    assert is_better_trajectory_checkpoint_v3(
        {"trajectory_ade_m": 1.0, "speed_profile_mae_mps": 0.2}, None
    )
    assert is_better_trajectory_checkpoint_v3(
        {"trajectory_ade_m": 1.0, "speed_profile_mae_mps": 0.1},
        {"trajectory_ade_m": 1.0, "speed_profile_mae_mps": 0.2},
    )
    with pytest.raises(ValueError, match="finite and non-negative"):
        is_better_trajectory_checkpoint_v3(
            {"trajectory_ade_m": float("nan"), "speed_profile_mae_mps": 0.1},
            None,
        )
