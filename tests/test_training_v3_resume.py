from dataclasses import replace
from pathlib import Path

import pytest
import torch

from aic_transfuser_lite.contracts.model_batch_v3 import ModelBatchV3, TrainingTargetsV3
from aic_transfuser_lite.contracts.model_output_v3 import ModelOutputV3
from aic_transfuser_lite.training.checkpoint_v3 import ExperimentIdentityV3
from aic_transfuser_lite.training.losses_v3 import LossWeightsV3, compute_losses_v3
from aic_transfuser_lite.training.train_v3 import TrainerV3


class TinyV3(torch.nn.Module):
    def __init__(self) -> None:
        super().__init__()
        self.xy = torch.nn.Linear(2, 30)
        self.speed = torch.nn.Linear(2, 15)

    def forward(self, batch: ModelBatchV3) -> ModelOutputV3:
        x = batch.ego[:, -1, :2]
        return ModelOutputV3(
            trajectory_xy=self.xy(x).view(-1, 1, 15, 2),
            trajectory_speed_mps=torch.nn.functional.softplus(self.speed(x)).view(-1, 1, 15),
            candidate_logits=torch.zeros(x.shape[0], 1),
        )


def _batch(value: float) -> ModelBatchV3:
    targets = TrainingTargetsV3(
        trajectory_xy_m=torch.full((1, 15, 2), value),
        trajectory_mask=torch.ones(1, 15, dtype=torch.bool),
        speed_mps=torch.full((1, 15), abs(value)),
        speed_mask=torch.ones(1, 15, dtype=torch.bool),
    )
    return ModelBatchV3(
        image=torch.zeros(1, 1, 3, 8, 8), image_mask=torch.ones(1, 1, dtype=torch.bool),
        lidar=torch.zeros(1, 1, 2, 8), lidar_mask=torch.ones(1, 1, dtype=torch.bool),
        ego=torch.tensor([[[value, 1.0]]]), ego_feature_mask=torch.ones(1, 1, 2, dtype=torch.bool),
        command_history=torch.zeros(1, 1, 3), command_mask=torch.ones(1, 1, dtype=torch.bool),
        sensor_dt_sec=torch.zeros(1, 1, 2), targets=targets,
    )


IDENTITY = ExperimentIdentityV3("dataset", "split", "view", "contract", 42)


def _trainer(initial: dict[str, torch.Tensor]) -> TrainerV3:
    model = TinyV3()
    model.load_state_dict(initial)
    return TrainerV3(
        model=model,
        batches=[_batch(1.0), _batch(2.0), _batch(3.0)],
        optimizer=torch.optim.Adam(model.parameters(), lr=1e-3),
        identity=IDENTITY,
    )


def test_pause_resume_is_exact(tmp_path: Path) -> None:
    torch.manual_seed(7)
    initial = TinyV3().state_dict()
    uninterrupted = _trainer(initial)
    uninterrupted.train_steps(7)

    paused = _trainer(initial)
    paused.train_steps(3)
    checkpoint = tmp_path / "last.pt"
    paused.save(checkpoint)
    resumed = _trainer(initial)
    resumed.resume(checkpoint)
    resumed.train_steps(4)

    assert resumed.global_step == uninterrupted.global_step == 7
    assert resumed.sampler.state_dict() == uninterrupted.sampler.state_dict()
    for key, value in uninterrupted.model.state_dict().items():
        torch.testing.assert_close(resumed.model.state_dict()[key], value, rtol=0, atol=0)


@pytest.mark.parametrize("field", ["dataset_hash", "split_hash", "view_hash", "contract_hash"])
def test_resume_rejects_identity_mismatch(tmp_path: Path, field: str) -> None:
    initial = TinyV3().state_dict()
    source = _trainer(initial)
    checkpoint = tmp_path / "last.pt"
    source.save(checkpoint)
    target = _trainer(initial)
    target.identity = replace(IDENTITY, **{field: "different"})
    with pytest.raises(ValueError, match=field):
        target.resume(checkpoint)


def test_raw_and_weighted_losses_are_separate() -> None:
    batch = _batch(2.0)
    report = compute_losses_v3(TinyV3()(batch), batch.targets, LossWeightsV3(2.0, 3.0))
    log = report.scalar_log()
    assert log["loss_weighted/trajectory"] == pytest.approx(2.0 * log["loss_raw/trajectory"])
    assert log["loss_weighted/speed_profile"] == pytest.approx(3.0 * log["loss_raw/speed_profile"])


def test_nan_loss_fails_immediately() -> None:
    batch = _batch(1.0)
    output = TinyV3()(batch)
    output.trajectory_xy[0, 0, 0, 0] = float("nan")
    with pytest.raises(FloatingPointError, match="non-finite"):
        compute_losses_v3(output, batch.targets, LossWeightsV3())
