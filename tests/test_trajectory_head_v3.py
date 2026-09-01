import pytest
import torch

from aic_transfuser_lite.models.heads.speed_profile import SpeedProfileHead
from aic_transfuser_lite.models.heads.trajectory import TrajectoryHead


def test_dense_heads_return_finite_si_outputs() -> None:
    feature = torch.randn(3, 16)
    trajectory, logits = TrajectoryHead(16)(feature)
    speed = SpeedProfileHead(16)(feature)
    assert trajectory.shape == (3, 1, 15, 2)
    assert logits.shape == (3, 1)
    assert speed.shape == (3, 1, 15)
    assert torch.isfinite(trajectory).all()
    assert torch.isfinite(speed).all()
    assert (speed >= 0.0).all()


@pytest.mark.parametrize("head", [TrajectoryHead, SpeedProfileHead])
def test_dense_heads_reject_non_matrix_features(head: type[torch.nn.Module]) -> None:
    with pytest.raises(ValueError, match=r"\[B,D\]"):
        head(8)(torch.zeros(8))
