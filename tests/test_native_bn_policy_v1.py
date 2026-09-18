import pytest
import torch
from torch import nn

from aic_transfuser_lite.training.native_bn_policy_v1 import (
    copy_batchnorm_statistics, freeze_batchnorm_statistics,
)


def test_freeze_preserves_statistics_but_learns_affine() -> None:
    model = nn.Sequential(nn.Linear(3, 3), nn.BatchNorm1d(3))
    model.train()
    assert freeze_batchnorm_statistics(model) == ["1"]
    before = {k: v.clone() for k, v in model[1].named_buffers()}
    optimizer = torch.optim.SGD(model.parameters(), lr=0.1)
    old_bias = model[1].bias.detach().clone()
    model(torch.ones(8, 3) * 7).sum().backward()
    optimizer.step()
    assert all(torch.equal(v, before[k]) for k, v in model[1].named_buffers())
    assert not torch.equal(old_bias, model[1].bias)
    assert model[0].training and not model[1].training


def test_copy_exchanges_only_selected_statistics() -> None:
    src = nn.Sequential(nn.BatchNorm1d(3), nn.BatchNorm1d(3))
    dst = nn.Sequential(nn.BatchNorm1d(3), nn.BatchNorm1d(3))
    with torch.no_grad():
        src[0].running_mean.fill_(5)
        src[0].weight.fill_(9)
        src[1].running_mean.fill_(8)
    assert copy_batchnorm_statistics(dst, src, prefix="0") == ["0"]
    assert torch.equal(dst[0].running_mean, src[0].running_mean)
    assert torch.equal(dst[0].weight, torch.ones(3))
    assert torch.equal(dst[1].running_mean, torch.zeros(3))


def test_invalid_statistics_cannot_partially_copy() -> None:
    src = nn.Sequential(nn.BatchNorm1d(3), nn.BatchNorm1d(3))
    dst = nn.Sequential(nn.BatchNorm1d(3), nn.BatchNorm1d(3))
    src[0].running_mean.fill_(5)
    src[1].running_var.fill_(float("nan"))
    with pytest.raises(ValueError, match="invalid BatchNorm"):
        copy_batchnorm_statistics(dst, src)
    assert torch.equal(dst[0].running_mean, torch.zeros(3))


@pytest.mark.parametrize("model", [nn.Linear(3, 3), nn.BatchNorm1d(3, track_running_stats=False)])
def test_freeze_rejects_missing_statistics(model: nn.Module) -> None:
    with pytest.raises(ValueError):
        freeze_batchnorm_statistics(model)
