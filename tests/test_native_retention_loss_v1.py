from dataclasses import replace

import pytest
import torch

from aic_transfuser_lite.training.native_retention_loss_v1 import (
    RetainedNativeObjective, output_retention_sum,
)
from test_time_batched_evaluation_v1 import _manifest, _sample


class ZeroObjective:
    def validate_samples(self, samples):
        pass

    def __call__(self, prediction, target, support, samples):
        return prediction.sum() * 0


def test_retention_units_support_and_reference_detach():
    p = torch.full((2, 30, 2), .03, requires_grad=True)
    r = torch.zeros_like(p, requires_grad=True)
    mask = torch.zeros(2, 30, dtype=torch.bool)
    mask[0, :2] = True
    loss = output_retention_sum(p, r, mask)
    assert loss.item() == pytest.approx(.02)  # .03m minus beta/2; one supported anchor.
    loss.backward()
    assert r.grad is None
    assert torch.all(p.grad[0, :2] > 0)
    assert torch.all(p.grad[0, 2:] == 0) and torch.all(p.grad[1] == 0)


def test_unsupported_nan_reference_and_zero_support_are_safe():
    p = torch.ones(1, 30, 2, requires_grad=True)
    mask = torch.zeros(1, 30, dtype=torch.bool)
    loss = output_retention_sum(p, torch.full_like(p, float("nan")), mask)
    loss.backward()
    assert loss.item() == 0 and torch.equal(p.grad, torch.zeros_like(p))


@pytest.mark.parametrize("beta", [0., -1., float("nan")])
def test_invalid_beta_is_rejected(beta):
    with pytest.raises(ValueError):
        output_retention_sum(torch.zeros(1, 30, 2), torch.zeros(1, 30, 2),
                             torch.ones(1, 30, dtype=torch.bool), beta_m=beta)


def test_bad_shapes_and_nonfinite_supported_reference():
    with pytest.raises(ValueError):
        output_retention_sum(torch.zeros(1, 29, 2), torch.zeros(1, 29, 2), torch.ones(1, 29, dtype=torch.bool))
    with pytest.raises(ValueError):
        output_retention_sum(torch.zeros(1, 30, 2), torch.full((1, 30, 2), float("nan")), torch.ones(1, 30, dtype=torch.bool))


def test_native_predictions_receive_no_old_distillation_and_split_is_enforced():
    manifest = _manifest()
    train = next(r["run_id"] for r in manifest["runs"] if r["split"] == "train")
    old = _sample(train)
    new = replace(old, anchor_id="new-native-anchor")
    obj = RetainedNativeObjective(ZeroObjective(), ZeroObjective(), geometry_weight=0.,
          retention_weight=1., references={old.anchor_id: torch.zeros(30, 2)},
          old_anchor_runs={old.anchor_id: train}, split_manifest=manifest)
    p = torch.full((2, 30, 2), .03, requires_grad=True)
    loss = obj(p, torch.zeros_like(p), torch.ones(2, 30, dtype=torch.bool), [old, new])
    loss.backward()
    assert torch.all(p.grad[0] > 0) and torch.all(p.grad[1] == 0)
    heldout = next(r["run_id"] for r in manifest["runs"] if r["split"] == "validation")
    with pytest.raises(ValueError):
        RetainedNativeObjective(ZeroObjective(), ZeroObjective(), geometry_weight=0.,
          retention_weight=1., references={}, old_anchor_runs={old.anchor_id: heldout}, split_manifest=manifest)
    with pytest.raises(ValueError, match="identity"):
        obj.validate_samples([replace(old, run="wrong-run")])


def test_missing_reference_and_invalid_weight_are_rejected():
    manifest = _manifest()
    train = next(r["run_id"] for r in manifest["runs"] if r["split"] == "train")
    old = _sample(train)
    kwargs = dict(references={}, old_anchor_runs={old.anchor_id: train}, split_manifest=manifest)
    obj = RetainedNativeObjective(ZeroObjective(), ZeroObjective(), geometry_weight=0., retention_weight=1., **kwargs)
    with pytest.raises(ValueError, match="missing"):
        obj.validate_samples([old])
    with pytest.raises(ValueError):
        RetainedNativeObjective(ZeroObjective(), ZeroObjective(), geometry_weight=-.1, retention_weight=0., **kwargs)
