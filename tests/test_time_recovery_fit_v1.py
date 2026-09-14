from dataclasses import replace

import pytest
import torch

from aic_transfuser_lite.evaluation.time_recovery_fit_v1 import predict_recovery_fit
from test_time_batched_evaluation_v1 import _manifest, _sample, _Model


class LabelBlindModel(_Model):
    def forward(self, batch):
        assert batch.targets is None
        assert not torch.is_grad_enabled() and not self.training
        return super().forward(batch)


def context(split="train"):
    manifest = _manifest()
    run = next(r["run_id"] for r in manifest["runs"] if r["split"] == split)
    sample = _sample(run)
    return sample, dict(run_ids=[run], anchor_ids=[sample.anchor_id], split_manifest=manifest,
                        split=split, batch_size=1)


@pytest.mark.parametrize("split", ["train", "validation"])
def test_fit_uses_real_split_without_labels_gradients_or_parameter_changes(split):
    sample, args = context(split)
    model = LabelBlindModel()
    before = {k: v.clone() for k, v in model.state_dict().items()}
    result = predict_recovery_fit(model, [sample], **args)
    assert result.shape == (1, 30, 2) and torch.all(result == .5)
    assert all(torch.equal(v, model.state_dict()[k]) for k, v in before.items())
    assert all(p.grad is None for p in model.parameters())


def test_fit_keeps_order_and_counts_each_unique_anchor_once():
    sample, args = context()
    samples = [replace(sample, anchor_id=str(i)) for i in range(3)]
    args.update(anchor_ids=[s.anchor_id for s in samples], run_ids=[sample.run]*3, batch_size=2)
    assert predict_recovery_fit(LabelBlindModel(), samples, **args).shape == (3, 30, 2)
    with pytest.raises(ValueError, match="metadata/order"):
        predict_recovery_fit(LabelBlindModel(), samples[::-1], **args)
    args["anchor_ids"][1] = args["anchor_ids"][0]
    with pytest.raises(ValueError, match="unique"):
        predict_recovery_fit(LabelBlindModel(), samples, **args)


def test_train_cannot_be_called_validation_and_test_is_sealed():
    sample, args = context()
    with pytest.raises(ValueError, match="split boundary"):
        predict_recovery_fit(_Model(), [sample], **(args | {"split": "validation"}))
    with pytest.raises(ValueError, match="test is sealed"):
        predict_recovery_fit(_Model(), [sample], **(args | {"split": "test"}))


@pytest.mark.parametrize("kind", ["input", "teacher", "partial"])
def test_missing_support_fails_instead_of_dropping_difficult_anchors(kind):
    sample, args = context()
    if kind == "input":
        sample = replace(sample, inputs=None, input_invalid_reason="SENSOR_MISSING")
    elif kind == "teacher":
        sample = replace(sample, teacher=None)
    else:
        sample.teacher.xy_mask[-1] = False
    with pytest.raises(ValueError, match="input missing|teacher incomplete"):
        predict_recovery_fit(_Model(), [sample], **args)


@pytest.mark.parametrize("kind", ["nan", "shape"])
def test_bad_output_fails_instead_of_improving_metric_by_exclusion(kind):
    class BadModel(_Model):
        def forward(self, batch):
            output = super().forward(batch)
            return output*float("nan") if kind == "nan" else output[:, :29]
    sample, args = context()
    with pytest.raises(ValueError, match="prediction must be"):
        predict_recovery_fit(BadModel(), [sample], **args)
