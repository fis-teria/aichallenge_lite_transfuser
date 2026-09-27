"""Common IL data boundaries, real optimizer masking, and encoder equivalence."""
from __future__ import annotations

from dataclasses import fields, replace
from pathlib import Path

import numpy as np
import pytest
import torch

from aic_transfuser_lite.contracts.common_il_v1 import CommonILPrediction, CommonILTargets, common_il_contract
from aic_transfuser_lite.control.common_il_adapter_v1 import adapt_common_il_output
from aic_transfuser_lite.data.time_dataset_v1 import TimeSample
from aic_transfuser_lite.data.time_teacher_v1 import TimeTeacher
from aic_transfuser_lite.models.common_il_v1 import CommonILV1
from aic_transfuser_lite.training.common_il_v1 import (
    CommonILLossWeights, collate_common_il, common_il_loss, load_common_il_checkpoint,
    save_common_il_checkpoint, train_common_il_batch,
)
from aic_transfuser_lite.training.common_il_config_v1 import load_common_il_experiment
from aic_transfuser_lite.training.train_time_v1 import training_batch
from aic_transfuser_lite.training.time_config_v1 import build_time_model
from test_dinov3_model_integration import batch, config, dino_config  # noqa: F401

ROOT = Path(__file__).resolve().parents[1]


def samples(*, labelled_stop: bool = False) -> list[TimeSample]:
    inputs = batch()
    result = []
    for i in range(2):
        one = replace(inputs, **{f.name: getattr(inputs, f.name)[i:i+1] for f in fields(inputs)
                                 if isinstance(getattr(inputs, f.name), torch.Tensor)})
        teacher = TimeTeacher(np.arange(60, dtype=np.float32).reshape(30, 2) / 10,
            np.ones(30, bool), np.arange(30, dtype=np.float32) / 10, np.ones(30, bool),
            np.ones(30, bool), (), ("OK",))
        result.append(TimeSample(one, teacher, "run", f"run:ep:{i}", i,
                                 environment_stop_intent=bool(i) if labelled_stop else None))
    return result


def test_collation_selects_declared_six_times_without_future_or_stop_inference():
    source = samples()
    source[0] = replace(source[0], observed_stationary=True, stop_reason="HORIZON_END")
    inputs, targets = collate_common_il(source, torch.device("cpu"))
    assert inputs.targets is None
    assert common_il_contract()["waypoint_time_sec"] == [0.5, 1., 1.5, 2., 2.5, 3.]
    np.testing.assert_array_equal(targets.waypoints_m[0].numpy(), source[0].teacher.xy_m[[4, 9, 14, 19, 24, 29]])
    assert targets.target_speed_mps[:, 0].tolist() == pytest.approx([.4, .4])
    assert not targets.stop_mask.any()
    with pytest.raises(ValueError, match="valid policy"):
        collate_common_il([replace(source[0], inputs=None)], torch.device("cpu"))
    with pytest.raises(ValueError, match="input-only"):
        CommonILV1(config())(training_batch(source, torch.device("cpu")))


def test_masked_nan_is_excluded_before_loss_and_per_anchor_weighting():
    xy = torch.zeros(2, 6, 2, requires_grad=True)
    speed = torch.zeros(2, 1, requires_grad=True)
    stop = torch.zeros(2, 1, requires_grad=True)
    target_xy = torch.ones_like(xy)
    mask = torch.zeros(2, 6, dtype=torch.bool)
    mask[0, 0] = True
    mask[1] = True
    target_xy[1] = 3
    target_xy[~mask] = float("nan")
    targets = CommonILTargets(target_xy, mask, torch.tensor([[2.], [float("nan")]]),
        torch.tensor([[True], [False]]), torch.full((2, 1), float("nan")), torch.zeros(2, 1, dtype=torch.bool))
    loss, report = common_il_loss(CommonILPrediction(xy, speed, stop), targets)
    assert report["loss"] == {"waypoint": 2., "speed": 2., "stop": None}
    assert loss.item() == pytest.approx(4)
    loss.backward()
    assert torch.isfinite(xy.grad).all() and torch.equal(xy.grad[~mask], torch.zeros_like(xy.grad[~mask]))
    assert speed.grad[1].item() == 0 and stop.grad is None
    bad = replace(targets, stop_target=torch.full((2, 1), .5), stop_mask=torch.ones(2, 1, dtype=torch.bool))
    with pytest.raises(ValueError, match="binary"):
        common_il_loss(CommonILPrediction(xy, speed, stop), bad)


@pytest.mark.parametrize("field", ["waypoints_m", "target_speed_mps", "stop_logit"])
def test_nonfinite_prediction_is_rejected_even_for_an_unsupervised_head(field):
    prediction = CommonILPrediction(torch.zeros(2, 6, 2), torch.zeros(2, 1), torch.zeros(2, 1))
    bad = replace(prediction, **{field: torch.full_like(getattr(prediction, field), float("nan"))})
    with pytest.raises(ValueError, match="invalid common IL output"):
        bad.validate()


def test_missing_stop_labels_do_not_apply_adamw_decay_or_old_momentum():
    model = CommonILV1(config())
    optimizer = torch.optim.AdamW(model.parameters(), lr=1e-3, weight_decay=.1)
    first = train_common_il_batch(model, samples(labelled_stop=True), optimizer)
    assert first["updated"] and first["support"]["stop"] == 2
    before = {k: v.clone() for k, v in model.heads.stop_head.state_dict().items()}
    steps = {p: optimizer.state[p]["step"].clone() for p in model.heads.stop_head.parameters()}
    second = train_common_il_batch(model, samples(), optimizer)
    assert second["updated"] and second["loss"]["stop"] is None
    for key, value in model.heads.stop_head.state_dict().items():
        torch.testing.assert_close(value, before[key], rtol=0, atol=0)
    for parameter in model.heads.stop_head.parameters():
        assert parameter.grad is None
        assert optimizer.state[parameter]["step"] == steps[parameter]


def test_all_unsupported_skips_forward_batchnorm_and_optimizer(monkeypatch):
    model = CommonILV1(config())
    optimizer = torch.optim.AdamW(model.parameters(), lr=.001)
    source = [replace(s, teacher=None) for s in samples()]
    def forbidden(*args):
        raise AssertionError("unsupported batch entered model")
    monkeypatch.setattr(model, "forward", forbidden)
    result = train_common_il_batch(model, source, optimizer)
    assert not result["updated"] and not optimizer.state
    assert result["support"] == {"waypoint": 0, "speed": 0, "stop": 0, "stop_positive": 0}


def test_dino_common_initialization_outputs_and_frozen_gradients(dino_config):
    resnet = CommonILV1(config())
    dino = CommonILV1(config(dino_config))
    reference = dict(resnet.named_parameters())
    for name, parameter in dino.named_parameters():
        if not name.startswith("backbone.camera."):
            torch.testing.assert_close(parameter, reference[name], rtol=0, atol=0)
    inputs, targets = collate_common_il(samples(labelled_stop=True), torch.device("cpu"))
    output = dino(inputs)
    assert output.waypoints_m.shape == (2, 6, 2) and output.stop_probability.shape == (2, 1)
    loss, _ = common_il_loss(output, targets)
    loss.backward()
    assert all(p.grad is None and not p.requires_grad for p in dino.backbone.camera.backbone.parameters())
    for module in (dino.backbone.camera.projection, dino.heads.speed_head, dino.heads.stop_head, dino.heads.delta_head):
        assert any(p.grad is not None and torch.isfinite(p.grad).all() and p.grad.abs().sum() > 0 for p in module.parameters())


def test_checkpoint_roundtrip_and_legacy_family_are_distinct(tmp_path):
    model = CommonILV1(config()).eval()
    inputs = batch()
    with torch.no_grad():
        expected = model(inputs)
    path = tmp_path / "common.pt"
    save_common_il_checkpoint(path, model, metadata={"stop_status": "not_supervised"})
    loaded, metadata = load_common_il_checkpoint(path)
    assert metadata["stop_status"] == "not_supervised"
    with torch.no_grad():
        actual = loaded(inputs)
    for field in fields(expected):
        torch.testing.assert_close(getattr(actual, field.name), getattr(expected, field.name), rtol=0, atol=0)
    with pytest.raises(FileExistsError):
        save_common_il_checkpoint(path, model, metadata={})
    old = build_time_model(config()).eval()
    clone = build_time_model(config()).eval()
    clone.load_state_dict(old.state_dict(), strict=True)
    with torch.no_grad():
        torch.testing.assert_close(old(inputs), clone(inputs), rtol=0, atol=0)
    legacy = tmp_path / "legacy.pt"
    torch.save({"format": "aic_time_training_checkpoint_v1"}, legacy)
    with pytest.raises(ValueError, match="not a common"):
        load_common_il_checkpoint(legacy)


def test_config_arms_share_every_setting_except_camera_and_bound_updates(tmp_path):
    import json
    _, resnet = load_common_il_experiment(ROOT / "configs/common_il_v1/resnet18.json", workspace=ROOT)
    _, dino = load_common_il_experiment(ROOT / "configs/common_il_v1/dinov3.json", workspace=ROOT)
    assert {k: v for k, v in resnet.items() if k != "camera_encoder"} == {k: v for k, v in dino.items() if k != "camera_encoder"}
    resnet["pilot"]["max_optimizer_steps"] = 101
    invalid = tmp_path / "invalid.json"
    invalid.write_text(json.dumps(resnet), encoding="utf-8")
    with pytest.raises(ValueError, match="bounded pilot"):
        load_common_il_experiment(invalid, workspace=ROOT)


def test_adapter_preserves_grid_and_unknown_stop_uses_learned_scalar_speed():
    output = CommonILPrediction(torch.ones(1, 6, 2), torch.tensor([[4.]]), torch.tensor([[3.]]))
    kwargs = dict(observation_stamp_sec=12., speed_supervised=True, stop_supervised=False, speed_cap_mps=2.)
    plan = adapt_common_il_output(output, **kwargs)
    assert plan.frame_id == "base_link" and plan.observation_stamp_sec == 12.
    np.testing.assert_array_equal(plan.waypoint_times_sec, [.5, 1., 1.5, 2., 2.5, 3.])
    np.testing.assert_array_equal(plan.speed_profile_mps, np.full(6, 2.))
    assert plan.stop_probability is None
    with pytest.raises(ValueError, match="stop_probability is required"):
        plan.validate(require_stop_probability=True)
    stopped = adapt_common_il_output(replace(output, target_speed_mps=torch.tensor([[-.1]])), **{**kwargs, "stop_supervised": True})
    assert stopped.stop_probability == pytest.approx(torch.tensor(3.).sigmoid().item())
    assert not stopped.speed_profile_mps.any()
    with pytest.raises(ValueError, match="supervision"):
        adapt_common_il_output(output, **{**kwargs, "speed_supervised": False})
    with pytest.raises(ValueError, match="finite"):
        adapt_common_il_output(output, **{**kwargs, "observation_stamp_sec": float("nan")})


@pytest.mark.parametrize("weights", [{"waypoint": 0., "speed": 0., "stop": 0.}, {"speed": float("inf")}, {"stop": -1.}])
def test_invalid_loss_weights_are_rejected(weights):
    with pytest.raises(ValueError, match="weights"):
        CommonILLossWeights(**weights)


def test_stop_annotation_source_rule_and_anchor_membership(tmp_path):
    import importlib.util
    import json
    from types import SimpleNamespace
    spec = importlib.util.spec_from_file_location("common_il_pilot_tool", ROOT / "tools/train_common_il_pilot.py")
    tool = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tool)
    dataset = SimpleNamespace(identity={"manifest_sha256": "a" * 64}, anchor_ids=["run:ep:1"])
    assert tool.read_stop_annotations(None, dataset) == ({}, None)
    annotation = {"format": "explicit_environment_stop_v1", "cache_manifest_sha256": "a" * 64,
                  "rule_version": "manual_intent_v1", "labels": {"run:ep:1": True}}
    path = tmp_path / "stop.json"
    path.write_text(json.dumps(annotation), encoding="utf-8")
    labels, digest = tool.read_stop_annotations(path, dataset)
    assert labels == {"run:ep:1": True} and len(digest) == 64
    for change in ({"cache_manifest_sha256": "b" * 64}, {"rule_version": ""},
                   {"labels": {"test:ep:1": True}}, {"labels": {"run:ep:1": 1}}):
        path.write_text(json.dumps({**annotation, **change}), encoding="utf-8")
        with pytest.raises(ValueError, match="explicit stop annotations"):
            tool.read_stop_annotations(path, dataset)
