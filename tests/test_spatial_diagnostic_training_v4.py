from __future__ import annotations

from dataclasses import replace
import io
import json
from pathlib import Path

import numpy as np
from PIL import Image
import pytest
import torch

from aic_transfuser_lite.contracts.model_batch_v3 import ModelBatchV3, TrainingTargetsV3
from aic_transfuser_lite.data.spatial_diagnostic_inputs_v4 import build_inputs, epoch_keys, INPUT_FIELDS
from aic_transfuser_lite.models.full_control_lite_v3 import FullControlLiteV3
from aic_transfuser_lite.models.spatial_path_diagnostic_v4 import SpatialPathDiagnosticV4
from aic_transfuser_lite.training.spatial_diagnostic_v4 import masked_xy_loss, deterministic_candidates, optimizer_for, check_hash


def batch() -> ModelBatchV3:
    return ModelBatchV3(image=torch.randn(2, 4, 3, 16, 16), image_mask=torch.ones(2, 4, dtype=torch.bool),
        lidar=torch.rand(2, 4, 2, 16), lidar_mask=torch.ones(2, 4, dtype=torch.bool),
        ego=torch.rand(2, 10, 4), ego_feature_mask=torch.ones(2, 10, 4, dtype=torch.bool),
        command_history=torch.rand(2, 10, 3), command_mask=torch.ones(2, 10, dtype=torch.bool),
        sensor_dt_sec=torch.zeros(2, 4, 2))


KWARGS = dict(image_height=16, image_width=16, lidar_points=16, ego_dim=4, hidden_dim=16,
              camera_tokens_hw=(1, 1), lidar_tokens=2, fusion_depth=1, fusion_heads=4)


def test_masked_loss_invalid_tail_values_and_gradients() -> None:
    p = torch.randn(2, 20, 2, requires_grad=True)
    target = torch.randn_like(p)
    mask = torch.zeros(2, 20, dtype=torch.bool)
    mask[0, :2] = True; mask[1, :8] = True
    first = masked_xy_loss(p, target, mask)
    first.backward(); before = p.grad.clone(); p.grad.zero_()
    target[~mask] = float("nan")
    second = masked_xy_loss(p, target, mask)
    second.backward()
    torch.testing.assert_close(first, second, rtol=0, atol=0)
    torch.testing.assert_close(before, p.grad, rtol=0, atol=0)
    assert (p.grad[~mask] == 0).all()


def test_zero_support_no_optimizer_step() -> None:
    p = torch.nn.Parameter(torch.ones(2, 20, 2))
    optimizer = torch.optim.AdamW([p])
    loss = masked_xy_loss(p, torch.full_like(p, float("nan")), torch.zeros(2, 20, dtype=torch.bool))
    if loss is not None:
        loss.backward(); optimizer.step()
    assert loss is None and optimizer.state == {} and (p == 1).all()


def test_loss_anchor_balancing() -> None:
    p = torch.zeros(2, 20, 2)
    p[0] = 1.; p[1] = 2.
    mask = torch.ones(2, 20, dtype=torch.bool); mask[0, 1:] = False
    assert masked_xy_loss(p, torch.zeros_like(p), mask).item() == pytest.approx((.95+1.95)/2)


def test_teacher_change_cannot_change_forward() -> None:
    model = SpatialPathDiagnosticV4(**KWARGS).eval()
    inputs = batch()
    target = TrainingTargetsV3(torch.full((2, 20, 2), float("nan")), torch.zeros(2, 20, dtype=torch.bool),
                              torch.zeros(2, 20), torch.zeros(2, 20, dtype=torch.bool))
    with torch.no_grad():
        first = model(inputs)
        second = model(replace(inputs, targets=target))
    assert first.shape == (2, 20, 2)
    torch.testing.assert_close(first, second, rtol=0, atol=0)


def test_v3_features_preserve_default_heads_and_state_keys() -> None:
    model = FullControlLiteV3(**KWARGS).eval()
    clone = FullControlLiteV3(**KWARGS).eval()
    clone.load_state_dict(model.state_dict(), strict=True)
    inputs = batch()
    with torch.no_grad():
        output = model(inputs)
        features = clone.forward_features(inputs)
        xy, logits = clone.trajectory_head(features)
        speed = clone.speed_profile_head(features)
    torch.testing.assert_close(output.trajectory_xy, xy, rtol=0, atol=0)
    torch.testing.assert_close(output.trajectory_speed_mps, speed, rtol=0, atol=0)
    torch.testing.assert_close(output.candidate_logits, logits, rtol=0, atol=0)


def test_allowlist_old_heads_excluded_and_optimizer_exact() -> None:
    old = FullControlLiteV3(**KWARGS)
    model = SpatialPathDiagnosticV4(**KWARGS)
    original = {k: v.clone() for k, v in model.path_head.state_dict().items()}
    report = model.initialize_representation(old.state_dict())
    assert report["loaded"] and any(k.startswith("trajectory_head.") for k in report["skipped"])
    for k, v in original.items():
        torch.testing.assert_close(v, model.path_head.state_dict()[k])
    assert not any("trajectory_head" in k or "speed_profile_head" in k for k in model.state_dict())
    optimizer = optimizer_for(model)
    assert {id(p) for group in optimizer.param_groups for p in group["params"]} == {id(p) for p in model.parameters()}
    assert [g["lr"] for g in optimizer.param_groups] == [1e-4, 1e-3]


def test_synthetic_optimizer_one_step() -> None:
    torch.manual_seed(42)
    model = SpatialPathDiagnosticV4(**KWARGS)
    optimizer = optimizer_for(model)
    before = model.path_head[-1].weight.detach().clone()
    loss = masked_xy_loss(model(batch()), torch.zeros(2, 20, 2), torch.ones(2, 20, dtype=torch.bool))
    loss.backward(); torch.nn.utils.clip_grad_norm_(model.parameters(), 1., error_if_nonfinite=True); optimizer.step()
    assert not torch.equal(before, model.path_head[-1].weight)
    assert torch.isfinite(loss)


def test_history_epoch_gap_reset_run_and_command_causality() -> None:
    rows = [dict(run_id="train", segment_id="s", grid_stamp_ns=str(x), sample_id=str(x)) for x in (0, 100_000_000, 500_000_000)]
    keys = epoch_keys(rows)
    assert keys[0] == keys[1] and keys[1] != keys[2]
    rows.append(dict(rows[-1], run_id="other", sample_id="other"))
    assert epoch_keys(rows)[-1] != keys[-1]


def test_selected_input_padding_and_past_only_commands(tmp_path: Path) -> None:
    # Read the independently assembled runtime-compatible tensor contract, without a real Dataset.
    image = io.BytesIO(); Image.new("RGB", (16, 16), "white").save(image, format="JPEG")
    ranges = io.BytesIO(); np.save(ranges, np.ones(16, dtype=np.float32), allow_pickle=False)
    valid = io.BytesIO(); np.save(valid, np.ones(16, dtype=bool), allow_pickle=False)
    blobs = {"image": image.getvalue(), "ranges": ranges.getvalue(), "valid": valid.getvalue()}
    command = json.dumps(dict(valid=True, steering_rad=.1, speed_mps=.75, acceleration_mps2=.2))
    rows = [dict(run_id="train", segment_id="s", grid_stamp_ns=str(i*100_000_000), sample_id=str(i),
                 image_path="image", lidar_path="ranges", lidar_valid_path="valid", camera_delta_ms="0", lidar_delta_ms="1",
                 velocity_longitudinal_mps="1", velocity_lateral_mps="0", yaw_rate_rps="0", actual_steering_rad="0",
                 actual_steering_valid="True", nominal_command=command, final_command=command) for i in range(3)]
    inputs, history = build_inputs(rows, 2, blobs.__getitem__, height=16, width=16, lidar_points=16)
    assert inputs.image_mask.tolist() == [[False, True, True, True]]
    assert inputs.command_mask.tolist() == [[False]*8+[True]*2]
    assert history["command_stamps_ns"] == [None]*8+[0, 100_000_000]
    assert inputs.targets is None and inputs.ego_feature_mask[0, :7].sum() == 0


def test_selection_determinism_and_identity_rejection(tmp_path: Path) -> None:
    rows = [dict(run_id=f"r{i%3}", sample_id=str(i)) for i in range(3000)]
    assert deterministic_candidates(rows) == deterministic_candidates(rows)
    assert len(set(deterministic_candidates(rows))) == 2048
    path = tmp_path / "manifest.yaml"; path.write_bytes(b"synthetic")
    with pytest.raises(ValueError, match="identity"):
        check_hash(path, "0"*64)


def test_immutable_and_failed_execution_record(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    import yaml
    from aic_transfuser_lite.training.spatial_diagnostic_v4 import FIXED, run_experiment
    output = tmp_path / "diagnostic"
    config = tmp_path / "config.yaml"
    config.write_text(yaml.safe_dump({"diagnostic": FIXED, "dataset_root": str(tmp_path / "never_read")}))
    monkeypatch.setattr(torch.cuda, "is_available", lambda: False)
    assert run_experiment(config, output) == 1
    manifest = json.loads((output / "artifacts/execution_manifest.json").read_text())
    assert manifest["status"] == "FAILED_OR_BLOCKED" and manifest["steps"] == 0
    before = (output / "artifacts/execution_manifest.json").read_bytes()
    with pytest.raises(FileExistsError):
        run_experiment(config, output)
    assert (output / "artifacts/execution_manifest.json").read_bytes() == before
