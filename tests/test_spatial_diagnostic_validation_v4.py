"""Synthetic read-only checks only. No optimizer construction or step."""
from dataclasses import replace
import io
import json
from pathlib import Path

import numpy as np
import pytest
import torch
import yaml

from aic_transfuser_lite.contracts.model_batch_v3 import ModelBatchV3, TrainingTargetsV3
from aic_transfuser_lite.data.spatial_diagnostic_view_v4 import spatial_target
from aic_transfuser_lite.models.spatial_path_diagnostic_v4 import SpatialPathDiagnosticV4
from aic_transfuser_lite.evaluation.spatial_diagnostic_validation_v4 import (
    FIXED, Budget, ValidationReadOnlyAccess, aggregate, candidate_indices, check_identity, compare_replay,
    comparisons, digest, eligibility, execute, grouped, infer, metric_rows, select_validation, split_maps,
    state_inventory, strict_restore, train_template, read_checked,
)


@pytest.fixture(autouse=True)
def forbid_optimizer(monkeypatch: pytest.MonkeyPatch) -> None:
    def forbidden(*args: object, **kwargs: object) -> None:
        raise AssertionError("optimizer construction prohibited")
    monkeypatch.setattr(torch.optim.Optimizer, "__init__", forbidden)


def future(speed: float = 1.) -> np.ndarray:
    value = np.zeros((30, 8)); value[:, 0] = np.arange(1, 31)*.1
    value[:, 1] = value[:, 0]*speed; value[:, 4] = speed; value[:, 7] = 1
    return value


def candidate(i: int, run: str = "v", shape: str = "straight") -> dict:
    return {"sample_id": str(i), "run_id": run, "stamp_ns": i*100_000_000, "reasons": [],
            "shape": shape, "cut_reason": "stored_horizon_end", "current_ego_valid": True,
            "normal_recovery": "UNKNOWN", "collection_slice": "UNKNOWN", "role": "validation_main"}


def inputs() -> ModelBatchV3:
    return ModelBatchV3(image=torch.zeros(2, 4, 3, 16, 16), image_mask=torch.ones(2, 4, dtype=torch.bool),
        lidar=torch.ones(2, 4, 2, 16), lidar_mask=torch.ones(2, 4, dtype=torch.bool),
        ego=torch.ones(2, 10, 4), ego_feature_mask=torch.ones(2, 10, 4, dtype=torch.bool),
        command_history=torch.zeros(2, 10, 3), command_mask=torch.ones(2, 10, dtype=torch.bool), sensor_dt_sec=torch.zeros(2, 4, 2))


def model() -> SpatialPathDiagnosticV4:
    return SpatialPathDiagnosticV4(image_height=16, image_width=16, lidar_points=16, ego_dim=4,
        hidden_dim=16, camera_tokens_hw=(1, 1), lidar_tokens=2, fusion_depth=1, fusion_heads=4)


def test_split_source_and_duplicate_guards() -> None:
    assignments = [dict(run_id="a", split="train"), dict(run_id="b", split="validation")]
    runs = [dict(run_id="a", source_hash="1"), dict(run_id="b", source_hash="2")]
    assert split_maps(assignments, runs)["b"] == "validation"
    with pytest.raises(ValueError):
        split_maps(assignments + assignments[:1], runs)
    runs[1]["source_hash"] = "1"
    with pytest.raises(ValueError, match="source"):
        split_maps(assignments, runs)


def test_test_and_unselected_asset_denied_before_io(tmp_path: Path) -> None:
    access = ValidationReadOnlyAccess.__new__(ValidationReadOnlyAccess)
    access.root = tmp_path
    access.inventory = {"trajectories/test/x.npy": {}, "images/v/x.jpg": {}}
    access.allowed_futures, access.allowed_sensors = {}, set()
    access.mapping = {"test": "test", "train": "train"}; access.replay_ids = {"selected"}
    for path in access.inventory:
        with pytest.raises(ValueError, match="forbidden"):
            access.read(path)
    with pytest.raises(ValueError):
        access.authorize_future(dict(run_id="test", sample_id="x", trajectory_path="trajectories/test/x.npy"))
    with pytest.raises(ValueError):
        access.authorize_future(dict(run_id="train", sample_id="x", trajectory_path="trajectories/train/x.npy"))
    assert not list(tmp_path.iterdir())


def test_future_budget_stops_before_io(tmp_path: Path) -> None:
    access = ValidationReadOnlyAccess.__new__(ValidationReadOnlyAccess)
    access.root = tmp_path; access.inventory = {"trajectories/v/x.npy": {}}
    access.allowed_futures = {"trajectories/v/x.npy": "v"}; access.allowed_sensors = set()
    access.mapping = {"v": "validation"}; access.future_by_run = {"v": {str(i) for i in range(256)}}
    with pytest.raises(ValueError, match="budget"):
        access.read("trajectories/v/x.npy")


def test_candidate_selection_determinism_budgets_gap() -> None:
    rows = [dict(run_id=f"v{i%5}", sample_id=str(i)) for i in range(3000)]
    assert candidate_indices(rows) == candidate_indices(rows)
    assert len(set(candidate_indices(rows))) == 1280
    candidates = [candidate(i, "v"+str(i//256), ("left", "right", "straight")[i%3]) for i in range(1280)]
    selected = select_validation(candidates)
    assert selected == select_validation(candidates)
    assert len({c["sample_id"] for c in selected}) == len(selected) <= 180
    for run in {c["run_id"] for c in candidates}:
        main = [c for c in selected if c["run_id"] == run and c["role"] == "validation_main"]
        assert len(main) <= 32
        assert all(abs(a["stamp_ns"]-b["stamp_ns"]) >= 500_000_000 for i, a in enumerate(main) for b in main[i+1:])


def test_exact_old_eligibility_and_observation() -> None:
    row = dict(velocity_longitudinal_mps="1", velocity_lateral_mps="0", yaw_rate_rps="0", actual_steering_rad="0", actual_steering_valid="True")
    f = future(); view = spatial_target(f)
    assert eligibility(row, f, view) == ([], True)
    row["velocity_longitudinal_mps"] = "0"
    assert "stopped_observation" in eligibility(row, f, view)[0]
    f[0, 7] = 0; v = spatial_target(f)
    assert "short_or_unknown_support" in eligibility(row, f, v)[0]


@pytest.mark.parametrize("fault", ["none", "format", "key", "shape", "dtype", "nan"])
def test_strict_checkpoint_no_fallback(fault: str) -> None:
    original = model(); state = original.state_dict(); name = next(iter(state))
    payload = {"format": "SPATIAL_DIAGNOSTIC_NOT_RUNTIME", "state_dict": state}
    if fault == "format": payload["format"] = "wrong"
    if fault == "key": del state[name]
    if fault == "shape": state[name] = state[name].flatten()[:1]
    if fault == "dtype": state[name] = state[name].double()
    if fault == "nan": state[name] = torch.full_like(state[name], float("nan"))
    blob = io.BytesIO(); torch.save(payload, blob)
    if fault == "none":
        clone = model(); report = strict_restore(clone, blob.getvalue())
        assert not clone.training and not any(p.requires_grad for p in clone.parameters())
        assert report["state"] == state_inventory(clone)
    else:
        with pytest.raises(ValueError): strict_restore(model(), blob.getvalue())


def test_eval_state_buffer_and_teacher_invariance() -> None:
    network = model().eval(); batch = inputs(); before = state_inventory(network)
    target = TrainingTargetsV3(torch.full((2, 20, 2), float("nan")), torch.zeros(2, 20, dtype=torch.bool), torch.zeros(2, 20), torch.zeros(2, 20, dtype=torch.bool))
    with torch.inference_mode():
        actual = network(batch)
        altered = network(replace(batch, targets=target))
    torch.testing.assert_close(actual, altered, rtol=0, atol=0)
    assert before == state_inventory(network) and not actual.requires_grad


def test_replay_tolerance_contract_hash_failure(tmp_path: Path) -> None:
    a = np.zeros((2, 20, 2), np.float32)
    assert compare_replay(a, a, ["a", "b"])["passed"]
    b = a.copy(); b[1, 0, 0] = 2e-6
    assert compare_replay(b, a, ["a", "b"])["exceeded_ids"] == ["b"]
    with pytest.raises(ValueError): check_identity({"identity": "bad"}, "bad")
    file = tmp_path / "checkpoint.pt"; file.write_bytes(b"synthetic")
    with pytest.raises(ValueError, match="identity"): read_checked(file, "0"*64)


def test_train_template_missing_support_never_val_mean() -> None:
    xy = np.ones((2, 20, 2), np.float32); mask = np.ones((2, 20), bool); mask[:, -1] = False
    template, support = train_template(xy, mask)
    assert np.isnan(template[-1]).all() and support[-1] == 0
    xy[:] = 100  # Already-computed template has no reference to later/validation values.
    assert (template[:-1] == 1).all()


def test_metrics_roles_denominators_unknown_and_tail_crossing() -> None:
    f = future(); v = spatial_target(f)
    xy = np.stack([v["xy"]]*3); mask = np.zeros((3, 20), bool); mask[0, :1] = True; mask[1, :4] = True
    pred = np.zeros_like(xy)
    pred[0, :4] = [[1, 1], [0, 1], [1, 0], [2, 0]]
    pred[1, :4] = pred[0, :4]
    selected = [candidate(i) for i in range(3)]; selected[1]["role"] = "validation_observation"
    rows = metric_rows(pred, xy, mask, selected, [v]*3, np.array([True, True, False]))
    assert rows[0]["proper_crossing_all20"] and not rows[0]["proper_crossing_teacher_prefix"]
    assert rows[1]["proper_crossing_teacher_prefix"]
    assert rows[2]["unknown_reason"] == "NOT_PROCESSED" and rows[2]["ade_m"] is None
    group = grouped(rows)
    assert group["validation_main"]["overall"]["anchors"] == 2
    assert group["validation_observation"]["overall"]["anchors"] == 1
    assert comparisons(rows, rows)["groups"]["validation_main"]["overall"]["ties"] == 1


def test_first_missing_known_zero_and_nonfinite_not_dropped() -> None:
    missing = future(); missing[0, 7] = 0
    views = [spatial_target(missing), spatial_target(future(0))]
    assert views[0]["distance_status"] == "UNKNOWN_FIRST_MISSING"
    assert views[1]["raw_length_m"] == 0 and views[1]["distance_status"] == "KNOWN_OBSERVED"
    pred = np.full((2, 20, 2), np.nan)
    rows = metric_rows(pred, np.zeros_like(pred), np.ones((2, 20), bool), [candidate(0), candidate(1)], views, np.ones(2, bool))
    summary = aggregate(rows)
    assert summary["anchors"] == 2 and summary["scored_anchors"] == 0 and summary["nonfinite_all20_count"] == 2


def test_budget_and_output_collision_partial_record(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    with pytest.raises(TimeoutError): Budget(-1).check()
    config = tmp_path / "config.yaml"
    config.write_text(yaml.safe_dump({"evaluation": FIXED, "prior_run": str(tmp_path/"absent_prior"), "dataset_root": str(tmp_path/"absent_data")}))
    output = tmp_path / "new"
    assert execute(config, output) == 1
    recorded = (output/"artifacts/execution_manifest.json").read_bytes()
    assert json.loads(recorded)["status"] == "BLOCKED"
    with pytest.raises(FileExistsError): execute(config, output)
    assert (output/"artifacts/execution_manifest.json").read_bytes() == recorded
    def exhausted(*args: object, **kwargs: object) -> bytes:
        raise TimeoutError("synthetic fixed budget")
    monkeypatch.setattr("aic_transfuser_lite.evaluation.spatial_diagnostic_validation_v4.read_checked", exhausted)
    second = tmp_path / "partial"
    assert execute(config, second) == 1
    assert json.loads((second/"artifacts/execution_manifest.json").read_text())["status"] == "PARTIAL"


def test_nonfinite_inference_preserves_failed_and_unprocessed() -> None:
    from aic_transfuser_lite.data.spatial_diagnostic_inputs_v4 import INPUT_FIELDS
    class Nonfinite(torch.nn.Module):
        def __init__(self) -> None:
            super().__init__(); self.weight = torch.nn.Parameter(torch.ones(1))
        def forward(self, batch: ModelBatchV3) -> torch.Tensor:
            return torch.full((batch.batch_size, 20, 2), float("nan"))
    source = inputs()
    single = replace(source, **{field: getattr(source, field)[:1] for field in INPUT_FIELDS})
    predictions, processed = np.full((3, 20, 2), np.nan), np.zeros(3, bool)
    with pytest.raises(ValueError, match="nonfinite"):
        infer(Nonfinite().eval(), [single]*3, predictions, processed, Budget())
    assert processed.tolist() == [True, True, False]
