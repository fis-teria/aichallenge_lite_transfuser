from __future__ import annotations

from dataclasses import replace
import json
from pathlib import Path

import numpy as np
import pytest
import yaml

from aic_transfuser_lite.data.spatial_coverage_v4 import (
    SpatialAuditConfig, aggregate, bounded_pose_prefix, csv_rows, future_geometry,
    identity, load_annotations, resample_no_extrapolation, run_audit, sha256_file,
    source_entry, v3_row_status, validate_sources, write_csv,
)

ROOT = Path(__file__).parents[1]
CONFIG_PATH = ROOT / "configs/models/trajectory_authoritative_finetune_v3.yaml"


def future(speed: float = 1.0) -> np.ndarray:
    values = np.zeros((30, 8))
    values[:, 0] = np.arange(1, 31) * .1
    values[:, 1] = values[:, 0] * speed
    values[:, 4] = speed
    values[:, 7] = 1
    return values


def row(sample: str = "s", run: str = "run") -> dict[str, str]:
    command = json.dumps(dict(valid=True, steering_rad=0, speed_mps=.75, acceleration_mps2=0))
    return dict(sample_id=sample, run_id=run, segment_id="epoch0000", scenario_id="d1_sim",
                grid_stamp_ns="100000000", velocity_longitudinal_mps="0", velocity_lateral_mps="0",
                yaw_rate_rps="0", actual_steering_rad="0", actual_steering_valid="True",
                nominal_command=command, final_command=command, future_valid_count="30",
                future_step_count="30", trajectory_path=f"trajectories/{sample}.npy")


def model_config() -> dict:
    return yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))


def test_exact_filter_censor_and_nominal_fallback() -> None:
    r, f = row(), future(0)
    assert v3_row_status(r, f, model_config())["exclusion_primary"] == "contradictory_stationary"
    f[14, 7] = 0
    f[14, :7] = np.nan
    status = v3_row_status(r, f, model_config())
    assert status["v3_motion_assessment"] == "censored_future"
    assert status["v3_quality_member"] is True
    r["nominal_command"] = json.dumps({"valid": False})
    assert v3_row_status(r, f, model_config())["command_source"] == "final_fallback"


def test_signed_speed_not_abs_and_genuine_stop() -> None:
    f = future(0)
    f[:, 4] = -1
    assert v3_row_status(row(), f, model_config())["exclusion_primary"] == "contradictory_stationary"
    r = row()
    r["nominal_command"] = json.dumps(dict(valid=True, steering_rad=0, speed_mps=0, acceleration_mps2=0))
    assert v3_row_status(r, f, model_config())["v3_quality_member"]


def test_primary_exclusion_preserves_multi_flags() -> None:
    r = row()
    r.update(future_valid_count="0", actual_steering_valid="False")
    status = v3_row_status(r, future(), model_config())
    assert status["base_exclusion_primary"] == "zero_valid_future"
    assert status["v3_exclusion_flags"] == ["zero_valid_future", "invalid_current_ego"]


def test_noise_stationary_vs_accumulated_slow_motion() -> None:
    f = future(0)
    f[:, 1] = np.where(np.arange(30) % 2, .002, -.002)
    g = future_geometry(f, SpatialAuditConfig(), horizon_sec=3)
    assert g["raw_arc_m"] > .1
    assert g["noise_filtered_arc_m_provisional"] == 0
    assert g["maximum_hold_sec"] == pytest.approx(3)
    g = future_geometry(future(.02), SpatialAuditConfig(), horizon_sec=3)
    assert g["noise_filtered_arc_m_provisional"] > .05
    assert g["curvature_max_abs_per_m"] is None


def test_gap_invalid_padding_no_future_and_nan() -> None:
    f = future()
    f[5, 7] = 0
    f[5, 1:7] = np.nan
    g = future_geometry(f, SpatialAuditConfig(), horizon_sec=1.5)
    assert g["prefix_count"] == 5
    assert g["raw_arc_m"] == pytest.approx(.5)
    assert g["valid_count"] == 14
    f[:, 7] = 0
    assert future_geometry(f, SpatialAuditConfig(), horizon_sec=1.5)["raw_arc_m"] is None
    f[0, 7] = 1
    f[0, 1] = np.nan
    assert "nonfinite_valid_future" in future_geometry(f, SpatialAuditConfig(), horizon_sec=1.5)["flags"]


def test_arc_not_x_and_nonmonotonic_x_curve() -> None:
    f = future()
    angle = np.arange(1, 31) * .1
    f[:, 1] = np.sin(angle)
    f[:, 2] = 1 - np.cos(angle)
    g = future_geometry(f, SpatialAuditConfig(), horizon_sec=3)
    assert g["raw_arc_m"] > 2.9
    assert g["endpoint_x_m"] < .2
    assert g["curvature_max_abs_per_m"] == pytest.approx(1, abs=.01)
    assert g["raw_reaches_2m"]


def test_resample_duplicates_endpoint_and_corner_cut() -> None:
    xy = np.array([[0, 0], [.15, 0], [.15, 0], [.15, .13]])
    points, diagnostic = resample_no_extrapolation(xy)
    np.testing.assert_allclose(points[-1], xy[-1])
    assert len(np.unique(points, axis=0)) == len(points)
    assert diagnostic["corner_cut_loss_m"] > 0
    assert not diagnostic["extrapolated"]
    points, _ = resample_no_extrapolation(np.zeros((3, 2)))
    assert points.shape == (1, 2)


@pytest.mark.parametrize("bad", [np.zeros((8,)), np.zeros((2, 7)), np.empty((0, 8))])
def test_future_shape(bad: np.ndarray) -> None:
    with pytest.raises(ValueError, match="H,8"):
        future_geometry(bad, SpatialAuditConfig(), horizon_sec=1.5)


def pose_records() -> list[dict]:
    return [dict(run_id="a", split="train", segment_id="e", reset_id="r", route_intent="route1",
                 stamp_sec=i * .1, x_m=i * .03, y_m=0, speed_mps=.3, hold=False, reverse=False)
            for i in range(31)]


@pytest.mark.parametrize("key,value,reason", [
    ("run_id", "b", "run_id_boundary"), ("split", "test", "split_boundary"),
    ("segment_id", "e2", "segment_id_boundary"), ("reset_id", "r2", "reset_id_boundary"),
    ("route_intent", "route2", "route_intent_boundary"),
    ("stamp_sec", 0, "timestamp_non_monotonic"), ("stamp_sec", .5, "timestamp_gap"),
    ("x_m", 10, "teleport"), ("reverse", True, "reverse_boundary"),
    ("hold", None, "motion_intent_unknown"), ("x_m", float("nan"), "nonfinite_pose"),
])
def test_long_pose_boundaries(key: str, value: object, reason: str) -> None:
    records = pose_records()
    records[1][key] = value
    assert bounded_pose_prefix(records, config=SpatialAuditConfig()) == (1, reason)


def test_long_pose_hold_unknown_intent_and_limits() -> None:
    records = pose_records()
    assert bounded_pose_prefix(records, config=SpatialAuditConfig(), max_elapsed_sec=1.5) == (16, "elapsed_limit")
    assert bounded_pose_prefix(records, config=replace(SpatialAuditConfig(), maximum_distance_m=.2))[1] == "distance_limit"
    for r in records:
        r["hold"] = True
    assert bounded_pose_prefix(records, config=SpatialAuditConfig())[1] == "long_hold"
    records[0]["route_intent"] = "unknown"
    assert bounded_pose_prefix(records, config=SpatialAuditConfig())[0] == 0
    assert bounded_pose_prefix([], config=SpatialAuditConfig()) == (0, "SOURCE_UNAVAILABLE")


def fixture_dataset(tmp_path: Path) -> tuple[Path, Path, str]:
    root = tmp_path / "data"
    (root / "trajectories").mkdir(parents=True)
    rows = [row("s1", "run1"), row("s2", "run2"), row("s3", "run3")]
    for r in rows:
        np.save(root / r["trajectory_path"], future(0))
    write_csv(root / "samples.csv", rows, list(rows[0]))
    runs = [dict(run_id=r["run_id"], source_hash=r["run_id"], source_uri="file:///nonexistent-audit-test") for r in rows]
    write_csv(root / "runs.csv", runs, list(runs[0]))
    manifest = dict(complete=True, schema_version="aic_canonical_dataset_v3", runs=runs,
                    files=[dict(path=str(p.relative_to(root)).replace("\\", "/"), sha256=sha256_file(p))
                           for p in sorted(root.rglob("*")) if p.is_file()])
    digest = identity(manifest)
    (root / "manifest.yaml").write_text(yaml.safe_dump({**manifest, "manifest_sha256": digest}))
    split = tmp_path / "split.json"
    split.write_text(json.dumps(dict(dataset_manifest_sha256=digest, assignments=[
        dict(run_id=f"run{i}", split=s) for i, s in enumerate(("train", "validation", "test"), 1)])))
    return root, split, digest


def test_source_status_and_identity_failure(tmp_path: Path) -> None:
    assert source_entry("unknown", None)["status"] == "NOT_INSPECTED"
    assert source_entry("absent", tmp_path / "absent")["status"] == "MISSING"
    root, split, digest = fixture_dataset(tmp_path)
    assert validate_sources(root, split, digest)[1]["run2"] == "val"
    with pytest.raises(ValueError, match="expected dataset identity"):
        validate_sources(root, split, "wrong")
    (root / "samples.csv").write_text("changed")
    with pytest.raises(ValueError, match="metadata file identity"):
        validate_sources(root, split, digest)


def test_audit_all_anchor_ledger_default_test_and_determinism(tmp_path: Path) -> None:
    root, split, digest = fixture_dataset(tmp_path)
    before = {p: sha256_file(p) for p in root.rglob("*") if p.is_file()}
    args = dict(dataset_root=root, split_manifest=split, repo=ROOT, model_config_path=CONFIG_PATH,
                expected_identity=digest, config=SpatialAuditConfig())
    first = run_audit(**args, output=tmp_path / "audit1")
    assert first["status"] == "COMPLETE", first
    assert first["processed_geometry_count"] == 2
    ledger = csv_rows(tmp_path / "audit1/anchor_audit_ledger.csv")
    assert len(ledger) == 3
    assert ledger[-1]["geometry_status"] == "NOT_INSPECTED"
    assert ledger[-1]["h15_raw_arc_m"] == ""
    assert ledger[-1]["session_id"] == "unknown"
    run_audit(**args, output=tmp_path / "audit2")
    for name in ("anchor_audit_ledger.csv", "coverage_summary.json", "run_summary.json"):
        assert sha256_file(tmp_path / "audit1" / name) == sha256_file(tmp_path / "audit2" / name)
    assert before == {p: sha256_file(p) for p in before}
    with pytest.raises(FileExistsError):
        run_audit(**args, output=tmp_path / "audit1")
    with pytest.raises(ValueError, match="overlaps"):
        run_audit(**args, output=root / "audit")
    partial = run_audit(**{**args, "config": SpatialAuditConfig(max_anchors=1)}, output=tmp_path / "partial")
    assert partial["status"] == "PARTIAL"
    assert len(csv_rows(tmp_path / "partial/anchor_audit_ledger.csv")) == 3
    with pytest.raises(ValueError, match="explicit"):
        SpatialAuditConfig(splits=("test",)).validate()


def test_annotation_identity_not_silently_joined(tmp_path: Path) -> None:
    view = tmp_path / "view"
    view.mkdir()
    (view / "labels.csv").write_text("sample_id\ns\n")
    (view / "manifest.json").write_text(json.dumps(dict(dataset_manifest_sha256="wrong",
        labels_sha256=sha256_file(view / "labels.csv"))))
    with pytest.raises(ValueError, match="explicit parent"):
        load_annotations(view, "labels.csv", {"manifest_sha256": "actual"})


def test_unreadable_v3_label_does_not_become_quality_member() -> None:
    f = future(0)
    f[0, 1] = np.inf
    status = v3_row_status(row(), f, model_config())
    assert status["v3_motion_assessment"] == "UNREADABLE"
    assert status["v3_quality_member"] is None


def test_source_leakage_is_recomputed_not_trusted(tmp_path: Path) -> None:
    root, split, digest = fixture_dataset(tmp_path)
    manifest = yaml.safe_load((root / "manifest.yaml").read_text())
    manifest["runs"][1]["source_hash"] = manifest["runs"][0]["source_hash"]
    write_csv(root / "other.csv", manifest["runs"], list(manifest["runs"][0]))
    (root / "runs.csv").write_bytes((root / "other.csv").read_bytes())
    for entry in manifest["files"]:
        if entry["path"] == "runs.csv":
            entry["sha256"] = sha256_file(root / "runs.csv")
    manifest.pop("manifest_sha256")
    digest = identity(manifest)
    (root / "manifest.yaml").write_text(yaml.safe_dump({**manifest, "manifest_sha256": digest}))
    split_meta = json.loads(split.read_text())
    split_meta["dataset_manifest_sha256"] = digest
    split.write_text(json.dumps(split_meta))
    with pytest.raises(ValueError, match="source hash crosses"):
        validate_sources(root, split, digest)


def test_ledger_unknown_missing_case_episode_counts(tmp_path: Path) -> None:
    root, split, digest = fixture_dataset(tmp_path)
    output = tmp_path / "audit"
    result = run_audit(dataset_root=root, split_manifest=split, expected_identity=digest,
        output=output, repo=ROOT, model_config_path=CONFIG_PATH, config=SpatialAuditConfig())
    assert result["status"] == "COMPLETE"
    cases = json.loads((output / "recovery_case_matrix.json").read_text())
    assert [r for r in cases if r["split"] == "val" and r["side"] == "right" and r["near_far"] == "near"][0]["anchors"] == 0
    ledger = csv_rows(output / "anchor_audit_ledger.csv")
    assert {r["side"] for r in ledger} == {"unknown"}
    summary = next(s for s in result["summaries"] if s["split"] == "train")
    assert summary["estimated_episodes"] == 1
    assert summary["confirmed_episodes"] is None
    assert sum(summary["primary_exclusions"].values()) == summary["raw_anchors"]
    assert summary["multiple_exclusion_flags"]["contradictory_stationary"] == 1


@pytest.mark.parametrize("side", [-1, 1])
def test_left_right_curves_and_duplicate_endpoint_do_not_inflate_support(side: int) -> None:
    f = future()
    angles = np.arange(1, 31) * .1
    f[:, 1] = np.sin(angles)
    f[:, 2] = side * (1 - np.cos(angles))
    f[20:, 1:3] = f[19, 1:3]
    g = future_geometry(f, SpatialAuditConfig(), horizon_sec=3)
    assert g["raw_arc_m"] == pytest.approx(20 * 2 * np.sin(.05))
    assert g["duplicate_segments"] == 10
    assert g["continuation_impossible_evidence"] == "unknown"


def test_corrupt_asset_is_visible_partial_not_missing_zero(tmp_path: Path) -> None:
    root, split, digest = fixture_dataset(tmp_path)
    (root / "trajectories/s1.npy").write_bytes(b"bad")
    output = tmp_path / "audit"
    result = run_audit(dataset_root=root, split_manifest=split, expected_identity=digest,
        output=output, repo=ROOT, model_config_path=CONFIG_PATH, config=SpatialAuditConfig())
    assert result["status"] == "PARTIAL"
    ledger = csv_rows(output / "anchor_audit_ledger.csv")
    assert ledger[0]["geometry_status"] == "UNREADABLE"
    assert ledger[0]["h15_raw_arc_m"] == ""
    assert len(ledger) == 3
