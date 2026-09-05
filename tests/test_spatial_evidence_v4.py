from __future__ import annotations

from dataclasses import replace
from io import BytesIO
import json
import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from aic_transfuser_lite.data.spatial_evidence_v4 import (
    EvidenceConfig, EXPECTED, PREVIOUS_IMPLEMENTATION, choose_anchors, compare_horizons,
    evidence_for_anchor, interpolate_records, observed_boundaries, reproduce_future,
    run_evidence_audit, stopping_context, strict_polyline,
)
from aic_transfuser_lite.data.spatial_source_reader_v4 import (
    BudgetExceeded, MeteredStream, ReadBudget, ReadMeter, _expand, read_mcap_windows,
)
from aic_transfuser_lite.data.spatial_coverage_v4 import identity, run_audit, sha256_file, SpatialAuditConfig, write_csv

ROOT = Path(__file__).parents[1]


def future(speed: float = .5) -> np.ndarray:
    f = np.zeros((30, 8), dtype=np.float32)
    f[:, 0] = np.arange(1, 31) * .1
    f[:, 1] = np.arange(1, 31) * .1 * speed
    f[:, 4] = speed
    f[:, 7] = 1
    return f


def sample(sid: str = "s", run: str = "run") -> dict[str, str]:
    cmd = json.dumps({"valid": True, "steering_rad": 0., "speed_mps": .75, "acceleration_mps2": 0., "source_stamp_ns": 1000000000})
    return dict(sample_id=sid, run_id=run, grid_stamp_ns="1000000000", camera_delta_ms="0",
        segment_id="epoch0000", scenario_id="d1_sim", nominal_command=cmd, final_command=cmd,
        velocity_longitudinal_mps="0", velocity_lateral_mps="0", yaw_rate_rps="0", actual_steering_rad="0",
        actual_steering_valid="True", future_valid_count="30", future_step_count="30", trajectory_path=f"trajectories/{sid}.npy")


def old(sid: str = "s", run: str = "run") -> dict[str, str]:
    return dict(sample_id=sid, run_id=run, split="val", segment_id="epoch0000", grid_stamp_ns="1000000000",
        estimated_episode_id=f"{run}:inferred1", phase="hold", side="left", near_far="near", normal_recovery="recovery",
        h15_prefix_count="15", h15_raw_reaches_1m="False", h30_raw_reaches_1m="True", v3_motion_assessment="observed_motion")


def records(speed: float = .5) -> list[dict]:
    out = []
    for i in range(81):
        stamp = i * 50000000 + 500000000
        common = dict(source_id="fixture", semantic_stamp_ns=stamp, bag_stamp_ns=stamp,
                      timestamp_source="header.stamp", payload_sha256=f"hash{i}")
        out.append({**common, "topic": "/localization/kinematic_state", "value": dict(
            x_m=(stamp / 1e9 - 1) * speed + 90000, y_m=43000., yaw_rad=0., frame_id="map", child_frame_id="base_link")})
        out.append({**common, "topic": "/vehicle/status/velocity_status", "value": dict(longitudinal_mps=speed, lateral_mps=0., yaw_rate_rps=0.)})
        out.append({**common, "topic": "/clock", "value": {"clock_ns": stamp}})
    return out


def test_raw_source_reproduction_and_tier_separation() -> None:
    result = evidence_for_anchor(future(), sample(), old(), records(), True, EvidenceConfig())
    assert result["source_reproduction"]["status"] == "PASS"
    assert result["tier"] == "GEOMETRY_VERIFIED"
    assert result["path_supervision"]["status"] == "UNKNOWN"
    assert result["safety_verified"]["status"] == "UNKNOWN"
    assert result["context"]["driving_permission"]["status"] == "UNKNOWN"


def test_no_source_or_partial_read_cannot_promote_synthetic_checks() -> None:
    for recs, complete in (([], False), (records(), False)):
        result = evidence_for_anchor(future(), sample(), old(), recs, complete, EvidenceConfig())
        assert result["tier"] == "OBSERVED_ONLY"
        assert result["strict_verified_reaches"]["h30"] is None


def test_positive_command_offset_hold_and_missing_safety_not_permission() -> None:
    result = stopping_context(sample(), old(), [], 1000000000)
    assert result["driving_request"]["status"] == "PASS"
    for key in ("driving_permission", "intentional_stop_or_wait", "safety_record", "fault"):
        assert result[key]["status"] == "UNKNOWN"
    assert not result["offset_hold_is_stop_intent"]


def test_explicit_safety_record_retains_time_source_value() -> None:
    r = {"topic": "/safety_reason", "semantic_stamp_ns": 990000000, "bag_stamp_ns": 990000000,
         "source_id": "bag", "payload_sha256": "a", "value": {"data": "front_obstacle_inside_stopping_distance"}}
    result = stopping_context(sample(), old(), [r], 1000000000)
    assert result["safety_record"]["evidence"] == r
    assert result["driving_request"]["status"] == "PASS"
    assert result["driving_permission"]["status"] == "UNKNOWN"


def test_timestamp_shift_frame_mismatch_and_yaw_wrap() -> None:
    s = sample()
    s["camera_delta_ms"] = "200"
    accelerating = records()
    for r in accelerating:
        elapsed = r["semantic_stamp_ns"] / 1e9 - 1
        if r["topic"].endswith("kinematic_state"):
            r["value"]["x_m"] += .1 * elapsed**2
        elif r["topic"].endswith("velocity_status"):
            r["value"]["longitudinal_mps"] += .2 * elapsed
    f = future()
    f[:, 1] += .1 * f[:, 0] ** 2
    f[:, 4] += .2 * f[:, 0]
    assert reproduce_future(f, sample(), accelerating, EvidenceConfig())["status"] == "PASS"
    assert reproduce_future(f, s, accelerating, EvidenceConfig())["status"] == "FAIL"
    recs = [r for r in records() if r["topic"] == "/localization/kinematic_state"][:2]
    recs[0]["value"]["yaw_rad"] = math.pi - .1
    recs[1]["value"]["yaw_rad"] = -math.pi + .1
    v, _ = interpolate_records(recs, 525000000, fields=("yaw_rad",), tolerance_ns=50000000, pose=True)
    assert abs(v["yaw_rad"]) == pytest.approx(math.pi)
    recs[1]["value"]["frame_id"] = "odom"
    v, reason = interpolate_records(recs, 525000000, fields=("yaw_rad",), tolerance_ns=50000000, pose=True)
    assert v is None and reason["reason"] == "frame_mismatch"


@pytest.mark.parametrize("change", ["clock", "pose_reversal", "gap", "teleport", "frame"])
def test_real_record_boundary_checks(change: str) -> None:
    r = records()
    if change == "clock":
        next(v for v in r if v["topic"] == "/clock" and v["bag_stamp_ns"] == 2000000000)["value"]["clock_ns"] = 0
    elif change == "pose_reversal":
        next(v for v in r if v["topic"].endswith("kinematic_state") and v["bag_stamp_ns"] == 2000000000)["semantic_stamp_ns"] = 1000000000
    elif change == "gap":
        r = [v for v in r if not (v["topic"].endswith("kinematic_state") and 1500000000 < v["semantic_stamp_ns"] < 2000000000)]
    elif change == "teleport":
        next(v for v in r if v["topic"].endswith("kinematic_state") and v["bag_stamp_ns"] == 2000000000)["value"]["x_m"] += 100
    else:
        next(v for v in r if v["topic"].endswith("kinematic_state") and v["bag_stamp_ns"] == 2000000000)["value"]["frame_id"] = "other"
    assert observed_boundaries(r, 1000000000, 4000000000, EvidenceConfig())["status"] == "FAIL"


@pytest.mark.parametrize("event", ["reset", "route_intent_change", "run_boundary", "segment_boundary", "split_boundary"])
def test_explicit_boundary_events_cut_without_negative_labels(event: str) -> None:
    _, result = strict_polyline(future(), 30, EvidenceConfig(), [(1., event)])
    assert result["cut_reason"] == event
    assert result["elapsed_sec"] < 1
    assert result["negative_continuation_label"] is None


def test_horizons_share_rules_prefix_and_resampling() -> None:
    result = compare_horizons(future(), EvidenceConfig())
    assert result["raw_horizon_gains"]["1m"]
    assert result["common_prefix_agrees"]
    assert result["common_grid_max_residual_m"] == 0
    assert result["strict_diagnostic_v1"]["h15"]["arc_m"] == pytest.approx(.75)
    assert result["strict_diagnostic_v1"]["h30"]["arc_m"] == pytest.approx(1.5)


@pytest.mark.parametrize("case", ["first_missing", "partial", "nan", "duplicate", "reverse", "teleport", "gap"])
def test_geometry_edges(case: str) -> None:
    f = future()
    if case == "first_missing":
        f[0, 7] = 0
    elif case == "partial":
        f[10:, 7] = 0
    elif case == "nan":
        f[3, 1] = np.nan
    elif case == "duplicate":
        f[10:, 1:3] = f[9, 1:3]
    elif case == "reverse":
        f[10, 4] = -.1
    elif case == "teleport":
        f[10, 1] += 10
    else:
        f[10:, 0] += .5
    _, result = strict_polyline(f, 30, EvidenceConfig())
    assert result["arc_m"] <= 1.5
    assert result["negative_continuation_label"] is None
    if case == "first_missing":
        assert result["retained_steps"] == 0


def test_stationary_noise_hold_and_slow_motion() -> None:
    f = future(0)
    f[:, 1] = np.where(np.arange(30) % 2, .002, -.002)
    comparison = compare_horizons(f, EvidenceConfig())
    assert comparison["raw_v1"]["h30"]["raw_arc_m"] > .1
    assert comparison["strict_diagnostic_v1"]["h30"]["arc_m"] == 0
    assert comparison["strict_diagnostic_v1"]["h30"]["cut_reason"] == "observed_long_hold_not_intent"
    _, slow = strict_polyline(future(.02), 30, EvidenceConfig())
    assert slow["arc_m"] > .05


def test_selection_deterministic_failures_and_test_exclusion() -> None:
    rows = []
    for i in range(5):
        r = old(f"s{i}", f"r{i}")
        if i == 2:
            r.update(h30_raw_reaches_1m="False", h15_prefix_count="0", v3_motion_assessment="censored_future")
        rows.append(r)
    rows.append({**old("test", "test"), "split": "test"})
    a = choose_anchors(rows, EvidenceConfig())
    b = choose_anchors(list(reversed(rows)), EvidenceConfig())
    assert a == b
    assert len(a["selected"]) == 5
    assert "censored" in a["covered_tags"]
    assert "test" not in {r["sample_id"] for r in a["selected"]}
    assert "right_near_val" in a["absent_required_slices"]


def test_reader_budget_and_expansion_limit() -> None:
    meter = ReadMeter(ReadBudget(max_source_bytes=4))
    stream = MeteredStream(BytesIO(b"123456"), meter, "source_bytes")
    assert stream.read(3) == b"123"
    with pytest.raises(BudgetExceeded):
        stream.read(2)
    assert meter.source_bytes == 3
    with pytest.raises(BudgetExceeded):
        _expand(b"xx", "", 2, ReadMeter(ReadBudget(max_expanded_bytes=1)))
    with pytest.raises(BudgetExceeded):
        ReadMeter(ReadBudget(max_messages=1)).charge("decoded_messages", 2)
    meter = ReadMeter(ReadBudget(max_seconds=.001))
    meter.started -= 1
    with pytest.raises(BudgetExceeded, match="seconds"):
        meter.check()


@pytest.mark.parametrize("compressed", [False, True])
def test_real_mcap_fixture_reader_budget_and_windows(tmp_path: Path, compressed: bool) -> None:
    from rosbags.rosbag2 import Writer
    from rosbags.rosbag2.enums import StoragePlugin
    from rosbags.typesys import Stores, get_typestore
    import zstandard
    store = get_typestore(Stores.ROS2_HUMBLE)
    with Writer(tmp_path / "bag", version=9, storage_plugin=StoragePlugin.MCAP) as writer:
        con = writer.add_connection("/safety_reason", "std_msgs/msg/String", typestore=store)
        sensor = writer.add_connection("/not_allowed", "std_msgs/msg/String", typestore=store)
        for i in range(10):
            msg = store.types["std_msgs/msg/String"](data=f"reason{i}")
            raw = store.serialize_cdr(msg, "std_msgs/msg/String")
            writer.write(con, 1000000000 + i * 100000000, raw)
            writer.write(sensor, 1000000000 + i * 100000000, raw)
    path = next((tmp_path / "bag").glob("*.mcap"))
    if compressed:
        target = path.with_suffix(".mcap.zstd")
        target.write_bytes(zstandard.ZstdCompressor().compress(path.read_bytes()))
        path = target
    rows, report = read_mcap_windows(path, [(1200000000, 1400000000)], meter=ReadMeter(ReadBudget()))
    assert report["status"] == "COMPLETE", report
    assert [r["value"]["data"] for r in rows] == ["reason2", "reason3", "reason4"]
    assert report["actual"]["source_bytes"] > 0
    assert report["actual"]["temporary_bytes"] == 0
    _, limited = read_mcap_windows(path, [(0, 3000000000)], meter=ReadMeter(ReadBudget(max_source_bytes=8)))
    assert limited["status"] == "BUDGET_EXCEEDED"


def test_identity_mismatch_before_raw_and_output(tmp_path: Path) -> None:
    root, prior = tmp_path / "data", tmp_path / "prior"
    root.mkdir(); prior.mkdir()
    for p in (root / "manifest.yaml", tmp_path / "split", prior / "audit_manifest.json", prior / "anchor_audit_ledger.csv"):
        p.write_text("wrong")
    with pytest.raises(ValueError, match="identity mismatch"):
        run_evidence_audit(root=root, split=tmp_path / "split", previous=prior, output=tmp_path / "out", repo=ROOT,
                           config=EvidenceConfig(), budget=ReadBudget())
    assert not (tmp_path / "out").exists()


def test_source_protection_and_immutable_output(tmp_path: Path) -> None:
    args = dict(root=tmp_path / "data", split=tmp_path / "split", previous=tmp_path / "prior", repo=ROOT,
                config=EvidenceConfig(), budget=ReadBudget())
    with pytest.raises(ValueError, match="overlaps"):
        run_evidence_audit(**args, output=tmp_path / "data/out")
    output = tmp_path / "out"
    output.mkdir()
    with pytest.raises(FileExistsError):
        run_evidence_audit(**args, output=output)


def test_end_to_end_dry_partial_identity_and_source_immutability(tmp_path: Path) -> None:
    from aic_transfuser_lite.data.spatial_evidence_v4 import FIELDS
    from aic_transfuser_lite.data.spatial_coverage_v4 import csv_rows
    root, prior, raw = tmp_path / "data", tmp_path / "prior", tmp_path / "raw/run/bag"
    (root / "trajectories").mkdir(parents=True)
    prior.mkdir(); raw.mkdir(parents=True)
    (raw / "broken.mcap").write_bytes(b"not an MCAP file")
    (raw / "metadata.yaml").write_text(yaml.safe_dump({"rosbag2_bagfile_information": {
        "relative_file_paths": ["broken.mcap"], "topics_with_message_count": []}}))
    rows = [sample(f"s{i}", f"r{i}") for i in range(3)]
    for row in rows:
        np.save(root / row["trajectory_path"], future())
    write_csv(root / "samples.csv", rows, list(rows[0]))
    runs = [dict(run_id=r["run_id"], source_hash=r["run_id"], source_uri=raw.as_uri()) for r in rows]
    write_csv(root / "runs.csv", runs, list(runs[0]))
    manifest = dict(complete=True, schema_version="aic_canonical_dataset_v3", runs=runs,
        files=[dict(path=p.relative_to(root).as_posix(), sha256=sha256_file(p))
               for p in sorted(root.rglob("*")) if p.is_file()])
    digest = identity(manifest)
    (root / "manifest.yaml").write_text(yaml.safe_dump({**manifest, "manifest_sha256": digest}))
    split = tmp_path / "split.json"
    split.write_text(json.dumps(dict(dataset_manifest_sha256=digest,
        assignments=[dict(run_id=f"r{i}", split=s) for i, s in enumerate(("train", "val", "test"))])))
    ledger = [{**{k: "" for k in FIELDS}, **old(r["sample_id"], r["run_id"]),
               "split": s, "source_uri": raw.as_uri(), "source_hash": r["run_id"],
               "h15_raw_arc_m": ".75", "h30_raw_arc_m": "1.5"}
              for r, s in zip(rows, ("train", "val", "test"))]
    write_csv(prior / "anchor_audit_ledger.csv", ledger, list(FIELDS))
    (prior / "audit_manifest.json").write_text(json.dumps({"repository": {"head": PREVIOUS_IMPLEMENTATION}}))
    (prior / "source_inventory.json").write_text("[]")
    expected = {"dataset_identity": digest, **{k: sha256_file(p) for k, p in {
        "dataset_manifest": root / "manifest.yaml", "split": split,
        "previous_manifest": prior / "audit_manifest.json", "previous_ledger": prior / "anchor_audit_ledger.csv"}.items()}}
    before = {p: sha256_file(p) for directory in (root, prior, raw) for p in directory.rglob("*") if p.is_file()}
    args = dict(root=root, split=split, previous=prior, repo=ROOT, expected=expected,
                config=EvidenceConfig(), budget=ReadBudget())
    a = run_evidence_audit(**args, output=tmp_path / "dry1")
    b = run_evidence_audit(**args, output=tmp_path / "dry2")
    assert a["status"] == b["status"] == "DRY_RUN"
    assert a["raw_actual"]["source_bytes"] == 0
    assert a["source_reader_statuses"] == {"NOT_INSPECTED": 1}
    for name in ("selection.json", "anchor_evidence.json", "all_anchor_status.csv", "raw_read_plan.json"):
        assert sha256_file(tmp_path / "dry1" / name) == sha256_file(tmp_path / "dry2" / name)
    result = run_evidence_audit(**args, output=tmp_path / "actual", execute_raw=True,
                                approved_plan=tmp_path / "dry1/raw_read_plan.json")
    assert result["status"] == "PARTIAL"
    assert result["tiers"] == {"OBSERVED_ONLY": 1}
    assert result["source_reader_statuses"] == {"BLOCKED": 1}
    assert result["val_stopped_commanded_tracked"] == 1
    states = csv_rows(tmp_path / "actual/all_anchor_status.csv")
    assert states[-1]["additional_inspection"] == "NOT_INSPECTED"
    assert before == {p: sha256_file(p) for p in before}


def test_invalid_zstd_is_recorded_not_unhandled(tmp_path: Path) -> None:
    path = tmp_path / "bad.mcap.zstd"
    path.write_bytes(b"not zstandard")
    rows, report = read_mcap_windows(path, [(0, 1)], meter=ReadMeter(ReadBudget()))
    assert rows == []
    assert report["status"] == "BLOCKED"
    assert "ZstdError" in report["reason"]


def test_reference_shape_mtime_not_a_route_or_permission_validator() -> None:
    metadata = {**old(), "reference_mtime": "same", "reference_shape_matches": "True"}
    result = evidence_for_anchor(future(), sample(), metadata, records(), True, EvidenceConfig())
    assert result["path_supervision"]["status"] == "UNKNOWN"
    assert result["context"]["driving_permission"]["status"] == "UNKNOWN"
