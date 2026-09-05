"""JSON fixtures only: never call a raw reader, dataset loader, or optimizer."""
from __future__ import annotations

from copy import deepcopy
from dataclasses import replace
import json
import math
from pathlib import Path
import subprocess
import sys

import pytest

from aic_transfuser_lite.data.spatial_pose_conflict_v4 import (
    POSE, VELOCITY, OLD_COMMIT, DATASET_ID, ConflictConfig, Budget, analyze,
    build_groups, extraction, identity, inventory_records, run_audit, sha256_file,
)

ROOT = Path(__file__).parents[1]
DOMAIN = dict(source_domain="source1", clock_domain="sim", clock_epoch="epoch1")


def record(t: int = 1000000000, *, x: float = 0, yaw: float = 0,
           payload: str = "a", kind: str = POSE, known: bool = True) -> dict:
    return {"source_id": "fixture_source", "topic": kind,
        "type": "nav_msgs/msg/Odometry" if kind == POSE else "autoware_auto_vehicle_msgs/msg/VelocityReport",
        "bag_stamp_ns": t, "semantic_stamp_ns": t, "timestamp_source": "header.stamp",
        "payload_sha256": identity(payload), **(DOMAIN if known else {}),
        "value": dict(x_m=x, y_m=0., yaw_rad=yaw, frame_id="map", child_frame_id="base_link") if kind == POSE else
                 dict(longitudinal_mps=.5, lateral_mps=0., yaw_rate_rps=0.)}


def groups(rows: list, config: ConflictConfig = ConflictConfig()) -> list:
    return build_groups(inventory_records({"run": rows}, "fixture_file_hash"), Budget(config))[0]


def fixture() -> tuple[dict, list, dict]:
    rows, steps = [], []
    for i in range(31):
        t = 1000000000 + i * 100000000
        p = record(t, x=i * .05, payload=f"p{i}")
        v = record(t, kind=VELOCITY, payload=f"v{i}")
        rows.extend([p, v])
        def endpoint(r: dict) -> dict:
            return dict(target_ns=t, source_stamps_ns=[t, t], source_payload_hashes=[r["payload_sha256"]] * 2, reason="interpolated")
        if i == 0:
            base = endpoint(p)
        else:
            steps.append(dict(step=i, saved_valid=True, status="PASS", pose=endpoint(p), velocity=endpoint(v)))
    for t in (1000000000, 2500000000, 4000000000):
        r = record(t, kind="/clock", payload=f"clock{t}")
        r["value"] = {"clock_ns": t}
        rows.append(r)
    strict = {h: dict(arc_m=n*.05, retained_steps=n, elapsed_sec=n*.1, cut_reason="horizon_limit",
                     reaches={d: n*.05 >= float(d[:-1]) for d in ("0.5m", "1m", "1.5m", "2m")}) for h, n in (("h15", 15), ("h30", 30))}
    anchor = {"sample_id": "anchor", "run_id": "run", "tier": "OBSERVED_ONLY", **DOMAIN,
        "source_reproduction": dict(t_obs_ns=1000000000, status="PASS", anchor_interpolation=base, steps=steps),
        "local_window_boundaries": {"status": "PASS", "flags": []},
        "comparison": dict(strict_diagnostic_v1=strict, raw_v1={h: {"raw_arc_m": v["arc_m"]} for h, v in strict.items()},
                           common_grid_m=[0, .1, .2], common_grid_max_residual_m=0)}
    report = {"files": [dict(run_id="run", mode="indexed", status="COMPLETE", windows_bag_ns=[[0, 5000000000]],
                             source_stat_unchanged=True, decode_errors=[])]}
    return {"run": rows}, [anchor], report


def impacts(raw: dict, anchors: list, report: dict) -> list:
    return analyze(raw, anchors, report, "fixture_file_hash", ConflictConfig())[2]


def test_identical_payload_same_known_domain() -> None:
    r = record()
    g = groups([r, deepcopy(r)])[0]
    assert "BYTE_IDENTICAL_OBSERVED" in g["classification"]
    assert g["observed_projected_equality"]["status"] == "PASS"
    assert g["order_identity"]["status"] == "UNKNOWN"


def test_projected_equal_different_payload_is_not_full_odometry_equal() -> None:
    g = groups([record(), record(payload="b")])[0]
    assert "PROJECTED_GEOMETRY_EQUAL" in g["classification"]
    assert not g["payload_hash_equal"]


@pytest.mark.parametrize("delta,legacy", [(0.999e-8, 0), (1.001e-8, 1), (.13, 1)])
def test_nonzero_differences_do_not_become_noise_pass(delta: float, legacy: int) -> None:
    g = groups([record(), record(x=delta, payload="b")])[0]
    assert "NONZERO_DIFFERENCE_UNCALIBRATED" in g["classification"]
    assert "MATERIAL_DIFFERENCE_EVIDENCED" not in g["classification"]
    assert g["legacy_xy_gt_1e8"]["all_pairs_count"] == legacy
    assert g["physical_noise"]["status"] == "UNKNOWN"


def test_material_requires_explicit_independent_budget() -> None:
    with pytest.raises(ValueError): Budget(ConflictConfig(material_xy_budget_m=.01))
    g = groups([record(), record(x=.13, payload="b")], ConflictConfig(material_xy_budget_m=.01, material_budget_provenance="synthetic declared comparison budget"))[0]
    assert "MATERIAL_DIFFERENCE_EVIDENCED" in g["classification"]


@pytest.mark.parametrize("a,b,equal", [(math.pi, -math.pi, True), (math.pi, -math.pi + 1e-10, False), (0, .1, False)])
def test_yaw_only_wrap_and_small_difference(a: float, b: float, equal: bool) -> None:
    g = groups([record(yaw=a), record(yaw=b, payload="b")])[0]
    assert g["all_pair_maxima"]["xy_m"] == 0
    assert (g["observed_projected_equality"]["status"] == "PASS") == equal


@pytest.mark.parametrize("field", ["frame_id", "child_frame_id", "type"])
def test_frame_type_conflicts_not_hidden_by_grouping(field: str) -> None:
    a, b = record(), record(payload="b")
    (b if field == "type" else b["value"])[field] = "different"
    g = groups([a, b])[0]
    assert "FRAME_OR_TYPE_CONFLICT" in g["classification"]


def test_nonadjacent_three_candidates_all_pair_maxima() -> None:
    a, b, c = record(), record(x=.75e-8, payload="b"), record(x=1.5e-8, payload="c")
    result = groups([a, record(2000000000), b, c])
    g = next(g for g in result if g["candidate_count"] == 3)
    assert g["legacy_xy_gt_1e8"]["array_adjacent_within_group_count"] == 0
    assert g["legacy_xy_gt_1e8"]["all_pairs_count"] == 1
    assert g["all_pair_maxima"]["xy_m"] == 1.5e-8


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), None, "0"])
def test_nonfinite_missing_not_filtered_away(bad: object) -> None:
    a, b = record(), record(payload="b")
    b["value"]["x_m"] = bad
    g = groups([a, b])[0]
    assert "INVALID_OR_INCOMPLETE_RECORD" in g["classification"]
    assert g["all_pair_maxima"] is None
    assert g["observed_projected_equality"]["status"] == "UNKNOWN"


def test_input_permutation_preserves_classification_not_last_choice() -> None:
    a, b = record(), record(x=.13, payload="b")
    first, second = groups([a, b])[0], groups([b, a])[0]
    assert first["candidate_set_identity"] == second["candidate_set_identity"]
    assert first["classification"] == second["classification"]
    assert first["all_pair_maxima"] == second["all_pair_maxima"]
    assert first["same_bag_time_order_sensitive"]
    # The same index denotes a different chosen value after a permutation.
    assert inventory_records({"r": [a, b]}, "hash1")[-1]["record"] != inventory_records({"r": [b, a]}, "hash2")[-1]["record"]


def test_epoch_partition_and_unknown_bucket() -> None:
    a, b = record(), record(x=.13)
    b["clock_epoch"] = "epoch2"
    assert len(groups([a, b])) == 2
    for r in (a, b): r.pop("clock_epoch")
    result = groups([a, b])
    assert len(result) == 1
    assert result[0]["domain_identity"]["status"] == "UNKNOWN"


def test_h30_late_conflict_does_not_fail_h15() -> None:
    raw, anchors, report = fixture()
    raw["run"].append(record(3000000000, x=1.13, payload="duplicate"))
    a = impacts(raw, anchors, report)[0]
    assert a["scopes"]["h15"]["saved_valid_targets"]["dependency"]["status"] == "PASS"
    assert a["scopes"]["h30"]["saved_valid_targets"]["dependency"]["status"] == "FAIL"
    assert a["scopes"]["h30"]["source_consistent_prefix_candidate"]["steps"] == 19


def test_anchor_endpoint_conflict_propagates_all_steps() -> None:
    raw, anchors, report = fixture()
    raw["run"].append(record(x=.13, payload="duplicate"))
    a = impacts(raw, anchors, report)[0]
    assert a["anchor_pose_dependency"]["observed_difference"]
    assert all(s["combined"]["observed_difference"] for s in a["steps"])


def test_right_endpoint_affects_earlier_target() -> None:
    raw, anchors, report = fixture()
    p = anchors[0]["source_reproduction"]["steps"][4]["pose"]
    p.update(source_stamps_ns=[1400000000, 1600000000], source_payload_hashes=[identity("p4"), identity("p6")])
    raw["run"].append(record(1600000000, x=.43, payload="duplicate"))
    assert impacts(raw, anchors, report)[0]["steps"][4]["combined"]["observed_difference"]


def test_other_window_and_missing_domain_not_physical_pass() -> None:
    raw, anchors, report = fixture()
    raw["run"].extend([record(10000000000), record(10000000000, x=.13, payload="other")])
    a = impacts(raw, anchors, report)[0]
    assert not a["scopes"]["h30"]["saved_valid_targets"]["dependency"]["observed_difference"]
    for k in DOMAIN: anchors[0].pop(k)
    assert impacts(raw, anchors, report)[0]["scopes"]["h15"]["saved_valid_targets"]["dependency"]["status"] == "UNKNOWN"


@pytest.mark.parametrize("case", ["first_missing", "known_zero", "known_short"])
def test_support_unknown_zero_short_and_origin_grid(case: str) -> None:
    raw, anchors, report = fixture()
    anchor = anchors[0]
    strict = anchor["comparison"]["strict_diagnostic_v1"]
    if case == "first_missing":
        anchor["source_reproduction"]["steps"][0]["saved_valid"] = False
    for value in strict.values():
        value["arc_m"] = .03 if case == "known_short" else 0
        if case == "first_missing": value.update(retained_steps=0, elapsed_sec=0)
    anchor["comparison"]["common_grid_m"] = [0]
    a = impacts(raw, anchors, report)[0]
    support = a["scopes"]["h15"]["spatial_support"]
    assert support["support_m"] == (None if case == "first_missing" else .03 if case == "known_short" else 0)
    if case == "first_missing": assert all(v is None for v in support["reaches"].values())
    assert a["common_grid_comparison"]["comparability"] == "NOT_COMPARABLE"
    assert a["common_grid_comparison"]["new_residual_m"] is None


@pytest.mark.parametrize("change", ["forward", "stat_false", "decode", "partial", "missing"])
def test_reader_report_not_independent_completeness(change: str) -> None:
    _, _, report = fixture()
    f = report["files"][0]
    if change == "forward": f.update(mode="forward_stream", scan_stop="monotonic_assumption")
    if change == "stat_false": f["source_stat_unchanged"] = False
    if change == "decode": f["decode_errors"] = ["bad"]
    if change == "partial": f["status"] = "PARTIAL"
    files = [] if change == "missing" else [f]
    assert extraction(files, 1000000000, 2000000000)["status"] != "PASS"


def test_velocity_duplicate_and_invalid_do_not_inherit_pose_pass() -> None:
    raw, anchors, report = fixture()
    v = record(2000000000, kind=VELOCITY, payload="v_bad")
    v["value"]["lateral_mps"] = float("nan")
    raw["run"].append(v)
    step = impacts(raw, anchors, report)[0]["steps"][9]
    assert step["pose_dependency"]["status"] == "PASS"
    assert step["velocity_dependency"]["status"] == "UNKNOWN"


def test_no_xy_or_residual_reconstruction() -> None:
    a = impacts(*fixture())[0]
    assert a["original_numeric_reproduction_status"] == "PASS"
    assert a["independent_numeric_reproduction"]["status"] == "NOT_INSPECTED"
    assert a["scopes"]["h30"]["source_consistent_prefix_candidate"]["xy"] is None
    assert a["scopes"]["h30"]["source_consistent_prefix_candidate"]["support_m"] is None


def test_pair_and_record_budgets_keep_tracking() -> None:
    g = groups([record(x=i*.1, payload=str(i)) for i in range(5)], ConflictConfig(max_pairs_per_group=2))[0]
    assert g["all_pair_maxima"] is None
    assert g["pairs_measured"] == 2 and g["all_pairs_total"] == 10
    raw, anchors, reports = fixture()
    _, _, impacts_, summary = analyze(raw, anchors, reports, "hash", ConflictConfig(max_records=1))
    assert len(impacts_) == len(anchors)
    assert summary["status"] == "PARTIAL"
    assert summary["unprocessed_record_count"] == len(raw["run"])


def make_files(tmp_path: Path) -> tuple[Path, dict]:
    root = tmp_path / "inputs"
    root.mkdir()
    raw, anchors, reports = fixture()
    old = dict(code_commit=OLD_COMMIT, dataset_identity=DATASET_ID, selected_anchor_count=1,
               raw_actual={"decoded_messages": len(raw["run"])}, configuration={"position_tolerance_m": 2e-5})
    for name, obj in (("execution_manifest.json", old), ("raw_window_evidence.json", raw),
                      ("anchor_evidence.json", anchors), ("raw_read_report.json", reports)):
        (root / name).write_text(json.dumps(obj), encoding="utf-8")
    hashes = {name: sha256_file(root / name) for name in ("execution_manifest.json", "raw_window_evidence.json", "anchor_evidence.json")}
    return root, hashes


def test_allowlist_immutable_hash_determinism_and_counts(tmp_path: Path) -> None:
    root, hashes = make_files(tmp_path)
    (root / "forbidden_dataset_path.txt").write_text("must not open")
    before = {p: sha256_file(p) for p in root.iterdir()}
    args = dict(evidence_root=root, repo=ROOT, config=ConflictConfig(), expected_hashes=hashes, expected_count=1)
    a = run_audit(**args, output=tmp_path / "out1")
    b = run_audit(**args, output=tmp_path / "out2")
    assert a["status"] == b["status"] == "COMPLETE_DECLARED_SCOPE"
    assert a["logical_identity"] == b["logical_identity"]
    assert all(e["name"].endswith(".json") for e in a["input_files"])
    assert before == {p: sha256_file(p) for p in before}
    for name in ("pose_stamp_groups.json", "anchor_prefix_impact.json", "input_inventory.json"):
        assert sha256_file(tmp_path / "out1" / name) == sha256_file(tmp_path / "out2" / name)
    with pytest.raises(FileExistsError): run_audit(**args, output=tmp_path / "out1")
    with pytest.raises(ValueError): run_audit(**args, output=root / "out")
    (root / "anchor_evidence.json").write_text("[]")
    result = run_audit(**args, output=tmp_path / "bad")
    assert result["status"] == "BLOCKED" and result["exit_code"] == 3


def test_missing_size_limit_invalid_json_and_cli(tmp_path: Path) -> None:
    root, hashes = make_files(tmp_path)
    args = dict(evidence_root=root, repo=ROOT, expected_hashes=hashes, expected_count=1)
    limited = run_audit(**args, config=ConflictConfig(max_file_bytes=10), output=tmp_path / "limited")
    assert limited["status"] == "PARTIAL" and limited["exit_code"] == 2
    (root / "raw_read_report.json").unlink()
    missing = run_audit(**args, config=ConflictConfig(), output=tmp_path / "missing")
    assert missing["status"] == "BLOCKED"
    result = subprocess.run([sys.executable, str(ROOT / "tools/audit_spatial_pose_conflicts_v4.py"),
        "--evidence-root", str(root), "--output", str(tmp_path / "cli")], capture_output=True)
    assert result.returncode == 3


def test_no_forbidden_imports() -> None:
    result = subprocess.run([sys.executable, "-c",
        "import sys;sys.path.insert(0,'src');import aic_transfuser_lite.data.spatial_pose_conflict_v4;"
        "assert not any(k.startswith(('torch','rosbags','aic_transfuser_lite.training')) for k in sys.modules)"], cwd=ROOT)
    assert result.returncode == 0


def test_absent_recovery_payload_is_not_zero_conflict() -> None:
    raw, anchors, report = fixture()
    raw["run"] = []
    _, _, a, summary = analyze(raw, anchors, report, "hash", ConflictConfig())
    assert a[0]["anchor_pose_dependency"]["observed_difference"] is None
    assert a[0]["scopes"]["h30"]["saved_valid_targets"]["dependency"]["observed_difference"] is None
    assert summary["legacy_count_definitions_by_run"][0]["pose_duplicate_groups"] is None


def test_invalid_record_shape_and_missing_pose_field_partial() -> None:
    raw, anchors, report = fixture()
    raw["run"].append(None)
    del raw["run"][0]["value"]["yaw_rad"]
    _, _, a, summary = analyze(raw, anchors, report, "hash", ConflictConfig())
    assert summary["status"] == "PARTIAL"
    assert summary["invalid_record_count"] == 2
    assert a[0]["anchor_pose_dependency"]["status"] == "UNKNOWN"


def test_saved_json_nonfinite_output_is_explicit_and_invalid(tmp_path: Path) -> None:
    root, hashes = make_files(tmp_path)
    p = root / "raw_window_evidence.json"
    data = json.loads(p.read_text())
    data["run"][0]["value"]["x_m"] = float("nan")
    p.write_text(json.dumps(data))
    hashes["raw_window_evidence.json"] = sha256_file(p)
    result = run_audit(root, tmp_path / "out", ROOT, ConflictConfig(), expected_hashes=hashes, expected_count=1)
    assert result["status"] == "PARTIAL"
    inv = json.loads((tmp_path / "out/input_inventory.json").read_text())
    assert inv["records"][0]["record"]["value"]["x_m"] == {"invalid_number": "nan"}


def test_malformed_json_blocks_and_does_not_follow_paths(tmp_path: Path) -> None:
    root, hashes = make_files(tmp_path)
    (root / "raw_read_report.json").write_text('{"files": NaN, "files": []}')
    result = run_audit(root, tmp_path / "out", ROOT, ConflictConfig(), expected_hashes=hashes, expected_count=1)
    assert result["status"] == "BLOCKED"


def test_anchor_limit_preserves_ids_and_retained_inconsistency_partial() -> None:
    raw, anchors, report = fixture()
    anchors.append({**deepcopy(anchors[0]), "sample_id": "second"})
    _, _, a, summary = analyze(raw, anchors, report, "hash", ConflictConfig(max_anchors=1))
    assert [r["sample_id"] for r in a] == ["anchor", "second"]
    assert a[1]["processing"] == "NOT_INSPECTED_LIMIT"
    assert summary["status"] == "PARTIAL"
    anchors[0]["comparison"]["strict_diagnostic_v1"]["h15"]["elapsed_sec"] = 99
    assert analyze(raw, anchors[:1], report, "hash", ConflictConfig())[3]["status"] == "PARTIAL"
