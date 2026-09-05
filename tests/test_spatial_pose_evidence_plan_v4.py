"""Synthetic saved JSON only. No bag, dataset, optimizer, or ROS fixtures."""
from __future__ import annotations

import ast
from collections import Counter
from copy import deepcopy
from dataclasses import replace
import hashlib
import json
from pathlib import Path

import pytest

from aic_transfuser_lite.data import spatial_pose_evidence_plan_v4 as p

REPO = Path(__file__).resolve().parents[1]


def fixture() -> dict:
    run, other = p.NORMAL_RUNS
    raw = {run: [], other: []}
    definitions = [(p.POSE, 1_000_000_000, .01, "NONZERO_DIFFERENCE_UNCALIBRATED"),
                   (p.POSE, 1_100_000_000, 9.9e-9, "NONZERO_DIFFERENCE_UNCALIBRATED"),
                   (p.POSE, 1_200_000_000, 0., "PROJECTED_GEOMETRY_EQUAL"),
                   (p.VELOCITY, 1_000_000_000, 0., "SINGLE_OBSERVED_CANDIDATE"),
                   (p.VELOCITY, 1_200_000_000, 0., "SINGLE_OBSERVED_CANDIDATE"),
                   (p.POSE, 8_000_000_000, .13, "NONZERO_DIFFERENCE_UNCALIBRATED")]
    for index, (topic, stamp, _, _) in enumerate(definitions):
        for candidate in range(2 if topic == p.POSE else 1):
            raw[run].append({"topic": topic, "semantic_stamp_ns": stamp, "bag_stamp_ns": stamp,
                "source_id": "opaque-source", "type": "nav_msgs/msg/Odometry" if topic == p.POSE else "velocity",
                "payload_sha256": p.identity([index, candidate]), "value": {"saved": candidate}})
    raw[run].append({"topic": "/clock", "semantic_stamp_ns": 1_050_000_000, "bag_stamp_ns": 1_050_000_000,
        "source_id": "opaque-source", "type": "clock", "payload_sha256": p.identity("clock"), "value": {"clock_ns": 1_050_000_000}})
    # This is a stored diagnostic fixture, not geometric computation or classification.
    raw_hash = hashlib.sha256(json.dumps(raw).encode()).hexdigest()
    groups = []
    for index, (topic, stamp, xy, label) in enumerate(definitions):
        candidates = [(i, r) for i, r in enumerate(raw[run]) if r["topic"] == topic and r["semantic_stamp_ns"] == stamp]
        groups.append({"group_id": p.identity(["group", index]), "run_id": run, "topic": topic,
            "semantic_stamp_ns": stamp, "candidate_ids": [p.identity([raw_hash, run, i]) for i, _ in candidates],
            "candidate_count": len(candidates), "candidate_set_identity": p.identity(sorted(p.identity(r) for _, r in candidates)),
            "payload_hash_set": sorted(r["payload_sha256"] for _, r in candidates), "bag_stamp_set_ns": [stamp],
            "source_id_set": ["opaque-source"], "classification": [label], "observed_nonzero_difference": xy > 0,
            "all_pair_maxima": {"xy_m": xy, "yaw_rad": xy} if topic == p.POSE else {},
            "domain_identity": {"status": "UNKNOWN"}, "order_identity": {"status": "UNKNOWN"}})

    def dep(indices: list[int]) -> dict:
        endpoints = []
        for index in indices:
            g = groups[index]
            rid = g["candidate_ids"][-1]
            row = next(r for i, r in enumerate(raw[run]) if p.identity([raw_hash, run, i]) == rid)
            endpoints.append({"stamp_ns": g["semantic_stamp_ns"], "reported_payload_sha256": row["payload_sha256"],
                "candidate_group_ids": [g["group_id"]], "all_candidate_ids": g["candidate_ids"], "matching_record_ids": [rid]})
        return {"endpoints": endpoints, "status": "UNKNOWN", "observed_difference": any(groups[i]["observed_nonzero_difference"] for i in indices)}

    anchor = {"sample_id": "synthetic_anchor", "run_id": run, "processing": "PROCESSED", "original_tier": "OBSERVED_ONLY",
        "anchor_pose_dependency": dep([0, 1]),
        "steps": [{"step": n, "pose_dependency": dep([1, 2]), "velocity_dependency": dep([3, 4])} for n in (1, 2)],
        "independent_numeric_reproduction": {"status": "NOT_INSPECTED"},
        "scopes": {h: {"existing_strict_prefix": {"target_steps": [1, 2]}, "spatial_support": {"support_kind": "KNOWN_PREFIX", "support_m": .2}}
                   for h in ("h15", "h30")}}
    old = {"format": "spatial_evidence_v4_v1", "code_commit": p.OLD_COMMIT, "dataset_identity": p.identity("dataset"),
        "selected_anchor_count": 1, "raw_actual": {"decoded_messages": len(raw[run])}, "status": "PARTIAL", "tiers": {"OBSERVED_ONLY": 1},
        "command": ["MUST_NEVER_EXECUTE"], "source_path": "/never/open/raw.mcap"}
    new = {"format": "spatial_pose_conflict_v4_v1", "reclassification_commit": p.CONFLICT_COMMIT, "old_execution_commit": p.OLD_COMMIT,
        "old_dataset_identity_reported_not_dataset_read": old["dataset_identity"], "status": "COMPLETE_DECLARED_SCOPE", "input_unchanged": True, "input_files": []}
    result = {"conflict/execution_manifest.json": new, "evidence/execution_manifest.json": old,
        "conflict/pose_stamp_groups.json": {"groups": groups}, "conflict/anchor_prefix_impact.json": [anchor],
        "conflict/summary.json": {"status": "COMPLETE_DECLARED_SCOPE", "record_count": len(raw[run]), "anchor_count": 1,
            "group_count": len(groups), "topic_counts": dict(Counter(r["topic"] for r in raw[run]))},
        "evidence/raw_window_evidence.json": raw, "evidence/anchor_evidence.json": [{"sample_id": "synthetic_anchor", "run_id": run, "tier": "OBSERVED_ONLY"}],
        "evidence/selection.json": {"selected": [{"sample_id": "synthetic_anchor"}]},
        "evidence/raw_read_report.json": {"files": [{"run_id": r, "status": "COMPLETE", "mode": "indexed",
            "source_id": "opaque-source", "returned_messages": len(raw[r]), "path": "/never/open/raw.mcap"} for r in raw]},
        "_hashes": {"evidence/raw_window_evidence.json": raw_hash}}
    return result


def save_fixture(tmp_path: Path, data: dict | None = None) -> tuple:
    data = fixture() if data is None else data
    roots = {name: tmp_path / name for name in ("conflict", "evidence")}
    for root in roots.values():
        root.mkdir()
    hashes = {}
    for key in p.EXPECTED:
        namespace, name = key.split("/")
        payload = json.dumps(data[key]).encode()
        (roots[namespace] / name).write_bytes(payload)
        hashes[key] = hashlib.sha256(payload).hexdigest()
    assert hashes["evidence/raw_window_evidence.json"] == data["_hashes"]["evidence/raw_window_evidence.json"]
    return roots["conflict"], roots["evidence"], hashes


def test_deterministic_selection_merges_roles_and_keeps_global_max_scope() -> None:
    data = fixture()
    before = deepcopy(data)
    result = p.build_plan(data, p.Limits())
    assert result == p.build_plan(data, p.Limits()) and data == before
    seeds = result["proposal"]["seeds"]
    assert len(seeds) == 3
    assert any(set(s["roles"]) >= {"anchor_endpoint_nonzero", "large_strict_prefix_dependency"} for s in seeds)
    assert result["proposal"]["global_saved_maximum"]["depends_on_any_selected_strict_prefix"] is False


def test_absent_roles_do_not_invent_controls() -> None:
    data = fixture()
    groups, anchors, records = p.validate_inputs(data, p.Limits())
    groups = [g for g in groups if g["observed_nonzero_difference"]]
    selected = p.select_seeds(groups, p.group_references(anchors), records, p.Limits())
    assert selected["role_status"]["projected_equal_or_saved_singleton_control"]["status"] == "ABSENT"


def test_seed_cap_does_not_truncate_closure() -> None:
    result = p.build_plan(fixture(), replace(p.Limits(), max_seeds=1, closure_groups=1))["proposal"]
    assert len(result["seeds"]) == 1
    assert any(s["status"] == "BLOCKED_SEED_LIMIT" for s in result["role_status"].values())
    c = result["closures"][0]
    assert c["status"] == "CLAIM_CLOSURE_BLOCKED" and len(c["required_group_ids"]) == 5
    assert c["no_truncation"] is True and c["prefix_proved"] is False


def test_anchor_and_right_endpoints_are_causal_dependencies() -> None:
    data = fixture()
    groups = data["conflict/pose_stamp_groups.json"]["groups"]
    refs = p.group_references(data["conflict/anchor_prefix_impact.json"])
    assert any(r["affects_all_future"] for r in refs[groups[0]["group_id"]])
    right_refs = refs[groups[2]["group_id"]]
    assert {r["step"] for r in right_refs} == {1, 2}
    result = p.build_plan(data, p.Limits())["proposal"]
    for c in result["closures"]:
        if c.get("kind") == "FULL_SAVED_PREFIX_B_C":
            assert {g["group_id"] for g in groups[:5]} <= set(c["required_group_ids"])
            assert len(c["endpoint_dependencies"]) == 10


@pytest.mark.parametrize("kind,value", [("KNOWN_ZERO", 0), ("UNKNOWN_FIRST_FUTURE_MISSING", None)])
def test_unknown_known_zero_and_uninspected_preserved(kind: str, value: float | None) -> None:
    data = fixture()
    a = data["conflict/anchor_prefix_impact.json"][0]
    a["scopes"]["h30"]["spatial_support"] = {"support_kind": kind, "support_m": value}
    a["scopes"]["h30"]["existing_strict_prefix"]["target_steps"] = []
    cs = p.build_plan(data, p.Limits())["proposal"]["closures"]
    c = next(c for c in cs if c.get("horizon") == "h30")
    assert c["saved_support"] == {"support_kind": kind, "support_m": value}
    assert c["independent_numeric_reproduction"]["status"] == "NOT_INSPECTED"
    assert c["status"] == "CLAIM_CLOSURE_BLOCKED_NO_POSITIVE_SAVED_PREFIX"


def test_local_clock_never_proves_domain_or_alias_absence() -> None:
    result = p.build_plan(fixture(), p.Limits())["proposal"]
    c = result["closures"][0]
    assert c["saved_clock_records"]
    assert c["selection_policy_candidate_scope"]["whole_run_epoch_alias_absence"] == "UNKNOWN"
    assert c["complete_source_closure_status"].startswith("CLAIM_CLOSURE_BLOCKED")
    assert result["seeds"][0]["domain_status_reported"]["status"] == "UNKNOWN"
    assert any(binding[1] == "/clock" for binding in c["source_schema_requirements"])
    items = p.acquisition_items(c)
    assert items[0]["target_clock_record_ids"] and items[0]["target_group_ids"]


def test_claim_alternatives_and_unknown_cost_not_zero() -> None:
    claims = p.claim_requirements()
    assert "order not necessary" in str(claims["C"])
    assert "physical-last is not AnyReader-last" in str(claims["B"])
    result = p.build_plan(fixture(), p.Limits())["proposal"]
    assert all(v["estimate"] is None and v["authorized"] is False for v in result["future_budget_proposals"].values())
    assert result["future_budget_proposals"]["temporary_disk_bytes"]["proposed_limit"] == 0
    for key, value in p.FLAGS.items():
        assert result[key] == value


def test_allowlist_only_and_embedded_commands_never_followed(tmp_path: Path, monkeypatch) -> None:
    conflict, evidence, hashes = save_fixture(tmp_path)
    decoy = evidence / "outside_allowlist.mcap"
    decoy.write_text("do not open")
    opened = []
    original = Path.open

    def tracked(path, mode="r", *args, **kwargs):
        if "r" in mode:
            opened.append(path)
            assert path != decoy and "never" not in str(path)
        return original(path, mode, *args, **kwargs)

    monkeypatch.setattr(Path, "open", tracked)
    first = p.run_plan(conflict, evidence, tmp_path / "out1", REPO, expected=hashes)
    second = p.run_plan(conflict, evidence, tmp_path / "out2", REPO, expected=hashes)
    assert first["status"] == "COMPLETE_PLAN_ONLY"
    assert first["logical_plan_identity"] == second["logical_plan_identity"]
    assert first["input_unchanged"] is True
    assert all(path.name in {k.split("/")[1] for k in p.EXPECTED} |
               {"spatial_pose_evidence_plan_v4.py", "plan_spatial_pose_evidence_v4.py"} for path in opened)
    unresolved = json.loads((tmp_path / "out1/unresolved_and_unrecoverable.json").read_text())
    for field in ("sequence", "publish_time", "publisher_id", "channel_id"):
        assert field in unresolved["not_recorded_branches"]


@pytest.mark.parametrize("case", ["missing", "hash", "schema", "nested_schema", "count", "tier", "commit", "group_join", "endpoint", "partial", "limit"])
def test_invalid_inputs_block_or_partial(tmp_path: Path, case: str) -> None:
    data = fixture()
    if case == "schema":
        data["conflict/pose_stamp_groups.json"] = []
    if case == "nested_schema":
        data["conflict/pose_stamp_groups.json"]["groups"][0]["all_pair_maxima"] = None
    if case == "count":
        data["conflict/summary.json"]["record_count"] += 1
    if case == "tier":
        data["evidence/anchor_evidence.json"][0]["tier"] = "OTHER"
    if case == "commit":
        data["evidence/execution_manifest.json"]["code_commit"] = "bad"
    if case == "group_join":
        data["conflict/pose_stamp_groups.json"]["groups"][0]["candidate_ids"] = ["bad", "other"]
    if case == "endpoint":
        data["conflict/anchor_prefix_impact.json"][0]["anchor_pose_dependency"]["endpoints"][0]["matching_record_ids"] = []
    if case == "partial":
        data["conflict/execution_manifest.json"]["status"] = "PARTIAL"
    conflict, evidence, hashes = save_fixture(tmp_path, data)
    if case == "missing":
        (evidence / "selection.json").unlink()
    if case == "hash":
        hashes["conflict/summary.json"] = "0" * 64
    limits = replace(p.Limits(), max_records=1) if case == "limit" else p.Limits()
    result = p.run_plan(conflict, evidence, tmp_path / "out", REPO, limits, expected=hashes)
    assert result["status"] == ("PARTIAL" if case == "limit" else "BLOCKED")
    assert result["raw_execution_authorized"] is False
    assert json.loads((tmp_path / "out/minimal_read_proposal.json").read_text())["seeds"] == []


def test_size_limit_has_partial_artifact_not_uncaught_error(tmp_path: Path) -> None:
    c, e, hashes = save_fixture(tmp_path)
    result = p.run_plan(c, e, tmp_path / "out", REPO, replace(p.Limits(), max_total_bytes=1), expected=hashes)
    assert result["status"] == "PARTIAL"


def test_existing_output_and_containment_rejected(tmp_path: Path) -> None:
    c, e, hashes = save_fixture(tmp_path)
    for out in (c, c / "nested", tmp_path):
        with pytest.raises(ValueError, match="containment"):
            p.run_plan(c, e, out, REPO, expected=hashes)
    out = tmp_path / "existing"
    out.mkdir()
    with pytest.raises(FileExistsError):
        p.run_plan(c, e, out, REPO, expected=hashes)


def test_symlink_leaf_is_not_read(tmp_path: Path) -> None:
    c, e, hashes = save_fixture(tmp_path)
    path = e / "selection.json"
    target = tmp_path / "target.json"
    path.rename(target)
    path.symlink_to(target)
    result = p.run_plan(c, e, tmp_path / "out", REPO, expected=hashes)
    assert result["status"] == "BLOCKED"


def test_changed_input_invalidates_already_built_plan(tmp_path: Path, monkeypatch) -> None:
    c, e, hashes = save_fixture(tmp_path)
    original = p.build_plan

    def changed(inputs, limits):
        result = original(inputs, limits)
        (e / "selection.json").write_text("{}")
        return result

    monkeypatch.setattr(p, "build_plan", changed)
    result = p.run_plan(c, e, tmp_path / "out", REPO, expected=hashes)
    assert result["status"] == "BLOCKED" and result["input_unchanged"] is False
    assert json.loads((tmp_path / "out/minimal_read_proposal.json").read_text())["seeds"] == []


@pytest.mark.parametrize("value", [-1, 0, True, 1.5])
def test_invalid_limits(value) -> None:
    with pytest.raises(ValueError):
        replace(p.Limits(), max_records=value).validate()


def test_no_forbidden_imports_or_execution_cli_options() -> None:
    source = Path(p.__file__).read_text(encoding="utf-8")
    tree = ast.parse(source)
    imports = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imports.extend(n.name for n in node.names)
        if isinstance(node, ast.ImportFrom):
            imports.append(node.module or "")
    assert not any(any(token in name for token in ("torch", "rosbags", "training", "reader", "conflict_v4", "dataset")) for name in imports)
    cli = (REPO / "tools/plan_spatial_pose_evidence_v4.py").read_text()
    assert all(option not in cli for option in ("--execute", "--execute-raw", "--dataset-root"))


def test_duplicate_keys_and_nonfinite_JSON_rejected() -> None:
    with pytest.raises(ValueError):
        json.loads('{"a": 1, "a": 2}', object_pairs_hook=p.unique_object)
    with pytest.raises(ValueError):
        json.loads('{"a": NaN}', parse_constant=p.reject_constant)


def test_anchor_role_uses_own_anchor_even_when_earlier_future_depends_on_it() -> None:
    data = fixture()
    a = data["conflict/anchor_prefix_impact.json"][0]
    earlier = deepcopy(a)
    earlier["sample_id"] = "aaa_earlier"
    earlier["anchor_pose_dependency"] = deepcopy(a["steps"][0]["pose_dependency"])
    earlier["steps"][0]["pose_dependency"] = deepcopy(a["anchor_pose_dependency"])
    data["conflict/anchor_prefix_impact.json"].append(earlier)
    data["evidence/anchor_evidence.json"].append({"sample_id": earlier["sample_id"], "run_id": earlier["run_id"], "tier": "OBSERVED_ONLY"})
    data["evidence/selection.json"]["selected"].append({"sample_id": earlier["sample_id"]})
    data["conflict/summary.json"]["anchor_count"] = 2
    data["evidence/execution_manifest.json"]["selected_anchor_count"] = 2
    result = p.build_plan(data, p.Limits())["proposal"]
    seed = next(s for s in result["seeds"] if "anchor_endpoint_nonzero" in s["roles"])
    c = next(c for c in result["closures"] if c["claim_id"] == "partial_probe:" + seed["group_id"])
    assert c["sample_id"] == "synthetic_anchor"


def test_union_caps_separate_from_per_claim_caps() -> None:
    proposal = p.build_plan(fixture(), replace(p.Limits(), union_groups=1))["proposal"]
    assert proposal["probe_union"]["status"] == "CLAIM_CLOSURE_BLOCKED"
    assert len(proposal["probe_union"]["required_group_ids"]) > 1
    assert any(c["status"] == "BOUNDED_PROPOSAL_PENDING_DOMAIN_AND_COST" for c in proposal["closures"])


def test_code_policy_and_inputs_bind_logical_identity(tmp_path: Path) -> None:
    c, e, hashes = save_fixture(tmp_path)
    result = p.run_plan(c, e, tmp_path / "out1", REPO, expected=hashes)
    changed = p.run_plan(c, e, tmp_path / "out2", REPO, replace(p.Limits(), union_groups=63), expected=hashes)
    assert result["logical_plan_identity"] != changed["logical_plan_identity"]
    assert set(result["code_hashes"]) == {"spatial_pose_evidence_plan_v4.py", "plan_spatial_pose_evidence_v4.py"}
