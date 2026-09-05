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
    prior = tmp_path / "prior"
    prior.mkdir()
    base = fixture()
    result = p.build_plan(base, p.Limits())
    # Prior artifacts are synthetic, not obtained by executing any old reader.
    claims = p.legacy_claim_requirements()
    entries = [{"name": key, "sha256": value, "unchanged_after": True} for key, value in hashes.items()]
    manifest = {"format": "spatial_pose_evidence_plan_v4_v1", "status": "COMPLETE_PLAN_ONLY",
        "input_unchanged": True, "input_files": entries, "limits": p.asdict(p.Limits()),
        "code_hashes": {"synthetic_old_code": p.identity("old")}, "facts": result["facts"]}
    manifest["logical_plan_identity"] = p.identity({"policy": manifest["format"], "status": manifest["status"],
        "limits": manifest["limits"], "input_hashes": hashes, "code_hashes": manifest["code_hashes"], "result": result, "claims": claims})
    values = {"execution_manifest.json": manifest, "minimal_read_proposal.json": result["proposal"],
        "claim_requirements.json": claims, "input_manifest.json": {"files": entries},
        "unresolved_and_unrecoverable.json": {}, "report_ja.md": "synthetic prior report"}
    for name, value in values.items():
        (prior / name).write_text(value if name.endswith(".md") else json.dumps(value), encoding="utf-8")
    return roots["conflict"], roots["evidence"], hashes


@pytest.fixture(autouse=True)
def supply_explicit_synthetic_prior(monkeypatch):
    """Existing I/O regressions use explicitly bound synthetic prior-six artifacts."""
    original = p.run_plan

    def wrapped(c, e, out, repo, *args, **kwargs):
        if "prior_plan_root" not in kwargs:
            root = c.parent / "prior"
            payloads = {name: (root / name).read_bytes() for name in p.PRIOR_EXPECTED}
            m = json.loads(payloads["execution_manifest.json"])
            proposal = json.loads(payloads["minimal_read_proposal.json"])
            kwargs.update(prior_plan_root=root, prior_expected={name: hashlib.sha256(b).hexdigest() for name, b in payloads.items()},
                prior_identity=m["logical_plan_identity"],
                fixed_seeds={s["group_id"]: s["candidates"][0]["semantic_stamp_ns"] for s in proposal["seeds"]})
        return original(c, e, out, repo, *args, **kwargs)

    monkeypatch.setattr(p, "run_plan", wrapped)


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
               set(p.PRIOR_EXPECTED) | {"spatial_pose_evidence_plan_v4.py", "plan_spatial_pose_evidence_v4.py"} for path in opened)
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
    original = p.revise_plan

    def changed(*args, **kwargs):
        result = original(*args, **kwargs)
        (e / "selection.json").write_text("{}")
        return result

    monkeypatch.setattr(p, "revise_plan", changed)
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


def revision_inputs(tmp_path: Path) -> tuple:
    c, e, hashes = save_fixture(tmp_path)
    data = fixture()
    data["_hashes"] = hashes
    prior = {name: (tmp_path / "prior" / name).read_text() if name.endswith(".md") else
             json.loads((tmp_path / "prior" / name).read_text()) for name in p.PRIOR_EXPECTED}
    pid = prior["execution_manifest.json"]["logical_plan_identity"]
    seeds = {s["group_id"]: s["candidates"][0]["semantic_stamp_ns"] for s in prior["minimal_read_proposal.json"]["seeds"]}
    return data, prior, pid, seeds


def test_revised_pairs_preserve_seeds_roles_records_and_legacy(tmp_path: Path) -> None:
    data, prior, pid, seeds = revision_inputs(tmp_path)
    before = deepcopy(prior)
    result = p.revise_plan(data, prior, p.Limits(), p.Deadline(60), pid, seeds)
    plan = result["proposal"]
    assert prior == before
    assert plan["prior_logical_identity_recomputed"] == pid
    assert plan["seeds"] == prior["minimal_read_proposal.json"]["seeds"]
    assert plan["legacy_claims"] == prior["minimal_read_proposal.json"]["closures"]
    for claim in plan["closures"]:
        assert claim["closure_kind"] == "listed_record_binding"
        assert len(claim["required_group_ids"]) == 1 and len(claim["required_candidates"]) == 2
        assert claim["required_clock_records"] == claim["required_anchor_velocity_endpoints"] == []
        assert claim["weaker_separate_claim"] is True and "replaces_claim" not in claim
        assert claim["existence_binding"] == claim["unique_occurrence_binding"] == "NOT_EXECUTED"
    assert result == p.revise_plan(data, prior, p.Limits(), p.Deadline(60), pid, seeds)


@pytest.mark.parametrize("gid,stamp", list(p.FIXED_SEEDS.items()))
def test_fixed_four_pair_windows_are_exact_inclusive_ns(gid: str, stamp: int) -> None:
    source = fixture()
    old = p.build_plan(source, p.Limits())["proposal"]["seeds"][0]
    seed = deepcopy(old)
    seed["group_id"] = gid
    for r in seed["candidates"]:
        r["bag_stamp_ns"] = r["semantic_stamp_ns"] = stamp
    claim = p.pair_claim(seed, {"status": "SOURCE_LOCATOR_UNRESOLVED"}, p.Limits())
    assert claim["search_window_log_time_ns_inclusive"] == [stamp - 250_000_000, stamp + 250_000_000]
    assert claim["future_api_interval_contract"]["exclusive_stop_ns"] == stamp + 250_000_001
    assert claim["physical_chunk_count_estimate"] is None


def test_claim_specific_predicates_and_counterexample_direction() -> None:
    c = p.claim_requirements()["typed_claims"]
    listed = str(c["C_LISTED_PAIR_PROJECTION"]["necessary_conditions"])
    assert all(word not in listed for word in ("order", "AnyReader", "budget", "complete"))
    assert "counterexample" in str(c["C_COMPLETE_CANDIDATE_INVARIANCE"]["necessary_conditions"])
    assert "any_of" in str(c["B_STREAM_REPLAY"])
    assert "unique_occurrence" in c["B_RECORD_BINDING"]["binding_outcomes"]


def test_old_blocked_relative_probe_stays_blocked(tmp_path: Path) -> None:
    data, prior, _, seeds = revision_inputs(tmp_path)
    old = prior["minimal_read_proposal.json"]
    old["closures"][0]["status"] = "CLAIM_CLOSURE_BLOCKED"
    old["closures"][0]["exceeded_caps"] = [{"dimension": "window_ns", "required": 2924999946, "proposed_cap": 1000000000}]
    m = prior["execution_manifest.json"]
    pid = p.identity({"policy": m["format"], "status": m["status"], "limits": m["limits"], "input_hashes": data["_hashes"],
        "code_hashes": m["code_hashes"], "result": {"proposal": old, "facts": m["facts"]}, "claims": prior["claim_requirements.json"]})
    m["logical_plan_identity"] = pid
    result = p.revise_plan(data, prior, p.Limits(), p.Deadline(60), pid, seeds)
    assert result["proposal"]["legacy_claims"][0] == old["closures"][0]
    assert result["proposal"]["closures"][0]["exceeded_caps"] == []


def test_replay_missing_contract_not_required_for_binding(tmp_path: Path) -> None:
    data, prior, pid, seeds = revision_inputs(tmp_path)
    result = p.revise_plan(data, prior, p.Limits(), p.Deadline(60), pid, seeds)
    contract = next(c["replay_contract"] for c in result["proposal"]["legacy_replay_contracts"] if c["replay_contract"]["status"] == "MISSING_REPLAY_CONTRACT")
    assert contract["t_obs_ns"] is None and "t_obs_ns_and_provenance" in contract["missing_predicates"]
    assert all(r["intervening_records_complete"] == "UNKNOWN" for r in contract["rows"])
    assert contract["does_not_block_listed_record_binding"] is True


def test_empty_endpoint_vacuity_rejected_but_explicit_unknown_kept() -> None:
    data = fixture()
    e = data["conflict/anchor_prefix_impact.json"][0]["anchor_pose_dependency"]["endpoints"][0]
    for k in ("candidate_group_ids", "all_candidate_ids", "matching_record_ids"):
        e[k] = []
    with pytest.raises(ValueError, match="empty endpoint"):
        p.validate_inputs(data, p.Limits())
    e["resolution_status"] = "UNKNOWN"
    p.validate_inputs(data, p.Limits())


def test_h15_step16_rejected() -> None:
    data = fixture()
    a = data["conflict/anchor_prefix_impact.json"][0]
    a["steps"] = [{**deepcopy(a["steps"][0]), "step": i} for i in range(1, 17)]
    a["scopes"]["h15"]["existing_strict_prefix"]["target_steps"] = list(range(1, 17))
    with pytest.raises(ValueError, match="horizon"):
        p.validate_inputs(data, p.Limits())


def test_removed_original_endpoint_is_not_replay_complete() -> None:
    data = fixture()
    a = data["conflict/anchor_prefix_impact.json"][0]
    old = {"source_reproduction": {"t_obs_ns": 1, "t_obs_source": "stored", "anchor_interpolation": {
        "target_ns": 1, "source_stamps_ns": [1, 2], "source_payload_hashes": [p.identity(1), p.identity(2)]}}}
    a["anchor_pose_dependency"]["endpoints"] = []
    with pytest.raises(ValueError, match="removed/changed"):
        p.replay_contract(a, old, [1], data, p.Deadline(60))


def test_shared_stage_envelope_and_no_automatic_payload(tmp_path: Path) -> None:
    data, prior, pid, seeds = revision_inputs(tmp_path)
    result = p.revise_plan(data, prior, p.Limits(), p.Deadline(60), pid, seeds)
    approval = result["approval"]
    assert approval["automatic_stage_transition"] is False
    assert {s["envelope_ref"] for s in approval["stages"].values()} == {approval["envelope"]["envelope_id"]}
    assert all(s["authorized"] is False for s in approval["stages"].values())
    assert approval["stages"]["S1"]["automatic_fallback"] is False
    assert all(b["estimate"] is None for b in approval["envelope"]["limits"].values())
    assert approval["envelope"]["limits"]["chunks"]["proposed_limit"] == 8
    assert approval["physical_chunk_offsets"] is None


def test_git_failure_returns_empty_blocked_scope(tmp_path: Path, monkeypatch) -> None:
    c, e, hashes = save_fixture(tmp_path)
    def fail(*args, **kwargs):
        raise p.subprocess.CalledProcessError(1, "git")
    monkeypatch.setattr(p.subprocess, "check_output", fail)
    result = p.run_plan(c, e, tmp_path / "out", REPO, expected=hashes)
    assert result["status"] == "BLOCKED"
    assert json.loads((tmp_path / "out/approval_request.json").read_text())["claim_ids"] == []


def test_deadline_interrupts_analysis_with_partial_scope(tmp_path: Path, monkeypatch) -> None:
    c, e, hashes = save_fixture(tmp_path)
    def fail(self):
        raise ValueError("LIMIT: cooperative analysis deadline")
    monkeypatch.setattr(p.Deadline, "check", fail)
    result = p.run_plan(c, e, tmp_path / "out", REPO, expected=hashes)
    assert result["status"] == "PARTIAL"
    assert json.loads((tmp_path / "out/approval_request.json").read_text())["sources"] == []


@pytest.mark.parametrize("failed_name", ["minimal_read_proposal.json", "execution_manifest.pending.json"])
def test_mid_output_failure_cannot_publish_complete_manifest(tmp_path: Path, monkeypatch, failed_name: str) -> None:
    c, e, hashes = save_fixture(tmp_path)
    original = p.write_json
    def fail(path, value):
        if path.name == failed_name:
            raise OSError("synthetic disk error")
        return original(path, value)
    monkeypatch.setattr(p, "write_json", fail)
    result = p.run_plan(c, e, tmp_path / "out", REPO, expected=hashes)
    assert result["status"] == "BLOCKED"
    assert not (tmp_path / "out/execution_manifest.json").exists()
    assert json.loads((tmp_path / "out/error_manifest.json").read_text())["application_scope"] == []


def test_unwritable_error_manifest_reports_stderr(tmp_path: Path, monkeypatch, capsys) -> None:
    c, e, hashes = save_fixture(tmp_path)
    def fail(*args):
        raise OSError("unwritable")
    monkeypatch.setattr(p, "write_json", fail)
    result = p.run_plan(c, e, tmp_path / "out", REPO, expected=hashes)
    assert result["status"] == "BLOCKED"
    assert "error-manifest write failed" in capsys.readouterr().err


def test_prior_canonical_identity_tamper_rejected(tmp_path: Path) -> None:
    data, prior, pid, _ = revision_inputs(tmp_path)
    prior["minimal_read_proposal.json"]["seeds"][0]["roles"].append("injected")
    with pytest.raises(ValueError, match="prior logical identity"):
        p.verify_prior(prior, data, pid)


def test_six_known_zero_prefixes_are_not_positive_or_missing() -> None:
    rows = []
    for kind, count, status, steps in (("KNOWN_ZERO", 6, "BOUNDED_PROPOSAL_PENDING_DOMAIN_AND_COST", [1, 2, 3, 4]),
            ("KNOWN_PREFIX", 2, "CLAIM_CLOSURE_BLOCKED", [1, 2]),
            ("UNKNOWN_FIRST_FUTURE_MISSING", 2, "CLAIM_CLOSURE_BLOCKED_NO_POSITIVE_SAVED_PREFIX", [])):
        rows.extend({"kind": "FULL_SAVED_PREFIX_B_C", "status": status, "target_steps": steps,
                     "saved_support": {"support_kind": kind}} for _ in range(count))
    summary = p.support_summary(rows, [])
    assert summary["all_legacy_support_kinds"] == {"KNOWN_ZERO": 6, "KNOWN_PREFIX": 2, "UNKNOWN_FIRST_FUTURE_MISSING": 2}
    assert summary["within_observed_caps_support_kinds"] == {"KNOWN_ZERO": 6}
    assert summary["within_caps_retained_steps"] == {"4": 6}


def test_all_four_preserved_seed_records_with_distinct_pair_claims(tmp_path: Path) -> None:
    data, prior, _, fixed = revision_inputs(tmp_path)
    groups, anchors, records = p.validate_inputs(data, p.Limits())
    old = prior["minimal_read_proposal.json"]
    fourth = groups[5]
    extra = {"group_id": fourth["group_id"], "run_id": fourth["run_id"], "roles": ["large_strict_prefix_dependency"],
        "candidates": [p.record_binding(r, records) for r in sorted(fourth["candidate_ids"])],
        "saved_maxima": fourth["all_pair_maxima"], "related_anchor_steps": []}
    for seed in old["seeds"]:
        seed["roles"] = [r for r in seed["roles"] if r != "large_strict_prefix_dependency"]
    old["seeds"].append(extra)
    fixed[fourth["group_id"]] = fourth["semantic_stamp_ns"]
    c = p.closure("partial_probe:" + fourth["group_id"], {fourth["group_id"]}, [], fourth["run_id"],
                  {g["group_id"]: g for g in groups}, records, p.Limits())
    c.update(kind="DISTINCT_PARTIAL_PROBE_B_C", sample_id=None, target_steps=[])
    old["closures"].append(c)
    m = prior["execution_manifest.json"]
    pid = p.identity({"policy": m["format"], "status": m["status"], "limits": m["limits"], "input_hashes": data["_hashes"],
        "code_hashes": m["code_hashes"], "result": {"proposal": old, "facts": m["facts"]}, "claims": prior["claim_requirements.json"]})
    m["logical_plan_identity"] = pid
    revised = p.revise_plan(data, prior, p.Limits(), p.Deadline(60), pid, fixed)["proposal"]
    assert revised["seeds"] == old["seeds"] and revised["legacy_claims"] == old["closures"]
    assert len(revised["closures"]) == 4 and len(revised["union"]["record_ids"]) == 8
    assert {c["claim_id"] for c in revised["closures"]} == {"record_pair_binding:" + gid for gid in fixed}


def test_code_hash_failure_does_not_escape(tmp_path: Path, monkeypatch) -> None:
    c, e, hashes = save_fixture(tmp_path)
    original = p.bounded_read
    def fail(path, cap):
        if path.suffix == ".py":
            raise OSError("code hash unavailable")
        return original(path, cap)
    monkeypatch.setattr(p, "bounded_read", fail)
    result = p.run_plan(c, e, tmp_path / "out", REPO, expected=hashes)
    assert result["status"] == "BLOCKED"
    assert json.loads((tmp_path / "out/approval_request.json").read_text())["windows"] == []


def test_binding_cap_failure_not_cleared_by_union(tmp_path: Path) -> None:
    data, prior, pid, seeds = revision_inputs(tmp_path)
    plan = p.revise_plan(data, prior, p.Limits(closure_candidates=1), p.Deadline(60), pid, seeds)["proposal"]
    assert plan["union"]["status"] == "WITHIN_LISTED_CAPS_UNAPPROVED"
    assert all(c["status"] == "CLAIM_CLOSURE_BLOCKED" for c in plan["closures"])


def test_source_locator_does_not_even_stat_or_resolve_raw(monkeypatch) -> None:
    seed = {"run_id": p.NORMAL_RUNS[0], "candidates": [{"run_id": p.NORMAL_RUNS[0],
        "source_id": "example.mcap:metadata:" + "a" * 64}]}
    report = {"run_id": seed["run_id"], "source_id": seed["candidates"][0]["source_id"],
              "path": "/opaque/" + seed["run_id"] + "/example.mcap"}
    def forbidden(*args, **kwargs):
        raise AssertionError("source filesystem operation forbidden")
    for name in ("open", "stat", "lstat", "resolve", "exists"):
        monkeypatch.setattr(Path, name, forbidden)
    locator = p.source_locator(seed, [report])
    assert locator["status"] == "SAVED_LOCATOR_BOUND_NOT_SOURCE_INSPECTED"
    assert locator["source_full_sha256"] is None


def test_deadline_is_checked_inside_record_validation_loop() -> None:
    class CountingDeadline:
        calls = 0
        def check(self):
            self.calls += 1
            if self.calls == 3:
                raise ValueError("LIMIT: cooperative analysis deadline")
    deadline = CountingDeadline()
    with pytest.raises(ValueError, match="deadline"):
        p.validate_inputs(fixture(), p.Limits(), deadline)
    assert deadline.calls == 3
