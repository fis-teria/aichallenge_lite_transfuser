"""Saved-JSON-only evidence planning. No raw reader, classifier, or teacher generation.

Input shapes: groups [G], impacts [A] with steps [<=30], extracted records [R].
Timestamps are integer ns; saved diagnostic XY/yaw differences are m/rad.
Embedded paths and commands are opaque and never passed to filesystem/process APIs.
"""
from __future__ import annotations

from collections import Counter
from copy import deepcopy
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import stat
import subprocess
import sys
import time
from typing import Any, Mapping

VERSION = "spatial_pose_evidence_plan_v4_v2_record_binding"
PRIOR_IDENTITY = "394292c7c268715a5efc4cd408f0e9634835d5d4cf016729a01c9f4786dde71d"
PRIOR_EXPECTED = {
    "execution_manifest.json": "c94a3e6bb6330ae6abef547d3ee856e43098caddfed4833964a97b1b00ebee29",
    "minimal_read_proposal.json": "de16bdd46787a97fbbb61107a299b035b8d3a91704779366b4db7111da2eee25",
    "claim_requirements.json": "18d13e2060d976693215aef02d40b8b97719d19cea143b6f943caeb5d07c0342",
    "input_manifest.json": "77b03e6e1517e14b5de2a58e6ac845ea94255f85667a49e37fe694eeae1f5fd6",
    "unresolved_and_unrecoverable.json": "6f5afd399f499844fe539ae17bb3f4e490dd0b09e683b30b7d6173b90246a8ad",
    "report_ja.md": "5a9f766bf67960fdaed8a0a39e49eaba5cce89e737d9addcb4fa8872d87cc64b",
}
FIXED_SEEDS = {
    "8fbd120c37e872d2bc51ab58bc95813636caa7de04a335b560e9337c6993f12b": 6189999861,
    "208cfcac87744ded9ef39f3c85ac2ae6d3f545255e81239e33e8819e1947b20c": 256259994272,
    "353301a96a1f493e253091d78bef0c43c2b90818097aa892e392a5756250811d": 5449999878,
    "ffe514be1a0f7756c75006e4066566bcaf025610e726962ed1ecb9c93fca65f7": 939999978,
}
OLD_COMMIT = "7ae8298b71aa7bdbacf1d1757798bf0de66bcfc4"
CONFLICT_COMMIT = "befc97dd98434843245b888fb6d1d7110acc8a93"
NORMAL_RUNS = ("20260902-131505", "20260902-132822")
POSE = "/localization/kinematic_state"
VELOCITY = "/vehicle/status/velocity_status"
EXPECTED = {
    "conflict/execution_manifest.json": "2a305ef80f4a9ae979644ff77d75fc8010b7b4e01db26ac4bfcd02b680255e3f",
    "conflict/pose_stamp_groups.json": "c5734787fbd8a7bb14038c601398ac5ed098514e1cf9349d55a94cf26aa63f92",
    "conflict/anchor_prefix_impact.json": "12a3115b46fe4de310b6bc8cfc1de9e64a358890cde607592917890415c11e00",
    "conflict/summary.json": "da224114867d9d827212be5d179329564b759ba9dd3a8ba74d92c28427e1ef22",
    "evidence/execution_manifest.json": "4261af73079fdaafa7b2478b94447ab904fbd9bfcd61b7ed8529bd0c449d340b",
    "evidence/raw_window_evidence.json": "97d2ccd5992aacf7c20b894f30afc03de283ded61dd4e15cda3ea2b28d853d75",
    "evidence/anchor_evidence.json": "98399b988b6c69fd7c47668f91d1388a62d46f3cb03d7a1e442ea8582db5c7df",
    "evidence/raw_read_report.json": "e3da319b45ba62dbbfdea16dc2efa051c05c96010b5ba50d98f8e9a3e732f401",
    "evidence/selection.json": "d2518ac717712ff2c8860b053114f6a478a2a1c8c757884dcee8d58ac3560295",
}
FLAGS = {"mode": "PLAN_ONLY", "raw_execution_authorized": False,
         "raw_reads_performed": 0, "dataset_reads_performed": 0,
         "deployment_or_training_approved": False,
         "approval_gate": "PENDING_EXPLICIT_AUTHORIZATION"}


@dataclass(frozen=True)
class Limits:
    max_file_bytes: int = 16 * 1024**2
    max_total_bytes: int = 32 * 1024**2
    max_records: int = 10000
    max_groups: int = 2000
    max_anchors: int = 64
    max_steps: int = 30
    max_seeds: int = 4
    closure_groups: int = 32
    closure_candidates: int = 64
    closure_clock_records: int = 256
    closure_window_ns: int = 1_000_000_000
    union_groups: int = 64
    union_candidates: int = 128
    max_seconds: int = 60

    def validate(self) -> None:
        if any(type(v) is not int or v <= 0 for v in asdict(self).values()):
            raise ValueError("limits must be positive integers")
        if self.max_seeds > 4 or self.max_steps > 30:
            raise ValueError("seed/step hard cap exceeded")


def identity(value: Any) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, ensure_ascii=False,
                                    separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


class Deadline:
    """Cooperative analysis deadline, not preemption of JSON parse or OS I/O."""

    def __init__(self, seconds: int):
        self.end = time.monotonic() + seconds

    def check(self) -> None:
        require(time.monotonic() <= self.end, "LIMIT: cooperative analysis deadline")


def is_hash(value: Any) -> bool:
    return isinstance(value, str) and len(value) == 64 and all(c in "0123456789abcdef" for c in value)


def is_stamp(value: Any) -> bool:
    return type(value) is int and 0 <= value < 2**63


def dependencies(anchor: dict, steps: list[int]) -> list[tuple[str, int | None, dict]]:
    """Anchor endpoints affect every future; retain both left AND right endpoints."""
    result = [("anchor_pose", None, anchor["anchor_pose_dependency"])]
    for step in anchor["steps"]:
        if step["step"] in steps:
            result.extend((kind, step["step"], step[kind + "_dependency"])
                          for kind in ("pose", "velocity"))
    return result


def validate_inputs(inputs: dict, limits: Limits, deadline: Deadline | None = None) -> tuple[list, list, dict]:
    """Validate identities/joins; consume saved classifications, never recompute them."""
    new = inputs["conflict/execution_manifest.json"]
    old = inputs["evidence/execution_manifest.json"]
    summary = inputs["conflict/summary.json"]
    groups = inputs["conflict/pose_stamp_groups.json"]["groups"]
    anchors = inputs["conflict/anchor_prefix_impact.json"]
    raw = inputs["evidence/raw_window_evidence.json"]
    old_anchors = inputs["evidence/anchor_evidence.json"]
    selected = inputs["evidence/selection.json"]["selected"]
    require(new["reclassification_commit"] == CONFLICT_COMMIT and
            new["old_execution_commit"] == old["code_commit"] == OLD_COMMIT, "execution commit mismatch")
    require(new["old_dataset_identity_reported_not_dataset_read"] == old["dataset_identity"] and
            is_hash(old["dataset_identity"]), "reported Dataset identity mismatch")
    require(new["format"] == "spatial_pose_conflict_v4_v1" and
            old["format"] == "spatial_evidence_v4_v1", "format mismatch")
    require(new["status"] == summary["status"] == "COMPLETE_DECLARED_SCOPE", "prior conflict scope incomplete")
    require(old["status"] in ("PARTIAL", "COMPLETE"), "old evidence status invalid")
    require(new["input_unchanged"] is True, "prior input integrity unresolved")
    require(isinstance(groups, list) and isinstance(anchors, list) and isinstance(raw, dict), "root shapes")
    require(len(groups) <= limits.max_groups and len(anchors) <= limits.max_anchors, "LIMIT: groups/anchors")
    require(all(isinstance(rows, list) for rows in raw.values()), "raw JSON rows shape")
    count = sum(len(rows) for rows in raw.values())
    require(count <= limits.max_records, "LIMIT: records")
    require(count == summary["record_count"] == old["raw_actual"]["decoded_messages"], "record count mismatch")
    require(len(groups) == summary["group_count"], "group count mismatch")
    require(len(anchors) == summary["anchor_count"] == old["selected_anchor_count"], "anchor count mismatch")
    ids = [a["sample_id"] for a in anchors]
    require(all(isinstance(i, str) and i for i in ids) and len(set(ids)) == len(ids), "anchor identity")
    require(set(ids) == {a["sample_id"] for a in old_anchors} == {a["sample_id"] for a in selected}
            and len(ids) == len(old_anchors) == len(selected), "anchor join mismatch")
    raw_hash = inputs["_hashes"]["evidence/raw_window_evidence.json"]
    records = {}
    for run, rows in raw.items():
        for index, row in enumerate(rows):
            if deadline: deadline.check()
            require(isinstance(row, dict) and is_hash(row["payload_sha256"]) and
                    is_stamp(row["bag_stamp_ns"]) and is_stamp(row["semantic_stamp_ns"]), "record schema")
            require(all(isinstance(row[k], str) and row[k] for k in ("topic", "type", "source_id")), "record binding")
            require(isinstance(row["value"], dict), "record value shape")
            records[identity([raw_hash, run, index])] = {"run_id": run, **row}
    lookup = {g["group_id"]: g for g in groups}
    require(len(lookup) == len(groups), "duplicate group IDs")
    for g in groups:
        if deadline: deadline.check()
        require(is_hash(g["group_id"]) and is_stamp(g["semantic_stamp_ns"]), "group ID/stamp")
        require(isinstance(g["classification"], list) and all(isinstance(x, str) for x in g["classification"]), "classification schema")
        require(type(g["observed_nonzero_difference"]) is bool, "saved difference schema")
        candidates = g["candidate_ids"]
        require(len(candidates) == g["candidate_count"] > 0 and len(set(candidates)) == len(candidates), "candidate count")
        rows = [records[c] for c in candidates]
        require(all(r["run_id"] == g["run_id"] and r["topic"] == g["topic"] and
                    r["semantic_stamp_ns"] == g["semantic_stamp_ns"] for r in rows), "candidate join mismatch")
        require(sorted({r["payload_sha256"] for r in rows}) == g["payload_hash_set"] and
                sorted({r["bag_stamp_ns"] for r in rows}) == g["bag_stamp_set_ns"] and
                sorted({r["source_id"] for r in rows}) == g["source_id_set"], "candidate binding mismatch")
        original = [{k: v for k, v in r.items() if k != "run_id"} for r in rows]
        require(identity(sorted(identity(r) for r in original)) == g["candidate_set_identity"], "candidate set hash")
        for v in g["all_pair_maxima"].values():
            require(type(v) in (int, float) and math.isfinite(v) and v >= 0, "metric units/finite")
        if g["topic"] == POSE and g["candidate_count"] > 1:
            require(set(g["all_pair_maxima"]) == {"xy_m", "yaw_rad"}, "pose metric shape")
    previous = {a["sample_id"]: a for a in old_anchors}
    for a in anchors:
        if deadline: deadline.check()
        require(a["processing"] == "PROCESSED" and a["run_id"] == previous[a["sample_id"]]["run_id"], "anchor processing/run")
        require(a["original_tier"] == previous[a["sample_id"]]["tier"], "tier join mismatch")
        step_ids = [s["step"] for s in a["steps"]]
        require(len(step_ids) <= limits.max_steps and step_ids == list(range(1, len(step_ids) + 1)), "step shape/order")
        for h in ("h15", "h30"):
            scope = a["scopes"][h]
            ts = scope["existing_strict_prefix"]["target_steps"]
            require(isinstance(ts, list) and ts == list(range(1, len(ts) + 1)) and set(ts) <= set(step_ids)
                    and len(ts) <= int(h[1:]), "prefix step shape/horizon")
            require(scope["spatial_support"]["support_kind"] in
                    ("KNOWN_PREFIX", "KNOWN_ZERO", "UNKNOWN_FIRST_FUTURE_MISSING"), "support knownness")
        for kind, _, dep in dependencies(a, step_ids):
            require(isinstance(dep["endpoints"], list) and len(dep["endpoints"]) <= 2, "endpoint shape")
            for endpoint in dep["endpoints"]:
                require(is_stamp(endpoint["stamp_ns"]) and is_hash(endpoint["reported_payload_sha256"]), "endpoint identity")
                if not endpoint["candidate_group_ids"]:
                    require(endpoint.get("resolution_status") in ("UNKNOWN", "NOT_INSPECTED") and
                            not endpoint["all_candidate_ids"] and not endpoint["matching_record_ids"],
                            "empty endpoint references without explicit unresolved status")
                    continue
                expected_ids = set()
                for gid in endpoint["candidate_group_ids"]:
                    g = lookup[gid]
                    require(g["run_id"] == a["run_id"] and g["semantic_stamp_ns"] == endpoint["stamp_ns"] and
                            g["topic"] == (VELOCITY if kind == "velocity" else POSE), "endpoint group join")
                    expected_ids.update(g["candidate_ids"])
                require(expected_ids == set(endpoint["all_candidate_ids"]), "endpoint candidate scope")
                require(bool(endpoint["matching_record_ids"]) or endpoint.get("resolution_status") in
                        ("UNKNOWN", "NOT_INSPECTED"), "missing endpoint match without explicit unresolved status")
                require(set(endpoint["matching_record_ids"]) ==
                        {i for i in expected_ids if records[i]["payload_sha256"] == endpoint["reported_payload_sha256"]}, "endpoint hash join")
    require(dict(Counter(r["topic"] for r in records.values())) == summary["topic_counts"], "topic counts mismatch")
    reports = inputs["evidence/raw_read_report.json"]["files"]
    require(isinstance(reports, list) and len({r["run_id"] for r in reports}) == len(reports), "source report shape")
    for run in NORMAL_RUNS:
        r = next((r for r in reports if r["run_id"] == run), None)
        require(r is not None and r["status"] == "COMPLETE" and r["mode"] == "indexed", "normal extraction missing")
        require(r["returned_messages"] == len(raw.get(run, [])), "source report count")
        require(all(row["source_id"] == r["source_id"] for row in raw[run]), "source ID join")
    return groups, anchors, records


def legacy_claim_requirements() -> dict:
    return {
        "A": {"claim": "保存抽出物の候補差", "necessary_predicates": ["input_hash_join", "saved_projection_scope"],
              "alternative_sufficient_evidence": ["verified saved pair diagnostics; no new raw needed"],
              "unresolved": ["not physical error or safety"], "acquisition_can_resolve": []},
        "B": {"claim": "指定policyでの限定記録stream再現", "necessary_predicates": ["source_schema_binding", "bounded_domain_assignment", "complete_candidate_scope", "explicit_selection_policy"],
              "alternative_sufficient_evidence": ["explicit source/epoch assignment OR independently justified bounded partition", "order-independent selection OR verified policy-specific total order"],
              "unresolved": ["old converter requires historical AnyReader version, file/connection enumeration and equal-time tie evidence", "physical-last is not AnyReader-last; current library is not historical proof"],
              "acquisition_can_resolve": ["source/schema/record positions and local boundary hypotheses, conditional on actually recorded data"]},
        "C": {"claim": "候補選択への投影不変性・感度", "necessary_predicates": ["complete_candidate_scope", "compatible_frame_projection", "bounded_domain_assignment"],
              "alternative_sufficient_evidence": ["all relevant projected values equal: order not necessary", "sensitivity quantification needs no physical budget; acceptability is a separate budget predicate"],
              "unresolved": ["observed equality is not candidate completeness", "nonzero difference without calibrated budget", "no new teacher XY"],
              "acquisition_can_resolve": ["candidate set and projection provenance; quaternion only for yaw-projection hypothesis"]},
        "D": {"claim": "物理的に一意・正確なpose", "necessary_predicates": ["independent estimator semantics", "frame/time calibration", "physical validation evidence"],
              "alternative_sufficient_evidence": ["independent calibrated reference plus documented estimator provenance, not necessarily a publisher-ID field"],
              "unresolved": ["record order/channel/covariance alone insufficient", "may be UNRESOLVABLE_FROM_THIS_SOURCE"],
              "acquisition_can_resolve": ["at most provenance leads; B never promotes D"]},
        "E": {"claim": "教師採用・発進・Safety・MPC実行", "necessary_predicates": ["separate supervision policy", "intent/permission", "clearance/Safety", "vehicle/controller/environment validation"],
              "alternative_sufficient_evidence": ["independently approved task-specific gates, never Reference existence or B alone"],
              "unresolved": ["all execution/adoption outside this task"], "acquisition_can_resolve": []},
    }


def group_references(anchors: list) -> dict[str, list]:
    refs: dict[str, list] = {}
    for a in anchors:
        for kind, step, dep in dependencies(a, [s["step"] for s in a["steps"]]):
            for side, e in enumerate(dep["endpoints"]):
                for gid in e["candidate_group_ids"]:
                    refs.setdefault(gid, []).append({"sample_id": a["sample_id"], "kind": kind,
                        "step": step, "endpoint_index": side,
                        "affects_all_future": kind == "anchor_pose",
                        "strict_horizons": [h for h in ("h15", "h30") if
                            (a["scopes"][h]["existing_strict_prefix"]["target_steps"] if step is None else
                             step in a["scopes"][h]["existing_strict_prefix"]["target_steps"])]})
    return {gid: sorted(rows, key=identity) for gid, rows in refs.items()}


def select_seeds(groups: list, refs: dict, records: dict, limits: Limits) -> dict:
    pool = [g for g in groups if g["run_id"] in NORMAL_RUNS and g["topic"] == POSE]
    nonzero = [g for g in pool if g["observed_nonzero_difference"]]
    large = lambda g: (-g["all_pair_maxima"].get("xy_m", 0), g["group_id"])
    roles = {
        "anchor_endpoint_nonzero": sorted([g for g in nonzero if any(r["kind"] == "anchor_pose" for r in refs.get(g["group_id"], []))], key=large),
        "large_strict_prefix_dependency": sorted([g for g in nonzero if any(r["strict_horizons"] for r in refs.get(g["group_id"], []))], key=large),
        "legacy_threshold_neighbour": sorted(nonzero, key=lambda g: (abs(g["all_pair_maxima"].get("xy_m", 0) - 1e-8), g["group_id"])),
        "projected_equal_or_saved_singleton_control": sorted([g for g in pool if "PROJECTED_GEOMETRY_EQUAL" in g["classification"]], key=lambda g: g["group_id"])
            or sorted([g for g in pool if g["candidate_count"] == 1], key=lambda g: g["group_id"]),
    }
    chosen: dict[str, dict] = {}
    role_status = {}
    for role, candidates in roles.items():
        if not candidates:
            role_status[role] = {"status": "ABSENT"}
            continue
        g = candidates[0]
        gid = g["group_id"]
        if gid not in chosen and len(chosen) >= limits.max_seeds:
            role_status[role] = {"status": "BLOCKED_SEED_LIMIT", "candidate_group_id": gid}
            continue
        item = chosen.setdefault(gid, {"group_id": gid, "run_id": g["run_id"], "roles": [],
            "saved_classification": g["classification"], "saved_maxima": g["all_pair_maxima"],
            "domain_status_reported": g["domain_identity"], "order_status_reported": g["order_identity"],
            "candidates": [record_binding(rid, records) for rid in sorted(g["candidate_ids"])],
            "related_anchor_steps": refs.get(gid, [])})
        item["roles"].append(role)
        role_status[role] = {"status": "SELECTED", "group_id": gid}
    maximum = min(nonzero, key=large) if nonzero else None
    return {"seeds": sorted(chosen.values(), key=lambda s: s["group_id"]), "role_status": role_status,
        "selection_policy": "per-role ranked winner; merge identical winners; ties stable group ID; threshold is diagnostic only",
        "global_saved_maximum": None if maximum is None else {"group_id": maximum["group_id"],
            "saved_xy_m": maximum["all_pair_maxima"]["xy_m"],
            "depends_on_any_selected_strict_prefix": any(r["strict_horizons"] for r in refs.get(maximum["group_id"], [])),
            "not_a_target_prefix_error_bound": True},
        "sampling_scope": "biased diagnostic, not Dataset defect rate; no recovery transfer"}


def record_binding(rid: str, records: dict) -> dict:
    r = records[rid]
    return {"record_id": rid, **{k: r[k] for k in ("run_id", "source_id", "topic", "type", "payload_sha256", "bag_stamp_ns", "semantic_stamp_ns")}}


def closure(claim_id: str, gids: set[str], dep_rows: list, run: str, groups: dict,
            records: dict, limits: Limits) -> dict:
    endpoints, missing = [], []
    for kind, step, dep in dep_rows:
        if not dep["endpoints"]:
            missing.append({"kind": kind, "step": step, "status": dep["status"]})
        for e in dep["endpoints"]:
            gids.update(e["candidate_group_ids"])
            endpoints.append({"kind": kind, "step": step, **e})
    candidate_ids = sorted({rid for gid in gids for rid in groups[gid]["candidate_ids"]})
    candidates = [record_binding(rid, records) for rid in candidate_ids]
    times = [c["bag_stamp_ns"] for c in candidates]
    window = [max(0, min(times) - 250_000_000), max(times) + 250_000_000] if times else None
    clock = [record_binding(rid, records) for rid, r in sorted(records.items()) if window and
             r["run_id"] == run and r["topic"] == "/clock" and window[0] <= r["bag_stamp_ns"] <= window[1]]
    over = []
    for name, size, cap in (("groups", len(gids), limits.closure_groups),
                           ("candidates", len(candidates), limits.closure_candidates),
                           ("clock_records", len(clock), limits.closure_clock_records),
                           ("window_ns", window[1] - window[0] if window else 0, limits.closure_window_ns)):
        if size > cap:
            over.append({"dimension": name, "required": size, "proposed_cap": cap})
    return {"claim_id": claim_id, "run_id": run,
        "status": "CLAIM_CLOSURE_BLOCKED" if over or missing else "BOUNDED_PROPOSAL_PENDING_DOMAIN_AND_COST",
        "required_group_ids": sorted(gids), "required_candidates": candidates, "endpoint_dependencies": endpoints,
        "missing_endpoint_evidence": missing, "exceeded_caps": over,
        "local_clock_window_bag_ns": window, "window_margin_ns_proposed": 250_000_000,
        "saved_clock_records": clock, "clock_scope": "local hypothesis only; not independently assigned domain",
        "source_schema_requirements": sorted({(r["source_id"], r["topic"], r["type"]) for r in candidates + clock}),
        "selection_policy_candidate_scope": {"observed_ids_complete_in_saved_JSON": candidate_ids,
            "source_completeness": "UNKNOWN", "whole_run_epoch_alias_absence": "UNKNOWN",
            "required": "all candidates in an independently bound source/domain partition; if not bounded, CLAIM_CLOSURE_BLOCKED, do not expand to whole run"},
        "complete_source_closure_status": "CLAIM_CLOSURE_BLOCKED_PENDING_DOMAIN_PARTITION",
        "no_truncation": True, "prefix_proved": False}


def acquisition_items(c: dict) -> list:
    common = {"claim_id": c["claim_id"], "run_id": c["run_id"],
        "closure_status": c["status"], "target_group_ids": c["required_group_ids"],
        "target_source_schema_bindings": c["source_schema_requirements"],
        "target_clock_record_ids": [r["record_id"] for r in c["saved_clock_records"]],
        "target_record_ids": [r["record_id"] for r in c["required_candidates"]],
        "window_bag_ns": c["local_clock_window_bag_ns"], "endpoint_hashes": sorted({e["reported_payload_sha256"] for e in c["endpoint_dependencies"]}),
        "saved_candidate_payload_hashes": sorted({r["payload_sha256"] for r in c["required_candidates"]}),
        "on_missing_extra_candidate_or_schema_change": "invalidate candidate binding/completeness; preserve old artifacts; record difference and BLOCK dependent claim, no silent replacement",
        "remains_unknown": ["physical correctness D", "teacher/launch/Safety/controller E", "whole-run alias absence"],
        "authorization": "PENDING_EXPLICIT_AUTHORIZATION"}
    definitions = [
        ("source_schema_binding", "metadata/index then particular chunk if definitions absent from summary",
         ["source identity/stat and bounded hashes", "channel_id", "schema_id", "schema encoding/definition hash", "message encoding/topic/frame binding"],
         "saved type/source label is not schema-definition or raw-byte binding", "conditional B/C provenance only"),
        ("policy_specific_record_order", "particular chunk payload plus message/chunk index",
         ["file chunk_start_offset", "uncompressed record offset", "bag/log_time", "header stamp", "payload hash", "sequence and its meaning if useful", "publish_time and its provenance if useful"],
         "saved JSON array position is not physical or historical AnyReader order", "bounded recorded order; old converter only with independent historical version/enumeration/tie evidence"),
        ("bounded_domain_assignment", "particular candidate and local clock chunk payload; metadata if explicit domain exists",
         ["explicit source/clock epoch if recorded OR independently justified local partition", "local clock/header/log mapping and boundaries"],
         "saved clock samples support local consistency, not independent epoch assignment", "local hypothesis or explicit partition only; unbounded alias scope blocks claim"),
    ]
    return [{**common, "predicate": predicate, "future_stage": stage, "additional_fields_or_evidence": fields,
             "why_saved_JSON_insufficient": why, "can_update": update,
             "if_not_recorded": "NOT_RECORDED; use listed alternative evidence if sufficient, otherwise UNRESOLVABLE_FROM_THIS_SOURCE; no retry/expansion"}
            for predicate, stage, fields, why, update in definitions]


def build_plan(inputs: dict, limits: Limits) -> dict:
    """Legacy synthetic regression baseline; production revision never calls this selector."""
    limits.validate()
    groups, anchors, records = validate_inputs(inputs, limits)
    refs = group_references(anchors)
    selection = select_seeds(groups, refs, records, limits)
    lookup = {g["group_id"]: g for g in groups}
    by_anchor = {a["sample_id"]: a for a in anchors}
    closures = []
    for seed in selection["seeds"]:
        related = seed["related_anchor_steps"]
        # A separate single-target probe is never described as full-prefix verification.
        preferred = sorted(related, key=lambda r: (
            not ("anchor_endpoint_nonzero" in seed["roles"] and r["kind"] == "anchor_pose"),
            not bool(r["strict_horizons"]), r["sample_id"], r["step"] or 0))
        if preferred:
            ref = preferred[0]
            a = by_anchor[ref["sample_id"]]
            step = ref["step"] or (a["scopes"]["h30"]["existing_strict_prefix"]["target_steps"] or [s["step"] for s in a["steps"]])[0]
            deps = dependencies(a, [step])
            target = {"sample_id": a["sample_id"], "target_steps": [step],
                      "inside_saved_strict_prefix": step in a["scopes"]["h30"]["existing_strict_prefix"]["target_steps"],
                      "selection_reason": "anchor-role uses its own anchor first, then strict membership, stable anchor ID and earliest step; invalid targets remain diagnostics"}
        else:
            deps, target = [], {"sample_id": None, "target_steps": [], "scope": "seed-only control; no prefix claim"}
        c = closure("partial_probe:" + seed["group_id"], {seed["group_id"]}, deps, seed["run_id"], lookup, records, limits)
        c.update(kind="DISTINCT_PARTIAL_PROBE_B_C", **target)
        closures.append(c)
    related_ids = sorted({r["sample_id"] for s in selection["seeds"] for r in s["related_anchor_steps"]})
    for aid in related_ids:
        a = by_anchor[aid]
        for h in ("h15", "h30"):
            scope = a["scopes"][h]
            steps = scope["existing_strict_prefix"]["target_steps"]
            c = closure("full_saved_prefix:" + aid + ":" + h, set(), dependencies(a, steps), a["run_id"], lookup, records, limits)
            c.update(kind="FULL_SAVED_PREFIX_B_C", sample_id=aid, horizon=h, target_steps=steps,
                     saved_support=scope["spatial_support"], independent_numeric_reproduction=a["independent_numeric_reproduction"],
                     saved_tier=a["original_tier"])
            if not steps:
                c["status"] = "CLAIM_CLOSURE_BLOCKED_NO_POSITIVE_SAVED_PREFIX"
            closures.append(c)
    probes = [c for c in closures if c["kind"] == "DISTINCT_PARTIAL_PROBE_B_C"]
    union_gids = sorted({g for c in probes for g in c["required_group_ids"]})
    union_rids = sorted({r["record_id"] for c in probes for r in c["required_candidates"]})
    proposal = {**FLAGS, **selection, "closure_caps": asdict(limits), "closures": closures,
        "probe_union": {"required_group_ids": union_gids, "required_record_ids": union_rids,
            "status": "CLAIM_CLOSURE_BLOCKED" if len(union_gids) > limits.union_groups or len(union_rids) > limits.union_candidates else "WITHIN_OBSERVED_UNION_CAPS_NOT_SOURCE_COMPLETENESS"},
        "acquisition_items": [item for c in closures for item in acquisition_items(c)],
        "scheduling_scope": "full-prefix closures are alternative claims, not automatically scheduled work; blocked items cannot be acquired under these caps",
        "optional_projection_probe": {"predicate": "yaw projection provenance", "seed_group_ids": [s["group_id"] for s in selection["seeds"] if s["saved_maxima"].get("yaw_rad", 0) > 0],
            "fields": ["original quaternion", "projection convention and schema"], "stage": "same already approved candidate chunk only",
            "if_absent": "UNRESOLVABLE_FROM_THIS_SOURCE", "new_teacher_XY": False},
        "not_requested_without_specific_hypothesis": ["covariance", "publisher metadata beyond available source binding"],
        "future_budget_proposals": {name: {"proposed_limit": value, "estimate": None,
            "reason": "no raw/index inspected; independent stop cap, not a completion cost estimate", "authorized": False}
            for name, value in {"source_bytes": 64 * 1024**2, "expanded_bytes": 128 * 1024**2,
                "messages": 5000, "seconds": 60, "temporary_disk_bytes": 0,
                "single_record_bytes": 16 * 1024**2, "chunks": 8}.items()},
        "budget_policy": "fresh independent approval; no old remaining budget or repaired historical counters; metadata-only cost gate before payload approval",
        "future_reader_preconditions": ["retain consumed bytes on time-limit exception", "persist partial/error manifest and progress",
            "bind schema ID to definition hash", "separate assumed early-stop from verified window completeness",
            "propagate source stat/hash change", "missing topic/file/window never no-conflict PASS",
            "index log_time coverage is not header-stamp coverage", "disclose unverified full raw-byte immutability",
            "no reader implementation in this task"],
        "approval_conditions": ["explicit separate authorization of exact sources/stages/windows/claims and fresh caps",
            "review reader fixes and non-learning tests before acquisition", "metadata cost gate; stop if caps or domain closure insufficient",
            "new immutable output and original hash comparison; no Dataset/training/drive authorization"],
        "gates": {"independent_geometry_only_design": "SPECIFICATION_ONLY_NO_DATA_GENERATION",
            "real_data_adoption": "BLOCKED_SEPARATE_PROVENANCE_AND_SUPERVISION", "stop_teacher": "BLOCKED_INTENT_PERMISSION",
            "controller_mpc_oracle": "BLOCKED_ENVIRONMENT_VEHICLE_POLICY_NOT_IMPLEMENTED_BY_THIS_TASK"}}
    duplicate_pose = [g for g in groups if g["topic"] == POSE and g["candidate_count"] > 1]
    facts = {"verification_scope": "saved classifications/dependencies/counts joined to allowlisted JSON; no geometric remeasurement",
        "record_count": len(records), "anchor_count": len(anchors), "pose_group_count": sum(g["topic"] == POSE for g in groups),
        "duplicate_pose_count": sum(g["topic"] == POSE and g["candidate_count"] > 1 for g in groups),
        "anchor_endpoint_observed_difference_count": sum(a["anchor_pose_dependency"]["observed_difference"] is True for a in anchors),
        "saved_diagnostic_aggregation": {
            "candidate_count_distribution": dict(Counter(str(g["candidate_count"]) for g in duplicate_pose)),
            "classification_counts": dict(Counter(label for g in duplicate_pose for label in g["classification"])),
            "different_payload_groups": sum(len(g["payload_hash_set"]) > 1 for g in duplicate_pose),
            "positive_xy_at_most_1e8": sum(0 < g["all_pair_maxima"]["xy_m"] <= 1e-8 for g in duplicate_pose),
            "positive_yaw": sum(g["all_pair_maxima"]["yaw_rad"] > 0 for g in duplicate_pose),
            "xy_above_20um_not_safety": sum(g["all_pair_maxima"]["xy_m"] > 2e-5 for g in duplicate_pose),
            "max_xy_m": max((g["all_pair_maxima"]["xy_m"] for g in duplicate_pose), default=None),
            "max_yaw_rad": max((g["all_pair_maxima"]["yaw_rad"] for g in duplicate_pose), default=None),
            "strict_prefix_affected_anchors": {h: sum(bool(a["scopes"][h]["existing_strict_prefix"].get("observed_difference_steps", [])) for a in anchors) for h in ("h15", "h30")}},
        "prior_summary_reported": {k: v for k, v in inputs["conflict/summary.json"].items() if k != "elapsed_analysis_sec"},
        "old_tiers_reported": inputs["evidence/execution_manifest.json"].get("tiers"),
        "Dataset_identity_reported_not_body_verified": inputs["evidence/execution_manifest.json"]["dataset_identity"]}
    return {"proposal": proposal, "facts": facts}


def claim_requirements() -> dict:
    """Claim predicates are not interchangeable with a list of field names."""
    legacy = legacy_claim_requirements()
    specs = {
        "A_SAVED_DIFFERENCE": ("saved JSON diagnostics", ["saved_hash_join"],
            ["new raw", "physical accuracy", "safety"]),
        "B_RECORD_BINDING": ("listed record IDs and payload hashes in one explicit source", [
            "explicit_source", "record_locator", "payload_hash_binding",
            {"any_of": ["channel to schema definition/encoding binding", "independently verified equivalent decoding definition binding"]}],
            ["whole-run epoch alias absence", "complete candidate set", "historical AnyReader tie", "physical truth"]),
        "B_STREAM_REPLAY": ("explicit partition and selection policy", [
            {"any_of": ["explicit source/epoch partition", "record-defined partition", "independent interval correspondence"]},
            "candidate completeness in that partition", "explicit selection policy",
            {"any_of": ["order-independent policy", "policy-specific verified order"]}],
            ["physical-last equals historical AnyReader-last", "current version proves historical version", "B_RECORD_BINDING implies replay"]),
        "C_LISTED_PAIR_PROJECTION": ("exactly the two listed record ID/hash bindings", [
            "schema/frame/projection interpretation consistency", "listed pair binding"],
            ["physical order", "historical AnyReader tie", "all-candidate completeness", "physical budget required for numeric sensitivity", "unique physical pose"]),
        "C_COMPLETE_CANDIDATE_INVARIANCE": ("all candidates in explicit comparison partition", [
            {"any_of": ["explicit domain", "record-defined comparison partition", "independent interval correspondence"]},
            "schema/frame/projection consistency",
            {"any_of": ["complete candidate set plus equality for an invariance proof", "non-equal pair in the same comparison domain for a counterexample"]}],
            ["two equal candidates prove complete-set invariance", "local clock monotonicity proves domain"]),
        "D_PHYSICAL_ACCURACY": ("physical estimator claim", ["estimator semantics", "calibration", "independent physical validation"],
            ["record binding or covariance proves accuracy"]),
        "E_GEOMETRY_SUPERVISION": ("teacher adoption", ["independent supervision/adoption policy"], ["automatic B/C promotion"]),
        "E_STOP_LAUNCH_LABELS": ("stop/launch labels", ["intent and permission evidence"], ["geometry implies intent"]),
        "E_MOTION_PERMISSION_SAFETY": ("motion permission and safety", ["independent safety/clearance/permission validation"], ["Reference implies safety"]),
        "E_CONTROLLER_ORACLE": ("controller execution", ["vehicle/environment/controller policy validation"], ["current runtime is already longitudinal/lateral MPC"]),
    }
    typed = {name: {"claim_type": name, "universe": universe, "necessary_conditions": conditions,
                    "non_claims": nonclaims, "status": "NOT_EXECUTED"}
             for name, (universe, conditions, nonclaims) in specs.items()}
    typed["B_RECORD_BINDING"]["binding_outcomes"] = {
        "existence": "one or more matched occurrences; retain every match in the approved scope",
        "unique_occurrence": "requires independent locator/occurrence evidence; duplicate payload hash does not select original occurrence",
        "unscanned_occurrences": "UNKNOWN outside approved scope; no automatic expansion"}
    typed["B_STREAM_REPLAY"]["historical_converter_branch"] = ["historical AnyReader version", "file/connection enumeration",
        "tie rules", "evidence that excluded records cannot change old dedup selection"]
    typed["C_LISTED_PAIR_PROJECTION"]["acceptability"] = "separate calibrated budget predicate, not needed to quantify sensitivity"
    return {**legacy, "typed_claims": typed}


def verify_prior(prior: dict, inputs: dict, expected_identity: str) -> str:
    """Recompute the exact v1 canonical JSON identity, without running old code."""
    m = prior["execution_manifest.json"]
    require(m["format"] == "spatial_pose_evidence_plan_v4_v1", "prior format")
    require(m["status"] == "COMPLETE_PLAN_ONLY" and m["input_unchanged"] is True, "prior scope/integrity")
    hashes = {e["name"]: e["sha256"] for e in m["input_files"]}
    require(hashes == inputs["_hashes"], "prior original-nine-input binding mismatch")
    require(prior["input_manifest.json"]["files"] == m["input_files"], "prior input manifests differ")
    calculated = identity({"policy": m["format"], "limits": m["limits"], "input_hashes": hashes,
        "code_hashes": m["code_hashes"], "result": {"proposal": prior["minimal_read_proposal.json"], "facts": m["facts"]},
        "claims": prior["claim_requirements.json"], "status": m["status"]})
    require(calculated == m["logical_plan_identity"] == expected_identity, "prior logical identity mismatch")
    return calculated


def replay_contract(a: dict, old: dict, selected_steps: list[int], inputs: dict, deadline: Deadline) -> dict:
    """Reference stored interpolation contracts, never infer t_obs from a sample ID."""
    source = old.get("source_reproduction", {})
    if not isinstance(source, dict): source = {}
    missing = []
    t_obs = source.get("t_obs_ns")
    if not is_stamp(t_obs) or not source.get("t_obs_source"):
        missing.append("t_obs_ns_and_provenance")
    old_steps = {s["step"]: s for s in source.get("steps", [])}
    saved_steps = {s["step"]: s for s in a["steps"]}
    rows = []
    for kind, step, dep in dependencies(a, selected_steps):
        deadline.check()
        evidence = source.get("anchor_interpolation", {}) if step is None else old_steps.get(step, {}).get(kind, {})
        if not isinstance(evidence, dict): evidence = {}
        direct = dep.get("original_endpoint_evidence", {})
        if evidence and direct:
            require(evidence == direct, "old anchor interpolation contract mismatch")
        if is_stamp(evidence.get("target_ns")) and step is None and is_stamp(t_obs):
            require(evidence["target_ns"] == t_obs, "anchor target differs from stored t_obs")
        stamps, hashes = evidence.get("source_stamps_ns", []), evidence.get("source_payload_hashes", [])
        require(len(stamps) == len(hashes), "old interpolation endpoint array mismatch")
        if stamps:
            require(stamps == [e["stamp_ns"] for e in dep["endpoints"]] and
                    hashes == [e["reported_payload_sha256"] for e in dep["endpoints"]], "removed/changed original endpoints")
        if not evidence or not is_stamp(evidence.get("target_ns")) or len(stamps) != 2:
            missing.append(f"{kind}:{step}:target_or_endpoints")
        if any(not e["candidate_group_ids"] or not e["matching_record_ids"] for e in dep["endpoints"]):
            missing.append(f"{kind}:{step}:unresolved_endpoint")
        target = evidence.get("target_ns")
        exact = len(stamps) == 2 and stamps[0] == stamps[1]
        roles = []
        for index, endpoint in enumerate(dep["endpoints"]):
            roles.append({"role": "exact_match" if exact else "left" if index == 0 else "right", **endpoint})
        valid = None if step is None else old_steps.get(step, {}).get("saved_valid")
        if step is not None:
            if type(valid) is not bool:
                missing.append(f"step:{step}:saved_valid")
            elif "saved_valid" in saved_steps[step]:
                require(valid == saved_steps[step]["saved_valid"], "saved_valid join")
        rows.append({"kind": kind, "step": step, "target_ns": target, "saved_valid": valid,
            "endpoint_roles": roles, "bracket_kind": "EXACT_MATCH" if exact else "BRACKET" if len(stamps) == 2 else "UNKNOWN",
            "old_reason": evidence.get("reason"),
            "candidate_search_interval_header_ns": [min(stamps), max(stamps)] if len(stamps) == 2 else None,
            "interval_meaning": "must verify no intervening source stamp and nearest endpoints in specified partition; hashes alone insufficient",
            "intervening_records_complete": "UNKNOWN", "schema_frame_domain_binding": "UNKNOWN",
            "clock_frame_boundary_evidence": "MISSING_REPLAY_CONTRACT",
            "provenance": "evidence/anchor_evidence.json:source_reproduction joined to conflict dependency"})
    config = inputs["evidence/execution_manifest.json"].get("configuration", {})
    tolerance = config.get("interpolation_tolerance_ms")
    if type(tolerance) not in (float, int) or not math.isfinite(tolerance) or tolerance <= 0:
        missing.append("interpolation_tolerance_ms")
    missing.extend(["independent partition/source/schema/frame binding", "candidate interval completeness and boundary evidence",
                    "historical AnyReader version/enumeration/tie and excluded-record noninterference"])
    return {"status": "MISSING_REPLAY_CONTRACT", "t_obs_ns": t_obs, "t_obs_provenance": source.get("t_obs_source"),
        "rows": rows, "missing_predicates": sorted(set(missing)), "interpolation_tolerance_ms": tolerance,
        "policy_identity": identity({"execution_commit": OLD_COMMIT, "configuration": config}),
        "policy_identity_scope": "reported extraction policy; not historical AnyReader runtime provenance",
        "does_not_block_listed_record_binding": True}


def source_locator(seed: dict, reports: list) -> dict:
    """Only inspect string contents. Never construct a Path from a source locator."""
    bindings = {(r["run_id"], r["source_id"]) for r in seed["candidates"]}
    report = next((r for r in reports if (r.get("run_id"), r.get("source_id")) in bindings), {})
    text_path, source_id = report.get("path"), report.get("source_id", "")
    parts = source_id.split(":metadata:")
    metadata_hash = parts[1] if len(parts) == 2 and is_hash(parts[1]) else None
    valid = (len(bindings) == 1 and isinstance(text_path, str) and text_path.startswith("/") and
             ".." not in text_path.split("/") and text_path.rsplit("/", 1)[-1] == parts[0] and
             seed["run_id"] in text_path.split("/") and metadata_hash is not None)
    return {"status": "SAVED_LOCATOR_BOUND_NOT_SOURCE_INSPECTED" if valid else "SOURCE_LOCATOR_UNRESOLVED",
        "absolute_path_opaque": text_path, "run_id": seed["run_id"], "source_id": source_id,
        "metadata_sha256_reported": metadata_hash, "source_full_sha256": None,
        "provenance": "evidence/raw_read_report.json literal string + saved candidate source_id",
        "not_claimed": "not full raw hash or publisher ID; path never open/stat/resolve/existence-checked"}


def pair_claim(seed: dict, locator: dict, limits: Limits) -> dict:
    candidates = deepcopy(seed["candidates"])
    require(len(candidates) == 2 and len({c["record_id"] for c in candidates}) == 2, "seed must retain two record IDs")
    times = [r["bag_stamp_ns"] for r in candidates]
    window = [max(0, min(times) - 250_000_000), max(times) + 250_000_000]
    exceeded = [name for name, count, cap in (("groups", 1, limits.closure_groups),
        ("candidates", 2, limits.closure_candidates), ("window_ns", window[1] - window[0], limits.closure_window_ns)) if count > cap]
    requirements = claim_requirements()["typed_claims"]
    return {**FLAGS, "claim_id": "record_pair_binding:" + seed["group_id"], "claim_type": "B_RECORD_BINDING",
        "closure_kind": "listed_record_binding", "group_id": seed["group_id"], "roles": seed["roles"],
        "parent_diagnostic_link": "partial_probe:" + seed["group_id"], "weaker_separate_claim": True,
        "universe": {"source": locator, "listed_records": candidates}, "required_group_ids": [seed["group_id"]],
        "required_candidates": candidates, "required_clock_records": [], "required_anchor_velocity_endpoints": [],
        "search_window_log_time_ns_inclusive": window,
        "future_api_interval_contract": {"inclusive_start_ns": window[0], "exclusive_stop_ns": window[1] + 1,
            "checked_conversion_required": True, "meaning": "integer-ns inclusive window to start-inclusive/stop-exclusive API; not header-domain proof"},
        "status": "CLAIM_CLOSURE_BLOCKED" if exceeded else "SOURCE_LOCATOR_UNRESOLVED" if locator["status"] == "SOURCE_LOCATOR_UNRESOLVED" else "WITHIN_LISTED_CAPS_UNAPPROVED",
        "exceeded_caps": exceeded, "physical_chunk_count_estimate": None,
        "necessary_conditions": requirements["B_RECORD_BINDING"]["necessary_conditions"],
        "non_claims": requirements["B_RECORD_BINDING"]["non_claims"] + ["old partial/full-prefix proof", "relative target replay"],
        "existence_binding": "NOT_EXECUTED", "unique_occurrence_binding": "NOT_EXECUTED",
        "multiple_occurrences_policy": requirements["B_RECORD_BINDING"]["binding_outcomes"],
        "fields_by_predicate": {
            "source_record_binding": ["payload_sha256", "log_time", "header_stamp", "file_chunk_start_offset", "uncompressed_chunk_record_offset"],
            "decoding_binding": ["channel_id", "schema_id", "schema_definition_hash", "schema/message encoding"],
            "informational_only": ["sequence may be zero or recorder-defined", "publish_time may equal log_time", "channel ID is not publisher ID"]},
        "can_update": ["existence of listed source record matches", "occurrence positions and decoding bindings within approved chunks"],
        "missing_candidate_policy": "record NOT_FOUND_IN_APPROVED_SCOPE, no whole-source absence claim or automatic tracking",
        "missing_field_policy": "any_of equivalent evidence if sufficient; else NOT_RECORDED/UNRESOLVABLE_FROM_THIS_SOURCE; stop",
        "projection_claim": {**requirements["C_LISTED_PAIR_PROJECTION"], "claim_id": "listed_pair_projection:" + seed["group_id"],
            "universe": [{"record_id": r["record_id"], "payload_sha256": r["payload_sha256"]} for r in candidates],
            "saved_diagnostic_only": seed["saved_maxima"], "new_projection_computed": False,
            "conditional_counterexample": "non-equal pair disproves invariance only with shared comparison-domain evidence; no domain inferred here"}}


def revise_plan(inputs: dict, prior: dict, limits: Limits, deadline: Deadline,
                expected_identity: str = PRIOR_IDENTITY, fixed_seeds: dict = FIXED_SEEDS) -> dict:
    old_id = verify_prior(prior, inputs, expected_identity)
    groups, anchors, records = validate_inputs(inputs, limits, deadline)
    old_proposal = prior["minimal_read_proposal.json"]
    seeds = deepcopy(old_proposal["seeds"])
    require({s["group_id"] for s in seeds} == set(fixed_seeds) and len(seeds) == len(fixed_seeds) <= 4, "fixed four seed identity mismatch")
    lookup = {g["group_id"]: g for g in groups}
    for seed in seeds:
        deadline.check()
        group = lookup[seed["group_id"]]
        require(group["semantic_stamp_ns"] == fixed_seeds[seed["group_id"]], "fixed seed stamp mismatch")
        require(seed["candidates"] == [record_binding(r, records) for r in sorted(group["candidate_ids"])], "prior seed record/hash binding mismatch")
    by_anchor = {a["sample_id"]: a for a in anchors}
    old_anchors = {a["sample_id"]: a for a in inputs["evidence/anchor_evidence.json"]}
    legacy = deepcopy(old_proposal["closures"])
    supplements = []
    for c in legacy:
        deadline.check()
        aid = c.get("sample_id")
        supplement = {"claim_id": c["claim_id"], "legacy_status_unchanged": c["status"],
            "closure_kind": "full_saved_prefix" if c["kind"] == "FULL_SAVED_PREFIX_B_C" else "interpolation_target",
            "claim_types": ["B_STREAM_REPLAY", "C_COMPLETE_CANDIDATE_INVARIANCE"], "scheduled_for_acquisition": False}
        if aid:
            a = by_anchor[aid]
            steps = c["target_steps"]
            expected_deps = [{"kind": kind, "step": step, **e} for kind, step, dep in dependencies(a, steps) for e in dep["endpoints"]]
            require(c["endpoint_dependencies"] == expected_deps, "prior dependency closure changed")
            if c.get("horizon"):
                require(steps == a["scopes"][c["horizon"]]["existing_strict_prefix"]["target_steps"], "prior prefix steps changed")
            supplement["replay_contract"] = replay_contract(a, old_anchors[aid], steps, inputs, deadline)
        else:
            supplement["replay_contract"] = {"status": "NOT_APPLICABLE_NO_ANCHOR"}
        supplements.append(supplement)
    claims = []
    for seed in seeds:
        deadline.check()
        claims.append(pair_claim(seed, source_locator(seed, inputs["evidence/raw_read_report.json"]["files"]), limits))
    summary = support_summary(legacy, anchors)
    union_records = sorted({r["record_id"] for c in claims for r in c["required_candidates"]})
    proposal = {**FLAGS, "policy": VERSION, "prior_logical_identity_recomputed": old_id,
        "seeds": seeds, "legacy_claims": legacy, "legacy_replay_contracts": supplements,
        "closures": claims, "acquisition_items": claims,
        "claim_mapping": [{"parent_diagnostic_link": c["parent_diagnostic_link"], "new_claim_id": c["claim_id"], "weaker_separate_claim": True} for c in claims],
        "global_saved_maximum": old_proposal["global_saved_maximum"], "summary": summary,
        "scope_change": "listed record binding replaces no legacy claim; no unconditional order/domain/clock/anchor dependency for listed pairs",
        "union": {"groups": len(claims), "record_ids": union_records, "status": "CLAIM_CLOSURE_BLOCKED" if
            len(claims) > limits.max_seeds or len(claims) > limits.union_groups or len(union_records) > limits.union_candidates else "WITHIN_LISTED_CAPS_UNAPPROVED",
            "does_not_override_individual_caps": True, "physical_chunks": None},
        "optional_clock_claim": {"status": "NOT_REQUESTED", "required_count": 0, "possible_claim": "local diagnostic only; no thinning to prove absence of resets"},
        "legacy_caps": old_proposal["closure_caps"], "current_caps": asdict(limits)}
    budget = deepcopy(old_proposal["future_budget_proposals"])
    envelope = {"envelope_id": "shared_S1_S2_and_retries", "limits": budget,
        "ledger_contract": "all S1/S2 attempts and retries debit this one envelope; no per-stage reset or duplicate allocation; actual consumed bytes survive errors",
        "historical_budget_reused": False, "estimates": "all null; never inferred from JSON count or window duration"}
    approval = {**FLAGS, "claim_ids": [c["claim_id"] for c in claims], "sources": [source_locator(seeds[0], inputs["evidence/raw_read_report.json"]["files"])],
        "windows": [{"claim_id": c["claim_id"], "log_time_ns_inclusive": c["search_window_log_time_ns_inclusive"]} for c in claims],
        "envelope": envelope, "individual_statuses": {c["claim_id"]: c["status"] for c in claims}, "union_status": proposal["union"]["status"],
        "stages": {
            "S1": {"authorized": False, "scope": "only source binding, summary channel/schema, index, intersecting chunk descriptors and declared sizes",
                "payload_decode_or_expansion": False, "automatic_fallback": False, "envelope_ref": envelope["envelope_id"],
                "stop_on": ["no index", "missing definitions", "source mismatch", "individual/union/envelope cap", "unknown required locator"]},
            "S2": {"authorized": False, "requires": ["persisted S1 result", "independent review of corrected reader", "separate explicit payload authorization"],
                "scope": "only approved chunks, the eight listed record hashes and occurrence/schema/time/position bindings",
                "envelope_ref": envelope["envelope_id"], "stop_on": ["missing candidate: no follow-up search", "schema/source mismatch", "budget/partial/error"]}},
        "automatic_stage_transition": False, "physical_chunk_offsets": None, "dedup_physical_chunks": "only after S1; not from logical record count",
        "declared_size_limitations": "reference cost only, not safe actual expansion bound",
        "unresolved_fields": ["actual source binding", "channel/schema definitions", "chunk offsets/count/cost", "record occurrences", "reader implementation/review", "explicit stage permissions"],
        "reader_preconditions": old_proposal["future_reader_preconditions"],
        "forbidden_expansions": ["whole run", "other source", "additional window", "automatic missing-candidate tracking", "Dataset/training/drive"],
        "all_D_E_gates": "OUT_OF_SCOPE_NOT_APPROVED"}
    if any(c["status"] == "SOURCE_LOCATOR_UNRESOLVED" for c in claims):
        approval["unresolved_fields"].append("SOURCE_LOCATOR_UNRESOLVED")
    return {"proposal": proposal, "approval": approval, "facts": summary}


def support_summary(legacy: list, anchors: list) -> dict:
    """Count stored support kinds, not geometry or teacher eligibility."""
    full = [c for c in legacy if c["kind"] == "FULL_SAVED_PREFIX_B_C"]
    within = [c for c in full if c["status"] == "BOUNDED_PROPOSAL_PENDING_DOMAIN_AND_COST"]
    return {"legacy_full_prefix_count": len(full),
        "within_observed_caps_support_kinds": dict(Counter(c["saved_support"]["support_kind"] for c in within)),
        "within_caps_retained_steps": dict(Counter(str(len(c["target_steps"])) for c in within)),
        "all_legacy_support_kinds": dict(Counter(c["saved_support"]["support_kind"] for c in full)),
        "known_zero_is_not_positive_driving_path": True,
        "input_missing_first_future_anchors": sum(a["scopes"]["h30"]["spatial_support"]["support_kind"] == "UNKNOWN_FIRST_FUTURE_MISSING" for a in anchors)}


def unique_object(pairs: list) -> dict:
    result = {}
    for key, value in pairs:
        require(key not in result, "duplicate JSON key")
        result[key] = value
    return result


def reject_constant(value: str) -> None:
    raise ValueError("nonfinite JSON: " + value)


def safe_path(path: Path) -> Path:
    """Reject symlinks/reparse points in every component, including Windows junctions."""
    path = path.absolute()
    for part in (path, *path.parents):
        try:
            info = part.lstat()
        except FileNotFoundError:
            continue
        require(not stat.S_ISLNK(info.st_mode) and not
                getattr(info, "st_file_attributes", 0) & 0x400, "unsafe symlink/reparse path")
    return path.resolve()


def bounded_read(path: Path, cap: int) -> bytes:
    safe_path(path)
    require(stat.S_ISREG(path.stat().st_mode), "not regular JSON file")
    with path.open("rb") as stream:
        data = stream.read(cap + 1)
    require(len(data) <= cap, "LIMIT: JSON bytes")
    return data


def write_json(path: Path, value: Any) -> None:
    with path.open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(value, stream, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False)
        stream.write("\n")


def run_plan(conflict_root: Path, evidence_root: Path, output: Path, repo: Path,
             limits: Limits = Limits(), *, expected: Mapping[str, str] = EXPECTED,
             prior_plan_root: Path | None = None, prior_expected: Mapping[str, str] = PRIOR_EXPECTED,
             prior_identity: str = PRIOR_IDENTITY, fixed_seeds: dict = FIXED_SEEDS) -> dict:
    """Fixed original-nine and explicitly supplied prior-six leaves only; no raw I/O.

    Static path/hash checks do not eliminate concurrent TOCTOU/ABA races.
    The final COMPLETE manifest is published last by an atomic rename.
    """
    limits.validate()
    roots = {"conflict": safe_path(conflict_root), "evidence": safe_path(evidence_root)}
    if prior_plan_root is not None:
        roots["prior"] = safe_path(prior_plan_root)
    output = safe_path(output)
    for root in roots.values():
        require(not (output == root or output.is_relative_to(root) or root.is_relative_to(output)), "input/output containment")
    require(roots["conflict"] != roots["evidence"], "input roots must differ")
    if output.exists():
        raise FileExistsError("immutable output already exists")
    deadline = Deadline(limits.max_seconds)
    entries, inputs, blockers, partial = [], {}, [], []
    total = 0
    expected_all = dict(expected)
    if prior_plan_root is None:
        blockers.append("MISSING_PRIOR_PLAN_ROOT: fixed seeds and legacy identity not invented")
    else:
        expected_all.update({"prior/" + name: prior_expected.get(name) for name in PRIOR_EXPECTED})
    allowlist = list(EXPECTED) + (["prior/" + name for name in PRIOR_EXPECTED] if prior_plan_root is not None else [])
    for key in allowlist:
        namespace, name = key.split("/")
        path = roots[namespace] / name
        entry = {"name": key, "expected_sha256": expected_all.get(key),
            "hash_binding_provenance": "provided_report_later_recorded_not_original_independent_binding" if
                name in ("raw_read_report.json", "selection.json") else "provided_report_expected_hash"}
        entries.append(entry)
        try:
            safe_path(path)
            size = path.stat().st_size
            entry["size_bytes"] = size
            require(size <= limits.max_file_bytes and total + size <= limits.max_total_bytes, "LIMIT: input bytes")
            deadline.check()
            data = bounded_read(path, min(limits.max_file_bytes, limits.max_total_bytes - total))
            total += len(data)
            digest = hashlib.sha256(data).hexdigest()
            entry.update(sha256=digest, status="READ")
            require(is_hash(expected_all.get(key)) and digest == expected_all[key], "input hash mismatch")
            value = data.decode("utf-8") if name == "report_ja.md" else json.loads(data, object_pairs_hook=unique_object, parse_constant=reject_constant)
            require(isinstance(value, (dict, list)) or name == "report_ja.md", "JSON root type")
            deadline.check()
            inputs[key] = value
            entry.update(root_type=type(value).__name__, root_count=len(value))
        except (OSError, ValueError, RecursionError) as error:
            reason = key + ":" + str(error)
            (partial if "LIMIT:" in str(error) else blockers).append(reason)
            entry["status"] = "NOT_INSPECTED_LIMIT" if "LIMIT:" in str(error) else "BLOCKED"
            if "deadline" in str(error):
                break
    inputs["_hashes"] = {e["name"]: e["sha256"] for e in entries if "sha256" in e and not e["name"].startswith("prior/")}
    def empty_result() -> dict:
        return {"proposal": {**FLAGS, "seeds": [], "closures": [], "acquisition_items": [], "status": "BLOCKED_DEPENDENT_INPUT"},
                "approval": {**FLAGS, "claim_ids": [], "sources": [], "windows": [], "stages": {}, "status": "BLOCKED"}, "facts": {}}
    result = empty_result()
    if not blockers and not partial:
        try:
            # Bind the previous audit's observed inputs to this exact allowlist snapshot.
            for prior in inputs["conflict/execution_manifest.json"]["input_files"]:
                key = "evidence/" + prior["name"]
                require(key in inputs["_hashes"] and inputs["_hashes"][key] == prior["sha256"], "prior artifact input hash join")
            prior_inputs = {name: inputs["prior/" + name] for name in PRIOR_EXPECTED}
            result = revise_plan(inputs, prior_inputs, limits, deadline, prior_identity, fixed_seeds)
        except (KeyError, TypeError, ValueError, IndexError, StopIteration, AttributeError, OverflowError) as error:
            reason = "schema_or_join:" + str(error)
            (partial if "LIMIT:" in str(error) else blockers).append(reason)
    # Revalidate paths and bounded content, including inputs rejected after hashing.
    for entry in entries:
        if "sha256" not in entry:
            continue
        namespace, name = entry["name"].split("/")
        try:
            entry["unchanged_after"] = hashlib.sha256(bounded_read(roots[namespace] / name, limits.max_file_bytes)).hexdigest() == entry["sha256"]
        except (OSError, ValueError):
            entry["unchanged_after"] = False
        if not entry["unchanged_after"]:
            blockers.append("input changed:" + entry["name"])
    try:
        deadline.check()
    except ValueError as error:
        partial.append(str(error))
    code_paths = [Path(__file__).resolve(), repo / "tools/plan_spatial_pose_evidence_v4.py"]
    code_hashes, commit, working_tree = {}, None, None
    try:
        code_hashes = {p.name: hashlib.sha256(bounded_read(p, 1024**2)).hexdigest() for p in code_paths}
        git = lambda *args: subprocess.check_output(["git", "-C", str(repo), *args], text=True, timeout=10).strip()
        commit, working_tree = git("rev-parse", "HEAD"), git("status", "--porcelain")
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        blockers.append("code_or_git_identity:" + type(error).__name__)
    try:
        deadline.check()
    except ValueError as error:
        partial.append(str(error))
    if blockers or partial:
        result = empty_result()
    status = "BLOCKED" if blockers else "PARTIAL" if partial else "COMPLETE_PLAN_ONLY"
    requirements = claim_requirements()
    unresolved = {"blockers": blockers, "limits": partial,
        "not_recorded_branches": {"sequence": "zero/recorder counter need not prove publisher order; use verified policy order instead",
            "publish_time": "may equal log_time; no independent clock evidence without semantics",
            "channel_id": "stream identifier not publisher identity", "publisher_id": "not guaranteed; alternative provenance possible",
            "clock_epoch/source_domain": "not guaranteed; absent independent partition makes source closure blocked",
            "covariance": "not estimator accuracy proof; not requested without hypothesis"},
        "terminal_branch": "NOT_RECORDED -> alternatives if sufficient else UNRESOLVABLE_FROM_THIS_SOURCE; no unlimited retry",
        "nonpromotion": "A/B/C never imply D/E; existing tiers unchanged"}
    try:
        logical = identity({"policy": VERSION, "limits": asdict(limits), "input_hashes": inputs["_hashes"],
            "prior_hashes": {e["name"]: e.get("sha256") for e in entries if e["name"].startswith("prior/")},
            "prior_logical_identity": prior_identity, "code_hashes": code_hashes, "result": result, "claims": requirements, "status": status})
    except (ValueError, TypeError, RecursionError) as error:
        blockers.append("logical_identity:" + type(error).__name__)
        logical, result, status = None, empty_result(), "BLOCKED"
    manifest = {**FLAGS, "format": VERSION, "status": status, "logical_plan_identity": logical,
        "plan_commit": commit, "working_tree": working_tree,
        "code_hashes": code_hashes, "input_files": entries, "input_unchanged": all(e.get("unchanged_after") is True for e in entries),
        "created_at_utc": datetime.now(timezone.utc).isoformat(), "limits": asdict(limits),
        "old_execution_commit": OLD_COMMIT, "conflict_execution_commit": CONFLICT_COMMIT,
        "prior_plan_commit": "17ead237de8f1a6b8e52fdc18bcf7c01e75e5403",
        "result_document_commit": "69d21d376ac95dde881fa75d5773827fb68fd04e",
        "request_document_commit": None, "request_provenance": "external attachment 57bc7198-655f-4a9c-914d-a7d5afc860b9; no separate request commit",
        "prior_logical_identity_expected": prior_identity,
        "prior_logical_identity_recomputed": result["proposal"].get("prior_logical_identity_recomputed"),
        "original_nine_verified": all(e.get("unchanged_after") and e["status"] == "READ" for e in entries if not e["name"].startswith("prior/")) and len(inputs["_hashes"]) == 9,
        "prior_six_verified": len([e for e in entries if e["name"].startswith("prior/") and e.get("unchanged_after") and e["status"] == "READ"]) == 6,
        "io_limitations": "static path checks and before/after hashes do not exclude TOCTOU/ABA; cooperative deadlines cannot interrupt one JSON parse",
        "blockers": blockers, "limit_reasons": partial,
        "facts": result["facts"], "exit_code": 0 if status == "COMPLETE_PLAN_ONLY" else 2 if status == "PARTIAL" else 3}
    # Directory collisions remain strictly no-write; only this run owns a new directory.
    safe_path(output)
    try:
        output.mkdir(parents=True, exist_ok=False)
    except OSError as error:
        print("BLOCKED: cannot create immutable output; no error manifest saved: " + str(error), file=sys.stderr)
        raise
    try:
        for name, value in (("input_manifest.json", {"files": entries, "bytes_read_first_pass": total}),
                ("claim_requirements.json", requirements), ("minimal_read_proposal.json", result["proposal"]),
                ("approval_request.json", result["approval"]), ("unresolved_and_unrecoverable.json", unresolved)):
            write_json(output / name, value)
        report = (f"# claim別record結合probe計画\n\n状態: {status}\n\n旧identity: {prior_identity}\n新版identity: {logical}\n\n"
            f"seed数: {len(result['proposal']['seeds'])}。元9 JSON検証と旧6成果物検証をmanifestで分離。\n\n"
            "4seed維持。新record_pair_bindingはより弱い別claim。旧partial/full-prefixの全依存・statusはlegacy_claimsに保持。\n\n"
            "指定pairの存在結合と一意occurrence、投影差と許容性、全候補完全性、物理正確性、採用/実行gateを分離。\n\n"
            "上限内の旧full-prefix KNOWN_ZEROは正の走行pathではない。source locatorは保存文字列のみ。\n\n"
            "S1 metadata/index、S2 payloadは共通envelopeと別承認。rawのstat/存在確認も未実行。\n\n"
            "raw/Dataset読取・再分類・教師生成・tier変更・学習・推論・走行・pushは0。追加取得は未実行・未承認。\n")
        with (output / "report_ja.md").open("x", encoding="utf-8", newline="\n") as stream:
            stream.write(report)
        write_json(output / "execution_manifest.pending.json", manifest)
        (output / "execution_manifest.pending.json").rename(output / "execution_manifest.json")
    except (OSError, ValueError, TypeError) as error:
        manifest.update(status="BLOCKED", exit_code=3, logical_plan_identity=None,
                        blockers=blockers + ["output_finalization:" + type(error).__name__])
        # Partial proposal files are never authoritative without a final COMPLETE manifest.
        try:
            write_json(output / "error_manifest.json", {**manifest, "application_scope": [], "partial_files_not_authoritative": True})
        except (OSError, ValueError, TypeError) as manifest_error:
            print("BLOCKED: output and error-manifest write failed; partial directory is incomplete: " + str(manifest_error), file=sys.stderr)
        print("BLOCKED: output finalization failed: " + str(error), file=sys.stderr)
    return manifest
