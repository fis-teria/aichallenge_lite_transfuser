"""Saved-JSON-only evidence planning. No raw reader, classifier, or teacher generation.

Input shapes: groups [G], impacts [A] with steps [<=30], extracted records [R].
Timestamps are integer ns; saved diagnostic XY/yaw differences are m/rad.
Embedded paths and commands are opaque and never passed to filesystem/process APIs.
"""
from __future__ import annotations

from collections import Counter
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import stat
import subprocess
import time
from typing import Any, Mapping

VERSION = "spatial_pose_evidence_plan_v4_v1"
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


def validate_inputs(inputs: dict, limits: Limits) -> tuple[list, list, dict]:
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
            require(isinstance(row, dict) and is_hash(row["payload_sha256"]) and
                    is_stamp(row["bag_stamp_ns"]) and is_stamp(row["semantic_stamp_ns"]), "record schema")
            require(all(isinstance(row[k], str) and row[k] for k in ("topic", "type", "source_id")), "record binding")
            require(isinstance(row["value"], dict), "record value shape")
            records[identity([raw_hash, run, index])] = {"run_id": run, **row}
    lookup = {g["group_id"]: g for g in groups}
    require(len(lookup) == len(groups), "duplicate group IDs")
    for g in groups:
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
        require(a["processing"] == "PROCESSED" and a["run_id"] == previous[a["sample_id"]]["run_id"], "anchor processing/run")
        require(a["original_tier"] == previous[a["sample_id"]]["tier"], "tier join mismatch")
        step_ids = [s["step"] for s in a["steps"]]
        require(len(step_ids) <= limits.max_steps and step_ids == list(range(1, len(step_ids) + 1)), "step shape/order")
        for h in ("h15", "h30"):
            scope = a["scopes"][h]
            ts = scope["existing_strict_prefix"]["target_steps"]
            require(isinstance(ts, list) and ts == list(range(1, len(ts) + 1)) and set(ts) <= set(step_ids), "prefix step shape")
            require(scope["spatial_support"]["support_kind"] in
                    ("KNOWN_PREFIX", "KNOWN_ZERO", "UNKNOWN_FIRST_FUTURE_MISSING"), "support knownness")
        for kind, _, dep in dependencies(a, step_ids):
            require(isinstance(dep["endpoints"], list) and len(dep["endpoints"]) <= 2, "endpoint shape")
            for endpoint in dep["endpoints"]:
                require(is_stamp(endpoint["stamp_ns"]) and is_hash(endpoint["reported_payload_sha256"]), "endpoint identity")
                expected_ids = set()
                for gid in endpoint["candidate_group_ids"]:
                    g = lookup[gid]
                    require(g["run_id"] == a["run_id"] and g["semantic_stamp_ns"] == endpoint["stamp_ns"] and
                            g["topic"] == (VELOCITY if kind == "velocity" else POSE), "endpoint group join")
                    expected_ids.update(g["candidate_ids"])
                require(expected_ids == set(endpoint["all_candidate_ids"]), "endpoint candidate scope")
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


def claim_requirements() -> dict:
    return {
        "A": {"claim": "保存抽出物の候補差", "necessary_predicates": ["input_hash_join", "saved_projection_scope"],
              "alternative_sufficient_evidence": ["verified saved pair diagnostics; no new raw needed"],
              "unresolved": ["not physical error or safety"], "acquisition_can_resolve": []},
        "B": {"claim": "指定policyでの限定記録stream再現", "necessary_predicates": ["source_schema_binding", "bounded_domain_assignment", "complete_candidate_scope", "explicit_selection_policy"],
              "alternative_sufficient_evidence": ["explicit source/epoch assignment OR independently justified bounded partition", "order-independent selection OR verified policy-specific total order"],
              "unresolved": ["old converter requires historical AnyReader version, file/connection enumeration and equal-time tie evidence", "physical-last is not AnyReader-last; current library is not historical proof"],
              "acquisition_can_resolve": ["source/schema/record positions and local boundary hypotheses, conditional on actually recorded data"]},
        "C": {"claim": "候補選択への投影不変性・感度", "necessary_predicates": ["complete_candidate_scope", "compatible_frame_projection", "bounded_domain_assignment"],
              "alternative_sufficient_evidence": ["all relevant projected values equal: order not necessary", "nonzero sensitivity quantified under explicit independent error budget: not automatic PASS"],
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
        "source_schema_requirements": sorted({(r["source_id"], r["topic"], r["type"]) for r in candidates}),
        "selection_policy_candidate_scope": {"observed_ids_complete_in_saved_JSON": candidate_ids,
            "source_completeness": "UNKNOWN", "whole_run_epoch_alias_absence": "UNKNOWN",
            "required": "all candidates in an independently bound source/domain partition; if not bounded, CLAIM_CLOSURE_BLOCKED, do not expand to whole run"},
        "complete_source_closure_status": "CLAIM_CLOSURE_BLOCKED_PENDING_DOMAIN_PARTITION",
        "no_truncation": True, "prefix_proved": False}


def acquisition_items(c: dict) -> list:
    common = {"claim_id": c["claim_id"], "target_record_ids": [r["record_id"] for r in c["required_candidates"]],
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
        preferred = sorted(related, key=lambda r: (not bool(r["strict_horizons"]), r["sample_id"], r["step"] or 0))
        if preferred:
            ref = preferred[0]
            a = by_anchor[ref["sample_id"]]
            step = ref["step"] or (a["scopes"]["h30"]["existing_strict_prefix"]["target_steps"] or [s["step"] for s in a["steps"]])[0]
            deps = dependencies(a, [step])
            target = {"sample_id": a["sample_id"], "target_steps": [step]}
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
    facts = {"verification_scope": "saved classifications/dependencies/counts joined to allowlisted JSON; no geometric remeasurement",
        "record_count": len(records), "anchor_count": len(anchors), "pose_group_count": sum(g["topic"] == POSE for g in groups),
        "duplicate_pose_count": sum(g["topic"] == POSE and g["candidate_count"] > 1 for g in groups),
        "anchor_endpoint_observed_difference_count": sum(a["anchor_pose_dependency"]["observed_difference"] is True for a in anchors),
        "prior_summary_reported": {k: v for k, v in inputs["conflict/summary.json"].items() if k != "elapsed_analysis_sec"},
        "old_tiers_reported": inputs["evidence/execution_manifest.json"].get("tiers"),
        "Dataset_identity_reported_not_body_verified": inputs["evidence/execution_manifest.json"]["dataset_identity"]}
    return {"proposal": proposal, "facts": facts}


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
             limits: Limits = Limits(), *, expected: Mapping[str, str] = EXPECTED) -> dict:
    """Open only nine fixed JSON names and two local code files, never embedded paths.

    Missing/schema/hash/limit failures generate a BLOCKED/PARTIAL plan, not fake seeds.
    Unsafe output locations are rejected before any output write.
    """
    limits.validate()
    roots = {"conflict": safe_path(conflict_root), "evidence": safe_path(evidence_root)}
    output = safe_path(output)
    for root in roots.values():
        require(not (output == root or output.is_relative_to(root) or root.is_relative_to(output)), "input/output containment")
    require(roots["conflict"] != roots["evidence"], "input roots must differ")
    if output.exists():
        raise FileExistsError("immutable output already exists")
    started = time.monotonic()
    entries, inputs, blockers, partial = [], {}, [], []
    total = 0
    for key in EXPECTED:
        namespace, name = key.split("/")
        path = roots[namespace] / name
        entry = {"name": key, "expected_sha256": expected.get(key),
            "hash_binding_provenance": "provided_report_later_recorded_not_original_independent_binding" if
                name in ("raw_read_report.json", "selection.json") else "provided_report_expected_hash"}
        entries.append(entry)
        try:
            safe_path(path)
            size = path.stat().st_size
            entry["size_bytes"] = size
            require(size <= limits.max_file_bytes and total + size <= limits.max_total_bytes, "LIMIT: input bytes")
            require(time.monotonic() - started <= limits.max_seconds, "LIMIT: plan seconds")
            data = bounded_read(path, min(limits.max_file_bytes, limits.max_total_bytes - total))
            total += len(data)
            digest = hashlib.sha256(data).hexdigest()
            entry.update(sha256=digest, status="READ")
            require(is_hash(expected.get(key)) and digest == expected[key], "input hash mismatch")
            value = json.loads(data, object_pairs_hook=unique_object, parse_constant=reject_constant)
            require(isinstance(value, (dict, list)), "JSON root type")
            inputs[key] = value
            entry.update(root_type=type(value).__name__, root_count=len(value))
        except (OSError, ValueError, RecursionError) as error:
            reason = key + ":" + str(error)
            (partial if "LIMIT:" in str(error) else blockers).append(reason)
            entry["status"] = "NOT_INSPECTED_LIMIT" if "LIMIT:" in str(error) else "BLOCKED"
    inputs["_hashes"] = {e["name"]: e["sha256"] for e in entries if "sha256" in e}
    result = {"proposal": {**FLAGS, "seeds": [], "closures": [], "status": "BLOCKED_DEPENDENT_INPUT"}, "facts": {}}
    if not blockers and not partial:
        try:
            # Bind the previous audit's observed inputs to this exact allowlist snapshot.
            for prior in inputs["conflict/execution_manifest.json"]["input_files"]:
                key = "evidence/" + prior["name"]
                require(key in inputs["_hashes"] and inputs["_hashes"][key] == prior["sha256"], "prior artifact input hash join")
            result = build_plan(inputs, limits)
        except (KeyError, TypeError, ValueError, IndexError, StopIteration) as error:
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
    if time.monotonic() - started > limits.max_seconds:
        partial.append("LIMIT: plan seconds; generated scope not released")
    if blockers or partial:
        result = {"proposal": {**FLAGS, "seeds": [], "closures": [], "status": "BLOCKED_DEPENDENT_INPUT"}, "facts": {}}
    status = "BLOCKED" if blockers else "PARTIAL" if partial else "COMPLETE_PLAN_ONLY"
    code_paths = [Path(__file__).resolve(), repo / "tools/plan_spatial_pose_evidence_v4.py"]
    code_hashes = {p.name: hashlib.sha256(bounded_read(p, 1024**2)).hexdigest() for p in code_paths}
    git = lambda *args: subprocess.check_output(["git", "-C", str(repo), *args], text=True).strip()
    requirements = claim_requirements()
    unresolved = {"blockers": blockers, "limits": partial,
        "not_recorded_branches": {"sequence": "zero/recorder counter need not prove publisher order; use verified policy order instead",
            "publish_time": "may equal log_time; no independent clock evidence without semantics",
            "channel_id": "stream identifier not publisher identity", "publisher_id": "not guaranteed; alternative provenance possible",
            "clock_epoch/source_domain": "not guaranteed; absent independent partition makes source closure blocked",
            "covariance": "not estimator accuracy proof; not requested without hypothesis"},
        "terminal_branch": "NOT_RECORDED -> alternatives if sufficient else UNRESOLVABLE_FROM_THIS_SOURCE; no unlimited retry",
        "nonpromotion": "A/B/C never imply D/E; existing tiers unchanged"}
    logical = identity({"policy": VERSION, "limits": asdict(limits), "input_hashes": inputs["_hashes"],
        "code_hashes": code_hashes, "result": result, "claims": requirements, "status": status})
    manifest = {**FLAGS, "format": VERSION, "status": status, "logical_plan_identity": logical,
        "plan_commit": git("rev-parse", "HEAD"), "working_tree": git("status", "--porcelain"),
        "code_hashes": code_hashes, "input_files": entries, "input_unchanged": all(e.get("unchanged_after") is True for e in entries),
        "created_at_utc": datetime.now(timezone.utc).isoformat(), "limits": asdict(limits),
        "old_execution_commit": OLD_COMMIT, "conflict_execution_commit": CONFLICT_COMMIT,
        "result_document_commit": "abec800e031a07f2898612af5d8177651354fd84",
        "request_document_commit": None, "request_provenance": "external user attachment, not a separate repository commit",
        "facts": result["facts"], "exit_code": 0 if status == "COMPLETE_PLAN_ONLY" else 2 if status == "PARTIAL" else 3}
    safe_path(output)
    output.mkdir(parents=True, exist_ok=False)
    for name, value in (("input_manifest.json", {"files": entries, "JSON_bytes_read_first_pass": total}),
                        ("claim_requirements.json", requirements), ("minimal_read_proposal.json", result["proposal"]),
                        ("unresolved_and_unrecoverable.json", unresolved), ("execution_manifest.json", manifest)):
        write_json(output / name, value)
    report = (f"# 最小追加pose証拠取得計画\n\n状態: {status}\n\nlogical identity: `{logical}`\n\n"
        f"seed数: {len(result['proposal']['seeds'])}。選択理由・ID/hash・全endpoint closureはminimal_read_proposal.json。\n\n"
        "Aは保存候補差、Bは指定policyの記録再現、Cは投影不変性/感度、Dは物理正確性、Eは採用・走行。相互昇格しない。\n\n"
        "部分probeと全prefix closureは別claim。closure超過・domain未確定なら原本へ範囲拡大せずBLOCKED。\n\n"
        "仕様参照: [MCAP](https://mcap.dev/spec)、[AnyReader](https://ternaris.gitlab.io/rosbags/api/rosbags.highlevel.html)。実ファイルの記録内容は未確認。\n\n"
        "raw/Dataset読取・学習・推論・制御・走行・pushなし。既存tierは変更なし。\n\n"
        "追加取得は未実行・未承認。対象source/stage/window/claim、独立予算、reader修正・試験、原本不変性と新規出力先を別途明示承認するまで取得しない。\n")
    with (output / "report_ja.md").open("x", encoding="utf-8", newline="\n") as stream:
        stream.write(report)
    return manifest
