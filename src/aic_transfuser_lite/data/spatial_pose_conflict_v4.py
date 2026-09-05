"""Bounded reclassification of saved JSON only; no dataset/model/ROS imports.

Record indices identify saved JSON array positions, NOT MCAP physical order.
All coordinates remain Python float64 world values. No teacher XY is generated.
"""
from __future__ import annotations

from collections import Counter, defaultdict
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
from itertools import combinations
import json
import math
from pathlib import Path
import platform
import subprocess
import time
from typing import Any, Mapping, Sequence

VERSION = "spatial_pose_conflict_v4_v1"
OLD_COMMIT = "7ae8298b71aa7bdbacf1d1757798bf0de66bcfc4"
DATASET_ID = "181cf909b80589110574859990b0885005b7f9a0bb07cff1c24f38d6b090f388"
EXPECTED_HASHES = {
    "execution_manifest.json": "4261af73079fdaafa7b2478b94447ab904fbd9bfcd61b7ed8529bd0c449d340b",
    "anchor_evidence.json": "98399b988b6c69fd7c47668f91d1388a62d46f3cb03d7a1e442ea8582db5c7df",
    "raw_window_evidence.json": "97d2ccd5992aacf7c20b894f30afc03de283ded61dd4e15cda3ea2b28d853d75",
}
REQUIRED = tuple(EXPECTED_HASHES) + ("raw_read_report.json",)
ALLOWLIST = REQUIRED + ("selection.json",)
POSE = "/localization/kinematic_state"
VELOCITY = "/vehicle/status/velocity_status"
FIELDS = ("source_id", "topic", "type", "bag_stamp_ns", "semantic_stamp_ns",
          "timestamp_source", "payload_sha256", "value")
NOT_USUALLY_RECORDED = ("channel_id", "schema_id", "schema_definition_hash", "publish_time",
    "sequence", "chunk_offset", "record_offset", "publisher_id", "clock_epoch", "clock_domain",
    "source_domain", "quaternion", "covariance")
DISTANCES = ("0.5m", "1m", "1.5m", "2m")


def clean(value: Any) -> Any:
    """Represent invalid nonfinite JSON explicitly, never zero or silent null."""
    if isinstance(value, float) and not math.isfinite(value):
        return {"invalid_number": repr(value)}
    if isinstance(value, dict):
        return {str(k): clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    return value


def identity(value: Any) -> str:
    return hashlib.sha256(json.dumps(clean(value), sort_keys=True, ensure_ascii=False,
                                    separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def sha256_file(path: Path) -> str:
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest() if hasattr(hashlib, "file_digest") else _digest_stream(stream)


def _digest_stream(stream: Any) -> str:
    digest = hashlib.sha256()
    for block in iter(lambda: stream.read(65536), b""):
        digest.update(block)
    return digest.hexdigest()


def predicate(status: str, reason: str, **evidence: Any) -> dict[str, Any]:
    assert status in {"PASS", "FAIL", "UNKNOWN", "NOT_INSPECTED"}
    return {"status": status, "predicate": reason, **clean(evidence)}


def finite(value: Any) -> bool:
    try:
        return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
    except OverflowError:
        return False


def stamp(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and 0 <= value <= 2**63 - 1


def mapping(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


@dataclass(frozen=True)
class ConflictConfig:
    max_file_bytes: int = 16 * 1024**2
    max_total_bytes: int = 32 * 1024**2
    max_records: int = 10000
    max_anchors: int = 64
    max_steps: int = 30
    max_pairs_per_group: int = 4096
    max_total_pairs: int = 100000
    max_seconds: float = 60.0
    material_xy_budget_m: float | None = None
    material_budget_provenance: str | None = None

    def validate(self) -> None:
        for key, value in asdict(self).items():
            if key.startswith("max_") and (not finite(value) or value <= 0):
                raise ValueError(f"invalid limit: {key}")
        if self.material_xy_budget_m is not None:
            if not finite(self.material_xy_budget_m) or self.material_xy_budget_m <= 0 or not self.material_budget_provenance:
                raise ValueError("material budget requires positive value and independent provenance")


class Budget:
    def __init__(self, config: ConflictConfig):
        config.validate()
        self.config, self.started, self.pairs = config, time.monotonic(), 0
        self.reasons: set[str] = set()

    def available(self) -> bool:
        if time.monotonic() - self.started > self.config.max_seconds:
            self.reasons.add("analysis_time_limit")
            return False
        return True


def domain(record: Mapping[str, Any]) -> tuple[Any, ...] | None:
    values = tuple(record.get(k) for k in ("source_domain", "clock_domain", "clock_epoch"))
    return values if all(isinstance(v, (str, int)) and not isinstance(v, bool)
                         and str(v) not in {"", "UNKNOWN", "NOT_RECORDED"} for v in values) else None


def record_errors(record: Any) -> list[str]:
    if not isinstance(record, dict):
        return ["record_not_object"]
    errors = ["missing:" + k for k in FIELDS if k not in record]
    for key in ("bag_stamp_ns", "semantic_stamp_ns"):
        if not stamp(record.get(key)):
            errors.append("invalid:" + key)
    for key in ("source_id", "topic", "type", "timestamp_source", "payload_sha256"):
        if not isinstance(record.get(key), str) or not record[key]:
            errors.append("invalid:" + key)
    if not isinstance(record.get("value"), dict):
        errors.append("invalid:value")
    value = mapping(record.get("value"))
    def nonfinite_paths(v: Any, prefix: str) -> list[str]:
        if isinstance(v, float) and not math.isfinite(v): return ["nonfinite:" + prefix]
        if isinstance(v, dict): return [p for k, x in v.items() for p in nonfinite_paths(x, prefix + "." + str(k))]
        if isinstance(v, list): return [p for i, x in enumerate(v) for p in nonfinite_paths(x, prefix + f"[{i}]")]
        return []
    errors += nonfinite_paths(record, "record")
    keys = ("x_m", "y_m", "yaw_rad") if record.get("topic") == POSE else (
        "longitudinal_mps", "lateral_mps", "yaw_rate_rps") if record.get("topic") == VELOCITY else ()
    errors += ["nonfinite_or_missing:" + k for k in keys if not finite(value.get(k))]
    if record.get("topic") == POSE:
        errors += ["missing_frame:" + k for k in ("frame_id", "child_frame_id") if not isinstance(value.get(k), str) or not value[k]]
        if record.get("type") != "nav_msgs/msg/Odometry":
            errors.append("unexpected_pose_type")
    return sorted(set(errors))


def inventory_records(raw: Mapping[str, Any], file_hash: str) -> list[dict[str, Any]]:
    output = []
    for run, rows in sorted(raw.items()):
        if not isinstance(rows, list):
            raise ValueError("raw run must contain an array")
        for index, original in enumerate(rows):
            r = mapping(original)
            ref = identity([file_hash, run, index])
            value = mapping(r.get("value"))
            output.append({"record_id": ref, "run_id": run, "original_array_index": index,
                "input_file_sha256": file_hash, "record": r, "errors": record_errors(original),
                "domain": domain(r), "missing_fields": [k for k in FIELDS if k not in r],
                "not_recorded_provenance": [k for k in NOT_USUALLY_RECORDED if k not in r and k not in value],
                "float64_diagnostics": {k: {"hex": float(value[k]).hex(), "ulp": math.ulp(float(value[k]))}
                    for k in ("x_m", "y_m", "yaw_rad") if finite(value.get(k))},
                "relative_float32_quantization": "NOT_RECOMPUTED_NO_SAVED_FULL_FUTURE",
                "physical_noise_calibration": "UNKNOWN"})
    return output


def pair_metrics(a: Mapping[str, Any], b: Mapping[str, Any], kind: str) -> dict[str, float] | None:
    av, bv = mapping(a.get("value")), mapping(b.get("value"))
    fields = ("x_m", "y_m", "yaw_rad") if kind == POSE else ("longitudinal_mps", "lateral_mps", "yaw_rate_rps")
    if not all(finite(v.get(k)) for v in (av, bv) for k in fields):
        return None
    if kind == POSE:
        delta = av["yaw_rad"] - bv["yaw_rad"]
        if not finite(delta): return None
        result = {"xy_m": math.hypot(av["x_m"] - bv["x_m"], av["y_m"] - bv["y_m"]),
                  "yaw_rad": abs(math.remainder(delta, math.tau))}
    else:
        result = {k: abs(av[k] - bv[k]) for k in fields}
    return result if all(finite(v) for v in result.values()) else None


def classify_group(candidates: Sequence[Mapping[str, Any]], budget: Budget) -> dict[str, Any]:
    """All-pair numerical facts, orthogonal domain/order/physical interpretations."""
    kind = candidates[0]["record"].get("topic")
    count = len(candidates)
    complete, measured, maxima, nonzero = True, 0, {}, False
    legacy_all, legacy_first, legacy_adjacent = 0, 0, 0
    invalid = any(c["errors"] for c in candidates)
    for i, j in combinations(range(count), 2):
        if measured >= budget.config.max_pairs_per_group or budget.pairs >= budget.config.max_total_pairs or not budget.available():
            complete = False
            budget.reasons.add("pair_or_time_limit")
            break
        measured += 1
        budget.pairs += 1
        metrics = pair_metrics(candidates[i]["record"], candidates[j]["record"], kind)
        if metrics is None:
            invalid = True
            continue
        nonzero |= any(v > 0 for v in metrics.values())
        for key, value in metrics.items():
            maxima[key] = max(maxima.get(key, 0.0), value)
        if metrics.get("xy_m", 0) > 1e-8:
            legacy_all += 1
            legacy_first += i == 0
            legacy_adjacent += j == i + 1
    frames = sorted({(str(c["record"].get("type")), str(mapping(c["record"].get("value")).get("frame_id")),
                      str(mapping(c["record"].get("value")).get("child_frame_id"))) for c in candidates})
    hashes = sorted({str(c["record"].get("payload_sha256")) for c in candidates})
    sources = sorted({str(c["record"].get("source_id")) for c in candidates})
    known_domain = all(c["domain"] is not None for c in candidates) and len({c["domain"] for c in candidates}) == 1
    conflict = len(frames) > 1
    labels = []
    if count == 1:
        labels.append("SINGLE_OBSERVED_CANDIDATE")
    elif complete and not invalid and not conflict and not nonzero:
        labels.append("BYTE_IDENTICAL_OBSERVED" if len(hashes) == 1 and len(sources) == 1 and known_domain else "PROJECTED_GEOMETRY_EQUAL")
    if nonzero:
        labels.append("NONZERO_DIFFERENCE_UNCALIBRATED")
    material = budget.config.material_xy_budget_m
    if material is not None and maxima.get("xy_m", 0) > material and not conflict:
        labels.append("MATERIAL_DIFFERENCE_EVIDENCED")
    if conflict:
        labels.append("FRAME_OR_TYPE_CONFLICT")
    if invalid:
        labels.append("INVALID_OR_INCOMPLETE_RECORD")
    if not known_domain or len(sources) > 1 or count > 1:
        labels.append("ORDER_OR_EPOCH_AMBIGUOUS")
    if not complete:
        labels.append("PAIR_ANALYSIS_PARTIAL")
    consistency = "UNKNOWN" if invalid or not complete else "FAIL" if nonzero or conflict else "PASS"
    bag_values = [c["record"].get("bag_stamp_ns") for c in candidates]
    return {"group_id": identity([candidates[0]["run_id"], kind, candidates[0]["record"].get("semantic_stamp_ns"), candidates[0]["domain"]]),
        "run_id": candidates[0]["run_id"], "topic": kind, "semantic_stamp_ns": candidates[0]["record"].get("semantic_stamp_ns"),
        "explicit_domain": candidates[0]["domain"], "bucket_kind": "KNOWN_DOMAIN" if known_domain else "CANDIDATE_BUCKET_NOT_PHYSICAL_POSE_GROUP",
        "candidate_ids": [c["record_id"] for c in candidates], "candidate_count": count,
        "candidate_set_identity": identity(sorted(identity(c["record"]) for c in candidates)),
        "frame_type_set": frames, "payload_hash_set": hashes, "source_id_set": sources,
        "bag_stamp_set_ns": sorted({v for v in bag_values if stamp(v)}),
        "classification": labels, "all_pairs_total": count * (count - 1) // 2,
        "pairs_measured": measured, "all_pair_maxima": maxima if complete and not invalid else None,
        "measured_pair_maxima_lower_bound": maxima, "all_pair_maximum_status": "PASS" if complete and not invalid else "UNKNOWN",
        "legacy_xy_gt_1e8": {"all_pairs_count": legacy_all if complete and not invalid else None,
            "first_to_later_count": legacy_first if complete and not invalid else None,
            "array_adjacent_within_group_count": legacy_adjacent if complete and not invalid else None,
            "threshold_m": 1e-8, "physical_interpretation": "UNKNOWN"},
        "observed_projected_equality": predicate(consistency, "all_observed_candidate_projected_values_equal_not_physical_uniqueness"),
        "domain_identity": predicate("PASS" if known_domain and len(sources) == 1 else "UNKNOWN", "explicit_source_clock_epoch_identity"),
        "order_identity": predicate("UNKNOWN", "JSON_order_not_converter_or_MCAP_order"),
        "payload_hash_equal": len(hashes) == 1, "observed_nonzero_difference": nonzero,
        "material_budget": {"value_m": material, "provenance": budget.config.material_budget_provenance,
                            "scope": "explicit_budget_only_not_automatically_physical_noise_or_safety"},
        "reproduction_20um_comparison_only": maxima.get("xy_m", 0) > 2e-5 if complete and not invalid else None,
        "physical_noise": predicate("UNKNOWN", "no_physical_pose_noise_calibration"),
        "last_in_saved_array_id": candidates[-1]["record_id"], "first_in_saved_array_id": candidates[0]["record_id"],
        "first_last_difference": pair_metrics(candidates[0]["record"], candidates[-1]["record"], kind) if count > 1 else None,
        "same_bag_time_order_sensitive": len(set(v for v in bag_values if stamp(v))) < count and nonzero}


def build_groups(records: Sequence[Mapping[str, Any]], budget: Budget) -> tuple[list[dict[str, Any]], dict[tuple, list[dict[str, Any]]]]:
    buckets: dict[tuple, list[Mapping[str, Any]]] = defaultdict(list)
    for entry in records:
        r = entry["record"]
        if r.get("topic") in {POSE, VELOCITY} and stamp(r.get("semantic_stamp_ns")):
            buckets[(entry["run_id"], r["topic"], r["semantic_stamp_ns"], entry["domain"])].append(entry)
    groups, lookup = [], defaultdict(list)
    for key, candidates in sorted(buckets.items(), key=lambda item: json.dumps(item[0])):
        group = classify_group(candidates, budget)
        groups.append(group)
        lookup[key[:3]].append(group)
    return groups, lookup


def dependency(endpoint: Any, run: str, topic: str, lookup: Mapping[tuple, list[dict[str, Any]]],
               by_id: Mapping[str, Mapping[str, Any]], expected_domain: tuple | None = None) -> dict[str, Any]:
    e = mapping(endpoint)
    stamps, hashes = e.get("source_stamps_ns"), e.get("source_payload_hashes")
    out: dict[str, Any] = {"original_endpoint_evidence": clean(e), "endpoints": [], "observed_difference": None,
                          "frame_or_type_conflict": False, "invalid_candidate": False}
    if not isinstance(stamps, list) or len(stamps) != 2 or not all(stamp(s) for s in stamps) or not isinstance(hashes, list) or len(hashes) != 2:
        return {**out, "status": "UNKNOWN", "reasons": ["endpoint_stamp_hash_not_recorded"]}
    reasons = set()
    target = e.get("target_ns")
    if not stamp(target) or not stamps[0] <= target <= stamps[1] or e.get("reason") != "interpolated":
        reasons.add("original_interpolation_not_valid_or_ordered")
    for s, h in zip(stamps, hashes):
        groups = list(lookup.get((run, topic, s), []))
        if expected_domain is not None:
            groups = [g for g in groups if g["explicit_domain"] is None or tuple(g["explicit_domain"]) == expected_domain]
        ids = [rid for g in groups for rid in g["candidate_ids"]]
        matches = [rid for rid in ids if by_id[rid]["record"].get("payload_sha256") == h]
        if not groups or not matches:
            reasons.add("saved_source_endpoint_not_found")
        if len(matches) != 1:
            reasons.add("endpoint_record_identity_ambiguous")
        if expected_domain is None or len(groups) != 1 or any(g["domain_identity"]["status"] != "PASS" for g in groups):
            reasons.add("source_or_epoch_alias_not_resolved")
        for g in groups:
            out["observed_difference"] = bool(out["observed_difference"]) or g["observed_nonzero_difference"]
            out["frame_or_type_conflict"] |= "FRAME_OR_TYPE_CONFLICT" in g["classification"]
            out["invalid_candidate"] |= "INVALID_OR_INCOMPLETE_RECORD" in g["classification"]
            if g["observed_projected_equality"]["status"] == "UNKNOWN":
                reasons.add("candidate_values_invalid_or_not_fully_measured")
        out["endpoints"].append({"stamp_ns": s, "reported_payload_sha256": h, "candidate_group_ids": [g["group_id"] for g in groups],
            "matching_record_ids": matches, "all_candidate_ids": ids,
            "matches_saved_array_last": any(by_id[g["last_in_saved_array_id"]]["record"].get("payload_sha256") == h for g in groups),
            "converter_order_match": "UNKNOWN"})
    status = "UNKNOWN" if reasons else "FAIL" if out["observed_difference"] or out["frame_or_type_conflict"] else "PASS"
    return {**out, "status": status, "predicate": "interpolation_candidate_uniqueness_with_domain_evidence",
            "reasons": sorted(reasons), "original_choice_not_new_ground_truth": True}


def combine_dependencies(deps: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    statuses = [d["status"] for d in deps]
    return {"status": "FAIL" if "FAIL" in statuses else "UNKNOWN" if not statuses or "UNKNOWN" in statuses else "PASS",
            "observed_difference": (any(d.get("observed_difference") for d in deps)
                if any(d.get("observed_difference") is not None for d in deps) else None),
            "frame_or_type_conflict": any(d.get("frame_or_type_conflict") for d in deps),
            "invalid_candidate": any(d.get("invalid_candidate") for d in deps),
            "scope": "listed_interpolation_dependencies_not_all_window_records"}


def legacy_window(records: Sequence[Mapping[str, Any]], start: int, end: int, gap_sec: float = .2) -> dict[str, Any]:
    """Copy of old predicates, no import/execution of old reader or model modules."""
    if any(not isinstance(r, dict) for r in records):
        return {"status": "UNKNOWN", "flags": [], "reason": "invalid_record_cannot_assign_window"}
    poses = [r for r in records if r.get("topic") == POSE and stamp(r.get("semantic_stamp_ns"))
             and start - 50000000 <= r["semantic_stamp_ns"] <= end + 50000000]
    if any(record_errors(r) for r in poses):
        return {"status": "UNKNOWN", "flags": [], "reason": "legacy_input_invalid"}
    poses.sort(key=lambda r: r["bag_stamp_ns"])
    flags = set()
    for a, b in zip(poses, poses[1:]):
        dt = (b["semantic_stamp_ns"] - a["semantic_stamp_ns"]) / 1e9
        if dt < 0: flags.add("pose_timestamp_reversal")
        if dt > gap_sec: flags.add("pose_gap")
        if any(a["value"].get(k) != b["value"].get(k) for k in ("frame_id", "child_frame_id")):
            flags.add("frame_change")
        distance = pair_metrics(a, b, POSE)["xy_m"]
        if dt > 0 and distance / dt > 20: flags.add("teleport_provisional_20mps")
        if dt == 0 and distance > 1e-8: flags.add("conflicting_duplicate_pose_stamp")
    clocks = [r for r in records if r.get("topic") == "/clock" and stamp(r.get("bag_stamp_ns"))
              and start <= r["bag_stamp_ns"] <= end]
    if any(not stamp(mapping(r.get("value")).get("clock_ns")) for r in clocks):
        return {"status": "UNKNOWN", "flags": sorted(flags), "reason": "clock_record_invalid"}
    clocks.sort(key=lambda r: r["bag_stamp_ns"])
    if any(b["value"]["clock_ns"] < a["value"]["clock_ns"] for a, b in zip(clocks, clocks[1:])):
        flags.add("clock_reset")
    return {"status": "FAIL" if flags else "PASS" if len(poses) >= 2 and len(clocks) >= 2 else "UNKNOWN",
            "flags": sorted(flags), "pose_records": len(poses), "clock_records": len(clocks),
            "meaning": "legacy_window_predicate_not_physical_safety"}


def extraction(files: Sequence[Mapping[str, Any]], start: int, end: int) -> dict[str, Any]:
    relevant = [f for f in files if any(isinstance(w, list) and len(w) == 2 and all(stamp(t) for t in w)
                    and w[0] <= end and w[1] >= start for w in (f.get("windows_bag_ns") if isinstance(f.get("windows_bag_ns"), list) else []))]
    bad = any(f.get("source_stat_unchanged") is False or f.get("decode_errors") for f in relevant)
    return {"status": "FAIL" if bad else "UNKNOWN", "predicate": "independent_extraction_completeness",
        "reports": [{k: f.get(k) for k in ("mode", "scan_stop", "windows_bag_ns", "decode_errors", "source_stat_unchanged", "status", "source_id", "logical_payload_identity")} for f in relevant],
        "reader_reported_complete": bool(relevant) and all(f.get("status") == "COMPLETE" for f in relevant),
        "missing_predicates": ["independent_schema_channel_binding", "physical_order_and_epoch_coverage", "all_source_files_accounted_for"],
        "forward_monotonicity_unproven": any(f.get("mode") == "forward_stream" for f in relevant),
        "no_report_for_window": not relevant}


def support_scope(original: Mapping[str, Any], h: str, steps: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    comparison = mapping(original.get("comparison"))
    strict = mapping(mapping(comparison.get("strict_diagnostic_v1")).get(h))
    raw = mapping(mapping(comparison.get("raw_v1")).get(h))
    first_missing = not steps or steps[0].get("saved_valid") is not True
    arc = strict.get("arc_m")
    retained = strict.get("retained_steps")
    usable = not first_missing and finite(arc) and isinstance(retained, int) and retained > 0
    return {"status": "PASS" if usable else "UNKNOWN", "source": "old_saved_summary_not_remeasured",
        "support_m": arc if usable else None,
        "reaches": strict.get("reaches") if usable else {d: None for d in DISTANCES},
        "support_kind": "UNKNOWN_FIRST_FUTURE_MISSING" if first_missing else "KNOWN_ZERO" if usable and arc == 0 else "KNOWN_PREFIX" if usable else "UNKNOWN",
        "reported_retained_steps": retained, "reported_elapsed_sec": strict.get("elapsed_sec"),
        "raw_support_m_reported": raw.get("raw_arc_m"), "cut_reason": strict.get("cut_reason"),
        "tail_status": "UNKNOWN", "negative_continuation": None, "safe_endpoint": None,
        "hold_policy": "legacy_0.5s_abs_longitudinal_speed_le_0.01_only_not_standstill_or_intent"}


def anchor_impact(anchor: Mapping[str, Any], raw: Mapping[str, list], lookup: Mapping,
                  by_id: Mapping, reports: Sequence[Mapping], config: ConflictConfig) -> dict[str, Any]:
    run = anchor.get("run_id")
    repro = mapping(anchor.get("source_reproduction"))
    obs = repro.get("t_obs_ns")
    steps = repro.get("steps")
    if not isinstance(anchor.get("sample_id"), str) or not isinstance(run, str) or not stamp(obs) or not isinstance(steps, list) or len(steps) != 30 or len(steps) > config.max_steps:
        return {"sample_id": anchor.get("sample_id"), "processing": "INVALID_OR_LIMITED", "status": "UNKNOWN"}
    if any(not isinstance(s, dict) or s.get("step") != i + 1 or not isinstance(s.get("saved_valid"), bool) for i, s in enumerate(steps)):
        return {"sample_id": anchor.get("sample_id"), "processing": "INVALID_OR_LIMITED", "status": "UNKNOWN"}
    dom = domain(anchor)
    base = dependency(repro.get("anchor_interpolation"), run, POSE, lookup, by_id, dom)
    analyzed = []
    for step in steps:
        p = dependency(step.get("pose"), run, POSE, lookup, by_id, dom)
        v = dependency(step.get("velocity"), run, VELOCITY, lookup, by_id, dom)
        analyzed.append({"step": step["step"], "saved_valid": step["saved_valid"],
            "original_numeric_reproduction_status": step.get("status", "UNKNOWN"),
            "pose_dependency": p, "velocity_dependency": v,
            "combined": combine_dependencies([base, p, v])})
    scopes = {}
    files = [f for f in reports if f.get("run_id") == run]
    for h, length in (("h15", 15), ("h30", 30)):
        chosen = analyzed[:length]
        support = support_scope(anchor, h, chosen)
        retained = support["reported_retained_steps"]
        retained_valid = isinstance(retained, int) and not isinstance(retained, bool) and 0 <= retained <= len(chosen)
        if retained_valid and retained:
            e = mapping(steps[retained - 1].get("pose"))
            retained_valid = stamp(e.get("target_ns")) and finite(support["reported_elapsed_sec"]) and abs((e["target_ns"] - obs) / 1e9 - support["reported_elapsed_sec"]) < 1e-6 and all(s["saved_valid"] for s in chosen[:retained])
        prefix = chosen[:retained] if retained_valid else []
        source_prefix = 0
        for s in chosen:
            if not s["saved_valid"] or s["combined"]["status"] != "PASS": break
            source_prefix += 1
        observed_prefix = 0
        for s in chosen:
            if not s["saved_valid"] or s["combined"]["observed_difference"] or s["combined"]["frame_or_type_conflict"] or s["combined"]["invalid_candidate"]: break
            if not s["pose_dependency"]["endpoints"] or not s["velocity_dependency"]["endpoints"] or not base["endpoints"]: break
            observed_prefix += 1
        def aggregate(rows: Sequence[Mapping]) -> dict:
            return {"target_steps": [s["step"] for s in rows], "count": len(rows),
                "dependency": combine_dependencies([s["combined"] for s in rows]),
                "observed_difference_steps": [s["step"] for s in rows if s["combined"]["observed_difference"]],
                "status_counts": dict(Counter(s["combined"]["status"] for s in rows))}
        window = legacy_window(raw.get(run, []), obs, obs + length * 100000000)
        prefix_scope = aggregate(prefix)
        prefix_end = obs + round(support["reported_elapsed_sec"] * 1e9) if retained_valid and finite(support["reported_elapsed_sec"]) else None
        prefix_scope["window_end_ns"] = prefix_end
        prefix_scope["local_boundary_evidence"] = predicate("UNKNOWN" if prefix else "NOT_INSPECTED",
            "prefix_interval_no_boundary_not_proven", legacy_recomputed=(legacy_window(raw.get(run, []), obs, prefix_end) if prefix and prefix_end is not None else None))
        prefix_scope["extraction_completeness"] = extraction(files, obs, prefix_end) if prefix and prefix_end is not None else predicate("NOT_INSPECTED", "no_positive_time_prefix_to_check")
        scopes[h] = {"all_targets": aggregate(chosen), "saved_valid_targets": aggregate([s for s in chosen if s["saved_valid"]]),
            "existing_strict_prefix": prefix_scope, "retained_metadata_consistent": retained_valid,
            "spatial_support": support, "legacy_window_recomputed": window,
            "local_boundary_evidence": predicate("UNKNOWN", "window_checks_do_not_prove_no_clock_frame_epoch_boundary", legacy=window),
            "extraction_completeness": extraction(files, obs, obs + length * 100000000),
            "source_consistent_prefix_candidate": {"steps": source_prefix, "support_m": None, "xy": None,
                "reason": "stop_before_first_invalid_or_unknown_dependency_no_xy_reconstruction"},
            "observed_difference_free_prefix_diagnostic": {"steps": observed_prefix, "certified": False,
                "reason": "conditional_on_saved_stamp_hash_aliases_not_resolved"},
            "reconsideration": predicate("UNKNOWN", "old_tier_unchanged_independent_provenance_missing"),
            "minimum_path_length_gate": predicate("UNKNOWN" if support["support_m"] is None else "PASS" if support["support_m"] >= .1 else "FAIL", "reported_support_at_least_0.1m_not_geometry_validity"),
            "path_supervision": predicate("UNKNOWN", "intent_and_environment_not_reviewed"),
            "permission": predicate("UNKNOWN", "explicit_permission_not_established"),
            "safety": predicate("NOT_INSPECTED", "no_clearance_or_vehicle_execution_audit")}
    grid = mapping(anchor.get("comparison")).get("common_grid_m")
    positive = sum(v > 0 for v in grid) if isinstance(grid, list) and all(finite(v) and v >= 0 for v in grid) else None
    original_boundary = mapping(anchor.get("local_window_boundaries"))
    new_boundary = scopes["h30"]["legacy_window_recomputed"]
    return {"sample_id": anchor.get("sample_id"), "run_id": run, "processing": "PROCESSED", "status": "UNKNOWN",
        "original_tier": anchor.get("tier"), "original_numeric_reproduction_status": repro.get("status", "UNKNOWN"),
        "independent_numeric_reproduction": predicate("NOT_INSPECTED", "saved_full_future_not_available_no_new_residuals"),
        "anchor_pose_dependency": base, "steps": analyzed, "scopes": scopes,
        "legacy_h30_status_and_flags_match": original_boundary.get("status") == new_boundary["status"] and sorted(original_boundary.get("flags", [])) == new_boundary["flags"],
        "common_grid_comparison": {"status": "NOT_INSPECTED" if not positive else "UNKNOWN", "comparability": "NOT_COMPARABLE" if positive == 0 else "REPORTED_POSITIVE_GRID_ONLY" if positive else "UNKNOWN",
            "positive_grid_points": positive, "new_residual_m": None, "old_residual_m": mapping(anchor.get("comparison")).get("common_grid_max_residual_m")},
        "anchor_issues": ([h + ":retained_step_time_inconsistent" for h in scopes if not scopes[h]["retained_metadata_consistent"]]
                          + [h + ":invalid_reported_arc" for h in scopes if not finite(mapping(mapping(mapping(anchor.get("comparison")).get("strict_diagnostic_v1")).get(h)).get("arc_m"))]),
        "new_teacher_or_runtime_adoption": False}


def analyze(raw: Mapping[str, list], anchors: Sequence[Mapping], reports: Mapping[str, Any],
            raw_hash: str, config: ConflictConfig) -> tuple[dict, dict, list, dict]:
    budget = Budget(config)
    records = inventory_records(raw, raw_hash)
    all_count = len(records)
    over_records = all_count > config.max_records
    if over_records: budget.reasons.add("record_limit_all_group_analysis_deferred")
    groups, lookup = build_groups([] if over_records else records, budget)
    by_id = {r["record_id"]: r for r in records}
    impacts = []
    for index, anchor in enumerate(anchors):
        if index >= config.max_anchors or not budget.available():
            budget.reasons.add("anchor_or_time_limit")
            impacts.append({"sample_id": mapping(anchor).get("sample_id"), "processing": "NOT_INSPECTED_LIMIT", "status": "NOT_INSPECTED"})
        else:
            impacts.append(anchor_impact(mapping(anchor), raw, lookup, by_id, reports.get("files", []), config))
    invalid_records = sum(bool(r["errors"]) for r in records)
    invalid_anchors = sum(r["processing"] == "INVALID_OR_LIMITED" or bool(r.get("anchor_issues")) for r in impacts)
    scope_counts = {}
    for h in ("h15", "h30"):
        scopes = [r["scopes"][h] for r in impacts if r["processing"] == "PROCESSED"]
        scope_counts[h] = {"anchors": len(scopes), "valid_target_dependency_status": dict(Counter(s["saved_valid_targets"]["dependency"]["status"] for s in scopes)),
            "anchors_with_observed_difference_in_valid_dependencies": sum(bool(s["saved_valid_targets"]["observed_difference_steps"]) for s in scopes),
            "anchors_with_observed_difference_in_strict_prefix": sum(bool(s["existing_strict_prefix"]["observed_difference_steps"]) for s in scopes),
            "support_knownness": dict(Counter(s["spatial_support"]["support_kind"] for s in scopes))}
    summary = {"status": "PARTIAL" if budget.reasons or invalid_records or invalid_anchors else "COMPLETE_DECLARED_SCOPE",
        "record_count": all_count, "invalid_record_count": invalid_records, "anchor_count": len(anchors),
        "anchor_processing": dict(Counter(r["processing"] for r in impacts)), "invalid_anchor_count": invalid_anchors,
        "unprocessed_record_count": all_count if over_records else 0,
        "topic_counts": dict(Counter(str(r["record"].get("topic")) for r in records)),
        "group_count": len(groups), "duplicate_group_count": sum(g["candidate_count"] > 1 for g in groups),
        "duplicate_classification": {kind: dict(Counter(label for g in groups if g["topic"] == kind and g["candidate_count"] > 1 for label in g["classification"])) for kind in (POSE, VELOCITY)},
        "legacy_h30_match_count": sum(r.get("legacy_h30_status_and_flags_match", False) for r in impacts),
        "original_reproduction": dict(Counter(r.get("original_numeric_reproduction_status", "NOT_INSPECTED") for r in impacts)),
        "scope_counts": scope_counts, "not_comparable_origin_only": sum(r.get("common_grid_comparison", {}).get("comparability") == "NOT_COMPARABLE" for r in impacts),
        "limit_reasons": sorted(budget.reasons), "pairs_measured": budget.pairs,
        "gates": {"geometry_only": "BLOCKED_INDEPENDENT_DOMAIN_ORDER_COMPLETENESS", "stop_teacher": "BLOCKED_INTENT_PERMISSION", "controller_mpc_oracle": "BLOCKED_ENVIRONMENT_VEHICLE_POLICY"},
        "new_geometric_teacher_count": 0, "raw_reads": 0, "dataset_reads": 0,
        "elapsed_analysis_sec": time.monotonic() - budget.started}
    counts_by_run = []
    for run in sorted(raw):
        selected = [g for g in groups if g["run_id"] == run and g["topic"] == POSE and g["candidate_count"] > 1]
        pose_present = any(mapping(r).get("topic") == POSE for r in raw[run])
        counts_by_run.append({"run_id": run, "pose_duplicate_groups": len(selected) if pose_present else None,
            "source_payload_observed": pose_present, "count_scope": "saved_pose_records_only_not_source_completeness",
            "first_to_later_legacy_pairs": sum(g["legacy_xy_gt_1e8"]["first_to_later_count"] or 0 for g in selected) if pose_present else None,
            "all_pair_legacy_pairs": sum(g["legacy_xy_gt_1e8"]["all_pairs_count"] or 0 for g in selected) if pose_present else None,
            "counts_complete": pose_present and all(g["all_pair_maximum_status"] == "PASS" for g in selected) and not over_records})
    summary["legacy_count_definitions_by_run"] = counts_by_run
    return {"records": records, "record_index_semantics": "input_hash_run_id_original_JSON_array_index_only"}, {"groups": groups}, impacts, summary


def _unique_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result = {}
    for key, value in pairs:
        if key in result: raise ValueError("duplicate JSON object key")
        result[key] = value
    return result


def write_json(path: Path, value: Any) -> None:
    with path.open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(clean(value), stream, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False)
        stream.write("\n")


def run_audit(evidence_root: Path, output: Path, repo: Path, config: ConflictConfig,
              *, expected_hashes: Mapping[str, str] = EXPECTED_HASHES,
              expected_dataset: str = DATASET_ID, expected_count: int = 29,
              command: Sequence[str] = ()) -> dict[str, Any]:
    """Only ALLOWLIST leaves are opened. Embedded commands/paths are opaque text."""
    config.validate()
    root, output = evidence_root.resolve(), output.resolve()
    if output == root or output.is_relative_to(root) or root.is_relative_to(output):
        raise ValueError("input/output overlap")
    if output.exists(): raise FileExistsError("immutable output already exists")
    inventory, inputs, blockers, partial = [], {}, [], []
    total = 0
    for name in ALLOWLIST:
        path = root / name
        if path.is_symlink() or path.resolve().parent != root:
            blockers.append("unsafe_allowlist_leaf:" + name)
            continue
        if not path.is_file():
            inventory.append({"name": name, "status": "MISSING"})
            if name in REQUIRED: blockers.append("required_missing:" + name)
            continue
        size = path.stat().st_size
        entry = {"name": name, "size_bytes": size, "expected_sha256": expected_hashes.get(name),
                 "independent_prior_hash_binding": name in expected_hashes}
        inventory.append(entry)
        if size > config.max_file_bytes or total + size > config.max_total_bytes:
            entry["status"] = "NOT_INSPECTED_SIZE_LIMIT"
            partial.append("JSON_size_limit:" + name)
            continue
        # Bounded read; no further file names can originate from JSON contents.
        with path.open("rb") as stream: data = stream.read(config.max_file_bytes + 1)
        total += len(data)
        if len(data) > config.max_file_bytes:
            partial.append("JSON_changed_size_limit:" + name)
            continue
        digest = hashlib.sha256(data).hexdigest()
        entry.update(sha256=digest, status="READ")
        if name in expected_hashes and digest != expected_hashes[name]:
            blockers.append("hash_mismatch:" + name)
            continue
        try:
            value = json.loads(data, object_pairs_hook=_unique_object)
            inputs[name] = value
            entry.update(root_type=type(value).__name__, root_count=len(value) if isinstance(value, (list, dict)) else None)
        except (ValueError, UnicodeError, RecursionError) as error:
            entry["status"] = "INVALID_JSON"
            blockers.append(f"invalid_json:{name}:{type(error).__name__}")
    old = mapping(inputs.get("execution_manifest.json"))
    if old and (old.get("code_commit") != OLD_COMMIT or old.get("dataset_identity") != expected_dataset or old.get("selected_anchor_count") != expected_count):
        blockers.append("old_execution_identity_or_count_mismatch")
    if expected_hashes == EXPECTED_HASHES and old:
        expected_config = {"interpolation_tolerance_ms": 50.0, "position_tolerance_m": 2e-5,
            "yaw_tolerance_rad": 1e-5, "speed_tolerance_mps": 1e-5, "distance_grid_m": .1,
            "maximum_gap_sec": .2, "hold_sec": .5, "noise_radius_m": .005}
        if any(mapping(old.get("configuration")).get(k) != v for k, v in expected_config.items()) or old.get("all_anchor_count") != 72697 or old.get("val_stopped_commanded_tracked") != 530 or old.get("selected_group_count") != 8:
            blockers.append("old_configuration_or_cohort_mismatch")
    raw, anchors, reports = inputs.get("raw_window_evidence.json"), inputs.get("anchor_evidence.json"), inputs.get("raw_read_report.json")
    if all(name in inputs for name in REQUIRED) and not blockers:
        if not isinstance(raw, dict) or not all(isinstance(v, list) for v in raw.values()) or not isinstance(anchors, list) or not isinstance(reports, dict) or not isinstance(reports.get("files"), list) or not all(isinstance(f, dict) for f in reports["files"]):
            blockers.append("invalid_input_root_schema")
        elif len(anchors) != old["selected_anchor_count"] or sum(len(v) for v in raw.values()) != mapping(old.get("raw_actual")).get("decoded_messages"):
            blockers.append("record_or_anchor_count_mismatch")
        elif len({mapping(a).get("sample_id") for a in anchors}) != len(anchors):
            blockers.append("duplicate_or_missing_anchor_ids")
    if "selection.json" in inputs and isinstance(anchors, list):
        selected = mapping(inputs["selection.json"]).get("selected", [])
        if not isinstance(selected, list) or {mapping(a).get("sample_id") for a in selected} != {mapping(a).get("sample_id") for a in anchors}:
            blockers.append("optional_selection_anchor_join_mismatch")
    if blockers or partial:
        recs, groups, impacts = {"records": []}, {"groups": []}, []
        summary = {"status": "BLOCKED" if blockers else "PARTIAL", "blockers": blockers, "limit_reasons": partial,
                   "unprocessed_anchor_count": len(anchors) if isinstance(anchors, list) else None}
    else:
        recs, groups, impacts, summary = analyze(raw, anchors, reports,
            next(e["sha256"] for e in inventory if e["name"] == "raw_window_evidence.json"), config)
    for entry in inventory:
        if "sha256" in entry:
            entry["unchanged_after"] = sha256_file(root / entry["name"]) == entry["sha256"]
            if not entry["unchanged_after"]: blockers.append("input_changed:" + entry["name"])
    if blockers: summary.update(status="BLOCKED", blockers=blockers)
    def git(*args: str) -> str:
        return subprocess.check_output(["git", "-C", str(repo), *args], text=True).strip()
    code = git("rev-parse", "HEAD")
    logical_summary = {k: v for k, v in summary.items() if k != "elapsed_analysis_sec"}
    code_files = (Path(__file__).resolve(), repo / "tools/audit_spatial_pose_conflicts_v4.py")
    code_hashes = {p.name: sha256_file(p) for p in code_files}
    logical_id = identity({"policy": VERSION, "configuration": asdict(config), "inputs": {e["name"]: e.get("sha256") for e in inventory},
                           "code_hashes": code_hashes, "summary": logical_summary, "groups": groups, "impacts": impacts})
    manifest = {"format": VERSION, "status": summary["status"], "old_execution_commit": old.get("code_commit"),
        "reclassification_commit": code, "working_tree": git("status", "--porcelain"), "configuration": asdict(config),
        "code_hashes": code_hashes, "input_files": inventory, "old_dataset_identity_reported_not_dataset_read": old.get("dataset_identity"),
        "old_configuration_reported": old.get("configuration"), "logical_identity": logical_id,
        "command": list(command), "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "environment": {"python": platform.python_version(), "platform": platform.platform()},
        "prohibited_actions_performed": [], "raw_reader_executed": False, "dataset_accessed": False,
        "input_unchanged": all(e.get("unchanged_after", True) for e in inventory),
        "exit_code": 0 if summary["status"] == "COMPLETE_DECLARED_SCOPE" else 2 if summary["status"] == "PARTIAL" else 3}
    output.mkdir(parents=True)
    write_json(output / "input_inventory.json", {"files": inventory, **recs})
    write_json(output / "pose_stamp_groups.json", groups)
    write_json(output / "anchor_prefix_impact.json", impacts)
    write_json(output / "summary.json", summary)
    write_json(output / "execution_manifest.json", manifest)
    text = (f"# 保存済みpose競合・prefix影響監査\n\n状態: {summary['status']}\n\n"
        f"旧実行commit: {old.get('code_commit')}\n新再分類commit: {code}\n\n"
        "既存の数値再現PASS/legacy FAILは元版の報告と区別して保持。新しいXY・実長・教師は生成しない。\n"
        "source/clock epoch/物理順序/抽出完全性が未証明なら、依存先の投影値が等しくても昇格しない。\n"
        "先頭欠損は支持null、原点のみgridはNOT_COMPARABLE。holdは旧縦速度policyで、停止意図ではない。\n\n"
        "geometry-only採用、停止教師、controller/MPC oracleは独立にBLOCKED。\n"
        "具体的件数はsummary.json、群はpose_stamp_groups.json、依存関係はanchor_prefix_impact.json。\n"
        "次に必要な証拠: clock/source domain、epoch、channel/schema binding、順序/offset、publisher識別。\n"
        "今回、新規raw読取・Dataset操作・学習・推論・走行・pushは実施していない。\n")
    with (output / "report_ja.md").open("x", encoding="utf-8", newline="\n") as stream: stream.write(text)
    return manifest
