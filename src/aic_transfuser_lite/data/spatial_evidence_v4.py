"""Versioned evidence sidecar, not training labels or controller/oracle input."""
from __future__ import annotations

from bisect import bisect_left
from collections import Counter, defaultdict
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import csv
import json
import math
from pathlib import Path
import platform
import subprocess
from typing import Any, Iterable, Mapping, Sequence
from urllib.parse import unquote, urlparse

import numpy as np

from .spatial_coverage_v4 import (
    DISTANCES_M, SpatialAuditConfig, csv_rows, future_geometry, identity, read_yaml,
    resample_no_extrapolation, sha256_file, source_entry, validate_sources, write_csv, write_json,
)
from .spatial_source_reader_v4 import ReadBudget, ReadMeter, TOPICS, read_mcap_windows

PREVIOUS_IMPLEMENTATION = "2a2558749706e7554362d3d49613d92fbe3030f6"
EXPECTED = {
    "dataset_identity": "181cf909b80589110574859990b0885005b7f9a0bb07cff1c24f38d6b090f388",
    "dataset_manifest": "d625f42ca05a18ea76952376c6392268191c4895d6e605e0c49ceaa66dcbe1de",
    "split": "7d0e433dbd032ad695227051573e7d8d17072fa4ea3b4e28f4c44f56fde27b4f",
    "previous_manifest": "458f6ebb5b44d1948e6264521ecf8d1e9f1217df8c9d59bf9a37bc238c55ed13",
    "previous_ledger": "35781616e8faab5117b0d9da7c8560519c5f196383ea66ab1465d999f5645e35",
}
VERSION = "spatial_evidence_v4_v1"
FIELDS = ("sample_id", "run_id", "split", "segment_id", "grid_stamp_ns", "source_uri", "source_hash",
          "estimated_episode_id", "normal_recovery", "side", "near_far", "phase", "geometry",
          "v3_motion_assessment", "h15_prefix_count", "h15_future_status", "h30_future_status",
          "h15_raw_reaches_1m", "h30_raw_reaches_1m", "h15_raw_arc_m", "h30_raw_arc_m")


@dataclass(frozen=True)
class EvidenceConfig:
    max_episodes: int = 8
    max_anchors: int = 32
    before_sec: float = 1.0
    after_sec: float = 3.1
    bag_header_margin_sec: float = 0.25
    interpolation_tolerance_ms: float = 50.0
    position_tolerance_m: float = 2e-5
    yaw_tolerance_rad: float = 1e-5
    speed_tolerance_mps: float = 1e-5
    distance_grid_m: float = 0.1
    maximum_gap_sec: float = 0.2
    hold_sec: float = 0.5
    noise_radius_m: float = 0.005

    def validate(self) -> None:
        for key, value in asdict(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"invalid evidence config: {key}")
        if self.after_sec < 3.05:
            raise ValueError("after_sec must cover h30 and interpolation endpoints")


def state(status: str, reason: str, *, evidence: Any = None) -> dict[str, Any]:
    if status not in {"PASS", "FAIL", "UNKNOWN", "NOT_INSPECTED"}:
        raise ValueError("unsupported evidence status")
    return {"status": status, "reason": reason, "evidence": evidence, "validator": VERSION}


def _true(value: str | None) -> bool:
    return str(value).lower() == "true"


def tags(row: Mapping[str, str]) -> set[str]:
    result = {f"run:{row['run_id']}"}
    stopped = bool(row.get("estimated_episode_id"))
    if stopped:
        result.add("stopped_commanded")
        if _true(row.get("h30_raw_reaches_1m")):
            result.add("stopped_h30_1m")
        elif row.get("h30_raw_reaches_1m"):
            result.add("stopped_short_both")
    if row.get("v3_motion_assessment") == "censored_future":
        result.add("censored")
    if row.get("h15_prefix_count") == "0":
        result.add("first_future_missing")
    if row.get("normal_recovery") == "recovery" and not _true(row.get("h15_raw_reaches_1m")) and _true(row.get("h30_raw_reaches_1m")):
        result.add(f"growth:{row['side']}:{row['near_far']}")
    if row.get("phase") == "hold":
        result.add("offset_hold_not_stop_intent")
    return result


def choose_anchors(rows: Sequence[Mapping[str, str]], config: EvidenceConfig) -> dict[str, Any]:
    """Deterministic coverage-first selection; includes failures, never test geometry.

    Stop episode IDs are inherited. Supplemental growth windows are explicitly
    diagnostic groups, NOT additional independent stop episodes.
    """
    config.validate()
    groups: dict[str, list[Mapping[str, str]]] = defaultdict(list)
    for row in rows:
        if row["split"] != "val":
            continue
        if row.get("estimated_episode_id"):
            group = row["estimated_episode_id"]
        elif any(t.startswith("growth:") for t in tags(row)):
            group = f"diagnostic_growth_window:{row['run_id']}:{row['phase']}"
        else:
            continue
        groups[group].append(row)
    wanted = {t for group in groups.values() for row in group for t in tags(row)}
    wanted.discard("stopped_commanded")
    selected: list[dict[str, Any]] = []
    chosen_groups: list[str] = []
    covered: set[str] = set()
    # Greedy across candidate groups; run coverage first, rare cases second.
    def weight(tag: str) -> int:
        return 100 if tag.startswith("run:") else 20 if tag in {"stopped_h30_1m", "first_future_missing"} else 10
    for _ in range(config.max_episodes):
        remaining = [g for g in sorted(groups) if g not in chosen_groups]
        if not remaining or len(selected) >= config.max_anchors:
            break
        group = min(remaining, key=lambda g: (-sum(weight(t) for t in
                    set().union(*(tags(r) for r in groups[g])) - covered - {"stopped_commanded"}), g))
        candidates = sorted(groups[group], key=lambda r: (int(r["grid_stamp_ns"]), r["sample_id"]))
        chosen_groups.append(group)
        for _ in range(min(4, config.max_anchors - len(selected))):
            if not candidates:
                break
            row = min(candidates, key=lambda r: (-sum(weight(t) for t in tags(r) - covered - {"stopped_commanded"}),
                                                int(r["grid_stamp_ns"]), r["sample_id"]))
            candidates.remove(row)
            covered.update(tags(row))
            selected.append({"sample_id": row["sample_id"], "run_id": row["run_id"],
                             "group_id": group, "group_kind": "diagnostic_window" if group.startswith("diagnostic_") else "inferred_stop_episode",
                             "selection_tags": sorted(tags(row)), "rank": len(selected) + 1})
    return {"selected": selected, "group_count": len(chosen_groups), "groups": chosen_groups,
            "priority": "run coverage; rare stopped h30/first missing; other failure/growth/hold slices; timestamp/ID tie break",
            "covered_tags": sorted(covered), "uncovered_available_tags": sorted(wanted - covered),
            "absent_required_slices": ["right_near_val"] if not any("growth:right:near" in tags(r) for r in rows if r["split"] == "val") else [],
            "max_episodes_is_work_budget_not_independence_gate": True}


def interpolate_records(records: Sequence[Mapping[str, Any]], target_ns: int, *,
                        fields: Sequence[str], tolerance_ns: int, pose: bool = False) -> tuple[dict[str, Any] | None, dict[str, Any]]:
    """Independent linear interpolation, shortest wrapped yaw; nanoseconds/SI."""
    by_stamp = {int(r["semantic_stamp_ns"]): r for r in records}
    stamps = sorted(by_stamp)
    index = bisect_left(stamps, target_ns)
    if index < len(stamps) and stamps[index] == target_ns:
        left = right = by_stamp[stamps[index]]
    elif index == 0 or index == len(stamps):
        return None, {"reason": "outside_observed_source_window", "target_ns": target_ns}
    else:
        left, right = by_stamp[stamps[index - 1]], by_stamp[stamps[index]]
    lo, hi = int(left["semantic_stamp_ns"]), int(right["semantic_stamp_ns"])
    evidence = {"target_ns": target_ns, "source_stamps_ns": [lo, hi],
                "source_payload_hashes": [left["payload_sha256"], right["payload_sha256"]],
                "maximum_endpoint_delta_ms": max(abs(lo - target_ns), abs(hi - target_ns)) / 1e6,
                "interpolation_gap_ms": (hi - lo) / 1e6}
    if max(abs(lo - target_ns), abs(hi - target_ns)) > tolerance_ns:
        return None, {**evidence, "reason": "interpolation_tolerance_exceeded"}
    if pose and any(left["value"].get(k) != right["value"].get(k) for k in ("frame_id", "child_frame_id")):
        return None, {**evidence, "reason": "frame_mismatch"}
    alpha = (target_ns - lo) / (hi - lo) if hi != lo else 0.0
    out = {}
    for field in fields:
        a, b = left["value"].get(field), right["value"].get(field)
        if a is None or b is None or not math.isfinite(float(a)) or not math.isfinite(float(b)):
            return None, {**evidence, "reason": "nonfinite_source"}
        delta = float(b) - float(a)
        if field == "yaw_rad":
            delta = math.atan2(math.sin(delta), math.cos(delta))
        value = float(a) + alpha * delta
        out[field] = math.atan2(math.sin(value), math.cos(value)) if field == "yaw_rad" else value
    if pose:
        out.update({k: left["value"].get(k) for k in ("frame_id", "child_frame_id")})
    return out, {**evidence, "reason": "interpolated"}


def reproduce_future(future: np.ndarray, sample: Mapping[str, str], records: Sequence[Mapping[str, Any]],
                     config: EvidenceConfig) -> dict[str, Any]:
    """Reconstruct from raw header-stamped pose/velocity, never other anchor future."""
    values = np.asarray(future, dtype=np.float64)
    if values.shape != (30, 8):
        raise ValueError("saved future must be [30,8]")
    obs = int(sample["grid_stamp_ns"]) + round(float(sample["camera_delta_ms"]) * 1e6)
    poses = [r for r in records if r["topic"] == "/localization/kinematic_state"]
    velocities = [r for r in records if r["topic"] == "/vehicle/status/velocity_status"]
    tol = round(config.interpolation_tolerance_ms * 1e6)
    base, base_evidence = interpolate_records(poses, obs, fields=("x_m", "y_m", "yaw_rad"), tolerance_ns=tol, pose=True)
    results = []
    for index, saved in enumerate(values):
        target = obs + round((index + 1) * .1 * 1e9)
        p, p_e = interpolate_records(poses, target, fields=("x_m", "y_m", "yaw_rad"), tolerance_ns=tol, pose=True)
        v, v_e = interpolate_records(velocities, target, fields=("longitudinal_mps", "lateral_mps", "yaw_rate_rps"), tolerance_ns=tol)
        item: dict[str, Any] = {"step": index + 1, "saved_valid": bool(saved[7] == 1),
            "pose": p_e, "velocity": v_e, "position_residual_m": None, "yaw_residual_rad": None,
            "speed_residual_mps": None, "status": "UNKNOWN"}
        if base is None or p is None or v is None:
            item["reason"] = "anchor_pose_unavailable" if base is None else "pose_" + p_e["reason"] if p is None else "velocity_" + v_e["reason"]
        elif any(base[k] != p[k] for k in ("frame_id", "child_frame_id")):
            item.update(status="FAIL", reason="frame_changed_since_observation")
        elif saved[7] != 1:
            item["reason"] = "saved_invalid_despite_available_sources_epoch_or_conversion_context_unknown"
        else:
            dx, dy = p["x_m"] - base["x_m"], p["y_m"] - base["y_m"]
            c, s = math.cos(base["yaw_rad"]), math.sin(base["yaw_rad"])
            expected = np.array([c * dx + s * dy, -s * dx + c * dy])
            position_error = float(np.linalg.norm(saved[1:3] - expected))
            yaw_error = abs(math.atan2(math.sin(saved[3] - p["yaw_rad"] + base["yaw_rad"]), math.cos(saved[3] - p["yaw_rad"] + base["yaw_rad"])))
            speed_error = abs(float(saved[4]) - v["longitudinal_mps"])
            secondary_error = max(abs(float(saved[5]) - v["lateral_mps"]), abs(float(saved[6]) - v["yaw_rate_rps"]))
            okay = (np.isfinite(saved[:7]).all() and position_error <= config.position_tolerance_m
                and yaw_error <= config.yaw_tolerance_rad and speed_error <= config.speed_tolerance_mps
                and secondary_error <= config.speed_tolerance_mps)
            item.update(status="PASS" if okay else "FAIL", reason="float32_tolerance_match" if okay else "source_residual_exceeds_tolerance",
                position_residual_m=position_error if math.isfinite(position_error) else None,
                yaw_residual_rad=yaw_error if math.isfinite(yaw_error) else None,
                speed_residual_mps=speed_error if math.isfinite(speed_error) else None)
        results.append(item)
    valid_results = [r for r in results if r["saved_valid"]]
    result_status = "FAIL" if any(r["status"] == "FAIL" for r in valid_results) else "PASS" if valid_results and all(r["status"] == "PASS" for r in valid_results) else "UNKNOWN"
    return {"status": result_status, "t_obs_ns": obs, "t_obs_source": "grid_stamp_ns + camera_delta_ms(source-grid)",
            "camera_header_directly_decoded": False, "anchor_pose": base, "anchor_interpolation": base_evidence,
            "steps": results, "tolerance_status": "float32_derived_provisional_not_sensor_calibration",
            "saved_relative_time_max_residual_ns": float(np.max(np.abs(values[:, 0] - np.arange(1, 31) * .1))) * 1e9}


def observed_boundaries(records: Sequence[Mapping[str, Any]], start_ns: int, end_ns: int,
                        config: EvidenceConfig) -> dict[str, Any]:
    """Source-connected local-window checks; no claims about unobserved resets/intent."""
    poses = sorted([r for r in records if r["topic"] == "/localization/kinematic_state"
                    and start_ns - 50000000 <= r["semantic_stamp_ns"] <= end_ns + 50000000], key=lambda r: r["bag_stamp_ns"])
    flags: list[str] = []
    for left, right in zip(poses, poses[1:]):
        dt = (right["semantic_stamp_ns"] - left["semantic_stamp_ns"]) / 1e9
        if dt < 0:
            flags.append("pose_timestamp_reversal")
        if dt > config.maximum_gap_sec:
            flags.append("pose_gap")
        if any(left["value"].get(k) != right["value"].get(k) for k in ("frame_id", "child_frame_id")):
            flags.append("frame_change")
        dx = right["value"]["x_m"] - left["value"]["x_m"]
        dy = right["value"]["y_m"] - left["value"]["y_m"]
        if dt > 0 and math.hypot(dx, dy) / dt > 20:
            flags.append("teleport_provisional_20mps")
        if dt == 0 and math.hypot(dx, dy) > 1e-8:
            flags.append("conflicting_duplicate_pose_stamp")
    clocks = sorted([r for r in records if r["topic"] == "/clock" and start_ns <= r["bag_stamp_ns"] <= end_ns], key=lambda r: r["bag_stamp_ns"])
    if any(b["value"]["clock_ns"] < a["value"]["clock_ns"] for a, b in zip(clocks, clocks[1:])):
        flags.append("clock_reset")
    return {"status": "FAIL" if flags else "PASS" if len(poses) >= 2 and len(clocks) >= 2 else "UNKNOWN",
            "flags": sorted(set(flags)), "pose_records": len(poses), "clock_records": len(clocks),
            "physical_reset_without_clock_change": "UNKNOWN", "route_intent_change": "UNKNOWN",
            "kinematic_vehicle_limits": "UNKNOWN", "window_start_ns": start_ns, "window_end_ns": end_ns}


def strict_polyline(future: np.ndarray, steps: int, config: EvidenceConfig,
                    boundary_events: Sequence[tuple[float, str]] = ()) -> tuple[np.ndarray, dict[str, Any]]:
    """Causal identical h15/h30 rules, SI units. No filling gaps or extrapolation.

    Offset-hold annotation is never a boundary. Only speed-observed long hold
    or explicit source events can cut geometry. Cutoffs are not negative labels.
    """
    values = np.asarray(future, dtype=np.float64)
    if values.shape != (30, 8):
        raise ValueError("future requires [30,8]")
    points, last_t, hold_time = [np.zeros(2)], 0.0, 0.0
    previous_xy = np.zeros(2)
    reason, retained = "horizon_limit", 0
    for row in values[:steps]:
        if row[7] != 1 or not np.isfinite(row[:7]).all():
            reason = "invalid_future_gap"
            break
        dt = row[0] - last_t
        if dt <= 0 or dt > config.maximum_gap_sec + 1e-7:
            reason = "timestamp_boundary"
            break
        if np.linalg.norm(row[1:3] - previous_xy) / dt > 20:
            reason = "teleport_provisional_20mps"
            break
        event = next((r for t, r in boundary_events if t <= row[0]), None)
        if event:
            reason = event
            break
        if row[4] < -.01:
            reason = "reverse_motion"
            break
        hold_time = hold_time + dt if abs(row[4]) <= .01 else 0.0
        if hold_time >= config.hold_sec - 1e-7:
            reason = "observed_long_hold_not_intent"
            break
        if np.linalg.norm(row[1:3] - points[-1]) >= config.noise_radius_m:
            points.append(row[1:3])
        retained += 1
        last_t = float(row[0])
        previous_xy = row[1:3]
    xy = np.asarray(points)
    arc = float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum())
    return xy, {"arc_m": arc, "elapsed_sec": last_t, "retained_steps": retained,
                "endpoint_xy_m": xy[-1].tolist(), "cut_reason": reason,
                "negative_continuation_label": None, "safe_stop_endpoint": None,
                "noise_status": "provisional_last_reliable_point_5mm",
                "reaches": {f"{d:g}m": arc >= d for d in DISTANCES_M}}


def compare_horizons(future: np.ndarray, config: EvidenceConfig) -> dict[str, Any]:
    raw = {f"h{h}": future_geometry(future, SpatialAuditConfig(), horizon_sec=h / 10) for h in (15, 30)}
    p15, a = strict_polyline(future, 15, config)
    p30, b = strict_polyline(future, 30, config)
    # Compare on a single common distance grid, not each horizon's terminal grid.
    common = min(a["arc_m"], b["arc_m"])
    grid = np.arange(0.0, common + 1e-12, config.distance_grid_m)
    def on_grid(points: np.ndarray) -> np.ndarray:
        arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
        return np.column_stack([np.interp(grid, arc, points[:, k]) for k in (0, 1)])
    residual = float(np.linalg.norm(on_grid(p15) - on_grid(p30), axis=1).max())
    return {"source_comparison": "same_saved_future_h15_vs_h30_not_reconstructed_long_pose",
            "raw_v1": raw, "strict_diagnostic_v1": {"h15": a, "h30": b},
            "common_grid_m": grid.tolist(), "common_grid_max_residual_m": residual,
            "common_prefix_agrees": residual <= 1e-9,
            "raw_horizon_gains": {f"{d:g}m": bool(raw['h30'][f'raw_reaches_{d:g}m']) and not bool(raw['h15'][f'raw_reaches_{d:g}m']) for d in DISTANCES_M},
            "rule_change_separate_from_horizon_change": True}


def stopping_context(sample: Mapping[str, str], ledger: Mapping[str, str], records: Sequence[Mapping[str, Any]],
                     obs_ns: int) -> dict[str, Any]:
    """Positive request != permission. Reasons may co-occur; offset hold != stop."""
    commands = {name: json.loads(sample[name]) for name in ("nominal_command", "final_command")}
    normalized_commands = {name: {k: (None if isinstance(v, float) and not math.isfinite(v) else v) for k, v in value.items()} for name, value in commands.items()}
    selected = next(((name, value) for name, value in commands.items() if value.get("valid")
                     and all(math.isfinite(float(value[k])) for k in ("steering_rad", "speed_mps", "acceleration_mps2"))), None)
    request = state("UNKNOWN", "command_not_recorded") if selected is None else state(
        "PASS" if selected[1]["speed_mps"] > 0 else "FAIL", "recorded_command_request_not_permission",
        evidence={"source": selected[0], "timestamp_ns": selected[1].get("source_stamp_ns"), "value_mps": selected[1]["speed_mps"]})
    latest: dict[str, Mapping[str, Any]] = {}
    for rec in records:
        age = obs_ns - rec["semantic_stamp_ns"]
        if 0 <= age <= 350000000:
            if rec["topic"] not in latest or rec["semantic_stamp_ns"] > latest[rec["topic"]]["semantic_stamp_ns"]:
                latest[rec["topic"]] = rec
    safety = latest.get("/safety_reason")
    safety_state = state("UNKNOWN", "no_fresh_record_is_not_safety_clear") if safety is None else state("PASS", "explicit_safety_record_not_clearance_proof", evidence=safety)
    return {"commands": normalized_commands, "driving_request": request,
        "driving_permission": state("UNKNOWN", "complete_timestamped_preflight_not_available"),
        "intentional_stop_or_wait": state("UNKNOWN", "zero_command_or_offset_hold_does_not_identify_intent"),
        "safety_record": safety_state, "fault": state("UNKNOWN", "controller_fault_not_established"),
        "actuator_response_cause": state("UNKNOWN", "observed_nonmotion_does_not_establish_actuator_fault"),
        "mode_and_state_records": [r for t, r in latest.items() if t in {"/awsim/state", "/vehicle/status/control_mode", "/vehicle/status/gear_status", "/overtake/race_armed"}],
        "phase": ledger.get("phase", "unknown"), "offset_hold_is_stop_intent": False,
        "route_intent": state("UNKNOWN", "planned_reference_not_runtime_route_input"),
        "clearance": state("NOT_INSPECTED", "camera_lidar_environment_not_decoded")}


def evidence_for_anchor(future: np.ndarray, sample: Mapping[str, str], old: Mapping[str, str],
                        records: Sequence[Mapping[str, Any]], complete_read: bool,
                        config: EvidenceConfig) -> dict[str, Any]:
    comparison = compare_horizons(future, config)
    repro = reproduce_future(future, sample, records, config)
    obs = repro["t_obs_ns"]
    boundaries = observed_boundaries(records, obs, obs + 3000000000, config)
    base = repro["anchor_pose"]
    frame = state("UNKNOWN", "source_frame_not_available") if base is None else state(
        "PASS" if base.get("frame_id") and base.get("child_frame_id") == "base_link" else "FAIL",
        "raw_pose_frame_and_child_checked_rear_axle_unknown", evidence=base)
    timing = state("PASS" if repro["saved_relative_time_max_residual_ns"] <= 250 else "FAIL",
                   "saved_float32_grid_residual_limit_250ns", evidence=repro["saved_relative_time_max_residual_ns"])
    timed = [r for r in records if r["topic"] in {"/localization/kinematic_state", "/vehicle/status/velocity_status"}]
    maximum_offset = max((abs(r["semantic_stamp_ns"] - r["bag_stamp_ns"]) for r in timed), default=None)
    mapping = state("UNKNOWN" if not timed else "PASS" if maximum_offset <= config.bag_header_margin_sec * 1e9
                    and all(r["timestamp_source"] in {"header.stamp", "stamp"} for r in timed) else "FAIL",
                    "decoded_header_vs_bag_record_within_explicit_margin", evidence={"max_abs_delta_ns": maximum_offset})
    good = complete_read and repro["status"] == "PASS" and boundaries["status"] == "PASS" and frame["status"] == "PASS" and timing["status"] == "PASS" and mapping["status"] == "PASS"
    length = comparison["strict_diagnostic_v1"]["h30"]["arc_m"]
    tier = "GEOMETRY_VERIFIED" if good and length >= .1 else "OBSERVED_ONLY"
    reasons = []
    if not complete_read:
        reasons.append("raw_window_incomplete_or_not_inspected")
    for name, status in (("source_reproduction", repro["status"]), ("boundary", boundaries["status"]), ("frame", frame["status"]), ("bag_header_mapping", mapping["status"])):
        if status != "PASS":
            reasons.append(f"{name}:{status}")
    if length < .1:
        reasons.append("insufficient_stable_spatial_support")
    return {"sample_id": sample["sample_id"], "run_id": sample["run_id"], "split": old["split"],
        "tier": tier, "tier_scope": "strict_contiguous_prefix_under_provisional_noise_and_boundary_checks_only",
        "geometry_observed": state("PASS" if np.any(future[:, 7] == 1) else "FAIL", "stored_h30_measured_future"),
        "timestamp_alignment": timing, "bag_header_mapping": mapping, "frame_alignment": frame, "source_reproduction": repro,
        "local_window_boundaries": boundaries, "comparison": comparison,
        "strict_verified_reaches": {h: (comparison["strict_diagnostic_v1"][h]["reaches"] if tier == "GEOMETRY_VERIFIED" else None) for h in ("h15", "h30")},
        "motion_observed": state("UNKNOWN" if not np.any(future[:, 7] == 1) else "PASS" if np.nanmax(np.linalg.norm(future[:, 1:3], axis=1), initial=0) >= .1 else "FAIL", "observed_displacement_threshold_0.1m_not_permission"),
        "context": stopping_context(sample, old, records, obs), "geometry_tier_missing_evidence": reasons,
        "path_supervision": state("UNKNOWN", "intent_environment_training_policy_not_reviewed"),
        "safety_verified": state("UNKNOWN", "geometry_is_not_safe_execution"),
        "negative_continuation_label": None}


def _metadata_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return [{k: r.get(k, "") for k in FIELDS} for r in csv.DictReader(stream)]


def _merge_windows(windows: Sequence[tuple[int, int]]) -> list[tuple[int, int]]:
    result: list[tuple[int, int]] = []
    for lo, hi in sorted(windows):
        if result and lo <= result[-1][1]:
            result[-1] = (result[-1][0], max(hi, result[-1][1]))
        else:
            result.append((lo, hi))
    return result


def make_plan(selection: Mapping[str, Any], old_rows: Mapping[str, Mapping[str, str]],
              samples: Mapping[str, Mapping[str, str]], config: EvidenceConfig,
              budget: ReadBudget) -> dict[str, Any]:
    by_run: dict[str, list[tuple[int, int]]] = defaultdict(list)
    roots: dict[str, Path] = {}
    for chosen in selection["selected"]:
        sid = chosen["sample_id"]
        row, sample = old_rows[sid], samples[sid]
        stamp = int(sample["grid_stamp_ns"]) + round(float(sample["camera_delta_ms"]) * 1e6)
        by_run[row["run_id"]].append((max(0, stamp - round((config.before_sec + config.bag_header_margin_sec) * 1e9)),
                                      stamp + round((config.after_sec + config.bag_header_margin_sec) * 1e9)))
        uri = urlparse(row["source_uri"])
        if uri.scheme != "file":
            raise ValueError("unsupported non-local raw source")
        roots[row["run_id"]] = Path(unquote(uri.path))
    runs = []
    for run_id, windows in sorted(by_run.items()):
        raw_root = roots[run_id]
        metadata = source_entry("metadata", raw_root / "metadata.yaml")
        files, available = [], []
        if metadata["status"] == "PRESENT":
            meta = read_yaml(raw_root / "metadata.yaml")["rosbag2_bagfile_information"]
            available = [{"topic": t["topic_metadata"]["name"], "type": t["topic_metadata"]["type"], "message_count": t["message_count"]}
                         for t in meta["topics_with_message_count"] if t["topic_metadata"]["name"] in TOPICS]
            for name in meta["relative_file_paths"]:
                path = (raw_root / name).resolve()
                if not path.is_relative_to(raw_root.resolve()):
                    raise ValueError("bag path escapes source root")
                files.append({"path": str(path), "size_bytes": path.stat().st_size if path.exists() else None,
                    "status": "PRESENT" if path.exists() else "MISSING",
                    "read_strategy": "bounded_forward_file_zstd" if path.suffix in {".zstd", ".zst"} else "bounded_index_if_available",
                    "seek_index_status": "NOT_INSPECTED"})
        runs.append({"run_id": run_id, "raw_root": str(raw_root), "metadata": metadata,
                     "files": files, "available_topics": available, "windows_bag_ns": _merge_windows(windows)})
    return {"format": VERSION + "_read_plan", "selection": selection, "configuration": asdict(config),
            "budget": asdict(budget), "runs": runs, "topics": list(TOPICS),
            "time_mapping_assumption": "canonical observation header approx bag log within explicit margin; validated after read",
            "temporary_disk_policy": "no_extracted_bag_or_sensor_files", "test_geometry": False}


def summarize_evidence(evidence: Sequence[Mapping[str, Any]], keys: Sequence[str],
                       old_rows: Mapping[str, Mapping[str, str]]) -> list[dict[str, Any]]:
    grouped: dict[tuple[str, ...], list[Mapping[str, Any]]] = defaultdict(list)
    for row in evidence:
        old = old_rows[row["sample_id"]]
        grouped[tuple(old.get(k) or "unknown" for k in keys)].append(row)
    output = []
    for group, rows in sorted(grouped.items()):
        item: dict[str, Any] = {**dict(zip(keys, group)), "selected_anchors": len(rows),
            "tiers": dict(Counter(r["tier"] for r in rows)),
            "source_reproduction": dict(Counter(r["source_reproduction"]["status"] for r in rows)),
            "unknown_reasons": dict(Counter(reason for r in rows for reason in r["geometry_tier_missing_evidence"])),
            "strict_cut_reasons_h30": dict(Counter(r["comparison"]["strict_diagnostic_v1"]["h30"]["cut_reason"] for r in rows))}
        for horizon in ("h15", "h30"):
            for distance in DISTANCES_M:
                key = f"{distance:g}m"
                raw = [r["comparison"]["raw_v1"][horizon][f"raw_reaches_{key}"] for r in rows]
                diagnostic = [r["comparison"]["strict_diagnostic_v1"][horizon]["reaches"][key] for r in rows]
                known = [r["strict_verified_reaches"][horizon][key] for r in rows if r["strict_verified_reaches"][horizon] is not None]
                item[f"{horizon}_{key}"] = {"raw_count": sum(v is True for v in raw),
                    "raw_known": sum(v is not None for v in raw), "strict_diagnostic_count": sum(diagnostic),
                    "strict_verified_count": sum(known) if known else None, "strict_verified_denominator": len(known),
                    "strict_unknown": len(rows) - len(known)}
        output.append(item)
    return output


def run_evidence_audit(*, root: Path, split: Path, previous: Path, output: Path, repo: Path,
                       config: EvidenceConfig, budget: ReadBudget, execute_raw: bool = False,
                       approved_plan: Path | None = None, expected: Mapping[str, str] = EXPECTED,
                       command: Sequence[str] = ()) -> dict[str, Any]:
    """Two-step dry-run -> identical read plan execution. All existing sources read-only."""
    config.validate(); budget.validate()
    protected = [root.resolve(), previous.resolve(), split.resolve()]
    output = output.resolve()
    if any(output == p or output.is_relative_to(p) or p.is_relative_to(output) for p in protected):
        raise ValueError("output overlaps protected source")
    if output.exists():
        raise FileExistsError("immutable evidence output already exists")
    # Identity checks happen before any raw payload read or output creation.
    paths = {"dataset_manifest": root / "manifest.yaml", "split": split,
             "previous_manifest": previous / "audit_manifest.json", "previous_ledger": previous / "anchor_audit_ledger.csv"}
    hashes = {name: sha256_file(path) for name, path in paths.items()}
    if any(hashes[k] != expected[k] for k in hashes):
        raise ValueError("source or previous audit identity mismatch; BLOCKED before writes")
    previous_meta = json.loads(paths["previous_manifest"].read_text(encoding="utf-8"))
    if previous_meta["repository"]["head"] != PREVIOUS_IMPLEMENTATION:
        raise ValueError("previous implementation identity mismatch")
    subprocess.run(["git", "-C", str(repo), "merge-base", "--is-ancestor", PREVIOUS_IMPLEMENTATION, "HEAD"], check=True)
    manifest, assignments, runs = validate_sources(root, split, expected["dataset_identity"])
    old_list = _metadata_rows(paths["previous_ledger"])
    old = {r["sample_id"]: r for r in old_list}
    if len(old) != len(old_list):
        raise ValueError("duplicate previous anchor ID")
    selected = choose_anchors(old_list, config)
    selected_ids = {r["sample_id"] for r in selected["selected"]}
    samples = {}
    canonical_ids = set()
    with (root / "samples.csv").open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            sid = row["sample_id"]
            canonical_ids.add(sid)
            if sid not in old or any(row[k] != old[sid][k] for k in ("run_id", "segment_id", "grid_stamp_ns")):
                raise ValueError("previous ledger/canonical identity differs")
            if assignments[row["run_id"]] != old[sid]["split"]:
                raise ValueError("previous ledger split differs")
            if sid in selected_ids:
                samples[sid] = row
    if canonical_ids != set(old):
        raise ValueError("previous ledger anchor cohort differs")
    plan = make_plan(selected, old, samples, config, budget)
    for run in plan["runs"]:
        raw = Path(run["raw_root"]).resolve()
        if output.is_relative_to(raw.parent) or raw.is_relative_to(output):
            raise ValueError("output overlaps raw/Reference source")
    if execute_raw:
        if approved_plan is None:
            raise ValueError("execute-raw requires prior dry-run plan")
        earlier = json.loads(approved_plan.read_text(encoding="utf-8"))
        if identity(earlier) != identity(plan):
            raise ValueError("dry-run plan/config/source changed; BLOCKED")
    inventory = []
    previous_inventory_path = previous / "source_inventory.json"
    inventory.append(source_entry("previous_inventory", previous_inventory_path))
    previous_inventory = json.loads(previous_inventory_path.read_text(encoding="utf-8"))
    selected_runs = {r["run_id"] for r in selected["selected"]}
    for entry in previous_inventory:
        if entry.get("file_sha256") and (any(entry["name"].endswith(":" + run) for run in selected_runs)
                                         or entry["name"] in {"phase_manifest", "phase_labels"}):
            current = source_entry(entry["name"], Path(entry["path"]))
            current["previous_sha256"] = entry["file_sha256"]
            current["matches_previous"] = current.get("file_sha256") == entry["file_sha256"]
            if not current["matches_previous"]:
                raise ValueError(f"previous source changed: {entry['name']}")
            inventory.append(current)
    output.mkdir(parents=True)
    write_json(output / "raw_read_plan.json", plan)
    write_json(output / "selection.json", selected)
    write_csv(output / "selected_anchors.csv", selected["selected"], ("sample_id", "run_id", "group_id", "group_kind", "selection_tags", "rank"))
    all_records: dict[str, list[dict[str, Any]]] = defaultdict(list)
    read_reports: list[dict[str, Any]] = []
    complete_by_run: dict[str, bool] = {}
    meter = ReadMeter(budget)
    for run in plan["runs"]:
        complete_by_run[run["run_id"]] = execute_raw and bool(run["files"])
        for file in run["files"]:
            if not execute_raw or file["status"] != "PRESENT":
                read_reports.append({**file, "run_id": run["run_id"], "status": "NOT_INSPECTED" if not execute_raw else "MISSING"})
                complete_by_run[run["run_id"]] = False
                continue
            records, report = read_mcap_windows(Path(file["path"]), run["windows_bag_ns"], meter=meter)
            report["run_id"] = run["run_id"]
            read_reports.append(report)
            all_records[run["run_id"]].extend(records)
            complete_by_run[run["run_id"]] &= report["status"] == "COMPLETE"
            print(json.dumps({"run": run["run_id"], "status": report["status"], "actual": report["actual"]}), flush=True)
    write_json(output / "raw_read_report.json", {"budget": asdict(budget), "actual": meter.snapshot(), "files": read_reports})
    write_json(output / "raw_window_evidence.json", all_records)
    futures = {f["path"]: f["sha256"] for f in manifest["files"]}
    del manifest
    evidence = []
    for chosen in selected["selected"]:
        sid = chosen["sample_id"]
        sample = samples[sid]
        path = (root / sample["trajectory_path"]).resolve()
        if not path.is_relative_to(root.resolve()) or sha256_file(path) != futures[sample["trajectory_path"]]:
            raise ValueError("selected trajectory identity mismatch")
        array = np.load(path, allow_pickle=False)
        item = evidence_for_anchor(array, sample, old[sid], all_records[sample["run_id"]], complete_by_run[sample["run_id"]], config)
        item["source_trajectory_sha256"] = futures[sample["trajectory_path"]]
        item["selected_group_id"] = chosen["group_id"]
        # Comparison to previous raw rules is anchor-level, not a new all-val audit.
        item["previous_raw_length_matches"] = all(
            (not old[sid][f"h{h}_raw_arc_m"] and item["comparison"]["raw_v1"][f"h{h}"]["raw_arc_m"] is None)
            or (bool(old[sid][f"h{h}_raw_arc_m"]) and math.isclose(float(old[sid][f"h{h}_raw_arc_m"]), item["comparison"]["raw_v1"][f"h{h}"]["raw_arc_m"], abs_tol=1e-9)) for h in (15, 30))
        evidence.append(item)
    by_id = {r["sample_id"]: r for r in evidence}
    states = [{"sample_id": r["sample_id"], "run_id": r["run_id"], "split": r["split"],
               "stopped_commanded_cohort": bool(r["estimated_episode_id"]),
               "previous_ledger_sha256": hashes["previous_ledger"],
               "additional_inspection": "INSPECTED" if r["sample_id"] in by_id else "NOT_INSPECTED",
               "tier": by_id[r["sample_id"]]["tier"] if r["sample_id"] in by_id else "NOT_INSPECTED",
               "driving_permission": "UNKNOWN" if r["sample_id"] in by_id else "NOT_INSPECTED"} for r in old_list]
    write_csv(output / "all_anchor_status.csv", states, list(states[0]))
    write_json(output / "anchor_evidence.json", evidence)
    summaries = {"run": summarize_evidence(evidence, ("run_id",), old),
                 "stop_episode": summarize_evidence(evidence, ("estimated_episode_id",), old),
                 "motion": summarize_evidence(evidence, ("v3_motion_assessment",), old),
                 "case": summarize_evidence(evidence, ("side", "near_far"), old)}
    write_json(output / "comparison_summary.json", summaries)
    tiers = dict(Counter(r["tier"] for r in evidence))
    gate = "CONDITIONAL_GO_PREFIX_ONLY" if tiers.get("GEOMETRY_VERIFIED", 0) else "BLOCKED_NO_SOURCE_VERIFIED_PREFIX"
    schema = {"format": "geometry_only_converter_proposal_v4_v1", "generated_dataset": False,
        "input_join": ["dataset_manifest_identity", "sample_id", "trajectory_sha256", "evidence_validator_version"],
        "proposed_fields": {"xy_m": "[N,2] original observation base_link; no rear axle offset",
            "arc_m": "[N] same fixed 0.1m grid, no extrapolation", "mask": "[N] observed valid prefix only",
            "source_horizon_sec": "1.5 or 3.0", "tier": "separate geometry from path supervision/safety",
            "cut_reason": "nonnegative-label censor reason", "permission": "UNKNOWN unless explicit preflight evidence",
            "clearance": "UNKNOWN/NOT_INSPECTED; never false-as-safe"},
        "adoption_policy": ["GEOMETRY_VERIFIED only, strict prefix scope", "not executable or training-approved path labels",
                            "reject failed source/time/frame checks", "preserve unresolved intent and clearance", "run split unchanged"],
        "geometry_only_gate": gate, "stop_teacher_gate": "BLOCKED_INTENT_PERMISSION_INCOMPLETE", "controller_oracle_gate": "BLOCKED_ENVIRONMENT_VEHICLE_POLICY"}
    write_json(output / "geometry_converter_schema_proposal.json", schema)
    changed = [name for name, path in paths.items() if sha256_file(path) != hashes[name]]
    changed += [entry["name"] for entry in inventory if entry.get("file_sha256") and sha256_file(Path(entry["path"])) != entry["file_sha256"]]
    if changed:
        raise ValueError(f"source changed during audit: {changed}")
    def git(*args: str) -> str:
        return subprocess.check_output(["git", "-C", str(repo), *args], text=True).strip()
    report = {"format": VERSION, "status": "DRY_RUN" if not execute_raw else "PARTIAL" if any(r["status"] != "COMPLETE" for r in read_reports) else "COMPLETE_LIMITED_SCOPE",
        "created_at_utc": datetime.now(timezone.utc).isoformat(), "code_commit": git("rev-parse", "HEAD"),
        "branch": git("branch", "--show-current") or "DETACHED", "origin": git("remote", "get-url", "origin"),
        "working_tree": git("status", "--porcelain"), "previous_implementation": PREVIOUS_IMPLEMENTATION,
        "code_hashes": {str(p.relative_to(repo)): sha256_file(p) for p in (Path(__file__).resolve(),
             repo / "src/aic_transfuser_lite/data/spatial_source_reader_v4.py", repo / "tools/audit_spatial_evidence_v4.py")},
        "source_hashes": hashes, "dataset_identity": expected["dataset_identity"], "source_unchanged": True,
        "plan_identity": identity(plan), "configuration": asdict(config), "budget": asdict(budget), "command": list(command),
        "environment": {"python": platform.python_version(), "platform": platform.platform(), "numpy": np.__version__},
        "all_anchor_count": len(states), "val_stopped_commanded_tracked": sum(r["split"] == "val" and bool(r["estimated_episode_id"]) for r in old_list),
        "selected_anchor_count": len(evidence), "selected_group_count": selected["group_count"],
        "selected_stopped_commanded": sum(bool(old[r["sample_id"]]["estimated_episode_id"]) for r in evidence),
        "tiers": tiers, "gates": schema, "test_geometry_inspected": False,
        "raw_actual": meter.snapshot(), "source_reader_statuses": dict(Counter(r["status"] for r in read_reports)),
        "uncovered_selection_tags": selected["uncovered_available_tags"],
        "all_selected_raw_matches_previous": all(r["previous_raw_length_matches"] for r in evidence),
        "all_common_prefixes_agree": all(r["comparison"]["common_prefix_agrees"] for r in evidence)}
    write_json(output / "source_inventory_delta.json", inventory)
    write_json(output / "execution_manifest.json", report)
    lines = ["# h30教師適格性・停止文脈の限定監査", "", f"状態: {report['status']}",
        f"全anchor追跡 {len(states)}、val stopped-commanded追跡 {report['val_stopped_commanded_tracked']}。",
        f"選択 {len(evidence)} anchors / {selected['group_count']} groups（推定停止episodeと補足診断windowを区別）。",
        f"tiers: {tiers}", "", "geometry-only: " + gate,
        "停止教師設計・controller oracle: BLOCKED（明示的意図/許可/環境・車体制約が未確定）。",
        "geometry verifiedは指定暫定規則でsource整合を確認したprefixのみ。安全/学習採用の承認ではない。",
        "offset-holdは横offset保持のannotationで、停止指示ではない。未記録Safety理由は安全確認ではない。",
        "比較A/Bは同じ保存済みfutureのh15/h30。新規長期pose教師・モデル改善・controller受理の評価ではない。",
        "未調査anchor/testはNOT_INSPECTED。距離不足/hold/記録終端を負のcontinuation教師にしない。",
        "詳細: anchor_evidence.json, raw_read_report.json, comparison_summary.json, geometry_converter_schema_proposal.json。"]
    (output / "report_ja.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return report
