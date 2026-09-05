"""Read-only spatial teacher audit. No model, image, LiDAR or optimizer execution.

Geometry is measured outcome, NEVER a safety/continuation/launch-permission label.
All noise/boundary thresholds below are provisional diagnostics, not V4 targets.
"""
from __future__ import annotations

from collections import Counter, defaultdict
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import csv
import hashlib
import json
import math
from pathlib import Path
import platform
import subprocess
import time
from typing import Any, Iterable, Mapping, Sequence
from urllib.parse import unquote, urlparse

import numpy as np
import yaml

from .dataset_view_v3 import (
    ControlTargetBoundsV3, MotionTargetFilterConfigV3,
    _ego_row, _selected_command, assess_commanded_motion_target_v3,
)

DISTANCES_M = (0.5, 1.0, 1.5, 2.0)
SAVED_COMMIT = "2989f9389415c121824c585754b8e10d7904a659"


@dataclass(frozen=True)
class SpatialAuditConfig:
    """SI-unit, provisional limits; caps apply only to detailed geometry reads."""

    splits: tuple[str, ...] = ("train", "val")
    detailed_test: bool = False
    max_anchors: int = 100000
    max_seconds: float = 600.0
    maximum_gap_sec: float = 0.2
    hold_sec: float = 0.5
    episode_gap_sec: float = 0.5
    noise_radius_m: float = 0.005
    stationary_speed_mps: float = 0.01
    teleport_speed_mps: float = 20.0
    maximum_distance_m: float = 2.0
    curvature_min_segment_m: float = 0.02
    curvature_min_support_m: float = 0.2

    def validate(self) -> None:
        if not self.splits or not set(self.splits) <= {"train", "val", "test"}:
            raise ValueError("splits must be train/val/test")
        if len(set(self.splits)) != len(self.splits):
            raise ValueError("duplicate splits")
        if "test" in self.splits and not self.detailed_test:
            raise ValueError("test geometry requires explicit detailed_test")
        if self.max_anchors <= 0:
            raise ValueError("max_anchors must be positive")
        for key, value in asdict(self).items():
            if isinstance(value, float) and (not math.isfinite(value) or value <= 0):
                raise ValueError(f"{key} must be finite and positive")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def identity(value: Any) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     ensure_ascii=False, allow_nan=False).encode()).hexdigest()


def read_yaml(path: Path) -> Any:
    # Safe C loader avoids minutes of Python object parsing for the 51 MB manifest.
    with path.open(encoding="utf-8") as stream:
        return yaml.load(stream, Loader=getattr(yaml, "CSafeLoader", yaml.SafeLoader))


def csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def source_entry(name: str, path: Path | None, *, inspect: bool = True) -> dict[str, Any]:
    entry: dict[str, Any] = {"name": name, "path": str(path) if path else None}
    if path is None or not inspect:
        return {**entry, "status": "NOT_INSPECTED"}
    try:
        if not path.exists():
            return {**entry, "status": "MISSING"}
        if path.is_file():
            return {**entry, "status": "PRESENT", "size_bytes": path.stat().st_size,
                    "file_sha256": sha256_file(path)}
        return {**entry, "status": "PRESENT", "kind": "directory"}
    except OSError as error:
        return {**entry, "status": "UNREADABLE", "reason": str(error)}


def validate_sources(root: Path, split_path: Path, expected_identity: str | None
                     ) -> tuple[dict[str, Any], dict[str, str], list[dict[str, str]]]:
    manifest = read_yaml(root / "manifest.yaml")
    if manifest.get("complete") is not True or (root / ".incomplete").exists():
        raise ValueError("canonical dataset is incomplete")
    payload = {k: v for k, v in manifest.items() if k != "manifest_sha256"}
    if identity(payload) != manifest.get("manifest_sha256"):
        raise ValueError("manifest internal identity mismatch")
    if expected_identity and manifest["manifest_sha256"] != expected_identity:
        raise ValueError("expected dataset identity mismatch")
    split = json.loads(split_path.read_text(encoding="utf-8"))
    if split.get("dataset_manifest_sha256") != manifest["manifest_sha256"]:
        raise ValueError("split dataset identity mismatch")
    assignments: dict[str, str] = {}
    for row in split["assignments"]:
        if row["run_id"] in assignments or row["split"] not in {"train", "val", "validation", "test"}:
            raise ValueError("duplicate run assignment or unsupported split")
        assignments[row["run_id"]] = "val" if row["split"] == "validation" else row["split"]
    runs = csv_rows(root / "runs.csv")
    source_splits: dict[str, set[str]] = defaultdict(set)
    for run in runs:
        if run["run_id"] not in assignments:
            raise ValueError("run missing from split")
        source_splits[run["source_hash"]].add(assignments[run["run_id"]])
    if any(len(values) > 1 for values in source_splits.values()):
        raise ValueError("source hash crosses split")
    # Verify metadata bytes against the inventory, not only self-consistent YAML.
    entries = {item["path"]: item for item in manifest["files"]}
    for name in ("samples.csv", "runs.csv"):
        if name not in entries or sha256_file(root / name) != entries[name]["sha256"]:
            raise ValueError(f"metadata file identity mismatch: {name}")
    return manifest, assignments, runs


def resample_no_extrapolation(xy_m: np.ndarray, spacing_m: float = 0.1
                              ) -> tuple[np.ndarray, dict[str, Any]]:
    """Finite contiguous polyline [N,2], starting at origin; never duplicate end.

    This diagnostic does not validate safety or reconnect invalid-mask gaps.
    Corner-cut loss is the difference between original and resampled chord lengths.
    """
    points = np.asarray(xy_m, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) < 1:
        raise ValueError("polyline must be [N,2], N>=1")
    if not np.isfinite(points).all() or not math.isfinite(spacing_m) or spacing_m <= 0:
        raise ValueError("finite polyline and positive spacing required")
    distances = np.linalg.norm(np.diff(points, axis=0), axis=1)
    points = points[np.r_[True, distances > 1e-12]]
    arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    targets = np.arange(0, arc[-1] + 1e-12, spacing_m)
    if arc[-1] - targets[-1] > 1e-9:
        targets = np.r_[targets, arc[-1]]
    targets = np.minimum(targets, arc[-1])
    result = np.column_stack([np.interp(targets, arc, points[:, axis]) for axis in (0, 1)])
    chord_length = float(np.linalg.norm(np.diff(result, axis=0), axis=1).sum())
    return result, {"original_arc_m": float(arc[-1]), "resampled_arc_m": chord_length,
                    "corner_cut_loss_m": max(0.0, float(arc[-1]) - chord_length),
                    "extrapolated": False}


def bounded_pose_prefix(records: Sequence[Mapping[str, Any]], *, config: SpatialAuditConfig,
                        max_elapsed_sec: float = 3.0) -> tuple[int, str]:
    """Validate optional world-pose records, including anchor at index zero.

    Required explicit fields: run_id, split, segment_id, reset_id, route_intent,
    stamp_sec, x_m, y_m, speed_mps, hold, reverse. Never infer missing intent/reset.
    Returns retained count and first censor boundary; does not emit teacher data.
    """
    config.validate()
    if not math.isfinite(max_elapsed_sec) or max_elapsed_sec <= 0:
        raise ValueError("positive elapsed limit required")
    if not records:
        return 0, "SOURCE_UNAVAILABLE"
    first = records[0]
    keys = ("run_id", "split", "segment_id", "reset_id", "route_intent")
    if any(first.get(key) in (None, "", "unknown") for key in keys):
        return 0, "boundary_or_route_intent_unknown"
    length, hold_since = 0.0, None
    for index, row in enumerate(records):
        for key in keys:
            if row.get(key) != first[key]:
                return index, f"{key}_boundary"
        if row.get("reverse") is None or row.get("hold") is None:
            return index, "motion_intent_unknown"
        if not all(math.isfinite(float(row.get(k, math.nan))) for k in
                   ("stamp_sec", "x_m", "y_m", "speed_mps")):
            return index, "nonfinite_pose"
        if row["reverse"] or float(row["speed_mps"]) < -config.stationary_speed_mps:
            return index, "reverse_boundary"
        stamp = float(row["stamp_sec"])
        if stamp - float(first["stamp_sec"]) > max_elapsed_sec + 1e-9:
            return index, "elapsed_limit"
        if row["hold"]:
            hold_since = stamp if hold_since is None else hold_since
            if stamp - hold_since >= config.hold_sec - 1e-9:
                return index, "long_hold"
        else:
            hold_since = None
        if index:
            prev = records[index - 1]
            dt = stamp - float(prev["stamp_sec"])
            if dt <= 0:
                return index, "timestamp_non_monotonic"
            if dt > config.maximum_gap_sec + 1e-9:
                return index, "timestamp_gap"
            ds = math.hypot(float(row["x_m"]) - float(prev["x_m"]),
                            float(row["y_m"]) - float(prev["y_m"]))
            if ds / dt > config.teleport_speed_mps:
                return index, "teleport"
            if length + ds > config.maximum_distance_m + 1e-9:
                return index, "distance_limit"
            length += ds
    return len(records), "source_end"


def future_geometry(future: np.ndarray, config: SpatialAuditConfig, *, horizon_sec: float
                    ) -> dict[str, Any]:
    """Audit canonical [H,8]: t,x,y,yaw,v_long,v_lat,yaw_rate,valid.

    Frame base_link@observation; m, s, rad, m/s, rad/s. Prefix metrics include
    the known ego origin at t=0. Disconnected valid points are counted but not joined.
    """
    config.validate()
    values = np.asarray(future, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 8 or not len(values):
        raise ValueError("future requires nonempty [H,8]")
    flags: list[str] = []
    mask_ok = np.isfinite(values[:, 7]) & np.isin(values[:, 7], [0, 1])
    if not mask_ok.all():
        flags.append("invalid_mask")
    selected = values[:int(round(horizon_sec / 0.1))]
    valid = (selected[:, 7] == 1) & np.isfinite(selected[:, :7]).all(axis=1)
    if np.any((selected[:, 7] == 1) & ~np.isfinite(selected[:, :7]).all(axis=1)):
        flags.append("nonfinite_valid_future")
    count = int(valid.sum())
    expected = int(round(horizon_sec / 0.1))
    status = "none" if not count else "full" if count == expected else "partial"
    points, times, speeds, yaws = [np.zeros(2)], [0.0], [], [0.0]
    boundary = "horizon_limit"
    for row, good in zip(selected, valid):
        if not good:
            boundary = "invalid_future_gap"
            break
        dt = row[0] - times[-1]
        if dt <= 0:
            boundary = "timestamp_non_monotonic"
            break
        if dt > config.maximum_gap_sec + 1e-7:
            boundary = "timestamp_gap"
            break
        if row[0] > horizon_sec + 1e-6:
            boundary = "horizon_limit"
            break
        if np.linalg.norm(row[1:3] - points[-1]) / dt > config.teleport_speed_mps:
            boundary = "teleport"
            break
        points.append(row[1:3]); times.append(float(row[0]))
        speeds.append(float(row[4])); yaws.append(float(row[3]))
    if len(selected) < expected and boundary == "horizon_limit":
        boundary = "stored_horizon_end"
    xy = np.asarray(points)
    segments = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    raw_length = float(segments.sum())
    # Accumulate displacement from LAST RELIABLE POINT, not per-frame deadband.
    reliable = [0]
    for i in range(1, len(xy)):
        if np.linalg.norm(xy[i] - xy[reliable[-1]]) >= config.noise_radius_m:
            reliable.append(i)
    max_disp = float(np.linalg.norm(xy, axis=1).max())
    stationary = bool(speeds) and max(abs(v) for v in speeds) <= config.stationary_speed_mps
    stationary = stationary and max_disp < 2 * config.noise_radius_m
    if stationary:
        reliable = [0]
        flags.append("stationary_jitter_provisional")
    filtered = xy[reliable]
    support = float(np.linalg.norm(np.diff(filtered, axis=0), axis=1).sum())
    curvature, curvature_reason, heading_change = None, "insufficient_spatial_support", None
    if len(filtered) >= 3 and support >= config.curvature_min_support_m:
        d = np.diff(filtered, axis=0)
        ds = np.linalg.norm(d, axis=1)
        if np.all(ds >= config.curvature_min_segment_m):
            headings = np.unwrap(np.arctan2(d[:, 1], d[:, 0]))
            heading_change = float(np.abs(np.diff(headings)).sum())
            curvature = float(np.max(np.abs(np.diff(headings)) / ((ds[1:] + ds[:-1]) / 2)))
            curvature_reason = "computable_provisional"
        else:
            curvature_reason = "short_segments_noise_sensitive"
    hold_duration, maximum_hold, previous = 0.0, 0.0, 0.0
    for stamp, speed in zip(times[1:], speeds):
        hold_duration = hold_duration + stamp - previous if abs(speed) <= config.stationary_speed_mps else 0.0
        maximum_hold = max(maximum_hold, hold_duration)
        previous = stamp
    if maximum_hold >= config.hold_sec:
        flags.append("long_hold_observed_not_intent")
    if any(v < -config.stationary_speed_mps for v in speeds):
        flags.append("reverse_motion")
    if boundary not in ("horizon_limit", "stored_horizon_end"):
        flags.append(boundary)
    last = selected[np.flatnonzero(valid)[-1]] if count else None
    result: dict[str, Any] = {
        "valid_count": count, "future_status": status, "prefix_count": len(xy) - 1,
        "raw_arc_m": raw_length if len(xy) > 1 else None,
        "noise_filtered_arc_m_provisional": support if len(xy) > 1 else None,
        "support_quality": "provisional_uncalibrated" if len(xy) > 1 else "no_contiguous_support",
        "endpoint_x_m": float(last[1]) if last is not None else None,
        "endpoint_y_m": float(last[2]) if last is not None else None,
        "max_displacement_m": float(np.linalg.norm(selected[valid, 1:3], axis=1).max()) if count else None,
        "prefix_elapsed_sec": times[-1], "duplicate_segments": int(np.sum(segments <= 1e-12)),
        "segment_min_m": float(segments.min()) if len(segments) else None,
        "segment_max_m": float(segments.max()) if len(segments) else None,
        "heading_change_rad": heading_change, "curvature_max_abs_per_m": curvature,
        "curvature_status": curvature_reason, "maximum_hold_sec": maximum_hold,
        "censor_reason": boundary, "flags": flags,
        "path_loss_eligibility": "unknown_missing_route_clearance_and_noise_calibration",
        "continuation_impossible_evidence": "unknown", "safe_endpoint_label": None,
    }
    for distance in DISTANCES_M:
        label = f"{distance:g}m"
        result[f"raw_reaches_{label}"] = raw_length >= distance if len(xy) > 1 else None
        result[f"provisional_reaches_{label}"] = support >= distance if len(xy) > 1 else None
    return result


def v3_row_status(row: dict[str, str], future: np.ndarray | None, model_config: dict[str, Any]
                  ) -> dict[str, Any]:
    """Call saved V3 helpers exactly, including float32 command clipping/order."""
    bounds = ControlTargetBoundsV3(**model_config["model"]["control_bounds"],
                                  control_dt_sec=float(model_config["model"]["control_dt_sec"]))
    command = _selected_command(row, bounds=bounds)
    data = model_config["data"]
    _, mask = _ego_row(row, tuple(data["ego_features"]), abs_limits=data.get("ego_abs_limits"))
    flags = []
    if command is None:
        flags.append("missing_selected_command")
    if int(row["future_valid_count"]) <= 0:
        flags.append("zero_valid_future")
    if not bool(mask.all()):
        flags.append("invalid_current_ego")
    base_primary = flags[0] if flags else "none"
    assessment = "NOT_INSPECTED"
    cfg = MotionTargetFilterConfigV3(**model_config["targets"]["motion_target_filter"])
    if not flags and future is not None:
        assert command is not None
        try:
            assessment = assess_commanded_motion_target_v3(
                future, current_speed_mps=float(row["velocity_longitudinal_mps"]),
                commanded_speed_mps=float(command[0][1]), config=cfg).value
        except ValueError:
            # Preserve this raw anchor instead of aborting/dropping a corrupt label.
            assessment = "UNREADABLE"
        if cfg.enabled and assessment == "contradictory_stationary":
            flags.append("contradictory_stationary")
    return {"ego_valid": bool(mask.all()), "command_source": command[1] if command else "unknown",
            "command_speed_mps": float(command[0][1]) if command else None,
            "base_exclusion_primary": base_primary, "exclusion_primary": flags[0] if flags else "none",
            "v3_exclusion_flags": flags, "v3_motion_assessment": assessment,
            "v3_quality_member": (not flags) if future is not None and assessment != "UNREADABLE" else None}


def repository_provenance(repo: Path) -> dict[str, Any]:
    def git(*args: str) -> str:
        return subprocess.check_output(["git", "-C", str(repo), *args], text=True).strip()
    status = git("status", "--porcelain=v1", "--untracked-files=all")
    saved = git("rev-parse", f"{SAVED_COMMIT}^{{commit}}")
    if subprocess.run(["git", "-C", str(repo), "merge-base", "--is-ancestor", saved, "HEAD"]).returncode:
        raise ValueError("saved V3 is not an ancestor of audited HEAD")
    paths = [Path(__file__).resolve(), repo / "tools/audit_spatial_coverage_v4.py"]
    return {"root": git("rev-parse", "--show-toplevel"), "origin": git("remote", "get-url", "origin"),
            "branch": git("branch", "--show-current") or "DETACHED", "head": git("rev-parse", "HEAD"),
            "working_tree": status, "dirty": bool(status), "saved_v3_commit": saved,
            "dirty_patch_sha256": hashlib.sha256(git("diff", "HEAD", "--binary").encode()).hexdigest(),
            "code_hashes": {str(p.relative_to(repo)): sha256_file(p) for p in paths}}


def write_json(path: Path, value: Any) -> None:
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, ensure_ascii=False, allow_nan=False)
        stream.write("\n")


def write_csv(path: Path, rows: Iterable[Mapping[str, Any]], fields: Sequence[str]) -> None:
    with path.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: json.dumps(v, ensure_ascii=False, allow_nan=False) if isinstance(v, (dict, list)) else v
                             for k, v in row.items()})


def aggregate(rows: Sequence[Mapping[str, Any]], keys: Sequence[str]) -> list[dict[str, Any]]:
    groups: dict[tuple[str, ...], list[Mapping[str, Any]]] = defaultdict(list)
    for row in rows:
        groups[tuple(str(row.get(key) or "unknown") for key in keys)].append(row)
    output = []
    for values, group in sorted(groups.items()):
        counts: dict[str, Any] = {"raw_anchors": len(group),
            "processed_geometry": sum(r["geometry_status"] == "PRESENT" for r in group),
            "not_inspected_geometry": sum(r["geometry_status"] == "NOT_INSPECTED" for r in group),
            "run_count": len({r["run_id"] for r in group}),
            "source_count": len({r["source_hash"] for r in group}),
            "estimated_episodes": len({r["estimated_episode_id"] for r in group if r["estimated_episode_id"]}),
            "confirmed_episodes": None, "confirmed_sessions": None,
            "primary_exclusions": dict(Counter(str(r["exclusion_primary"]) for r in group)),
            "base_exclusions": dict(Counter(str(r["base_exclusion_primary"]) for r in group)),
            "multiple_exclusion_flags": dict(Counter(f for r in group for f in r["v3_exclusion_flags"])),
            "quality_flag_counts": dict(Counter(f for r in group for f in r["quality_flags"])),
            "motion_assessments": dict(Counter(str(r["v3_motion_assessment"]) for r in group)),
            "v3_quality_assessed": sum(r["v3_quality_member"] is not None for r in group),
            "v3_quality_members": (sum(r["v3_quality_member"] is True for r in group)
                if any(r["v3_quality_member"] is not None for r in group) else None)}
        for horizon in ("h15", "h20", "h30"):
            for metric in ("raw_arc_m", "noise_filtered_arc_m_provisional"):
                present = [r.get(f"{horizon}_{metric}") for r in group if r.get(f"{horizon}_{metric}") is not None]
                counts[f"{horizon}_{metric}"] = {"denominator": len(present), "mean": float(np.mean(present)) if present else None,
                    "p50": float(np.median(present)) if present else None,
                    "p95": float(np.percentile(present, 95)) if present else None}
            for distance in DISTANCES_M:
                for kind in ("raw", "provisional"):
                    name = f"{horizon}_{kind}_reaches_{distance:g}m"
                    known = [r[name] for r in group if r.get(name) is not None]
                    counts[name] = {"count": sum(known) if known else None, "denominator": len(known),
                                    "fraction": sum(known) / len(known) if known else None}
        output.append({**dict(zip(keys, values)), **counts})
    return output


def load_annotations(view: Path | None, filename: str, manifest: dict[str, Any],
                     parent_root: Path | None = None) -> dict[str, dict[str, str]]:
    if view is None:
        return {}
    meta = json.loads((view / "manifest.json").read_text(encoding="utf-8"))
    if sha256_file(view / filename) != meta.get("labels_sha256"):
        raise ValueError("annotation labels identity mismatch")
    if meta.get("dataset_manifest_sha256") != manifest["manifest_sha256"]:
        if parent_root is None:
            raise ValueError("annotation belongs to another dataset; explicit parent required")
        parent = read_yaml(parent_root / "manifest.yaml")
        if identity({k: v for k, v in parent.items() if k != "manifest_sha256"}) != parent["manifest_sha256"]:
            raise ValueError("annotation parent internal identity mismatch")
        if parent["manifest_sha256"] != meta.get("dataset_manifest_sha256"):
            raise ValueError("annotation parent identity mismatch")
        current_runs = {r["run_id"]: r["source_hash"] for r in manifest["runs"]}
        if any(current_runs.get(r["run_id"]) != r["source_hash"] for r in parent["runs"]):
            raise ValueError("annotation parent run/source is not a subset")
        assets = {r["path"]: r["sha256"] for r in manifest["files"]}
        if any(assets.get(r["path"]) != r["sha256"] for r in parent["files"]
               if r["path"].startswith("trajectories/")):
            raise ValueError("annotation parent trajectory identities differ")
    rows = csv_rows(view / filename)
    result = {r["sample_id"]: r for r in rows}
    if len(result) != len(rows):
        raise ValueError("duplicate annotation sample ID")
    return result


def inventory_raw(runs: Sequence[Mapping[str, str]]) -> tuple[list[dict[str, Any]], dict[str, dict[str, Any]]]:
    """Only named small metadata/Reference sidecars; never read compressed bag payloads."""
    inventory, facts = [], {}
    for run in runs:
        uri = urlparse(run["source_uri"])
        if uri.scheme != "file":
            inventory.append({"name": f"raw:{run['run_id']}", "status": "UNSUPPORTED", "uri": run["source_uri"]})
            continue
        raw = Path(unquote(uri.path))
        meta_entry = source_entry(f"raw_metadata:{run['run_id']}", raw / "metadata.yaml")
        info: dict[str, Any] = {"collection_case": "unknown", "teacher_controller": "unknown"}
        if meta_entry["status"] == "PRESENT":
            meta = read_yaml(raw / "metadata.yaml")["rosbag2_bagfile_information"]
            topics = [{"name": t["topic_metadata"]["name"], "type": t["topic_metadata"]["type"],
                       "message_count": t["message_count"]} for t in meta["topics_with_message_count"]]
            meta_entry.update(topics=topics, payload_status="NOT_INSPECTED",
                              reason="bounded metadata-only: no decompression or bag extraction")
            info["pose_topic_present"] = any(t["type"] == "nav_msgs/msg/Odometry" for t in topics)
            info["safety_topic_present"] = any("safety_reason" in t["name"] for t in topics)
        inventory.append(meta_entry)
        preflight = raw.parent / "preflight.json"
        entry = source_entry(f"preflight:{run['run_id']}", preflight)
        inventory.append(entry)
        if entry["status"] == "PRESENT":
            pre = json.loads(preflight.read_text(encoding="utf-8"))
            if pre.get("run_id") != run["run_id"]:
                raise ValueError("preflight run identity mismatch")
            info.update(collection_case=pre.get("collection_case_id", "unknown"),
                        teacher_controller=pre.get("teacher_controller_id", "unknown"))
        for name in ("recovery_reference_v3.csv", "recovery_reference_v3.intervals.csv", "base_mpc_collection_reference.csv"):
            path = raw.parent / name
            entry = source_entry(f"{name}:{run['run_id']}", path)
            if entry["status"] == "PRESENT":
                rows = csv_rows(path)
                entry.update(row_count=len(rows), columns=list(rows[0]) if rows else [],
                             inference_available="unknown", anchor_alignment="NOT_INSPECTED",
                             frame_id="unknown", vehicle_geometry_verified=False,
                             clearance="unknown", safe_teacher=False)
                if rows and {"x_m", "y_m", "s_m"} <= set(rows[0]):
                    xy = np.array([[float(r["x_m"]), float(r["y_m"])] for r in rows])
                    if not np.isfinite(xy).all():
                        entry["geometry_status"] = "UNREADABLE"
                    else:
                        ds = np.linalg.norm(np.diff(xy, axis=0), axis=1)
                        entry.update(geometry_status="PRESENT", arc_m=float(ds.sum()),
                                     maximum_segment_m=float(ds.max()) if len(ds) else None,
                                     timestamp_available=False, continuity_approved=False)
            inventory.append(entry)
        facts[run["run_id"]] = info
    return inventory, facts


def run_audit(*, dataset_root: Path, split_manifest: Path, output: Path, repo: Path,
              model_config_path: Path, config: SpatialAuditConfig,
              expected_identity: str | None = None, behavior_view: Path | None = None,
              phase_view: Path | None = None, phase_parent: Path | None = None,
              command: Sequence[str] = ()) -> dict[str, Any]:
    config.validate()
    root, output = dataset_root.resolve(), output.resolve()
    protected = [root, split_manifest.resolve(), model_config_path.resolve()]
    protected += [p.resolve() for p in (behavior_view, phase_view, phase_parent) if p]
    if any(output == p or output.is_relative_to(p) or p.is_relative_to(output) for p in protected):
        raise ValueError("output overlaps protected source")
    if output.exists():
        raise FileExistsError(f"immutable output already exists: {output}")
    output.mkdir(parents=True)
    started = time.monotonic()
    inventory = [source_entry(name, path) for name, path in (
        ("canonical", root), ("manifest", root / "manifest.yaml"), ("samples", root / "samples.csv"),
        ("runs", root / "runs.csv"), ("split", split_manifest), ("model_config", model_config_path),
        ("behavior_manifest", behavior_view / "manifest.json" if behavior_view else None),
        ("behavior_labels", behavior_view / "behavior_labels.csv" if behavior_view else None),
        ("phase_manifest", phase_view / "manifest.json" if phase_view else None),
        ("phase_labels", phase_view / "phase_labels.csv" if phase_view else None),
        ("phase_parent_manifest", phase_parent / "manifest.yaml" if phase_parent else None))]
    report: dict[str, Any] = {"format": "spatial_coverage_audit_v4_v1", "status": "BLOCKED",
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "configuration": asdict(config), "threshold_status": "provisional_not_calibrated",
        "command": list(command), "environment": {"python": platform.python_version(),
        "platform": platform.platform(), "numpy": np.__version__, "yaml": yaml.__version__},
        "source_root": str(root), "output": str(output), "blocking_reasons": []}
    try:
        report["repository"] = repository_provenance(repo.resolve())
        manifest, assignments, runs = validate_sources(root, split_manifest, expected_identity)
        report["dataset_manifest_identity"] = manifest["manifest_sha256"]
        report["manifest_file_sha256"] = sha256_file(root / "manifest.yaml")
        report["split_file_sha256"] = sha256_file(split_manifest)
        report["dataset_schema"] = manifest.get("schema_version")
        if manifest.get("schema_version") != "aic_canonical_dataset_v3":
            raise ValueError("unsupported canonical schema")
        model = read_yaml(model_config_path)
        behavior = load_annotations(behavior_view, "behavior_labels.csv", manifest)
        phases = load_annotations(phase_view, "phase_labels.csv", manifest, phase_parent)
        raw_inventory, run_facts = inventory_raw(runs)
        inventory.extend(raw_inventory)
        runtime_path = repo / "ros2_ws/src/aic_e2e_runtime/config/runtime.v3.trajectory_authoritative.param.yaml"
        inventory.append(source_entry("controller_and_vehicle_parameter_profile", runtime_path))
        inventory.extend([
            {"name": "canonical_future_and_mask", "status": "PRESENT", "payload_scope": list(config.splits),
             "contract": "[H,8] observation base_link; mask column7; teacher-only"},
            {"name": "reset_segmentation", "status": "PRESENT", "evidence": "samples.segment_id clock epochs",
             "physical_reset_teleport_annotation": "NOT_INSPECTED"},
            {"name": "absolute_pose_odometry", "status": "PRESENT" if any(f.get("pose_topic_present") for f in run_facts.values()) else "NOT_INSPECTED",
             "evidence": "raw metadata topic counts only", "payload_status": "NOT_INSPECTED"},
            {"name": "stop_reason", "status": "PRESENT" if any(f.get("safety_topic_present") for f in run_facts.values()) else "NOT_INSPECTED",
             "evidence": "raw metadata topic counts only", "anchor_alignment": "NOT_INSPECTED"},
            {"name": "route_intent_inference_input", "status": "UNSUPPORTED", "evidence": "no route intent field in selected V3 ModelBatch"},
            {"name": "launch_permission_and_operation_mode", "status": "NOT_INSPECTED",
             "reason": "nominal command or recorded topic is not anchor launch permission"},
            {"name": "vehicle_footprint_rear_axle_and_clearance", "status": "NOT_INSPECTED",
             "reason": "wheelbase parameter alone does not establish body footprint or pose reference"},
        ])
        if phase_view:
            phase_meta = json.loads((phase_view / "manifest.json").read_text(encoding="utf-8"))
            inventory_by_name = {entry["name"]: entry for entry in raw_inventory}
            for source in phase_meta.get("sources", []):
                for field, prefix in (("bag_metadata_sha256", "raw_metadata"),
                    ("generated_reference_sha256", "recovery_reference_v3.csv"),
                    ("intervals_sha256", "recovery_reference_v3.intervals.csv"),
                    ("base_reference_sha256", "base_mpc_collection_reference.csv")):
                    entry = inventory_by_name.get(f"{prefix}:{source['run_id']}")
                    if entry and entry["status"] == "PRESENT":
                        entry["phase_source_identity_match"] = entry["file_sha256"] == source[field]
                        if not entry["phase_source_identity_match"]:
                            raise ValueError(f"phase source identity mismatch: {entry['name']}")
        rows = csv_rows(root / "samples.csv")
        if len({r["sample_id"] for r in rows}) != len(rows):
            raise ValueError("duplicate canonical sample ID")
        rows.sort(key=lambda r: (r["run_id"], r["segment_id"], int(r["grid_stamp_ns"]), r["sample_id"]))
        sample_by_id = {r["sample_id"]: r for r in rows}
        for annotations in (behavior, phases):
            for sid, annotation in annotations.items():
                canonical = sample_by_id.get(sid)
                if canonical is None or any(annotation[k] != canonical[k] for k in ("run_id", "grid_stamp_ns")):
                    raise ValueError("annotation sample/run/timestamp identity mismatch")
        by_run = {r["run_id"]: r for r in runs}
        file_hashes = {r["path"]: r["sha256"] for r in manifest["files"]}
        del manifest
        ledger: list[dict[str, Any]] = []
        episode_last: dict[tuple[str, str], tuple[int, int]] = {}
        detailed_count, cap_reason = 0, None
        inspected_trajectories = hashlib.sha256()
        for row in rows:
            split = assignments[row["run_id"]]
            phase = phases.get(row["sample_id"], {})
            info = run_facts.get(row["run_id"], {})
            case = info.get("collection_case", "unknown")
            # Explicit preflight case labels, never fill measured geometry from names.
            kind = "recovery" if case.startswith("offset_") else "normal" if row["scenario_id"] == "d1_sim" else "unknown"
            side = "left" if case in ("offset_left_far", "offset_left_near") else "right" if case in ("offset_right_far", "offset_right_near") else "unknown"
            band = "far" if case in ("offset_left_far", "offset_right_far") else "near" if case in ("offset_left_near", "offset_right_near") else "unknown"
            record: dict[str, Any] = {"sample_id": row["sample_id"], "run_id": row["run_id"], "split": split,
                "source_hash": by_run[row["run_id"]]["source_hash"], "source_uri": by_run[row["run_id"]]["source_uri"],
                "session_id": "unknown", "confirmed_episode_id": None, "estimated_episode_id": None,
                "episode_method": "stopped_commanded_gap_0.5s_inferred_not_independent",
                "segment_id": row["segment_id"], "reset_id": row["segment_id"], "reset_evidence": "converter_clock_epoch_only",
                "grid_stamp_ns": int(row["grid_stamp_ns"]), "normal_recovery": kind,
                "collection_case": case, "side": side, "near_far": band, "case_label_provenance": "preflight_metadata",
                "geometry": phase.get("geometry", "unknown"), "phase": phase.get("phase", "unknown"),
                "phase_side": phase.get("side", "unknown"), "route_intent": "unknown",
                "behavior_label": (behavior[row["sample_id"]].get("behavior_label", "unknown")
                    if behavior.get(row["sample_id"], {}).get("behavior_valid", "false").lower() == "true" else "unknown"),
                "stop_intent": "unknown", "launch_permission": "unknown", "safety_reason": "unknown",
                "teacher_source": "measured_canonical_future", "pose_reference": "base_link_not_verified_rear_axle",
                "stored_steps_metadata": int(row["future_step_count"]), "stored_valid_metadata": int(row["future_valid_count"]),
                "geometry_status": "NOT_INSPECTED", "geometry_error": None,
                "long_pose_status": "NOT_INSPECTED" if info.get("pose_topic_present") else "SOURCE_UNAVAILABLE",
                "long_teacher_eligibility": "unknown_route_reset_hold_permission",
                "quality_flags": []}
            current_speed = float(row["velocity_longitudinal_mps"])
            record["motion_class"] = "unknown" if not math.isfinite(current_speed) else "stopped" if abs(current_speed) <= .05 else "slow" if abs(current_speed) <= .2 else "moving"
            future = None
            if split in config.splits:
                if detailed_count >= config.max_anchors:
                    cap_reason = "max_anchors"
                elif time.monotonic() - started >= config.max_seconds:
                    cap_reason = "max_seconds"
                else:
                    detailed_count += 1
                    try:
                        path = (root / row["trajectory_path"]).resolve()
                        if not path.is_relative_to(root):
                            raise ValueError("trajectory path escapes canonical root")
                        actual_sha = sha256_file(path)
                        if actual_sha != file_hashes.get(row["trajectory_path"]):
                            raise ValueError("trajectory hash mismatch")
                        inspected_trajectories.update(f"{row['sample_id']}:{actual_sha}\n".encode())
                        future = np.load(path, allow_pickle=False)
                        for label, horizon in (("h15", 1.5), ("h20", 2.0), ("h30", 3.0)):
                            geo = future_geometry(future, config, horizon_sec=horizon)
                            record.update({f"{label}_{key}": value for key, value in geo.items()})
                            record["quality_flags"].extend(f"{label}:{f}" for f in geo["flags"])
                        record["geometry_status"] = "PRESENT"
                    except (OSError, ValueError) as error:
                        record["geometry_status"] = "MISSING" if isinstance(error, FileNotFoundError) else "UNREADABLE"
                        record["geometry_error"] = str(error)
                        future = None
            record.update(v3_row_status(row, future, model))
            if record["base_exclusion_primary"] == "none" and math.isfinite(current_speed) and abs(current_speed) <= .05 and record["command_speed_mps"] >= .5:
                key = (row["run_id"], row["segment_id"])
                stamp = int(row["grid_stamp_ns"])
                last_stamp, ep = episode_last.get(key, (-10**30, 0))
                if stamp - last_stamp > config.episode_gap_sec * 1e9:
                    ep += 1
                episode_last[key] = (stamp, ep)
                record["estimated_episode_id"] = f"{key[0]}:{key[1]}:inferred{ep:04d}"
            ledger.append(record)
            if detailed_count and detailed_count % 10000 == 0 and record["geometry_status"] == "PRESENT":
                print(f"audited_geometry={detailed_count} ledger_rows={len(ledger)}", flush=True)
        fields = sorted({key for row in ledger for key in row})
        write_csv(output / "anchor_audit_ledger.csv", ledger, fields)
        summaries = aggregate(ledger, ("split",))
        write_json(output / "coverage_summary.json", summaries)
        write_json(output / "run_summary.json", aggregate(ledger, ("split", "run_id")))
        write_json(output / "episode_summary.json", aggregate([r for r in ledger if r["estimated_episode_id"]], ("split", "estimated_episode_id")))
        slices = {}
        for key in ("session_id", "normal_recovery", "motion_class", "side", "near_far", "geometry", "phase",
                    "teacher_source", "h15_censor_reason", "h15_curvature_status", "quality_flags"):
            slices[key] = aggregate(ledger, ("split", key))
        write_json(output / "slice_summary.json", slices)
        # Explicit empty measured-case slices; augmentation is never observed data.
        matrix = [{"split": split, "side": side, "near_far": band,
                   "anchors": sum(r["split"] == split and r["side"] == side and r["near_far"] == band for r in ledger)}
                  for split in ("train", "val", "test") for side in ("left", "right") for band in ("near", "far")]
        write_json(output / "recovery_case_matrix.json", matrix)
        errors = sum(r["geometry_status"] in ("MISSING", "UNREADABLE") for r in ledger)
        report.update(status="PARTIAL" if cap_reason or errors else "COMPLETE",
                      coverage_scope="train_val_geometry_test_metadata" if not config.detailed_test else "explicit_test_geometry_separate",
                      raw_anchor_count=len(ledger), attempted_geometry_count=detailed_count,
                      processed_geometry_count=sum(r["geometry_status"] == "PRESENT" for r in ledger),
                      cap_reason=cap_reason, geometry_errors=errors, summaries=summaries,
                      inspected_trajectory_identity=inspected_trajectories.hexdigest(),
                      oracle_ab_executed=False, model_inference_executed=False, training_executed=False,
                      next_gate="BLOCKED: confirm pose reference, aligned intent/reset/hold and clearance; calibrate noise")
        # Re-hash the metadata that was actually used. Sensor/bag payloads were not read.
        changed = [e["name"] for e in inventory if e.get("file_sha256") and sha256_file(Path(e["path"])) != e["file_sha256"]]
        report["source_metadata_unchanged"] = not changed
        if changed:
            raise ValueError(f"source metadata changed during audit: {changed}")
    except (OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError) as error:
        report["status"] = "BLOCKED"
        report["blocking_reasons"].append(f"{type(error).__name__}: {error}")
    report["elapsed_seconds"] = time.monotonic() - started
    write_json(output / "source_inventory.json", inventory)
    write_json(output / "audit_manifest.json", report)
    lines = ["# Spatial Path V4 coverage監査", "", f"状態: {report['status']}", "",
             "観測futureの幾何監査。安全経路・発進許可・continuation不可の教師を生成していない。",
             "ノイズ閾値は暫定。h15=先頭15点、h20/h30=保存済みfutureの追加診断であり、長期poseのteacher oracle A/Bではない。",
             "testは既定でmetadataのみ。未検査は距離0ではなくnull。confirmed session/episodeはunknown。", "",
             "| split | raw | 幾何処理済み | V3品質集合 |", "|---|---:|---:|---:|"]
    for row in report.get("summaries", []):
        lines.append(f"| {row['split']} | {row['raw_anchors']} | {row['processed_geometry']} | {row['v3_quality_members'] if row['processed_geometry'] else '未検査'} |")
    lines += ["", "次gate: " + report.get("next_gate", "BLOCKED"), "",
              "詳細な定義・runtime配線の読取結果はリポジトリ docs/spatial_path_v4_coverage_audit.md。",
              "blocking reasons: " + json.dumps(report["blocking_reasons"], ensure_ascii=False)]
    (output / "report_ja.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return report
