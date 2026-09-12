"""File-backed, whole-run time teachers with explicit receipt-clock freeze.

Raw images remain in verified SQLite bags. Labels are [N,30,2] metres in
base_link@observation at 0.1 s intervals. Invalid anchors remain in the audit.
This materializer does not fit preprocessing, optimize weights, or score test.
"""
from __future__ import annotations

from bisect import bisect_left, bisect_right
from collections import Counter
from dataclasses import asdict
from decimal import Decimal
import hashlib
import json
from pathlib import Path
from typing import Any, Sequence

import numpy as np

from .time_dataset_v1 import (TimeDatasetConfig, _eligible, _endpoints, _pose_anchor,
                              assemble_time_sample, anchor_identity)
from .time_history_v1 import TimeEvent, select_time_history
from .time_split_v1 import validate_time_split, verify_time_split_sources
from .time_sqlite_reader_v1 import load_event, read_time_sqlite_run
from .time_teacher_v1 import build_time_teacher


class EventWindows:
    """Binary-search a single epoch; keep memory proportional to one raw run."""
    def __init__(self, events: Sequence[TimeEvent]) -> None:
        self.rows: dict[str, list[TimeEvent]] = {}
        for e in events:
            self.rows.setdefault(e.epoch, []).append(e)
        for rows in self.rows.values():
            rows.sort(key=lambda e: (e.capture_ns, e.available_ns, e.sequence))
        self.stamps = {key: [e.capture_ns for e in rows] for key, rows in self.rows.items()}

    def at(self, anchor: TimeEvent) -> tuple[TimeEvent, ...]:
        rows, stamps = self.rows[anchor.epoch], self.stamps[anchor.epoch]
        left = bisect_left(stamps, anchor.capture_ns - 1_100_000_000)
        right = bisect_right(stamps, anchor.capture_ns + 3_050_000_000)
        return tuple(rows[left:right])


def audit_anchor(events: Sequence[TimeEvent], anchor: TimeEvent, *,
                 config: TimeDatasetConfig, bounds: tuple[int, int], freeze_ns: int,
                 intervention_ns: int | None) -> tuple[Any, dict[str, Any]]:
    """Audit reference selection without substituting pixels for model inputs.

Input eligibility uses the same causal selectors as assemble_time_inputs.
Full tensor construction is checked separately by replaying selected anchors.
"""
    eligible = _eligible(events, anchor, config, freeze_ns, bounds)
    history: dict[str, list[list[int]]] = {}
    selected: dict[str, tuple[TimeEvent | None, ...]] = {}
    for role, length, previous in (("camera", config.camera_history_length, False),
            ("lidar", config.lidar_history_length, False),
            ("final_command", config.command_history_length, True),
            ("actual_steering", config.ego_history_length, False)):
        selected[role] = select_time_history(eligible, role=role, run=anchor.run, epoch=anchor.epoch,
            capture_clock=config.capture_clock, available_clock=config.available_clock,
            freeze_ns=freeze_ns, observation_ns=anchor.capture_ns, length=length,
            tolerance_ns=config.tolerance_ns, previous_only=previous)
        history[role] = [[] if e is None else [e.sequence] for e in selected[role]]
    history["velocity"] = []
    current_speed_valid = False
    for i in range(config.ego_history_length):
        stamp = anchor.capture_ns - (config.ego_history_length - 1 - i) * 100_000_000
        ep = _endpoints(eligible, "velocity", stamp, config.tolerance_ns)
        history["velocity"].append([e.sequence for e in ep])
        if i == config.ego_history_length - 1:
            current_speed_valid = bool(ep) and all(np.isfinite(e.payload.longitudinal_mps) for e in ep)
    current_camera, current_scan = selected["camera"][-1], selected["lidar"][-1]
    invalid = None
    if (current_camera is None or current_camera.capture_ns != anchor.capture_ns or current_scan is None):
        invalid = "CURRENT_SENSOR_MISSING"
    elif not current_speed_valid:
        invalid = "CURRENT_LONGITUDINAL_SPEED_MISSING"
    teacher = None
    pose_refs: list[int] = []
    try:
        pose, ref = _pose_anchor(eligible, anchor, config)
        pose_refs = [r["sequence"] for r in ref["sources"]]
        teacher = build_time_teacher(tuple(e for e in events if e.role in {"pose", "velocity"}), pose,
            epoch_start_ns=bounds[0], epoch_end_ns=bounds[1], intervention_ns=intervention_ns,
            tolerance_ms=config.teacher_tolerance_ms)
        reasons = sorted(set(teacher.reasons))
    except ValueError as exc:
        reasons = [str(exc)]
    intervention = intervention_ns is not None and anchor.capture_ns + 3_000_000_000 >= intervention_ns
    row = {"anchor_id": anchor_identity(anchor), "camera_row_id": anchor.sequence,
           "epoch": anchor.epoch, "observation_ns": anchor.capture_ns, "freeze_ns": freeze_ns,
           "epoch_bounds_ns": list(bounds), "input_invalid_reason": invalid,
           "teacher_reasons": reasons, "history_row_ids": history, "observation_pose_row_ids": pose_refs,
           "stop_reason": "COLLECTION_INTERVENTION" if intervention else "UNKNOWN",
           "input_eligible": invalid is None,
           "usable_partial": invalid is None and teacher is not None and bool(teacher.xy_mask.any()),
           "usable_full": invalid is None and teacher is not None and bool(teacher.xy_mask.all())}
    return teacher, row


def intervention_from_probe(probe: dict[str, Any]) -> int:
    """Collection braking is provenance, never an inferred environment stop."""
    if probe.get("fault") is not None or probe.get("stop_confirmed") is not True:
        raise ValueError("collection run fault or stop confirmation missing")
    value = probe.get("brake_sim")
    if type(value) not in (float, int) or not np.isfinite(value) or value <= 0:
        raise ValueError("valid brake_sim seconds required")
    return int(Decimal(str(value)) * 1_000_000_000)


def _write_json(path: Path, value: Any) -> None:
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")


def materialize_time_run(run_dir: Path, destination: Path, *, run_id: str, split: str,
                         freeze_delay_ns: int = 0) -> dict[str, Any]:
    """Generate all camera anchors, independent masks, refs, and a real replay smoke.

freeze = camera receipt + a fixed explicit delay in receipt ns. Default zero
does not invent measured preprocessing latency; bag receipt is only a proxy.
"""
    if type(freeze_delay_ns) is not int or freeze_delay_ns < 0:
        raise ValueError("freeze delay must be nonnegative integer ns")
    if split not in {"train", "validation", "test"}:
        raise ValueError("invalid split")
    probe = json.loads((run_dir / "probe_summary.json").read_text(encoding="utf-8"))
    intervention = intervention_from_probe(probe)
    index = read_time_sqlite_run(run_dir, run_id)
    if len(index.epochs) != 1:
        raise ValueError("multiple clock epochs need per-epoch collection intervention annotations")
    destination.mkdir(parents=True, exist_ok=False)
    config = TimeDatasetConfig()
    windows = EventWindows(index.events)
    bounds = {e.epoch_id: (e.first_sim_stamp_ns, e.last_sim_stamp_ns) for e in index.epochs}
    # A duplicate capture timestamp is one decision opportunity. Freeze is that
    # first camera receipt; later arrivals remain in the index for later cuts.
    cameras: dict[tuple[str, int], TimeEvent] = {}
    for e in sorted(index.events, key=lambda e: (e.available_ns, e.sequence)):
        if e.role == "camera":
            cameras.setdefault((e.epoch, e.capture_ns), e)
    anchors = sorted(cameras.values(), key=lambda e: (e.epoch, e.capture_ns))
    n = len(anchors)
    xy = np.full((n, 30, 2), np.nan, np.float32)
    velocity = np.full((n, 30), np.nan, np.float32)
    xy_mask = np.zeros((n, 30), bool)
    velocity_mask = np.zeros((n, 30), bool)
    interval_mask = np.zeros((n, 30), bool)
    input_eligible = np.zeros(n, bool)
    usable_full = np.zeros(n, bool)
    usable_partial = np.zeros(n, bool)
    invalids: Counter[str] = Counter()
    reasons: Counter[str] = Counter()
    stops: Counter[str] = Counter()
    selected_smoke: list[tuple[TimeEvent, dict[str, Any]]] = []
    with (destination / "anchors.jsonl").open("x", encoding="utf-8") as output:
        for i, anchor in enumerate(anchors):
            teacher, row = audit_anchor(windows.at(anchor), anchor, config=config, bounds=bounds[anchor.epoch],
                freeze_ns=anchor.available_ns + freeze_delay_ns, intervention_ns=intervention)
            row.update({"label_index": i, "run_id": run_id, "split": split})
            if teacher is not None:
                xy[i], velocity[i] = teacher.xy_m, teacher.velocity_mps
                xy_mask[i], velocity_mask[i], interval_mask[i] = teacher.xy_mask, teacher.velocity_mask, teacher.interval_mask
            input_eligible[i], usable_full[i], usable_partial[i] = row["input_eligible"], row["usable_full"], row["usable_partial"]
            invalids.update([row["input_invalid_reason"] or "OK"])
            reasons.update(row["teacher_reasons"])
            stops.update([row["stop_reason"]])
            output.write(json.dumps(row, separators=(",", ":"), allow_nan=False) + "\n")
            if row["usable_full"] and len(selected_smoke) < 2:
                selected_smoke.append((anchor, row))
    np.savez_compressed(destination / "teachers.npz", xy_m=xy, xy_mask=xy_mask,
        velocity_mps=velocity, velocity_mask=velocity_mask, interval_mask=interval_mask,
        input_eligible=input_eligible, usable_full=usable_full, usable_partial=usable_partial,
        observation_ns=np.asarray([a.capture_ns for a in anchors], np.int64),
        freeze_ns=np.asarray([a.available_ns + freeze_delay_ns for a in anchors], np.int64),
        camera_row_id=np.asarray([a.sequence for a in anchors], np.int64))
    smoke = []
    for anchor, row in selected_smoke:
        # Load only sensors actually selected by the shared history selector.
        sensor_ids = {rid for role in ("camera", "lidar") for slot in row["history_row_ids"][role] for rid in slot}
        events = tuple(load_event(run_dir, e) if e.sequence in sensor_ids else e
                       for e in windows.at(anchor) if e.role not in {"camera", "lidar"} or e.sequence in sensor_ids)
        real_anchor = next(e for e in events if e.sequence == anchor.sequence)
        sample = assemble_time_sample(events, real_anchor, config=config,
            epoch_start_ns=bounds[anchor.epoch][0], epoch_end_ns=bounds[anchor.epoch][1],
            freeze_ns=row["freeze_ns"], intervention_ns=intervention)
        if sample.inputs is None or sample.teacher is None:
            raise ValueError(f"real replay failed: {sample.input_invalid_reason}; {sample.teacher_reasons}")
        j = row["label_index"]
        np.testing.assert_allclose(sample.teacher.xy_m, xy[j], rtol=0, atol=0, equal_nan=True)
        np.testing.assert_array_equal(sample.teacher.xy_mask, xy_mask[j])
        smoke.append({"anchor_id": row["anchor_id"], "status": "PASS", "image_shape": list(sample.inputs.image.shape),
                      "lidar_shape": list(sample.inputs.lidar.shape)})
    measured = np.asarray([e.payload.longitudinal_mps for e in index.events if e.role == "velocity"], dtype=np.float64)
    measured = measured[np.isfinite(measured)]
    report = {"run_id": run_id, "split": split, "raw_run_dir": str(run_dir.resolve()),
        "anchors": n, "duplicate_camera_captures": index.topic_counts.get("camera", n) - n,
        "input_eligible": int(input_eligible.sum()), "usable_partial": int(usable_partial.sum()),
        "usable_full": int(usable_full.sum()), "teacher_any": int(xy_mask.any(axis=1).sum()),
        "teacher_full": int(xy_mask.all(axis=1).sum()), "xy_support_per_horizon": xy_mask.sum(axis=0).tolist(),
        "velocity_support_per_horizon": velocity_mask.sum(axis=0).tolist(),
        "input_reasons": dict(invalids), "teacher_anchor_reasons": dict(reasons), "stop_reasons": dict(stops),
        "intervention_ns": intervention, "topic_counts": index.topic_counts,
        "sensor_metadata": index.sensor_metadata, "fallback_counts": index.fallback_counts,
        "sqlite_integrity": index.integrity,
        "epochs": [asdict(e) for e in index.epochs], "replay_smoke": smoke,
        "measured_longitudinal_mps_quantiles": np.quantile(measured, [0, .5, .95, 1]).tolist() if measured.size else None}
    _write_json(destination / "audit.json", report)
    (destination / "raw").symlink_to(run_dir.resolve(), target_is_directory=True)
    return report


def materialize_time_corpus(raw_root: Path, destination: Path, split_path: Path, *,
                            freeze_delay_ns: int = 0, only_run: str | None = None) -> dict[str, Any]:
    """Verify all sources before publishing separate train/validation/test paths."""
    manifest = json.loads(split_path.read_text(encoding="utf-8"))
    validate_time_split(manifest)
    if hashlib.sha256((raw_root / "MANIFEST.json").read_bytes()).hexdigest() != manifest["receipt_sha256"]:
        raise ValueError("packaging receipt identity mismatch")
    verified = verify_time_split_sources(manifest, raw_root)
    chosen = [r for r in verified["runs"] if only_run is None or r["run_id"] == only_run]
    if not chosen:
        raise ValueError("no matching run")
    destination.mkdir(parents=True, exist_ok=False)
    _write_json(destination / "split_verified.json", verified)
    _write_json(destination / "contract.json", {"format": "time_sqlite_corpus_v1", "dt_s": 0.1,
        "points": 30, "frame": "base_link_at_observation", "freeze_delay_receipt_ns": freeze_delay_ns,
        "availability": "bag_receipt_proxy_not_measured_preprocessing_completion",
        "config": asdict(TimeDatasetConfig()), "split_sha256": verified["manifest_sha256"],
        "pilot_only": only_run, "stop_probability": None, "runtime_ready": False})
    reports = []
    for run in chosen:
        report = materialize_time_run(raw_root / "runs" / run["run_id"],
            destination / run["split"] / run["run_id"], run_id=run["run_id"], split=run["split"],
            freeze_delay_ns=freeze_delay_ns)
        reports.append(report)
        print(json.dumps({key: report[key] for key in ("run_id", "anchors", "teacher_full", "usable_full")}), flush=True)
    totals = {}
    for split in ("train", "validation", "test"):
        selected = [r for r in reports if r["split"] == split]
        totals[split] = {key: sum(r[key] for r in selected) for key in
                        ("anchors", "input_eligible", "teacher_any", "teacher_full", "usable_partial", "usable_full")}
        totals[split]["runs"] = len(selected)
    report = {"format": "time_corpus_audit_v1", "source_hashes_verified": True,
        "split_sha256": verified["manifest_sha256"], "freeze_delay_receipt_ns": freeze_delay_ns,
        "scope": verified["scope"], "pilot_only": only_run, "splits": totals, "runs": reports,
        "training_executed": False, "test_model_evaluation_executed": False}
    _write_json(destination / "audit.json", report)
    return report
