"""Bounded real-bag replay and timing; train split only, no model optimization.

Outputs report.json and an immutable future-target cache under --output.
DataLoader timings include CPU assembly from selected decoded events, not bag
I/O. Raw hashing, indexing, decoding, and cache I/O are timed separately.
"""
from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict, fields, replace
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time
from typing import Any, Sequence

import numpy as np
import torch
from torch.utils.data import DataLoader

from aic_transfuser_lite.data.future_sequence_dataset_v1 import (
    FutureSequenceCache, FutureSequenceConfig, FutureSequenceTargets,
    TemporalTrainingDataset, TemporalTrainingSample, _nearest,
    build_future_sequence_targets, build_temporal_training_sample, cache_identity,
)
from aic_transfuser_lite.data.time_corpus_v1 import EventWindows, intervention_from_probe
from aic_transfuser_lite.data.time_dataset_v1 import TimeDatasetConfig, _eligible
from aic_transfuser_lite.data.time_history_v1 import TimeEvent, select_time_history
from aic_transfuser_lite.data.time_split_v1 import content_sha256, validate_time_split
from aic_transfuser_lite.data.time_sqlite_reader_v1 import load_event, read_time_sqlite_run


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, value: Any) -> None:
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write("\n")


def select_rows(rows: list[dict[str, Any]], count: int) -> list[dict[str, Any]]:
    """Keep start/end, a missing input and intervention boundary in the audit."""
    if not rows or not 4 <= count <= 128:
        raise ValueError("need nonempty anchors and 4..128 samples per run")
    indices = {0, len(rows) - 1}
    for predicate in (lambda r: not r["input_eligible"],
                      lambda r: r["stop_reason"] == "COLLECTION_INTERVENTION"):
        match = next((i for i, row in enumerate(rows) if predicate(row)), None)
        if match is not None:
            indices.add(match)
    for i in np.linspace(0, len(rows) - 1, min(len(rows), count), dtype=int):
        if len(indices) >= min(len(rows), count):
            break
        indices.add(int(i))
    return [rows[i] for i in sorted(indices)]


def decode_window(raw: Path, events: Sequence[TimeEvent], anchor: TimeEvent,
                  freeze: int, bounds: tuple[int, int], config: TimeDatasetConfig,
                  future: FutureSequenceConfig, decoded: dict[int, TimeEvent]) -> tuple[TimeEvent, ...]:
    """Decode only selected historical/future sensors; retain compact states."""
    needed = {anchor.sequence}
    eligible = _eligible(events, anchor, config, freeze, bounds)
    for role, length in (("camera", config.camera_history_length), ("lidar", config.lidar_history_length)):
        selected = select_time_history(
            eligible, role=role, run=anchor.run, epoch=anchor.epoch,
            capture_clock=config.capture_clock, available_clock=config.available_clock,
            freeze_ns=freeze, observation_ns=anchor.capture_ns, length=length,
            tolerance_ns=config.tolerance_ns,
        )
        needed.update(e.sequence for e in selected if e is not None)
        for horizon in future.horizons_sec:
            event = _nearest(events, anchor, role, anchor.capture_ns + round(horizon * 1e9),
                             future.tolerance_ns, bounds)
            if event is not None:
                needed.add(event.sequence)
    result = []
    for event in events:
        if event.role in {"camera", "lidar"}:
            if event.sequence not in needed:
                continue
            if event.sequence not in decoded:
                decoded[event.sequence] = load_event(raw, event)
            event = decoded[event.sequence]
        result.append(event)
    return tuple(result)


def target_digest(target: FutureSequenceTargets) -> str:
    digest = hashlib.sha256()
    for field in fields(target):
        value = getattr(target, field.name)
        digest.update(field.name.encode())
        digest.update(value.numpy().tobytes())
    return digest.hexdigest()


def assert_future_isolation(events: Sequence[TimeEvent], anchor: TimeEvent, freeze: int,
                            bounds: tuple[int, int], intervention: int,
                            config: TimeDatasetConfig, future: FutureSequenceConfig,
                            original: TemporalTrainingSample) -> None:
    changed = []
    cutoff = anchor.capture_ns + round(min(future.horizons_sec) * 1e9) - future.tolerance_ns
    for event in events:
        payload = event.payload
        if event.capture_ns >= cutoff:
            if event.role == "camera":
                payload = replace(payload, image_rgb=255 - payload.image_rgb)
            elif event.role == "lidar":
                payload = replace(payload, ranges_m=np.full_like(payload.ranges_m, 7.0))
            elif event.role == "velocity":
                payload = replace(payload, longitudinal_mps=payload.longitudinal_mps + 2.0)
            elif event.role in {"actual_steering", "final_command"}:
                payload = replace(payload, steering_rad=payload.steering_rad + 0.2)
        changed.append(replace(event, payload=payload))
    altered = build_temporal_training_sample(
        changed, anchor, dataset_config=config, future_config=future,
        epoch_bounds=bounds, freeze_ns=freeze, intervention_ns=intervention,
    )
    assert original.policy.inputs is not None and altered.policy.inputs is not None
    for field in fields(original.policy.inputs):
        first, second = getattr(original.policy.inputs, field.name), getattr(altered.policy.inputs, field.name)
        if isinstance(first, torch.Tensor):
            torch.testing.assert_close(first, second, rtol=0, atol=0)
        else:
            assert first == second
    assert original.policy.inputs.targets is None
    for name in ("image", "lidar", "ego", "applied_action"):
        assert not torch.equal(getattr(original.future, name), getattr(altered.future, name)), name


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--corpus-root", type=Path, required=True)
    parser.add_argument("--run-ids", nargs="+", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--samples-per-run", type=int, default=32)
    parser.add_argument("--batch-size", type=int, default=4)
    parser.add_argument("--threads", type=int, default=2)
    parser.add_argument("--max-source-gib", type=float, default=4.0)
    args = parser.parse_args()
    if (not 1 <= len(args.run_ids) <= 2 or len(set(args.run_ids)) != len(args.run_ids)
            or not 4 <= args.samples_per_run <= 128 or not 1 <= args.batch_size <= 16
            or not 1 <= args.threads <= 8 or not 0 < args.max_source_gib <= 8):
        parser.error("bounded probe requires 1..2 runs, 4..128 samples/run, batch 1..16, threads 1..8, source <=8 GiB")
    torch.set_num_threads(args.threads)
    corpus = args.corpus_root.resolve(strict=True)
    split = read_json(corpus / "split_verified.json")
    validate_time_split(split, require_verified=True)
    runs = {row["run_id"]: row for row in split["runs"]}
    if any(rid not in runs or runs[rid]["split"] != "train" for rid in args.run_ids):
        raise ValueError("only existing train runs may be probed")
    contract = read_json(corpus / "contract.json")
    config, future = TimeDatasetConfig(), FutureSequenceConfig()
    if (contract["config"] != json.loads(json.dumps(asdict(config)))
            or contract["split_sha256"] != split["manifest_sha256"]):
        raise ValueError("corpus contract mismatch")
    raw_dirs = {rid: (corpus / "train" / rid / "raw").resolve(strict=True) for rid in args.run_ids}
    total_bytes = sum(p.stat().st_size for raw in raw_dirs.values() for p in raw.glob("bag/*.db3"))
    if total_bytes > args.max_source_gib * 1024 ** 3:
        raise ValueError("source byte budget exceeded")
    args.output.mkdir(parents=True, exist_ok=False)
    plan = {"run_ids": args.run_ids, "samples_per_run": args.samples_per_run,
            "source_bytes": total_bytes, "batch_size": args.batch_size, "num_workers": 0,
            "threads": args.threads, "training_steps": 0, "test_used": False,
            "git_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip()}
    write_json(args.output / "plan.json", plan)
    artifact_inventory = {row["path"]: row for row in read_json(corpus / "artifact_manifest.json")["files"]}
    records, selected_rows, run_reports = [], [], []
    epoch_bounds, interventions = {}, {}
    source_inventory = []
    reference = []
    start = time.perf_counter()
    for rid in args.run_ids:
        source = corpus / "train" / rid
        raw = raw_dirs[rid]
        started = time.perf_counter()
        files = sorted(raw.glob("bag/*.db3")) + sorted((raw / "types").rglob("*.idl")) + [raw / "probe_summary.json"]
        expected = {s["path"].split(f"runs/{rid}/", 1)[1]: s["sha256"] for s in runs[rid]["sources"]}
        for path in files:
            digest = sha256(path)
            relative = path.relative_to(raw).as_posix()
            if path.suffix == ".db3" and expected.get(relative) != digest:
                raise ValueError(f"raw source changed: {path}")
            source_inventory.append({"run_id": rid, "path": relative, "sha256": digest})
        for name in ("anchors.jsonl", "teachers.npz", "audit.json"):
            relative = f"train/{rid}/{name}"
            if sha256(source / name) != artifact_inventory[relative]["sha256"]:
                raise ValueError(f"corpus artifact changed: {relative}")
        hash_sec = time.perf_counter() - started
        rows = [json.loads(line) for line in (source / "anchors.jsonl").read_text().splitlines() if line.strip()]
        selected = select_rows(rows, args.samples_per_run)
        with np.load(source / "teachers.npz", allow_pickle=False) as labels:
            reference.extend({k: labels[k][row["label_index"]].copy()
                              for k in ("xy_m", "xy_mask", "velocity_mps", "velocity_mask")} for row in selected)
        started = time.perf_counter()
        index = read_time_sqlite_run(raw, rid)
        index_sec = time.perf_counter() - started
        if len(index.epochs) != 1:
            raise ValueError("per-epoch intervention annotations required for multiple epochs")
        intervention = intervention_from_probe(read_json(raw / "probe_summary.json"))
        for epoch in index.epochs:
            epoch_bounds[rid, epoch.epoch_id] = (epoch.first_sim_stamp_ns, epoch.last_sim_stamp_ns)
            interventions[rid, epoch.epoch_id] = intervention
        by_id = {e.sequence: e for e in index.events}
        windows, decoded = EventWindows(index.events), {}
        started = time.perf_counter()
        for row in selected:
            anchor = by_id[row["camera_row_id"]]
            bounds = epoch_bounds[rid, anchor.epoch]
            if (anchor.capture_ns != row["observation_ns"] or list(bounds) != row["epoch_bounds_ns"]
                    or row["freeze_ns"] != anchor.available_ns + contract["freeze_delay_receipt_ns"]):
                raise ValueError("anchor/freeze/epoch drift")
            events = decode_window(raw, windows.at(anchor), anchor, row["freeze_ns"], bounds, config, future, decoded)
            records.append((events, decoded[anchor.sequence], row["freeze_ns"]))
            selected_rows.append(row)
        run_reports.append({"run_id": rid, "split": "train", "source_anchors": len(rows),
            "selected": len(selected), "hash_sec": hash_sec, "index_sec": index_sec,
            "decode_sec": time.perf_counter() - started, "decoded_sensors": len(decoded),
            "integrity": index.integrity, "fallback_counts": index.fallback_counts,
            "sensor_metadata": index.sensor_metadata, "intervention_ns": intervention,
            "recorded_receipt_span_sec": (index.epochs[0].last_bag_stamp_ns - index.epochs[0].first_bag_stamp_ns) / 1e9})
        print(json.dumps(run_reports[-1]), flush=True)
    ready_sec = time.perf_counter() - start
    dataset = TemporalTrainingDataset(records, dataset_config=config, future_config=future,
                                      epoch_bounds=epoch_bounds, intervention_ns=interventions)
    started = time.perf_counter()
    samples = [sample for batch in DataLoader(dataset, batch_size=args.batch_size, num_workers=0,
                                              collate_fn=list) for sample in batch]
    loader_sec = time.perf_counter() - started
    details, isolation = [], []
    for sample, row, labels in zip(samples, selected_rows, reference):
        sample.validate()
        sample.future.validate(image_shape=config.image_shape, lidar_shape=config.lidar_shape)
        assert (sample.policy.inputs is not None) == row["input_eligible"]
        if sample.policy.teacher is None:
            assert not labels["xy_mask"].any() and not labels["velocity_mask"].any()
        else:
            for name, expected_array in labels.items():
                np.testing.assert_allclose(getattr(sample.policy.teacher, name), expected_array,
                                           rtol=0, atol=0, equal_nan=True)
        if sample.policy.inputs is not None:
            sample.policy.inputs.validate(require_current=False)
            assert sample.policy.inputs.targets is None
        teacher = sample.policy.teacher
        details.append({"anchor_id": row["anchor_id"], "input_valid": sample.policy.inputs is not None,
            "input_reason": sample.policy.input_invalid_reason,
            "xy_points_valid": int(teacher.xy_mask.sum()) if teacher is not None else 0,
            "target_speed_valid": bool(sample.target_speed_mask.item()), "stop_valid": bool(sample.stop_mask.item()),
            "stop_reason": sample.policy.stop_reason, "future_image": sample.future.image_mask.tolist(),
            "future_lidar": sample.future.lidar_mask.tolist(), "future_ego": sample.future.ego_mask.tolist(),
            "future_action": sample.future.applied_action_mask.tolist(),
            "source_timestamp_ns": sample.future.source_timestamp_ns.tolist()})
    for rid in args.run_ids:
        i = next(i for i, s in enumerate(samples) if s.policy.run == rid and s.policy.inputs is not None
                 and s.future.image_mask.all() and s.future.lidar_mask.all() and s.future.applied_action_mask.all())
        events, anchor, freeze = records[i]
        assert_future_isolation(events, anchor, freeze, epoch_bounds[rid, anchor.epoch],
                                interventions[rid, anchor.epoch], config, future, samples[i])
        isolation.append(samples[i].policy.anchor_id)
    identity = cache_identity(source_sha256=content_sha256(source_inventory), dataset_config=config, future_config=future)
    cache = FutureSequenceCache(args.output / "cache", identity)
    started = time.perf_counter()
    expected_digests = {}
    for (events, anchor, _), sample in zip(records, samples):
        value, reused = cache.get_or_build(sample.policy.anchor_id, lambda: build_future_sequence_targets(
            events, anchor, dataset_config=config, future_config=future, epoch_bounds=epoch_bounds[anchor.run, anchor.epoch]))
        assert not reused
        expected_digests[sample.policy.anchor_id] = target_digest(value)
        assert expected_digests[sample.policy.anchor_id] == target_digest(sample.future)
    cold_sec = time.perf_counter() - started
    write_json(args.output / "cache_expected.json", expected_digests)
    # A fresh interpreter must reuse every record without calling the builder.
    child = """
import json, sys, time
from pathlib import Path
import torch
sys.path.insert(0, 'tools')
from check_future_sequence_real_data import FutureSequenceCache, read_json, target_digest
torch.set_num_threads(int(sys.argv[2]))
root=Path(sys.argv[1]); cache=FutureSequenceCache(root/'cache', read_json(root/'cache'/'identity.json'))
def forbidden(): raise RuntimeError('cache unexpectedly rebuilt')
t=time.perf_counter(); count=0
for key,digest in read_json(root/'cache_expected.json').items():
    value,reused=cache.get_or_build(key, forbidden)
    assert reused and target_digest(value)==digest
    count+=1
print(json.dumps({'reused':count, 'read_and_verify_sec':time.perf_counter()-t}))
"""
    reopened = json.loads(subprocess.check_output([sys.executable, "-c", child, str(args.output.resolve()),
                                                   str(args.threads)], text=True))
    report = {"status": "PASS", "plan": plan, "runs": run_reports,
        "counts": {"samples": len(samples), "valid_inputs": sum(r["input_valid"] for r in details),
            "usable_full_xy": sum(r["input_valid"] and r["xy_points_valid"] == 30 for r in details),
            "target_speed_valid": sum(r["target_speed_valid"] for r in details),
            "stop_valid": sum(r["stop_valid"] for r in details),
            "input_reasons": dict(Counter(r["input_reason"] or "OK" for r in details)),
            "stop_reasons": dict(Counter(r["stop_reason"] for r in details)),
            **{name: np.asarray([r[name] for r in details], dtype=int).sum(axis=0).tolist()
               for name in ("future_image", "future_lidar", "future_ego", "future_action")}},
        "timing": {"existing_bags_to_decoded_records_sec": ready_sec, "dataloader_assembly_sec": loader_sec,
            "existing_bags_to_assembled_samples_sec": ready_sec + loader_sec,
            "assembly_samples_per_sec": len(samples) / loader_sec,
            "future_cache_build_write_verify_sec": cold_sec, "fresh_process_cache": reopened,
            "cache_bytes": sum(p.stat().st_size for p in (args.output / "cache").rglob('*') if p.is_file())},
        "checks": {"original_il_teacher_replay": "PASS", "future_mutation_input_unchanged": isolation,
                   "dinov3_pretrained_forward_backward": "NOT_RUN_WEIGHTS_MISSING"},
        "limitations": ["Existing AWSIM bags, not physical vehicle data; no new collection or closed-loop trial.",
            "Train-run representative audit includes boundaries; no frame-random split or throughput extrapolation.",
            "Receipt freeze is camera receipt + 50 ms, a proxy, not measured preprocessing completion.",
            "applied_action is the recorded final command at message stamp; physical actuator timing is unverified.",
            "No explicit environment stop annotations; stop masks remain false.",
            "OS file caches are uncontrolled; future cache timings exclude policy inputs and encoder inference."],
        "identity": identity, "sources": source_inventory, "samples": details}
    write_json(args.output / "report.json", report)
    print(json.dumps({k: report[k] for k in ("status", "counts", "timing", "checks")}), flush=True)


if __name__ == "__main__":
    main()
