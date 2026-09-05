"""One bounded train-only diagnostic experiment. No runtime promotion or raw I/O."""
from __future__ import annotations

from collections import defaultdict
from copy import deepcopy
import csv
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import io
import json
import math
from pathlib import Path, PurePosixPath
import random
import shutil
import subprocess
import time
import traceback

import numpy as np
import torch
from torch.nn import functional as F
import yaml

from aic_transfuser_lite.data.spatial_diagnostic_view_v4 import GRID, TEACHER_CONTRACT, spatial_target, polyline_length, self_intersects
from aic_transfuser_lite.data.spatial_diagnostic_inputs_v4 import (
    INPUT_CONTRACT, INPUT_FIELDS, FEATURES, EGO_LIMITS, BOUNDS, build_inputs, combine_inputs, epoch_keys,
)
from aic_transfuser_lite.data.dataset_view_v3 import _ego_row
from aic_transfuser_lite.models.spatial_path_diagnostic_v4 import SpatialPathDiagnosticV4

DATASET_ID = "181cf909b80589110574859990b0885005b7f9a0bb07cff1c24f38d6b090f388"
MANIFEST_HASH = "d625f42ca05a18ea76952376c6392268191c4895d6e605e0c49ceaa66dcbe1de"
SPLIT_HASH = "7d0e433dbd032ad695227051573e7d8d17072fa4ea3b4e28f4c44f56fde27b4f"
LEDGER_HASH = "35781616e8faab5117b0d9da7c8560519c5f196383ea66ab1465d999f5645e35"
FIXED = {"seed": 42, "max_candidates": 2048, "max_train": 64, "max_observation": 16,
    "steps": 500, "active_seconds": 1800, "final_eval_reserve_seconds": 60, "microbatch": 2,
    "accumulation": 4, "backbone_lr": 1e-4, "head_lr": 1e-3, "weight_decay": 0.0,
    "beta_m": 0.1, "clip_norm": 1.0, "eval_every": 50, "precision": "float32",
    "augmentation": False, "scheduler": None, "initialization": "scratch", "smoke_steps": 1}


def sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def clean(value: object) -> object:
    if isinstance(value, dict):
        return {str(k): clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    if isinstance(value, np.ndarray):
        return clean(value.tolist())
    if isinstance(value, np.generic):
        return clean(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def digest(value: object) -> str:
    return sha(json.dumps(clean(value), sort_keys=True, ensure_ascii=False, separators=(",", ":"), allow_nan=False).encode())


def write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(clean(value), indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")


def check_hash(path: Path, expected: str) -> bytes:
    data = path.read_bytes()
    if sha(data) != expected:
        raise ValueError("identity mismatch: " + path.name)
    return data


class CanonicalAccess:
    def __init__(self, root: Path, split_path: Path):
        self.root = root
        self.manifest_bytes = check_hash(root / "manifest.yaml", MANIFEST_HASH)
        self.manifest = yaml.load(self.manifest_bytes, Loader=yaml.CSafeLoader)
        claimed = self.manifest["manifest_sha256"]
        if claimed != DATASET_ID or digest({k: v for k, v in self.manifest.items() if k != "manifest_sha256"}) != claimed:
            raise ValueError("Dataset internal identity mismatch")
        if tuple(self.manifest.get(k) for k in ("coordinate_frame", "distance_unit", "angle_unit", "time_unit")) != ("base_link@t_obs", "m", "rad", "s"):
            raise ValueError("Dataset frame/units unknown")
        split = json.loads(check_hash(split_path, SPLIT_HASH))
        if split["dataset_manifest_sha256"] != DATASET_ID:
            raise ValueError("split identity mismatch")
        assignments = split["assignments"]
        if len({r["run_id"] for r in assignments}) != len(assignments):
            raise ValueError("duplicate split run")
        self.train_runs = {r["run_id"] for r in assignments if r["split"] == "train"}
        self.inventory = {f["path"]: f for f in self.manifest["files"]}
        if len(self.inventory) != len(self.manifest["files"]):
            raise ValueError("duplicate asset inventory")
        self.read_hashes: dict[str, str] = {}
        self.allowed_sensor_paths: set[str] = set()
        self.future_blobs: dict[str, bytes] = {}
        rows = list(csv.DictReader(io.StringIO(self.read("samples.csv").decode())))
        runs = list(csv.DictReader(io.StringIO(self.read("runs.csv").decode())))
        source_splits: dict[str, set] = defaultdict(set)
        split_by_run = {r["run_id"]: r["split"] for r in assignments}
        for r in runs:
            source_splits[r["source_hash"]].add(split_by_run[r["run_id"]])
        if any(len(s) > 1 for s in source_splits.values()):
            raise ValueError("source hash spans splits")
        self.rows = [r for r in rows if r["run_id"] in self.train_runs]
        self.rows.sort(key=lambda r: (r["run_id"], r["segment_id"], int(r["grid_stamp_ns"])))
        if len({r["sample_id"] for r in rows}) != len(rows):
            raise ValueError("duplicate sample ID")
        self.summary = {"rows": len(rows), "runs": len(runs), "train_metadata_rows": len(self.rows),
                        "split_runs": {s: sum(r["split"] == s for r in assignments) for s in ("train", "validation", "test")},
                        "source_hash_split_overlap": 0, "scene_independence": "UNKNOWN"}

    def read(self, relative: str) -> bytes:
        path = PurePosixPath(relative)
        if path.is_absolute() or ".." in path.parts or "\\" in relative or relative not in self.inventory:
            raise ValueError("untrusted canonical asset locator")
        if relative not in ("samples.csv", "runs.csv"):
            if path.parts[0] == "trajectories":
                if relative not in self.future_blobs and len(self.future_blobs) >= FIXED["max_candidates"]:
                    raise ValueError("future candidate budget exceeded")
            elif relative not in self.allowed_sensor_paths:
                raise ValueError("unselected sensor access")
        current = self.root
        for component in path.parts:
            current = current / component
            if current.is_symlink():
                raise ValueError("canonical symlink forbidden")
        data = current.read_bytes()
        item = self.inventory[relative]
        if len(data) != item["size_bytes"] or sha(data) != item["sha256"]:
            raise ValueError("selected canonical asset identity mismatch: " + relative)
        self.read_hashes[relative] = sha(data)
        if path.parts[0] == "trajectories":
            self.future_blobs[relative] = data
        return data


def deterministic_candidates(rows: list[dict]) -> list[int]:
    rng = random.Random(42)
    per_run: dict[str, list[int]] = defaultdict(list)
    for i, row in enumerate(rows):
        per_run[row["run_id"]].append(i)
    for values in per_run.values():
        rng.shuffle(values)
    order = []
    keys = sorted(per_run)
    while len(order) < FIXED["max_candidates"] and any(per_run.values()):
        for key in keys:
            if per_run[key] and len(order) < FIXED["max_candidates"]:
                order.append(per_run[key].pop())
    return order


def prepare_selection(access: CanonicalAccess, ledger_path: Path, output: Path) -> tuple[list[dict], dict, list[dict]]:
    # Read-only ledger is a candidate annotation source, not an acquisition plan.
    annotations = {}
    if ledger_path.is_file():
        blob = check_hash(ledger_path, LEDGER_HASH)
        annotations = {r["sample_id"]: r for r in csv.DictReader(io.StringIO(blob.decode())) if r["split"] == "train"}
    candidates, views = [], {}
    for index in deterministic_candidates(access.rows):
        row = access.rows[index]
        future = np.load(io.BytesIO(access.read(row["trajectory_path"])), allow_pickle=False)
        view = spatial_target(future)
        views[row["sample_id"]] = view
        annotation = annotations.get(row["sample_id"], {})
        _, ego_mask = _ego_row(row, FEATURES, abs_limits=EGO_LIMITS)
        reason = []
        if not bool(ego_mask.all()):
            reason.append("invalid_current_ego")
        if not np.isfinite(future[future[:, 7] == 1, :7]).all():
            reason.append("nonfinite_valid_future")
        if abs(float(row["velocity_longitudinal_mps"])) <= 0.05:
            reason.append("stopped_observation")
        if view["processed_length_m"] is None or view["processed_length_m"] < 0.5:
            reason.append("short_or_unknown_support")
        reason.extend(view["flags"])
        if view["cut_reason"] in ("position_jump", "invalid_time_grid_or_gap", "nonfinite_valid"):
            reason.append(view["cut_reason"])
        candidates.append({"sample_id": row["sample_id"], "row_index": index, "run_id": row["run_id"],
            "segment_id": row["segment_id"], "stamp_ns": int(row["grid_stamp_ns"]), "split": "train",
            "collection_slice": annotation.get("collection_case", "UNKNOWN"),
            "normal_recovery": annotation.get("normal_recovery", "UNKNOWN"),
            "shape": view["shape"], "shape_provenance": "future geometry diagnostic NOT route intent",
            "confirmed_episode": annotation.get("confirmed_episode_id") or "UNKNOWN",
            "estimated_episode": annotation.get("estimated_episode_id") or "UNKNOWN",
            "quality_flags": annotation.get("quality_flags", "UNKNOWN"),
            "existing_filter_flags": annotation.get("v3_exclusion_flags", "UNKNOWN"),
            "source_quality": "UNKNOWN_NOT_RAW_REVERIFIED", "hold_reasons_unknown": True,
            "raw_support_m": view["raw_length_m"], "processed_support_m": view["processed_length_m"],
            "path_points": int(view["mask"].sum()), "cut_reason": view["cut_reason"],
            "reasons": reason, "current_ego_valid": bool(ego_mask.all())})
    buckets: dict[tuple, list] = defaultdict(list)
    for c in candidates:
        if not c["reasons"]:
            buckets[(c["run_id"], c["shape"])].append(c)
    selected = []
    while len(selected) < 64 and any(buckets.values()):
        for key in sorted(buckets):
            if not buckets[key] or len(selected) >= 64:
                continue
            item = buckets[key].pop(0)
            if all(other["run_id"] != item["run_id"] or abs(other["stamp_ns"]-item["stamp_ns"]) >= 500_000_000 for other in selected):
                selected.append({**item, "role": "train", "selection_reason": "moving supported; run/shape round robin; >=0.5s same-run gap"})
    used = {c["sample_id"] for c in selected}
    observation_buckets: dict[tuple, list] = defaultdict(list)
    for c in candidates:
        if c["sample_id"] not in used and c["current_ego_valid"]:
            key = (c["cut_reason"], c["reasons"][0] if c["reasons"] else "heldout_train_observation", c["normal_recovery"])
            observation_buckets[key].append(c)
    observations = []
    while len(observations) < 16 and any(observation_buckets.values()):
        for key in sorted(observation_buckets):
            if observation_buckets[key] and len(observations) < 16:
                observations.append({**observation_buckets[key].pop(0), "role": "observation_only", "selection_reason": "not added to optimizer; diagnostic cohort"})
    selection = {"seed": 42, "candidate_count": len(candidates), "train_count": len(selected),
                 "observation_count": len(observations), "selected": selected+observations, "candidates": candidates}
    selection["identity"] = digest(selection)
    write_json(output / "artifacts/selection.json", selection)  # Fixed BEFORE model/training.
    chosen = selection["selected"]
    if not selected:
        raise ValueError("no train examples with path support")
    targets, masks, metadata = [], [], []
    for item in chosen:
        row, view = access.rows[item["row_index"]], views[item["sample_id"]]
        destination = output / "evidence/canonical_futures" / (item["sample_id"] + ".npy")
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(access.future_blobs[row["trajectory_path"]])
        metadata.append({"canonical": row, "future_sha256": access.read_hashes[row["trajectory_path"]],
                         "geometry": {k: v for k, v in view.items() if k not in ("xy", "mask", "grid_m", "raw_prefix_xy")}})
        targets.append(view["xy"])
        masks.append(view["mask"])
    write_json(output / "evidence/selected_sample_metadata.json", metadata)
    np.savez_compressed(output / "evidence/spatial_targets.npz", sample_ids=np.array([c["sample_id"] for c in chosen]),
                        xy=np.stack(targets), mask=np.stack(masks), grid_m=GRID)
    return chosen, selection, [views[c["sample_id"]] for c in chosen]


def masked_xy_loss(predicted: torch.Tensor, target: torch.Tensor, mask: torch.Tensor) -> torch.Tensor | None:
    """Anchor-balanced SmoothL1(beta=0.1m). No origin/tail/zero-support anchors."""
    if predicted.shape != target.shape or predicted.shape[-2:] != (20, 2) or mask.shape != target.shape[:2] or mask.dtype != torch.bool:
        raise ValueError("XY loss shape/mask mismatch")
    if not torch.isfinite(predicted).all() or not torch.isfinite(target[mask]).all():
        raise ValueError("nonfinite valid loss input")
    present = mask.any(dim=1)
    if not present.any():
        return None
    safe_target = torch.where(mask[..., None], target, torch.zeros_like(target))
    safe_prediction = torch.where(mask[..., None], predicted, torch.zeros_like(predicted))
    point = F.smooth_l1_loss(safe_prediction, safe_target, beta=0.1, reduction="none").mean(dim=-1)
    return (point.sum(dim=1)[present] / mask.sum(dim=1)[present]).mean()


def metrics(predictions: np.ndarray, target: np.ndarray, mask: np.ndarray, selected: list[dict], views: list[dict]) -> dict:
    if predictions.shape != target.shape or not np.isfinite(predictions).all():
        raise ValueError("invalid evaluation predictions")
    rows = []
    for i, item in enumerate(selected):
        valid = mask[i]
        error = np.linalg.norm(predictions[i, valid]-target[i, valid], axis=-1)
        delta = np.abs(predictions[i, valid]-target[i, valid])
        loss = float(np.where(delta < .1, .5*delta**2/.1, delta-.05).mean()) if len(error) else None
        rows.append({**{k: item[k] for k in ("sample_id", "run_id", "collection_slice", "normal_recovery", "shape", "role")},
            "ade_m": float(error.mean()) if len(error) else None, "loss": loss, "valid_points": int(valid.sum()),
            "raw_teacher_length_m": views[i]["raw_length_m"], "processed_teacher_length_m": views[i]["processed_length_m"],
            "teacher_resampled_length_m": views[i]["resampled_length_m"],
            "predicted_length_on_teacher_support_m": polyline_length(predictions[i, valid]) if valid.any() else None,
            "finite_all_prediction_points": True, "origin_concentration": bool(np.linalg.norm(predictions[i], axis=1).max() < .02),
            "self_intersection": self_intersects(predictions[i]), "curvature": None, "curvature_reason": "NOT_COMPUTED_DIAGNOSTIC_OPTIONAL",
            **{f"error_{distance:.1f}m": float(np.linalg.norm(predictions[i, k]-target[i, k])) if valid[k] else None
               for distance, k in ((.5, 4), (1., 9), (1.5, 14), (2., 19))}})
    def aggregate(part: list[dict]) -> dict:
        def mean(key: str) -> float | None:
            values = [r[key] for r in part if r[key] is not None]
            return float(np.mean(values)) if values else None
        return {"anchors": len(part), "loss_anchors": sum(r["valid_points"] > 0 for r in part),
            "valid_points": sum(r["valid_points"] for r in part), "ade_m": mean("ade_m"), "loss": mean("loss"),
            "origin_concentrated_count": sum(r["origin_concentration"] for r in part),
            "self_intersection_count": sum(r["self_intersection"] for r in part),
            "distance_errors": {f"{d:.1f}m": {"error_m": mean(f"error_{d:.1f}m"),
                "anchors": sum(r[f"error_{d:.1f}m"] is not None for r in part)} for d in (.5, 1., 1.5, 2.)}}
    groups = {}
    for key in ("role", "run_id", "collection_slice", "shape", "normal_recovery"):
        groups[key] = {label: aggregate([r for r in rows if str(r[key]) == label]) for label in sorted({str(r[key]) for r in rows})}
    return {"rows": rows, "groups": groups, "train": aggregate([r for r in rows if r["role"] == "train"]),
            "observation": aggregate([r for r in rows if r["role"] == "observation_only"])}


def optimizer_for(model: SpatialPathDiagnosticV4) -> torch.optim.Optimizer:
    return torch.optim.AdamW([{"params": model.backbone.parameters(), "lr": 1e-4},
                              {"params": model.path_head.parameters(), "lr": 1e-3}], weight_decay=0.0)


def checkpoint(path: Path, model: SpatialPathDiagnosticV4) -> dict:
    path.parent.mkdir(parents=True, exist_ok=True)
    torch.save({"format": "SPATIAL_DIAGNOSTIC_NOT_RUNTIME", "state_dict": model.state_dict()}, path)
    return {"name": path.name, "sha256": sha(path.read_bytes()), "bytes": path.stat().st_size,
            "parameters": sum(p.numel() for p in model.parameters()), "packaged": False}


def utc() -> str:
    return datetime.now(timezone.utc).isoformat()


def run_experiment(config_path: Path, output: Path) -> int:
    if output.exists():
        raise FileExistsError("immutable diagnostic run already exists")
    config = yaml.safe_load(config_path.read_text())
    if config["diagnostic"] != FIXED:
        raise ValueError("diagnostic hyperparameters/limits differ from fixed contract")
    root = Path(config["dataset_root"])
    if root.resolve() == output.resolve() or root.resolve() in output.resolve().parents:
        raise ValueError("output overlaps canonical")
    output.mkdir(parents=True)
    for folder in ("artifacts", "evidence", "provenance", "figures", "logs", "checkpoints"):
        (output / folder).mkdir()
    repo = Path(__file__).resolve().parents[3]
    commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=repo, text=True).strip()
    dirty = subprocess.check_output(["git", "status", "--porcelain"], cwd=repo, text=True)
    manifest = {"mode": "OBSERVED_DIAGNOSTIC_ONLY", "execution_commit": commit, "dirty": dirty, "start_utc": utc(),
        "status": "STARTED", "dataset_identity": DATASET_ID, "raw_reads_performed": 0,
        "runtime_deployment_approved": False, "teacher_adoption_approved": False, "steps": 0,
        "smoke_steps": 0, "optimizer_skips": 0, "val_test_asset_reads": 0,
        "implementation_complete": True, "overfit_executed": False, "fit_target_met": None}
    write_json(output / "artifacts/execution_manifest.json", manifest)
    try:
        if dirty:
            raise ValueError("execution checkout dirty")
        tracked = subprocess.check_output(["git", "ls-files", "src", "tools/train_spatial_diagnostic_v4.py",
            "configs/spatial_diagnostic_v4.yaml"], cwd=repo, text=True).splitlines()
        manifest["code_hashes"] = {name: sha((repo / name).read_bytes()) for name in tracked}
        manifest["permitted_scope"] = "train canonical only; bounded diagnostic; no raw, val/test assets, runtime, push"
        (output / "artifacts/resolved_config.yaml").write_text(yaml.safe_dump(config, sort_keys=True))
        write_json(output / "artifacts/input_contract.json", {**INPUT_CONTRACT, "identity": digest(INPUT_CONTRACT)})
        write_json(output / "artifacts/teacher_contract.json", {**TEACHER_CONTRACT, "identity": digest(TEACHER_CONTRACT)})
        env = {"torch": torch.__version__, "numpy": np.__version__, "cuda": torch.version.cuda,
            "python": __import__("sys").version,
            "gpu_available": torch.cuda.is_available(), "free_disk_bytes": shutil.disk_usage(output).free,
            "device": torch.cuda.get_device_name(0) if torch.cuda.is_available() else None,
            "command": __import__("sys").argv}
        write_json(output / "provenance/environment.json", env)
        if not torch.cuda.is_available():
            raise RuntimeError("BLOCKED_NO_GPU; synthetic tests only")
        access = CanonicalAccess(root, Path(config["split_manifest"]))
        write_json(output / "artifacts/dataset_identity.json", {**access.summary, "internal_identity": DATASET_ID,
                   "manifest_file_sha256": MANIFEST_HASH, "split_file_sha256": SPLIT_HASH})
        selected, selection, views = prepare_selection(access, Path(config["coverage_ledger"]), output)
        manifest.update(selection_identity=selection["identity"], train_count=selection["train_count"],
                        observation_count=selection["observation_count"], future_candidates_read=len(access.future_blobs))
        write_json(output / "artifacts/execution_manifest.json", manifest)
        fixture_root = output / "evidence/synthetic_geometry_fixtures"
        fixture_root.mkdir()
        for kind in ("straight", "left", "right", "slow", "missing_first"):
            fixture = np.zeros((30, 8), dtype=np.float64)
            fixture[:, 0] = np.arange(1, 31) * .1
            fixture[:, 7] = 1
            fixture[:, 4] = .03 if kind == "slow" else 1
            fixture[:, 1] = fixture[:, 0] * fixture[:, 4]
            if kind in ("left", "right"):
                fixture[:, 1] = np.sin(fixture[:, 0])
                fixture[:, 2] = (1 - np.cos(fixture[:, 0])) * (1 if kind == "left" else -1)
            if kind == "missing_first":
                fixture[0, 7] = 0
                fixture[0, :7] = np.nan
            converted = spatial_target(fixture)
            np.savez_compressed(fixture_root / (kind + ".npz"), future=fixture, xy=converted["xy"],
                mask=converted["mask"], grid_m=GRID)
        data = np.load(output / "evidence/spatial_targets.npz", allow_pickle=False)
        target, mask = data["xy"], data["mask"]
        keys = epoch_keys(access.rows)
        from aic_transfuser_lite.models.temporal.gru import select_epoch_history
        for item in selected:
            for idx in select_epoch_history(keys, anchor_index=item["row_index"], length=4).indices:
                access.allowed_sensor_paths.update(access.rows[idx][k] for k in ("image_path", "lidar_path", "lidar_valid_path"))
        batches, histories = [], []
        cache: dict[str, bytes] = {}
        def read_sensor(path: str) -> bytes:
            if path not in cache:
                cache[path] = access.read(path)
            return cache[path]
        for item in selected:
            batch, history = build_inputs(access.rows, item["row_index"], read_sensor, keys=keys)
            batches.append(batch)
            histories.append(history)
        from aic_transfuser_lite.data.dataset_view_v3 import _LazyTemporalTrainingBatchesV3
        legacy = _LazyTemporalTrainingBatchesV3(root=root, rows=access.rows, epoch_keys=keys,
            row_index_by_epoch_stamp={(r["run_id"], r["segment_id"], int(r["grid_stamp_ns"])): i for i, r in enumerate(access.rows)},
            usable_anchors=[c["row_index"] for c in selected[:2]], behavior_by_sample=None,
            image_height=224, image_width=384, lidar_points=750, lidar_min_range_m=0, lidar_max_range_m=25,
            ego_features=FEATURES, ego_abs_limits=EGO_LIMITS, trajectory_steps=15, control_sequence_steps=10,
            camera_history_length=4, ego_history_length=10, command_history_length=10, control_target_bounds=BOUNDS,
            batch_size=1, max_batches=None, base_exclusion_counts={}, motion_target_candidate_count=0,
            motion_target_observed_count=0, motion_target_rejected_count=0, motion_target_censored_count=0,
            motion_target_censored_stationary_prefix_count=0)
        for i in range(min(2, len(batches))):
            old = legacy[i]
            for field in INPUT_FIELDS:
                torch.testing.assert_close(getattr(old, field), getattr(batches[i], field), rtol=0, atol=0)
        write_json(output / "artifacts/input_parity.json", {"status": "PASS", "sample_ids": [c["sample_id"] for c in selected[:2]],
            "fields": INPUT_FIELDS, "rtol": 0, "atol": 0,
            "scope": "two selected recorded anchors vs V3 lazy materializer, same epoch keys; not full runtime parity"})
        example_root = output / "evidence/input_examples"
        example_root.mkdir()
        for i in range(min(2, len(batches))):
            np.savez_compressed(example_root / f"input_{i}.npz", **{k: getattr(batches[i], k).numpy() for k in INPUT_FIELDS})
        write_json(example_root / "provenance.json", {"representatives": histories[:2], "all_selected_histories": histories,
            "asset_hashes": access.read_hashes, "original_sensor_examples": "NOT_INCLUDED; tensor replay possible, independent original preprocessing limited"})
        cache.clear()
        train_ids = [i for i, c in enumerate(selected) if c["role"] == "train"]
        shapes = {selected[i]["shape"] for i in train_ids}
        degenerate = len(train_ids) < 8 or len(shapes) < 2
        write_json(output / "artifacts/diagnostic_scope.json", {"degenerate": degenerate, "shape_classes": sorted(shapes),
                   "main_step_limit": 1 if degenerate else 500, "selection_frozen": True})
        torch.set_num_threads(4)
        torch.manual_seed(42)
        np.random.seed(42)
        random.seed(42)
        torch.backends.cuda.matmul.allow_tf32 = False
        torch.backends.cudnn.allow_tf32 = False
        torch.backends.cudnn.benchmark = False
        model = SpatialPathDiagnosticV4(image_height=224, image_width=384, lidar_points=750, ego_dim=4).float()
        write_json(output / "artifacts/initialization_map.json", model.initialize_representation())
        model = model.cuda()
        inventory = [checkpoint(output / "checkpoints/initial.pt", model)]
        write_json(output / "provenance/checkpoint_inventory.json", inventory)
        started = time.monotonic()
        manifest["active_start_utc"] = utc()
        def check_time() -> None:
            if time.monotonic()-started > 1800:
                raise TimeoutError("diagnostic active budget exhausted")
        def evaluate(current: SpatialPathDiagnosticV4, intervention: bool = False) -> np.ndarray:
            current.eval()
            predictions = []
            with torch.no_grad():
                for index in range(0, len(batches), 2):
                    check_time()
                    ids = list(range(index, min(index+2, len(batches))))
                    batch = combine_inputs([batches[i] for i in ids], "cuda")
                    if intervention:
                        from dataclasses import replace
                        other = combine_inputs([batches[(i+1) % len(batches)] for i in ids], "cuda")
                        batch = replace(batch, image=other.image, image_mask=other.image_mask, lidar=other.lidar, lidar_mask=other.lidar_mask)
                    predictions.append(current(batch).cpu().numpy())
            result = np.concatenate(predictions)
            if not np.isfinite(result).all():
                raise ValueError("nonfinite prediction")
            return result
        initial = evaluate(model)
        np.savez_compressed(output / "evidence/predictions_initial.npz", sample_ids=data["sample_ids"], xy=initial)
        initial_metrics = metrics(initial, target, mask, selected, views)
        write_json(output / "artifacts/metrics_initial.json", initial_metrics)
        template = np.zeros((20, 2), dtype=np.float32)
        template_support = []
        for k in range(20):
            ids = [i for i in train_ids if mask[i, k]]
            template_support.append(len(ids))
            if ids:
                template[k] = target[ids, k].mean(axis=0)
        baselines = {"zero": np.zeros_like(target), "mean_train": np.broadcast_to(template, target.shape).copy(),
                     "straight": np.broadcast_to(np.stack((GRID, np.zeros(20)), axis=-1), target.shape).copy()}
        if any(mask[:, k].any() and not template_support[k] for k in range(20)):
            raise ValueError("mean baseline undefined on a scored distance; no fabricated template allowed")
        write_json(output / "artifacts/baseline_metrics.json", {"template_support": template_support,
            "mean_template_note": "Zero placeholder where no train teacher exists; no such point may be scored",
            "results": {name: metrics(p, target, mask, selected, views) for name, p in baselines.items()}})
        np.savez_compressed(output / "evidence/baseline_predictions.npz", **baselines)
        rng = random.Random(42)
        def update(current: SpatialPathDiagnosticV4, optimizer: torch.optim.Optimizer, ids: list[int]) -> dict:
            current.train()
            optimizer.zero_grad(set_to_none=True)
            losses, valid_anchors = [], sum(bool(mask[i].any()) for i in ids)
            if not valid_anchors:
                return {"skipped": True}
            for offset in range(0, len(ids), 2):
                check_time()
                part = ids[offset:offset+2]
                prediction = current(combine_inputs([batches[i] for i in part], "cuda"))
                loss = masked_xy_loss(prediction, torch.from_numpy(target[part]).cuda(), torch.from_numpy(mask[part]).cuda())
                if loss is None:
                    continue
                count = sum(bool(mask[i].any()) for i in part)
                (loss * (count / valid_anchors)).backward()
                losses.append(float(loss.detach()))
            norm = torch.nn.utils.clip_grad_norm_(current.parameters(), 1.0, error_if_nonfinite=True)
            if not all(p.grad is None or torch.isfinite(p.grad).all() for p in current.parameters()):
                raise ValueError("nonfinite gradient")
            check_time()
            optimizer.step()
            if not all(torch.isfinite(p).all() for p in current.parameters()):
                raise ValueError("nonfinite parameter")
            return {"skipped": False, "loss": float(np.mean(losses)), "gradient_norm_before_clip": float(norm)}
        smoke = deepcopy(model)
        smoke_optimizer = optimizer_for(smoke)
        smoke_ids = [train_ids[i % len(train_ids)] for i in range(8)]
        smoke_result = update(smoke, smoke_optimizer, smoke_ids)
        manifest["smoke_steps"] = int(not smoke_result["skipped"])
        write_json(output / "artifacts/smoke.json", {**smoke_result, "weights_inherited_by_main": False})
        del smoke, smoke_optimizer
        torch.cuda.empty_cache()
        torch.manual_seed(42)
        optimizer = optimizer_for(model)
        write_json(output / "artifacts/optimizer.json", {"kind": "fresh AdamW", "weight_decay": 0,
            "parameter_names": [n for n, p in model.named_parameters() if p.requires_grad],
            "group_lrs": [1e-4, 1e-3], "old_losses_called": []})
        torch.cuda.reset_peak_memory_stats()
        order = train_ids.copy()
        cursor = len(order)
        step_limit = 1 if degenerate else 500
        time_stopped = False
        with (output / "logs/metrics.jsonl").open("x", encoding="utf-8") as log:
            for step in range(1, step_limit+1):
                if time.monotonic()-started >= 1740:
                    time_stopped = True
                    break
                ids = []
                for _ in range(8):
                    if cursor >= len(order):
                        rng.shuffle(order)
                        cursor = 0
                    ids.append(order[cursor]); cursor += 1
                record = update(model, optimizer, ids)
                if record["skipped"]:
                    manifest["optimizer_skips"] += 1
                    continue
                manifest["steps"] += 1
                manifest["overfit_executed"] = True
                record.update(step=manifest["steps"], active_seconds=time.monotonic()-started, lrs=[1e-4, 1e-3],
                              gpu_peak_memory_bytes=torch.cuda.max_memory_allocated())
                if step % 50 == 0:
                    record["evaluation"] = metrics(evaluate(model), target, mask, selected, views)["train"]
                log.write(json.dumps(clean(record), allow_nan=False) + "\n"); log.flush()
                if step == 1 or step % 25 == 0:
                    print(json.dumps(clean(record)), flush=True)
        final = evaluate(model)
        np.savez_compressed(output / "evidence/predictions_final.npz", sample_ids=data["sample_ids"], xy=final)
        final_metrics = metrics(final, target, mask, selected, views)
        write_json(output / "artifacts/aggregate_metrics.json", {"initial": initial_metrics, "final": final_metrics})
        with (output / "artifacts/per_anchor_metrics.csv").open("x", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=["stage", *final_metrics["rows"][0]])
            writer.writeheader()
            writer.writerows({"stage": stage, **r} for stage, report in (("initial", initial_metrics), ("final", final_metrics)) for r in report["rows"])
        manifest["intervention"] = "NOT_EXECUTED_TIME_BUDGET"
        if time.monotonic()-started < 1740:
            intervened = evaluate(model, intervention=True)
            write_json(output / "artifacts/input_intervention.json", {"policy": "cyclic next selected anchor Camera/LiDAR only, once",
                "mean_output_change_m": float(np.linalg.norm(intervened-final, axis=-1).mean()),
                "metrics": metrics(intervened, target, mask, selected, views)})
            manifest["intervention"] = "EXECUTED_ONCE"
        check_time()
        manifest["active_seconds"] = time.monotonic()-started
        inventory.append(checkpoint(output / "checkpoints/final.pt", model))
        write_json(output / "provenance/checkpoint_inventory.json", inventory)
        before, after = initial_metrics["train"]["ade_m"], final_metrics["train"]["ade_m"]
        comparable = before is not None and before > .02
        manifest.update(status="PARTIAL_TIME_LIMIT" if time_stopped else "DIAGNOSTIC_COMPLETE", degenerate=degenerate,
            initial_ade_m=before, final_ade_m=after, relative_improvement=1-after/before if comparable else None,
            fit_target_met=bool(not degenerate and comparable and after <= .02 and after <= .2*before),
            final_checkpoint_policy="LAST_STEP_NOT_BEST", gpu_peak_memory_bytes=torch.cuda.max_memory_allocated())
        draw_examples(output, selected, views, target, mask, initial, final, final_metrics)
        write_json(output / "provenance/selected_asset_inventory.json", access.read_hashes)
        # Verify only the metadata and selected assets already read; never expand scope.
        check_hash(root / "manifest.yaml", MANIFEST_HASH)
        check_hash(Path(config["split_manifest"]), SPLIT_HASH)
        for relative, expected in access.read_hashes.items():
            if sha((root / relative).read_bytes()) != expected:
                raise ValueError("input changed after diagnostic")
        manifest["selected_inputs_unchanged"] = True
        return_code = 0
    except Exception as error:
        manifest.update(status="FAILED_OR_BLOCKED", error_type=type(error).__name__, error=str(error))
        traceback.print_exc()
        return_code = 1
    manifest["end_utc"] = utc()
    write_json(output / "artifacts/execution_manifest.json", manifest)
    print(json.dumps(clean(manifest), ensure_ascii=False), flush=True)
    return return_code


def draw_examples(output: Path, selected: list[dict], views: list[dict], target: np.ndarray, mask: np.ndarray,
                  initial: np.ndarray, final: np.ndarray, report: dict) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    chosen = []
    for field, value in (("normal_recovery", "normal"), ("normal_recovery", "recovery"), ("shape", "left"),
                         ("shape", "right"), ("role", "observation_only")):
        ids = [i for i, c in enumerate(selected) if c[field] == value and i not in chosen]
        if ids:
            chosen.append(ids[0])
    worst = sorted(range(len(selected)), key=lambda i: report["rows"][i]["ade_m"] or -1, reverse=True)
    chosen += [i for i in worst if i not in chosen][:8-len(chosen)]
    for i in chosen:
        fig, ax = plt.subplots(figsize=(6, 6))
        ax.plot(views[i]["raw_prefix_xy"][:, 0], views[i]["raw_prefix_xy"][:, 1], color="gray", label="observed prefix")
        ax.plot(target[i, mask[i], 0], target[i, mask[i], 1], "go-", label="teacher support")
        ax.plot(initial[i, :, 0], initial[i, :, 1], "b--", label="initial (tail unknown)")
        ax.plot(final[i, :, 0], final[i, :, 1], "r--", label="final (tail unknown)")
        ax.scatter([0], [0], color="black", label="origin, not scored")
        ax.set(xlabel="X forward [m]", ylabel="Y left [m]", title=f"{selected[i]['sample_id']}\nsupport={mask[i].sum()} grid points; OBSERVED DIAGNOSTIC ONLY")
        ax.axis("equal"); ax.grid(); ax.legend(fontsize=7)
        fig.tight_layout(); fig.savefig(output / "figures" / f"example_{i:02d}.png", dpi=110); plt.close(fig)
