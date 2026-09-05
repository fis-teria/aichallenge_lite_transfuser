"""Fixed-checkpoint, bounded read-only replay/validation. No optimizer or trainer import."""
from __future__ import annotations

from collections import defaultdict
import csv
from datetime import datetime, timezone
import hashlib
import io
import json
import math
from pathlib import Path, PurePosixPath
import random
import shutil
import subprocess
import sys
import time
import traceback

import numpy as np
import torch
import yaml

from aic_transfuser_lite.data.dataset_view_v3 import _ego_row
from aic_transfuser_lite.data.spatial_diagnostic_inputs_v4 import INPUT_CONTRACT, INPUT_FIELDS, FEATURES, EGO_LIMITS, build_inputs, combine_inputs, epoch_keys
from aic_transfuser_lite.data.spatial_diagnostic_view_v4 import GRID, TEACHER_CONTRACT, spatial_target, polyline_length, self_intersects
from aic_transfuser_lite.models.spatial_path_diagnostic_v4 import SpatialPathDiagnosticV4, tensor_hash
from aic_transfuser_lite.models.temporal.gru import select_epoch_history

DATASET_ID = "181cf909b80589110574859990b0885005b7f9a0bb07cff1c24f38d6b090f388"
MANIFEST_SHA = "d625f42ca05a18ea76952376c6392268191c4895d6e605e0c49ceaa66dcbe1de"
SPLIT_SHA = "7d0e433dbd032ad695227051573e7d8d17072fa4ea3b4e28f4c44f56fde27b4f"
CHECKPOINT_SHA = "0316692543a901d9d6b96718c5651d6739367aa851f83fb3a133c126ccc8919f"
INPUT_ID = "77fff3f9c5875b6f116cebe35fde88d42d459aa04c195676c44e5c6caca5edc7"
TEACHER_ID = "cdb668834d4baf60c31fa5f934d01d8782f02544bc6fa29053d5a850432cc344"
SELECTION_ID = "698aba59f8cb7779006a8b070b6b00b406e8c93922f5fade252d19205907096b"
PRIOR_COMMIT = "f33b197df9eb1e6ae2af70c9661d8b1d88ab4ef0"
LEDGER_SHA = "35781616e8faab5117b0d9da7c8560519c5f196383ea66ab1465d999f5645e35"
# From the previously verified review packet, not recalibrated using validation.
PRIOR_HASHES = {
    "artifacts/execution_manifest.json": "717ddca4ade62194fbc0081d272cb0711a10de308fc63fd32d152977f23cb611",
    "artifacts/resolved_config.yaml": "b889f3f712d3eace143e6d449fae0d5170a2b8a28f5494145160203d96e89068",
    "artifacts/selection.json": "0be0341568aca0b5f2cb086be22e2b4db3ed0c17f51b07930ee678f926c4169d",
    "evidence/baseline_predictions.npz": "5d9bc4c5ec174b8d5772193b7523a81ede6b8c8672149c13ee7f60e8f9ca5939",
    "evidence/predictions_final.npz": "4a1cdcbcbf06e1a37fd78dc5c9bbf8e2b3dfe3c420a686599dd2d7d0f11f6985",
    "evidence/spatial_targets.npz": "efde209c302c42d5b81a5d44908261c6351dbd2a19371a19d0b3570c411c17d7",
}
FIXED = {"seed": 42, "candidates_per_run": 256, "main_per_run": 32, "observation_per_run": 4,
         "minimum_gap_ns": 500_000_000, "active_seconds": 1800, "microbatch": 2,
         "rtol": 1e-5, "atol_m": 1e-6, "dtype": "float32", "device": "cuda", "optimizer_steps": 0}


def sha(blob: bytes) -> str:
    return hashlib.sha256(blob).hexdigest()


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
    return sha(json.dumps(clean(value), ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False).encode())


def write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(clean(value), indent=2, ensure_ascii=False, allow_nan=False)+"\n", encoding="utf-8")


def read_checked(path: Path, expected: str) -> bytes:
    if path.is_symlink():
        raise ValueError("symlink forbidden")
    blob = path.read_bytes()
    if sha(blob) != expected:
        raise ValueError("identity mismatch: " + path.name)
    return blob


def check_identity(document: dict, expected: str) -> None:
    if document.get("identity") != expected or digest({k: v for k, v in document.items() if k != "identity"}) != expected:
        raise ValueError("contract/selection identity mismatch")


def split_maps(assignments: list[dict], runs: list[dict]) -> dict[str, str]:
    mapping = {r["run_id"]: r["split"] for r in assignments}
    if len(mapping) != len(assignments) or set(mapping) != {r["run_id"] for r in runs}:
        raise ValueError("duplicate/missing split run")
    sources: dict[str, set] = defaultdict(set)
    for r in runs:
        if not r["source_hash"]:
            raise ValueError("missing source identity")
        sources[r["source_hash"]].add(mapping[r["run_id"]])
    if any(len(s) > 1 for s in sources.values()):
        raise ValueError("source spans splits")
    return mapping


class ValidationReadOnlyAccess:
    """Separate explicit adapter: old train-only CanonicalAccess remains untouched."""
    def __init__(self, root: Path, split: Path, replay_ids: list[str]):
        self.root = root
        manifest = yaml.load(read_checked(root / "manifest.yaml", MANIFEST_SHA), Loader=yaml.CSafeLoader)
        if manifest.get("manifest_sha256") != DATASET_ID or digest({k: v for k, v in manifest.items() if k != "manifest_sha256"}) != DATASET_ID:
            raise ValueError("Dataset internal identity mismatch")
        if tuple(manifest.get(k) for k in ("coordinate_frame", "distance_unit", "angle_unit", "time_unit")) != ("base_link@t_obs", "m", "rad", "s"):
            raise ValueError("frame/units mismatch")
        self.inventory = {f["path"]: f for f in manifest["files"]}
        if len(self.inventory) != len(manifest["files"]):
            raise ValueError("duplicate inventory")
        self.hashes: dict[str, str] = {}
        self.allowed_sensors: set[str] = set()
        self.allowed_futures: dict[str, str] = {}
        self.future_blobs: dict[str, bytes] = {}
        self.future_by_run: dict[str, set[str]] = defaultdict(set)
        self.mapping: dict[str, str] = {}
        rows = list(csv.DictReader(io.StringIO(self.read("samples.csv").decode())))
        runs = list(csv.DictReader(io.StringIO(self.read("runs.csv").decode())))
        splits = json.loads(read_checked(split, SPLIT_SHA))
        if splits["dataset_manifest_sha256"] != DATASET_ID:
            raise ValueError("split Dataset mismatch")
        self.mapping = split_maps(splits["assignments"], runs)
        self.runs = {r["run_id"]: r for r in runs}
        self.val_runs = sorted(r for r, s in self.mapping.items() if s == "validation")
        if len(self.val_runs) != 5:
            raise ValueError("expected exactly five validation runs")
        self.rows = {s: sorted([r for r in rows if self.mapping[r["run_id"]] == s],
                              key=lambda r: (r["run_id"], r["segment_id"], int(r["grid_stamp_ns"]))) for s in ("train", "validation")}
        if len({r["sample_id"] for r in rows}) != len(rows):
            raise ValueError("duplicate sample ID")
        self.index = {s: {r["sample_id"]: i for i, r in enumerate(rs)} for s, rs in self.rows.items()}
        if len(replay_ids) != 64 or len(set(replay_ids)) != 64 or any(i not in self.index["train"] for i in replay_ids):
            raise ValueError("fixed replay must be 64 unique train anchors")
        self.replay_ids = set(replay_ids)
        self.summary = {"all_metadata_anchors": len(rows), "all_runs": len(runs),
            "validation_metadata_anchors": len(self.rows["validation"]), "source_split_overlap": 0,
            "per_validation_run": {run: sum(r["run_id"] == run for r in self.rows["validation"]) for run in self.val_runs},
            "session_independence": "UNKNOWN"}

    def authorize_future(self, row: dict) -> None:
        split = self.mapping[row["run_id"]]
        if split == "test" or (split == "train" and row["sample_id"] not in self.replay_ids):
            raise ValueError("forbidden test/unselected train future")
        if split not in ("train", "validation"):
            raise ValueError("unexpected split")
        path = row["trajectory_path"]
        if PurePosixPath(path).parts[:2] != ("trajectories", row["run_id"]):
            raise ValueError("future locator/run mismatch")
        self.allowed_futures[path] = row["run_id"]

    def authorize_sensors(self, split: str, ids: list[str]) -> None:
        rows = self.rows[split]
        keys = epoch_keys(rows)
        for sample_id in ids:
            if split == "train" and sample_id not in self.replay_ids:
                raise ValueError("unselected train sensor")
            for i in select_epoch_history(keys, anchor_index=self.index[split][sample_id], length=4).indices:
                for field, category in (("image_path", "images"), ("lidar_path", "lidar"), ("lidar_valid_path", "lidar")):
                    path = rows[i][field]
                    if PurePosixPath(path).parts[:2] != (category, rows[i]["run_id"]):
                        raise ValueError("sensor locator/run mismatch")
                    self.allowed_sensors.add(path)

    def read(self, name: str) -> bytes:
        parts = PurePosixPath(name)
        if parts.is_absolute() or ".." in parts.parts or "\\" in name or name not in self.inventory:
            raise ValueError("invalid asset path")
        if name not in ("samples.csv", "runs.csv") and name not in self.allowed_futures and name not in self.allowed_sensors:
            raise ValueError("unselected/test asset read forbidden")
        if name in self.allowed_futures:
            run = self.allowed_futures[name]
            cap = 256 if self.mapping[run] == "validation" else 64
            if name not in self.future_by_run[run] and len(self.future_by_run[run]) >= cap:
                raise ValueError("candidate future budget exceeded")
        current = self.root
        for part in parts.parts:
            current /= part
            if current.is_symlink():
                raise ValueError("canonical symlink")
        blob = read_checked(current, self.inventory[name]["sha256"])
        if len(blob) != self.inventory[name]["size_bytes"]:
            raise ValueError("asset size mismatch")
        self.hashes[name] = sha(blob)
        if name in self.allowed_futures:
            self.future_by_run[self.allowed_futures[name]].add(name)
            self.future_blobs[name] = blob
        return blob


def eligibility(row: dict, future: np.ndarray, view: dict) -> tuple[list[str], bool]:
    """Exact f33b197 prepare_selection predicate, without new exclusion thresholds."""
    _, mask = _ego_row(row, FEATURES, abs_limits=EGO_LIMITS)
    reason = []
    if not bool(mask.all()):
        reason.append("invalid_current_ego")
    if not np.isfinite(future[future[:, 7] == 1, :7]).all():
        reason.append("nonfinite_valid_future")
    if abs(float(row["velocity_longitudinal_mps"])) <= .05:
        reason.append("stopped_observation")
    if view["processed_length_m"] is None or view["processed_length_m"] < .5:
        reason.append("short_or_unknown_support")
    reason.extend(view["flags"])
    if view["cut_reason"] in ("position_jump", "invalid_time_grid_or_gap", "nonfinite_valid"):
        reason.append(view["cut_reason"])
    return reason, bool(mask.all())


def candidate_indices(rows: list[dict]) -> list[int]:
    rng = random.Random(42)
    buckets: dict[str, list[int]] = defaultdict(list)
    for i, r in enumerate(rows):
        buckets[r["run_id"]].append(i)
    selected = []
    for run in sorted(buckets):
        rng.shuffle(buckets[run])
        selected.extend(list(reversed(buckets[run]))[:256])
    return selected


def select_validation(candidates: list[dict]) -> list[dict]:
    selected = []
    for run in sorted({c["run_id"] for c in candidates}):
        group = [c for c in candidates if c["run_id"] == run]
        buckets: dict[str, list] = defaultdict(list)
        for c in group:
            if not c["reasons"]:
                buckets[c["shape"]].append(c)
        main = []
        while len(main) < 32 and any(buckets.values()):
            for shape in sorted(buckets):
                if buckets[shape] and len(main) < 32:
                    item = buckets[shape].pop(0)
                    if all(abs(item["stamp_ns"] - o["stamp_ns"]) >= 500_000_000 for o in main):
                        main.append({**item, "role": "validation_main"})
        used = {c["sample_id"] for c in main}
        obs_buckets: dict[tuple, list] = defaultdict(list)
        for c in group:
            if c["sample_id"] not in used and c["current_ego_valid"]:
                key = (c["cut_reason"], c["reasons"][0] if c["reasons"] else "heldout_train_observation", c["normal_recovery"])
                obs_buckets[key].append(c)
        obs = []
        while len(obs) < 4 and any(obs_buckets.values()):
            for key in sorted(obs_buckets):
                if obs_buckets[key] and len(obs) < 4:
                    obs.append({**obs_buckets[key].pop(0), "role": "validation_observation"})
        selected.extend(main+obs)
    return selected


def state_inventory(model: torch.nn.Module) -> dict:
    parameters = dict(model.named_parameters())
    return {n: {"shape": list(t.shape), "dtype": str(t.dtype), "numel": t.numel(), "sha256": tensor_hash(t),
                "kind": "parameter" if n in parameters else "buffer"} for n, t in model.state_dict().items()}


def strict_restore(model: torch.nn.Module, blob: bytes) -> dict:
    payload = torch.load(io.BytesIO(blob), map_location="cpu", weights_only=True)
    if set(payload) != {"format", "state_dict"} or payload["format"] != "SPATIAL_DIAGNOSTIC_NOT_RUNTIME":
        raise ValueError("checkpoint format mismatch")
    expected, state = model.state_dict(), payload["state_dict"]
    if set(expected) != set(state):
        raise ValueError("checkpoint state key mismatch")
    for name, tensor in state.items():
        if not isinstance(tensor, torch.Tensor) or tensor.shape != expected[name].shape or tensor.dtype != expected[name].dtype:
            raise ValueError("checkpoint shape/dtype mismatch: " + name)
        if not torch.isfinite(tensor).all():
            raise ValueError("checkpoint nonfinite: " + name)
    model.load_state_dict(state, strict=True)
    model.eval().requires_grad_(False)
    return {"load": "weights_only=True, full strict restore", "loaded": sorted(state), "missing": [], "unexpected": [],
            "state": state_inventory(model)}


def compare_replay(actual: np.ndarray, expected: np.ndarray, ids: list[str]) -> dict:
    if actual.shape != expected.shape or actual.shape != (len(ids), 20, 2):
        raise ValueError("replay shape mismatch")
    delta = np.abs(actual-expected)
    close = np.isclose(actual, expected, rtol=1e-5, atol=1e-6) & np.isfinite(actual) & np.isfinite(expected)
    return {"passed": bool(close.all()), "rtol": 1e-5, "atol_m": 1e-6,
        "max_abs_difference_m": float(delta.max()), "mean_abs_difference_m": float(delta.mean()),
        "exceeded_ids": [sample_id for i, sample_id in enumerate(ids) if not close[i].all()],
        "per_anchor": [{"sample_id": sample_id, "max_abs_difference_m": float(delta[i].max()),
                        "mean_abs_difference_m": float(delta[i].mean())} for i, sample_id in enumerate(ids)]}


def train_template(xy: np.ndarray, mask: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    template = np.full((20, 2), np.nan, dtype=np.float32)
    support = mask.sum(axis=0)
    for k in range(20):
        if support[k]:
            template[k] = xy[mask[:, k], k].mean(axis=0)
    return template, support


def metric_rows(predictions: np.ndarray, targets: np.ndarray, mask: np.ndarray, selected: list[dict],
                views: list[dict], processed: np.ndarray, baseline: bool = False) -> list[dict]:
    """No silent denominator shrink: every selected anchor retained, unknown errors null."""
    if predictions.shape != targets.shape or targets.shape != (len(selected), 20, 2) or mask.shape != (len(selected), 20):
        raise ValueError("metric shape mismatch")
    result = []
    for i, item in enumerate(selected):
        valid, pred = mask[i], predictions[i]
        finite_all = bool(np.isfinite(pred).all())
        finite_support = bool(np.isfinite(pred[valid]).all())
        evaluable = bool(processed[i] and valid.any() and finite_support and (finite_all or baseline))
        error = np.linalg.norm(pred[valid] - targets[i, valid], axis=1) if evaluable else None
        result.append({**{k: item[k] for k in ("sample_id", "run_id", "role", "shape", "normal_recovery", "collection_slice")},
            "processed": bool(processed[i]), "valid_points": int(valid.sum()), "scored_points": int(valid.sum()) if evaluable else 0,
            "ade_m": float(error.mean()) if error is not None else None,
            "unknown_reason": None if evaluable else "NOT_PROCESSED" if not processed[i] else "NO_TEACHER_SUPPORT" if not valid.any() else "NONFINITE_OR_UNDEFINED_PREDICTION",
            "finite_all20": finite_all if processed[i] else None, "origin_concentrated_all20": bool(np.linalg.norm(pred, axis=1).max() < .02) if processed[i] and finite_all else None,
            "proper_crossing_all20": self_intersects(pred) if processed[i] and finite_all else None,
            "proper_crossing_teacher_prefix": self_intersects(pred[valid]) if processed[i] and finite_support and valid.any() else None,
            "raw_teacher_length_m": views[i]["raw_length_m"], "processed_teacher_length_m": views[i]["processed_length_m"],
            "resampled_teacher_length_m": views[i]["resampled_length_m"], "distance_status": views[i]["distance_status"],
            "predicted_teacher_prefix_length_m": polyline_length(pred[valid]) if evaluable else None,
            **{f"error_{d:.1f}m": float(np.linalg.norm(pred[k]-targets[i, k])) if processed[i] and valid[k] and np.isfinite(pred[k]).all() and (finite_all or baseline) else None
               for d, k in ((.5, 4), (1., 9), (1.5, 14), (2., 19))}})
    return result


def aggregate(rows: list[dict]) -> dict:
    def mean(key: str) -> float | None:
        values = [r[key] for r in rows if r[key] is not None]
        return float(np.mean(values)) if values else None
    scored = [r for r in rows if r["ade_m"] is not None]
    return {"anchors": len(rows), "processed": sum(r["processed"] for r in rows),
        "supported_anchors": sum(r["valid_points"] > 0 for r in rows), "unsupported_anchors": sum(r["valid_points"] == 0 for r in rows),
        "scored_anchors": len(scored), "valid_points": sum(r["valid_points"] for r in rows),
        "scored_points": sum(r["scored_points"] for r in rows), "ade_m": mean("ade_m"),
        "worst_anchor": max(scored, key=lambda r: r["ade_m"])["sample_id"] if scored else None,
        "proper_crossing_all20_count": sum(r["proper_crossing_all20"] is True for r in rows),
        "proper_crossing_prefix_count": sum(r["proper_crossing_teacher_prefix"] is True for r in rows),
        "nonfinite_all20_count": sum(r["finite_all20"] is False for r in rows),
        "origin_concentrated_count": sum(r["origin_concentrated_all20"] is True for r in rows),
        "distance_errors": {f"{d:.1f}m": {"error_m": mean(f"error_{d:.1f}m"), "anchors": sum(r[f"error_{d:.1f}m"] is not None for r in rows)} for d in (.5, 1., 1.5, 2.)}}


def grouped(rows: list[dict]) -> dict:
    result = {}
    for role in sorted({r["role"] for r in rows}):
        subset = [r for r in rows if r["role"] == role]
        by_run = {run: aggregate([r for r in subset if r["run_id"] == run]) for run in sorted({r["run_id"] for r in subset})}
        defined = [run for run, values in by_run.items() if values["ade_m"] is not None]
        result[role] = {"overall": aggregate(subset), "run_id": by_run,
            "worst_run": max(defined, key=lambda run: by_run[run]["ade_m"]) if defined else None,
            **{field: {label: aggregate([r for r in subset if r[field] == label]) for label in sorted({r[field] for r in subset})}
               for field in ("shape", "normal_recovery", "collection_slice")}}
    return result


def comparisons(model: list[dict], baseline: list[dict]) -> dict:
    paired = []
    for a, b in zip(model, baseline):
        if a["sample_id"] != b["sample_id"]:
            raise ValueError("paired ID mismatch")
        delta = a["ade_m"] - b["ade_m"] if a["ade_m"] is not None and b["ade_m"] is not None else None
        paired.append({**{k: a[k] for k in ("sample_id", "role", "run_id", "shape", "normal_recovery", "collection_slice")}, "model_minus_baseline_m": delta})
    def summary(items: list[dict]) -> dict:
        values = [r["model_minus_baseline_m"] for r in items if r["model_minus_baseline_m"] is not None]
        return {"anchors": len(items), "paired_anchors": len(values), "unknown_anchors": len(items)-len(values),
            "mean_model_minus_baseline_m": float(np.mean(values)) if values else None,
            "model_wins": sum(v < 0 for v in values), "ties": sum(v == 0 for v in values), "model_losses": sum(v > 0 for v in values)}
    groups = {}
    for role in sorted({r["role"] for r in paired}):
        subset = [r for r in paired if r["role"] == role]
        groups[role] = {"overall": summary(subset), **{field: {label: summary([r for r in subset if r[field] == label]) for label in sorted({r[field] for r in subset})}
            for field in ("run_id", "shape", "normal_recovery", "collection_slice")}}
    return {"rows": paired, "groups": groups}


class Budget:
    def __init__(self, seconds: float = 1800):
        self.start, self.seconds = time.monotonic(), seconds

    def check(self) -> None:
        if time.monotonic() - self.start >= self.seconds:
            raise TimeoutError("fixed evaluation budget exhausted")


def infer(model: torch.nn.Module, batches: list, predicted: np.ndarray, processed: np.ndarray, budget: Budget) -> None:
    if model.training:
        raise ValueError("model must be eval before inference")
    with torch.inference_mode():
        for i in range(0, len(batches), 2):
            budget.check()
            output = model(combine_inputs(batches[i:i+2], next(model.parameters()).device)).cpu().numpy()
            predicted[i:i+len(output)] = output
            processed[i:i+len(output)] = True
            if output.shape != (len(batches[i:i+2]), 20, 2) or not np.isfinite(output).all():
                raise ValueError("nonfinite/invalid model prediction; remaining anchors unprocessed")
            budget.check()


def execute(config_path: Path, output: Path) -> int:
    if output.exists():
        raise FileExistsError("immutable evaluation output exists")
    config = yaml.safe_load(config_path.read_text())
    if config["evaluation"] != FIXED:
        raise ValueError("fixed evaluation config mismatch")
    prior, root = Path(config["prior_run"]), Path(config["dataset_root"])
    if any(p.resolve() == output.resolve() or p.resolve() in output.resolve().parents for p in (prior, root)):
        raise ValueError("output overlaps protected inputs")
    output.mkdir(parents=True)
    for directory in ("artifacts", "evidence", "logs", "provenance", "figures"):
        (output / directory).mkdir()
    repo = Path(__file__).resolve().parents[3]
    command = lambda *args: subprocess.check_output(["git", *args], cwd=repo, text=True).strip()
    manifest = {"mode": "FIXED_STEP500_LIMITED_VALIDATION", "execution_commit": command("rev-parse", "HEAD"),
        "dirty": command("status", "--porcelain"), "start_utc": datetime.now(timezone.utc).isoformat(), "status": "STARTED",
        "teacher_adoption_approved": False, "runtime_deployment_approved": False, "new_training_optimizer_steps": 0,
        "optimizer_constructed": False, "raw_reads_performed": 0, "test_asset_reads": 0, "train_replay_passed": False}
    write_json(output / "artifacts/execution_manifest.json", manifest)
    access = None
    model = None
    state_before = None
    val_selected: list[dict] = []
    val_views: list[dict] = []
    val_xy = val_mask = val_pred = val_done = None
    train_selected: list[dict] = []
    train_views: list[dict] = []
    train_xy = train_mask = train_pred = train_done = None
    template = None
    budget = Budget()
    try:
        if manifest["dirty"]:
            raise ValueError("dirty execution checkout")
        manifest["code_hashes"] = {name: sha((repo / name).read_bytes()) for name in command("ls-files", "src", "tools/evaluate_spatial_diagnostic_validation_v4.py", "configs/spatial_diagnostic_validation_v4.yaml").splitlines()}
        (output / "artifacts/resolved_config.yaml").write_text(yaml.safe_dump(config, sort_keys=True))
        prior_blobs = {name: read_checked(prior / name, expected) for name, expected in PRIOR_HASHES.items()}
        write_json(output / "provenance/prior_asset_hashes.json", PRIOR_HASHES)
        old_execution = json.loads(prior_blobs["artifacts/execution_manifest.json"])
        if old_execution["execution_commit"] != PRIOR_COMMIT or old_execution["steps"] != 500 or old_execution["final_checkpoint_policy"] != "LAST_STEP_NOT_BEST":
            raise ValueError("prior final selection policy mismatch")
        for name, expected in old_execution["code_hashes"].items():
            # Existing source semantics must remain byte-identical, including teacher/model/preprocessing.
            if sha((repo / name).read_bytes()) != expected:
                raise ValueError("prior source changed: " + name)
        for key, contract, expected in (("input", INPUT_CONTRACT, INPUT_ID), ("teacher", TEACHER_CONTRACT, TEACHER_ID)):
            old = json.loads((prior / f"artifacts/{key}_contract.json").read_text())
            check_identity(old, expected)
            if digest(contract) != expected:
                raise ValueError("current contract changed")
            write_json(output / f"artifacts/{key}_contract.json", old)
        old_config = yaml.safe_load(prior_blobs["artifacts/resolved_config.yaml"])
        if old_config != yaml.safe_load((repo / "configs/spatial_diagnostic_v4.yaml").read_text()):
            raise ValueError("prior resolved config mismatch")
        if old_config["dataset_root"] != str(root) or old_config["split_manifest"] != config["split_manifest"]:
            raise ValueError("fixed Dataset locator mismatch")
        old_selection = json.loads(prior_blobs["artifacts/selection.json"])
        check_identity(old_selection, SELECTION_ID)
        old_indices = [i for i, c in enumerate(old_selection["selected"]) if c["role"] == "train"]
        train_selected = [{**old_selection["selected"][i], "role": "train_replay"} for i in old_indices]
        train_ids = [c["sample_id"] for c in train_selected]
        write_json(output / "artifacts/train_selection.json", {"prior_identity": SELECTION_ID, "selected": train_selected})
        access = ValidationReadOnlyAccess(root, Path(config["split_manifest"]), train_ids)
        write_json(output / "artifacts/dataset_identity.json", {**access.summary, "dataset_identity": DATASET_ID,
            "manifest_sha256": MANIFEST_SHA, "split_sha256": SPLIT_SHA})
        old_target = np.load(io.BytesIO(prior_blobs["evidence/spatial_targets.npz"]), allow_pickle=False)
        old_prediction = np.load(io.BytesIO(prior_blobs["evidence/predictions_final.npz"]), allow_pickle=False)
        old_baselines = np.load(io.BytesIO(prior_blobs["evidence/baseline_predictions.npz"]), allow_pickle=False)
        np.testing.assert_array_equal(old_target["sample_ids"][old_indices], train_ids)
        np.testing.assert_array_equal(old_prediction["sample_ids"][old_indices], train_ids)
        train_xy, train_mask = old_target["xy"][old_indices], old_target["mask"][old_indices]
        template, template_support = train_template(train_xy, train_mask)
        np.testing.assert_array_equal(template, old_baselines["mean_train"][0])
        np.savez_compressed(output / "evidence/train_targets.npz", sample_ids=np.array(train_ids), xy=train_xy, mask=train_mask, grid_m=GRID)
        np.savez_compressed(output / "evidence/prior_train_predictions.npz", sample_ids=np.array(train_ids), xy=old_prediction["xy"][old_indices])
        np.savez_compressed(output / "evidence/train_template.npz", xy=template, support=template_support,
                            prior_baseline=old_baselines["mean_train"][old_indices])
        metadata = []
        for item in train_selected:
            budget.check()
            row = access.rows["train"][access.index["train"][item["sample_id"]]]
            access.authorize_future(row)
            blob = access.read(row["trajectory_path"])
            view = spatial_target(np.load(io.BytesIO(blob), allow_pickle=False))
            np.testing.assert_array_equal(view["xy"], train_xy[len(train_views)])
            np.testing.assert_array_equal(view["mask"], train_mask[len(train_views)])
            train_views.append(view)
            path = output / "evidence/train_futures" / (item["sample_id"] + ".npy")
            path.parent.mkdir(exist_ok=True)
            path.write_bytes(blob)
            metadata.append({"canonical": row, "future_sha256": sha(blob)})
        write_json(output / "evidence/train_sample_metadata.json", metadata)
        torch.set_num_threads(4)
        torch.manual_seed(42)
        torch.backends.cuda.matmul.allow_tf32 = False
        torch.backends.cudnn.allow_tf32 = False
        torch.backends.cudnn.benchmark = False
        if not torch.cuda.is_available():
            raise ValueError("BLOCKED_NO_CUDA; no CPU fallback")
        write_json(output / "provenance/environment.json", {"python": sys.version, "torch": torch.__version__, "numpy": np.__version__,
            "cuda": torch.version.cuda, "gpu": torch.cuda.get_device_name(0), "free_bytes": shutil.disk_usage(output).free, "argv": sys.argv})
        if Path(config["checkpoint"]) != prior / "checkpoints/final.pt":
            raise ValueError("only the explicitly fixed final.pt is allowed")
        checkpoint_blob = read_checked(Path(config["checkpoint"]), CHECKPOINT_SHA)
        model = SpatialPathDiagnosticV4(image_height=224, image_width=384, lidar_points=750, ego_dim=4).float()
        load_map = strict_restore(model, checkpoint_blob)
        model = model.cuda()
        torch.cuda.reset_peak_memory_stats()
        state_before = state_inventory(model)
        write_json(output / "provenance/checkpoint_load_map.json", {**load_map, "checkpoint_sha256": CHECKPOINT_SHA})
        write_json(output / "provenance/state_before.json", state_before)
        del checkpoint_blob
        def inputs(split: str, selected: list[dict], save_examples: bool = False) -> list:
            access.authorize_sensors(split, [c["sample_id"] for c in selected])
            rows, batches, histories, cache = access.rows[split], [], [], {}
            keys = epoch_keys(rows)
            def read_sensor(name: str) -> bytes:
                if name not in cache:
                    cache[name] = access.read(name)
                return cache[name]
            for i, item in enumerate(selected):
                budget.check()
                batch, history = build_inputs(rows, access.index[split][item["sample_id"]], read_sensor, keys=keys)
                batches.append(batch)
                histories.append(history)
                if save_examples and i < 2:
                    folder = output / "evidence/input_examples"
                    folder.mkdir(exist_ok=True)
                    np.savez_compressed(folder / f"input_{i}.npz", **{k: getattr(batch, k).numpy() for k in INPUT_FIELDS})
            write_json(output / f"evidence/{split}_histories.json", histories)
            return batches
        train_pred, train_done = np.full((64, 20, 2), np.nan, dtype=np.float32), np.zeros(64, dtype=bool)
        batches = inputs("train", train_selected)
        infer(model, batches, train_pred, train_done, budget)
        del batches
        replay = compare_replay(train_pred, old_prediction["xy"][old_indices], train_ids)
        write_json(output / "artifacts/train_replay_comparison.json", replay)
        np.savez_compressed(output / "evidence/train_replay_differences.npz", sample_ids=np.array(train_ids),
                            delta_xy=train_pred-old_prediction["xy"][old_indices])
        if not replay["passed"]:
            raise ValueError("BLOCKED_TRAIN_REPLAY; validation inference not started")
        manifest["train_replay_passed"] = True
        print("TRAIN_REPLAY_PASS max_difference_m=" + str(replay["max_abs_difference_m"]), flush=True)
        write_json(output / "artifacts/execution_manifest.json", manifest)
        # Metadata annotations only: no following source_uri, no raw or S1 access.
        annotation_blob = read_checked(Path(config["annotation_ledger"]), LEDGER_SHA)
        annotations = {r["sample_id"]: r for r in csv.DictReader(io.StringIO(annotation_blob.decode())) if r["split"] == "validation"}
        candidates, candidate_views = [], {}
        for index in candidate_indices(access.rows["validation"]):
            budget.check()
            row = access.rows["validation"][index]
            access.authorize_future(row)
            future = np.load(io.BytesIO(access.read(row["trajectory_path"])), allow_pickle=False)
            view = spatial_target(future)
            reasons, current_valid = eligibility(row, future, view)
            annotation = annotations.get(row["sample_id"], {})
            candidates.append({"sample_id": row["sample_id"], "run_id": row["run_id"], "split": "validation", "row_index": index,
                "segment_id": row["segment_id"], "stamp_ns": int(row["grid_stamp_ns"]), "shape": view["shape"],
                "shape_provenance": "future geometry diagnostic, NOT route intent", "reasons": reasons,
                "current_ego_valid": current_valid, "cut_reason": view["cut_reason"], "path_points": int(view["mask"].sum()),
                "collection_slice": annotation.get("collection_case", "UNKNOWN"), "normal_recovery": annotation.get("normal_recovery", "UNKNOWN"),
                "source_hash": access.runs[row["run_id"]]["source_hash"], "source_quality": "UNKNOWN_NOT_RAW_REVERIFIED",
                "quality_flags": annotation.get("quality_flags", "UNKNOWN"), "existing_filter_flags": annotation.get("v3_exclusion_flags", "UNKNOWN"),
                "distance_status": view["distance_status"], "raw_length_m": view["raw_length_m"], "processed_length_m": view["processed_length_m"]})
            candidate_views[row["sample_id"]] = view
            # Progress survives budget/failure without inventing a completed selection.
            write_json(output / "artifacts/candidate_progress.json", candidates)
        val_selected = select_validation(candidates)
        role_by_id = {c["sample_id"]: c["role"] for c in val_selected}
        for c in candidates:
            c["role"] = role_by_id.get(c["sample_id"], "not_selected")
            c["selection_reason"] = "shape round robin, gap >=0.5s" if c["role"] == "validation_main" else "fixed observation bucket" if c["role"] == "validation_observation" else "ineligible" if c["reasons"] else "quota_or_gap"
        selection = {"seed": 42, "selected": val_selected, "candidates": candidates,
            "denominators": {run: {"metadata": access.summary["per_validation_run"][run],
                "candidates": sum(c["run_id"] == run for c in candidates), "main_eligible": sum(c["run_id"] == run and not c["reasons"] for c in candidates),
                "main_selected": sum(c["run_id"] == run and c["role"] == "validation_main" for c in val_selected),
                "observation_selected": sum(c["run_id"] == run and c["role"] == "validation_observation" for c in val_selected)} for run in access.val_runs}}
        selection["identity"] = digest(selection)
        write_json(output / "artifacts/selection.json", selection)
        manifest["validation_selection_identity"] = selection["identity"]
        if not val_selected:
            raise ValueError("BLOCKED_NO_VALIDATION_SELECTION")
        val_views = [candidate_views[c["sample_id"]] for c in val_selected]
        val_xy, val_mask = np.stack([v["xy"] for v in val_views]), np.stack([v["mask"] for v in val_views])
        metadata = []
        for c, v in zip(val_selected, val_views):
            row = access.rows["validation"][c["row_index"]]
            blob = access.future_blobs[row["trajectory_path"]]
            dest = output / "evidence/validation_futures" / (c["sample_id"] + ".npy")
            dest.parent.mkdir(exist_ok=True)
            dest.write_bytes(blob)
            metadata.append({"canonical": row, "future_sha256": sha(blob), "geometry": {k: value for k, value in v.items() if k not in ("xy", "mask", "grid_m", "raw_prefix_xy")}})
        write_json(output / "evidence/validation_sample_metadata.json", metadata)
        np.savez_compressed(output / "evidence/validation_targets.npz", sample_ids=np.array([c["sample_id"] for c in val_selected]), xy=val_xy, mask=val_mask, grid_m=GRID)
        val_pred, val_done = np.full(val_xy.shape, np.nan, dtype=np.float32), np.zeros(len(val_selected), dtype=bool)
        batches = inputs("validation", val_selected, save_examples=True)
        infer(model, batches, val_pred, val_done, budget)
        del batches
        manifest["status"] = "EVALUATION_COMPLETE"
    except TimeoutError as error:
        manifest.update(status="PARTIAL", error=str(error))
        traceback.print_exc()
    except Exception as error:
        manifest.update(status="BLOCKED", error_type=type(error).__name__, error=str(error))
        traceback.print_exc()
    finally:
        manifest["active_seconds"] = time.monotonic() - budget.start
        if model is not None and torch.cuda.is_available():
            manifest["gpu_peak_allocated_bytes"] = torch.cuda.max_memory_allocated()
        if model is not None and state_before is not None:
            after = state_inventory(model)
            write_json(output / "provenance/state_after.json", after)
            manifest["parameter_buffer_unchanged"] = after == state_before
            if after != state_before:
                manifest.update(status="BLOCKED", error="parameter/buffer mutated")
        all_metrics = {}
        for name, selected, views, xy, mask, prediction, done in (("train", train_selected, train_views, train_xy, train_mask, train_pred, train_done),
                ("validation", val_selected, val_views, val_xy, val_mask, val_pred, val_done)):
            if prediction is None:
                continue
            np.savez_compressed(output / f"evidence/{name}_predictions.npz", sample_ids=np.array([c["sample_id"] for c in selected]), xy=prediction, processed=done)
            if len(views) != len(selected):
                continue
            base = {"zero": np.zeros_like(xy), "straight": np.broadcast_to(np.stack((GRID, np.zeros(20)), axis=-1), xy.shape).copy(),
                    "mean_train": np.broadcast_to(template, xy.shape).copy()}
            np.savez_compressed(output / f"evidence/{name}_baselines.npz", **base)
            model_rows = metric_rows(prediction, xy, mask, selected, views, done)
            rows = {"model": model_rows, **{key: metric_rows(p, xy, mask, selected, views, np.ones(len(selected), dtype=bool), baseline=True) for key, p in base.items()}}
            all_metrics[name] = {"predictions": {key: {"rows": rs, "groups": grouped(rs)} for key, rs in rows.items()},
                "paired_comparisons": {key: comparisons(model_rows, rows[key]) for key in base}}
            with (output / f"artifacts/{name}_per_anchor_metrics.csv").open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=["predictor", *model_rows[0]])
                writer.writeheader()
                for key, rs in rows.items():
                    writer.writerows({"predictor": key, **r} for r in rs)
        write_json(output / "artifacts/metrics.json", all_metrics)
        if manifest["status"] == "EVALUATION_COMPLETE" and val_pred is not None:
            draw_figures(output, val_selected, val_views, val_xy, val_mask, val_pred, all_metrics["validation"]["predictions"]["model"]["rows"])
        if access is not None:
            write_json(output / "provenance/asset_hashes.json", access.hashes)
            manifest["future_reads_by_run"] = {k: len(v) for k, v in access.future_by_run.items()}
            manifest["unique_sensor_assets_read"] = sum(k.startswith(("images/", "lidar/")) for k in access.hashes)
            try:
                read_checked(root / "manifest.yaml", MANIFEST_SHA)
                read_checked(Path(config["split_manifest"]), SPLIT_SHA)
                for path, expected in access.hashes.items():
                    read_checked(root / path, expected)
                for path, expected in PRIOR_HASHES.items():
                    read_checked(prior / path, expected)
                read_checked(Path(config["checkpoint"]), CHECKPOINT_SHA)
                manifest["inputs_checkpoint_unchanged"] = True
            except Exception as error:
                manifest.update(status="BLOCKED", inputs_checkpoint_unchanged=False, error=str(error))
        if val_pred is not None:
            write_json(output / "artifacts/processing_ledger.json", [{"sample_id": c["sample_id"], "role": c["role"],
                "status": "processed" if val_done[i] else "NOT_PROCESSED", "reason": None if val_done[i] else manifest.get("error", "unknown")} for i, c in enumerate(val_selected)])
        manifest["end_utc"] = datetime.now(timezone.utc).isoformat()
        manifest["checkpoint_sha256"] = CHECKPOINT_SHA
        write_json(output / "artifacts/execution_manifest.json", manifest)
        print(json.dumps(clean(manifest), ensure_ascii=False), flush=True)
    return 0 if manifest["status"] == "EVALUATION_COMPLETE" else 1


def draw_figures(output: Path, selected: list[dict], views: list[dict], xy: np.ndarray, mask: np.ndarray,
                 prediction: np.ndarray, rows: list[dict]) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    chosen = []
    for field, value in (("normal_recovery", "normal"), ("shape", "left"), ("shape", "right"), ("normal_recovery", "recovery"), ("role", "validation_observation")):
        candidates = [i for i, c in enumerate(selected) if c[field] == value and i not in chosen]
        if candidates:
            chosen.append(candidates[0])
    unsupported = [i for i, c in enumerate(selected) if not mask[i].any() and i not in chosen]
    if unsupported:
        chosen.append(unsupported[0])
    worst = sorted(range(len(selected)), key=lambda i: rows[i]["ade_m"] if rows[i]["ade_m"] is not None else -1, reverse=True)
    chosen += [i for i in worst if i not in chosen][:8-len(chosen)]
    write_json(output / "figures/figure_selection.json", [{"index": i, "sample_id": selected[i]["sample_id"]} for i in chosen])
    for i in chosen:
        fig, ax = plt.subplots(figsize=(6, 6))
        raw = views[i]["raw_prefix_xy"]
        ax.plot(raw[:, 0], raw[:, 1], color="gray", label="observed contiguous prefix")
        ax.plot(xy[i, mask[i], 0], xy[i, mask[i], 1], "go-", label="teacher support")
        ax.plot(prediction[i, :, 0], prediction[i, :, 1], "r--", label="model all20 (tail UNKNOWN)")
        ax.plot(prediction[i, mask[i], 0], prediction[i, mask[i], 1], "r-", label="model on teacher support")
        ax.plot(GRID, np.zeros(20), "b:", label="straight baseline")
        ax.scatter([0], [0], color="black", label="origin not scored")
        ax.set(xlabel="X forward [m]", ylabel="Y left [m]", title=f"{selected[i]['sample_id']}\n{selected[i]['role']}; support={mask[i].sum()} / 20")
        ax.axis("equal"); ax.grid(); ax.legend(fontsize=7)
        fig.tight_layout(); fig.savefig(output / "figures" / f"example_{i:03d}.png", dpi=110); plt.close(fig)
