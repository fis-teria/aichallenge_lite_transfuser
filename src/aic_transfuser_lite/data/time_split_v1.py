"""Whole-run fixed-course holdout and source lineage for the time baseline.

No frames are inspected to choose a split. A receipt-only draft cannot authorize
training: verify every raw bag at its destination before consuming the manifest.
"""
from __future__ import annotations

from collections import Counter
import copy
import hashlib
import json
from pathlib import Path, PurePosixPath
from typing import Any, Iterable

FORMAT = "time_run_holdout_v1"
SPLITS = ("train", "validation", "test")
COUNTS = {"train": 6, "validation": 2, "test": 2}


def content_sha256(value: Any) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode("utf-8")).hexdigest()


def _sha(value: Any) -> None:
    if type(value) is not str or len(value) != 64 or any(c not in "0123456789abcdef" for c in value):
        raise ValueError("expected lowercase SHA-256")


def _path(value: Any) -> None:
    if (type(value) is not str or not value or "\\" in value or ":" in value
            or PurePosixPath(value).is_absolute() or ".." in PurePosixPath(value).parts
            or str(PurePosixPath(value)) != value):
        raise ValueError("source path must be a normalized relative POSIX path")


def _identity(manifest: dict[str, Any]) -> str:
    # Destination verification changes status, not the frozen assignment identity.
    return content_sha256({k: v for k, v in manifest.items()
                           if k not in {"manifest_sha256", "sources_verified"}})


def validate_time_split(manifest: dict[str, Any], *, require_verified: bool = False) -> None:
    keys = {"format", "scope", "seed", "counts_per_speed", "receipt_sha256", "runs",
            "sources_verified", "manifest_sha256"}
    if type(manifest) is not dict or set(manifest) != keys:
        raise ValueError("time split schema mismatch")
    if manifest["format"] != FORMAT or manifest["scope"] != "same_course_same_conditions_run_holdout":
        raise ValueError("time split is only a fixed-condition run holdout")
    if type(manifest["seed"]) is not int or manifest["seed"] < 0:
        raise ValueError("split seed must be nonnegative integer")
    counts = manifest["counts_per_speed"]
    if type(counts) is not dict or counts != COUNTS or any(type(v) is not int for v in counts.values()):
        raise ValueError("time split requires exactly 6/2/2 runs per speed setting")
    _sha(manifest["receipt_sha256"])
    if type(manifest["sources_verified"]) is not bool:
        raise ValueError("sources_verified must be bool")
    if require_verified and not manifest["sources_verified"]:
        raise ValueError("raw sources have not been hash verified at the destination")
    if type(manifest["runs"]) is not list or not manifest["runs"]:
        raise ValueError("split requires run records")
    ids: set[str] = set()
    hashes: set[str] = set()
    paths: set[str] = set()
    counts_seen: Counter[tuple[int, str]] = Counter()
    for run in manifest["runs"]:
        if type(run) is not dict or set(run) != {"run_id", "speed_cap_kmh", "split", "sources"}:
            raise ValueError("run schema mismatch")
        rid = run["run_id"]
        if type(rid) is not str or not rid or rid in ids:
            raise ValueError("duplicate or invalid run identity")
        ids.add(rid)
        if type(run["speed_cap_kmh"]) is not int or run["speed_cap_kmh"] <= 0:
            raise ValueError("speed setting must be positive integer km/h, not measured speed")
        if run["split"] not in SPLITS:
            raise ValueError("unknown split")
        counts_seen[run["speed_cap_kmh"], run["split"]] += 1
        if type(run["sources"]) is not list or not run["sources"]:
            raise ValueError("run requires raw bag sources")
        for source in run["sources"]:
            if type(source) is not dict or set(source) != {"path", "sha256"}:
                raise ValueError("source schema mismatch")
            _path(source["path"])
            _sha(source["sha256"])
            if source["path"] in paths or source["sha256"] in hashes:
                raise ValueError("duplicate raw content/path across runs or parts")
            paths.add(source["path"])
            hashes.add(source["sha256"])
    for speed in {speed for speed, _ in counts_seen}:
        if any(counts_seen[speed, split] != count for split, count in COUNTS.items()):
            raise ValueError("each speed setting requires 6/2/2 whole runs")
    if manifest["manifest_sha256"] != _identity(manifest):
        raise ValueError("frozen split manifest hash mismatch")


def build_time_split(runs: Iterable[dict[str, Any]], *, receipt_sha256: str,
                     seed: int = 42) -> dict[str, Any]:
    """Assign 10 runs per speed via stable seeded identity hashing; never fit on data."""
    if type(seed) is not int or seed < 0:
        raise ValueError("seed must be nonnegative integer")
    records = copy.deepcopy(list(runs))
    if not records or any(type(r) is not dict or set(r) != {"run_id", "speed_cap_kmh", "sources"}
                          for r in records):
        raise ValueError("expected run identity, speed setting and raw sources")
    for speed in {r["speed_cap_kmh"] for r in records}:
        group = sorted((r for r in records if r["speed_cap_kmh"] == speed),
                       key=lambda r: content_sha256([seed, r["run_id"]]))
        if len(group) != 10:
            raise ValueError("exactly ten runs per speed setting required")
        for i, run in enumerate(group):
            run["split"] = "train" if i < 6 else "validation" if i < 8 else "test"
    result = {"format": FORMAT, "scope": "same_course_same_conditions_run_holdout",
              "seed": seed, "counts_per_speed": dict(COUNTS), "receipt_sha256": receipt_sha256,
              "runs": sorted(records, key=lambda r: r["run_id"]), "sources_verified": False}
    result["manifest_sha256"] = _identity(result)
    validate_time_split(result)
    return result


def split_from_packaging_receipt(path: Path, *, seed: int = 42) -> dict[str, Any]:
    """Use receipt IDs and bag hashes only; copying metadata is not raw verification."""
    raw = path.read_bytes()
    receipt = json.loads(raw)
    if receipt.get("scope") != "RAW_20_COMPLETED_LAPS_NOT_CANONICAL":
        raise ValueError("not a completed-lap packaging receipt")
    runs = []
    for run in receipt["runs"]:
        prefix = f"runs/{run['run_id']}/bag/"
        sources = [{"path": f["path"], "sha256": f["sha256"]} for f in receipt["files"]
                   if f["path"].startswith(prefix) and f["path"].endswith((".db3", ".mcap"))]
        runs.append({"run_id": run["run_id"], "speed_cap_kmh": run["speed_cap_kmh"],
                     "sources": sources})
    return build_time_split(runs, receipt_sha256=hashlib.sha256(raw).hexdigest(), seed=seed)


def verify_time_split_sources(manifest: dict[str, Any], root: Path) -> dict[str, Any]:
    """Read-only stream hashing after SSD migration. No copy, extraction or deletion."""
    validate_time_split(manifest)
    base = root.resolve(strict=True)
    for run in manifest["runs"]:
        for source in run["sources"]:
            path = (base / source["path"]).resolve(strict=True)
            if not path.is_relative_to(base):
                raise ValueError("source escapes dataset root")
            digest = hashlib.sha256()
            with path.open("rb") as handle:
                for block in iter(lambda: handle.read(1024 * 1024), b""):
                    digest.update(block)
            if digest.hexdigest() != source["sha256"]:
                raise ValueError(f"raw source hash mismatch: {source['path']}")
    result = copy.deepcopy(manifest)
    result["sources_verified"] = True
    return result


def assert_split_membership(manifest: dict[str, Any], run_ids: Iterable[str], *, split: str,
                            require_verified: bool = True) -> None:
    validate_time_split(manifest, require_verified=require_verified)
    if split not in SPLITS:
        raise ValueError("unknown split")
    allowed = {r["run_id"] for r in manifest["runs"] if r["split"] == split}
    if not set(run_ids) <= allowed:
        raise ValueError("run crosses the requested split boundary")


def audit_initialization_lineage(manifest: dict[str, Any], *, mode: str,
                                 source_checkpoint_sha256: str | None = None,
                                 seen_source_sha256: list[str] | None = None) -> dict[str, Any]:
    """Unknown inherited data never earns a test-unseen claim. Scratch is explicit."""
    validate_time_split(manifest)
    if mode not in {"scratch", "finetune"}:
        raise ValueError("initialization must be scratch or finetune")
    if mode == "scratch":
        if source_checkpoint_sha256 is not None or seen_source_sha256 is not None:
            raise ValueError("scratch cannot have inherited weights/data")
        return {"mode": mode, "source_checkpoint_sha256": None,
                "status": "SCRATCH", "scratch_comparison_required": False}
    _sha(source_checkpoint_sha256)
    if seen_source_sha256 is None or seen_source_sha256 == []:
        return {"mode": mode, "source_checkpoint_sha256": source_checkpoint_sha256,
                "status": "UNKNOWN", "scratch_comparison_required": True}
    if type(seen_source_sha256) is not list:
        raise ValueError("seen source digests must be a list or unknown")
    for value in seen_source_sha256:
        _sha(value)
    heldout = {s["sha256"] for r in manifest["runs"] if r["split"] != "train" for s in r["sources"]}
    overlap = sorted(heldout.intersection(seen_source_sha256))
    return {"mode": mode, "source_checkpoint_sha256": source_checkpoint_sha256,
            "status": "HELDOUT_OVERLAP" if overlap else "DECLARED_RAW_HASH_DISJOINT",
            "overlap_sha256": overlap, "seen_source_sha256": sorted(set(seen_source_sha256)),
            "scratch_comparison_required": bool(overlap)}


def command_comparison_plan(manifest: dict[str, Any], *, seed: int, max_anchors: int,
                            max_optimizer_steps: int) -> dict[str, Any]:
    """Freeze a bounded paired experiment; no training or automatic test evaluation."""
    validate_time_split(manifest)
    if type(seed) is not int or seed < 0 or any(type(v) is not int or v <= 0
                                              for v in (max_anchors, max_optimizer_steps)):
        raise ValueError("experiment requires explicit seed and positive finite budgets")
    return {"format": "time_command_comparison_v1", "split_sha256": manifest["manifest_sha256"],
            "scope": manifest["scope"], "initialization": "scratch", "seed": seed,
            "sampling": "natural_frame_distribution", "loss_weighting": "uniform_supported_anchor",
            "max_anchors_per_arm": max_anchors, "max_optimizer_steps_per_arm": max_optimizer_steps,
            "arms": [{"name": "command_off", "use_command_history": False},
                     {"name": "command_on", "use_command_history": True}],
            "test_usage": "sealed_until_final_evaluation", "preprocessing_fit_split": "train",
            "report": ["natural_frames", "speed_setting_counts", "measured_speed_distribution",
                       "run_macro", "worst_run", "horizon_support", "input_invalid", "output_reject"],
            "runtime_ready": False}
