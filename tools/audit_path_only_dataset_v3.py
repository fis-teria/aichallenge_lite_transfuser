#!/usr/bin/env python3
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import csv
import hashlib
import json
from pathlib import Path
import sys
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from aic_transfuser_lite.data.dataset_view_v3 import (  # noqa: E402
    ControlTargetBoundsV3,
    load_temporal_training_batches_v3,
)
from aic_transfuser_lite.data.storage_v3 import validate_complete_dataset  # noqa: E402
from aic_transfuser_lite.training.train_v3 import (  # noqa: E402
    load_full_control_config_v3,
    motion_target_filter_config_v3,
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _bounds(config: dict[str, Any]) -> ControlTargetBoundsV3:
    model = config["model"]
    raw = model["control_bounds"]
    return ControlTargetBoundsV3(
        max_steering_rad=float(raw["max_steering_rad"]),
        max_steering_rate_radps=float(raw["max_steering_rate_radps"]),
        max_speed_mps=float(raw["max_speed_mps"]),
        min_acceleration_mps2=float(raw["min_acceleration_mps2"]),
        max_acceleration_mps2=float(raw["max_acceleration_mps2"]),
        min_jerk_mps3=float(raw["min_jerk_mps3"]),
        max_jerk_mps3=float(raw["max_jerk_mps3"]),
        control_dt_sec=float(model["control_dt_sec"]),
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--split-manifest", type=Path, required=True)
    parser.add_argument("--view-config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(f"audit output already exists: {args.output}")

    config = load_full_control_config_v3(args.config)
    view = yaml.safe_load(args.view_config.read_text(encoding="utf-8"))
    manifest = validate_complete_dataset(args.dataset_root)
    split_manifest = json.loads(args.split_manifest.read_text(encoding="utf-8"))
    if split_manifest.get("dataset_manifest_sha256") != manifest["manifest_sha256"]:
        raise ValueError("split manifest and Dataset V3 identity differ")
    leakage = split_manifest.get("leakage")
    if not isinstance(leakage, dict) or leakage.get("status") != "PASS":
        raise ValueError("split leakage status is not PASS")
    assignments = split_manifest.get("assignments")
    if not isinstance(assignments, list) or not assignments:
        raise ValueError("split assignments are missing")
    run_to_split = {str(row["run_id"]): str(row["split"]) for row in assignments}
    if len(run_to_split) != len(assignments):
        raise ValueError("split assignments contain duplicate run IDs")

    sample_counts: Counter[str] = Counter()
    with (args.dataset_root / "samples.csv").open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        sample_fields = tuple(reader.fieldnames or ())
        for row in reader:
            sample_counts[run_to_split[row["run_id"]]] += 1

    run_counts = Counter(run_to_split.values())
    source_splits: dict[str, set[str]] = defaultdict(set)
    for run in manifest["runs"]:
        source_splits[str(run["source_hash"])].add(run_to_split[str(run["run_id"])])
    source_hash_overlap_count = sum(len(splits) > 1 for splits in source_splits.values())
    if source_hash_overlap_count:
        raise ValueError("identical source hashes cross splits")

    data = config["data"]
    model = config["model"]
    common = {
        "image_height": int(data["image_height"]),
        "image_width": int(data["image_width"]),
        "lidar_points": int(data["lidar_points"]),
        "lidar_min_range_m": float(data["lidar_min_range_m"]),
        "lidar_max_range_m": float(data["lidar_max_range_m"]),
        "ego_features": tuple(data["ego_features"]),
        "ego_abs_limits": data.get("ego_abs_limits"),
        "trajectory_steps": int(model["trajectory_steps"]),
        "control_sequence_steps": int(model["control_sequence_steps"]),
        "camera_history_length": int(view["camera_history_length"]),
        "ego_history_length": int(view["ego_history_length"]),
        "command_history_length": int(view["command_history_length"]),
        "control_target_bounds": _bounds(config),
        "batch_size": 2,
        "motion_target_filter": motion_target_filter_config_v3(config),
    }
    accounting: dict[str, dict[str, Any]] = {}
    for split in ("train", "validation", "test"):
        loader = load_temporal_training_batches_v3(
            args.dataset_root,
            args.split_manifest,
            split=split,
            **common,
        )
        raw = int(sample_counts[split])
        base_excluded = sum(loader.base_exclusion_counts.values())
        accounting[split] = {
            "runs": int(run_counts[split]),
            "raw_samples": raw,
            "base_exclusions": dict(loader.base_exclusion_counts),
            "base_excluded_total": base_excluded,
            "unfiltered_valid": raw - base_excluded,
            "motion_candidates": loader.motion_target_candidate_count,
            "motion_observed_complete": loader.motion_target_observed_count,
            "motion_censored": loader.motion_target_censored_count,
            "motion_censored_stationary_prefix": (
                loader.motion_target_censored_stationary_prefix_count
            ),
            "motion_contradictory_complete_rejected": (
                loader.motion_target_rejected_count
            ),
            "teacher_quality_remaining": len(loader.usable_anchors),
            "remaining_launch_candidates": (
                loader.motion_target_candidate_count
                - loader.motion_target_rejected_count
            ),
            "legacy_partial_future_rejection_reconstruction": (
                loader.motion_target_rejected_count
                + loader.motion_target_censored_stationary_prefix_count
            ),
        }
    payload = {
        "format": "aic_path_only_dataset_audit_v3",
        "dataset_id": manifest["dataset_id"],
        "dataset_manifest_internal_sha256": manifest["manifest_sha256"],
        "dataset_manifest_file_sha256": _sha256(args.dataset_root / "manifest.yaml"),
        "samples_csv_sha256": _sha256(args.dataset_root / "samples.csv"),
        "runs_csv_sha256": _sha256(args.dataset_root / "runs.csv"),
        "split_manifest_file_sha256": _sha256(args.split_manifest),
        "split_manifest_internal_sha256": split_manifest.get("manifest_sha256"),
        "leakage": leakage,
        "source_hash_overlap_recomputed": source_hash_overlap_count,
        "accounting": accounting,
        "totals": {
            "runs": sum(item["runs"] for item in accounting.values()),
            "raw_samples": sum(item["raw_samples"] for item in accounting.values()),
            "base_excluded": sum(
                item["base_excluded_total"] for item in accounting.values()
            ),
        },
        "independence_limitations": [
            "leakage checks prove recorded identifiers and fingerprints do not cross splits",
            "separate runs from the same simulator/course are not statistically independent environments",
            "five validation runs limit run-level uncertainty resolution",
        ],
        "planned_reference_fields_in_samples_csv": [
            name
            for name in sample_fields
            if "planned" in name.lower() or "reference" in name.lower()
        ],
    }
    args.output.mkdir(parents=True)
    (args.output / "dataset_audit.json").write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps({"status": "COMPLETE", "output": str(args.output)}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
