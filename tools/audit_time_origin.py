"""Native WSL audit of near-origin teachers and fixed FP32 predictions.

Reads train/validation small cached arrays; never reads or selects on test.
The old heading-only predicate is preserved here as a diagnostic comparator.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
from typing import Any

import numpy as np

from aic_transfuser_lite.data.time_training_cache_v1 import TimeTrainingCacheDataset


def old_foldback(xy_m: np.ndarray) -> bool:
    """Recorded v1 predicate over finite [30,2] metre coordinates."""
    steps = np.diff(np.vstack((np.zeros((1, 2)), xy_m)), axis=0)
    moving = steps[np.linalg.norm(steps, axis=1) > .01]
    heading = np.arctan2(moving[:, 1], moving[:, 0])
    turns = np.arctan2(np.sin(np.diff(heading)), np.cos(np.diff(heading)))
    return bool(np.any(np.abs(turns) > 1.2))


def quantiles(values: np.ndarray) -> dict[str, float] | None:
    if not values.size:
        return None
    return dict(zip(("min", "median", "p95", "p99", "max"),
                    np.quantile(values, (0, .5, .95, .99, 1)).tolist()))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cache", type=Path, required=True)
    ap.add_argument("--predictions", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args()
    identity = json.loads((args.cache / "identity.json").read_text())
    # Verify every small file consumed by this audit. Sensor payloads are not read.
    checked = []
    for row in identity["cache_files"]:
        if row["path"].endswith(("inputs.npz", "labels.npz", "anchors.jsonl")):
            path = (args.cache / row["path"]).resolve()
            if not path.is_relative_to(args.cache.resolve()):
                raise ValueError("cache path outside root")
            data = path.read_bytes()
            if len(data) != row["bytes"] or hashlib.sha256(data).hexdigest() != row["sha256"]:
                raise ValueError(f"changed audit source: {row['path']}")
            checked.append(row)
    predictions = np.load(args.predictions, allow_pickle=False)
    output: dict[str, Any] = {"scope": "OFFLINE_TRAIN_VALIDATION_ORIGIN_DIAGNOSTIC",
        "test_evaluated": False, "retraining": False,
        "cache_manifest_sha256": identity["manifest_sha256"], "checked_files": checked,
        "prediction_file": str(args.predictions),
        "prediction_sha256": hashlib.sha256(args.predictions.read_bytes()).hexdigest(), "splits": {}}
    for split in ("train", "validation"):
        dataset = TimeTrainingCacheDataset(args.cache, split, verify_hashes=False)
        ego = np.concatenate([r["inputs"]["ego"] for r in dataset._runs])
        full = dataset.input_valid & dataset.xy_mask.all(axis=1)
        teacher = dataset.targets
        if split == "validation" and predictions.shape != teacher.shape:
            raise ValueError("validation prediction shape/order contract mismatch")
        speed = np.abs(ego[:, -1, 0])
        selected = {"all": full, "stationary_le_0p03": full & (speed <= .03),
            "creep_0p03_to_0p25": full & (speed > .03) & (speed <= .25),
            "slow_0p25_to_0p6": full & (speed > .25) & (speed <= .6),
            "moving_gt_0p6": full & (speed > .6)}
        groups = {}
        for name, mask in selected.items():
            gt = teacher[mask]
            row: dict[str, Any] = {"anchors": int(mask.sum()), "current_speed_mps": quantiles(speed[mask]),
                "teacher_first_norm_m": quantiles(np.linalg.norm(gt[:, 0], axis=1)),
                "teacher_first_y_m": quantiles(gt[:, 0, 1]),
                "teacher_old_foldback": sum(old_foldback(x) for x in gt),
                "teacher_3s_endpoint_norm_m": quantiles(np.linalg.norm(gt[:, -1], axis=1))}
            if split == "validation":
                pred = predictions[mask]
                if not np.isfinite(pred).all():
                    raise ValueError("nonfinite eligible predictions")
                row.update(prediction_old_foldback=sum(old_foldback(x) for x in pred),
                    prediction_first_error_m=quantiles(np.linalg.norm(pred[:, 0] - gt[:, 0], axis=1)),
                    prediction_first_x_m=quantiles(pred[:, 0, 0]),
                    prediction_first_y_m=quantiles(pred[:, 0, 1]))
            groups[name] = row
        output["splits"][split] = {"total_anchors": len(dataset), "full_supported_inputs": int(full.sum()),
            "run_counts": dict(Counter(dataset.run_ids)), "groups": groups}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as stream:
        json.dump(output, stream, indent=2, allow_nan=False)
    print(json.dumps({"splits": output["splits"], "checked_files": len(checked)}, allow_nan=False))


if __name__ == "__main__":
    main()
