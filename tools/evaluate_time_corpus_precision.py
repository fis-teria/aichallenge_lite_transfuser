"""Re-evaluate selected TimePath checkpoints at a specified precision."""
from __future__ import annotations

import argparse
import hashlib
import json
import resource
from pathlib import Path
from typing import Any

import numpy as np
import torch
from aic_transfuser_lite.data.time_split_v1 import content_sha256

from aic_transfuser_lite.data.time_training_cache_v1 import (
    TimeTrainingCacheDataset, verify_time_training_cache,
)
from aic_transfuser_lite.evaluation.time_batched_v1 import evaluate_time_batched
from aic_transfuser_lite.training.time_checkpoint_v1 import (
    TimeCheckpointIdentity, load_time_checkpoint,
)
from aic_transfuser_lite.training.time_config_v1 import TimeModelConfig, build_time_model


def _read(path: Path) -> Any:
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def _write_new(path: Path, value: Any) -> None:
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write("\n")


def _sha(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def evaluate(run: Path, cache: Path, output: Path, *, batch_size: int,
             workers: int, precision: str) -> dict[str, Any]:
    if output.exists():
        raise FileExistsError(f"output already exists: {output}")
    if precision not in {"float32", "bf16"}:
        raise ValueError("precision must be float32 or bf16")
    if type(batch_size) is not int or batch_size <= 0 or type(workers) is not int or workers < 0:
        raise ValueError("positive batch size and nonnegative workers required")
    if not torch.cuda.is_available():
        raise RuntimeError("precision reevaluation requires CUDA")
    comparison = _read(run / "comparison.json")
    teacher = _read(run / "teacher_manifest.json")
    if teacher["manifest_sha256"] != content_sha256({k:v for k,v in teacher.items() if k != "manifest_sha256"}):
        raise ValueError("teacher manifest digest mismatch")
    if comparison.get("status") != "COMPLETE" or comparison.get("test_evaluated") is not False:
        raise ValueError("paired run is incomplete or test was evaluated")
    cache_identity = verify_time_training_cache(cache)
    if cache_identity.get("manifest_sha256") != teacher.get("cache_sha256"):
        raise ValueError("cache does not match training teacher manifest")
    validation = TimeTrainingCacheDataset(cache, "validation", verify_hashes=False)
    if len(validation) != teacher["validation_anchor_count"]:
        raise ValueError("validation anchor count differs from trained manifest")
    output.mkdir(parents=True, exist_ok=False)
    torch.set_num_threads(4)
    soft, hard = resource.getrlimit(resource.RLIMIT_NOFILE)
    resource.setrlimit(resource.RLIMIT_NOFILE, (max(soft, min(hard, 8192)), hard))
    torch.manual_seed(42)
    torch.cuda.manual_seed_all(42)
    torch.backends.cuda.matmul.allow_tf32 = False
    torch.backends.cudnn.allow_tf32 = False
    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.deterministic = True
    summaries: dict[str, Any] = {}
    for arm_name in ("command_off", "command_on"):
        arm = run / arm_name
        result = _read(arm / "result.json")
        plan = _read(arm / "plan.json")
        best = arm / "best.pt"
        if result.get("status") != "COMPLETE" or not best.is_file():
            raise ValueError(f"{arm_name} selected checkpoint is unavailable")
        config = TimeModelConfig.from_dict(result["config"])
        if validation.config != config.dataset_config():
            raise ValueError(f"{arm_name} cache preprocessing config differs from checkpoint config")
        identity = TimeCheckpointIdentity(**plan["identity"])
        if (identity.teacher_manifest_sha256 != teacher["manifest_sha256"]
                or identity.split_manifest_sha256 != validation.split_manifest["manifest_sha256"]):
            raise ValueError("selected checkpoint teacher/split identity mismatch")
        print(json.dumps({"phase":"EVALUATING", "arm":arm_name, "precision":precision}), flush=True)
        model = build_time_model(config).to("cuda")
        load_time_checkpoint(best, config=config, identity=identity, model=model, mode="finetune")
        metrics, predictions = evaluate_time_batched(
            model, validation, run_ids=validation.run_ids,
            split_manifest=validation.split_manifest, batch_size=batch_size,
            workers=workers, precision=precision)
        metric_path = output / f"{arm_name}_metrics.json"
        prediction_path = output / f"{arm_name}_predictions.npy"
        _write_new(metric_path, metrics)
        with prediction_path.open("xb") as stream:
            np.save(stream, predictions.numpy(), allow_pickle=False)
        summaries[arm_name] = {
            "source_best_checkpoint": str(best.resolve()),
            "source_best_sha256": _sha(best),
            "selected_epoch": result.get("best_epoch"),
            "selection_precision": plan.get("precision"),
            "reevaluation_precision": precision,
            "metrics_path": str(metric_path.resolve()),
            "predictions_path": str(prediction_path.resolve()),
            "run_macro_3s_raw_error_m": metrics.get("run_macro_mean", {}).get("3s", {}).get("raw_error_m"),
            "all_point_ade_m": metrics.get("all_point_ade_m"),
        }
        print(json.dumps({"phase":"ARM_COMPLETE", "arm":arm_name, **summaries[arm_name]}), flush=True)
        del model
        torch.cuda.empty_cache()
    summary = {"status": "COMPLETE", "arms": summaries,
               "cache_manifest_sha256": cache_identity["manifest_sha256"],
               "teacher_manifest_sha256": teacher.get("manifest_sha256"),
               "scope": "offline_fixed_condition_run_holdout_precision_reevaluation",
               "test_evaluated": False, "runtime_ready": False,
               "epoch_reselection": False, "retraining": False}
    _write_new(output / "summary.json", summary)
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--cache", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--precision", choices=("float32", "bf16"), default="float32")
    args = parser.parse_args()
    evaluate(args.run, args.cache, args.output, batch_size=args.batch_size,
             workers=args.workers, precision=args.precision)


if __name__ == "__main__":
    main()
