from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import sys
import time
from typing import Any

import numpy as np
import torch
from torch.utils.data import DataLoader, Dataset
import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from aic_transfuser_lite.config import validate_v1_config
from aic_transfuser_lite.data.dataset_v2 import DrivingDatasetV2
from aic_transfuser_lite.models.factory import build_model
from aic_transfuser_lite.training.checkpoint_v1 import load_v1_checkpoint
from aic_transfuser_lite.training.metrics import (
    V1MetricAccumulator,
    controller_proxy_steering,
    curvature_bucket_indices,
)


METRIC_VERSION = "transfuser_lite_v1_offline_metrics_v1"
FIRST_CORNER_SPEC_VERSION = "transfuser_lite_v1_first_corner_spec_v1"
REFERENCE_VERSION = "transfuser_lite_v1_offline_reference_v1"
ABLATION_SEED = 20260810
ABLATION_SCENARIOS = (
    "image_shuffle",
    "lidar_shuffle",
    "speed_shuffle",
    "lidar_invalid_mask_all_zero",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Evaluate a Dataset-v2 TransFuser Lite v1 checkpoint"
    )
    parser.add_argument("--root", type=Path, default=REPO_ROOT)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument(
        "--test-index",
        type=Path,
        default=Path("datasets/processed/aic_real_dataset_v2/test_index.csv"),
    )
    parser.add_argument("--first-corner-spec", type=Path, required=True)
    parser.add_argument("--reference-metrics", type=Path, default=None)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--num-workers", type=int, default=4)
    parser.add_argument("--latency-runs", type=int, default=100)
    parser.add_argument("--device", choices=("auto", "cpu", "cuda"), default="auto")
    parser.add_argument(
        "--latency-precision", choices=("fp32", "amp"), default="fp32"
    )
    return parser.parse_args()


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def resolve_path(root: Path, path: Path) -> Path:
    return path.resolve() if path.is_absolute() else (root / path).resolve()


def choose_device(requested: str) -> torch.device:
    if requested == "auto":
        return torch.device("cuda" if torch.cuda.is_available() else "cpu")
    if requested == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("--device cuda requested but CUDA is unavailable")
    return torch.device(requested)


def make_derangement(length: int, seed: int) -> torch.Tensor:
    """Return a deterministic permutation with no unchanged sample."""
    if length < 2:
        raise ValueError("Shuffle ablations require at least two samples")
    expected = torch.arange(length)
    generator = torch.Generator().manual_seed(seed)
    for _ in range(100):
        permutation = torch.randperm(length, generator=generator)
        if bool(torch.all(permutation != expected)):
            return permutation
    raise RuntimeError("Failed to construct a derangement in 100 attempts")


class AblatedDataset(Dataset[dict[str, torch.Tensor]]):
    """Keep each target fixed while changing exactly one model input."""

    def __init__(
        self,
        base: DrivingDatasetV2,
        scenario: str,
        permutation: torch.Tensor | None = None,
    ) -> None:
        allowed = {"baseline", *ABLATION_SCENARIOS}
        if scenario not in allowed:
            raise ValueError(f"Unknown ablation scenario: {scenario}")
        if scenario.endswith("shuffle"):
            if permutation is None or tuple(permutation.shape) != (len(base),):
                raise ValueError("Shuffle scenario requires a full-length permutation")
            if sorted(int(value) for value in permutation.tolist()) != list(
                range(len(base))
            ):
                raise ValueError("Ablation permutation is not one-to-one")
            if bool(torch.any(permutation == torch.arange(len(base)))):
                raise ValueError("Ablation permutation contains unchanged samples")
        self.base = base
        self.scenario = scenario
        self.permutation = permutation

    def __len__(self) -> int:
        return len(self.base)

    def __getitem__(self, index: int) -> dict[str, torch.Tensor]:
        sample = dict(self.base[index])
        if self.scenario == "baseline":
            return sample
        if self.scenario == "lidar_invalid_mask_all_zero":
            lidar = sample["lidar"].clone()
            if lidar.ndim != 2 or lidar.shape[0] != 2:
                raise ValueError("Dataset v2 LiDAR must have range and validity channels")
            lidar[1].zero_()
            sample["lidar"] = lidar
            return sample

        assert self.permutation is not None
        shuffled = self.base[int(self.permutation[index])]
        key = {
            "image_shuffle": "image",
            "lidar_shuffle": "lidar",
            "speed_shuffle": "ego",
        }[self.scenario]
        sample[key] = shuffled[key]
        return sample


def _validate_number(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be numeric")
    result = float(value)
    if not np.isfinite(result) or result < 0.0:
        raise ValueError(f"{name} must be finite and non-negative")
    return result


def validate_first_corner_spec(
    path: Path, *, test_index: Path, metadata_path: Path, frame: Any
) -> tuple[dict[str, Any], list[str]]:
    spec = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(spec, dict) or spec.get("format_version") != FIRST_CORNER_SPEC_VERSION:
        raise ValueError(f"Expected {FIRST_CORNER_SPEC_VERSION} first-corner spec")
    dataset = spec.get("dataset")
    if not isinstance(dataset, dict) or set(dataset) != {
        "test_index_sha256",
        "metadata_sha256",
    }:
        raise ValueError("first-corner spec dataset hashes are incomplete")
    actual = {
        "test_index_sha256": sha256_file(test_index),
        "metadata_sha256": sha256_file(metadata_path),
    }
    if dataset != actual:
        raise ValueError(
            f"first-corner spec dataset hash mismatch: expected {dataset}, got {actual}"
        )
    sample_ids = spec.get("sample_ids")
    if (
        not isinstance(sample_ids, list)
        or not sample_ids
        or any(not isinstance(value, str) or not value for value in sample_ids)
        or len(sample_ids) != len(set(sample_ids))
    ):
        raise ValueError("first-corner sample_ids must be a non-empty unique string list")
    frame_ids = frame["sample_id"].astype(str).tolist()
    positions = {sample_id: index for index, sample_id in enumerate(frame_ids)}
    missing = [sample_id for sample_id in sample_ids if sample_id not in positions]
    if missing:
        raise ValueError(f"first-corner sample IDs are absent from test split: {missing}")
    ordered_positions = [positions[sample_id] for sample_id in sample_ids]
    if ordered_positions != sorted(ordered_positions):
        raise ValueError("first-corner sample IDs are not in test-index order")
    return spec, sample_ids


def load_reference_metrics(path: Path | None) -> dict[str, Any] | None:
    if path is None:
        return None
    reference = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(reference, dict) or reference.get("format_version") != REFERENCE_VERSION:
        raise ValueError(f"Expected {REFERENCE_VERSION} reference metrics")
    test = reference.get("test")
    if not isinstance(test, dict) or set(test) != {
        "ade_m",
        "fde_m",
        "sharp_controller_proxy_mae_rad",
    }:
        raise ValueError("reference test metrics are incomplete or contain unknown keys")
    return {
        **reference,
        "test": {
            name: _validate_number(value, f"reference.test.{name}")
            for name, value in test.items()
        },
    }


def evaluate_loader(
    model: torch.nn.Module,
    loader: DataLoader,
    device: torch.device,
    *,
    num_waypoints: int,
    straight_threshold_per_m: float,
    sharp_threshold_per_m: float,
    controller_wheelbase_m: float,
) -> tuple[dict[str, Any], dict[str, torch.Tensor]]:
    accumulator = V1MetricAccumulator(
        num_waypoints=num_waypoints,
        straight_threshold_per_m=straight_threshold_per_m,
        sharp_threshold_per_m=sharp_threshold_per_m,
        controller_wheelbase_m=controller_wheelbase_m,
    )
    predicted_chunks: list[torch.Tensor] = []
    target_chunks: list[torch.Tensor] = []
    predicted_speed_chunks: list[torch.Tensor] = []
    target_speed_chunks: list[torch.Tensor] = []
    model.eval()
    with torch.inference_mode():
        for batch in loader:
            image = batch["image"].to(device, non_blocking=True)
            lidar = batch["lidar"].to(device, non_blocking=True)
            ego = batch["ego"].to(device, non_blocking=True)
            output = model(image, lidar, ego)
            if set(output) != {"waypoints", "target_speed"}:
                raise RuntimeError(
                    f"Static v1 output contract drifted: {sorted(output)}"
                )
            metric_batch = {
                "waypoints": batch["waypoints"].to(device, non_blocking=True),
                "target_speed": batch["target_speed"].to(
                    device, non_blocking=True
                ),
            }
            accumulator.update(output, metric_batch)
            predicted_chunks.append(output["waypoints"].detach().cpu())
            target_chunks.append(batch["waypoints"].detach().cpu())
            predicted_speed_chunks.append(output["target_speed"].detach().cpu())
            target_speed_chunks.append(batch["target_speed"].detach().cpu())

    predicted = torch.cat(predicted_chunks)
    target = torch.cat(target_chunks)
    predicted_control = controller_proxy_steering(
        predicted, wheelbase_m=controller_wheelbase_m
    )
    target_control = controller_proxy_steering(
        target, wheelbase_m=controller_wheelbase_m
    )
    distance = torch.linalg.vector_norm(predicted - target, dim=-1)
    details = {
        "distance_m": distance,
        "controller_error_rad": predicted_control - target_control,
        "speed_error_mps": torch.cat(predicted_speed_chunks).flatten()
        - torch.cat(target_speed_chunks).flatten(),
        "curvature_bucket": curvature_bucket_indices(
            target,
            straight_threshold_per_m=straight_threshold_per_m,
            sharp_threshold_per_m=sharp_threshold_per_m,
        ),
    }
    return accumulator.finalize(), details


def subset_summary(details: dict[str, torch.Tensor], indices: torch.Tensor) -> dict[str, Any]:
    if indices.ndim != 1 or indices.numel() == 0:
        raise ValueError("Metric subset must contain at least one one-dimensional index")
    distance = details["distance_m"][indices]
    control_error = details["controller_error_rad"][indices]
    speed_error = details["speed_error_mps"][indices]
    return {
        "sample_count": int(indices.numel()),
        "ade_m": float(distance.mean()),
        "fde_m": float(distance[:, -1].mean()),
        "waypoint_horizon_mae_m": distance.mean(dim=0).tolist(),
        "speed_mae_mps": float(speed_error.abs().mean()),
        "controller_proxy_mae_rad": float(control_error.abs().mean()),
        "controller_proxy_bias_rad": float(control_error.mean()),
    }


def first_corner_summary(
    details: dict[str, torch.Tensor], frame: Any, sample_ids: list[str]
) -> dict[str, Any]:
    positions = {
        sample_id: index
        for index, sample_id in enumerate(frame["sample_id"].astype(str).tolist())
    }
    indices = torch.tensor([positions[sample_id] for sample_id in sample_ids])
    return {"sample_ids": sample_ids, **subset_summary(details, indices)}


def ratio(numerator: float | None, denominator: float | None) -> float | None:
    if numerator is None or denominator is None or denominator <= 0.0:
        return None
    return float(numerator) / float(denominator)


def benchmark_latency(
    model: torch.nn.Module,
    item: dict[str, torch.Tensor],
    device: torch.device,
    *,
    runs: int,
    precision: str,
) -> dict[str, Any]:
    if runs < 1:
        raise ValueError("latency-runs must be positive")
    if precision == "amp" and device.type != "cuda":
        raise ValueError("AMP latency requires CUDA")
    image = item["image"].unsqueeze(0).to(device)
    lidar = item["lidar"].unsqueeze(0).to(device)
    ego = item["ego"].unsqueeze(0).to(device)
    amp = precision == "amp"
    timings: list[float] = []
    model.eval()
    with torch.inference_mode():
        for _ in range(20):
            with torch.autocast(
                device_type=device.type, dtype=torch.float16, enabled=amp
            ):
                model(image, lidar, ego)
        if device.type == "cuda":
            torch.cuda.synchronize()
        for _ in range(runs):
            start = time.perf_counter()
            with torch.autocast(
                device_type=device.type, dtype=torch.float16, enabled=amp
            ):
                model(image, lidar, ego)
            if device.type == "cuda":
                torch.cuda.synchronize()
            timings.append((time.perf_counter() - start) * 1000.0)
    return {
        "runs": runs,
        "batch_size": 1,
        "precision": precision,
        "p50_ms": float(np.quantile(timings, 0.50)),
        "p95_ms": float(np.quantile(timings, 0.95)),
        "max_ms": float(np.max(timings)),
    }


def threshold_gate(value: float, threshold: float, *, strict: bool = False) -> dict[str, Any]:
    passed = value < threshold if strict else value <= threshold
    return {
        "status": "PASS" if passed else "FAIL",
        "value": value,
        "operator": "<" if strict else "<=",
        "threshold": threshold,
    }


def build_gate_report(
    baseline: dict[str, Any],
    first_corner: dict[str, Any],
    ablations: dict[str, Any],
    latency: dict[str, Any],
    reference: dict[str, Any] | None,
    ego_features: tuple[str, ...],
) -> dict[str, Any]:
    if reference is None:
        ade_gate: dict[str, Any] = {
            "status": "NOT_EVALUATED_MISSING_REFERENCE",
            "value": baseline["ade_m"],
        }
        fde_gate: dict[str, Any] = {
            "status": "NOT_EVALUATED_MISSING_REFERENCE",
            "value": baseline["fde_m"],
        }
    else:
        ade_gate = threshold_gate(baseline["ade_m"], reference["test"]["ade_m"])
        fde_gate = threshold_gate(baseline["fde_m"], reference["test"]["fde_m"])

    sharp_value = baseline["curvature_buckets"]["sharp"][
        "controller_proxy_mae_rad"
    ]
    if sharp_value is None:
        raise ValueError("Test split contains no sharp-curvature samples")
    sharp_reference_gate = (
        {
            "status": "NOT_EVALUATED_MISSING_REFERENCE",
            "value": sharp_value,
        }
        if reference is None
        else threshold_gate(
            sharp_value,
            reference["test"]["sharp_controller_proxy_mae_rad"]
        )
    )

    ablation_gates: dict[str, Any] = {}
    for name, summary in ablations.items():
        value = summary["sharp_controller_proxy_mae_ratio_vs_baseline"]
        ablation_gates[name] = (
            {"status": "FAIL_UNDEFINED_BASELINE", "value": None, "threshold": 5.0}
            if value is None
            else threshold_gate(value, 5.0)
        )
    turn_features = {"yaw_rate_rps", "steering_rad"}.intersection(ego_features)
    turn_state_gate = (
        {
            "status": "PASS_STRUCTURALLY_ABSENT",
            "ratio_by_construction": 1.0,
            "threshold": 3.0,
            "ego_features": list(ego_features),
        }
        if not turn_features
        else {
            "status": "NOT_EVALUATED_TURN_STATE_PRESENT",
            "present_features": sorted(turn_features),
            "threshold": 3.0,
        }
    )
    return {
        "test_ade_not_worse_than_reference": ade_gate,
        "test_fde_not_worse_than_reference": fde_gate,
        "sharp_controller_proxy_mae_not_worse_than_reference": sharp_reference_gate,
        "sharp_controller_proxy_mae_absolute_cap_rad": threshold_gate(
            sharp_value, 0.053
        ),
        "first_corner_abs_controller_bias_rad": threshold_gate(
            abs(first_corner["controller_proxy_bias_rad"]), 0.03, strict=True
        ),
        "single_modality_ratio_not_above_5x": ablation_gates,
        "turn_state_zero_or_shuffle_not_above_3x": turn_state_gate,
        "stop_head_disabled_model_stop_events": {
            "status": "PASS",
            "value": 0,
            "threshold": 0,
        },
        "gpu_batch1_p95_latency_ms": threshold_gate(
            latency["p95_ms"], 20.0, strict=True
        ),
    }


def atomic_write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp-{os.getpid()}")
    try:
        temporary.write_text(
            json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def main() -> None:
    args = parse_args()
    if args.batch_size < 1 or args.num_workers < 0:
        raise ValueError("batch-size must be positive and num-workers non-negative")
    root = args.root.resolve()
    checkpoint_path = resolve_path(root, args.checkpoint)
    test_index = resolve_path(root, args.test_index)
    corner_spec_path = resolve_path(root, args.first_corner_spec)
    reference_path = (
        resolve_path(root, args.reference_metrics)
        if args.reference_metrics is not None
        else None
    )
    output_path = resolve_path(root, args.output)

    checkpoint = load_v1_checkpoint(checkpoint_path, map_location="cpu")
    config = checkpoint["resolved_config"]
    validate_v1_config(config)
    if int(config["data"].get("format_version", -1)) != 2:
        raise ValueError("v1 offline evaluator accepts Dataset format 2 only")
    if config["model"]["heads"] != {
        "waypoints": True,
        "target_speed": True,
        "stop": False,
        "behavior_mode": False,
        "direct_control_aux": False,
    }:
        raise ValueError("v1 offline evaluator requires the static two-head contract")
    resolved_text = yaml.safe_dump(config, sort_keys=False, allow_unicode=True)
    resolved_sha256 = sha256_bytes(resolved_text.encode("utf-8"))
    if checkpoint["resolved_config_sha256"] != resolved_sha256:
        raise ValueError("Checkpoint resolved-config hash does not match its config")

    dataset = DrivingDatasetV2(test_index, config, training=False)
    corner_spec, corner_ids = validate_first_corner_spec(
        corner_spec_path,
        test_index=test_index,
        metadata_path=dataset.metadata_path,
        frame=dataset.frame,
    )
    reference = load_reference_metrics(reference_path)
    device = choose_device(args.device)

    # Full checkpoint state makes external ImageNet cache/network unnecessary.
    construction_config = copy.deepcopy(config)
    construction_config["model"]["camera"]["pretrained"] = False
    model = build_model(construction_config).to(device)
    model.load_state_dict(checkpoint["model"], strict=True)
    model.eval()

    sampler_cfg = config["training"]["sampler"]
    straight_threshold = float(sampler_cfg["straight_threshold_per_m"])
    sharp_threshold = float(sampler_cfg["sharp_threshold_per_m"])
    vehicle_provenance = dataset.metadata.get("vehicle_config_provenance")
    if not isinstance(vehicle_provenance, dict):
        raise ValueError("Dataset metadata lacks vehicle_config_provenance")
    controller_wheelbase_m = _validate_number(
        vehicle_provenance.get("wheelbase_m"),
        "metadata.vehicle_config_provenance.wheelbase_m",
    )
    if controller_wheelbase_m <= 0.0:
        raise ValueError("metadata wheelbase_m must be positive")
    permutation = make_derangement(len(dataset), ABLATION_SEED)
    loader_kwargs = {
        "batch_size": args.batch_size,
        "shuffle": False,
        "num_workers": args.num_workers,
        "pin_memory": device.type == "cuda",
    }
    baseline, baseline_details = evaluate_loader(
        model,
        DataLoader(AblatedDataset(dataset, "baseline"), **loader_kwargs),
        device,
        num_waypoints=int(config["data"]["num_waypoints"]),
        straight_threshold_per_m=straight_threshold,
        sharp_threshold_per_m=sharp_threshold,
        controller_wheelbase_m=controller_wheelbase_m,
    )
    corner_summary = first_corner_summary(
        baseline_details, dataset.frame, corner_ids
    )
    baseline_sharp = baseline["curvature_buckets"]["sharp"][
        "controller_proxy_mae_rad"
    ]
    ablations: dict[str, Any] = {}
    for scenario in ABLATION_SCENARIOS:
        summary, _ = evaluate_loader(
            model,
            DataLoader(
                AblatedDataset(dataset, scenario, permutation), **loader_kwargs
            ),
            device,
            num_waypoints=int(config["data"]["num_waypoints"]),
            straight_threshold_per_m=straight_threshold,
            sharp_threshold_per_m=sharp_threshold,
            controller_wheelbase_m=controller_wheelbase_m,
        )
        scenario_sharp = summary["curvature_buckets"]["sharp"][
            "controller_proxy_mae_rad"
        ]
        summary["sharp_controller_proxy_mae_ratio_vs_baseline"] = ratio(
            scenario_sharp, baseline_sharp
        )
        ablations[scenario] = summary

    latency = benchmark_latency(
        model,
        dataset[0],
        device,
        runs=args.latency_runs,
        precision=args.latency_precision,
    )
    ego_features = tuple(str(value) for value in config["data"]["ego_features"])
    report = {
        "metric_version": METRIC_VERSION,
        "artifact": {
            "checkpoint": str(checkpoint_path),
            "checkpoint_sha256": sha256_file(checkpoint_path),
            "checkpoint_epoch": int(checkpoint["epoch"]),
            "checkpoint_global_step": int(checkpoint["global_step"]),
            "resolved_config_sha256": resolved_sha256,
            "test_index": str(test_index),
            "test_index_sha256": sha256_file(test_index),
            "metadata": str(dataset.metadata_path),
            "metadata_sha256": sha256_file(dataset.metadata_path),
            "first_corner_spec": str(corner_spec_path),
            "first_corner_spec_sha256": sha256_file(corner_spec_path),
            "reference_metrics": str(reference_path) if reference_path else None,
            "reference_metrics_sha256": (
                sha256_file(reference_path) if reference_path else None
            ),
            "evaluator": str(Path(__file__).resolve()),
            "evaluator_sha256": sha256_file(Path(__file__).resolve()),
        },
        "environment": {
            "device": str(device),
            "gpu": torch.cuda.get_device_name(device)
            if device.type == "cuda"
            else None,
            "torch_version": torch.__version__,
        },
        "contract": {
            "dataset_format_version": 2,
            "model_inputs": ["image", "lidar_range_and_validity", "speed_mps"],
            "model_outputs": ["waypoints", "target_speed"],
            "teacher_or_debug_inputs": [],
            "curvature_thresholds_per_m": {
                "straight_upper": straight_threshold,
                "sharp_lower": sharp_threshold,
            },
            "controller_wheelbase_m": controller_wheelbase_m,
            "ablation_seed": ABLATION_SEED,
            "ablation_permutation_sha256": sha256_bytes(
                permutation.numpy().astype(np.int64).tobytes()
            ),
            "first_corner_spec": corner_spec,
        },
        "test": baseline,
        "first_corner": corner_summary,
        "ablations": ablations,
        "batch1_latency": latency,
        "reference_metrics": reference,
    }
    report["offline_gate"] = build_gate_report(
        baseline,
        corner_summary,
        ablations,
        latency,
        reference,
        ego_features,
    )
    atomic_write_json(output_path, report)
    print(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False))
    print(f"OUTPUT={output_path}")


if __name__ == "__main__":
    main()
