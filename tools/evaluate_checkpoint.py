from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np
import torch
from torch.utils.data import DataLoader


METRIC_VERSION = "transfuser_lite_diagnostic_v3"
ABLATION_SEED = 20260809


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Evaluate one TransFuser Lite checkpoint without mutating v0 artifacts."
    )
    parser.add_argument(
        "--root",
        type=Path,
        default=Path(__file__).resolve().parents[1],
    )
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--corner-spec", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--num-workers", type=int, default=4)
    parser.add_argument("--latency-runs", type=int, default=100)
    parser.add_argument("--device", choices=("auto", "cpu", "cuda"), default="auto")
    return parser.parse_args()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def resolve_path(root: Path, path: Path) -> Path:
    return path.resolve() if path.is_absolute() else (root / path).resolve()


def controller_steering(
    waypoints: np.ndarray,
    *,
    wheelbase_m: float,
    min_lookahead_m: float,
    max_steer_rad: float,
) -> np.ndarray:
    result = np.empty(len(waypoints), dtype=np.float32)
    for sample_index, points in enumerate(waypoints):
        distances = np.linalg.norm(points, axis=1)
        candidates = np.flatnonzero(distances >= min_lookahead_m)
        point = points[int(candidates[0])] if len(candidates) else points[-1]
        x, y = float(point[0]), float(point[1])
        curvature = 2.0 * y / max(x * x + y * y, 1e-6)
        result[sample_index] = np.clip(
            math.atan(wheelbase_m * curvature), -max_steer_rad, max_steer_rad
        )
    return result


def maximum_abs_curvature(waypoints: np.ndarray) -> np.ndarray:
    """Return max discrete curvature for each [N,2] future path.

    The origin is prepended because every waypoint is expressed in the
    observation ego frame. This metric describes label geometry only; the
    caller separately records whether the label came from measured pose or a
    command-derived proxy.
    """

    origin = np.zeros((len(waypoints), 1, 2), dtype=np.float32)
    points = np.concatenate((origin, waypoints.astype(np.float32, copy=False)), axis=1)
    first = points[:, :-2]
    middle = points[:, 1:-1]
    last = points[:, 2:]
    edge_a = middle - first
    edge_b = last - middle
    chord = last - first
    cross = edge_a[..., 0] * chord[..., 1] - edge_a[..., 1] * chord[..., 0]
    denominator = (
        np.linalg.norm(edge_a, axis=2)
        * np.linalg.norm(edge_b, axis=2)
        * np.linalg.norm(chord, axis=2)
    )
    curvature = np.zeros_like(denominator, dtype=np.float32)
    valid = denominator > 1e-6
    curvature[valid] = 2.0 * np.abs(cross[valid]) / denominator[valid]
    return curvature.max(axis=1)


def absolute_second_difference(waypoints: np.ndarray) -> np.ndarray:
    """Match training.losses.waypoint_smoothness, retaining per-sample values."""

    if waypoints.shape[1] < 3:
        return np.zeros(len(waypoints), dtype=np.float32)
    second = waypoints[:, 2:] - 2.0 * waypoints[:, 1:-1] + waypoints[:, :-2]
    return np.abs(second).mean(axis=(1, 2))


def forward_model(
    model: torch.nn.Module,
    model_name: str,
    image: torch.Tensor,
    lidar: torch.Tensor,
    ego: torch.Tensor,
) -> dict[str, torch.Tensor]:
    if model_name == "lidar_only":
        return model(lidar, ego)
    return model(image, lidar, ego)


def scenario_feature_indices(
    ego_features: tuple[str, ...], scenario: str
) -> tuple[int, ...]:
    if scenario == "speed_shuffle":
        names = {"speed_mps", "longitudinal_speed_mps"}
    elif scenario in {"turn_state_zero", "turn_state_shuffle"}:
        names = {"yaw_rate_rps", "steering_rad"}
    else:
        return ()
    return tuple(index for index, name in enumerate(ego_features) if name in names)


def apply_scenario(
    image: torch.Tensor,
    lidar: torch.Tensor,
    ego: torch.Tensor,
    scenario: str,
    generator: torch.Generator,
    ego_features: tuple[str, ...],
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    if scenario == "baseline":
        return image, lidar, ego

    permutation = torch.randperm(image.shape[0], generator=generator).to(image.device)
    if scenario == "image_shuffle":
        image = image[permutation]
    elif scenario == "lidar_shuffle":
        lidar = lidar[permutation]
    elif scenario == "speed_shuffle":
        indices = scenario_feature_indices(ego_features, scenario)
        if indices:
            ego = ego.clone()
            ego[:, indices] = ego[permutation][:, indices]
    elif scenario == "turn_state_zero":
        indices = scenario_feature_indices(ego_features, scenario)
        if indices:
            ego = ego.clone()
            ego[:, indices] = 0.0
    elif scenario == "turn_state_shuffle":
        indices = scenario_feature_indices(ego_features, scenario)
        if indices:
            ego = ego.clone()
            ego[:, indices] = ego[permutation][:, indices]
    else:
        raise ValueError(f"Unknown scenario: {scenario}")
    return image, lidar, ego


def collect_predictions(
    model: torch.nn.Module,
    model_name: str,
    loader: DataLoader,
    device: torch.device,
    scenario: str,
    ego_features: tuple[str, ...],
) -> dict[str, np.ndarray]:
    model.eval()
    chunks: dict[str, list[np.ndarray]] = {
        "waypoints": [],
        "target_waypoints": [],
        "target_speed": [],
        "target_target_speed": [],
    }
    optional_names = ("direct_control", "stop_logit", "mode_logits")
    generator = torch.Generator(device="cpu").manual_seed(ABLATION_SEED)
    observed_optional: set[str] | None = None

    with torch.inference_mode():
        for batch in loader:
            image = batch["image"].to(device, non_blocking=True)
            lidar = batch["lidar"].to(device, non_blocking=True)
            ego = batch["ego"].to(device, non_blocking=True)
            image, lidar, ego = apply_scenario(
                image, lidar, ego, scenario, generator, ego_features
            )
            output = forward_model(model, model_name, image, lidar, ego)
            current_optional = {name for name in optional_names if name in output}
            if observed_optional is None:
                observed_optional = current_optional
                for name in sorted(current_optional):
                    chunks[name] = []
            elif current_optional != observed_optional:
                raise RuntimeError("Model output keys changed between batches")

            chunks["waypoints"].append(output["waypoints"].detach().cpu().numpy())
            chunks["target_waypoints"].append(batch["waypoints"].numpy())
            chunks["target_speed"].append(
                output["target_speed"].detach().cpu().numpy()
            )
            chunks["target_target_speed"].append(batch["target_speed"].numpy())
            for name in sorted(current_optional):
                chunks[name].append(output[name].detach().cpu().numpy())

    return {name: np.concatenate(parts) for name, parts in chunks.items()}


def subset_summary(details: dict[str, np.ndarray], mask: np.ndarray) -> dict[str, Any]:
    count = int(mask.sum())
    if count == 0:
        return {"n": 0}
    result: dict[str, Any] = {
        "n": count,
        "ade_m": float(details["ade_m"][mask].mean()),
        "fde_m": float(details["fde_m"][mask].mean()),
        "controller_steering_mae_rad_vs_command_proxy": float(
            np.abs(details["controller_error_rad_vs_command_proxy"][mask]).mean()
        ),
        "controller_steering_bias_rad_vs_command_proxy": float(
            details["controller_error_rad_vs_command_proxy"][mask].mean()
        ),
        "target_speed_mae_mps": float(np.abs(details["speed_error_mps"][mask]).mean()),
        "prediction_absolute_second_difference_m": float(
            details["prediction_absolute_second_difference_m"][mask].mean()
        ),
        "target_absolute_second_difference_m": float(
            details["target_absolute_second_difference_m"][mask].mean()
        ),
        "absolute_second_difference_bias_m": float(
            details["absolute_second_difference_bias_m"][mask].mean()
        ),
    }
    target_second = result["target_absolute_second_difference_m"]
    result["prediction_to_target_absolute_second_difference_ratio"] = (
        None
        if target_second <= 0.0
        else result["prediction_absolute_second_difference_m"] / target_second
    )
    if "direct_head_error_rad_vs_command_proxy" in details:
        result["direct_head_mae_rad_vs_command_proxy"] = float(
            np.abs(details["direct_head_error_rad_vs_command_proxy"][mask]).mean()
        )
    return result


def summarize_predictions(
    prediction: dict[str, np.ndarray],
    frame: Any,
    *,
    wheelbase_m: float,
    min_lookahead_m: float,
    max_steer_rad: float,
) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    if "direct_steering_rad" not in frame.columns:
        raise ValueError(
            "Diagnostic v0 comparison requires direct_steering_rad; "
            "do not silently substitute zero or call it measured steering"
        )
    predicted_waypoints = prediction["waypoints"]
    target_waypoints = prediction["target_waypoints"]
    point_error = np.linalg.norm(predicted_waypoints - target_waypoints, axis=2)
    steering_proxy = frame["direct_steering_rad"].to_numpy(dtype=np.float32)
    predicted_steering = controller_steering(
        predicted_waypoints,
        wheelbase_m=wheelbase_m,
        min_lookahead_m=min_lookahead_m,
        max_steer_rad=max_steer_rad,
    )
    details: dict[str, np.ndarray] = {
        "ade_m": point_error.mean(axis=1),
        "fde_m": point_error[:, -1],
        "controller_error_rad_vs_command_proxy": predicted_steering - steering_proxy,
        "speed_error_mps": (
            prediction["target_speed"][:, 0]
            - prediction["target_target_speed"][:, 0]
        ),
        "max_abs_teacher_curvature_1pm": maximum_abs_curvature(target_waypoints),
    }
    predicted_second = absolute_second_difference(predicted_waypoints)
    target_second = absolute_second_difference(target_waypoints)
    details["prediction_absolute_second_difference_m"] = predicted_second
    details["target_absolute_second_difference_m"] = target_second
    details["absolute_second_difference_bias_m"] = predicted_second - target_second
    if "direct_control" in prediction:
        details["direct_head_error_rad_vs_command_proxy"] = (
            prediction["direct_control"][:, 0] - steering_proxy
        )

    all_samples = np.ones(len(frame), dtype=bool)
    summary = subset_summary(details, all_samples)
    summary["horizon_mae_m"] = point_error.mean(axis=0).tolist()
    summary["controller"] = {
        "wheelbase_m": wheelbase_m,
        "min_lookahead_m": min_lookahead_m,
        "max_steer_rad": max_steer_rad,
    }
    curvature = details["max_abs_teacher_curvature_1pm"]
    summary["curvature_buckets"] = {
        "straight": subset_summary(details, curvature < 0.03),
        "curve": subset_summary(details, (curvature >= 0.03) & (curvature < 0.12)),
        "sharp": subset_summary(details, curvature >= 0.12),
    }
    absolute_proxy = np.abs(steering_proxy)
    summary["steering_proxy_buckets"] = {
        "straight_abs_lt_0_05": subset_summary(details, absolute_proxy < 0.05),
        "medium_abs_0_15_to_0_30": subset_summary(
            details, (absolute_proxy >= 0.15) & (absolute_proxy < 0.30)
        ),
        "sharp_abs_ge_0_30": subset_summary(details, absolute_proxy >= 0.30),
    }
    if "stop_logit" in prediction:
        probability = 1.0 / (1.0 + np.exp(-prediction["stop_logit"][:, 0]))
        summary["optional_stop_head"] = {
            "min_probability": float(probability.min()),
            "mean_probability": float(probability.mean()),
            "max_probability": float(probability.max()),
            "above_0_6": int((probability >= 0.6).sum()),
        }
    if "mode_logits" in prediction:
        mode = prediction["mode_logits"].argmax(axis=1)
        keys, counts = np.unique(mode, return_counts=True)
        summary["optional_mode_head_histogram"] = {
            str(int(key)): int(count) for key, count in zip(keys, counts, strict=True)
        }
    return summary, details


def validate_corner_spec(
    spec: dict[str, Any], data_root: Path
) -> dict[str, dict[str, str]]:
    expected_hashes = spec["dataset_sha256"]
    actual_hashes: dict[str, str] = {}
    for relative, expected in expected_hashes.items():
        path = data_root / relative
        actual = sha256_file(path)
        actual_hashes[relative] = actual
        if actual != expected:
            raise ValueError(
                f"Dataset hash mismatch for {path}: expected {expected}, got {actual}"
            )
    windows = spec["windows"]
    for split in ("train", "val", "test"):
        if split not in windows:
            raise ValueError(f"Corner spec is missing split {split}")
        if set(windows[split]) != {"start_sample_id", "end_sample_id"}:
            raise ValueError(f"Invalid corner window for {split}: {windows[split]}")
    return {"actual": actual_hashes, "expected": expected_hashes}


def fixed_corner_summary(
    frame: Any,
    details: dict[str, np.ndarray],
    window: dict[str, str],
) -> dict[str, Any]:
    if "sample_id" not in frame.columns:
        raise ValueError("Fixed first-corner metric requires sample_id")
    sample_ids = frame["sample_id"].astype(str).to_numpy()
    start_positions = np.flatnonzero(sample_ids == window["start_sample_id"])
    end_positions = np.flatnonzero(sample_ids == window["end_sample_id"])
    if len(start_positions) != 1 or len(end_positions) != 1:
        raise ValueError(f"Fixed corner IDs are not unique/present: {window}")
    start = int(start_positions[0])
    end = int(end_positions[0])
    if start > end:
        raise ValueError(f"Fixed corner IDs are reversed: {window}")
    mask = np.zeros(len(frame), dtype=bool)
    mask[start : end + 1] = True
    return {
        "start_sample_id": window["start_sample_id"],
        "end_sample_id": window["end_sample_id"],
        **subset_summary(details, mask),
    }


def ratio(numerator: float, denominator: float) -> float | None:
    return None if denominator <= 0.0 else numerator / denominator


def benchmark_latency(
    model: torch.nn.Module,
    model_name: str,
    item: dict[str, torch.Tensor],
    device: torch.device,
    runs: int,
) -> dict[str, Any]:
    image = item["image"].unsqueeze(0).to(device)
    lidar = item["lidar"].unsqueeze(0).to(device)
    ego = item["ego"].unsqueeze(0).to(device)
    timings: list[float] = []
    model.eval()
    with torch.inference_mode():
        for _ in range(20):
            forward_model(model, model_name, image, lidar, ego)
        if device.type == "cuda":
            torch.cuda.synchronize()
        for _ in range(runs):
            start = time.perf_counter()
            forward_model(model, model_name, image, lidar, ego)
            if device.type == "cuda":
                torch.cuda.synchronize()
            timings.append((time.perf_counter() - start) * 1000.0)
    return {
        "runs": runs,
        "p50_ms": float(np.quantile(timings, 0.50)),
        "p95_ms": float(np.quantile(timings, 0.95)),
        "max_ms": float(np.max(timings)),
    }


def choose_device(requested: str) -> torch.device:
    if requested == "auto":
        return torch.device("cuda" if torch.cuda.is_available() else "cpu")
    if requested == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("--device cuda requested but CUDA is unavailable")
    return torch.device(requested)


def main() -> None:
    args = parse_args()
    root = args.root.resolve()
    checkpoint_path = resolve_path(root, args.checkpoint)
    corner_spec_path = resolve_path(root, args.corner_spec)
    output_path = resolve_path(root, args.output)
    sys.path.insert(0, str(root / "src"))

    from aic_transfuser_lite.data.dataset import DrivingDataset
    from aic_transfuser_lite.data.ego_features import configured_ego_features
    from aic_transfuser_lite.models.factory import build_model

    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
    if not isinstance(checkpoint, dict) or "model" not in checkpoint or "config" not in checkpoint:
        raise ValueError("Checkpoint must contain model and config")
    config = checkpoint["config"]
    ego_features = configured_ego_features(config["data"])
    model_name = str(config["model"]["name"])
    data_root = root / "datasets/processed/aic_real_dataset"
    corner_spec = json.loads(corner_spec_path.read_text(encoding="utf-8"))
    dataset_hashes = validate_corner_spec(corner_spec, data_root)

    datasets = {
        split: DrivingDataset(data_root / f"{split}_index.csv", config)
        for split in ("train", "val", "test")
    }
    loaders = {
        split: DataLoader(
            dataset,
            batch_size=args.batch_size,
            shuffle=False,
            num_workers=args.num_workers,
            pin_memory=True,
        )
        for split, dataset in datasets.items()
    }
    device = choose_device(args.device)
    model = build_model(config).to(device)
    model.load_state_dict(checkpoint["model"], strict=True)
    model.eval()

    runtime_config = corner_spec["controller"]
    wheelbase_m = float(runtime_config["wheelbase_m"])
    min_lookahead_m = float(runtime_config["min_lookahead_m"])
    max_steer_rad = float(runtime_config["max_steer_rad"])
    baseline: dict[str, Any] = {}
    baseline_details: dict[str, dict[str, np.ndarray]] = {}
    for split in ("train", "val", "test"):
        prediction = collect_predictions(
            model, model_name, loaders[split], device, "baseline", ego_features
        )
        summary, details = summarize_predictions(
            prediction,
            datasets[split].frame,
            wheelbase_m=wheelbase_m,
            min_lookahead_m=min_lookahead_m,
            max_steer_rad=max_steer_rad,
        )
        summary["fixed_first_corner"] = fixed_corner_summary(
            datasets[split].frame,
            details,
            corner_spec["windows"][split],
        )
        baseline[split] = summary
        baseline_details[split] = details

    ablations: dict[str, Any] = {}
    baseline_test = baseline["test"]
    for scenario in (
        "image_shuffle",
        "lidar_shuffle",
        "speed_shuffle",
        "turn_state_zero",
        "turn_state_shuffle",
    ):
        prediction = collect_predictions(
            model, model_name, loaders["test"], device, scenario, ego_features
        )
        summary, _ = summarize_predictions(
            prediction,
            datasets["test"].frame,
            wheelbase_m=wheelbase_m,
            min_lookahead_m=min_lookahead_m,
            max_steer_rad=max_steer_rad,
        )
        sharp_key = "sharp_abs_ge_0_30"
        baseline_sharp_proxy = baseline_test["steering_proxy_buckets"][sharp_key]
        scenario_sharp_proxy = summary["steering_proxy_buckets"][sharp_key]
        baseline_sharp_curvature = baseline_test["curvature_buckets"]["sharp"]
        scenario_sharp_curvature = summary["curvature_buckets"]["sharp"]
        summary["sharp_control_mae_ratio_vs_baseline"] = {
            "steering_command_proxy_bucket": ratio(
                scenario_sharp_proxy[
                    "controller_steering_mae_rad_vs_command_proxy"
                ],
                baseline_sharp_proxy[
                    "controller_steering_mae_rad_vs_command_proxy"
                ],
            ),
            "teacher_curvature_bucket": ratio(
                scenario_sharp_curvature[
                    "controller_steering_mae_rad_vs_command_proxy"
                ],
                baseline_sharp_curvature[
                    "controller_steering_mae_rad_vs_command_proxy"
                ],
            ),
        }
        affected_indices = scenario_feature_indices(ego_features, scenario)
        ego_scenario = scenario in {
            "speed_shuffle",
            "turn_state_zero",
            "turn_state_shuffle",
        }
        summary["ablation_contract"] = {
            "effective": bool(affected_indices) if ego_scenario else True,
            "affected_ego_features": [ego_features[index] for index in affected_indices],
            "structurally_absent": bool(ego_scenario and not affected_indices),
        }
        ablations[scenario] = summary

    use_valid_mask = bool(
        config.get("model", {}).get("lidar", {}).get("use_valid_mask", False)
    )
    ablations["lidar_invalid_mask_all_zero"] = {
        "status": "PENDING_MODEL_V1" if not use_valid_mask else "UNSUPPORTED_BY_V0_EVALUATOR",
        "reason": (
            "Current checkpoint input contract does not consume lidar_valid; "
            "fabricating an effect would be a no-op."
        ),
    }

    gpu_name = None
    if device.type == "cuda":
        gpu_name = torch.cuda.get_device_name(device)
    report: dict[str, Any] = {
        "metric_version": METRIC_VERSION,
        "gate_eligibility": "DIAGNOSTIC_ONLY_V0_COMMAND_PROXY_NOT_FINAL_DATASET_V2",
        "reference_provenance": {
            "controller_steering_reference": "direct_steering_rad command-derived v0 teacher proxy",
            "teacher_waypoints": "v0 command-integrated proxy, not measured future pose",
            "warning": "Do not label these references as actual/measured steering or pose.",
        },
        "artifact": {
            "checkpoint": str(checkpoint_path),
            "checkpoint_sha256": sha256_file(checkpoint_path),
            "checkpoint_epoch": int(checkpoint.get("epoch", -1)),
            "evaluator": str(Path(__file__).resolve()),
            "evaluator_sha256": sha256_file(Path(__file__).resolve()),
            "corner_spec": str(corner_spec_path),
            "corner_spec_sha256": sha256_file(corner_spec_path),
            "dataset_sha256": dataset_hashes,
        },
        "environment": {
            "device": str(device),
            "gpu": gpu_name,
            "torch_version": torch.__version__,
        },
        "model": {
            "name": model_name,
            "parameters": int(sum(parameter.numel() for parameter in model.parameters())),
            "trainable_parameters": int(
                sum(parameter.numel() for parameter in model.parameters() if parameter.requires_grad)
            ),
            "output_heads": {
                name: bool(config.get("model", {}).get("heads", {}).get(name, False))
                for name in ("waypoints", "target_speed", "stop", "behavior_mode", "direct_control_aux")
            },
            "ego_features": list(ego_features),
        },
        "checkpoint_config": config,
        "split_rows": {split: len(dataset) for split, dataset in datasets.items()},
        "baseline": baseline,
        "test_ablations": ablations,
        "batch1_latency": benchmark_latency(
            model,
            model_name,
            datasets["test"][0],
            device,
            args.latency_runs,
        ),
    }

    text = json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(text + "\n", encoding="utf-8")
    print(text)
    print(f"OUTPUT={output_path}")


if __name__ == "__main__":
    main()
