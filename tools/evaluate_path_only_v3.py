#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys
import time
from typing import Any

import numpy as np
import torch
import yaml

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from aic_transfuser_lite.contracts.behavior_v1 import BEHAVIOR_CLASS_NAMES_V1
from aic_transfuser_lite.data.dataset_view_v3 import (
    ControlTargetBoundsV3,
    MotionTargetFilterConfigV3,
    assess_commanded_motion_target_v3,
    load_temporal_training_batches_v3,
)
from aic_transfuser_lite.data.storage_v3 import validate_complete_dataset
from aic_transfuser_lite.evaluation.launch_replay_v3 import (
    load_path_only_replay_config_v3,
    replay_path_only_launch_v3,
)
from aic_transfuser_lite.control.executable_reference import (
    estimate_polyline_curvature_per_m,
    polyline_arc_length_m,
)
from aic_transfuser_lite.training.train_v3 import (
    build_full_control_model_v3,
    launch_readiness_gate_config_v3,
    load_full_control_config_v3,
    motion_target_filter_config_v3,
    move_batch_v3,
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _control_bounds(config: dict[str, Any]) -> ControlTargetBoundsV3:
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


def _loader_arguments(
    config: dict[str, Any],
    view: dict[str, Any],
    behavior_view: Path,
    *,
    batch_size: int,
    motion_filter: MotionTargetFilterConfigV3,
) -> dict[str, Any]:
    model = config["model"]
    data = config["data"]
    return {
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
        "control_target_bounds": _control_bounds(config),
        "batch_size": batch_size,
        "behavior_view_root": behavior_view,
        "motion_target_filter": motion_filter,
    }


def _phase_labels(path: Path | None) -> dict[str, dict[str, str]]:
    if path is None:
        return {}
    manifest = json.loads((path / "manifest.json").read_text(encoding="utf-8"))
    labels = path / "phase_labels.csv"
    if _sha256(labels) != manifest.get("labels_sha256"):
        raise ValueError("phase view labels SHA-256 mismatch")
    with labels.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    result = {row["sample_id"]: row for row in rows}
    if len(result) != len(rows):
        raise ValueError("phase view contains duplicate sample IDs")
    return result


def _path_geometry(
    path: np.ndarray, mask: np.ndarray, *, minimum_arc_length_m: float = 0.05
) -> tuple[float | None, float | None, float]:
    """Return stable terminal heading/max curvature with the ego origin included."""

    selected = np.asarray(path, dtype=np.float64)[mask]
    _, cumulative = polyline_arc_length_m(selected)
    arc_length = float(cumulative[-1])
    if len(selected) < 2 or arc_length < minimum_arc_length_m:
        return None, None, arc_length
    points = np.concatenate((np.zeros((1, 2), dtype=np.float64), selected), axis=0)
    heading = None
    for index in range(len(points) - 1, 0, -1):
        delta = points[index] - points[index - 1]
        if float(np.linalg.norm(delta)) > 1e-6:
            heading = float(math.atan2(delta[1], delta[0]))
            break
    curvature = estimate_polyline_curvature_per_m(selected)
    maximum_curvature = (
        None if not np.isfinite(curvature).all() else float(np.max(np.abs(curvature)))
    )
    return heading, maximum_curvature, arc_length


def _angle_error(left: float | None, right: float | None) -> float | None:
    if left is None or right is None:
        return None
    return abs(math.atan2(math.sin(left - right), math.cos(left - right)))


def _percentile(values: list[float], q: float) -> float | None:
    return None if not values else float(np.percentile(values, q))


def _aggregate(rows: list[dict[str, Any]], *, quality_only: bool) -> dict[str, Any]:
    selected = [row for row in rows if not quality_only or row["teacher_quality"]]
    if not selected:
        raise ValueError("evaluation cohort is empty")
    error_sum = sum(float(row["trajectory_error_sum_m"]) for row in selected)
    valid_count = sum(int(row["trajectory_valid_waypoints"]) for row in selected)
    run_error_sum: dict[str, float] = {}
    run_valid_count: dict[str, int] = {}
    for row in selected:
        run_id = str(row["run_id"])
        run_error_sum[run_id] = run_error_sum.get(run_id, 0.0) + float(
            row["trajectory_error_sum_m"]
        )
        run_valid_count[run_id] = run_valid_count.get(run_id, 0) + int(
            row["trajectory_valid_waypoints"]
        )
    run_mean = {key: run_error_sum[key] / run_valid_count[key] for key in sorted(run_error_sum)}
    run_detail = {
        run_id: {
            "sample_count": len(values),
            "waypoint_weighted_ade_m": run_mean[run_id],
            "frame_p90_ade_m": float(np.percentile(values, 90.0)),
            "frame_p95_ade_m": float(np.percentile(values, 95.0)),
        }
        for run_id in sorted(run_mean)
        for values in [[float(row["ade_m"]) for row in selected if row["run_id"] == run_id]]
    }
    ades = [float(row["ade_m"]) for row in selected]
    fdes = [float(row["fde_m"]) for row in selected]
    speeds = [float(row["speed_mae_mps"]) for row in selected]
    headings = [
        float(row["heading_error_rad"])
        for row in selected
        if row["heading_error_rad"] is not None
    ]
    curvatures = [
        float(row["curvature_error_per_m"])
        for row in selected
        if row["curvature_error_per_m"] is not None
    ]

    def slices(field: str) -> dict[str, dict[str, float]]:
        result: dict[str, dict[str, float]] = {}
        for value in sorted({str(row[field]) for row in selected}):
            subset = [row for row in selected if str(row[field]) == value]
            result[value] = {
                "sample_count": float(len(subset)),
                "ade_m": float(np.mean([float(row["ade_m"]) for row in subset])),
                "fde_m": float(np.mean([float(row["fde_m"]) for row in subset])),
            }
        return result

    return {
        "sample_count": len(selected),
        "trajectory_valid_waypoints": valid_count,
        "trajectory_waypoint_weighted_ade_m": error_sum / valid_count,
        "trajectory_frame_weighted_ade_m": float(np.mean(ades)),
        "trajectory_run_equal_ade_m": float(np.mean(tuple(run_mean.values()))),
        "trajectory_worst_run_ade_m": float(max(run_mean.values())),
        "trajectory_p90_ade_m": _percentile(ades, 90.0),
        "trajectory_p95_ade_m": _percentile(ades, 95.0),
        "fde_m": float(np.mean(fdes)),
        "speed_mae_mps": float(np.mean(speeds)),
        "heading_error_rad": None if not headings else float(np.mean(headings)),
        "curvature_error_per_m": (
            None if not curvatures else float(np.mean(curvatures))
        ),
        "run_ade_m": run_mean,
        "run_detail": run_detail,
        "slices": {
            field: slices(field)
            for field in ("phase", "geometry", "side", "behavior", "motion_assessment")
        },
    }


def _launch_summary(rows: list[dict[str, Any]], *, episode_gap_sec: float) -> dict[str, Any]:
    launch = [row for row in rows if row["launch_candidate"]]
    ready = [row for row in launch if row["launch_ready"]]
    grouped: dict[tuple[str, str], list[int]] = {}
    for row in launch:
        grouped.setdefault((row["run_id"], row["segment_id"]), []).append(
            int(row["grid_stamp_ns"])
        )
    gap_ns = int(round(episode_gap_sec * 1_000_000_000.0))
    episodes = 0
    for stamps in grouped.values():
        stamps.sort()
        episodes += int(bool(stamps)) + sum(
            right - left > gap_ns for left, right in zip(stamps, stamps[1:])
        )
    reasons: dict[str, int] = {}
    transformations: dict[str, int] = {}
    for row in launch:
        for reason in str(row["launch_reasons"]).split("|"):
            if reason:
                reasons[reason] = reasons.get(reason, 0) + 1
        for transformation in str(row["launch_transformations"]).split("|"):
            if transformation:
                transformations[transformation] = (
                    transformations.get(transformation, 0) + 1
                )
    def numeric_summary(field: str) -> dict[str, float | None]:
        values = [float(row[field]) for row in launch if row[field] is not None]
        return {
            "mean": None if not values else float(np.mean(values)),
            "p05": None if not values else float(np.percentile(values, 5.0)),
            "p50": None if not values else float(np.percentile(values, 50.0)),
            "p95": None if not values else float(np.percentile(values, 95.0)),
        }
    return {
        "anchor_count": len(launch),
        "ready_count": len(ready),
        "ready_fraction": len(ready) / len(launch) if launch else 0.0,
        "unknown_count": sum(
            row["motion_assessment"] == "censored_future" for row in launch
        ),
        "coverage_of_unfiltered_valid": len(launch) / len(rows) if rows else 0.0,
        "run_count": len({row["run_id"] for row in launch}),
        "run_distribution": {
            run_id: sum(row["run_id"] == run_id for row in launch)
            for run_id in sorted({row["run_id"] for row in launch})
        },
        "episode_count_estimate": episodes,
        "episode_gap_sec": episode_gap_sec,
        "motion_assessment_counts": {
            value: sum(row["motion_assessment"] == value for row in launch)
            for value in sorted({row["motion_assessment"] for row in launch})
        },
        "reference_rejection_reasons": dict(sorted(reasons.items())),
        "reference_transformation_counts": dict(sorted(transformations.items())),
        "reference_rejection_fraction": (
            sum(not bool(row["launch_reference_accepted"]) for row in launch)
            / len(launch)
            if launch
            else 0.0
        ),
        "initial_waypoint_forward_fraction": (
            sum(bool(row["predicted_initial_forward"]) for row in launch)
            / len(launch)
            if launch
            else 0.0
        ),
        "trimmed_fraction": (
            sum(int(row["trim_count"]) > 0 for row in launch) / len(launch)
            if launch
            else 0.0
        ),
        "max_x_only_false_positive_count": sum(
            bool(row["launch_max_x_only_false_positive"]) for row in launch
        ),
        "path_length_m": numeric_summary("predicted_path_length_m"),
        "endpoint_forward_m": numeric_summary("predicted_endpoint_forward_m"),
        "endpoint_displacement_m": numeric_summary(
            "predicted_endpoint_displacement_m"
        ),
        "maximum_forward_m": numeric_summary("predicted_maximum_forward_m"),
        "maximum_abs_curvature_per_m": numeric_summary(
            "maximum_abs_curvature_per_m"
        ),
        "endpoint_heading_rad": numeric_summary("predicted_endpoint_heading_rad"),
        "lookahead_distance_m": numeric_summary("lookahead_distance_m"),
        "controller_requested_speed_mps": numeric_summary(
            "controller_requested_speed_mps"
        ),
        "stop_probability_connected": False,
        "closed_loop_launch_proven": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--runtime-config", type=Path, required=True)
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--split-manifest", type=Path, required=True)
    parser.add_argument("--view-config", type=Path, required=True)
    parser.add_argument("--behavior-view", type=Path, required=True)
    parser.add_argument("--phase-view", type=Path)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--split", default="validation", choices=("train", "validation", "test"))
    parser.add_argument("--batch-size", type=int, default=2)
    parser.add_argument("--device", default="auto")
    args = parser.parse_args()
    wall_start = time.monotonic()
    started_at_utc = datetime.now(timezone.utc).isoformat()
    if args.output.exists():
        raise FileExistsError(f"evaluation output already exists: {args.output}")
    args.output.mkdir(parents=True)

    config = load_full_control_config_v3(args.config)
    view = yaml.safe_load(args.view_config.read_text(encoding="utf-8"))
    expected_view_history = {
        "camera_history_length": int(config["data"]["image_history_length"]),
        "lidar_history_length": int(config["data"]["lidar_history_length"]),
        "ego_history_length": int(config["data"]["ego_history_length"]),
        "command_history_length": int(config["data"]["command_history_length"]),
    }
    if any(
        int(view.get(name, 0)) != value
        for name, value in expected_view_history.items()
    ):
        raise ValueError("evaluation view and model/data history contracts differ")
    if view.get("command_history_alignment") != config["targets"][
        "command_history_alignment"
    ]:
        raise ValueError("evaluation command-history alignment differs")
    dataset_manifest = validate_complete_dataset(args.dataset_root)
    split_manifest = json.loads(args.split_manifest.read_text(encoding="utf-8"))
    if split_manifest.get("dataset_manifest_sha256") != dataset_manifest["manifest_sha256"]:
        raise ValueError("split manifest and Dataset V3 identity differ")
    assignments = split_manifest.get("assignments")
    if not isinstance(assignments, list) or not assignments:
        raise ValueError("split manifest assignments are missing")
    assignment_run_ids = [str(row["run_id"]) for row in assignments]
    if len(assignment_run_ids) != len(set(assignment_run_ids)):
        raise ValueError("split manifest contains duplicate run IDs")
    leakage = split_manifest.get("leakage")
    if not isinstance(leakage, dict) or leakage.get("status") != "PASS":
        raise ValueError("split manifest leakage status is not PASS")
    quality_filter = motion_target_filter_config_v3(config)
    filtered = load_temporal_training_batches_v3(
        args.dataset_root,
        args.split_manifest,
        split=args.split,
        **_loader_arguments(
            config,
            view,
            args.behavior_view,
            batch_size=args.batch_size,
            motion_filter=quality_filter,
        ),
    )
    unfiltered = load_temporal_training_batches_v3(
        args.dataset_root,
        args.split_manifest,
        split=args.split,
        **_loader_arguments(
            config,
            view,
            args.behavior_view,
            batch_size=args.batch_size,
            motion_filter=MotionTargetFilterConfigV3(enabled=False),
        ),
    )
    quality_ids = {
        item.sample_id
        for batch_index in range(len(filtered))
        for item in filtered.metadata_for_batch(batch_index)
    }
    phases = _phase_labels(args.phase_view)

    requested = args.device
    if requested == "auto":
        requested = "cuda" if torch.cuda.is_available() else "cpu"
    device = torch.device(requested)
    model = build_full_control_model_v3(config).to(device)
    checkpoint = torch.load(args.checkpoint, map_location="cpu", weights_only=True)
    if not isinstance(checkpoint, dict) or not isinstance(checkpoint.get("model"), dict):
        raise ValueError("checkpoint has no model state mapping")
    migration = model.migrate_v1_weights(checkpoint["model"])
    model_parameters = dict(model.named_parameters())
    model_buffers = dict(model.named_buffers())
    loaded_names = set(migration.loaded)
    coverage = {
        "loaded_keys": len(migration.loaded),
        "shape_mismatch": list(migration.shape_mismatch),
        "unmapped_source": list(migration.unmapped_v1),
        "missing_target": list(migration.new_v3),
        "loaded_parameter_numel": sum(
            value.numel() for name, value in model_parameters.items() if name in loaded_names
        ),
        "total_parameter_numel": sum(value.numel() for value in model_parameters.values()),
        "loaded_buffer_numel": sum(
            value.numel() for name, value in model_buffers.items() if name in loaded_names
        ),
        "total_buffer_numel": sum(value.numel() for value in model_buffers.values()),
    }
    if migration.shape_mismatch or migration.unmapped_v1 or migration.new_v3:
        raise ValueError("checkpoint is not an exact V3 model match")
    model.eval()
    gate = launch_readiness_gate_config_v3(config)
    if gate is None:
        raise ValueError("path-only evaluation requires an enabled launch gate")
    replay_config = load_path_only_replay_config_v3(
        args.runtime_config,
        trajectory_steps=int(config["model"]["trajectory_steps"]),
        minimum_endpoint_forward_m=gate.minimum_forward_progress_m,
    )

    rows: list[dict[str, Any]] = []
    with torch.inference_mode():
        for batch_index, source_batch in enumerate(unfiltered):
            metadata = unfiltered.metadata_for_batch(batch_index)
            batch = move_batch_v3(source_batch, device)
            assert batch.targets is not None
            output = model(batch)
            predicted_xy = output.trajectory_xy[:, 0].detach().cpu().numpy()
            predicted_speed = output.trajectory_speed_mps[:, 0].detach().cpu().numpy()
            target_xy = batch.targets.trajectory_xy_m.detach().cpu().numpy()
            target_speed = batch.targets.speed_mps.detach().cpu().numpy()
            target_mask = batch.targets.trajectory_mask.detach().cpu().numpy()
            speed_mask = batch.targets.speed_mask.detach().cpu().numpy()
            control = batch.targets.current_control.detach().cpu().numpy()
            control_mask = batch.targets.current_control_mask.detach().cpu().numpy()
            ego = batch.ego[:, -1].detach().cpu().numpy()
            behavior = batch.targets.behavior_class.detach().cpu().numpy()
            behavior_mask = batch.targets.behavior_mask.detach().cpu().numpy()
            for index, identity in enumerate(metadata):
                mask = target_mask[index].astype(bool)
                error = np.linalg.norm(predicted_xy[index] - target_xy[index], axis=1)
                valid_error = error[mask]
                if len(valid_error) == 0:
                    raise ValueError("unfiltered evaluation contains zero-valid future")
                valid_speed = speed_mask[index].astype(bool)
                speed_error = np.abs(
                    predicted_speed[index] - target_speed[index]
                )[valid_speed]
                if len(speed_error) == 0:
                    raise ValueError("unfiltered evaluation contains zero-valid speed")
                future = np.zeros((len(mask), 8), dtype=np.float64)
                future[:, 1:3] = target_xy[index]
                future[:, 4] = target_speed[index]
                future[:, 7] = mask
                assessment = assess_commanded_motion_target_v3(
                    future,
                    current_speed_mps=float(ego[index, 0]),
                    commanded_speed_mps=float(control[index, 1]),
                    config=quality_filter,
                )
                launch_candidate = bool(
                    abs(float(ego[index, 0])) <= gate.current_speed_max_mps
                    and bool(control_mask[index, 1])
                    and float(control[index, 1]) >= gate.commanded_speed_min_mps
                )
                replay = None
                if launch_candidate:
                    replay = replay_path_only_launch_v3(
                        predicted_xy[index],
                        predicted_speed[index],
                        current_speed_mps=float(ego[index, 0]),
                        yaw_rate_rps=float(ego[index, 2]),
                        actual_steering_rad=float(ego[index, 3]),
                        config=replay_config,
                    )
                phase = phases.get(identity.sample_id, {})
                pred_heading, pred_curvature, pred_arc_length = _path_geometry(
                    predicted_xy[index], mask
                )
                target_heading, target_curvature, target_arc_length = _path_geometry(
                    target_xy[index], mask
                )
                rows.append(
                    {
                        "sample_id": identity.sample_id,
                        "run_id": identity.run_id,
                        "segment_id": identity.segment_id,
                        "grid_stamp_ns": identity.grid_stamp_ns,
                        "teacher_quality": identity.sample_id in quality_ids,
                        "motion_assessment": assessment.value,
                        "future_valid_waypoints": int(mask.sum()),
                        "trajectory_error_sum_m": float(valid_error.sum()),
                        "trajectory_valid_waypoints": len(valid_error),
                        "ade_m": float(valid_error.mean()),
                        "fde_m": float(valid_error[-1]),
                        "speed_mae_mps": float(speed_error.mean()),
                        "heading_error_rad": _angle_error(pred_heading, target_heading),
                        "curvature_error_per_m": (
                            None
                            if pred_curvature is None or target_curvature is None
                            else abs(pred_curvature - target_curvature)
                        ),
                        "prediction_valid_arc_length_m": pred_arc_length,
                        "target_valid_arc_length_m": target_arc_length,
                        "phase": phase.get("phase", "unknown"),
                        "geometry": phase.get("geometry", "unknown"),
                        "side": phase.get("side", "unknown"),
                        "behavior": (
                            BEHAVIOR_CLASS_NAMES_V1[int(behavior[index])]
                            if behavior_mask[index]
                            else "unknown"
                        ),
                        "launch_candidate": launch_candidate,
                        "launch_ready": False if replay is None else replay.ready,
                        "launch_reference_accepted": (
                            False if replay is None else replay.reference_accepted
                        ),
                        "launch_reasons": (
                            "" if replay is None else "|".join(replay.reasons)
                        ),
                        "launch_transformations": (
                            "" if replay is None else "|".join(replay.transformations)
                        ),
                        "launch_max_x_only_false_positive": bool(
                            replay is not None
                            and replay.maximum_forward_m
                            >= gate.minimum_forward_progress_m
                            and not replay.ready
                        ),
                        "predicted_path_length_m": (
                            None if replay is None else replay.path_length_m
                        ),
                        "predicted_initial_forward_m": (
                            None if replay is None else replay.initial_forward_m
                        ),
                        "predicted_initial_forward": bool(
                            replay is not None
                            and replay.initial_forward_m
                            > replay_config.reference.minimum_initial_forward_m
                        ),
                        "predicted_endpoint_forward_m": (
                            None if replay is None else replay.endpoint_forward_m
                        ),
                        "predicted_maximum_forward_m": (
                            None if replay is None else replay.maximum_forward_m
                        ),
                        "predicted_endpoint_displacement_m": (
                            None if replay is None else replay.endpoint_displacement_m
                        ),
                        "trim_count": None if replay is None else replay.trim_count,
                        "maximum_abs_curvature_per_m": (
                            None if replay is None else replay.maximum_abs_curvature_per_m
                        ),
                        "predicted_endpoint_heading_rad": (
                            None if replay is None else replay.endpoint_heading_rad
                        ),
                        "lookahead_distance_m": (
                            None if replay is None else replay.lookahead_distance_m
                        ),
                        "controller_requested_speed_mps": (
                            None if replay is None else replay.controller_requested_speed_mps
                        ),
                        "controller_acceleration_mps2": (
                            None if replay is None else replay.controller_acceleration_mps2
                        ),
                        "controller_state": (
                            None if replay is None else replay.controller_state
                        ),
                    }
                )

    per_sample = args.output / "per_sample.csv"
    with per_sample.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    quality_summary = _aggregate(rows, quality_only=True)
    unfiltered_summary = _aggregate(rows, quality_only=False)
    launch_summary = _launch_summary(rows, episode_gap_sec=gate.episode_gap_sec)
    gate_pass = bool(
        launch_summary["anchor_count"] >= gate.minimum_samples
        and launch_summary["ready_fraction"] >= gate.minimum_ready_fraction
        and launch_summary["run_count"] >= gate.minimum_runs
        and launch_summary["episode_count_estimate"] >= gate.minimum_episodes
    )
    cohort_payload = {
        "split": args.split,
        "sample_ids": [row["sample_id"] for row in rows],
        "teacher_quality_ids": [row["sample_id"] for row in rows if row["teacher_quality"]],
        "future_valid_counts": [row["future_valid_waypoints"] for row in rows],
        "config_sha256": _sha256(args.config),
        "runtime_config_sha256": _sha256(args.runtime_config),
        "split_manifest_sha256": _sha256(args.split_manifest),
        "split_leakage": leakage,
    }
    cohort_text = json.dumps(cohort_payload, sort_keys=True, separators=(",", ":"))
    cohort_identity = hashlib.sha256(cohort_text.encode("utf-8")).hexdigest()
    summary = {
        "format": "aic_path_only_evaluation_v3",
        "started_at_utc": started_at_utc,
        "completed_at_utc": datetime.now(timezone.utc).isoformat(),
        "wall_time_sec": time.monotonic() - wall_start,
        "dataset_manifest_sha256": dataset_manifest["manifest_sha256"],
        "split_manifest_sha256": _sha256(args.split_manifest),
        "checkpoint_sha256": _sha256(args.checkpoint),
        "checkpoint_coverage": coverage,
        "sample_accounting": {
            "selected_split_rows": len(unfiltered.rows),
            "base_exclusions": dict(unfiltered.base_exclusion_counts),
            "unfiltered_valid": len(unfiltered.usable_anchors),
            "teacher_quality": len(filtered.usable_anchors),
            "motion_target_rejected": filtered.motion_target_rejected_count,
            "motion_target_censored": filtered.motion_target_censored_count,
            "motion_target_censored_stationary_prefix": (
                filtered.motion_target_censored_stationary_prefix_count
            ),
            "motion_target_candidates": filtered.motion_target_candidate_count,
            "motion_target_observed": filtered.motion_target_observed_count,
        },
        "cohort_identity_sha256": cohort_identity,
        "teacher_quality": quality_summary,
        "unfiltered_valid": unfiltered_summary,
        "stopped_commanded_motion": launch_summary,
        "screening_gate_pass": gate_pass,
        "screening_gate": gate.__dict__,
        "limitations": [
            "offline replay does not prove measured or closed-loop launch",
            "stop probability is not connected in the selected runtime profile",
            "phase labels are unavailable for normal-lap samples",
            "launch episodes are estimated by timestamp gaps",
            "test split must not be used for checkpoint selection",
        ],
    }
    (args.output / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    (args.output / "cohort_identity.json").write_text(
        json.dumps(
            {**cohort_payload, "cohort_identity_sha256": cohort_identity},
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    print(json.dumps({"status": "COMPLETE", "output": str(args.output), "gate_pass": gate_pass}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
