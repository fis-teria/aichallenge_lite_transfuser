from __future__ import annotations

from dataclasses import dataclass
from collections import defaultdict
from pathlib import Path
from typing import Sequence
import math

import numpy as np
import torch
import yaml

from aic_transfuser_lite.contracts.behavior_v1 import (
    BEHAVIOR_CLASS_NAMES_V1,
    BEHAVIOR_ONTOLOGY_V1,
    BEHAVIOR_SIDE_NAMES_V1,
)
from aic_transfuser_lite.contracts.model_batch_v3 import (
    COMMAND_HISTORY_ALIGNMENT_V3,
    CONTROL_SEQUENCE_ALIGNMENT_V3,
    ModelBatchV3,
)
from aic_transfuser_lite.data.dataset_view_v3 import MotionTargetFilterConfigV3
from aic_transfuser_lite.evaluation.launch_replay_v3 import (
    load_path_only_replay_config_v3,
    replay_path_only_launch_v3,
)
from aic_transfuser_lite.models.full_control_lite_v3 import FullControlLiteV3

from .checkpoint_v3 import (
    ExperimentIdentityV3,
    load_checkpoint_v3,
    save_checkpoint_v3,
)
from .losses_v3 import LossWeightsV3, compute_losses_v3
from .sampler_v3 import DeterministicSamplerV3


def balanced_class_weights_v3(
    batches: Sequence[ModelBatchV3], *, target_name: str, mask_name: str, class_count: int,
    require_all_classes: bool = True,
) -> tuple[float, ...]:
    """Derive inverse-frequency weights from the selected training split only."""

    counter = getattr(batches, "class_counts", None)
    if callable(counter):
        counts = counter(target_name, class_count)
    else:
        counts = torch.zeros(class_count, dtype=torch.long)
        for batch in batches:
            if batch.targets is None:
                continue
            target = getattr(batch.targets, target_name)
            mask = getattr(batch.targets, mask_name)
            if target is None or mask is None:
                continue
            selected = target[mask].detach().cpu()
            counts += torch.bincount(selected, minlength=class_count)[:class_count]
    missing = torch.nonzero(counts == 0, as_tuple=False).flatten().tolist()
    if missing and require_all_classes:
        raise ValueError(f"training split has no valid {target_name} samples for classes {missing}")
    total = float(counts.sum())
    divisor = counts.to(torch.float64)
    values = torch.where(
        divisor > 0, total / (float(class_count) * divisor), torch.ones_like(divisor)
    )
    return tuple(float(value) for value in values)


class TrainerV3:
    """Small deterministic trainer whose pause boundary is one optimizer step."""

    def __init__(
        self,
        *,
        model: torch.nn.Module,
        batches: Sequence[ModelBatchV3],
        optimizer: torch.optim.Optimizer,
        identity: ExperimentIdentityV3,
        loss_weights: LossWeightsV3 = LossWeightsV3(),
        gradient_accumulation_steps: int = 1,
        scheduler: object | None = None,
    ) -> None:
        if not batches:
            raise ValueError("TrainerV3 requires at least one batch")
        self.model = model
        self.batches = batches
        self.optimizer = optimizer
        self.identity = identity
        self.loss_weights = loss_weights
        if gradient_accumulation_steps <= 0:
            raise ValueError("gradient_accumulation_steps must be positive")
        self.gradient_accumulation_steps = gradient_accumulation_steps
        self.scheduler = scheduler
        first_parameter = next(iter(model.parameters()), None)
        self.device = torch.device("cpu") if first_parameter is None else first_parameter.device
        self.sampler = DeterministicSamplerV3(len(batches), seed=identity.seed)
        self.global_step = 0
        self.logs: list[dict[str, float]] = []

    def train_steps(
        self,
        count: int,
        *,
        micro_batches_per_optimizer_step: int | None = None,
    ) -> list[dict[str, float]]:
        if count < 0:
            raise ValueError("training step count must be non-negative")
        micro_batch_count = (
            self.gradient_accumulation_steps
            if micro_batches_per_optimizer_step is None
            else micro_batches_per_optimizer_step
        )
        if not 0 < micro_batch_count <= self.gradient_accumulation_steps:
            raise ValueError(
                "micro_batches_per_optimizer_step must be within gradient accumulation"
            )
        if bool(getattr(self.model, "freeze_except_control_sequence", False)):
            self.model.eval()
            sequence_head = getattr(self.model, "control_sequence_head", None)
            if sequence_head is None:
                raise ValueError("frozen-backbone training requires a control sequence head")
            sequence_head.train()
        else:
            self.model.train()
        produced: list[dict[str, float]] = []
        for _ in range(count):
            self.optimizer.zero_grad(set_to_none=True)
            accumulated: dict[str, float] = {}
            for _micro_step in range(micro_batch_count):
                batch = move_batch_v3(
                    self.batches[self.sampler.next_index()], self.device
                )
                if batch.targets is None:
                    raise ValueError("training batch is missing targets")
                output = self.model(batch)
                report = compute_losses_v3(output, batch.targets, self.loss_weights)
                if not torch.isfinite(report.total):
                    raise FloatingPointError("non-finite V3 loss before backward")
                (report.total / micro_batch_count).backward()
                micro_log = report.scalar_log()
                if (
                    output.behavior_logits is not None
                    and batch.targets.behavior_class is not None
                ):
                    _add_masked_accuracy(
                        micro_log,
                        "behavior",
                        output.behavior_logits,
                        batch.targets.behavior_class,
                        batch.targets.behavior_mask,
                    )
                if (
                    output.behavior_side_logits is not None
                    and batch.targets.behavior_side is not None
                ):
                    _add_masked_accuracy(
                        micro_log,
                        "behavior_side",
                        output.behavior_side_logits,
                        batch.targets.behavior_side,
                        batch.targets.behavior_side_mask,
                    )
                for key, value in micro_log.items():
                    accumulated[key] = accumulated.get(key, 0.0) + value
            self.optimizer.step()
            if self.scheduler is not None:
                self.scheduler.step()
            self.global_step += 1
            row = {
                "step": float(self.global_step),
                **{
                    key: value / micro_batch_count
                    for key, value in accumulated.items()
                },
            }
            self.logs.append(row)
            produced.append(row)
        return produced

    def save(self, path: Path) -> None:
        save_checkpoint_v3(
            path,
            identity=self.identity,
            model=self.model,
            optimizer=self.optimizer,
            scheduler=self.scheduler,
            sampler_state=self.sampler.state_dict(),
            global_step=self.global_step,
        )

    def resume(self, path: Path) -> None:
        sampler_state, self.global_step = load_checkpoint_v3(
            path,
            expected_identity=self.identity,
            model=self.model,
            optimizer=self.optimizer,
            scheduler=self.scheduler,
        )
        self.sampler.load_state_dict(sampler_state)


@dataclass(frozen=True)
class LaunchReadinessGateConfigV3:
    """Held-out path-readiness gate for stopped, commanded-motion samples."""

    current_speed_max_mps: float = 0.05
    commanded_speed_min_mps: float = 0.5
    minimum_forward_progress_m: float = 0.1
    minimum_samples: int = 20
    minimum_ready_fraction: float = 0.8
    minimum_runs: int = 0
    minimum_episodes: int = 0
    episode_gap_sec: float = 0.5
    runtime_parameter_path: str = (
        "ros2_ws/src/aic_e2e_runtime/config/"
        "runtime.v3.trajectory_authoritative.param.yaml"
    )

    def validate(self) -> None:
        values = (
            self.current_speed_max_mps,
            self.commanded_speed_min_mps,
            self.minimum_forward_progress_m,
            self.minimum_ready_fraction,
            self.episode_gap_sec,
        )
        if any(not math.isfinite(float(value)) for value in values):
            raise ValueError("launch readiness thresholds must be finite")
        if self.current_speed_max_mps < 0.0:
            raise ValueError("launch current speed threshold must be non-negative")
        if self.commanded_speed_min_mps <= self.current_speed_max_mps:
            raise ValueError("launch command threshold must exceed stopped threshold")
        if self.minimum_forward_progress_m <= 0.0:
            raise ValueError("launch minimum forward progress must be positive")
        if self.minimum_samples <= 0:
            raise ValueError("launch readiness minimum_samples must be positive")
        if self.minimum_runs < 0 or self.minimum_episodes < 0:
            raise ValueError("launch readiness run/episode minimums must be non-negative")
        if self.episode_gap_sec <= 0.0:
            raise ValueError("launch readiness episode gap must be positive")
        if not 0.0 <= self.minimum_ready_fraction <= 1.0:
            raise ValueError("launch readiness fraction must be within [0,1]")
        if not self.runtime_parameter_path.strip():
            raise ValueError("launch readiness runtime parameter path must not be empty")


def evaluate_trajectory_speed_v3(
    model: torch.nn.Module,
    batches: Sequence[ModelBatchV3],
    *,
    launch_gate: LaunchReadinessGateConfigV3 | None = None,
    launch_batches: Sequence[ModelBatchV3] | None = None,
) -> dict[str, object]:
    """Evaluate authoritative trajectory/speed heads on a held-out split.

    ``trajectory_ade_m`` is the mean Euclidean waypoint error in metres and
    ``speed_profile_mae_mps`` is the mean absolute speed error in m/s. Masks
    are counted element-wise, so a short final batch cannot bias the result.
    The model's prior train/eval state is restored before returning.
    """

    if not batches:
        raise ValueError("V3 validation requires at least one batch")
    first_parameter = next(iter(model.parameters()), None)
    device = torch.device("cpu") if first_parameter is None else first_parameter.device
    was_training = model.training
    trajectory_error_sum = 0.0
    trajectory_count = 0
    speed_error_sum = 0.0
    speed_count = 0
    run_trajectory_sum: dict[str, float] = defaultdict(float)
    run_trajectory_count: dict[str, int] = defaultdict(int)
    if launch_gate is not None:
        launch_gate.validate()
    model.eval()
    try:
        with torch.no_grad():
            metadata_provider = getattr(batches, "metadata_for_batch", None)
            for batch_index, source_batch in enumerate(batches):
                batch = move_batch_v3(source_batch, device)
                if batch.targets is None:
                    raise ValueError("V3 validation batch is missing targets")
                output = model(batch)
                if output.trajectory_xy.shape[1] != 1:
                    raise ValueError("V3 validation supports candidates K=1")
                predicted_xy = output.trajectory_xy[:, 0]
                predicted_speed = output.trajectory_speed_mps[:, 0]
                if predicted_xy.shape != batch.targets.trajectory_xy_m.shape:
                    raise ValueError("validation trajectory prediction/target shape mismatch")
                if predicted_speed.shape != batch.targets.speed_mps.shape:
                    raise ValueError("validation speed prediction/target shape mismatch")
                trajectory_error = torch.linalg.vector_norm(
                    predicted_xy - batch.targets.trajectory_xy_m, dim=-1
                )
                speed_error = torch.abs(predicted_speed - batch.targets.speed_mps)
                trajectory_values = trajectory_error[batch.targets.trajectory_mask]
                speed_values = speed_error[batch.targets.speed_mask]
                if not torch.isfinite(trajectory_values).all():
                    raise FloatingPointError("non-finite trajectory validation error")
                if not torch.isfinite(speed_values).all():
                    raise FloatingPointError("non-finite speed validation error")
                trajectory_error_sum += float(trajectory_values.sum().cpu())
                trajectory_count += int(trajectory_values.numel())
                speed_error_sum += float(speed_values.sum().cpu())
                speed_count += int(speed_values.numel())
                metadata = (
                    metadata_provider(batch_index)
                    if callable(metadata_provider)
                    else None
                )
                for row_index in range(batch.batch_size):
                    row_values = trajectory_error[row_index][
                        batch.targets.trajectory_mask[row_index]
                    ]
                    run_id = (
                        "__metadata_unavailable__"
                        if metadata is None
                        else metadata[row_index].run_id
                    )
                    run_trajectory_sum[run_id] += float(row_values.sum().cpu())
                    run_trajectory_count[run_id] += int(row_values.numel())
    finally:
        model.train(was_training)
    if trajectory_count == 0 or speed_count == 0:
        raise ValueError("V3 validation split has no valid trajectory/speed targets")
    run_ades = {
        run_id: run_trajectory_sum[run_id] / run_trajectory_count[run_id]
        for run_id in sorted(run_trajectory_sum)
        if run_trajectory_count[run_id] > 0
    }
    metrics: dict[str, object] = {
        "trajectory_ade_m": trajectory_error_sum / trajectory_count,
        "speed_profile_mae_mps": speed_error_sum / speed_count,
        "trajectory_valid_waypoints": float(trajectory_count),
        "speed_valid_waypoints": float(speed_count),
        "trajectory_run_equal_ade_m": float(np.mean(tuple(run_ades.values()))),
        "trajectory_worst_run_ade_m": float(max(run_ades.values())),
        "trajectory_run_ade_m": run_ades,
    }
    if launch_gate is not None:
        first_batch = batches[0]
        if first_batch.targets is None:
            raise ValueError("launch replay requires trajectory targets for shape identity")
        replay_config = load_path_only_replay_config_v3(
            launch_gate.runtime_parameter_path,
            trajectory_steps=int(first_batch.targets.trajectory_xy_m.shape[1]),
            minimum_endpoint_forward_m=launch_gate.minimum_forward_progress_m,
        )
        selected_launch_batches = batches if launch_batches is None else launch_batches
        launch_sample_count = 0
        launch_ready_count = 0
        reference_accepted_count = 0
        max_x_only_false_positive_count = 0
        requested_speed_at_launch_count = 0
        path_lengths: list[float] = []
        endpoint_forward: list[float] = []
        lookahead: list[float] = []
        reasons: dict[str, int] = defaultdict(int)
        launch_identities: list[tuple[str, str, int]] = []
        metadata_known = True
        model.eval()
        try:
            with torch.no_grad():
                launch_metadata_provider = getattr(
                    selected_launch_batches, "metadata_for_batch", None
                )
                for batch_index, source_batch in enumerate(selected_launch_batches):
                    batch = move_batch_v3(source_batch, device)
                    if (
                        batch.targets.current_control is None
                        or batch.targets.current_control_mask is None
                    ):
                        raise ValueError(
                            "launch readiness requires current control targets"
                        )
                    launch_rows = (
                        (torch.abs(batch.ego[:, -1, 0]) <= launch_gate.current_speed_max_mps)
                        & batch.targets.current_control_mask[:, 1]
                        & (
                            batch.targets.current_control[:, 1]
                            >= launch_gate.commanded_speed_min_mps
                        )
                    )
                    output = model(batch)
                    predicted_xy = output.trajectory_xy[:, 0]
                    predicted_speed = output.trajectory_speed_mps[:, 0]
                    metadata = (
                        launch_metadata_provider(batch_index)
                        if callable(launch_metadata_provider)
                        else None
                    )
                    for row_index in torch.flatnonzero(launch_rows).cpu().tolist():
                        launch_sample_count += 1
                        if metadata is None:
                            metadata_known = False
                        else:
                            item = metadata[row_index]
                            launch_identities.append(
                                (item.run_id, item.segment_id, item.grid_stamp_ns)
                            )
                        replay = replay_path_only_launch_v3(
                            predicted_xy[row_index].detach().cpu().numpy(),
                            predicted_speed[row_index].detach().cpu().numpy(),
                            current_speed_mps=float(batch.ego[row_index, -1, 0].cpu()),
                            yaw_rate_rps=float(batch.ego[row_index, -1, 2].cpu()),
                            actual_steering_rad=float(batch.ego[row_index, -1, 3].cpu()),
                            config=replay_config,
                        )
                        path_lengths.append(replay.path_length_m)
                        endpoint_forward.append(replay.endpoint_forward_m)
                        if replay.lookahead_distance_m is not None:
                            lookahead.append(replay.lookahead_distance_m)
                        reference_accepted_count += int(replay.reference_accepted)
                        launch_ready_count += int(replay.ready)
                        requested_speed_at_launch_count += int(
                            replay.controller_requested_speed_mps is not None
                            and replay.controller_requested_speed_mps
                            >= replay_config.minimum_controller_speed_mps
                        )
                        if (
                            replay.maximum_forward_m
                            >= launch_gate.minimum_forward_progress_m
                            and not replay.ready
                        ):
                            max_x_only_false_positive_count += 1
                        for reason in replay.reasons:
                            reasons[reason] += 1
        finally:
            model.train(was_training)
        run_ids = {item[0] for item in launch_identities}
        episodes = 0
        if metadata_known:
            grouped: dict[tuple[str, str], list[int]] = defaultdict(list)
            for run_id, segment_id, stamp_ns in launch_identities:
                grouped[(run_id, segment_id)].append(stamp_ns)
            gap_ns = int(round(launch_gate.episode_gap_sec * 1_000_000_000.0))
            for stamps in grouped.values():
                ordered = sorted(stamps)
                episodes += int(bool(ordered)) + sum(
                    right - left > gap_ns
                    for left, right in zip(ordered, ordered[1:])
                )
        ready_fraction = (
            launch_ready_count / launch_sample_count
            if launch_sample_count > 0
            else 0.0
        )
        metrics.update(
            {
                "launch_sample_count": float(launch_sample_count),
                "launch_path_ready_count": float(launch_ready_count),
                "launch_path_ready_fraction": ready_fraction,
                "launch_reference_accepted_count": float(reference_accepted_count),
                "launch_controller_speed_ready_count": float(
                    requested_speed_at_launch_count
                ),
                "launch_max_x_only_false_positive_count": float(
                    max_x_only_false_positive_count
                ),
                "launch_run_count": float(len(run_ids)) if metadata_known else None,
                "launch_episode_count": float(episodes) if metadata_known else None,
                "launch_metadata_known": metadata_known,
                "launch_mean_path_length_m": (
                    float(np.mean(path_lengths)) if path_lengths else 0.0
                ),
                "launch_mean_endpoint_forward_m": (
                    float(np.mean(endpoint_forward)) if endpoint_forward else 0.0
                ),
                "launch_mean_lookahead_distance_m": (
                    float(np.mean(lookahead)) if lookahead else 0.0
                ),
                "launch_rejection_reasons": dict(sorted(reasons.items())),
                "launch_stop_probability_connected": False,
                "launch_gate_pass": (
                    launch_sample_count >= launch_gate.minimum_samples
                    and ready_fraction >= launch_gate.minimum_ready_fraction
                    and (
                        launch_gate.minimum_runs == 0
                        or (metadata_known and len(run_ids) >= launch_gate.minimum_runs)
                    )
                    and (
                        launch_gate.minimum_episodes == 0
                        or (metadata_known and episodes >= launch_gate.minimum_episodes)
                    )
                ),
            }
        )
    return metrics


def is_better_trajectory_checkpoint_v3(
    candidate: dict[str, object], best: dict[str, object] | None
) -> bool:
    """Order research candidates by ADE then worst-run ADE; speed is diagnostic."""

    keys = ("trajectory_ade_m", "trajectory_worst_run_ade_m")
    for name in keys:
        value = float(candidate[name])
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"candidate {name} must be finite and non-negative")
    if best is None:
        return True
    for name in keys:
        value = float(best[name])
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"best {name} must be finite and non-negative")
    return tuple(float(candidate[name]) for name in keys) < tuple(
        float(best[name]) for name in keys
    )


def load_full_control_config_v3(path: str | Path) -> dict[str, object]:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(raw, dict) or raw.get("format") != "aic_model_config_v3":
        raise ValueError("not an aic_model_config_v3 configuration")
    for section in ("model", "data", "loss", "training"):
        if not isinstance(raw.get(section), dict):
            raise ValueError(f"full-control config missing {section} mapping")
    model = raw["model"]
    data = raw["data"]
    if model.get("name") != "full_control_lite_v3" or not bool(model.get("control_head_enabled")):
        raise ValueError("full-control training requires enabled FullControlLiteV3 control head")
    required_ego_prefix = [
        "longitudinal_speed_mps",
        "lateral_speed_mps",
        "yaw_rate_rps",
    ]
    ego_features = list(data.get("ego_features", []))
    if ego_features not in (
        required_ego_prefix,
        [*required_ego_prefix, "actual_steering_rad"],
    ):
        raise ValueError("full-control config has an unsupported explicit ego feature order")
    ego_abs_limits = data.get("ego_abs_limits")
    if ego_abs_limits is not None:
        if not isinstance(ego_abs_limits, dict) or set(ego_abs_limits) != set(ego_features):
            raise ValueError("ego_abs_limits must cover every configured ego feature exactly")
        if any(
            not math.isfinite(float(value)) or float(value) <= 0.0
            for value in ego_abs_limits.values()
        ):
            raise ValueError("ego_abs_limits values must be finite and positive")
    if float(raw["loss"].get("current_control", 0.0)) <= 0.0:
        raise ValueError("full-control config requires nonzero current_control loss")
    if not bool(model.get("control_sequence_head_enabled")):
        raise ValueError("full-control training requires enabled control_sequence head")
    if int(model.get("control_sequence_steps", 0)) <= 0:
        raise ValueError("full-control training requires positive control_sequence_steps")
    if float(raw["loss"].get("control_sequence", 0.0)) <= 0.0:
        raise ValueError("full-control config requires nonzero control_sequence loss")
    if not bool(model.get("behavior_head_enabled")):
        raise ValueError("full-control config requires enabled behavior head")
    if int(model.get("behavior_classes", 0)) != 5 or int(model.get("behavior_sides", 0)) != 3:
        raise ValueError("full-control behavior head requires aic_behavior_v1 dimensions 5/3")
    if float(raw["loss"].get("behavior", 0.0)) <= 0.0:
        raise ValueError("full-control config requires nonzero behavior loss")
    if float(raw["loss"].get("behavior_side", 0.0)) <= 0.0:
        raise ValueError("full-control config requires nonzero behavior_side loss")
    if int(raw["training"].get("checkpoint_every_steps", 0)) <= 0:
        raise ValueError("full-control training requires positive checkpoint_every_steps")
    if int(raw["training"].get("gradient_accumulation_steps", 1)) <= 0:
        raise ValueError("gradient_accumulation_steps must be positive")
    plan_consistency_weight = float(raw["loss"].get("plan_consistency", 0.0))
    plan_step_sec = float(raw["loss"].get("plan_step_sec", 0.1))
    if plan_consistency_weight < 0.0:
        raise ValueError("plan_consistency loss weight must be non-negative")
    if not math.isfinite(plan_step_sec) or plan_step_sec <= 0.0:
        raise ValueError("plan_step_sec must be finite and positive")
    targets = raw.get("targets")
    expected_targets = {
        "behavior_ontology": BEHAVIOR_ONTOLOGY_V1,
        "behavior_classes": list(BEHAVIOR_CLASS_NAMES_V1),
        "behavior_sides": list(BEHAVIOR_SIDE_NAMES_V1),
    }
    if not isinstance(targets, dict) or any(targets.get(key) != value for key, value in expected_targets.items()):
        raise ValueError("full-control config behavior target ontology/order mismatch")
    if targets.get("command_history_alignment") != COMMAND_HISTORY_ALIGNMENT_V3:
        raise ValueError("full-control config requires causal command history alignment")
    if targets.get("control_sequence_alignment") != CONTROL_SEQUENCE_ALIGNMENT_V3:
        raise ValueError("full-control config requires exact-grid control sequence alignment")
    command_history_length = int(data.get("command_history_length", 0))
    if not 0 < command_history_length <= int(model.get("max_ego_history", 0)):
        raise ValueError("command history length must fit max_ego_history")
    expected_history = {
        "image_history_length": int(model.get("max_sensor_history", 0)),
        "lidar_history_length": int(model.get("max_sensor_history", 0)),
        "ego_history_length": int(model.get("max_ego_history", 0)),
        "command_history_length": int(model.get("max_ego_history", 0)),
    }
    if any(int(data.get(name, 0)) != value for name, value in expected_history.items()):
        raise ValueError("model and data temporal history contracts differ")
    motion_target_filter_config_v3(raw)
    launch_readiness_gate_config_v3(raw)
    return raw


def launch_readiness_gate_config_v3(
    config: dict[str, object],
) -> LaunchReadinessGateConfigV3 | None:
    """Parse an optional fail-closed validation gate for launch path readiness."""

    validation = config.get("validation")
    if validation is None:
        return None
    if not isinstance(validation, dict):
        raise ValueError("validation must be a mapping")
    raw = validation.get("launch_gate")
    if raw is None:
        return None
    if not isinstance(raw, dict):
        raise ValueError("validation launch_gate must be a mapping")
    allowed = {
        "enabled",
        "current_speed_max_mps",
        "commanded_speed_min_mps",
        "minimum_forward_progress_m",
        "minimum_samples",
        "minimum_ready_fraction",
        "minimum_runs",
        "minimum_episodes",
        "episode_gap_sec",
        "runtime_parameter_path",
    }
    unknown = set(raw).difference(allowed)
    if unknown:
        raise ValueError(f"unknown validation launch_gate fields: {sorted(unknown)}")
    if not bool(raw.get("enabled", False)):
        return None
    result = LaunchReadinessGateConfigV3(
        current_speed_max_mps=float(raw.get("current_speed_max_mps", 0.05)),
        commanded_speed_min_mps=float(raw.get("commanded_speed_min_mps", 0.5)),
        minimum_forward_progress_m=float(
            raw.get("minimum_forward_progress_m", 0.1)
        ),
        minimum_samples=int(raw.get("minimum_samples", 20)),
        minimum_ready_fraction=float(raw.get("minimum_ready_fraction", 0.8)),
        minimum_runs=int(raw.get("minimum_runs", 0)),
        minimum_episodes=int(raw.get("minimum_episodes", 0)),
        episode_gap_sec=float(raw.get("episode_gap_sec", 0.5)),
        runtime_parameter_path=str(
            raw.get(
                "runtime_parameter_path",
                "ros2_ws/src/aic_e2e_runtime/config/"
                "runtime.v3.trajectory_authoritative.param.yaml",
            )
        ),
    )
    result.validate()
    return result


def motion_target_filter_config_v3(
    config: dict[str, object],
) -> MotionTargetFilterConfigV3:
    """Parse the optional Dataset V3 commanded-motion consistency filter."""

    targets = config.get("targets")
    if not isinstance(targets, dict):
        raise ValueError("full-control config targets must be a mapping")
    raw = targets.get("motion_target_filter")
    if raw is None:
        result = MotionTargetFilterConfigV3()
    elif not isinstance(raw, dict):
        raise ValueError("motion_target_filter must be a mapping")
    else:
        allowed = {
            "enabled",
            "stopped_speed_max_mps",
            "commanded_speed_min_mps",
            "minimum_future_speed_mps",
            "minimum_future_displacement_m",
            "horizon_steps",
        }
        unknown = set(raw).difference(allowed)
        if unknown:
            raise ValueError(f"unknown motion_target_filter fields: {sorted(unknown)}")
        result = MotionTargetFilterConfigV3(
            enabled=bool(raw.get("enabled", False)),
            stopped_speed_max_mps=float(raw.get("stopped_speed_max_mps", 0.05)),
            commanded_speed_min_mps=float(raw.get("commanded_speed_min_mps", 0.5)),
            minimum_future_speed_mps=float(raw.get("minimum_future_speed_mps", 0.2)),
            minimum_future_displacement_m=float(
                raw.get("minimum_future_displacement_m", 0.1)
            ),
            horizon_steps=int(raw.get("horizon_steps", 15)),
        )
    result.validate()
    return result


def build_full_control_model_v3(config: dict[str, object]) -> FullControlLiteV3:
    return FullControlLiteV3(**full_control_model_kwargs_v3(config))


def full_control_model_kwargs_v3(config: dict[str, object]) -> dict[str, object]:
    """Return the exact constructor contract embedded in runtime artifacts."""

    model = config["model"]
    data = config["data"]
    bounds = model["control_bounds"]
    return {
        "image_height": int(data["image_height"]),
        "image_width": int(data["image_width"]),
        "lidar_points": int(data["lidar_points"]),
        "ego_dim": len(data["ego_features"]),
        "hidden_dim": int(model["hidden_dim"]),
        "trajectory_steps": int(model["trajectory_steps"]),
        "candidates": int(model["candidates"]),
        "camera_tokens_hw": tuple(model["camera_tokens_hw"]),
        "lidar_tokens": int(model["lidar_tokens"]),
        "fusion_depth": int(model["fusion_depth"]),
        "fusion_heads": int(model["fusion_heads"]),
        "max_sensor_history": int(model["max_sensor_history"]),
        "max_ego_history": int(model["max_ego_history"]),
        "command_history_alignment": str(
            config["targets"]["command_history_alignment"]
        ),
        "control_head_enabled": True,
        "control_sequence_head_enabled": True,
        "control_sequence_steps": int(model["control_sequence_steps"]),
        "control_dt_sec": float(model["control_dt_sec"]),
        "max_steering_rad": float(bounds["max_steering_rad"]),
        "max_steering_rate_radps": float(bounds["max_steering_rate_radps"]),
        "max_speed_mps": float(bounds["max_speed_mps"]),
        "min_acceleration_mps2": float(bounds["min_acceleration_mps2"]),
        "max_acceleration_mps2": float(bounds["max_acceleration_mps2"]),
        "min_jerk_mps3": float(bounds["min_jerk_mps3"]),
        "max_jerk_mps3": float(bounds["max_jerk_mps3"]),
        "behavior_head_enabled": bool(model["behavior_head_enabled"]),
        "behavior_classes": int(model["behavior_classes"]),
        "behavior_sides": int(model["behavior_sides"]),
    }


def move_batch_v3(batch: ModelBatchV3, device: torch.device) -> ModelBatchV3:
    targets = batch.targets
    moved_targets = None
    if targets is not None:
        moved_targets = targets.__class__(
            trajectory_xy_m=targets.trajectory_xy_m.to(device),
            trajectory_mask=targets.trajectory_mask.to(device), speed_mps=targets.speed_mps.to(device),
            speed_mask=targets.speed_mask.to(device),
            current_control=None if targets.current_control is None else targets.current_control.to(device),
            current_control_mask=None if targets.current_control_mask is None else targets.current_control_mask.to(device),
            control_provenance=targets.control_provenance,
            control_sequence=None if targets.control_sequence is None else targets.control_sequence.to(device),
            control_sequence_mask=None if targets.control_sequence_mask is None else targets.control_sequence_mask.to(device),
            control_sequence_provenance=targets.control_sequence_provenance,
            control_sequence_time_sec=None if targets.control_sequence_time_sec is None else targets.control_sequence_time_sec.to(device),
            behavior_class=None if targets.behavior_class is None else targets.behavior_class.to(device),
            behavior_mask=None if targets.behavior_mask is None else targets.behavior_mask.to(device),
            behavior_side=None if targets.behavior_side is None else targets.behavior_side.to(device),
            behavior_side_mask=None if targets.behavior_side_mask is None else targets.behavior_side_mask.to(device),
        )
    return ModelBatchV3(
        image=batch.image.to(device), image_mask=batch.image_mask.to(device),
        lidar=batch.lidar.to(device), lidar_mask=batch.lidar_mask.to(device),
        ego=batch.ego.to(device), ego_feature_mask=batch.ego_feature_mask.to(device),
        command_history=batch.command_history.to(device), command_mask=batch.command_mask.to(device),
        sensor_dt_sec=batch.sensor_dt_sec.to(device), targets=moved_targets,
        requested_outputs=batch.requested_outputs,
    )


def _add_masked_accuracy(
    row: dict[str, float],
    name: str,
    logits: torch.Tensor,
    target: torch.Tensor,
    mask: torch.Tensor | None,
) -> None:
    if mask is None or not bool(mask.any()):
        return
    correct = logits[mask].argmax(dim=1) == target[mask]
    row[f"metric/{name}_accuracy"] = float(correct.float().mean().detach().cpu())
