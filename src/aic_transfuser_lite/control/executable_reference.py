from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math

import numpy as np


def _readonly_float64(value: np.ndarray) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64).copy()
    result.setflags(write=False)
    return result


@dataclass(frozen=True)
class AuthoritativePlanV3:
    """Model motion intent in the observation ego frame.

    ``trajectory_xy_m`` is ``[N,2]``, ``speed_profile_mps`` and
    ``waypoint_times_sec`` are ``[N]``. Waypoint times are measured from
    ``observation_stamp_sec`` and do not include the current state at ``t=0``.
    """

    trajectory_xy_m: np.ndarray
    speed_profile_mps: np.ndarray
    waypoint_times_sec: np.ndarray
    observation_stamp_sec: float
    frame_id: str = "base_link"
    stop_probability: float | None = None

    def validate(self, *, require_stop_probability: bool) -> None:
        trajectory = np.asarray(self.trajectory_xy_m)
        speeds = np.asarray(self.speed_profile_mps)
        times = np.asarray(self.waypoint_times_sec)
        if trajectory.ndim != 2 or trajectory.shape[1] != 2 or len(trajectory) < 2:
            raise ValueError("trajectory_xy_m must be [N,2] with N>=2")
        if speeds.shape != (len(trajectory),):
            raise ValueError("speed_profile_mps must be [N]")
        if times.shape != (len(trajectory),):
            raise ValueError("waypoint_times_sec must be [N]")
        if not np.isfinite(trajectory).all():
            raise ValueError("trajectory_xy_m must be finite")
        if not np.isfinite(speeds).all() or bool((speeds < 0.0).any()):
            raise ValueError("speed_profile_mps must be finite and non-negative")
        if (
            not np.isfinite(times).all()
            or bool((times <= 0.0).any())
            or bool((np.diff(times) <= 0.0).any())
        ):
            raise ValueError(
                "waypoint_times_sec must be finite, positive, and strictly increasing"
            )
        if not math.isfinite(float(self.observation_stamp_sec)):
            raise ValueError("observation_stamp_sec must be finite")
        if not self.frame_id.strip():
            raise ValueError("frame_id must not be empty")
        if self.stop_probability is None:
            if require_stop_probability:
                raise ValueError("stop_probability is required")
        elif not math.isfinite(float(self.stop_probability)) or not (
            0.0 <= float(self.stop_probability) <= 1.0
        ):
            raise ValueError("stop_probability must be within [0,1]")


@dataclass(frozen=True)
class ExecutableReferenceConfigV3:
    """Limits used to transform model intent into a retimed reference."""

    odd_speed_cap_mps: float
    max_lateral_acceleration_mps2: float
    stop_probability_threshold: float = 0.6
    safety_speed_cap_mps: float | None = None
    require_stop_probability: bool = False
    minimum_initial_forward_m: float = 1e-3
    maximum_initial_noise_radius_m: float = 0.05
    minimum_path_length_m: float = 1e-3
    minimum_retime_speed_mps: float = 1e-3

    def validate(self) -> None:
        positive = (
            self.odd_speed_cap_mps,
            self.max_lateral_acceleration_mps2,
            self.minimum_initial_forward_m,
            self.maximum_initial_noise_radius_m,
            self.minimum_path_length_m,
            self.minimum_retime_speed_mps,
        )
        if not all(math.isfinite(float(value)) and value > 0.0 for value in positive):
            raise ValueError("reference limits must be finite and positive")
        if self.safety_speed_cap_mps is not None and (
            not math.isfinite(float(self.safety_speed_cap_mps))
            or self.safety_speed_cap_mps <= 0.0
        ):
            raise ValueError("safety_speed_cap_mps must be finite and positive")
        if not math.isfinite(float(self.stop_probability_threshold)) or not (
            0.0 <= self.stop_probability_threshold <= 1.0
        ):
            raise ValueError("stop_probability_threshold must be within [0,1]")


@dataclass(frozen=True)
class ExecutableReferenceV3:
    """Validated spatial path and retimed speed reference in SI units."""

    trajectory_xy_m: np.ndarray
    arc_length_m: np.ndarray
    speed_mps: np.ndarray
    time_from_observation_sec: np.ndarray
    curvature_per_m: np.ndarray
    source_waypoint_times_sec: np.ndarray
    observation_stamp_sec: float
    frame_id: str
    reference_id: str
    transformations: tuple[str, ...]

    def validate(self) -> None:
        trajectory = np.asarray(self.trajectory_xy_m)
        count = len(trajectory) if trajectory.ndim >= 1 else 0
        if trajectory.shape != (count, 2) or count < 2:
            raise ValueError("executable trajectory must be [N,2] with N>=2")
        for name, values in (
            ("arc_length_m", self.arc_length_m),
            ("speed_mps", self.speed_mps),
            ("time_from_observation_sec", self.time_from_observation_sec),
            ("curvature_per_m", self.curvature_per_m),
            ("source_waypoint_times_sec", self.source_waypoint_times_sec),
        ):
            array = np.asarray(values)
            if array.shape != (count,) or not np.isfinite(array).all():
                raise ValueError(f"{name} must be finite [N]")
        if bool((np.asarray(self.speed_mps) < 0.0).any()):
            raise ValueError("executable speed must be non-negative")
        if bool((np.diff(np.asarray(self.arc_length_m)) < 0.0).any()):
            raise ValueError("arc length must be non-decreasing")
        if bool((np.diff(np.asarray(self.time_from_observation_sec)) <= 0.0).any()):
            raise ValueError("executable times must be strictly increasing")
        if len(self.reference_id) != 64:
            raise ValueError("reference_id must be a SHA-256 hex digest")


@dataclass(frozen=True)
class ExecutableReferenceDecisionV3:
    """Fail-closed result: invalid or stopping plans never yield a reference."""

    reference: ExecutableReferenceV3 | None
    stop_required: bool
    reasons: tuple[str, ...]

    def validate(self) -> None:
        if self.stop_required:
            if self.reference is not None or not self.reasons:
                raise ValueError("stop decision requires reasons and no reference")
            return
        if self.reference is None or self.reasons:
            raise ValueError("drive decision requires one reference and no stop reasons")
        self.reference.validate()


def polyline_arc_length_m(trajectory_xy_m: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return per-segment length and cumulative arc length from ego origin."""

    trajectory = np.asarray(trajectory_xy_m, dtype=np.float64)
    if trajectory.ndim != 2 or trajectory.shape[1] != 2 or len(trajectory) == 0:
        raise ValueError("trajectory_xy_m must be non-empty [N,2]")
    if not np.isfinite(trajectory).all():
        raise ValueError("trajectory_xy_m must be finite")
    with_origin = np.vstack((np.zeros((1, 2), dtype=np.float64), trajectory))
    segment_length = np.linalg.norm(np.diff(with_origin, axis=0), axis=1)
    return segment_length, np.cumsum(segment_length)


def estimate_polyline_curvature_per_m(trajectory_xy_m: np.ndarray) -> np.ndarray:
    """Estimate absolute curvature at each waypoint from adjacent chords."""

    trajectory = np.asarray(trajectory_xy_m, dtype=np.float64)
    if trajectory.ndim != 2 or trajectory.shape[1] != 2 or len(trajectory) < 2:
        raise ValueError("trajectory_xy_m must be [N,2] with N>=2")
    if not np.isfinite(trajectory).all():
        raise ValueError("trajectory_xy_m must be finite")
    points = np.vstack((np.zeros((1, 2), dtype=np.float64), trajectory))
    curvature = np.zeros(len(trajectory), dtype=np.float64)
    for index in range(1, len(points) - 1):
        first = points[index] - points[index - 1]
        second = points[index + 1] - points[index]
        chord = points[index + 1] - points[index - 1]
        denominator = np.linalg.norm(first) * np.linalg.norm(second) * np.linalg.norm(chord)
        if denominator > 1e-12:
            cross_z = first[0] * second[1] - first[1] * second[0]
            curvature[index - 1] = abs(2.0 * cross_z / denominator)
    if len(curvature) > 1:
        curvature[-1] = curvature[-2]
    return curvature


def _reference_id(
    plan: AuthoritativePlanV3,
    trajectory: np.ndarray,
    speeds: np.ndarray,
    times: np.ndarray,
) -> str:
    digest = hashlib.sha256()
    for array in (trajectory, speeds, times):
        digest.update(np.asarray(array, dtype="<f8").tobytes())
    digest.update(np.asarray([plan.observation_stamp_sec], dtype="<f8").tobytes())
    digest.update(plan.frame_id.encode("utf-8"))
    return digest.hexdigest()


def build_executable_reference_v3(
    plan: AuthoritativePlanV3,
    *,
    current_speed_mps: float,
    config: ExecutableReferenceConfigV3,
) -> ExecutableReferenceDecisionV3:
    """Validate and retime an authoritative plan, returning STOP on failure."""

    try:
        config.validate()
        plan.validate(require_stop_probability=config.require_stop_probability)
        current_speed = float(current_speed_mps)
        if not math.isfinite(current_speed) or current_speed < 0.0:
            raise ValueError("current_speed_mps must be finite and non-negative")
    except ValueError as error:
        result = ExecutableReferenceDecisionV3(
            reference=None,
            stop_required=True,
            reasons=(f"invalid_plan:{error}",),
        )
        result.validate()
        return result

    if (
        plan.stop_probability is not None
        and plan.stop_probability >= config.stop_probability_threshold
    ):
        result = ExecutableReferenceDecisionV3(
            reference=None,
            stop_required=True,
            reasons=("model_stop",),
        )
        result.validate()
        return result

    trajectory = np.asarray(plan.trajectory_xy_m, dtype=np.float64)
    predicted_speed = np.asarray(plan.speed_profile_mps, dtype=np.float64)
    source_times = np.asarray(plan.waypoint_times_sec, dtype=np.float64)
    transformations: list[str] = []
    if float(trajectory[0, 0]) <= config.minimum_initial_forward_m:
        forward_indexes = np.flatnonzero(
            trajectory[:, 0] > config.minimum_initial_forward_m
        )
        trim_count = int(forward_indexes[0]) if len(forward_indexes) else len(trajectory)
        leading_radius = np.linalg.norm(trajectory[:trim_count], axis=1)
        recoverable = (
            trim_count > 0
            and len(trajectory) - trim_count >= 2
            and bool((leading_radius <= config.maximum_initial_noise_radius_m).all())
        )
        if not recoverable:
            result = ExecutableReferenceDecisionV3(
                reference=None,
                stop_required=True,
                reasons=("initial_waypoint_not_forward",),
            )
            result.validate()
            return result
        trajectory = trajectory[trim_count:]
        predicted_speed = predicted_speed[trim_count:]
        source_times = source_times[trim_count:]
        transformations.append("trimmed_initial_nonforward_noise")

    segment_length, arc_length = polyline_arc_length_m(trajectory)
    if float(arc_length[-1]) < config.minimum_path_length_m:
        result = ExecutableReferenceDecisionV3(
            reference=None,
            stop_required=True,
            reasons=("path_too_short",),
        )
        result.validate()
        return result

    curvature = estimate_polyline_curvature_per_m(trajectory)
    curvature_speed_cap = np.full(len(trajectory), math.inf, dtype=np.float64)
    curved = curvature > 1e-12
    curvature_speed_cap[curved] = np.sqrt(
        config.max_lateral_acceleration_mps2 / curvature[curved]
    )
    cap = np.minimum(curvature_speed_cap, config.odd_speed_cap_mps)
    if bool((predicted_speed > config.odd_speed_cap_mps).any()):
        transformations.append("odd_speed_cap")
    if bool((curvature_speed_cap < np.minimum(predicted_speed, config.odd_speed_cap_mps)).any()):
        transformations.append("curvature_speed_cap")
    if config.safety_speed_cap_mps is not None:
        if bool((np.minimum(predicted_speed, cap) > config.safety_speed_cap_mps).any()):
            transformations.append("safety_speed_cap")
        cap = np.minimum(cap, config.safety_speed_cap_mps)
    executable_speed = np.minimum(predicted_speed, cap)
    if plan.stop_probability is None:
        transformations.append("stop_probability_unavailable")

    source_step_times = np.diff(np.concatenate(([0.0], source_times)))
    executable_step_times = np.empty(len(trajectory), dtype=np.float64)
    previous_speed = current_speed
    for index, distance in enumerate(segment_length):
        average_speed = 0.5 * (previous_speed + float(executable_speed[index]))
        if distance > config.minimum_path_length_m and (
            average_speed < config.minimum_retime_speed_mps
        ):
            result = ExecutableReferenceDecisionV3(
                reference=None,
                stop_required=True,
                reasons=(f"non_executable_speed:segment={index}",),
            )
            result.validate()
            return result
        executable_step_times[index] = (
            float(distance) / average_speed
            if distance > config.minimum_path_length_m
            else float(source_step_times[index])
        )
        previous_speed = float(executable_speed[index])
    executable_times = np.cumsum(executable_step_times)
    if not np.isfinite(executable_times).all() or bool((np.diff(executable_times) <= 0.0).any()):
        result = ExecutableReferenceDecisionV3(
            reference=None,
            stop_required=True,
            reasons=("invalid_retime_result",),
        )
        result.validate()
        return result

    reference = ExecutableReferenceV3(
        trajectory_xy_m=_readonly_float64(trajectory),
        arc_length_m=_readonly_float64(arc_length),
        speed_mps=_readonly_float64(executable_speed),
        time_from_observation_sec=_readonly_float64(executable_times),
        curvature_per_m=_readonly_float64(curvature),
        source_waypoint_times_sec=_readonly_float64(source_times),
        observation_stamp_sec=float(plan.observation_stamp_sec),
        frame_id=plan.frame_id,
        reference_id=_reference_id(plan, trajectory, executable_speed, executable_times),
        transformations=tuple(transformations),
    )
    result = ExecutableReferenceDecisionV3(reference, False, ())
    result.validate()
    return result
