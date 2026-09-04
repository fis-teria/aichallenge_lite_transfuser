from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

import numpy as np
import yaml

from aic_transfuser_lite.control.delay_aware_controller import (
    DelayAwareControllerConfig,
)
from aic_transfuser_lite.control.executable_reference import (
    AuthoritativePlanV3,
    ExecutableReferenceConfigV3,
    build_executable_reference_v3,
    polyline_arc_length_m,
)
from aic_transfuser_lite.control.longitudinal_controller_v3 import (
    LongitudinalControllerConfigV3,
    LongitudinalControllerV3,
)
from aic_transfuser_lite.control.trajectory_authoritative_controller import (
    control_from_executable_reference_v3,
)
from aic_transfuser_lite.runtime.control_projection import (
    normalize_measured_speed_for_projection,
)


@dataclass(frozen=True)
class PathOnlyReplayConfigV3:
    waypoint_times_sec: tuple[float, ...]
    reference: ExecutableReferenceConfigV3
    controller: DelayAwareControllerConfig
    longitudinal: LongitudinalControllerConfigV3
    minimum_endpoint_forward_m: float = 0.1
    minimum_controller_speed_mps: float = 0.2

    def validate(self) -> None:
        self.reference.validate()
        self.controller.__post_init__()
        self.longitudinal.validate()
        if len(self.waypoint_times_sec) < 2:
            raise ValueError("launch replay requires at least two waypoint times")
        if not all(
            math.isfinite(value) and value > 0.0
            for value in self.waypoint_times_sec
        ):
            raise ValueError("launch replay waypoint times must be finite and positive")
        if any(
            right <= left
            for left, right in zip(
                self.waypoint_times_sec, self.waypoint_times_sec[1:]
            )
        ):
            raise ValueError("launch replay waypoint times must increase")
        if (
            not math.isfinite(self.minimum_endpoint_forward_m)
            or self.minimum_endpoint_forward_m <= 0.0
        ):
            raise ValueError("launch replay endpoint threshold must be positive")
        if (
            not math.isfinite(self.minimum_controller_speed_mps)
            or self.minimum_controller_speed_mps <= 0.0
        ):
            raise ValueError("launch replay controller speed threshold must be positive")


@dataclass(frozen=True)
class LaunchReplayResultV3:
    ready: bool
    reference_accepted: bool
    reasons: tuple[str, ...]
    transformations: tuple[str, ...]
    initial_forward_m: float
    maximum_forward_m: float
    endpoint_forward_m: float
    endpoint_displacement_m: float
    path_length_m: float
    trim_count: int
    maximum_abs_curvature_per_m: float | None
    endpoint_heading_rad: float | None
    controller_requested_speed_mps: float | None
    controller_acceleration_mps2: float | None
    controller_state: str | None
    lookahead_distance_m: float | None
    stop_probability_connected: bool = False


def load_path_only_replay_config_v3(
    path: str | Path,
    *,
    trajectory_steps: int,
    minimum_endpoint_forward_m: float,
) -> PathOnlyReplayConfigV3:
    """Load the same checked-in ROS parameter profile used at runtime."""

    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    try:
        params = raw["/**"]["ros__parameters"]
    except (KeyError, TypeError) as error:
        raise ValueError("runtime parameter YAML has no /**/ros__parameters") from error
    step_sec = float(params["trajectory_step_sec"])
    if trajectory_steps <= 1 or not math.isfinite(step_sec) or step_sec <= 0.0:
        raise ValueError("runtime trajectory horizon must be finite and non-trivial")
    waypoint_times = tuple(step_sec * (index + 1) for index in range(trajectory_steps))
    safety_cap = float(params["executable_reference_safety_speed_cap_mps"])
    path_only_speed = float(params["executable_reference_path_only_target_speed_mps"])
    reference = ExecutableReferenceConfigV3(
        odd_speed_cap_mps=float(params["executable_reference_odd_speed_cap_mps"]),
        max_lateral_acceleration_mps2=float(
            params["executable_reference_max_lateral_acceleration_mps2"]
        ),
        stop_probability_threshold=float(params["stop_prob_threshold"]),
        safety_speed_cap_mps=None if safety_cap == 0.0 else safety_cap,
        require_stop_probability=bool(
            params["executable_reference_require_stop_probability"]
        ),
        speed_source=str(params["executable_reference_speed_source"]),
        path_only_target_speed_mps=(
            None if path_only_speed == 0.0 else path_only_speed
        ),
    )
    controller = DelayAwareControllerConfig(
        waypoint_times_sec=waypoint_times,
        estimated_delay_sec=float(params["estimated_delay_sec"]),
        base_preview_sec=float(params["base_preview_sec"]),
        min_preview_sec=float(params["min_preview_sec"]),
        max_preview_sec=float(params["max_preview_sec"]),
        minimum_lookahead_distance_m=float(params["minimum_lookahead_distance_m"]),
        wheelbase_m=float(params["wheelbase_m"]),
        max_steer_rad=float(params["max_steer_rad"]),
        min_accel_mps2=float(params["min_accel_mps2"]),
        max_accel_mps2=float(params["max_accel_mps2"]),
        speed_kp=float(params["speed_kp"]),
        max_steering_rate_radps=float(params["max_steering_rate_radps"]),
        control_period_sec=float(params["control_period_sec"]),
    )
    longitudinal = LongitudinalControllerConfigV3(
        control_dt_sec=float(params["control_period_sec"]),
        reference_horizon_sec=float(params["longitudinal_reference_horizon_sec"]),
        speed_kp=float(params["speed_kp"]),
        speed_ki=float(params["speed_ki"]),
        integral_limit_mps_sec=float(params["longitudinal_integral_limit_mps_sec"]),
        acceleration_gain=float(params["longitudinal_acceleration_gain"]),
        acceleration_bias_mps2=float(params["longitudinal_acceleration_bias_mps2"]),
        min_acceleration_mps2=float(params["min_accel_mps2"]),
        max_acceleration_mps2=float(params["max_accel_mps2"]),
        min_jerk_mps3=float(params["min_jerk_mps3"]),
        max_jerk_mps3=float(params["max_jerk_mps3"]),
        stopped_speed_mps=float(params["launch_assist_stopped_speed_mps"]),
        moving_speed_mps=float(params["launch_moving_speed_mps"]),
        launch_min_reference_speed_mps=float(
            params["launch_assist_min_commanded_speed_mps"]
        ),
        launch_acceleration_floor_mps2=float(
            params["launch_assist_acceleration_floor_mps2"]
        ),
        launch_timeout_sec=float(params["launch_timeout_sec"]),
        response_timeout_sec=float(params["launch_response_timeout_sec"]),
        minimum_launch_speed_delta_mps=float(
            params["launch_minimum_speed_delta_mps"]
        ),
    )
    result = PathOnlyReplayConfigV3(
        waypoint_times_sec=waypoint_times,
        reference=reference,
        controller=controller,
        longitudinal=longitudinal,
        minimum_endpoint_forward_m=float(minimum_endpoint_forward_m),
        minimum_controller_speed_mps=float(
            params["launch_assist_min_commanded_speed_mps"]
        ),
    )
    result.validate()
    if result.reference.speed_source != "path_only_constant":
        raise ValueError("launch replay requires path_only_constant runtime speed")
    return result


def replay_path_only_launch_v3(
    trajectory_xy_m: np.ndarray,
    model_speed_mps: np.ndarray,
    *,
    current_speed_mps: float,
    yaw_rate_rps: float,
    actual_steering_rad: float,
    config: PathOnlyReplayConfigV3,
) -> LaunchReplayResultV3:
    """Replay one prediction through runtime reference and controller code.

    This proves only that runtime would request launch from the supplied path;
    it does not predict measured vehicle response or a closed-loop launch.
    """

    config.validate()
    trajectory = np.asarray(trajectory_xy_m, dtype=np.float64)
    speeds = np.asarray(model_speed_mps, dtype=np.float64)
    if trajectory.ndim != 2 or trajectory.shape[1] != 2:
        raise ValueError("launch replay trajectory must be [N,2]")
    if speeds.shape != (len(trajectory),):
        raise ValueError("launch replay model speed must be [N]")
    if len(trajectory) != len(config.waypoint_times_sec):
        raise ValueError("launch replay horizon differs from runtime waypoint times")
    finite = np.isfinite(trajectory).all() and np.isfinite(speeds).all()
    if not finite:
        raise ValueError("launch replay prediction must be finite")
    _, cumulative = polyline_arc_length_m(trajectory)
    endpoint = trajectory[-1]
    endpoint_heading = None
    if len(trajectory) >= 2:
        delta = trajectory[-1] - trajectory[-2]
        if float(np.linalg.norm(delta)) > 1e-6:
            endpoint_heading = float(math.atan2(delta[1], delta[0]))
    plan = AuthoritativePlanV3(
        trajectory_xy_m=trajectory,
        speed_profile_mps=speeds,
        waypoint_times_sec=np.asarray(config.waypoint_times_sec),
        observation_stamp_sec=0.0,
        frame_id="base_link",
        stop_probability=None,
    )
    measured_speed = normalize_measured_speed_for_projection(current_speed_mps)
    decision = build_executable_reference_v3(
        plan, current_speed_mps=measured_speed, config=config.reference
    )
    common = {
        "initial_forward_m": float(trajectory[0, 0]),
        "maximum_forward_m": float(trajectory[:, 0].max()),
        "endpoint_forward_m": float(endpoint[0]),
        "endpoint_displacement_m": float(np.linalg.norm(endpoint)),
        "path_length_m": float(cumulative[-1]),
        "endpoint_heading_rad": endpoint_heading,
    }
    if decision.stop_required or decision.reference is None:
        return LaunchReplayResultV3(
            ready=False,
            reference_accepted=False,
            reasons=decision.reasons,
            transformations=(),
            trim_count=0,
            maximum_abs_curvature_per_m=None,
            controller_requested_speed_mps=None,
            controller_acceleration_mps2=None,
            controller_state=None,
            lookahead_distance_m=None,
            **common,
        )
    reference = decision.reference
    trim_count = len(trajectory) - len(reference.trajectory_xy_m)
    try:
        controller = LongitudinalControllerV3(config.longitudinal)
        control = control_from_executable_reference_v3(
            reference,
            current_longitudinal_speed_mps=measured_speed,
            yaw_rate_rps=float(yaw_rate_rps),
            actual_steering_rad=float(actual_steering_rad),
            config=config.controller,
            longitudinal_controller=controller,
            drive_preflight_ready=True,
        )
    except ValueError as error:
        return LaunchReplayResultV3(
            ready=False,
            reference_accepted=True,
            reasons=(f"controller_rejected:{error}",),
            transformations=reference.transformations,
            trim_count=trim_count,
            maximum_abs_curvature_per_m=float(
                np.max(np.abs(reference.curvature_per_m))
            ),
            controller_requested_speed_mps=None,
            controller_acceleration_mps2=None,
            controller_state=None,
            lookahead_distance_m=None,
            **common,
        )
    requested_speed = float(control.control.commanded_speed_mps)
    reasons: list[str] = []
    if common["endpoint_forward_m"] < config.minimum_endpoint_forward_m:
        reasons.append("endpoint_forward_too_short")
    if requested_speed < config.minimum_controller_speed_mps:
        reasons.append("controller_speed_below_launch_threshold")
    if control.longitudinal is None:
        reasons.append("longitudinal_diagnostics_missing")
        state = None
        acceleration = None
    else:
        state = control.longitudinal.state.value
        acceleration = float(control.longitudinal.acceleration_mps2)
        if control.longitudinal.fault_reason is not None:
            reasons.append(f"longitudinal_fault:{control.longitudinal.fault_reason}")
    return LaunchReplayResultV3(
        ready=not reasons,
        reference_accepted=True,
        reasons=tuple(reasons),
        transformations=reference.transformations,
        trim_count=trim_count,
        maximum_abs_curvature_per_m=float(
            np.max(np.abs(reference.curvature_per_m))
        ),
        controller_requested_speed_mps=requested_speed,
        controller_acceleration_mps2=acceleration,
        controller_state=state,
        lookahead_distance_m=float(control.control.lookahead_distance_m),
        **common,
    )
