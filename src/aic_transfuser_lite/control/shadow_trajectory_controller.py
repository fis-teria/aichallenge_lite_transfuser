from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .delay_aware_controller import (
    DelayAwareControllerConfig,
    DelayAwareControlResult,
    control_from_waypoints_delay_aware,
)


@dataclass(frozen=True)
class ShadowTrajectoryControlResult:
    """Debug-only controller proposal; never grants nominal authority."""

    control: DelayAwareControlResult
    target_speed_mps: float
    calibration_status: str = "unverified"
    nominal_control_eligible: bool = False


def shadow_control_from_trajectory_speed_profile(
    trajectory_xy_m: np.ndarray,
    speed_profile_mps: np.ndarray,
    *,
    current_longitudinal_speed_mps: float,
    yaw_rate_rps: float,
    actual_steering_rad: float,
    config: DelayAwareControllerConfig,
    delay_override_sec: float | None = None,
) -> ShadowTrajectoryControlResult:
    """Compute a debug proposal from matching ``[N,2]`` and ``[N]`` outputs.

    Waypoint times, speeds, vehicle state, and controller outputs use SI units.
    The speed target is interpolated at the same delay-adjusted preview time as
    the lateral controller. This helper cannot produce an authority-eligible
    command while V3 calibration is unverified.
    """

    points = np.asarray(trajectory_xy_m, dtype=np.float32)
    speeds = np.asarray(speed_profile_mps, dtype=np.float32)
    if points.ndim != 2 or points.shape[1:] != (2,) or points.shape[0] < 1:
        raise ValueError(f"trajectory_xy_m must be [N,2], got {points.shape}")
    if speeds.shape != (points.shape[0],):
        raise ValueError(
            f"speed_profile_mps must be [N] matching trajectory, got {speeds.shape}"
        )
    if len(config.waypoint_times_sec) != points.shape[0]:
        raise ValueError("waypoint time count must match trajectory and speed profile")
    if not np.isfinite(speeds).all():
        raise ValueError("speed_profile_mps must be finite")
    if bool((speeds < 0.0).any()):
        raise ValueError("speed_profile_mps must be non-negative")

    delay = config.estimated_delay_sec if delay_override_sec is None else float(delay_override_sec)
    if not math.isfinite(delay) or delay < 0.0:
        raise ValueError("delay must be finite and non-negative")
    preview_time_sec = float(
        np.clip(
            config.base_preview_sec + delay,
            config.min_preview_sec,
            config.max_preview_sec,
        )
    )
    target_speed_mps = float(
        np.interp(preview_time_sec, config.waypoint_times_sec, speeds)
    )
    control = control_from_waypoints_delay_aware(
        points,
        target_speed_mps=target_speed_mps,
        current_longitudinal_speed_mps=current_longitudinal_speed_mps,
        yaw_rate_rps=yaw_rate_rps,
        actual_steering_rad=actual_steering_rad,
        config=config,
        delay_override_sec=delay,
    )
    if not math.isclose(control.preview_time_sec, preview_time_sec, abs_tol=1e-12):
        raise AssertionError("lateral and longitudinal preview times diverged")
    return ShadowTrajectoryControlResult(
        control=control,
        target_speed_mps=target_speed_mps,
    )
