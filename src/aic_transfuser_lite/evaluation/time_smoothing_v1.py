"""Offline, within-prediction smoothing diagnostics; no runtime integration.

The 0.1 s grid, observation origin, and 3 s endpoint are fixed. Each method
only sees one already-predicted path, never future sensor frames or teachers.
These filters do not constrain collision clearance, heading, or curvature.
"""
from __future__ import annotations

from functools import lru_cache
import math
from typing import Any

import numpy as np

from aic_transfuser_lite.control.time_reference_v1 import TimePlan, TimedBodyPose, prepare_time_reference
from aic_transfuser_lite.control.time_trial_v1 import time_trial_control
from aic_transfuser_lite.control.vehicle_motion_v1 import effective_response_length


METHODS = ("raw", "mean3", "mean5", "d2_lambda1", "d2_lambda10", "d2_lambda100")
DT_S = 0.1


@lru_cache(maxsize=len(METHODS))
def _operator(method: str) -> np.ndarray:
    """31-point linear operator, including the known zero observation origin."""
    if method not in METHODS:
        raise ValueError("UNKNOWN_SMOOTHING_METHOD")
    operator = np.eye(31)
    if method.startswith("mean"):
        radius = int(method[4:]) // 2
        for i in range(1, 30):
            # Symmetric shrinking windows preserve constant-velocity lines.
            local_radius = min(radius, i, 30 - i)
            operator[i] = 0.
            operator[i, i-local_radius:i+local_radius+1] = 1. / (2*local_radius+1)
    elif method.startswith("d2_lambda"):
        weight = float(method.removeprefix("d2_lambda"))
        difference = np.diff(np.eye(31), n=2, axis=0)
        normal = np.eye(31) + weight * difference.T @ difference
        rhs = np.eye(31)[1:-1].copy()
        rhs[:, [0, 30]] -= normal[1:-1][:, [0, 30]]
        operator[1:-1] = np.linalg.solve(normal[1:-1, 1:-1], rhs)
    operator.setflags(write=False)
    return operator


def smooth_time_paths(xy_m: np.ndarray, method: str) -> np.ndarray:
    """Finite [30,2] or [N,30,2] metres -> new float64 array on the same time grid.

    d2 solves sum(||q_i-p_i||^2) + lambda*sum(||q_i-2q_(i+1)+q_(i+2)||^2),
    with q_0=0 and q_30=p_30. Lambda is a dimensionless discrete-grid weight,
    not a physical acceleration limit. Endpoints are assigned exactly.
    """
    xy = np.asarray(xy_m, dtype=float)
    if xy.ndim not in (2, 3) or xy.shape[-2:] != (30, 2) or not xy.size or not np.isfinite(xy).all():
        raise ValueError("SMOOTHING_REQUIRES_FINITE_30X2_METRES")
    points = np.concatenate((np.zeros((*xy.shape[:-2], 1, 2)), xy), axis=-2)
    result = (_operator(method) @ points)[..., 1:, :].copy()
    result[..., -1, :] = xy[..., -1, :]
    if not np.isfinite(result).all():
        raise ValueError("SMOOTHING_NONFINITE_RESULT")
    return result


def distribution(values: Any) -> dict[str, float | int] | None:
    """Explicit empty support; inputs must be finite scalar samples."""
    array = np.asarray(values, dtype=float).reshape(-1)
    if not len(array):
        return None
    if not np.isfinite(array).all():
        raise ValueError("NONFINITE_DISTRIBUTION")
    return {"count": int(array.size), "min": float(array.min()), "mean": float(array.mean()),
            "median": float(np.median(array)), "p95": float(np.quantile(array, .95)),
            "max": float(array.max())}


def path_change_metrics(raw: np.ndarray, changed: np.ndarray) -> dict[str, Any]:
    """Paired same-time displacement and discrete acceleration proxies, SI units.

    Position change is not nearest-polyline distance, teacher error, or clearance.
    The acceleration proxy divides second differences by (0.1 s)^2; near-zero
    segment lengths never enter a curvature denominator.
    """
    raw = np.asarray(raw, dtype=float).reshape(-1, 30, 2)
    changed = np.asarray(changed, dtype=float).reshape(-1, 30, 2)
    if raw.shape != changed.shape or not len(raw) or not np.isfinite(raw).all() or not np.isfinite(changed).all():
        raise ValueError("PAIRED_FINITE_PATHS_REQUIRED")
    delta = np.linalg.norm(changed - raw, axis=-1)
    first_speed = []
    acceleration_rms = []
    for paths in (raw, changed):
        points = np.concatenate((np.zeros((len(paths), 1, 2)), paths), axis=1)
        speed = np.linalg.norm(np.diff(points, axis=1), axis=-1) / DT_S
        first_speed.append(speed[:, :3].mean(axis=1))
        acceleration = np.diff(points, n=2, axis=1) / DT_S**2
        acceleration_rms.append(np.sqrt(np.mean(np.sum(acceleration**2, axis=-1), axis=1)))
    return {"paths": len(raw), "same_time_point_displacement_m": distribution(delta),
            "per_path_max_displacement_m": distribution(delta.max(axis=1)),
            "same_time_lateral_displacement_abs_m": distribution(abs(changed[..., 1]-raw[..., 1])),
            "endpoint_max_displacement_m": float(delta[:, -1].max()),
            "first_0p3s_implied_speed_change_abs_mps": distribution(abs(first_speed[1]-first_speed[0])),
            "raw_discrete_acceleration_rms_mps2": distribution(acceleration_rms[0]),
            "changed_discrete_acceleration_rms_mps2": distribution(acceleration_rms[1])}


def probe_recorded_pp(xy_m: np.ndarray, command: dict[str, Any], config: dict[str, Any]) -> dict[str, Any]:
    """Replay geometry and PP only at the recorded pose/speed; no scan admission.

    No propagation of hypothetical vehicle motion or steering actuator state is
    performed. A success means the pure control calculation has a valid point.
    """
    if config.get("lookahead_policy") != "stopping_preview_v1" or config.get("speed_policy") != "fixed_5kmh":
        raise ValueError("DIAGNOSTIC_REQUIRES_RECORDED_FIXED_5KMH_STOPPING_PREVIEW")
    details = command["details"]
    observed = TimedBodyPose(**details["observation_pose"])
    current = TimedBodyPose(**details["current_pose"])
    plan = TimePlan(command["plan_id"], observed, xy_m)
    speed = command["speed_mps"]
    offset = (config["geometry"]["rear_axle_forward_in_base_link_m"], 0.)
    result: dict[str, Any] = {"reason": "PP_OK", "selected_steer_rad": None,
                              "preview_min_m": None, "candidate_count": 0,
                              "best_steering_margin_rad": None}
    try:
        reference = prepare_time_reference(plan, current, rear_axle_offset_m=offset)
        minimum = max(1., .4 + max(0., speed)*.5 + speed**2/2)
        length = effective_response_length(max(0., speed), config["vehicle_model_policy"])
        angles = []
        for x, y in np.asarray(reference.xy_current_m[1:], dtype=np.float32).tolist():
            squared = x*x + y*y
            if x > 1e-6 and minimum <= math.sqrt(squared) <= minimum+.5:
                angles.append(math.atan(length * 2*y/max(squared, 1e-6)))
        result.update(preview_min_m=minimum, candidate_count=len(angles),
                      best_steering_margin_rad=.3-min(map(abs, angles)) if angles else None)
        control = time_trial_control(plan, current, speed_mps=speed, rear_axle_offset_m=offset,
            speed_policy=config["speed_policy"], lookahead_policy=config["lookahead_policy"],
            vehicle_model_policy=config["vehicle_model_policy"])
        if not math.isclose(minimum, control["minimum_preview_distance_m"], abs_tol=1e-12):
            raise AssertionError("diagnostic and controller preview differ")
        result["selected_steer_rad"] = control["steer_rad"]
    except ValueError as exc:
        result["reason"] = str(exc)
    return result
