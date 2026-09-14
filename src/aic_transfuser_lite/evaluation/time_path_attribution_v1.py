"""Recorded-path diagnostics, without changing predictions or vehicle commands."""
from __future__ import annotations

import math
from typing import Any

import numpy as np

from ..control.polyline_lookahead_v1 import select_polyline_lookahead
from ..control.time_reference_v1 import TimedBodyPose
from ..control.vehicle_motion_v1 import AWSIM_POLICY, effective_response_length
from .time_corner_comparison_v1 import RecordedLine


def lateral_decomposition(predicted_world_xy_m: np.ndarray, predicted_progress_m: np.ndarray,
                          actual_world_xy_m: np.ndarray, reference_world_xy_m: np.ndarray,
                          tangent_yaw_rad: float, progress_m: float) -> dict[str, Any]:
    """Split lateral error at ONE common reference station, in metres.

    Prediction [N,2], strictly increasing reference progress [N], actual [2],
    reference [2]. No endpoint extension; along-track speed error is excluded.
    Actual-reference = prediction-reference + actual-prediction in the same
    local normal basis. This identity is geometric, not causal attribution:
    later closed-loop replanning can affect the observed future position.
    """
    xy, s = np.asarray(predicted_world_xy_m, float), np.asarray(predicted_progress_m, float)
    actual, reference = np.asarray(actual_world_xy_m, float), np.asarray(reference_world_xy_m, float)
    if (xy.ndim != 2 or xy.shape[1:] != (2,) or len(xy) < 2 or s.shape != (len(xy),)
            or actual.shape != (2,) or reference.shape != (2,)
            or not all(np.isfinite(v).all() for v in (xy, s, actual, reference))
            or not np.isfinite([tangent_yaw_rad, progress_m]).all()):
        raise ValueError("DECOMPOSITION_SHAPE_OR_NONFINITE")
    if np.any(np.diff(s) <= 1e-9):
        raise ValueError("PREDICTION_PROGRESS_NOT_INCREASING")
    if not s[0] <= progress_m <= s[-1]:
        raise ValueError("FUTURE_OUTSIDE_PREDICTED_PROGRESS")
    point = np.array([np.interp(progress_m, s, xy[:, axis]) for axis in (0, 1)])
    normal = np.array([-math.sin(tangent_yaw_rad), math.cos(tangent_yaw_rad)])
    actual_left = float((actual-reference)@normal)
    prediction_left = float((point-reference)@normal)
    following_left = float((actual-point)@normal)
    return {"actual_left_m": actual_left, "prediction_left_m": prediction_left,
            "following_left_m": following_left, "matched_prediction_world_xy_m": point.tolist(),
            "sum_error_m": actual_left-prediction_left-following_left}


def reference_preview(reference: RecordedLine, current: TimedBodyPose, *, speed_mps: float,
                      remaining_horizon_s: float, rear_axle_offset_m: float,
                      vehicle_model_policy: str = AWSIM_POLICY) -> dict[str, Any]:
    """Same PP geometry on a measured normal line at the recorded current state.

    Samples [31,2] are existing teacher poses at matched-progress time + [0,H].
    This is an offline target sensitivity, NOT a valid off-line-state TimePlan:
    its first point is on the normal line, not the displaced vehicle origin.
    No runtime path validation, actuator, scan admission or driving is claimed.
    """
    if (not np.isfinite([speed_mps, remaining_horizon_s, rear_axle_offset_m]).all()
            or not 0. < remaining_horizon_s <= 3. or abs(rear_axle_offset_m) > .002):
        raise ValueError("REFERENCE_PREVIEW_CONTRACT")
    length = effective_response_length(speed_mps, vehicle_model_policy)
    projected = reference.project(np.array([current.x_m, current.y_m]), yaw_hint_rad=current.yaw_rad)
    times = np.linspace(0., remaining_horizon_s, 31)
    samples = [reference.index.at(projected.stamp_ns+round(t*1e9)) for t in times]
    world = np.array([[p.x_m, p.y_m] for p in samples])
    c, s = math.cos(current.yaw_rad), math.sin(current.yaw_rad)
    local = (world-[current.x_m, current.y_m])@np.array([[c, -s], [s, c]])-[rear_axle_offset_m, 0.]
    minimum = max(1., .4+.5*speed_mps+speed_mps**2/2)
    maximum = minimum+.5
    try:
        selection = select_polyline_lookahead(local, times, minimum_m=minimum, maximum_m=maximum,
                                               response_length_m=length)
    except ValueError as exc:
        if str(exc) != "STEERING_FEASIBLE_LOOKAHEAD_MISSING":
            raise
        maximum = minimum+1.
        selection = select_polyline_lookahead(local, times, minimum_m=minimum, maximum_m=maximum,
                                               response_length_m=length)
    return {"scope": "GEOMETRIC_NORMAL_LINE_SENSITIVITY_NOT_TIMEPLAN_OR_DRIVE_ADMISSION",
            "required_tire_rad": selection["required_tire_rad"], "selection": selection,
            "search_band_m": [minimum, maximum], "world_xy_m": world.tolist()}
