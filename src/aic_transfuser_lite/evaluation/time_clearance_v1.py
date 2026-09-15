"""Offline comparison helpers; never grants permission to send vehicle commands."""
from __future__ import annotations

from bisect import bisect_left
from collections import Counter
from typing import Any, Sequence

import numpy as np

from ..control.curvature_support_v2 import curvature_support_envelope, clearance_dimensions, STANDARD_CLEARANCE
from ..control.time_reference_v1 import TimedBodyPose
from ..control.time_trial_v1 import interpolate_body_pose
from ..control.turning_scan_guard import check_turning_scan
from ..control.vehicle_motion_v1 import AWSIM_POLICY, stopping_motion


class PoseIndex:
    """Original capture nanoseconds; ambiguous stamps and >50 ms gaps reject."""

    def __init__(self, poses: Sequence[TimedBodyPose]) -> None:
        if not poses:
            raise ValueError("EMPTY_POSES")
        counts = Counter(p.stamp_ns for p in poses)
        self.ambiguous = {t for t, n in counts.items() if n > 1}
        self.rows = sorted({p.stamp_ns: p for p in poses}.values(), key=lambda p: p.stamp_ns)
        self.times = [p.stamp_ns for p in self.rows]

    def at(self, stamp_ns: int) -> TimedBodyPose:
        i = bisect_left(self.times, stamp_ns)
        if i < len(self.rows) and self.times[i] == stamp_ns:
            rows = self.rows[i:i+1]
        else:
            rows = self.rows[max(0, i-1):i+1]
        if any(p.stamp_ns in self.ambiguous for p in rows):
            raise ValueError("AMBIGUOUS_POSE_STAMP")
        return interpolate_body_pose(rows, stamp_ns)


def project_to_polyline(point_xy_m: np.ndarray, path_xy_m: np.ndarray) -> dict[str, Any]:
    """Point [2] and ordered path [N,2] in metres; signed distance is left positive."""
    p, xy = np.asarray(point_xy_m, float), np.asarray(path_xy_m, float)
    if (p.shape != (2,) or xy.ndim != 2 or xy.shape[1] != 2 or len(xy) < 2
            or not np.isfinite(p).all() or not np.isfinite(xy).all()):
        raise ValueError("POLYLINE_SHAPE_OR_NONFINITE")
    delta = np.diff(xy, axis=0)
    length2 = np.sum(delta**2, axis=1)
    usable = length2 > 1e-12
    if not usable.any():
        raise ValueError("POLYLINE_STATIONARY")
    fraction = np.clip(np.sum((p-xy[:-1])*delta, axis=1)/np.maximum(length2, 1e-12), 0., 1.)
    nearest = xy[:-1]+fraction[:, None]*delta
    distance = np.linalg.norm(p-nearest, axis=1)
    i = int(np.argmin(np.where(usable, distance, np.inf)))
    cross = delta[i, 0]*(p[1]-nearest[i, 1])-delta[i, 1]*(p[0]-nearest[i, 0])
    return {"distance_m": float(distance[i]), "left_distance_m": float(np.sign(cross)*distance[i]),
            "nearest_xy_m": nearest[i].tolist(), "segment_index": i, "fraction": float(fraction[i])}


def scan_margin(scan: dict[str, Any], sensor: Sequence[float], *, speed_mps: float,
                measured_rad: float, issued_rad: float, previous_rad: float,
                yaw_rate_radps: float, lateral_mps: float,
                clearance_profile: str = STANDARD_CLEARANCE) -> dict[str, Any]:
    """Original LiDAR ranges [N]; exact existing monitor plus signed rejected margin.

    Validation and motion contracts stay in the production guard. A negative
    margin is a ray-distance difference, not a physical collision measurement.
    """
    kwargs = dict(speed_mps=speed_mps, measured_steer_rad=measured_rad,
                  issued_steer_rad=issued_rad, previous_steer_rad=previous_rad,
                  scan_in_current_rear=tuple(sensor), envelope_policy="curvature_support_v2",
                  vehicle_model_policy=AWSIM_POLICY, heading_rate_radps=yaw_rate_radps,
                  reported_lateral_mps=lateral_mps, clearance_profile=clearance_profile)
    try:
        result = check_turning_scan(np.asarray(scan["ranges"]), scan["angle_min"],
            scan["angle_increment"], scan["range_min"], scan["range_max"], **kwargs)
        return {"reason": "PASS", "minimum_ray_margin_m": result["minimum_ray_margin_m"]}
    except ValueError as exc:
        if str(exc) != "STOPPING_SWEEP_OCCUPIED":
            return {"reason": str(exc), "minimum_ray_margin_m": None}
    motion = stopping_motion(speed_mps, measured_rad, issued_rad, previous_rad,
        policy=AWSIM_POLICY, heading_rate_radps=yaw_rate_radps, reported_lateral_mps=lateral_mps)
    v = max(0., speed_mps)
    reserve, _ = clearance_dimensions(clearance_profile)
    n, h, _ = curvature_support_envelope(*motion["curvature_interval_per_m"], reserve+.5*v+v*v/2,
        scan["angle_increment"], np.asarray(sensor[:2]),
        lateral_padding_m=motion["lateral_displacement_bound_m"], clearance_profile=clearance_profile)
    a = scan["angle_min"]+np.arange(len(scan["ranges"]))*scan["angle_increment"]+sensor[2]
    projected = n @ np.column_stack([np.cos(a), np.sin(a)]).T
    remaining = h-n @ np.asarray(sensor[:2])
    parallel = abs(projected) < 1e-12
    intersections = remaining[:, None]/np.where(parallel, 1., projected)
    near = np.where(projected < -1e-12, intersections, -np.inf).max(axis=0)
    far = np.where(projected > 1e-12, intersections, np.inf).min(axis=0)
    outside = (parallel & (remaining[:, None] < 0)).any(axis=0)
    required = np.where(~outside & (far >= np.maximum(near, 0.)), far, 0.)
    ranges = np.asarray(scan["ranges"], dtype=float)
    margins = np.where(required > 0, np.minimum(ranges, scan["range_max"])-required, np.inf)
    return {"reason": "STOPPING_SWEEP_OCCUPIED", "minimum_ray_margin_m": float(margins.min())}
