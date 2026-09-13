"""Steering-aware forward-LiDAR proximity monitor for bounded AWSIM trials.

SI units throughout. Sweeps the inflated kart rectangle over stopping travel,
using an interval between measured and issued steering. Tests the observed rays
against that sweep, including occluded portions. This is NOT all-around free
space certification: a forward scanner cannot observe current rear/side space.
The 0.5 s delay and 1 m/s^2 effective braking remain trial assumptions.
"""
from __future__ import annotations

import math
from typing import Any, Sequence

import numpy as np

from .time_reference_v1 import TimedBodyPose
from .time_trial_v1 import interpolate_body_pose
from .vehicle_motion_v1 import IDEAL_POLICY, stopping_motion


def select_aligned_scan(capture_receipts_ns: Sequence[tuple[int, int]], poses: Sequence[TimedBodyPose],
                        current: TimedBodyPose, *, now_sim_ns: int, now_receipt_ns: int) -> tuple[int, TimedBodyPose]:
    """Newest fresh scan with a bracketed capture pose; never extrapolate/relabel.

    Candidate pairs are (original capture in sim ns, callback in monotonic ns).
    A just-arrived scan may precede the next pose callback. Use an older scan
    only while it still satisfies the existing 150 ms capture / 300 ms receipt
    limits. The runtime bounds the history to four scans and clears on reset.
    """
    for index in sorted(range(len(capture_receipts_ns)), key=lambda i: capture_receipts_ns[i][0], reverse=True):
        captured, received = capture_receipts_ns[index]
        if (captured > current.stamp_ns or not -20_000_000 <= now_sim_ns-captured <= 150_000_000
                or not 0 <= now_receipt_ns-received <= 300_000_000):
            continue
        try:
            pose = interpolate_body_pose(poses, captured)
        except ValueError as exc:
            if str(exc) != "OBSERVATION_POSE_MISSING":
                raise
            continue
        return index, pose
    raise ValueError("FRESH_ALIGNED_SCAN_MISSING")


def scan_pose_in_rear(captured: TimedBodyPose, current: TimedBodyPose,
                      rear_axle_forward_m: float) -> tuple[float, float, float]:
    """Original scan-header base_link pose -> current rear axle, no restamping."""
    if ((captured.clock, captured.epoch, captured.world_frame, captured.body_frame) !=
            (current.clock, current.epoch, current.world_frame, current.body_frame)
            or current.body_frame != "base_link" or not 0 <= current.stamp_ns-captured.stamp_ns <= 150_000_000
            or not math.isfinite(rear_axle_forward_m) or abs(rear_axle_forward_m) > .002):
        raise ValueError("SCAN_POSE_IDENTITY_OR_AGE")
    lidar = 1.649999976158142
    dx = captured.x_m + lidar*math.cos(captured.yaw_rad)-current.x_m
    dy = captured.y_m + lidar*math.sin(captured.yaw_rad)-current.y_m
    c, s = math.cos(current.yaw_rad), math.sin(current.yaw_rad)
    yaw = math.atan2(math.sin(captured.yaw_rad-current.yaw_rad), math.cos(captured.yaw_rad-current.yaw_rad))
    return dx*c+dy*s-rear_axle_forward_m, -dx*s+dy*c, yaw


def stopping_sweep(speed_mps: float, measured_steer_rad: float, issued_steer_rad: float,
                   previous_steer_rad: float | None = None, *, vehicle_model_policy: str = IDEAL_POLICY,
                   heading_rate_radps: float | None = None,
                   reported_lateral_mps: float | None = None) -> tuple[np.ndarray, np.ndarray]:
    """Return rear-axle poses [S,3] and isotropic inflation [S], in metres/rad.

    All varying curvatures between the two steering values are enclosed by
    positional error <= half_curvature_span*s^2/2 and heading error <= span*s.
    Sampling inflation also covers motion between adjacent pose samples.
    """
    motion = stopping_motion(speed_mps, measured_steer_rad, issued_steer_rad, previous_steer_rad,
        policy=vehicle_model_policy, heading_rate_radps=heading_rate_radps, reported_lateral_mps=reported_lateral_mps)
    return _sweep_from_motion(speed_mps, motion)


def _sweep_from_motion(speed_mps: float, motion: dict[str, Any]) -> tuple[np.ndarray, np.ndarray]:
    speed = max(0., speed_mps)
    travel = .4 + speed*.5 + speed**2/2.
    k = np.asarray(motion["curvature_interval_per_m"])
    middle, half_span = float((k.min()+k.max())/2.), float((k.max()-k.min())/2.)
    distances = np.linspace(0., travel, int(math.ceil(travel/.025))+1)
    yaw = middle*distances
    if abs(middle) < 1e-10:
        xy = np.column_stack([distances, np.zeros_like(distances)])
    else:
        xy = np.column_stack([np.sin(yaw)/middle, (1.-np.cos(yaw))/middle])
    # Existing root-frame front=1.5 -> rear-axle front=1.984; rear overhang=.510.
    # Existing +/- .85 m half-width includes the 1.30 m kart plus .20 m each side.
    radius = math.hypot(1.984, .85)
    between_samples = (distances[1]-distances[0])/2.*(1.+radius*float(max(abs(k))))
    inflation = (half_span*(distances**2/2.+radius*distances)+between_samples
                 + motion["lateral_displacement_bound_m"])
    return np.column_stack([xy, yaw]), inflation


def check_turning_scan(ranges: np.ndarray, angle_min: float, angle_increment: float,
                       range_min: float, range_max: float, *, speed_mps: float,
                       measured_steer_rad: float, issued_steer_rad: float,
                       scan_in_current_rear: tuple[float, float, float],
                       previous_steer_rad: float | None = None,
                       envelope_policy: str = "isotropic_v1", vehicle_model_policy: str = IDEAL_POLICY,
                       heading_rate_radps: float | None = None,
                       reported_lateral_mps: float | None = None) -> dict[str, Any]:
    """Scan ranges [N] at its original capture pose, expressed in current rear.

    scan_in_current_rear=(forward_m,left_m,yaw_rad), obtained using time-aligned
    poses and the calibrated +1.65 m base_link LiDAR offset. +inf is an AWSIM
    no-return ray, usable only to range_max. NaN/-inf/out-of-range reject. The
    old +/-1.3 rad required FOV is preserved; all supplied rays are checked.
    """
    r = np.asarray(ranges, dtype=float)
    if envelope_policy not in ("isotropic_v1", "curvature_support_v2"):
        raise ValueError("SCAN_ENVELOPE_POLICY")
    if (r.ndim != 1 or not 100 <= len(r) <= 4096
            or len(scan_in_current_rear) != 3
            or not np.isfinite([angle_min, angle_increment, range_min, range_max, *scan_in_current_rear]).all()
            or not 0 < angle_increment <= .02 or not 0 <= range_min < range_max <= 100.
            or (len(r)-1)*angle_increment > 2*math.pi+.02):
        raise ValueError("SCAN_CONTRACT")
    angles = angle_min+np.arange(len(r))*angle_increment
    if angles.min() > -1.3 or angles.max() < 1.3 or (abs(angles) <= 1.3).sum() < 100:
        raise ValueError("SCAN_COVERAGE")
    if np.any(np.isnan(r) | np.isneginf(r) | (r < range_min) | (np.isfinite(r) & (r > range_max))):
        raise ValueError("SCAN_UNKNOWN")
    motion = stopping_motion(speed_mps, measured_steer_rad, issued_steer_rad, previous_steer_rad,
        policy=vehicle_model_policy, heading_rate_radps=heading_rate_radps, reported_lateral_mps=reported_lateral_mps)
    poses, inflation = _sweep_from_motion(speed_mps, motion)
    sensor = np.asarray(scan_in_current_rear, dtype=float)
    if np.linalg.norm(sensor[:2]-[1.649, 0.]) > .35 or abs(sensor[2]) > .15:
        raise ValueError("SCAN_POSE_ALIGNMENT")
    if envelope_policy == "curvature_support_v2":
        from .curvature_support_v2 import check_support_ranges
        return check_support_ranges(r, angles, range_max, angle_increment, sensor, speed_mps,
                                    measured_steer_rad, issued_steer_rad, previous_steer_rad, motion=motion)
    # Include angular gaps conservatively in the rectangle inflation. No
    # obstacle behind a hit is declared free solely because that hit lies off
    # the centerline of the predicted path.
    reach = float(np.linalg.norm(poses[:, :2], axis=1).max())+2.2+float(inflation.max())
    inflation += reach*angle_increment
    c, s = np.cos(poses[:, 2]), np.sin(poses[:, 2])
    delta = sensor[:2]-poses[:, :2]
    origins = np.column_stack([delta[:, 0]*c+delta[:, 1]*s, -delta[:, 0]*s+delta[:, 1]*c])
    ray_angle = angles[None, :]+sensor[2]-poses[:, 2, None]
    directions = np.stack([np.cos(ray_angle), np.sin(ray_angle)], axis=-1)
    lo = np.column_stack([-.510-inflation, -.85-inflation])
    hi = np.column_stack([1.984+inflation, .85+inflation])
    parallel = abs(directions) < 1e-12
    safe_directions = np.where(parallel, 1., directions)
    a = (lo[:, None, :]-origins[:, None, :])/safe_directions
    b = (hi[:, None, :]-origins[:, None, :])/safe_directions
    enter, leave = np.minimum(a, b), np.maximum(a, b)
    enter = np.where(parallel, -np.inf, enter)
    leave = np.where(parallel, np.inf, leave)
    outside = np.any(parallel & ((origins[:, None, :] < lo[:, None, :]) | (origins[:, None, :] > hi[:, None, :])), axis=-1)
    near, far = enter.max(axis=-1), leave.min(axis=-1)
    intersects = ~outside & (far >= np.maximum(near, 0.))
    required = np.max(np.where(intersects, far, 0.), axis=0)
    observed = required > 0.
    margins = np.minimum(r, range_max)-required
    if np.any(margins[observed] <= 0.):
        raise ValueError("STOPPING_SWEEP_OCCUPIED")
    return {"policy": "STEERING_INTERVAL_SWEEP_V1", "vehicle_motion": motion,
        "scope": "FORWARD_SCAN_PROXIMITY_NOT_ALL_AROUND_FREE_SPACE",
        "minimum_ray_margin_m": float(margins[observed].min()) if observed.any() else float(range_max),
        "checked_rays": int(observed.sum()), "sweep_samples": len(poses),
        "stopping_travel_m": .4+max(0., speed_mps)*.5+speed_mps**2/2.,
        "measured_steer_rad": measured_steer_rad, "issued_steer_rad": issued_steer_rad,
        "previous_steer_rad": previous_steer_rad,
        "maximum_sweep_inflation_m": float(inflation.max()), "full_body_free_space_verified": False}
