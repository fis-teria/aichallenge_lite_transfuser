"""Observed-only spatial targets; no I/O, torch, raw reader or teacher promotion."""
from __future__ import annotations

import numpy as np

TEACHER_CONTRACT = {
    "version": "spatial_diagnostic_view_v4_v1", "tier": "OBSERVED_DIAGNOSTIC_ONLY",
    "frame": "base_link@t_obs", "rear_axle_alignment": "UNKNOWN", "xy_units": "m",
    "future_shape": [30, 8], "grid_m": [round(0.1 * k, 1) for k in range(1, 21)],
    "origin_scored": False, "extrapolation": False, "time_step_s": 0.1,
    "time_tolerance_s": 1e-5, "maximum_gap_s": 0.2, "jump_speed_mps": 20.0,
    "noise_radius_m": 0.005, "stationary_speed_mps": 0.01, "hold_s": 0.5,
    "corner_cut_max_m": 0.05, "corner_cut_max_fraction": 0.1,
    "threshold_provenance": "coverage v4 provisional 5mm/20mps/0.2s/0.5s; uncalibrated; corner cut diagnostic only",
    "mask_meaning": "observed teacher coverage, NOT predicted validity or motion permission",
}
GRID = np.asarray(TEACHER_CONTRACT["grid_m"], dtype=np.float64)


def polyline_length(xy: np.ndarray) -> float:
    return float(np.linalg.norm(np.diff(np.concatenate((np.zeros((1, 2)), xy)), axis=0), axis=1).sum())


def self_intersects(xy: np.ndarray) -> bool:
    points = np.concatenate((np.zeros((1, 2)), xy))
    def cross(a: np.ndarray, b: np.ndarray) -> float:
        return float(a[0] * b[1] - a[1] * b[0])
    for i in range(len(points) - 1):
        a, b = points[i:i + 2]
        for j in range(i + 2, len(points) - 1):
            c, d = points[j:j + 2]
            if cross(b-a, c-a) * cross(b-a, d-a) < -1e-12 and cross(d-c, a-c) * cross(d-c, b-c) < -1e-12:
                return True
    return False


def spatial_target(future: np.ndarray, frame: str = "base_link@t_obs") -> dict:
    """Convert [30,8] t,x,y,yaw,v_long,v_lat,yaw_rate,valid to XY[20,2]/mask[20].

    Only this anchor's contiguous prefix is used. Invalid original bytes are not
    modified. Origin assists geometry but is never a target. Thresholds are not
    safety criteria; a missing first point yields unknown distance, not zero.
    """
    source = np.asarray(future)
    if source.shape != (30, 8) or source.dtype.hasobject or frame != TEACHER_CONTRACT["frame"]:
        raise ValueError("future shape/dtype/frame mismatch")
    if not np.isin(source[:, 7], [0, 1]).all():
        raise ValueError("invalid mask values")
    points, times, speeds = [np.zeros(2)], [0.0], []
    cut = "stored_horizon_end"
    for i, row in enumerate(source.astype(np.float64)):
        if row[7] == 0:
            cut = "invalid_future"
            break
        if not np.isfinite(row[:7]).all():
            cut = "nonfinite_valid"
            break
        dt = float(row[0] - times[-1])
        if dt <= 0 or dt > 0.2 + 1e-7 or abs(row[0] - (i+1)*0.1) > 1e-5:
            cut = "invalid_time_grid_or_gap"
            break
        if np.linalg.norm(row[1:3] - points[-1]) / dt > 20.0:
            cut = "position_jump"
            break
        points.append(row[1:3].copy())
        times.append(float(row[0]))
        speeds.append(float(row[4]))
    xy = np.asarray(points)
    raw_length = float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum()) if speeds else None
    reliable = [xy[0]]
    for point in xy[1:]:
        if np.linalg.norm(point - reliable[-1]) >= 0.005:
            reliable.append(point)
    flags = []
    if speeds and max(abs(v) for v in speeds) <= 0.01 and np.linalg.norm(xy, axis=1).max() < 0.01:
        reliable = [xy[0]]
        flags.append("stationary_jitter_provisional")
    hold = maximum_hold = 0.0
    for dt, speed in zip(np.diff(times), speeds):
        hold = hold + dt if abs(speed) <= 0.01 else 0.0
        maximum_hold = max(maximum_hold, hold)
    if maximum_hold >= 0.5 - 1e-6:
        flags.append("long_hold_not_intent")
    if any(v < -0.01 for v in speeds):
        flags.append("reverse_motion")
    segments = np.diff(xy, axis=0)
    if len(segments) > 1 and np.any(np.sum(segments[:-1] * segments[1:], axis=1) < -0.000025):
        flags.append("direction_reversal")
    filtered = np.asarray(reliable)
    s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(filtered, axis=0), axis=1))]
    length = float(s[-1]) if speeds else None
    mask = GRID <= s[-1]
    target = np.zeros((20, 2), dtype=np.float32)
    if mask.any():
        target[mask] = np.stack([np.interp(GRID[mask], s, filtered[:, axis]) for axis in (0, 1)], axis=-1)
    resampled_length = polyline_length(target[mask])
    covered = float(GRID[mask][-1]) if mask.any() else 0.0
    corner_cut = max(0.0, covered - resampled_length)
    if corner_cut > max(0.05, 0.1 * covered):
        flags.append("large_resampling_corner_cut")
    if self_intersects(filtered[1:]):
        flags.append("observed_self_intersection")
    endpoint = xy[-1].tolist() if speeds else None
    shape = "unknown" if not mask.any() else "left" if target[mask][-1, 1] > 0.05 else "right" if target[mask][-1, 1] < -0.05 else "straight"
    return {"xy": target, "mask": mask, "grid_m": GRID.copy(), "raw_prefix_xy": xy,
        "prefix_count": len(speeds), "raw_length_m": raw_length, "processed_length_m": length,
        "resampled_length_m": resampled_length, "covered_grid_m": covered, "corner_cut_m": corner_cut,
        "endpoint_xy_m": endpoint, "cut_reason": cut, "flags": flags, "shape": shape,
        "maximum_hold_s": float(maximum_hold), "tier": "OBSERVED_DIAGNOSTIC_ONLY",
        "distance_status": "KNOWN_OBSERVED" if speeds else "UNKNOWN_FIRST_MISSING"}
