"""Exact polyline/PP feasibility audit; offline only, no controller replacement.

Segments retain their original endpoints and time order. Quadratic boundary
roots partition each segment, so a narrow feasible interval is not missed by
a chosen resampling step. A returned midpoint is rechecked after float32 cast.
"""
from __future__ import annotations

import math
from typing import Any

import numpy as np

from ..control.polyline_lookahead_v1 import segment_intervals, unit_roots


def angle_for_point(point: np.ndarray, response_length_m: float) -> float:
    """The same scalar PP angle formula as the controller, without clipping."""
    x, y = map(float, point)
    return math.atan(response_length_m*2*y/max(x*x+y*y, 1e-6))


def audit_polyline(points_m: np.ndarray, remaining_s: np.ndarray, *, minimum_m: float,
                   maximum_m: float, response_length_m: float, limit_rad: float = .3) -> dict[str, Any]:
    """Analyze [N,2] current-rear-frame m and strictly increasing [N] future s.

    Index zero is the interpolated current-time plan point, which the existing
    discrete selector drops. Continuous intervals follow adjacent original
    points, without shortcuts across rejected/nonforward points or extrapolation.
    """
    points = np.asarray(points_m, dtype=float); times = np.asarray(remaining_s, dtype=float)
    if (points.ndim != 2 or points.shape[1:] != (2,) or len(points) < 2
            or times.shape != (len(points),) or not np.isfinite(points).all() or not np.isfinite(times).all()
            or times[0] < -1e-12 or not (np.diff(times) > 0).all()):
        raise ValueError('INVALID_TIMED_POLYLINE')
    vertices = []
    for index, point in enumerate(points[1:], 1):
        target = point.astype(np.float32)
        distance = math.hypot(*map(float, target))
        angle = angle_for_point(target, response_length_m)
        vertices.append({'index': index, 'remaining_s': float(times[index]), 'xy_m': point.tolist(),
            'distance_m': distance, 'angle_rad': angle,
            'in_band': bool(target[0] > 1e-6 and minimum_m <= distance <= maximum_m),
            'angle_admitted': abs(angle) <= limit_rad,
            'float32_angle_error_rad': angle-angle_for_point(point, response_length_m)})
    feasible = []
    for index, (a, b) in enumerate(zip(points, points[1:])):
        for left, right in segment_intervals(a, b, minimum_m, maximum_m, response_length_m, limit_rad):
            middle = (left+right)/2
            point = a+middle*(b-a)
            target = point.astype(np.float32)
            distance = math.hypot(*map(float, target))
            angle = angle_for_point(target, response_length_m)
            strict = target[0] > 1e-6 and minimum_m <= distance <= maximum_m and abs(angle) <= limit_rad
            endpoint = [a+u*(b-a) for u in (left, right)]
            feasible.append({'segment_index': index, 'fraction_interval': [left, right],
                'remaining_s_interval': [float(times[index]+u*(times[index+1]-times[index])) for u in (left, right)],
                'endpoint_radius_m': [float(np.linalg.norm(p)) for p in endpoint],
                'interval_length_m': float((right-left)*np.linalg.norm(b-a)),
                'midpoint_xy_m': point.tolist(), 'midpoint_float32_xy_m': target.astype(float).tolist(),
                'midpoint_distance_m': distance, 'midpoint_angle_rad': angle,
                'midpoint_margin_rad': limit_rad-abs(angle), 'midpoint_strict_float32_pass': bool(strict)})
    in_band = [p for p in vertices if p['in_band']]
    selected = next((p for p in vertices if p['in_band'] and p['angle_admitted']), None)
    outside = next((p for p in vertices if p['xy_m'][0] > 1e-6 and p['distance_m'] > maximum_m and p['angle_admitted']), None)
    floating64 = [p for p in points[1:] if p[0] > 1e-6 and minimum_m <= np.linalg.norm(p) <= maximum_m]
    return {'preview_band_m': [minimum_m, maximum_m], 'response_length_m': response_length_m,
        'vertices': vertices, 'discrete_selected': selected, 'discrete_candidate_count': len(in_band),
        'discrete_best_abs_angle_rad': min(abs(p['angle_rad']) for p in in_band) if in_band else None,
        'float64_vertex_pass': any(abs(angle_for_point(p, response_length_m)) <= limit_rad for p in floating64),
        'maximum_float32_angle_error_rad': max(abs(p['float32_angle_error_rad']) for p in vertices),
        'continuous_intervals': feasible,
        'continuous_midpoint': next((p for p in feasible if p['midpoint_strict_float32_pass']), None),
        'first_feasible_vertex_beyond_band': outside,
        'maximum_original_radius_m': float(np.linalg.norm(points[1:], axis=1).max())}
