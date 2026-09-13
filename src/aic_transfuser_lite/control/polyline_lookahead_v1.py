"""Bounded PP targets on unchanged timed polylines, in current rear-axle SI units."""
from __future__ import annotations

import math
from typing import Any

import numpy as np


def unit_roots(coefficients: tuple[float, float, float]) -> list[float]:
    """Real roots of a*u²+b*u+c inside [0,1], including linear/tangent cases."""
    a, b, c = coefficients
    if not all(math.isfinite(v) for v in coefficients):
        raise ValueError('NONFINITE_POLYNOMIAL')
    scale = max(1., abs(a), abs(b), abs(c))
    if abs(a) <= 1e-14*scale:
        roots = [] if abs(b) <= 1e-14*scale else [-c/b]
    else:
        discriminant = b*b-4*a*c
        if discriminant < -1e-14*scale*scale:
            return []
        if abs(discriminant) <= 1e-14*scale*scale:
            # A cancelled tangency must not become a fictitious finite interval.
            roots = [-b/(2*a)]
        else:
            square = math.sqrt(discriminant)
            q = -.5*(b+math.copysign(square, b))
            roots = [q/a, c/q]
    return sorted({max(0., min(1., u)) for u in roots if -1e-12 <= u <= 1.+1e-12})


def segment_intervals(a: np.ndarray, b: np.ndarray, minimum_m: float, maximum_m: float,
                      response_length_m: float, limit_rad: float | None) -> list[tuple[float, float]]:
    """Fractions on segment a->b satisfying forward/radial/optional angle bounds.

    Position in m; length in m; angles in rad. A 2e-12 coefficient-scaled
    numerical tolerance is used only to classify algebraic roots, not to
    authorize a float32 PP target. Zero-width tangencies are reported explicitly.
    """
    a = np.asarray(a, dtype=float); b = np.asarray(b, dtype=float)
    if (a.shape != (2,) or b.shape != (2,) or not np.isfinite([a, b]).all()
            or not all(math.isfinite(v) for v in (minimum_m, maximum_m, response_length_m))
            or not 0 < minimum_m < maximum_m or response_length_m <= 0
            or (limit_rad is not None and (not math.isfinite(limit_rad) or not 0 < limit_rad < math.pi/2))):
        raise ValueError('INVALID_SEGMENT_AUDIT')
    delta = b-a
    aa = float(delta@delta); bb = float(2*a@delta); cc = float(a@a)
    constraints = [(aa, bb, cc-minimum_m**2), (-aa, -bb, maximum_m**2-cc),
                   (0., float(delta[0]), float(a[0]-1e-6))]
    if limit_rad is not None:
        tangent = math.tan(limit_rad)
        for sign in (-1., 1.):
            constraints.append((tangent*aa, tangent*bb+sign*2*response_length_m*float(delta[1]),
                                tangent*cc+sign*2*response_length_m*float(a[1])))
    roots = sorted({0., 1., *(u for polynomial in constraints for u in unit_roots(polynomial))})

    def admitted(u: float) -> bool:
        return all((p*u+q)*u+r >= -2e-12*max(1., abs(p), abs(q), abs(r)) for p, q, r in constraints)

    intervals = [(left, right) for left, right in zip(roots, roots[1:])
                 if right > left and admitted((left+right)/2)]
    for u in roots:
        if admitted(u) and not any(left-1e-12 <= u <= right+1e-12 for left, right in intervals):
            intervals.append((u, u))
    return sorted(intervals)


def select_polyline_lookahead(points_m: np.ndarray, remaining_s: np.ndarray, *,
                             minimum_m: float, maximum_m: float,
                             response_length_m: float, limit_rad: float = .3) -> dict[str, Any]:
    """Prefer an existing vertex, otherwise the first feasible segment midpoint.

    points_m is [N,2] in the current rear-axle frame (m), remaining_s is [N]
    strictly increasing nonnegative time (s); index zero is the current-time
    interpolated plan point. No extrapolation, point removal or path mutation.
    All selected targets are checked in PP's actual float32 representation.
    Algebraic tolerance classifies roots only; it never relaxes admission.
    """
    points = np.asarray(points_m, dtype=float)
    times = np.asarray(remaining_s, dtype=float)
    if (points.ndim != 2 or points.shape[1:] != (2,) or not 2 <= len(points) <= 31
            or times.shape != (len(points),) or not np.isfinite(points).all()
            or not np.isfinite(times).all() or times[0] != 0. or times[-1] > 3.
            or not (np.diff(times) > 0).all()):
        raise ValueError('INVALID_LOOKAHEAD_REFERENCE')
    if (not all(math.isfinite(v) for v in (minimum_m, maximum_m, response_length_m, limit_rad))
            or not 0 < minimum_m < maximum_m or response_length_m <= 0 or not 0 < limit_rad < math.pi/2):
        raise ValueError('INVALID_LOOKAHEAD_BOUNDS')

    def selected(point: np.ndarray, kind: str, a: int, b: int, fraction: float,
                 interval: tuple[float, float]) -> dict[str, Any] | None:
        target = point.astype(np.float32)
        x, y = map(float, target)
        squared = x*x+y*y
        distance = math.sqrt(squared)
        angle = math.atan(response_length_m*2*y/max(squared, 1e-6))
        if not (x > 1e-6 and minimum_m <= distance <= maximum_m and abs(angle) <= limit_rad):
            return None
        return {'kind': kind, 'reference_indices': [a, b], 'fraction': fraction,
                'fraction_interval': list(interval), 'xy_m': [x, y],
                'remaining_s': float(times[a]+fraction*(times[b]-times[a])),
                'reference_interval_s': [float(times[a]), float(times[b])],
                'distance_m': distance, 'required_tire_rad': angle,
                'steering_margin_rad': limit_rad-abs(angle),
                'distance_margin_m': min(distance-minimum_m, maximum_m-distance)}

    # Keep every existing admitted vertex and its order, including its float32
    # representation. Interpolation is only a fallback after all vertices fail.
    for i in range(1, len(points)):
        if points[i, 0] > 1e-6:
            result = selected(points[i], 'vertex', i, i, 0., (0., 0.))
            if result is not None:
                return result
    # Use original adjacency, including the interpolated point at time zero.
    # Never connect across a rejected/backward intermediate point.
    for i, (a, b) in enumerate(zip(points, points[1:])):
        for left, right in segment_intervals(a, b, minimum_m, maximum_m, response_length_m, limit_rad):
            middle = (left+right)/2
            result = selected(a+middle*(b-a), 'segment', i, i+1, middle, (left, right))
            if result is not None:
                return result
    raise ValueError('STEERING_FEASIBLE_LOOKAHEAD_MISSING')
