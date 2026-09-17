"""Shared velocity-scaled PP horizon, separate from obstacle stopping distance.

The original timed polyline remains the only source of steering targets.
This contract controls preview coverage, not collision-free braking coverage.
"""
from __future__ import annotations

import math


TIME_LOOKAHEAD_POLICY = 'velocity_time_preview_v1'
PREVIEW_TIME_S = 1.5
PREVIEW_OFFSET_M = .4
MINIMUM_PREVIEW_M = 1.


def time_preview_distance(speed_mps: float) -> float:
    """Required radial PP distance in metres for finite nonnegative m/s."""
    if not math.isfinite(speed_mps) or speed_mps < 0:
        raise ValueError('TIME_PREVIEW_SPEED')
    return max(MINIMUM_PREVIEW_M, PREVIEW_OFFSET_M + PREVIEW_TIME_S * speed_mps)


def time_horizon_speed_cap(endpoint_distance_m: float, *, reserve_m: float,
                           extra_delay_s: float) -> float:
    """Invert preview(v) + reserve + extra_delay*v <= endpoint, returning m/s.

    Both branches of max(1 m, .4 m + 1.5 s*v) must fit. A horizon shorter
    than the zero-speed requirement yields zero; it is never extrapolated.
    """
    if (not all(math.isfinite(v) for v in (endpoint_distance_m, reserve_m, extra_delay_s))
            or endpoint_distance_m < 0 or reserve_m < 0 or extra_delay_s <= 0):
        raise ValueError('TIME_PREVIEW_HORIZON')
    usable_m = endpoint_distance_m - reserve_m
    return max(0., min((usable_m - PREVIEW_OFFSET_M) / (PREVIEW_TIME_S + extra_delay_s),
                       (usable_m - MINIMUM_PREVIEW_M) / extra_delay_s))
