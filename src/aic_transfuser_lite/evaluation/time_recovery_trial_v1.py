"""Measured recovery metrics; SI units relative to an observed nominal guide."""
from __future__ import annotations

import math
from typing import Any

import numpy as np


def recovery_window_metrics(samples: np.ndarray) -> dict[str, Any]:
    """Samples [N,4]: time since takeover s, lateral m, heading rad, speed m/s.

    A return requires <=5 cm and <=2 degrees for a continuous second while
    moving >=0.5 m/s. Gaps >250 ms break continuity. Stopping is not recovery.
    This measures return to the observed guide, not road-edge clearance.
    """
    x = np.asarray(samples, dtype=float)
    if (x.ndim != 2 or x.shape[1] != 4 or len(x) < 1 or not np.isfinite(x).all()
            or x[0, 0] < 0 or np.any(np.diff(x[:, 0]) <= 0)):
        raise ValueError('RECOVERY_METRIC_SHAPE_FINITE_TIME')
    x = x[x[:, 0] <= 10.]
    if not len(x):
        raise ValueError('RECOVERY_WINDOW_EMPTY')
    within = (abs(x[:, 1]) <= .05) & (abs(x[:, 2]) <= math.radians(2.)) & (x[:, 3] >= .5)
    began = None
    first_return = None
    for i, good in enumerate(within):
        if not good:
            began = None
        elif began is None or (i and x[i, 0] - x[i-1, 0] > .25):
            began = float(x[i, 0])
        elif x[i, 0] - began >= 1. and first_return is None:
            first_return = began
    initial_outside = bool(abs(x[0, 1]) > .05 or abs(x[0, 2]) > math.radians(2.))
    covered = bool(x[0, 0] <= .25 and x[-1, 0] >= 9.75 and np.max(np.diff(x[:, 0]), initial=0.) <= .25)
    return dict(samples=len(x), first_sample_s=float(x[0, 0]), final_sample_s=float(x[-1, 0]),
        initial_outside_return_tolerance=initial_outside,
        initial_lateral_m=float(x[0, 1]), initial_heading_deg=math.degrees(float(x[0, 2])),
        initial_speed_mps=float(x[0, 3]), max_abs_lateral_m=float(abs(x[:, 1]).max()),
        max_abs_heading_deg=math.degrees(float(abs(x[:, 2]).max())),
        final_lateral_m=float(x[-1, 1]), final_heading_deg=math.degrees(float(x[-1, 2])),
        first_sustained_return_s=first_return, ten_second_window_covered=covered,
        moving_within_tolerance_at_window_end=bool(within[-1]),
        recovered_in_ten_second_window=bool(initial_outside and covered and first_return is not None and within[-1]),
        tolerance=dict(lateral_m=.05, heading_deg=2., sustained_s=1., minimum_speed_mps=.5, maximum_gap_s=.25))
