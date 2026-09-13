"""Bounded, spatially decimated RViz trace. Never a teacher or control input."""
from __future__ import annotations

from collections import deque
import math
from typing import Sequence


class ObservedPathTrace:
    """Keep original map XY metres at >= spacing_m for display only.

    A backwards simulation stamp starts a new display epoch. Raw odometry is
    recorded independently at its original rate, without this decimation.
    """
    def __init__(self, *, spacing_m: float = .1, max_points: int = 10000) -> None:
        if not math.isfinite(spacing_m) or spacing_m <= 0 or type(max_points) is not int or max_points < 2:
            raise ValueError('DISPLAY_TRACE_CONFIG')
        self.spacing_m = spacing_m
        self.points: deque[tuple[float, float]] = deque(maxlen=max_points)
        self.last_stamp_ns: int | None = None

    def add(self, stamp_ns: int, xy_m: Sequence[float]) -> None:
        if (type(stamp_ns) is not int or stamp_ns < 0 or len(xy_m) != 2
                or not all(math.isfinite(v) for v in xy_m)):
            raise ValueError('DISPLAY_POSE_CONTRACT')
        if self.last_stamp_ns is not None and stamp_ns < self.last_stamp_ns:
            self.points.clear()
        if stamp_ns == self.last_stamp_ns:
            return
        self.last_stamp_ns = stamp_ns
        xy = (float(xy_m[0]), float(xy_m[1]))
        if not self.points or math.dist(xy, self.points[-1]) >= self.spacing_m:
            self.points.append(xy)
