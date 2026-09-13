"""Distance-resolved validation of unchanged 0.1 s time waypoints.

This is a bounded AWSIM acceptance policy, not a collision or feasibility proof.
The 3 cm residual budget is explicit; it is not a calibrated sensor uncertainty.
Short segments do not define reliable headings. All raw points still undergo
tube and backtracking checks, including the origin and unresolved final tail.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

import numpy as np


@dataclass(frozen=True)
class TimeGeometryConfig:
    position_budget_m: float = .03
    heading_baseline_m: float = .25
    max_step_m: float = .6
    wheelbase_m: float = 1.087
    max_steer_rad: float = .5

    def __post_init__(self) -> None:
        values = tuple(vars(self).values())
        if (not np.isfinite(values).all() or not 0 < self.position_budget_m <= .03
                or not 4 * self.position_budget_m < self.heading_baseline_m <= .5
                or not 0 < self.max_step_m <= .6 or self.wheelbase_m <= 0
                or not 0 < self.max_steer_rad <= .5):
            raise ValueError("TIME_GEOMETRY_CONFIG")


def validate_time_geometry(xy_m: np.ndarray, config: TimeGeometryConfig = TimeGeometryConfig()) -> dict[str, Any]:
    """Check finite [30,2] metres without altering timing, values, or control path.

    Chords are used ONLY for validation. Within each chord, raw points must
    remain within the positional budget plus the maximum-steering arc sagitta,
    and cannot backtrack by more than the budget. Resolved headings must agree
    with the initial vehicle heading and bounded-curvature changes, including
    angular uncertainty of endpoints. A non-resolved path never authorizes motion.
    """
    xy = np.asarray(xy_m, dtype=float)
    if xy.shape != (30, 2) or not np.isfinite(xy).all():
        raise ValueError("TIME_PATH_SHAPE_FINITE")
    points = np.vstack((np.zeros((1, 2)), xy))
    if np.linalg.norm(np.diff(points, axis=0), axis=1).max() > config.max_step_m:
        raise ValueError("TIME_PATH_STEP_DISCONTINUITY")
    budget = config.position_budget_m
    curvature = math.tan(config.max_steer_rad) / config.wheelbase_m
    indices = [0]
    for i in range(1, len(points)):
        if np.linalg.norm(points[i] - points[indices[-1]]) >= config.heading_baseline_m:
            indices.append(i)
    if indices[-1] != 30 and np.linalg.norm(points[-1] - points[indices[-1]]) > 2 * budget:
        indices.append(30)
    max_deviation = max_backtrack = max_turn = 0.
    previous_heading = previous_half_arc = previous_uncertainty = None
    for a, b in zip(indices, indices[1:]):
        vector = points[b] - points[a]
        length = float(np.linalg.norm(vector))
        direction = vector / length
        chunk = points[a:b+1] - points[a]
        along = chunk @ direction
        backtrack = float(np.max(np.maximum.accumulate(along) - along))
        max_backtrack = max(max_backtrack, backtrack)
        if along.min() < -budget or along.max() > length + budget or backtrack > budget:
            raise ValueError("TIME_PATH_BACKTRACK")
        ratio = curvature * length / 2
        if ratio >= 1:
            raise ValueError("TIME_PATH_CHORD_TOO_LONG")
        half_arc = math.asin(ratio)
        sagitta = curvature * length * length / (4 * (1 + math.sqrt(1 - ratio * ratio)))
        deviation = float(np.max(np.abs(chunk[:, 0] * direction[1] - chunk[:, 1] * direction[0])))
        max_deviation = max(max_deviation, deviation)
        if deviation > budget + sagitta:
            raise ValueError("TIME_PATH_TUBE_DEVIATION")
        heading = math.atan2(vector[1], vector[0])
        uncertainty = math.asin(min(1., (budget if a == 0 else 2 * budget) / length))
        if previous_heading is None:
            if abs(heading) > half_arc + uncertainty:
                raise ValueError("TIME_PATH_INITIAL_DIRECTION")
        else:
            turn = abs(math.atan2(math.sin(heading - previous_heading), math.cos(heading - previous_heading)))
            max_turn = max(max_turn, turn)
            if turn > previous_half_arc + half_arc + previous_uncertainty + uncertainty:
                raise ValueError("TIME_PATH_RESOLVED_TURN")
        previous_heading, previous_half_arc, previous_uncertainty = heading, half_arc, uncertainty
    if indices[-1] != 30:
        tail = points[indices[-1]:] - points[indices[-1]]
        if previous_heading is None:
            if np.linalg.norm(tail, axis=1).max() > budget:
                raise ValueError("TIME_PATH_UNRESOLVED_EXCURSION")
        else:
            direction = np.array([math.cos(previous_heading), math.sin(previous_heading)])
            along = tail @ direction
            backtrack = float(np.max(np.maximum.accumulate(along) - along))
            max_backtrack = max(max_backtrack, backtrack)
            if along.min() < -budget or backtrack > budget:
                raise ValueError("TIME_PATH_BACKTRACK")
            deviation = np.abs(tail[:, 0] * direction[1] - tail[:, 1] * direction[0])
            # Last short tail is checked against the last resolved heading.
            allowed = budget + np.linalg.norm(tail, axis=1) * math.sin(previous_uncertainty) + curvature * np.linalg.norm(tail, axis=1)**2
            max_deviation = max(max_deviation, float(deviation.max()))
            if np.any(deviation > allowed):
                raise ValueError("TIME_PATH_TUBE_DEVIATION")
    return {"version": "distance_resolved_time_geometry_v2", "motion_resolved": len(indices) > 1,
            "validation_indices": indices, "raw_points_checked": 31,
            "position_budget_m": budget, "heading_baseline_m": config.heading_baseline_m,
            "max_curvature_per_m": curvature, "max_tube_deviation_m": max_deviation,
            "max_backtrack_m": max_backtrack, "max_resolved_turn_rad": max_turn}
