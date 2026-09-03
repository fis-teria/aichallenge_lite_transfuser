from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from aic_transfuser_lite.control.executable_reference import (
    AuthoritativePlanV3,
    polyline_arc_length_m,
)


def _readonly(value: np.ndarray) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64).copy()
    result.setflags(write=False)
    return result


@dataclass(frozen=True)
class PlanConsistencyMetricsV3:
    """Trajectory/speed consistency metrics; speeds are m/s."""

    segment_length_m: np.ndarray
    geometric_speed_mps: np.ndarray
    trapezoidal_speed_mps: np.ndarray
    speed_residual_mps: np.ndarray
    mean_absolute_error_mps: float
    max_absolute_error_mps: float
    normalized_huber_mean: float

    def validate(self) -> None:
        count = len(np.asarray(self.segment_length_m))
        for name, values in (
            ("segment_length_m", self.segment_length_m),
            ("geometric_speed_mps", self.geometric_speed_mps),
            ("trapezoidal_speed_mps", self.trapezoidal_speed_mps),
            ("speed_residual_mps", self.speed_residual_mps),
        ):
            array = np.asarray(values)
            if array.shape != (count,) or not np.isfinite(array).all():
                raise ValueError(f"{name} must be finite [N]")
        scalars = (
            self.mean_absolute_error_mps,
            self.max_absolute_error_mps,
            self.normalized_huber_mean,
        )
        if not all(math.isfinite(value) and value >= 0.0 for value in scalars):
            raise ValueError("plan consistency scalar metrics must be finite and non-negative")


def evaluate_plan_consistency_v3(
    plan: AuthoritativePlanV3,
    *,
    current_speed_mps: float,
    speed_scale_mps: float = 1.0,
    huber_delta: float = 1.0,
) -> PlanConsistencyMetricsV3:
    """Compute ``E_plan`` using polyline distance and trapezoidal speed.

    Segment zero starts at ego origin at ``t=0``. Later segments use adjacent
    predicted waypoints, so curved paths are measured by polyline arc length
    instead of straight-line distance to the final endpoint.
    """

    plan.validate(require_stop_probability=False)
    current_speed = float(current_speed_mps)
    if not math.isfinite(current_speed) or current_speed < 0.0:
        raise ValueError("current_speed_mps must be finite and non-negative")
    if not math.isfinite(speed_scale_mps) or speed_scale_mps <= 0.0:
        raise ValueError("speed_scale_mps must be finite and positive")
    if not math.isfinite(huber_delta) or huber_delta <= 0.0:
        raise ValueError("huber_delta must be finite and positive")

    segment_length, _ = polyline_arc_length_m(plan.trajectory_xy_m)
    times = np.asarray(plan.waypoint_times_sec, dtype=np.float64)
    step_times = np.diff(np.concatenate(([0.0], times)))
    geometric_speed = segment_length / step_times
    predicted_speed = np.asarray(plan.speed_profile_mps, dtype=np.float64)
    previous_speed = np.concatenate(([current_speed], predicted_speed[:-1]))
    trapezoidal_speed = 0.5 * (previous_speed + predicted_speed)
    residual = geometric_speed - trapezoidal_speed
    normalized_absolute = np.abs(residual / speed_scale_mps)
    huber = np.where(
        normalized_absolute <= huber_delta,
        0.5 * normalized_absolute**2,
        huber_delta * (normalized_absolute - 0.5 * huber_delta),
    )
    metrics = PlanConsistencyMetricsV3(
        segment_length_m=_readonly(segment_length),
        geometric_speed_mps=_readonly(geometric_speed),
        trapezoidal_speed_mps=_readonly(trapezoidal_speed),
        speed_residual_mps=_readonly(residual),
        mean_absolute_error_mps=float(np.mean(np.abs(residual))),
        max_absolute_error_mps=float(np.max(np.abs(residual))),
        normalized_huber_mean=float(np.mean(huber)),
    )
    metrics.validate()
    return metrics
