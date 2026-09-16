"""Observed speed-step decision; does not authorize vehicle commands."""
from __future__ import annotations

from typing import Any
import numpy as np


def assess_speed_step(target_kmh: float, lap_completed: bool, speed_samples_mps: np.ndarray) -> dict[str, Any]:
    """Use the user's lap-completion criterion; report speed attainment separately.

    Samples [N] are actual command-time measurements during the active trial,
    including temporary braking. Speeds <=0.1 m/s exclude standing/launch noise.
    A moving median >=90% of target is a separate diagnostic, not another lap
    acceptance condition. This single run does not establish repeatability.
    """
    samples = np.asarray(speed_samples_mps, dtype=float)
    if (not np.isfinite(target_kmh) or target_kmh < 5 or target_kmh % 5 != 0
            or type(lap_completed) is not bool or samples.ndim != 1
            or not np.isfinite(samples).all() or np.any(samples < -.03)):
        raise ValueError('SPEED_STEP_INPUT')
    moving = samples[samples > .1]*3.6
    median = float(np.median(moving)) if len(moving) else None
    threshold = .9*target_kmh
    reached = median is not None and median >= threshold
    status = 'PASS' if lap_completed else 'LAP_NOT_COMPLETED'
    return dict(status=status, target_kmh=target_kmh, lap_completed=lap_completed,
                moving_samples=len(moving), moving_median_kmh=median,
                maximum_measured_kmh=float(samples.max()*3.6) if len(samples) else None,
                speed_attainment_reference_kmh=threshold, target_speed_reached=reached,
                next_target_kmh=target_kmh+5 if status == 'PASS' else None,
                scope='ONE_OBSERVED_STEP_NOT_REPEATABILITY_OR_BRAKING_VALIDATION')
