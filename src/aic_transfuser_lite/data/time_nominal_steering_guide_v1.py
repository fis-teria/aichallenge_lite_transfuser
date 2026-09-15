"""Teacher-only steering along measured nominal progress, then live PP recovery.

Guide shape [N,2]: reference progress m, ROS steering-input rad (not wheel rad).
The independently checked final command limiter and scan supervisor remain the
caller's responsibility. This is an AWSIM data-collection policy, not inference.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping

import numpy as np

POLICY = 'nominal_guide_then_pp_v1'


@dataclass(frozen=True)
class NominalSteeringGuide:
    values: np.ndarray

    def __post_init__(self) -> None:
        values = np.array(self.values, dtype=float, copy=True)
        if (values.ndim != 2 or values.shape[1] != 2 or len(values) < 2
                or not np.isfinite(values).all() or np.any(np.diff(values[:, 0]) <= 0.)
                or np.any(np.diff(values[:, 0]) > .25) or np.any(np.abs(values[:, 1]) > .5)):
            raise ValueError('STEERING_GUIDE_SHAPE_FINITE_GAPS_OR_BOUNDS')
        values.setflags(write=False)
        object.__setattr__(self, 'values', values)

    def at(self, s_m: float) -> float:
        if not math.isfinite(s_m) or not self.values[0, 0] <= s_m <= self.values[-1, 0]:
            raise ValueError('STEERING_GUIDE_SUPPORT')
        return float(np.interp(s_m, self.values[:, 0], self.values[:, 1]))


def guide_control(*, pp_rad: float, guide_rad: float, pulse_rad: float,
                  amplitude_rad: float, phase: str) -> dict[str, Any]:
    """Blend only while pulse phase is hold; zero pulse means pure live PP.

    A full-amplitude pulse selects 100% nominal-course steering. Both cosine
    ramps also blend the base command, so a corner uses current course progress
    throughout. The return ramp is still hold and is excluded from teachers.
    """
    if (not all(math.isfinite(v) for v in (pp_rad, guide_rad, pulse_rad, amplitude_rad))
            or abs(pp_rad) > .640001 or abs(guide_rad) > (.5 if phase == 'hold' else .640001)
            or not 0. < abs(amplitude_rad) <= .1
            or abs(pulse_rad) > abs(amplitude_rad)+1e-12
            or phase not in ('baseline', 'hold', 'recovery') or phase != 'hold' and pulse_rad != 0.):
        raise ValueError('GUIDE_CONTROL_INPUT')
    weight = min(1., abs(pulse_rad/amplitude_rad)) if phase == 'hold' else 0.
    base = (1.-weight)*pp_rad+weight*guide_rad
    return dict(policy=POLICY, phase=phase, guide_weight=weight, pp_angle_rad=pp_rad,
                guide_angle_rad=guide_rad, base_angle_rad=base,
                pulse_angle_rad=pulse_rad, requested_angle_rad=base+pulse_rad)


def validate_guide_control_row(row: Mapping[str, Any]) -> None:
    """Prevent guide-assisted or return-ramp samples being labeled PP recovery."""
    if not row['pulse']['applied']:
        return
    data = row.get('guide_control')
    if not isinstance(data, dict) or data.get('policy') != POLICY:
        raise ValueError('GUIDE_CONTROL_METADATA_MISSING')
    try:
        expected = guide_control(pp_rad=data['pp_angle_rad'], guide_rad=data['guide_angle_rad'],
            pulse_rad=row['pulse']['requested_rad'], amplitude_rad=.1, phase=row['phase'])
        if (data.keys() != expected.keys() or any(data[k] != v for k, v in expected.items())
                or data['pp_angle_rad'] != row['nominal_angle_rad']):
            raise ValueError('GUIDE_CONTROL_METADATA_MISMATCH')
    except (KeyError, TypeError) as exc:
        raise ValueError('GUIDE_CONTROL_METADATA_MISSING') from exc
