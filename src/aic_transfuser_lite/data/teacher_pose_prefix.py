"""Offline, run-scoped teacher truncation before localization loses consistency.

This is an additional exclusion mask. It never certifies clearance, assigns a
split, repairs poses, or makes the diagnostic IMU an inference input.
"""
from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass
import math
from typing import Any, Sequence

import numpy as np


@dataclass(frozen=True)
class HeadingPrefixConfig:
    max_error_rad: float = math.radians(5.)
    baseline_spread_rad: float = math.radians(3.)
    warmup_ns: int = 1_000_000_000
    baseline_ns: int = 2_000_000_000
    smoothing_ns: int = 500_000_000
    persistence_ns: int = 1_000_000_000
    backoff_ns: int = 500_000_000
    max_gap_ns: int = 150_000_000

    def __post_init__(self) -> None:
        for value in (self.max_error_rad, self.baseline_spread_rad):
            if not math.isfinite(value) or not 0 < value < math.pi/2:
                raise ValueError('heading thresholds must be finite radians in (0, pi/2)')
        for name in ('warmup_ns', 'baseline_ns', 'smoothing_ns', 'persistence_ns', 'backoff_ns', 'max_gap_ns'):
            value = getattr(self, name)
            if type(value) is not int or value < 0 or (name != 'warmup_ns' and value == 0):
                raise ValueError('durations must be positive integer ns; warmup may be zero')


def wrap_rad(value: float | np.ndarray) -> Any:
    return np.arctan2(np.sin(value), np.cos(value))


def heading_prefix(samples: Sequence[tuple[int, float]], *, drive_start_ns: int,
                   observed_end_ns: int, config: HeadingPrefixConfig = HeadingPrefixConfig()) -> dict[str, Any]:
    """First bad prefix from (capture ns, EKF yaw minus IMU yaw [rad]).

    A fixed initial frame offset is removed; absolute initial pose validity is
    NOT established here. Nonfinite/missing observations fail closed. A sustained
    violation is backdated through the trailing filter and margin, then latched.
    """
    if any(type(v) is not int or v < 0 for v in (drive_start_ns, observed_end_ns)) or observed_end_ns <= drive_start_ns:
        raise ValueError('invalid simulation epoch bounds [ns]')
    if any(type(t) is not int or t < 0 for t, _ in samples):
        raise ValueError('capture stamps must be nonnegative integer ns')
    if any(a[0] >= b[0] for a, b in zip(samples, samples[1:])):
        raise ValueError('heading captures must be strictly increasing in one epoch')
    begin = drive_start_ns + config.warmup_ns
    baseline_end = begin + config.baseline_ns
    values = [(t, d) for t, d in samples if begin <= t <= observed_end_ns]
    baseline = [(t, d) for t, d in values if t <= baseline_end]
    result: dict[str, Any] = dict(policy='teacher_pose_prefix_v1', config=asdict(config),
        valid_from_ns=begin, valid_until_ns=begin, invalid_from_ns=begin,
        reason='BASELINE_UNVERIFIED', baseline_offset_rad=None, baseline_spread_rad=None,
        first_violation_ns=None, confirmed_at_ns=None, trace=[])
    if (len(baseline) < 5 or baseline[0][0]-begin > config.max_gap_ns
            or baseline_end-baseline[-1][0] > config.max_gap_ns
            or any(b[0]-a[0] > config.max_gap_ns for a, b in zip(baseline, baseline[1:]))
            or any(not math.isfinite(d) for _, d in baseline)):
        return result
    delta = np.asarray([d for _, d in baseline])
    offset = float(np.arctan2(np.sin(delta).mean(), np.cos(delta).mean()))
    spread = float(np.percentile(np.abs(wrap_rad(delta-offset)), 95))
    result.update(baseline_offset_rad=offset, baseline_spread_rad=spread)
    if spread > config.baseline_spread_rad:
        result['reason'] = 'BASELINE_UNSTABLE'
        return result
    result.update(valid_until_ns=observed_end_ns, invalid_from_ns=None, reason='NO_DRIFT_DETECTED')
    window: deque[tuple[int, float]] = deque()
    violation: int | None = None
    previous = begin
    for stamp, difference in values:
        if stamp-previous > config.max_gap_ns or not math.isfinite(difference):
            cutoff = previous if stamp-previous > config.max_gap_ns else stamp
            cutoff = max(begin, cutoff-config.backoff_ns)
            result.update(valid_until_ns=cutoff, invalid_from_ns=cutoff, reason='HEADING_EVIDENCE_GAP',
                          first_violation_ns=stamp, confirmed_at_ns=stamp)
            break
        previous = stamp
        error = float(wrap_rad(difference-offset))
        window.append((stamp, error))
        while window and stamp-window[0][0] > config.smoothing_ns:
            window.popleft()
        smoothed = float(np.median([d for _, d in window]))
        result['trace'].append(dict(stamp_ns=stamp, error_rad=error, median_error_rad=smoothed))
        if stamp <= baseline_end:
            continue
        if abs(smoothed) >= config.max_error_rad:
            if violation is None:
                violation = stamp
            if stamp-violation >= config.persistence_ns:
                cutoff = max(begin, violation-config.smoothing_ns-config.backoff_ns)
                result.update(valid_until_ns=cutoff, invalid_from_ns=cutoff, reason='HEADING_DRIFT',
                              first_violation_ns=violation, confirmed_at_ns=stamp)
                break
        else:
            violation = None
    else:
        if violation is not None:
            cutoff = max(begin, violation-config.smoothing_ns-config.backoff_ns)
            result.update(valid_until_ns=cutoff, invalid_from_ns=cutoff, reason='UNRESOLVED_TAIL_DRIFT',
                          first_violation_ns=violation)
        elif observed_end_ns-previous > config.max_gap_ns:
            cutoff = max(begin, previous-config.backoff_ns)
            result.update(valid_until_ns=cutoff, invalid_from_ns=cutoff, reason='HEADING_EVIDENCE_GAP',
                          first_violation_ns=previous)
    return result


def prefix_anchor_mask(observation_ns: np.ndarray, *, valid_from_ns: int, valid_until_ns: int,
                       horizon_ns: int = 3_000_000_000, endpoint_tolerance_ns: int = 50_000_000) -> np.ndarray:
    """Bool [N]; the complete future plus its latest interpolation support is pre-cut.

    Both observation and source support must be strictly before valid_until_ns.
    The 50 ms guard conservatively covers unknown saved interpolation endpoints.
    """
    times = np.asarray(observation_ns)
    if times.ndim != 1 or times.dtype != np.int64 or np.any(times < 0):
        raise ValueError('observation_ns must be int64 [N], nonnegative')
    if any(type(v) is not int or v < 0 for v in (valid_from_ns, valid_until_ns, horizon_ns, endpoint_tolerance_ns)):
        raise ValueError('prefix bounds and horizon must be nonnegative integer ns')
    if valid_until_ns < valid_from_ns or horizon_ns == 0:
        raise ValueError('invalid prefix interval or horizon')
    # Subtract Python integers rather than overflowing int64 timestamps.
    latest = valid_until_ns-horizon_ns-endpoint_tolerance_ns
    return (times >= valid_from_ns) & (times < latest)


def mask_teacher_arrays(arrays: dict[str, np.ndarray], keep: np.ndarray) -> dict[str, np.ndarray]:
    """Copy [N,30,2] m / [N,30] m/s labels; excluded labels become NaN, masks false."""
    n = len(keep)
    if keep.dtype != np.bool_ or keep.shape != (n,):
        raise ValueError('keep must be bool [N]')
    expected = {'observation_ns': (n,), 'xy_m': (n,30,2), 'xy_mask': (n,30),
                'velocity_mps': (n,30), 'velocity_mask': (n,30), 'forward_avoidance_eligible': (n,)}
    if set(arrays) != set(expected):
        raise ValueError('unexpected observed teacher array contract')
    for key, shape in expected.items():
        if arrays[key].shape != shape:
            raise ValueError(f'{key} must have shape {shape}')
    for key in ('xy_mask', 'velocity_mask', 'forward_avoidance_eligible'):
        if arrays[key].dtype != np.bool_:
            raise ValueError('teacher masks must be bool')
    for key, mask in (('xy_m','xy_mask'), ('velocity_mps','velocity_mask')):
        if not np.issubdtype(arrays[key].dtype, np.floating) or not np.isfinite(arrays[key][arrays[mask]]).all():
            raise ValueError('valid teacher values must be finite floating point')
    copied = {key: value.copy() for key, value in arrays.items()}
    for key in ('xy_mask', 'velocity_mask', 'forward_avoidance_eligible'):
        copied[key][~keep] = False
    for key in ('xy_m', 'velocity_mps'):
        copied[key][~keep] = np.nan
    return copied
