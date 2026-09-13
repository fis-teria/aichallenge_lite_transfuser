"""Pure contracts for measured AWSIM recovery collection; SI units and sim ns.

Reference/phase information is teacher/debug-only. It is never a model input
or a substitute for the measured future pose. Existing V3 eligibility is kept.
"""
from __future__ import annotations

from dataclasses import dataclass
import csv
import math
from pathlib import Path
from typing import Sequence

import numpy as np

from .recovery_reference_v3 import MpcReferencePointV3, _recompute_geometry

TARGET_MPS = 5.0 / 3.6
OVERSPEED_MPS = 6.0 / 3.6
PHASES = {"baseline", "approach", "hold", "recovery", "braking", "invalid"}
ELIGIBLE_PHASES = {"baseline", "recovery"}


def load_pose_course(path: Path) -> tuple[MpcReferencePointV3, ...]:
    """Read official map pose CSV [N,8], replace only its speed with 5 km/h."""
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.reader(stream)
        if next(reader) != ["x", "y", "z", "x_quat", "y_quat", "z_quat", "w_quat", "speed"]:
            raise ValueError("POSE_COURSE_COLUMNS")
        rows = np.asarray([[float(v) for v in row] for row in reader], dtype=float)
    if rows.ndim != 2 or rows.shape[1] != 8 or len(rows) < 20 or not np.isfinite(rows).all():
        raise ValueError("POSE_COURSE_SHAPE_FINITE")
    if not np.allclose((rows[:, 3:7] ** 2).sum(axis=1), 1., atol=.01):
        raise ValueError("POSE_COURSE_QUATERNION")
    if np.linalg.norm(rows[-1, :2] - rows[0, :2]) < 1e-6:
        rows = rows[:-1]
    s, _, kappa = _recompute_geometry(rows[:, 0], rows[:, 1])
    yaw = np.arctan2(2*(rows[:, 6]*rows[:, 5] + rows[:, 3]*rows[:, 4]),
                     1-2*(rows[:, 4]**2 + rows[:, 5]**2))
    return tuple(MpcReferencePointV3(float(s[i]), float(r[0]), float(r[1]),
                 float(yaw[i]), float(kappa[i]), TARGET_MPS, 0.) for i, r in enumerate(rows))


def project_course(xy: np.ndarray, position: Sequence[float], yaw_rad: float) -> dict[str, float | int]:
    """Project map pose onto a closed reference [N,2]; left offset is positive."""
    xy = np.asarray(xy, dtype=float)
    point = np.asarray(position, dtype=float)
    if (xy.ndim != 2 or xy.shape[1] != 2 or len(xy) < 3 or point.shape != (2,)
            or not np.isfinite(xy).all() or not np.isfinite(point).all() or not math.isfinite(yaw_rad)):
        raise ValueError("COURSE_PROJECTION_SHAPE_FINITE")
    segments = np.roll(xy, -1, axis=0) - xy
    length = np.linalg.norm(segments, axis=1)
    if np.any(length <= 1e-6):
        raise ValueError("COURSE_ZERO_SEGMENT")
    alpha = np.clip(((point-xy)*segments).sum(axis=1)/(length**2), 0., 1.)
    residual = point - (xy + alpha[:, None]*segments)
    error = np.linalg.norm(residual, axis=1)
    heading = np.arctan2(segments[:, 1], segments[:, 0])
    aligned = np.cos(heading-yaw_rad) > .5
    if not aligned.any():
        raise ValueError("COURSE_HEADING_MISMATCH")
    index = int(np.argmin(np.where(aligned, error, np.inf)))
    if error[index] > 2.:
        raise ValueError("COURSE_TOO_FAR")
    s = np.r_[0., np.cumsum(length[:-1])]
    lateral = (segments[index, 0]*residual[index, 1] - segments[index, 1]*residual[index, 0])/length[index]
    return {"s_m": float(s[index]+alpha[index]*length[index]), "offset_m": float(lateral),
            "distance_m": float(error[index]), "segment_index": index,
            "heading_error_rad": math.atan2(math.sin(yaw_rad-heading[index]), math.cos(yaw_rad-heading[index]))}


def phase_at_s(s_m: float, intervals: Sequence[dict]) -> str:
    """Half-open spatial intervals; do not inherit V3's hold eligibility."""
    if not math.isfinite(s_m) or s_m < 0:
        raise ValueError("PHASE_PROGRESS_INVALID")
    result = "baseline"
    previous_end = -1.
    for item in intervals:
        start, end, phase = item["start_s_m"], item["end_s_m"], item["phase"]
        if phase not in {"approach", "hold", "recovery"} or not 0 <= start < end or start < previous_end:
            raise ValueError("PHASE_INTERVAL_INVALID")
        if start <= s_m < end:
            result = phase
        previous_end = end
    return result


@dataclass(frozen=True)
class PhaseWindow:
    start_ns: int
    end_ns: int
    phase: str

    def __post_init__(self) -> None:
        if (type(self.start_ns) is not int or type(self.end_ns) is not int
                or not 0 <= self.start_ns < self.end_ns or self.phase not in PHASES):
            raise ValueError("PHASE_WINDOW_INVALID")


def recovery_teacher_mask(observation_ns: int, windows: Sequence[PhaseWindow]) -> np.ndarray:
    """Return bool [30]: whole future interval must stay in eligible phases.

    Excluding an anchor does not delete its camera/ego history. Unsupported
    gaps, approach, hold, braking and invalid observations cannot form labels.
    This phase mask must still be ANDed with sensor and observed-future masks.
    """
    if type(observation_ns) is not int or observation_ns < 0:
        raise ValueError("PHASE_OBSERVATION_INVALID")
    for left, right in zip(windows, windows[1:]):
        if left.end_ns > right.start_ns:
            raise ValueError("PHASE_WINDOWS_OVERLAP_OR_ORDER")
    mask = np.zeros(30, dtype=bool)
    for i in range(30):
        endpoint = observation_ns + (i+1)*100_000_000
        cursor = observation_ns
        for window in windows:
            if window.end_ns <= cursor:
                continue
            if window.start_ns > cursor or window.phase not in ELIGIBLE_PHASES:
                break
            if endpoint < window.end_ns:
                mask[i] = True
                break
            cursor = window.end_ns
    return mask


def validate_nominal(*, stamp_ns: int, now_ns: int, received_ns: int, now_wall_ns: int,
                     target_mps: float, acceleration_mps2: float, steering_input_rad: float,
                     measured_speed_mps: float) -> None:
    """Reject stale/nonfinite/out-of-contract teacher commands before actuation."""
    if any(type(t) is not int or t < 0 for t in (stamp_ns, now_ns, received_ns, now_wall_ns)):
        raise ValueError("NOMINAL_CLOCK_INVALID")
    if not -20_000_000 <= now_ns-stamp_ns <= 150_000_000 or not 0 <= now_wall_ns-received_ns <= 300_000_000:
        raise ValueError("NOMINAL_STALE_OR_FUTURE")
    if not np.isfinite([target_mps, acceleration_mps2, steering_input_rad, measured_speed_mps]).all():
        raise ValueError("NOMINAL_NONFINITE")
    if not math.isclose(target_mps, TARGET_MPS, abs_tol=1e-5):
        raise ValueError("NOMINAL_FIXED_SPEED_MISMATCH")
    if not -.03 <= measured_speed_mps <= OVERSPEED_MPS:
        raise ValueError("OVERSPEED_OR_REVERSE")
    if abs(steering_input_rad) > .5:
        raise ValueError("NOMINAL_STEERING_LIMIT")

