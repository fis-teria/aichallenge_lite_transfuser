"""Pure contracts for measured AWSIM recovery collection; SI units and sim ns.

Reference/phase information is teacher/debug-only. It is never a model input
or a substitute for the measured future pose. Existing V3 eligibility is kept.
"""
from __future__ import annotations

from dataclasses import dataclass
import csv
import math
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np

from .recovery_reference_v3 import MpcReferencePointV3, _recompute_geometry

TARGET_MPS = 5.0 / 3.6
OVERSPEED_MPS = 6.0 / 3.6
PHASES = {"baseline", "approach", "hold", "recovery", "braking", "invalid"}
ELIGIBLE_PHASES = {"baseline", "recovery"}
COLLECTION_SPEED_POLICIES = ("legacy_gain1_v1", "aligned_gain4_v1")


def collection_speed_gain(policy: str) -> float:
    """PP longitudinal gain [1/s]; both policies retain target 5/3.6 m/s."""
    if policy not in COLLECTION_SPEED_POLICIES:
        raise ValueError('COLLECTION_SPEED_POLICY')
    return 4.0 if policy == 'aligned_gain4_v1' else 1.0


def validate_collection_speed_parameters(policy: str, parameters: Mapping[str, object]) -> None:
    """Check the actually loaded PP parameters before granting drive authority."""
    gain = collection_speed_gain(policy)
    if parameters.get('use_external_target_vel') is not True:
        raise ValueError('COLLECTION_EXTERNAL_SPEED_DISABLED')
    for key, expected in (('external_target_vel', TARGET_MPS), ('speed_proportional_gain', gain)):
        value = parameters.get(key)
        if (type(value) not in (float, int) or not math.isfinite(value)
                or not math.isclose(value, expected, rel_tol=0., abs_tol=1e-6)):
            raise ValueError('COLLECTION_LOADED_SPEED_MISMATCH:' + key)


def collection_snapshot_retry_allowed(reason: str, *, attempt: int, elapsed_ns: int) -> bool:
    """One fresh-snapshot retry, only for timing and before the 100 ms deadline.

    Reserve 20 ms for the second computation. Geometric, actuator, source and
    clock-reset faults never retry. A retry sends no unvalidated go command.
    """
    return (attempt == 0 and 0 <= elapsed_ns <= 80_000_000
            and (reason.startswith('STALE_') or reason in {
                'CLOCK_STALE', 'NOMINAL_STALE_OR_FUTURE', 'FRESH_ALIGNED_SCAN_MISSING',
                'STATE_FRAME_OR_CAPTURE_SKEW'}))


def check_collection_decision_age(*, started_ns: int, now_ns: int) -> None:
    """Bound sensor selection plus computation before any go publication."""
    if (type(started_ns) is not int or type(now_ns) is not int or started_ns < 0
            or not 0 <= now_ns-started_ns <= 100_000_000):
        raise ValueError('COLLECTION_COMPUTATION_TIMEOUT')


def collection_snapshot_retry_wait_ns(elapsed_ns: int) -> int:
    """Yield up to 20 ms for real ROS arrivals, retaining 20 ms to recompute.

    Called only after the single timing retry is admitted. This never creates
    samples or extends the total 100 ms decision deadline.
    """
    if type(elapsed_ns) is not int or not 0 <= elapsed_ns <= 80_000_000:
        raise ValueError('SNAPSHOT_RETRY_WAIT_CONTRACT')
    return min(20_000_000, 80_000_000-elapsed_ns)


def check_collection_input_time(role: str, *, capture_ns: int, receipt_ns: int,
                                now_sim_ns: int, now_wall_ns: int) -> None:
    """Use existing TimePath input budgets, retaining original capture clocks.

    Camera inference admission uses 500 ms (time_path_node.py); the 150 ms
    control-state / scan contract is a different, higher-rate input budget.
    Trajectory publication is 1 Hz and the official PP uses a 1.5 s limit.
    """
    limits = {'camera': (500_000_000, 500_000_000), 'trajectory': (1_500_000_000, 2_000_000_000),
              **{r: (150_000_000, 300_000_000) for r in ('pose', 'velocity', 'steering', 'imu', 'scan', 'nominal')}}
    if role not in limits or any(type(t) is not int or t < 0 for t in (capture_ns, receipt_ns, now_sim_ns, now_wall_ns)):
        raise ValueError('INPUT_TIME_CONTRACT')
    capture_limit, receipt_limit = limits[role]
    if not -20_000_000 <= now_sim_ns-capture_ns <= capture_limit or not 0 <= now_wall_ns-receipt_ns <= receipt_limit:
        raise ValueError('STALE_'+role)


def select_collection_input(role: str, capture_receipts_ns: Sequence[tuple[int, int]], *,
                            now_sim_ns: int, now_wall_ns: int) -> int | None:
    """Newest admissible original sample; a newer in-flight clock never retimes it.

    DDS topics arrive independently. Keep a bounded history so a future-stamped
    arrival does not evict the still-fresh usable sample before /clock catches
    up. If no sample satisfies the unchanged per-role limits, fail closed.
    """
    usable = []
    for i, (capture, receipt) in enumerate(capture_receipts_ns):
        try:
            check_collection_input_time(role, capture_ns=capture, receipt_ns=receipt,
                                        now_sim_ns=now_sim_ns, now_wall_ns=now_wall_ns)
        except ValueError as exc:
            if str(exc) != 'STALE_'+role:
                raise
            continue
        usable.append((capture, receipt, i))
    return max(usable)[2] if usable else None


def select_collection_motion(histories: Mapping[str, Sequence[tuple[int, int]]], *,
                             now_sim_ns: int, now_wall_ns: int, include_imu: bool = False) -> dict[str, int] | None:
    """Newest coherent measured motion triple, with unchanged 50 ms skew limits.

    Prefer the newest admissible velocity, then the newest pose and steering
    within its skew window. Every member must independently satisfy its capture
    and receipt deadline. Return indices into the original bounded histories;
    never interpolate, extrapolate, or alter a measurement's stamp.
    """
    peers = ('pose', 'steering', 'imu') if include_imu else ('pose', 'steering')
    usable: dict[str, list[tuple[int, int, int]]] = {}
    for role in ('velocity', *peers):
        usable[role] = []
        for i, (capture, receipt) in enumerate(histories.get(role, ())):
            if select_collection_input(role, [(capture, receipt)],
                    now_sim_ns=now_sim_ns, now_wall_ns=now_wall_ns) is not None:
                usable[role].append((capture, receipt, i))
    for velocity_stamp, _, velocity_index in sorted(usable['velocity'], reverse=True):
        selected = {'velocity': velocity_index}
        for role in peers:
            aligned = [item for item in usable[role] if abs(item[0]-velocity_stamp) <= 50_000_000]
            if not aligned:
                break
            selected[role] = max(aligned)[2]
        if len(selected) == len(peers)+1:
            return selected
    return None


def validate_collection_imu_axes(transforms: Mapping[str, tuple[str, Sequence[float]]]) -> None:
    """Verify imu_link -> sensor_kit_base_link -> base_link preserves +Z.

    Angular Z is invariant under these measured static yaw-only rotations;
    sensor translations do not alter angular velocity. Never assume a pitched
    or rolled IMU's raw Z is body yaw rate.
    """
    for child, parent in (('imu_link', 'sensor_kit_base_link'), ('sensor_kit_base_link', 'base_link')):
        if child not in transforms:
            raise ValueError('IMU_AXES_MISSING')
        actual_parent, rotation = transforms[child]
        q = np.asarray(rotation, dtype=float)
        if (actual_parent != parent or q.shape != (4,) or not np.isfinite(q).all()
                or abs(float(q@q)-1.) > 1e-6 or q[0]**2+q[1]**2 > 1e-8):
            raise ValueError('IMU_AXES_NOT_VERTICAL')


def collection_imu_yaw_rate(angular_radps: Sequence[float], frame: str) -> float:
    """Measured IMU rad/s; callers must validate axes, time and source first."""
    value = np.asarray(angular_radps, dtype=float)
    if frame != 'imu_link' or value.shape != (3,) or not np.isfinite(value).all():
        raise ValueError('IMU_RATE_CONTRACT')
    return float(value[2])


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


def reference_rows_with_wrap(points: Sequence[MpcReferencePointV3], tail_m: float = 12.) -> list[list[float]]:
    """Open controller CSV with a continuous closing segment and >=12 m prefix.

    The official PP searches only forward to the end of the supplied array.
    Internal projection uses a periodic course without a duplicate endpoint;
    exporting that internal representation directly loses its closing segment.
    The copied prefix also preserves the PP's up-to-10 m curvature window at
    the lap boundary. It changes no physical reference coordinates or speeds.
    """
    if len(points) < 20 or not math.isfinite(tail_m) or tail_m <= 0:
        raise ValueError('REFERENCE_WRAP_CONTRACT')
    xy = np.asarray([[p.x_m, p.y_m] for p in points])
    edges = np.linalg.norm(np.roll(xy, -1, axis=0)-xy, axis=1)
    if not np.isfinite(edges).all() or (edges <= 1e-6).any() or edges.sum() <= tail_m:
        raise ValueError('REFERENCE_WRAP_GEOMETRY')
    count = int(np.searchsorted(np.r_[0., np.cumsum(edges[:-1])], tail_m))+1
    sequence = [*points, *points[:count]]
    rows = []; progress = 0.
    for i, p in enumerate(sequence):
        if i:
            previous = sequence[i-1]
            progress += math.hypot(p.x_m-previous.x_m, p.y_m-previous.y_m)
        rows.append([progress, p.x_m, p.y_m, p.psi_rad, p.kappa_radpm, p.vx_mps, p.ax_mps2])
    return rows


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


def collection_phase_windows(rows: Sequence[Mapping[str, object]]) -> tuple[PhaseWindow, ...]:
    """Legacy decision stamps or versioned pulse publication stamps, never mixed.

Pulse recovery requires an emitted zero-perturbation teacher command. Missing
telemetry, conflicting same-stamp phases and later perturbations block labels.
"""
    schemas = {row.get('annotation_schema') for row in rows}
    versioned = 'measured_steering_pulse_v1' in schemas
    if schemas - {None, 'measured_steering_pulse_v1'} or (versioned and None in schemas):
        raise ValueError('PHASE_ANNOTATION_SCHEMA')
    phases: dict[int, str] = {}
    sequence = 0; previous_stamp = -1; previous_wall = -1
    for row in rows:
        if not versioned:
            if row['sim_ns'] is not None:
                phases[row['sim_ns']] = row['phase']
            continue
        emitted = row.get('publication')
        if emitted is None:
            if row['phase'] not in ('invalid', 'braking'):
                raise ValueError('PHASE_UNPUBLISHED_TEACHER')
            continue
        t, wall, seq = emitted['sim_ns'], emitted['monotonic_ns'], emitted['sequence']
        if (any(type(v) is not int or v < 0 for v in (t, wall, seq)) or seq != sequence+1
                or t < previous_stamp or wall <= previous_wall):
            raise ValueError('PHASE_PUBLICATION_ORDER')
        sequence = seq; previous_stamp = t; previous_wall = wall
        phase = row['phase']
        if phase not in PHASES:
            raise ValueError('PHASE_PUBLICATION_VALUE')
        if phase in ELIGIBLE_PHASES:
            pulse = row['pulse']
            allowed_stage = {'recovery'} if phase == 'recovery' else {'waiting', 'skipped', 'complete'}
            target = row['target_speed_mps']
            if (pulse['applied'] is not True or pulse['state']['stage'] not in allowed_stage
                    or pulse['requested_rad'] != 0. or pulse['effective_rad'] != 0.
                    or type(target) not in (float, int) or not math.isfinite(target)
                    or not math.isclose(target, TARGET_MPS, abs_tol=1e-5)):
                raise ValueError('PHASE_PERTURBATION_IN_TEACHER')
        if t in phases and phases[t] != phase:
            phases[t] = 'invalid'
        else:
            phases[t] = phase
    windows: list[PhaseWindow] = []
    ordered = sorted(phases)
    for a, b in zip(ordered, ordered[1:]):
        phase = phases[a] if b-a <= 150_000_000 else 'invalid'
        if windows and windows[-1].phase == phase and windows[-1].end_ns == a:
            windows[-1] = PhaseWindow(windows[-1].start_ns, b, phase)
        else:
            windows.append(PhaseWindow(a, b, phase))
    return tuple(windows)


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
    # The pinned official PP emits bounded nominal input up to 0.64 rad.
    # The final publisher separately retains the existing 0.5 rad / 0.8 rad/s
    # limits BEFORE the unchanged stopping-sweep monitor is evaluated.
    if abs(steering_input_rad) > .640001:
        raise ValueError("NOMINAL_STEERING_LIMIT")


def bounded_collection_command(angle_rad: float, acceleration_mps2: float,
                               previous_angle_rad: float, dt_wall_s: float) -> tuple[float, float]:
    """Final AWSIM input (rad, m/s^2); fixed existing amplitude/rate limits."""
    if (not np.isfinite([angle_rad, acceleration_mps2, previous_angle_rad, dt_wall_s]).all()
            or abs(previous_angle_rad) > .500001 or not 0 <= dt_wall_s <= .1):
        raise ValueError('FINAL_COMMAND_CONTRACT')
    bounded = float(np.clip(angle_rad, -.5, .5))
    issued = float(np.clip(bounded, previous_angle_rad-.8*dt_wall_s, previous_angle_rad+.8*dt_wall_s))
    return issued, float(np.clip(acceleration_mps2, -1., 1.))
