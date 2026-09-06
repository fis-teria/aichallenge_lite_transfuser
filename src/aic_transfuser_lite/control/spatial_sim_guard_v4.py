"""Fail-closed simulator dispatch checks. No model, ROS or filesystem I/O."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .spatial_path_adapter_v4 import transform


def scan_coverage(states_in_scan_base: np.ndarray, scan: dict, cfg: dict) -> dict:
    """Check complete rectangular footprints against the current finite scan.

    A grid covers the rectangle including its interior. Each grid location is
    inflated by a half-cell diagonal plus .05 m; all intersected angular bins
    must be observed, with both finite rays extending beyond the inflated cell.
    Invalid returns and outside-FOV are UNKNOWN, never free. This conservative
    2D discretized observation test is not a proof against all missed obstacles.
    No static route/map and no prior footprint-as-free shortcut.
    """
    states = np.asarray(states_in_scan_base, dtype=float)
    ranges = np.asarray(scan['ranges'], dtype=float)
    amin, inc = float(scan['angle_min']), float(scan['angle_increment'])
    if states.ndim != 2 or states.shape[1] != 5 or not len(states) or not np.isfinite(states).all():
        raise ValueError('INVALID_FOOTPRINT_STATES')
    if ranges.shape != (750,) or not np.isfinite([amin, inc]).all() or inc <= 0:
        raise ValueError('INVALID_SCAN_GEOMETRY')
    step = .10
    x = np.linspace(-cfg['rear_overhang_m'], cfg['wheelbase_m']+cfg['front_overhang_m'],
                    int(np.ceil((cfg['rear_overhang_m']+cfg['wheelbase_m']+cfg['front_overhang_m'])/step))+1)
    y = np.linspace(-cfg['body_width_m']/2, cfg['body_width_m']/2,
                    int(np.ceil(cfg['body_width_m']/step))+1)
    cells = np.stack(np.meshgrid(x, y), axis=-1).reshape(-1, 2)
    radius = step/np.sqrt(2)+.05
    valid = np.isfinite(ranges) & (ranges > scan['range_min']) & (ranges <= scan['range_max'])
    unknown = occupied = 0
    min_margin = float('inf')
    for state in states:
        points = transform(cells, state[:3])-np.array([cfg['lidar_x_in_base_m'], cfg['lidar_y_in_base_m']])
        r = np.linalg.norm(points, axis=1)
        a = np.arctan2(points[:, 1], points[:, 0])
        span = np.arcsin(np.minimum(1., radius/np.maximum(r, 1e-9)))
        lo = np.floor((a-span-amin)/inc).astype(int)
        hi = np.ceil((a+span-amin)/inc).astype(int)
        for distance, left, right in zip(r, lo, hi):
            if distance <= radius or left < 0 or right >= len(ranges) or not np.all(valid[left:right+1]):
                unknown += 1
                continue
            margin = float(np.min(ranges[left:right+1])*np.cos(inc)-distance-radius)
            min_margin = min(min_margin, margin)
            occupied += int(margin <= 0)
    return dict(verified=unknown == 0 and occupied == 0, unknown_cells=unknown,
                obstructed_cells=occupied, checked_cells=len(cells)*len(states),
                minimum_margin_m=min_margin if np.isfinite(min_margin) else None,
                reason='UNKNOWN_FREE_SPACE' if unknown else 'FOOTPRINT_OBSTRUCTION' if occupied else None,
                policy='CURRENT_SCAN_FULL_RECTANGLE_CELLS_V1', inflation_m=radius,
                prior_footprint_assumed_free=False)


@dataclass(frozen=True)
class MotionEvidence:
    isolation: bool = False
    consumer: bool = False
    pose_timing: bool = False
    footprint_profile: bool = False
    clearance: bool = False
    collision_monitor: bool = False
    logger: bool = False
    command_history: bool = False

    def reason(self) -> str | None:
        for field, valid in vars(self).items():
            if valid is not True:
                return 'UNVERIFIED_' + field.upper()
        return None


class OperationLease:
    """Independent supervisor gate; late results cannot renew an expired lease.

    The parent alone publishes controls. A blocked worker cannot publish or
    keep alive its old acceleration. Once a powered lease faults it is latched.
    """
    def __init__(self, maximum_speed_mps: float = .30):
        self.maximum_speed_mps = maximum_speed_mps
        self.fault: str | None = None
        self.powered = False
        self.last_operation: str | None = None

    def reject(self, result: dict, *, now_ns: int, now_sim_ns: int, epoch: str,
               current_input_id: str, current_state_ns: int, healthy: bool) -> str | None:
        if self.fault:
            return self.fault
        request = result.get('request')
        why = None
        if not healthy:
            why = 'SUPERVISOR_UNHEALTHY'
        elif result.get('epoch') != epoch or result.get('input_id') != current_input_id:
            why = 'STALE_INPUT_OR_EPOCH'
        elif now_ns >= result.get('deadline_monotonic_ns', 0):
            why = 'WORKER_DEADLINE'
        elif not 0 <= now_sim_ns-result.get('observed_sim_ns', -1) <= 200_000_000:
            why = 'STALE_OBSERVATION'
        elif not 0 <= now_sim_ns-current_state_ns <= 200_000_000:
            why = 'STALE_STATE'
        elif request is None or not result.get('solver_accepted'):
            why = 'NO_ACCEPTED_MPC_REQUEST'
        elif result.get('motion_rejection'):
            why = result['motion_rejection']
        elif request['operation_id'] == self.last_operation:
            why = 'ALREADY_SENT'
        if why and self.powered:
            self.fault = why
        return why

    def sent(self, operation: dict) -> None:
        self.last_operation = operation['operation_id']
        self.powered |= operation['acceleration_mps2'] > 0

    def watchdog(self, *, now_ns: int, last_accepted_ns: int, state_received_ns: int,
                 speed_mps: float | None, worker_alive: bool, logger_ok: bool) -> str | None:
        reason = ('LOGGER_FAILED' if not logger_ok else 'WORKER_EXITED' if not worker_alive else
                  'VELOCITY_UNKNOWN' if speed_mps is None or not np.isfinite(speed_mps) else
                  'OVERSPEED' if abs(speed_mps) > self.maximum_speed_mps else
                  'STATE_STALE' if now_ns-state_received_ns > 250_000_000 else
                  'WORKER_STALL' if now_ns-last_accepted_ns > 250_000_000 else None)
        if self.powered and reason:
            self.fault = reason
        return self.fault or reason
