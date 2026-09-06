"""Pure sim command gate. No ROS/network/publisher and no implicit arming.

An integration must obtain the evidence externally and run an INDEPENDENT
watchdog; this helper alone is neither isolation proof nor a live controller.
"""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class SnapshotKey:
    epoch: str
    input_id: str
    path_id: str
    state_id: str
    observed_sim_ns: int
    deadline_monotonic_ns: int


def send_rejection(key: SnapshotKey, current: SnapshotKey, *, now_monotonic_ns: int,
                   now_sim_ns: int, max_age_ns: int, isolation_verified: bool,
                   interface_verified: bool, frame_verified: bool, clearance_verified: bool,
                   watchdog_healthy: bool, logger_healthy: bool, communication_healthy: bool,
                   solver_accepted: bool, enabled: bool = False) -> str | None:
    """Call AFTER solver completion and again immediately BEFORE dispatch."""
    checks = [('DISABLED', enabled), ('BLOCKED_SIM_ISOLATION', isolation_verified),
              ('INTERFACE_UNVERIFIED', interface_verified), ('FRAME_UNVERIFIED', frame_verified),
              ('FREE_SPACE_UNVERIFIED', clearance_verified), ('WATCHDOG_UNHEALTHY', watchdog_healthy),
              ('LOGGER_UNHEALTHY', logger_healthy), ('COMMUNICATION_UNHEALTHY', communication_healthy),
              ('SOLVER_REJECTED', solver_accepted)]
    for reason, good in checks:
        if good is not True:
            return reason
    if key != current:
        return 'STALE_SNAPSHOT_OR_RESET'
    if now_monotonic_ns >= key.deadline_monotonic_ns:
        return 'EXPIRED_SOLVER_RESULT'
    if not 0 <= now_sim_ns-key.observed_sim_ns <= max_age_ns:
        return 'STALE_STATE_OR_CLOCK_RESET'
    return None


class AckermannDispatch:
    """One rate integration per operation ID; only acknowledge after real send.

    Actual inspected consumer accepts acceleration SI and tire angle rad, not
    pedal fractions. Its longitudinal.speed field is an explicit desired-speed
    reference (ignored by AWSIM), not measured speed or applied acceleration.
    No requested value is a sent-history sample until acknowledge_sent returns.
    """
    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.sent: dict | None = None

    def request(self, operation_id: str, u: np.ndarray, *, measured_delta_rad: float,
                control_dt_s: float, target_speed_mps: float) -> dict:
        values = np.r_[u, measured_delta_rad, control_dt_s, target_speed_mps]
        if np.asarray(u).shape != (2,) or not np.isfinite(values).all():
            raise ValueError('NONFINITE_OR_SHAPE_COMMAND')
        if not 0 < control_dt_s <= self.cfg['controller_dt_s']*1.5:
            raise ValueError('CONTROL_DT_INVALID')
        if self.sent and operation_id == self.sent['operation_id']:
            raise ValueError('ALREADY_SENT_NO_REINTEGRATION')
        a, rate = map(float,u)
        delta = measured_delta_rad+rate*control_dt_s
        if not (-self.cfg['braking_max_mps2'] <= a <= self.cfg['acceleration_max_mps2']
                and abs(rate) <= self.cfg['steering_rate_limit_rad_s']
                and abs(delta) <= self.cfg['steering_limit_rad']
                and 0 <= target_speed_mps <= self.cfg['maximum_speed_mps']):
            raise ValueError('COMMAND_LIMIT')
        return dict(operation_id=operation_id, acceleration_mps2=a,
                    steering_tire_angle_rad=delta, steering_tire_rotation_rate_rad_s=rate,
                    desired_speed_reference_mps=target_speed_mps,
                    control_dt_s=control_dt_s, status='REQUESTED_NOT_SENT', applied=None)

    def acknowledge_sent(self, request: dict, *, sent_sim_ns: int,
                         sent_monotonic_ns: int) -> dict:
        if request['status'] != 'REQUESTED_NOT_SENT':
            raise ValueError('NOT_A_NEW_REQUEST')
        if self.sent and request['operation_id']==self.sent['operation_id']:
            raise ValueError('ALREADY_ACKNOWLEDGED')
        self.sent = dict(request, status='SENT_NOT_APPLIED_CONFIRMED',
                         sent_sim_ns=sent_sim_ns,sent_monotonic_ns=sent_monotonic_ns,
                         policy='SIM_ONLY_POLICY_CHANGED')
        return dict(self.sent)


def sent_history_sample(sent: dict, *, current_observation_ns: int,
                        available_cutoff_ns: int) -> tuple[float,float,float]:
    """Reject proposals / future / unavailable receipts. No measured-v fallback."""
    if (sent.get('status')!='SENT_NOT_APPLIED_CONFIRMED'
        or sent.get('policy')!='SIM_ONLY_POLICY_CHANGED'
        or sent['sent_sim_ns'] >= current_observation_ns
        or sent['sent_monotonic_ns'] > available_cutoff_ns):
        raise ValueError('COMMAND_NOT_CAUSALLY_SENT')
    values = (sent['steering_tire_angle_rad'],sent['desired_speed_reference_mps'],sent['acceleration_mps2'])
    if not np.isfinite(values).all():
        raise ValueError('MISSING_SENT_FIELD')
    return values
