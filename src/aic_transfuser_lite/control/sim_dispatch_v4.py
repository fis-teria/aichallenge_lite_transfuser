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


class SimControlSchedule:
    """One normal publication per sim tick; dt describes the NEXT hold interval.

    A pending operation can survive a newer camera, but cannot supersede an
    already sent newer plan, cross an epoch, or obtain a renewed deadline.
    Emergency braking is a separate interrupt; it still updates previous send.
    """
    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.period_ns = int(cfg.get('command_schedule_s',cfg['controller_dt_s'])*1e9)
        self.next_tick_ns: int | None = None
        self.last_sim_ns: int | None = None
        self.last_a = 0.
        self.adopted_observation_ns = -1
        self.sent_ids: set[str] = set()
        self.epoch: str | None = None

    def reset(self, epoch: str) -> None:
        self.next_tick_ns = self.last_sim_ns = None
        self.last_a = 0.
        self.adopted_observation_ns = -1
        self.sent_ids.clear()
        self.epoch = epoch

    def due(self, sim_ns: int) -> bool:
        return sim_ns >= 0 and (self.last_sim_ns is None or sim_ns > self.last_sim_ns) and (
            self.next_tick_ns is None or sim_ns >= self.next_tick_ns)

    def timing(self, sim_ns: int) -> dict:
        if not self.due(sim_ns): raise ValueError('WAIT_POSITIVE_CONTROL_TICK')
        next_tick = (sim_ns//self.period_ns+1)*self.period_ns
        return dict(model_dt_s=self.cfg['controller_dt_s'],
                    control_interval_s=(next_tick-sim_ns)*1e-9,
                    actual_send_interval_s=None if self.last_sim_ns is None else (sim_ns-self.last_sim_ns)*1e-9,
                    next_tick_ns=next_tick, delta_rate_integration='NEXT_SCHEDULE_INTERVAL_ONCE')

    def rejection(self, result: dict, *, epoch: str, now_ns: int, sim_ns: int,
                  state: np.ndarray, state_ns: int) -> str | None:
        if result['epoch'] != epoch: return 'EPOCH_MISMATCH'
        if result['operation_id'] in self.sent_ids: return 'ALREADY_SENT'
        if result['observed_sim_ns'] <= self.adopted_observation_ns: return 'OLDER_THAN_ADOPTED'
        if now_ns >= result['deadline_monotonic_ns']: return 'EXPIRED'
        if not 0 <= sim_ns-result['observed_sim_ns'] <= 200_000_000: return 'STALE_OBSERVATION'
        if not 0 <= sim_ns-state_ns <= 200_000_000: return 'STALE_CURRENT_STATE'
        dt = (state_ns-result['state_source_ns'])*1e-9
        if not 0 <= dt <= .2: return 'STATE_TIME_MISMATCH'
        prior = np.asarray(result['current_state'],dtype=float)
        latest = np.asarray(state,dtype=float)
        if prior.shape != (5,) or latest.shape != (5,) or not np.isfinite(np.r_[prior,latest]).all():
            return 'INVALID_STATE'
        # Bounded physical consistency, not predicted states passed as measured.
        if np.linalg.norm(prior[:2]-latest[:2]) > self.cfg['maximum_speed_mps']*dt+.005:
            return 'STATE_POSITION_MISMATCH'
        if abs(prior[3]-latest[3]) > self.cfg['braking_max_mps2']*dt+.001:
            return 'STATE_SPEED_MISMATCH'
        if abs(prior[4]-latest[4]) > self.cfg['steering_rate_limit_rad_s']*dt+.001:
            return 'STATE_STEERING_MISMATCH'
        yaw_diff = (prior[2]-latest[2]+np.pi)%(2*np.pi)-np.pi
        if abs(yaw_diff) > self.cfg['maximum_speed_mps']/self.cfg['wheelbase_m']*np.tan(self.cfg['steering_limit_rad'])*dt+.005:
            return 'STATE_YAW_MISMATCH'
        if not result.get('solver_accepted'): return 'SOLVER_REJECTED'
        if result.get('motion_rejection'): return result['motion_rejection']
        if not self.due(sim_ns): return 'WAIT_POSITIVE_CONTROL_TICK'
        actual = self.timing(sim_ns)['actual_send_interval_s']
        if actual is None: return 'NO_PREVIOUS_REAL_COMMAND'
        if abs(result['first_control'][0]-self.last_a) > self.cfg['jerk_limit_mps3']*actual+1e-6:
            return 'SEND_JERK_LIMIT'
        return None

    def sent(self, sim_ns: int, acceleration: float, *, operation_id: str,
             observation_ns: int | None = None) -> None:
        # Call strictly AFTER publisher success; failed proposals consume no ID.
        self.last_sim_ns, self.last_a = sim_ns, acceleration
        self.next_tick_ns = (sim_ns//self.period_ns+1)*self.period_ns
        if observation_ns is not None:
            self.sent_ids.add(operation_id)
            self.adopted_observation_ns = observation_ns


class StopObservation:
    """Every velocity sample, source time based, with missing/clock-gap checks."""
    def __init__(self):
        self.start_ns: int | None = None
        self.last_ns: int | None = None
        self.confirmed = False

    def add(self, ns: int, speed: float, *, fresh: bool) -> bool:
        if self.last_ns is not None and ns == self.last_ns: return self.confirmed
        gap = self.last_ns is not None and not 0 < ns-self.last_ns <= 100_000_000
        if gap or not fresh or not np.isfinite(speed) or abs(speed) > .03:
            self.start_ns = None
            self.confirmed = False
        self.last_ns = ns
        if fresh and not gap and np.isfinite(speed) and abs(speed) <= .03:
            if self.start_ns is None: self.start_ns = ns
            self.confirmed = ns-self.start_ns >= 1_000_000_000
        return self.confirmed
