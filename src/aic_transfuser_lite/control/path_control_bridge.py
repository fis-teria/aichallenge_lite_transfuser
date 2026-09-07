"""Path-to-control bridge with in-memory shadow output ONLY; no ROS or I/O.

XY metres, yaw/tire steer radians, speed m/s, acceleration m/s².
Plan/state/TF times share explicit clock+epoch; generation time is monotonic
metadata and is never subtracted from observation-clock timestamps.
"""
from __future__ import annotations
from dataclasses import dataclass
from copy import deepcopy
import math
import time
import numpy as np
from .waypoint_controller import ControllerConfig, control_from_waypoints


@dataclass(frozen=True)
class Limits:
    provenance: str
    fixture_only: bool
    wheelbase_m: float
    rear_x_in_base_m: float
    steer_rad: float
    steer_rate_rps: float
    lateral_accel_mps2: float
    accel_mps2: float
    brake_mps2: float
    speed_cap_mps: float
    lookahead_m: float
    resample_m: float
    max_segment_m: float
    max_turn_rad: float
    max_initial_offset_m: float
    max_heading_error_rad: float
    max_plan_jump_m: float
    path_ttl_s: float
    state_ttl_s: float
    future_tolerance_s: float
    period_s: float
    stop_delay_s: float


@dataclass(frozen=True)
class Plan:
    id: str
    source: str
    source_s: float
    generated_mono_s: float
    expires_s: float
    clock: str
    epoch: str
    frame: str
    reference_point: str
    xy_m: tuple[tuple[float, float], ...]
    kind: str = 'GEOMETRIC'
    speed_plan_id: str | None = None
    speed_mps: float | None = None
    speed_source: str | None = None


@dataclass(frozen=True)
class Vehicle:
    id: str
    stamp_s: float
    clock: str
    epoch: str
    # base origin in a fixed local odometry frame
    pose: tuple[float, float, float]
    speed_mps: float
    tire_steer_rad: float


@dataclass(frozen=True)
class PathPose:
    plan_id: str
    stamp_s: float
    clock: str
    epoch: str
    base_in_local: tuple[float, float, float]
    evidence: str


def from_v4_record(record: dict, *, expires_s: float) -> Plan:
    """Accept recorded raw output, never infer a ROS Path topic contract.

    V4 nominal_s is neither actual distance nor time. No model speed is invented.
    Raw list is copied to immutable tuples; no output/snapshot mutation.
    """
    o = record['output']; stamp = o['t_obs']
    generated = record['timing']['snapshot_ready']
    if (o['status'] != 'SHAPE_FINITE_ONLY' or stamp['status'] != 'KNOWN' or
            generated['status'] != 'KNOWN' or generated['domain'] != 'MONOTONIC' or
            np.asarray(o['model_xy_m']).shape != (20, 2)):
        raise ValueError('V4_RECORD_CONTRACT_UNKNOWN')
    return Plan(o['output_id'], 'V4_RECORD:model_xy_m', int(stamp['ns'])*1e-9,
                int(generated['ns'])*1e-9, expires_s,
                stamp['domain']+':'+stamp['clock_id'], stamp['epoch_id'], o['frame'],
                o['pose_reference_point'], tuple(tuple(p) for p in o['model_xy_m']))


def _transform(xy: np.ndarray, pose: tuple[float, float, float], inverse=False) -> np.ndarray:
    c, s = math.cos(pose[2]), math.sin(pose[2])
    rotation = np.array([[c, -s], [s, c]])
    return (xy-np.array(pose[:2]))@rotation if inverse else xy@rotation.T+pose[:2]


class ShadowBridge:
    """Caller invokes tick independently of plan delivery; no background timers.

    A rejection clears the plan. Never last-valid fallback. Expired output has
    valid=False, null command, and expires_s=now rather than an implied actuator stop.
    """
    def __init__(self, limits: Limits | None, *, fixture_mode: bool = False):
        self.limits = limits
        self.fixture_mode = fixture_mode
        self.plan = None
        self.world = None
        self.reference = None
        self.arc = None
        self.curvature = None
        self.last_source = None
        self.last_tick = None
        self.clock_epoch = None
        self.reason = 'NO_PLAN'
        self.last_input = None
        self.audit = {}

    def _invalidate(self, reason: str) -> None:
        self.plan = self.world = self.reference = self.arc = self.curvature = None
        self.reason = reason

    def _limits_valid(self) -> bool:
        p = self.limits
        if p is None or not p.provenance or p.provenance in ('UNKNOWN', 'MISSING'):
            return False
        if p.fixture_only and not self.fixture_mode:
            return False
        numbers = [v for k, v in vars(p).items() if k not in ('provenance', 'fixture_only')]
        return (all(type(v) in (int, float) and math.isfinite(v) for v in numbers) and
                all(v > 0 for k, v in vars(p).items() if k not in
                    ('provenance', 'fixture_only', 'rear_x_in_base_m', 'future_tolerance_s', 'stop_delay_s')) and
                p.future_tolerance_s >= 0 and p.stop_delay_s >= 0 and p.steer_rad < math.pi/2 and
                p.max_turn_rad < math.pi and p.period_s <= p.state_ttl_s)

    def _clock(self, now: float, clock: str, epoch: str) -> bool:
        if not math.isfinite(now) or not clock or not epoch:
            self._invalidate('CLOCK_UNKNOWN'); return False
        if ((self.clock_epoch is not None and self.clock_epoch != (clock, epoch)) or
                (self.last_tick is not None and now < self.last_tick)):
            self._invalidate('CLOCK_RESET')
            self.last_source = None
            self.clock_epoch = (clock, epoch)
            self.last_tick = now
            return False
        self.clock_epoch = (clock, epoch)
        return True

    def accept(self, plan: Plan, pose: PathPose | None, *, now_s: float) -> bool:
        """Geometric adapter; timed trajectory is explicitly unsupported, not deduped."""
        start = time.perf_counter()
        old = self.reference
        self.last_input = (plan.id, plan.source)
        self._invalidate('PLAN_REJECTED')
        def reject(reason):
            self._invalidate(reason); return False
        if not self._limits_valid(): return reject('VEHICLE_CONTRACT_UNKNOWN')
        if not self._clock(now_s, plan.clock, plan.epoch): return False
        p = self.limits
        if not all(math.isfinite(v) for v in (plan.source_s, plan.generated_mono_s, plan.expires_s)):
            return reject('PLAN_TIME_NONFINITE')
        if plan.source_s > now_s+p.future_tolerance_s: return reject('PLAN_FUTURE')
        if now_s-plan.source_s > p.path_ttl_s or now_s >= plan.expires_s: return reject('PLAN_STALE')
        if self.last_source is not None and plan.source_s <= self.last_source: return reject('PLAN_OUT_OF_ORDER')
        self.last_source = plan.source_s  # rejected new generation also blocks older fallback
        if not plan.id or not plan.source: return reject('PLAN_ID_MISSING')
        if plan.kind != 'GEOMETRIC': return reject('TIMED_TRAJECTORY_UNSUPPORTED')
        if plan.frame != 'base_link' or plan.reference_point != 'BASE_LINK_ORIGIN':
            return reject('FRAME_CONTRACT_UNKNOWN')
        if (pose is None or not pose.evidence or pose.plan_id != plan.id or
                (pose.clock, pose.epoch) != (plan.clock, plan.epoch) or pose.stamp_s != plan.source_s or
                len(pose.base_in_local) != 3 or not np.isfinite(pose.base_in_local).all()):
            return reject('OBSERVATION_TF_MISSING')
        if plan.speed_mps is not None and (plan.speed_plan_id != plan.id or not plan.speed_source or
                not math.isfinite(plan.speed_mps) or plan.speed_mps < 0):
            return reject('SPEED_PLAN_MISMATCH')
        try: raw = np.asarray(plan.xy_m, dtype=float)
        except (ValueError, TypeError): return reject('PATH_SHAPE')
        if raw.ndim != 2 or raw.shape[1:] != (2,) or not 2 <= len(raw) <= 2048:
            return reject('PATH_SHAPE')
        if not np.isfinite(raw).all(): return reject('PATH_NONFINITE')
        lengths = np.linalg.norm(np.diff(raw, axis=0), axis=1)
        if np.any(lengths > p.max_segment_m): return reject('PATH_DISCONTINUITY')
        clean = raw[np.r_[True, lengths > 1e-9]].copy()
        if len(clean) < 2: return reject('PATH_TOO_SHORT')
        seg = np.diff(clean, axis=0); ds = np.linalg.norm(seg, axis=1)
        angles = np.unwrap(np.arctan2(seg[:, 1], seg[:, 0]))
        turns = np.diff(angles)
        if np.any(np.abs(turns) > p.max_turn_rad): return reject('PATH_FOLDBACK')
        # Check vertex curvature before resampling; don't smooth a discontinuity away.
        k = turns/((ds[1:]+ds[:-1])/2)
        if np.any(np.abs(np.arctan(p.wheelbase_m*k)) > p.steer_rad): return reject('CURVATURE_INFEASIBLE')
        arc = np.r_[0., np.cumsum(ds)]
        if arc[-1] < p.lookahead_m: return reject('HORIZON_SHORT')
        if arc[-1]/p.resample_m > 4095: return reject('REFERENCE_BUDGET')
        query = np.unique(np.r_[np.arange(0., arc[-1], p.resample_m), arc[-1]])
        local = np.column_stack([np.interp(query, arc, clean[:, d]) for d in (0, 1)])
        world = _transform(local, pose.base_in_local)
        if old is not None and np.linalg.norm(world[0]-old[0]) > p.max_plan_jump_m:
            return reject('PLAN_JUMP')
        self.plan = deepcopy(plan)
        self.reference = world
        self.world = _transform(raw, pose.base_in_local)
        self.arc = query
        self.curvature = float(np.max(np.abs(k))) if len(k) else 0.
        self.reason = None
        # Exact piecewise-linear resampling, no fit. Symmetric point-to-polyline distance.
        def distance(points, poly):
            a = poly[:-1]; d = np.diff(poly, axis=0); den = np.sum(d*d, axis=1)
            den = np.maximum(den, 1e-24)
            return max(float(np.min(np.linalg.norm(a+np.clip(np.sum((x-a)*d, axis=1)/den,0,1)[:,None]*d-x,axis=1))) for x in points)
        self.audit = dict(raw_points=len(raw), reference_points=len(local), duplicate_removed=len(raw)-len(clean),
                          max_raw_reference_difference_m=max(distance(raw,local),distance(local,clean)),
                          adapter_duration_s=time.perf_counter()-start, actual_length_m=float(arc[-1]),
                          nominal_s_used_as_time=False, resampling='PIECEWISE_LINEAR_NO_SMOOTHING')
        return True

    def tick(self, *, now_s: float, clock: str, epoch: str, state: Vehicle | None) -> dict:
        started = time.perf_counter()
        previous_tick = self.last_tick
        good_clock = self._clock(now_s, clock, epoch)
        self.last_tick = now_s
        out = dict(valid=False, reason=self.reason, command=None, expires_s=now_s,
                   output_kind='IN_MEMORY_SHADOW_ONLY', actuator_connected=False,
                   plan_id=None if self.last_input is None else self.last_input[0],
                   source=None if self.last_input is None else self.last_input[1],
                   control_period_s=None if previous_tick is None else now_s-previous_tick,
                   collision_clearance='NOT_EVALUATED', **self.audit)
        def invalid(reason):
            self._invalidate(reason)
            out.update(reason=reason, processing_s=time.perf_counter()-started)
            return out
        if not good_clock: return invalid('CLOCK_RESET')
        if not self._limits_valid(): return invalid('VEHICLE_CONTRACT_UNKNOWN')
        if self.plan is None: return invalid(self.reason or 'NO_PLAN')
        p, plan = self.limits, self.plan
        out.update(input_age_s=now_s-plan.source_s, source_stamp_s=plan.source_s,
                   generated_monotonic_s=plan.generated_mono_s)
        if now_s >= min(plan.expires_s, plan.source_s+p.path_ttl_s): return invalid('PLAN_STALE')
        if (state is None or not state.id or (state.clock,state.epoch)!=(clock,epoch) or
                len(state.pose)!=3 or not np.isfinite((*state.pose,state.stamp_s,state.speed_mps,state.tire_steer_rad)).all()):
            return invalid('ODOMETRY_OR_STATE_MISSING')
        if not -p.future_tolerance_s <= now_s-state.stamp_s <= p.state_ttl_s: return invalid('STATE_STALE_OR_FUTURE')
        if state.speed_mps < 0 or state.speed_mps > p.speed_cap_mps or abs(state.tire_steer_rad)>p.steer_rad:
            return invalid('STATE_OUT_OF_LIMITS')
        rear = _transform(np.array([[p.rear_x_in_base_m,0.]]),state.pose)[0]
        segments = np.diff(self.reference,axis=0); a = self.reference[:-1]
        den = np.maximum(np.sum(segments**2,axis=1),1e-24)
        u = np.clip(np.sum((rear-a)*segments,axis=1)/den,0,1)
        projected = a+u[:,None]*segments
        j = int(np.argmin(np.linalg.norm(projected-rear,axis=1)))
        offset = float(np.linalg.norm(projected[j]-rear))
        heading = math.atan2(segments[j,1],segments[j,0])
        error = math.atan2(math.sin(heading-state.pose[2]),math.cos(heading-state.pose[2]))
        if offset>p.max_initial_offset_m or abs(error)>p.max_heading_error_rad: return invalid('INITIAL_DEVIATION')
        progress = self.arc[j]+u[j]*(self.arc[j+1]-self.arc[j])
        remaining = float(self.arc[-1]-progress)
        # Geometry-only trial speed is explicitly configured, never derived from point spacing.
        speed = p.speed_cap_mps if plan.speed_mps is None else min(plan.speed_mps,p.speed_cap_mps)
        source = 'EXPLICIT_TRIAL_POLICY' if plan.speed_mps is None else plan.speed_source
        if speed>0 and remaining<p.lookahead_m: return invalid('REMAINING_HORIZON_SHORT')
        curve_cap = math.sqrt(p.lateral_accel_mps2/max(self.curvature,1e-12))
        # v*delay + v²/(2b) <= remaining; terminal speed zero, not constant-speed feed.
        b, delay = p.brake_mps2, p.stop_delay_s+p.period_s
        stop_cap = max(0., math.sqrt((b*delay)**2+2*b*remaining)-b*delay)
        target_speed = min(speed,curve_cap,stop_cap)
        target_s = min(self.arc[-1], progress+p.lookahead_m)
        target_world = np.array([[np.interp(target_s,self.arc,self.reference[:,0]),
                                  np.interp(target_s,self.arc,self.reference[:,1])]])
        local = _transform(target_world,(rear[0],rear[1],state.pose[2]),inverse=True)
        if local[0,0]<=0 and speed>0: return invalid('TARGET_BEHIND')
        desired = math.atan2(2*p.wheelbase_m*local[0,1],float(np.sum(local[0]**2)))
        if abs(desired)>p.steer_rad: return invalid('PP_STEER_INFEASIBLE')
        cmd = control_from_waypoints(local,target_speed,state.speed_mps,
            ControllerConfig(p.wheelbase_m,p.lookahead_m,p.steer_rad,-p.brake_mps2,p.accel_mps2,1.))
        dt = p.period_s if previous_tick is None else now_s-previous_tick
        if dt<=0 or dt>2*p.period_s: return invalid('CONTROL_PERIOD_INVALID')
        steer = float(np.clip(cmd.steering_rad,state.tire_steer_rad-p.steer_rate_rps*dt,state.tire_steer_rad+p.steer_rate_rps*dt))
        if state.speed_mps**2*abs(math.tan(steer))/p.wheelbase_m>p.lateral_accel_mps2:
            return invalid('LATERAL_ACCEL_INFEASIBLE')
        out.update(valid=True,reason='SHADOW_TRACKABLE_NOT_COLLISION_PROOF',
                   expires_s=min(now_s+p.period_s,plan.expires_s,plan.source_s+p.path_ttl_s,state.stamp_s+p.state_ttl_s),
                   command=dict(tire_steering_rad=steer,acceleration_mps2=cmd.acceleration_mps2),
                   steering_rate_rps=(steer-state.tire_steer_rad)/dt,steer_rate_limited=steer!=cmd.steering_rad,
                   target_speed_mps=target_speed,speed_source=source,remaining_m=remaining,
                   cross_track_m=offset,heading_error_rad=error,state_id=state.id,
                   processing_s=time.perf_counter()-started)
        return out
