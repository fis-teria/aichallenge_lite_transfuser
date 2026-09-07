"""Synthetic-only, map/ROS/I/O-free local stopping monitor.

All distances metres, angles radians, times seconds in ONE caller clock/epoch.
Geometry is a bounded union of square cells in an explicit fixed local frame.
LOCAL_CLEAR is conditional on supplied evidence, not motion or safety approval.
No live calibration defaults. No model, controller, publisher or persistent store.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
import math
from typing import Literal

Status = Literal['LOCAL_CLEAR', 'OBSTRUCTED', 'UNKNOWN', 'STALE', 'FAULT']
Cell = tuple[int, int]


@dataclass(frozen=True)
class State:
    id: str
    time_s: float
    x_m: float
    y_m: float
    yaw_rad: float
    speed_mps: float
    steer_rad: float
    acceleration_mps2: float
    xy_error_m: float
    yaw_error_rad: float
    speed_error_mps: float
    steer_error_rad: float = 0.
    acceleration_error_mps2: float = 0.


@dataclass(frozen=True)
class Operation:
    id: str
    acceleration_mps2: float
    steering_rate_rps: float
    duration_s: float
    expires_s: float


@dataclass(frozen=True)
class StopPolicy:
    version: str
    command_braking_mps2: float
    effective_braking_mps2: float | None
    jerk_mps3: float
    monitor_delay_s: float
    communication_delay_s: float
    brake_response_s: float
    steering_policy: str = 'HOLD_AT_BRAKE_START'


@dataclass(frozen=True)
class Profile:
    version: str
    calibration_source: str | None
    static_confirmed: bool
    detection_contract: str | None
    minimum_obstacle_width_m: float | None
    scan_height_confirmed: bool | None
    wheelbase_m: float
    rear_overhang_m: float
    front_overhang_m: float
    width_m: float
    max_speed_mps: float
    max_acceleration_mps2: float
    max_steer_rad: float
    max_steering_rate_rps: float
    # Model/actuation tracking budget, distinct from current-state measurement error.
    tracking_xy_m: float
    tracking_yaw_rad: float
    tracking_speed_mps: float
    cell_m: float
    extent_m: float
    dt_s: float
    horizon_s: float
    scan_ttl_s: float
    state_ttl_s: float
    decision_ttl_s: float
    max_steps: int
    max_cells: int
    max_rays: int
    max_scans: int
    max_checks: int


@dataclass(frozen=True)
class Scan:
    id: str
    instance: str
    epoch: str
    frame: str
    tf_version: str
    # Original observation time NEVER changes on reprojection/reuse.
    observed_s: float
    received_s: float
    x_m: float
    y_m: float
    yaw_rad: float
    angle_min_rad: float
    angle_increment_rad: float
    ranges_m: tuple[float, ...]
    range_min_m: float
    range_max_m: float
    range_error_m: float
    pose_error_m: float
    angle_error_rad: float
    acquisition_error_s: float


@dataclass(frozen=True)
class History:
    generation: str
    scans: tuple[Scan, ...]
    complete: bool = True  # Caller must mark loss/eviction; no hidden hit erasure.


@dataclass(frozen=True)
class InitialEvidence:
    id: str
    instance: str
    epoch: str
    frame: str
    tf_version: str
    profile_version: str
    observed_s: float
    valid_until_s: float
    kind: str
    # Explicitly confirmed rectangle; NOT automatically set to current footprint.
    confirmed_box_m: tuple[float, float, float, float]
    anchor_state: State


@dataclass(frozen=True)
class Request:
    instance: str
    epoch: str
    frame: str
    tf_version: str
    state: State
    previous: Operation
    candidate: Operation
    stop: StopPolicy
    profile: Profile
    history: History
    initial: InitialEvidence | None


@dataclass(frozen=True)
class Tube:
    time_s: float
    x_m: float
    y_m: float
    yaw_rad: float
    speed_mps: float
    steer_rad: float
    xy_budget_m: float
    yaw_budget_rad: float
    speed_budget_mps: float


@dataclass(frozen=True)
class Decision:
    status: Status
    reason: str
    binding_hash: str
    evaluated_s: float
    valid_until_s: float
    evidence_until_s: float
    checked_cells: tuple[Cell, ...]
    tubes: tuple[Tube, ...]
    stopped: bool
    travel_m: float
    candidate: Operation
    instance: str
    epoch: str
    frame: str
    tf_version: str
    profile_hash: str
    initial_hash: str
    evidence_trace: tuple[str, ...]
    runtime_permission: bool = False
    contact_telemetry: str = 'MISSING'
    stop_required: bool = True
    stopping_clearance: str = 'UNVERIFIED'


def content_hash(value: object) -> str:
    """Content binding (including invalid scan tokens), not an authentication MAC."""
    data = asdict(value) if hasattr(value, '__dataclass_fields__') else value
    return hashlib.sha256(json.dumps(data, sort_keys=True, separators=(',', ':'),
                                    allow_nan=True).encode()).hexdigest()


class _Budget(Exception):
    pass


class _Checks:
    def __init__(self, maximum: int):
        self.remaining = maximum

    def use(self, n: int = 1) -> None:
        self.remaining -= n
        if self.remaining < 0:
            raise _Budget('GEOMETRY_BUDGET')


def _angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _finite(*values: float) -> bool:
    return all(type(v) in (float, int) and math.isfinite(v) for v in values)


def _validate(r: Request, now: float) -> tuple[Status, str] | None:
    p, s = r.profile, r.state
    def missing(record: object) -> bool:
        return any(v is None or (isinstance(v, str) and v in ('MISSING', 'UNKNOWN'))
                   for v in asdict(record).values())
    if missing(p) or missing(r.stop):
        return 'UNKNOWN', 'PROFILE_MISSING_OR_NONSTATIC'
    if any(missing(record) for record in (s, r.previous, r.candidate)):
        return 'UNKNOWN', 'STATE_OR_OPERATION_MISSING'
    for record in (s, r.previous, r.candidate, r.stop, p):
        for v in asdict(record).values():
            if isinstance(v, (int, float)) and not isinstance(v, bool) and not math.isfinite(v):
                return 'FAULT', 'NONFINITE_CONTRACT'
    if not _finite(now) or not all((r.instance, r.epoch, r.frame, r.tf_version, s.id,
                                    r.candidate.id, r.previous.id, p.version, r.stop.version)):
        return 'FAULT', 'IDENTITY_OR_TIME'
    positive = (p.wheelbase_m, p.width_m, p.max_speed_mps, p.max_acceleration_mps2,
                p.max_steer_rad, p.max_steering_rate_rps, p.cell_m, p.extent_m, p.dt_s,
                p.horizon_s, p.scan_ttl_s, p.state_ttl_s, p.decision_ttl_s,
                r.stop.command_braking_mps2, r.stop.jerk_mps3)
    nonnegative = (p.front_overhang_m, p.rear_overhang_m, p.tracking_xy_m,
                   p.tracking_yaw_rad, p.tracking_speed_mps, s.xy_error_m, s.yaw_error_rad,
                   s.speed_error_mps, r.previous.duration_s, r.candidate.duration_s,
                   s.steer_error_rad, s.acceleration_error_mps2,
                   r.stop.monitor_delay_s, r.stop.communication_delay_s, r.stop.brake_response_s)
    if not _finite(*positive, *nonnegative) or min(positive) <= 0 or min(nonnegative) < 0 or not 0 < p.max_steer_rad < 1.3:
        return 'FAULT', 'INVALID_BOUNDS'
    for value, maximum in ((p.max_steps, 2000), (p.max_cells, 20000), (p.max_rays, 4096),
                            (p.max_scans, 32), (p.max_checks, 5_000_000)):
        if type(value) is not int or not 1 <= value <= maximum:
            return 'FAULT', 'INVALID_RESOURCE_BUDGET'
    if (s.speed_mps < 0 or s.speed_mps+s.speed_error_mps > p.max_speed_mps or
            abs(s.steer_rad) > p.max_steer_rad or
            not -r.stop.command_braking_mps2 <= s.acceleration_mps2 <= p.max_acceleration_mps2):
        return 'FAULT', 'STATE_OUT_OF_PROFILE'
    for op in (r.previous, r.candidate):
        if (not -r.stop.command_braking_mps2 <= op.acceleration_mps2 <= p.max_acceleration_mps2 or
                abs(op.steering_rate_rps) > p.max_steering_rate_rps):
            return 'FAULT', 'OPERATION_OUT_OF_PROFILE'
    if now < s.time_s or now-s.time_s > p.state_ttl_s:
        return 'STALE', 'STATE_STALE'
    if (r.candidate.duration_s <= 0 or now+r.previous.duration_s+r.candidate.duration_s > r.candidate.expires_s or
            (r.previous.duration_s > 0 and now+r.previous.duration_s > r.previous.expires_s)):
        return 'STALE', 'OPERATION_EXPIRED'
    if (not p.calibration_source or p.static_confirmed is not True or
            p.scan_height_confirmed is not True or p.detection_contract != 'OPAQUE_MIN_WIDTH_V1' or
            p.minimum_obstacle_width_m is None or p.minimum_obstacle_width_m <= 0 or
            r.stop.effective_braking_mps2 is None):
        return 'UNKNOWN', 'PROFILE_MISSING_OR_NONSTATIC'
    if not 0 < r.stop.effective_braking_mps2 <= r.stop.command_braking_mps2:
        return 'FAULT', 'INVALID_EFFECTIVE_BRAKING'
    if r.stop.steering_policy != 'HOLD_AT_BRAKE_START':
        return 'UNKNOWN', 'STOP_POLICY_UNSUPPORTED'
    if s.steer_error_rad != 0 or s.acceleration_error_mps2 != 0:
        return 'UNKNOWN', 'STEER_OR_ACCEL_ERROR_MODEL_UNSUPPORTED'
    if not r.history.complete:
        return 'UNKNOWN', 'HISTORY_LOSS'
    if not r.history.generation or not 1 <= len(r.history.scans) <= p.max_scans:
        return 'UNKNOWN', 'HISTORY_BUDGET_OR_MISSING'
    ids: set[str] = set()
    for scan in r.history.scans:
        if scan.id in ids or not scan.id:
            return 'FAULT', 'DUPLICATE_SCAN_ID'
        ids.add(scan.id)
        if (scan.instance, scan.epoch, scan.frame, scan.tf_version) != (r.instance, r.epoch, r.frame, r.tf_version):
            return 'UNKNOWN', 'SCAN_FRAME_EPOCH_TF_CHANGED'
        numbers = [v for k, v in asdict(scan).items()
                   if k not in ('ranges_m', 'id', 'instance', 'epoch', 'frame', 'tf_version')]
        if any(v is None or (isinstance(v, str) and v in ('MISSING', 'UNKNOWN')) for v in numbers):
            return 'UNKNOWN', 'SCAN_BOUNDS_MISSING'
        if (not _finite(*numbers) or not 2 <= len(scan.ranges_m) <= p.max_rays or
                scan.angle_increment_rad <= 0 or (len(scan.ranges_m)-1)*scan.angle_increment_rad > 2*math.pi or
                not 0 <= scan.range_min_m < scan.range_max_m or
                min(scan.range_error_m, scan.pose_error_m, scan.angle_error_rad, scan.acquisition_error_s) < 0 or
                scan.angle_error_rad >= math.pi/2):
            return 'FAULT', 'INVALID_SCAN_CONTRACT'
        if not 0 <= scan.observed_s <= scan.received_s <= now:
            return 'FAULT', 'SCAN_CLOCK_ORDER'
    if all(now-(x.observed_s-x.acquisition_error_s) >= p.scan_ttl_s for x in r.history.scans):
        return 'STALE', 'ALL_SCANS_EXPIRED'
    return None


def _footprint_cells(t: Tube, p: Profile, extra_m: float, checks: _Checks) -> set[Cell]:
    """Conservative OBB/cell SAT. Square cells intersecting the inflated body.

    State/model/temporal errors go here; sensor errors NEVER enter this function.
    Isotropic rotation displacement bound is converted to an OBB expansion.
    """
    rear, front = p.rear_overhang_m, p.wheelbase_m+p.front_overhang_m
    radius = math.hypot(max(rear, front), p.width_m/2)
    margin = t.xy_budget_m+radius*min(2., abs(t.yaw_budget_rad))+extra_m
    hx, hy = (front+rear)/2+margin, p.width_m/2+margin
    c, s = math.cos(t.yaw_rad), math.sin(t.yaw_rad)
    cx, cy = t.x_m+(front-rear)/2*c, t.y_m+(front-rear)/2*s
    ex, ey = abs(c)*hx+abs(s)*hy, abs(s)*hx+abs(c)*hy
    if max(abs(cx)+ex, abs(cy)+ey) > p.extent_m:
        raise _Budget('LOCAL_EXTENT_BUDGET')
    x0, x1 = math.floor((cx-ex)/p.cell_m), math.floor((cx+ex)/p.cell_m)
    y0, y1 = math.floor((cy-ey)/p.cell_m), math.floor((cy+ey)/p.cell_m)
    if (x1-x0+1)*(y1-y0+1) > p.max_cells:
        raise _Budget('CELL_BUDGET')
    result: set[Cell] = set()
    h = p.cell_m/2
    for i in range(x0, x1+1):
        for j in range(y0, y1+1):
            checks.use()
            dx, dy = (i+.5)*p.cell_m-cx, (j+.5)*p.cell_m-cy
            if (abs(dx) <= ex+h and abs(dy) <= ey+h and
                    abs(c*dx+s*dy) <= hx+h*(abs(c)+abs(s)) and
                    abs(-s*dx+c*dy) <= hy+h*(abs(c)+abs(s))):
                result.add((i, j))
    return result


def _rollout(r: Request, now: float, checks: _Checks) -> tuple[set[Cell], tuple[Tube, ...], float]:
    """Candidate first, then jerk-limited brake. Finite synthetic bicycle envelope.

    Acceleration is slew-limited; steering rate is saturated by explicit bounds.
    Each interval is covered by a speed/yaw-rate displacement bound, not just its
    endpoint rectangles. Numerical integration error grows separately.
    """
    p, s, stop = r.profile, r.state, r.stop
    x, y, yaw, v, delta, a = s.x_m, s.y_m, s.yaw_rad, s.speed_mps, s.steer_rad, s.acceleration_mps2
    upper_v = v+s.speed_error_mps+p.tracking_speed_mps
    lower_v = max(0., v-s.speed_error_mps-p.tracking_speed_mps)
    lower_a = a
    reaction = stop.monitor_delay_s+stop.communication_delay_s+stop.brake_response_s
    phases = [(now-s.time_s+r.previous.duration_s, r.previous.acceleration_mps2, r.previous.steering_rate_rps),
              (r.candidate.duration_s, r.candidate.acceleration_mps2, r.candidate.steering_rate_rps),
              (reaction, r.candidate.acceleration_mps2, r.candidate.steering_rate_rps)]
    cells: set[Cell] = set()
    tubes: list[Tube] = []
    elapsed = travel = numeric_xy = numeric_yaw = 0.
    omega = p.max_speed_mps*math.tan(p.max_steer_rad)/p.wheelbase_m
    accel_bound = max(stop.command_braking_mps2, p.max_acceleration_mps2)
    yaw_accel = (accel_bound*math.tan(p.max_steer_rad)+
                 p.max_speed_mps*p.max_steering_rate_rps/math.cos(p.max_steer_rad)**2)/p.wheelbase_m
    body_radius = math.hypot(max(p.rear_overhang_m, p.wheelbase_m+p.front_overhang_m), p.width_m/2)
    def capture(dt: float) -> None:
        t = Tube(s.time_s+elapsed, x, y, yaw, v, delta,
                 s.xy_error_m+p.tracking_xy_m+numeric_xy,
                 s.yaw_error_rad+p.tracking_yaw_rad+numeric_yaw,
                 max(upper_v-v, v-lower_v, 0.))
        tubes.append(t)
        cells.update(_footprint_cells(t, p, (p.max_speed_mps+body_radius*omega)*dt, checks))
        if len(cells) > p.max_cells:
            raise _Budget('CELL_BUDGET')
    step = 0
    for duration, wanted, rate in phases+[(math.inf, -float(stop.effective_braking_mps2), 0.)]:
        braking = math.isinf(duration)
        remaining = duration
        while remaining > 1e-12:
            # Candidate phases cannot be skipped merely because initial v=a=0.
            if braking and upper_v <= 0 and a <= 0:
                capture(0.)
                return cells, tuple(tubes), travel
            if step >= p.max_steps or elapsed >= p.horizon_s:
                raise _Budget('STOP_NOT_REACHED_WITHIN_BUDGET')
            dt = min(p.dt_s, remaining, p.horizon_s-elapsed)
            capture(dt)
            # Braking response lies between commanded maximum and guaranteed
            # effective braking. Keep both speed envelopes, not just nominal stop.
            upper_wanted = max(wanted, -float(stop.effective_braking_mps2))
            lower_wanted = -stop.command_braking_mps2 if braking else wanted
            a_next = max(a-stop.jerk_mps3*dt, min(a+stop.jerk_mps3*dt, upper_wanted))
            lower_a = max(lower_a-stop.jerk_mps3*dt, min(lower_a+stop.jerk_mps3*dt, lower_wanted))
            speed_error = max(upper_v-v, v-lower_v, 0.)
            nv = max(0., v+a_next*dt)
            upper_v = max(0., upper_v+a_next*dt)
            lower_v = max(0., lower_v+lower_a*dt)
            if upper_v > p.max_speed_mps:
                raise _Budget('REACHABLE_SPEED_OUT_OF_PROFILE')
            distance = (v+nv)*dt/2
            x += distance*math.cos(yaw)
            y += distance*math.sin(yaw)
            yaw += distance*math.tan(delta)/p.wheelbase_m
            delta = max(-p.max_steer_rad, min(p.max_steer_rad, delta+rate*dt))
            # Explicit bounded numerical and state/model error propagation.
            speed_error = max(speed_error, upper_v-nv, nv-lower_v, 0.)
            numeric_xy += (speed_error+p.max_speed_mps*(
                s.yaw_error_rad+p.tracking_yaw_rad+numeric_yaw))*dt + .5*(accel_bound+p.max_speed_mps*omega)*dt**2
            numeric_yaw += .5*yaw_accel*dt**2 + speed_error*math.tan(p.max_steer_rad)/p.wheelbase_m*dt
            elapsed += dt
            remaining -= dt
            travel += distance
            v, a = nv, a_next
            step += 1
    raise AssertionError('unreachable')


def _initial_cells(r: Request, now: float, previous: Decision | None, checks: _Checks) -> tuple[set[Cell], float, str]:
    e, p, s = r.initial, r.profile, r.state
    if e is None:
        return set(), now, 'INITIAL_MISSING'
    if ((e.instance, e.epoch, e.frame, e.tf_version, e.profile_version) !=
            (r.instance, r.epoch, r.frame, r.tf_version, p.version) or
            not e.kind or len(e.confirmed_box_m) != 4 or not _finite(e.observed_s, e.valid_until_s, *e.confirmed_box_m) or
            not e.observed_s == e.anchor_state.time_s <= now < e.valid_until_s):
        return set(), now, 'INITIAL_EXPIRED_OR_CHANGED'
    x0, y0, x1, y1 = e.confirmed_box_m
    if x1 <= x0 or y1 <= y0 or max(map(abs, e.confirmed_box_m)) > p.extent_m:
        return set(), now, 'INITIAL_SCOPE_INVALID'
    if s == e.anchor_state:
        cells = set()
        for i in range(math.ceil(x0/p.cell_m), math.floor(x1/p.cell_m)):
            for j in range(math.ceil(y0/p.cell_m), math.floor(y1/p.cell_m)):
                checks.use()
                cells.add((i, j))
                if len(cells) > p.max_cells:
                    raise _Budget('INITIAL_CELL_BUDGET')
        return cells, e.valid_until_s, 'INITIAL_EXPLICIT_SCOPE'
    if previous is None or previous.status != 'LOCAL_CLEAR':
        return set(), now, 'INITIAL_CONTINUATION_UNPROVEN'
    if ((previous.instance, previous.epoch, previous.frame, previous.tf_version, previous.profile_hash) !=
            (r.instance, r.epoch, r.frame, r.tf_version, content_hash(p)) or
            previous.initial_hash != content_hash(e) or
            now >= previous.evidence_until_s or r.previous.id != previous.candidate.id or
            r.previous.acceleration_mps2 != previous.candidate.acceleration_mps2 or
            r.previous.steering_rate_rps != previous.candidate.steering_rate_rps):
        return set(), now, 'INITIAL_CONTINUATION_CHANGED'
    # Restrictive timed membership: measured state+error must fit a prior tube.
    matches = [t for t in previous.tubes if abs(t.time_s-s.time_s) <= 1e-9 and
               math.hypot(t.x_m-s.x_m, t.y_m-s.y_m)+s.xy_error_m <= t.xy_budget_m+1e-12 and
               abs(_angle(t.yaw_rad-s.yaw_rad))+s.yaw_error_rad <= t.yaw_budget_rad+1e-12 and
               abs(t.speed_mps-s.speed_mps)+s.speed_error_mps <= t.speed_budget_mps+1e-12 and
               abs(t.steer_rad-s.steer_rad) <= 1e-12]
    if not matches:
        return set(), now, 'INITIAL_STATE_OUTSIDE_CHECKED_TUBE'
    current = _footprint_cells(Tube(s.time_s, s.x_m, s.y_m, s.yaw_rad, s.speed_mps, s.steer_rad,
                                   s.xy_error_m, s.yaw_error_rad, s.speed_error_mps), p, 0., checks)
    if not current.issubset(previous.checked_cells):
        return set(), now, 'INITIAL_FOOTPRINT_OUTSIDE_CHECKED_REGION'
    return set(previous.checked_cells), min(e.valid_until_s, previous.evidence_until_s), 'INITIAL_MAINTAINED_WITHIN_CHECKED_TUBE'


def _valid_hit(scan: Scan, value: float) -> bool:
    # Exact max, no-return, NaN/inf and out-of-range do not certify FREE.
    return _finite(value) and scan.range_min_m < value < scan.range_max_m


def _cell_hit(cell: Cell, scan: Scan, index: int, p: Profile) -> bool:
    v = scan.ranges_m[index]
    angle = scan.yaw_rad+scan.angle_min_rad+index*scan.angle_increment_rad
    x, y = scan.x_m+v*math.cos(angle), scan.y_m+v*math.sin(angle)
    translation, angular = _sensor_error(scan, p)
    error = scan.range_error_m+translation+v*min(2., angular)
    lo_x, lo_y = cell[0]*p.cell_m, cell[1]*p.cell_m
    dx = max(lo_x-x, 0., x-lo_x-p.cell_m)
    dy = max(lo_y-y, 0., y-lo_y-p.cell_m)
    return math.hypot(dx, dy) <= error


def _sensor_error(scan: Scan, p: Profile) -> tuple[float, float]:
    """Sensor uncertainty owner: no repetition in footprint inflation."""
    return (scan.pose_error_m+p.max_speed_mps*scan.acquisition_error_s,
            scan.angle_error_rad+p.max_speed_mps*math.tan(p.max_steer_rad)/p.wheelbase_m*scan.acquisition_error_s)


def _cell_free(cell: Cell, scan: Scan, p: Profile, checks: _Checks) -> bool:
    """Whole cell inside eroded observed sector under explicit detection contract.

    Beam gaps only certify space under the caller's opaque minimum-width AND
    scan-height detectability assumption. This is not calibrated for AWSIM.
    """
    x = (cell[0]+.5)*p.cell_m-scan.x_m
    y = (cell[1]+.5)*p.cell_m-scan.y_m
    distance = math.hypot(x, y)
    translation, angular = _sensor_error(scan, p)
    radius = p.cell_m/math.sqrt(2)+translation
    if distance <= radius or distance-radius <= scan.range_min_m:
        return False
    angle = _angle(math.atan2(y, x)-scan.yaw_rad)
    span = math.asin(radius/distance)+angular
    lo = math.floor((angle-span-scan.angle_min_rad)/scan.angle_increment_rad)
    hi = math.ceil((angle+span-scan.angle_min_rad)/scan.angle_increment_rad)
    if lo < 0 or hi >= len(scan.ranges_m):
        return False
    checks.use(hi-lo+1)
    selected = scan.ranges_m[lo:hi+1]
    if not all(_valid_hit(scan, v) for v in selected):
        return False
    reach = distance+radius
    gap = 2*reach*min(2., scan.angle_increment_rad/2+angular)+2*scan.range_error_m
    if gap >= float(p.minimum_obstacle_width_m):
        return False
    return reach < min(selected)-scan.range_error_m


def evaluate(r: Request, *, now_s: float, prior: Decision | None = None) -> Decision:
    """Pure bounded evaluation. No environmental map parameter or I/O.

    Frames/relative poses must already have explicit, bounded provenance in r.
    Evidence supplied by callers is trusted typed input, not independently sensed.
    """
    trace: list[str] = []
    cells: set[Cell] = set()
    tubes: tuple[Tube, ...] = ()
    travel = 0.
    stopped = False
    evidence_until = now_s
    def result(status: Status, reason: str) -> Decision:
        deadline = min(now_s+r.profile.decision_ttl_s, r.candidate.expires_s,
                       r.state.time_s+r.profile.state_ttl_s, evidence_until) if status == 'LOCAL_CLEAR' else now_s
        return Decision(status, reason, content_hash(r), now_s, deadline, evidence_until,
                        tuple(sorted(cells)), tubes, stopped, travel, r.candidate,
                        r.instance, r.epoch, r.frame, r.tf_version, content_hash(r.profile), content_hash(r.initial), tuple(trace),
                        stop_required=status != 'LOCAL_CLEAR',
                        stopping_clearance='CONDITIONAL_MODEL_CLEAR' if status == 'LOCAL_CLEAR' else 'UNVERIFIED')
    invalid = _validate(r, now_s)
    if invalid:
        return result(*invalid)
    checks = _Checks(r.profile.max_checks)
    try:
        seed, seed_until, seed_reason = _initial_cells(r, now_s, prior, checks)
        trace.append(seed_reason)
        cells, tubes, travel = _rollout(r, now_s, checks)
        stopped = True
        # Old hit evidence is retained even when it no longer certifies FREE.
        # No UNKNOWN scan clears an old hit; bounded history loss is UNKNOWN.
        for scan in r.history.scans:
            fresh = now_s-(scan.observed_s-scan.acquisition_error_s) < r.profile.scan_ttl_s
            trace.append(f'{scan.id}:' + ('FREE_ELIGIBLE' if fresh else 'FREE_TTL_EXPIRED_HITS_RETAINED'))
            for index, value in enumerate(scan.ranges_m):
                if not _valid_hit(scan, value):
                    continue
                for cell in cells | seed:
                    checks.use()
                    if _cell_hit(cell, scan, index, r.profile):
                        if cell in seed:
                            trace.append(f'{scan.id}:INITIAL_EVIDENCE_REVOKED_BY_HIT')
                            return result('OBSTRUCTED', 'INITIAL_HIT_CONTRADICTION')
                        return result('OBSTRUCTED', 'STOPPING_REGION_HIT')
        if seed_reason not in ('INITIAL_EXPLICIT_SCOPE', 'INITIAL_MAINTAINED_WITHIN_CHECKED_TUBE'):
            return result('UNKNOWN', seed_reason)
        # Finite static history: reuse original observation times, never now.
        fresh_scans = [x for x in r.history.scans if now_s-(x.observed_s-x.acquisition_error_s) < r.profile.scan_ttl_s]
        evidence_until = min(seed_until, *(x.observed_s-x.acquisition_error_s+r.profile.scan_ttl_s for x in fresh_scans))
        for cell in cells-seed:
            if not any(_cell_free(cell, scan, r.profile, checks) for scan in fresh_scans):
                trace.append(f'UNKNOWN_CELL:{cell[0]},{cell[1]}')
                return result('UNKNOWN', 'UNOBSERVED_STOPPING_REGION')
        return result('LOCAL_CLEAR', 'CONDITIONAL_STATIC_PROFILE_CLEAR')
    except _Budget as exc:
        trace.append(str(exc))
        return result('UNKNOWN', str(exc))


def revalidate(decision: Decision, request: Request, *, now_s: float, monitor_alive: bool) -> tuple[bool, str]:
    """Fail-closed pre-send check, NOT a send operation. Any content change revokes.

    Exact binding is deliberately stricter than allowing a changed adapter command.
    UNKNOWN et al never authorize resending the old ordinary operation.
    """
    if not monitor_alive:
        return False, 'MONITOR_STOPPED'
    if decision.status != 'LOCAL_CLEAR':
        return False, 'NO_CLEAR_DECISION'
    if not _finite(now_s) or not decision.evaluated_s <= now_s < decision.valid_until_s:
        return False, 'DECISION_EXPIRED'
    if content_hash(request) != decision.binding_hash:
        return False, 'BINDING_CHANGED'
    return True, 'MATCHED_CONDITIONAL_DECISION_NOT_PUBLISH'
