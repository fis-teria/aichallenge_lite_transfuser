"""Synthetic/offline time-plan to existing Pure Pursuit calculation; no ROS I/O.

All pose origins must explicitly be the rear axle. No implicit base_link offset.
Prediction horizon is NOT a mission endpoint. Safety/actuator integration is absent.
"""
from __future__ import annotations
from dataclasses import dataclass
import math
import numpy as np
from .waypoint_controller import ControllerConfig, ControlCommand, control_from_waypoints

TIME_NS=tuple(i*100_000_000 for i in range(1,31))

@dataclass(frozen=True)
class TimedBodyPose:
    stamp_ns: int
    clock: str
    epoch: str
    world_frame: str
    body_frame: str
    x_m: float
    y_m: float
    yaw_rad: float

    def __post_init__(self) -> None:
        if type(self.stamp_ns) is not int or self.stamp_ns<0 or not all((self.clock,self.epoch,self.world_frame,self.body_frame)):
            raise ValueError('invalid pose identity')
        if not np.isfinite([self.x_m,self.y_m,self.yaw_rad]).all():raise ValueError('nonfinite pose')

@dataclass(frozen=True)
class TimePlan:
    plan_id: str
    observation: TimedBodyPose
    xy_m: np.ndarray
    time_ns: tuple[int,...] = TIME_NS

    def __post_init__(self) -> None:
        if not self.plan_id or type(self.time_ns) is not tuple or any(type(t) is not int for t in self.time_ns) or self.time_ns!=TIME_NS:
            raise ValueError('time plan identity/grid mismatch')
        xy=np.array(self.xy_m,dtype=float,copy=True)
        if xy.shape!=(30,2) or not np.isfinite(xy).all():raise ValueError('time plan requires finite [30,2] metres')
        xy.setflags(write=False)
        object.__setattr__(self,'xy_m',xy)

@dataclass(frozen=True)
class TimeReference:
    plan_id: str
    remaining_sec: np.ndarray
    xy_current_m: np.ndarray
    target_speed_mps: float
    age_sec: float
    status: str = 'OFFLINE_CONTROL_CALCULATION_ONLY'


def prepare_time_reference(plan: TimePlan, current: TimedBodyPose, *, max_age_sec: float = .5,
                           speed_window_sec: float = .3) -> TimeReference:
    obs=plan.observation
    if (current.clock,current.epoch,current.world_frame,current.body_frame)!=(obs.clock,obs.epoch,obs.world_frame,obs.body_frame):
        raise ValueError('CLOCK_EPOCH_FRAME_MISMATCH')
    if current.body_frame!='rear_axle':raise ValueError('EXPLICIT_REAR_AXLE_TRANSFORM_REQUIRED')
    if not np.isfinite([max_age_sec,speed_window_sec]).all() or max_age_sec<=0 or speed_window_sec<=0:
        raise ValueError('invalid time reference settings')
    age=(current.stamp_ns-obs.stamp_ns)*1e-9
    if age<0:raise ValueError('CLOCK_RESET_OR_FUTURE_PLAN')
    if age>max_age_sec or age>=3.:raise ValueError('STALE_OR_EXHAUSTED_PLAN')
    times=np.arange(31,dtype=float)/10
    source=np.vstack((np.zeros((1,2)),plan.xy_m))
    def sample(t: np.ndarray) -> np.ndarray:
        return np.column_stack([np.interp(t,times,source[:,i]) for i in range(2)])
    remaining=np.r_[age,times[times>age]]
    points=sample(remaining)
    c,s=math.cos(obs.yaw_rad),math.sin(obs.yaw_rad)
    world=points@np.array([[c,s],[-s,c]])+np.array([obs.x_m,obs.y_m])
    c,s=math.cos(current.yaw_rad),math.sin(current.yaw_rad)
    local=(world-np.array([current.x_m,current.y_m]))@np.array([[c,-s],[s,c]])
    end=min(3.,age+speed_window_sec)
    speed_times=np.r_[age,times[(times>age)&(times<end)],end]
    # Integrate source interval chord lengths, never distance from current pose.
    speed=float(np.linalg.norm(np.diff(sample(speed_times),axis=0),axis=1).sum()/(end-age))
    return TimeReference(plan.plan_id,remaining-age,local,speed,age)


def reference_control(reference: TimeReference, *, current_speed_mps: float,
                      config: ControllerConfig = ControllerConfig()) -> ControlCommand:
    """Execute existing PP steering + bounded proportional longitudinal calculation.

    Zero-length stationary reference has no heading. Short moving references use
    existing PP endpoint fallback without imposing horizon-end speed zero.
    This is not a collision check, supervisor decision or authorization to drive.
    """
    if not math.isfinite(current_speed_mps) or current_speed_mps<0:raise ValueError('invalid measured speed')
    if not all(math.isfinite(float(v)) for v in vars(config).values()) or config.wheelbase_m<=0 or config.min_lookahead_m<=0 or config.max_steer_rad<=0 or not config.min_accel_mps2<0<config.max_accel_mps2 or config.speed_kp<0:
        raise ValueError('invalid controller configuration')
    if reference.target_speed_mps<=1e-9:
        return control_from_waypoints(np.zeros((1,2)),0.,current_speed_mps,config)
    points=reference.xy_current_m[1:]
    if not np.isfinite(points).all() or not np.any(points[:,0]>1e-6):raise ValueError('NO_FORWARD_REFERENCE')
    return control_from_waypoints(points[points[:,0]>1e-6],reference.target_speed_mps,current_speed_mps,config)


class TimeReferenceAdapter:
    """Retires cached plan on clock reset/epoch mismatch/expiry; never restamps."""
    def __init__(self) -> None:
        self.plan: TimePlan | None=None
        self.last: TimedBodyPose | None=None
        self.reason='NO_PLAN'

    def accept(self, plan: TimePlan) -> None:
        # A newly received plan may be newer than the previous poll. Validate
        # against the NEXT controller timestamp, never the previous cycle.
        self.plan=plan

    def poll(self, current: TimedBodyPose) -> TimeReference | None:
        if self.last is not None and ((current.clock,current.epoch)!=(self.last.clock,self.last.epoch) or current.stamp_ns<self.last.stamp_ns):
            self.plan=None;self.reason='CLOCK_RESET'
        self.last=current
        if self.plan is None:return None
        try:result=prepare_time_reference(self.plan,current)
        except ValueError as exc:
            self.reason=str(exc);self.plan=None;return None
        self.reason='OFFLINE_REFERENCE_READY'
        return result
