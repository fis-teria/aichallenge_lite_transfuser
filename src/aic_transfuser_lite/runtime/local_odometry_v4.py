"""ROS-free SE(2) integration of body-frame VelocityReport, SI units.

No GNSS, IMU, map, TF lookup, model, vehicle geometry or controller input.
The origin is the first accepted sample; drift is not globally corrected.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class LocalPose:
    stamp_ns: int
    x_m: float
    y_m: float
    yaw_rad: float
    epoch: int


class LocalOdometry:
    """The max_speed/max_yaw names are retained as warning thresholds only."""
    def __init__(self, *, max_gap_s: float, max_speed_mps: float,
                 max_yaw_rate_rps: float) -> None:
        if not all(math.isfinite(x) and x > 0 for x in (max_gap_s,max_speed_mps,max_yaw_rate_rps)):
            raise ValueError('ODOMETRY_LIMITS_REQUIRED')
        self.max_gap_s=max_gap_s
        self.max_speed_mps=max_speed_mps
        self.max_yaw_rate_rps=max_yaw_rate_rps
        self.epoch=-1
        self.reset()

    def reset(self) -> None:
        self.epoch+=1
        self.pose: LocalPose | None=None
        self.velocity: tuple[float,float,float] | None=None
        self.fault: str | None=None
        self.warning: dict | None=None

    def fail(self, reason: str) -> None:
        self.fault=reason
        raise ValueError(reason)

    def update(self, stamp_ns: int, vx: float, vy: float, wz: float) -> LocalPose | None:
        """Return a new pose once per stamp; exact duplicate returns None.

        Trapezoidal endpoint body twist is integrated with an SE(2) exponential.
        Any invalid observation latches a fault until explicit clock/session reset.
        """
        if self.fault: raise ValueError(self.fault)
        self.warning=None
        if type(stamp_ns) is not int or stamp_ns<0:
            self.fail('INVALID_STAMP')
        velocity=(vx,vy,wz)
        if not all(math.isfinite(v) for v in velocity): self.fail('NONFINITE_VELOCITY')
        speed_exceeded=math.hypot(vx,vy)>self.max_speed_mps
        yaw_exceeded=abs(wz)>self.max_yaw_rate_rps
        if self.pose is None:
            self.pose=LocalPose(stamp_ns,0.,0.,0.,self.epoch)
        else:
            dt=(stamp_ns-self.pose.stamp_ns)*1e-9
            if dt==0:
                if velocity==self.velocity: return None
                self.fail('CONFLICTING_VELOCITY_STAMP')
            if dt<0: self.fail('OUT_OF_ORDER')
            if dt>self.max_gap_s: self.fail('VELOCITY_GAP')
            assert self.velocity is not None
            u,v,w=(a*.5+b*.5 for a,b in zip(self.velocity,velocity))
            theta=w*dt
            if not math.isfinite(theta): self.fail('NONFINITE_INTEGRATION')
            a=dt if abs(theta)<1e-8 else math.sin(theta)/w
            b=dt*theta*.5 if abs(theta)<1e-8 else (1-math.cos(theta))/w
            dx,dy=a*u-b*v,b*u+a*v
            c,s=math.cos(self.pose.yaw_rad),math.sin(self.pose.yaw_rad)
            candidate=LocalPose(stamp_ns,self.pose.x_m+c*dx-s*dy,
                                self.pose.y_m+s*dx+c*dy,
                                math.atan2(math.sin(self.pose.yaw_rad+theta),
                                           math.cos(self.pose.yaw_rad+theta)),self.epoch)
            if not all(math.isfinite(v) for v in (candidate.x_m,candidate.y_m,candidate.yaw_rad)):
                self.fail('NONFINITE_INTEGRATION')
            self.pose=candidate
        self.velocity=velocity
        if speed_exceeded or yaw_exceeded:
            self.warning=dict(event='LOCAL_ODOMETRY_PROFILE_WARNING',stamp_ns=stamp_ns,
                              epoch=self.epoch,vx_mps=vx,vy_mps=vy,yaw_rate_rps=wz,
                              speed_exceeded=speed_exceeded,yaw_rate_exceeded=yaw_exceeded,
                              speed_warning_mps=self.max_speed_mps,
                              yaw_rate_warning_rps=self.max_yaw_rate_rps,
                              action='INTEGRATED_UNCLIPPED')
        return self.pose
