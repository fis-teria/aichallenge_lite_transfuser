"""Bounded timestamp join and bicycle yaw estimate, no ROS or actuator calls.

Velocity is body-frame m/s at the reported reference point. Steering is the
reported front-wheel angle in rad, positive left. Geometry is explicit, not
inferred from a mesh origin. Slip and report noise remain unmodelled.
"""
import math
from .local_odometry_v4 import LocalOdometry, LocalPose


class SteeringOdometry:
    """Interpolate steering at velocity stamp; never extrapolate or use raw wz."""

    def __init__(self, *, wheelbase_m: float, reference_left_offset_m: float,
                 max_gap_s: float = .25, capacity: int = 32) -> None:
        if not math.isfinite(wheelbase_m) or wheelbase_m <= 0:
            raise ValueError('WHEELBASE_REQUIRED')
        if not math.isfinite(reference_left_offset_m):
            raise ValueError('REFERENCE_OFFSET_REQUIRED')
        if type(capacity) is not int or capacity < 2:
            raise ValueError('INVALID_CAPACITY')
        self.wheelbase_m = wheelbase_m
        self.offset_m = reference_left_offset_m
        self.capacity = capacity
        self.core = LocalOdometry(max_gap_s=max_gap_s, max_speed_mps=10., max_yaw_rate_rps=2.)
        self.steering: dict[int, float] = {}
        self.pending: dict[int, tuple[float, float, float]] = {}
        self.last: dict[str, tuple] = {}

    def reset(self) -> None:
        self.core.reset()
        self.steering.clear()
        self.pending.clear()
        self.last.clear()

    def _accept(self, role: str, ns: int, values: tuple) -> bool:
        if self.core.fault:
            self.core.fail(self.core.fault)
        if type(ns) is not int or ns < 0:
            self.core.fail('INVALID_STAMP')
        prior = self.last.get(role)
        if prior is not None:
            if ns < prior[0]:
                self.core.fail('OUT_OF_ORDER_'+role)
            if ns == prior[0]:
                if values == prior[1]:
                    return False
                self.core.fail('CONFLICTING_'+role)
        self.last[role] = (ns, values)
        return True

    def add_steering(self, ns: int, angle_rad: float) -> None:
        if not math.isfinite(angle_rad) or abs(angle_rad) >= math.pi/2:
            self.core.fail('INVALID_STEERING')
        if self._accept('steering', ns, (angle_rad,)):
            if len(self.steering) >= self.capacity:
                self.core.fail('STEERING_BUFFER_FULL')
            self.steering[ns] = angle_rad

    def add_velocity(self, ns: int, vx: float, vy: float, raw_wz: float) -> None:
        if not all(math.isfinite(v) for v in (vx, vy)):
            self.core.fail('NONFINITE_LINEAR_VELOCITY')
        # Raw heading is diagnostic only, including nonfinite reports.
        if self._accept('velocity', ns, (vx, vy, repr(raw_wz))):
            if len(self.pending) >= self.capacity:
                self.core.fail('VELOCITY_BUFFER_FULL')
            self.pending[ns] = (vx, vy, raw_wz)

    def drain(self, now_ns: int) -> list[tuple[LocalPose, dict]]:
        if self.core.fault:
            self.core.fail(self.core.fault)
        output = []
        gap = int(self.core.max_gap_s*1e9)
        for ns in sorted(self.pending):
            if now_ns-ns > gap:
                self.core.fail('STEERING_JOIN_TIMEOUT')
            before = [s for s in self.steering if s <= ns]
            after = [s for s in self.steering if s >= ns]
            if not before or not after:
                break
            lo, hi = max(before), min(after)
            if hi-lo > gap:
                self.core.fail('STEERING_GAP')
            weight = 0. if lo == hi else (ns-lo)/(hi-lo)
            delta = self.steering[lo]*(1-weight)+self.steering[hi]*weight
            vx, vy, raw = self.pending[ns]
            curvature = math.tan(delta)/self.wheelbase_m
            # vx(point)=vx(rear)-w*left_offset; preserve measured vy(point).
            denominator = 1-self.offset_m*curvature
            if denominator <= 1e-6:
                self.core.fail('SINGULAR_REFERENCE_GEOMETRY')
            wz = vx*curvature/denominator
            pose = self.core.update(ns, vx, vy, wz)
            del self.pending[ns]
            if pose is not None:
                output.append((pose, dict(stamp_ns=ns, steering_stamps_ns=[lo, hi],
                    steering_weight=weight, steering_rad=delta, vx_mps=vx, vy_mps=vy,
                    raw_heading_rate=raw if math.isfinite(raw) else repr(raw),
                    estimated_yaw_rate_rps=wz, source='VELOCITY_STEERING_BICYCLE',
                    warning=self.core.warning)))
        # Keep latest predecessor for pending inputs; expire unused old history.
        cutoff = min(self.pending) if self.pending else now_ns-gap
        older = [s for s in self.steering if s <= cutoff]
        keep = max(older) if older else None
        for s in list(self.steering):
            if s < cutoff and s != keep:
                del self.steering[s]
        return output
