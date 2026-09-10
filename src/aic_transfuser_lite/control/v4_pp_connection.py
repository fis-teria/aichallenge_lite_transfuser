"""Shadow-only PP connection state; no ROS, model inference or actuator I/O."""
import math
from .path_control_bridge import Limits, Plan, PathPose, Vehicle
from .v4_pp_reference_adapter import V4PPReferenceAdapter


def pp_center_pose(root_pose, wheelbase_m: float, rear_x_in_root_m: float):
    """Existing PP subtracts wheelbase/2: supply axle midpoint, not Vehicle root."""
    x, y, yaw = root_pose
    offset = rear_x_in_root_m + wheelbase_m / 2
    return (x + offset * math.cos(yaw), y + offset * math.sin(yaw), yaw)


class ShadowPPConnection:
    def __init__(self, limits: Limits, frame: str, shadow_spacing_m: float | None = None):
        # Artificial policy is permitted only in this non-actuating connection.
        self.shadow_spacing_m = shadow_spacing_m
        self.adapter = V4PPReferenceAdapter(limits, fixed_frame=frame, fixture_mode=True,shadow_spacing_m=shadow_spacing_m)
        self.limits = limits
        self.reason = 'NO_PLAN'
        self.context = None

    def invalidate(self, reason: str):
        self.adapter._clear(reason)
        self.reason = reason

    def accept(self, record: dict, now_s: float) -> bool:
        try:
            if record['event'] != 'PLAN':
                self.invalidate('SESSION_END'); return False
            if record['transport_kind'] != 'DIAGNOSTIC_NOT_CONTROL':
                raise ValueError('SOURCE_KIND')
            context = (record['session_id'], record['clock'], record['epoch'])
            if self.context is not None and self.context != context:
                self.adapter = V4PPReferenceAdapter(self.limits,
                    fixed_frame=self.adapter.fixed_frame, fixture_mode=True,shadow_spacing_m=self.shadow_spacing_m)
                self.context = context
                self.invalidate('CONTEXT_RESET'); return False
            self.context = context
            # A NEW shadow policy, not extension of the raw diagnostic RUN permission.
            # Original accepted/expiry fields remain unchanged in the received record.
            p = Plan(record['output_id'], record['source'], record['source_s'],
                record['generated_monotonic_s'], record['source_s']+self.limits.path_ttl_s,
                record['clock'], record['epoch'], record['frame'], record['reference_point'],
                tuple(map(tuple, record['raw_xy_m'])))
            tf = PathPose(p.id, p.source_s, p.clock, p.epoch,
                tuple(record['observation_pose_xyyaw']), record['observation_pose_evidence'])
            ok = self.adapter.accept(p, tf, pose_frame=record['pose_frame'], now_s=now_s)
            self.reason = self.adapter.reason
            return ok
        except (KeyError, TypeError, ValueError, OverflowError):
            self.invalidate('PLAN_PACKET_INVALID'); return False

    def tick(self, now_s: float, state: Vehicle | None):
        if self.context is None:
            return None
        _, clock, epoch = self.context
        reference = self.adapter.poll(now_s=now_s, clock=clock, epoch=epoch)
        if reference is None:
            self.reason = self.adapter.reason; return None
        if len(reference.xy_m) > 100:
            self.invalidate('ROS_TRAJECTORY_POINT_LIMIT'); return None
        check = self.adapter._bridge.tick(now_s=now_s, clock=clock, epoch=epoch, state=state)
        if not check['valid']:
            self.invalidate(check['reason']); return None
        self.reason = 'SHADOW_REFERENCE_READY'
        return reference
