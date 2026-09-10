"""Prepare V4 geometry for a future existing-PP consumer; no ROS or command I/O.

This is a reference preparation stage, NOT a RUN permission or collision check.
Positions are metres in an explicitly bound fixed local frame, headings radians,
speeds m/s. Plan times use their named clock/epoch, never wall receipt time.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
import math

import numpy as np

from .path_control_bridge import Limits, PathPose, Plan, ShadowBridge


@dataclass(frozen=True)
class PreparedReference:
    plan_id: str
    source: str
    frame: str
    clock: str
    epoch: str
    source_s: float
    generated_mono_s: float
    expires_s: float
    xy_m: tuple[tuple[float, float], ...]
    yaw_rad: tuple[float, ...]
    speed_mps: tuple[float, ...]
    speed_source: str
    raw_reference_difference_m: float
    # Deliberately cannot represent an authorization to drive.
    status: str = 'PREPARED_NOT_CONTROL_AUTHORIZED'


class V4PPReferenceAdapter:
    """Reuse existing geometry validation without modifying V4 or PP.

    The caller must bind PathPose to fixed_frame from the actual pose source;
    neither an RViz Path nor a PLAN log missing clock/epoch supplies this binding.
    poll is required on every consumer cycle, including cycles without a plan.
    Missing live limits remain disabled; artificial limits require fixture_mode.
    """

    def __init__(self, limits: Limits | None, *, fixed_frame: str,
                 fixture_mode: bool = False, shadow_spacing_m: float | None = None):
        if shadow_spacing_m is not None and (not fixture_mode or shadow_spacing_m not in (.2,.3)):
            raise ValueError('SHADOW_ONLY_SPACING')
        self.shadow_spacing_m = shadow_spacing_m
        self._bridge = ShadowBridge(limits, fixture_mode=fixture_mode)
        self.fixed_frame = fixed_frame
        self._prepared: PreparedReference | None = None
        self._last_now: float | None = None
        self.reason = 'NO_PLAN'

    def _clear(self, reason: str) -> None:
        self._prepared = None
        # Retire the same generation in the reused geometry validator as well.
        self._bridge._invalidate(reason)
        self.reason = reason

    def accept(self, plan: Plan, pose: PathPose | None, *,
               pose_frame: str, now_s: float) -> bool:
        """Require uncorrected [20,2] V4 output and an explicit pose-frame binding."""
        if (self._last_now is not None and now_s < self._last_now):
            self._clear('CLOCK_RESET')
            self._last_now = now_s
            return False
        self._last_now = now_s
        if (not self.fixed_frame or self.fixed_frame in ('UNKNOWN', 'MISSING', 'base_link')
                or pose_frame != self.fixed_frame):
            self._clear('FIXED_FRAME_UNKNOWN')
            return False
        try:
            shape = np.asarray(plan.xy_m, dtype=float).shape
        except (TypeError, ValueError):
            shape = ()
        if plan.source != 'FIXED_V4_UNCORRECTED' or shape != (20, 2):
            self._clear('V4_OUTPUT_CONTRACT_MISMATCH')
            return False
        # Fixed V4 is geometry-only. Never reinterpret nominal point spacing as speed.
        if any(v is not None for v in (plan.speed_mps, plan.speed_source, plan.speed_plan_id)):
            self._clear('V4_SPEED_CONTRACT_MISMATCH')
            return False
        checked_plan=plan
        deviation_upper=0.
        if self.shadow_spacing_m is not None:
            from .shadow_resample_v4 import resample_shadow
            try:
                resampled,deviation_upper=resample_shadow(plan.xy_m,self.shadow_spacing_m,.03)
                checked_plan=replace(plan,xy_m=tuple(map(tuple,resampled)))
            except ValueError as exc:
                self._clear(str(exc)); return False
        if not self._bridge.accept(checked_plan, pose, now_s=now_s):
            self._prepared = None
            self.reason = self._bridge.reason
            return False
        limits = self._bridge.limits
        assert limits is not None
        if self.shadow_spacing_m is not None and deviation_upper+self._bridge.audit['max_raw_reference_difference_m']>.03:
            self._clear('FINAL_REFERENCE_DEVIATION_EXCEEDED'); return False
        xy = self._bridge.reference.copy()
        arc = self._bridge.arc.copy()
        delta = np.diff(xy, axis=0)
        heading = np.arctan2(delta[:, 1], delta[:, 0])
        # The terminal direction is the last nonzero segment, not atan2(0,0).
        yaw = np.r_[heading, heading[-1]]
        curvature = self._bridge.curvature
        curve_cap = math.sqrt(limits.lateral_accel_mps2 / max(curvature, 1e-12))
        cap = min(limits.speed_cap_mps, curve_cap)
        remaining = arc[-1] - arc
        delay = limits.stop_delay_s + limits.period_s
        bd = limits.brake_mps2 * delay
        speed = np.minimum(cap, np.sqrt(bd * bd + 2 * limits.brake_mps2 * remaining) - bd)
        speed[-1] = 0.0
        self._prepared = PreparedReference(
            plan.id, plan.source, self.fixed_frame, plan.clock, plan.epoch,
            plan.source_s, plan.generated_mono_s,
            min(plan.expires_s, plan.source_s + limits.path_ttl_s),
            tuple(map(tuple, xy)), tuple(yaw), tuple(speed),
            'EXPLICIT_TRIAL_POLICY_NOT_MODEL_SPEED',
            deviation_upper+self._bridge.audit['max_raw_reference_difference_m'])
        self.reason = 'PREPARED_NOT_CONTROL_AUTHORIZED'
        return True

    def poll(self, *, now_s: float, clock: str, epoch: str) -> PreparedReference | None:
        """Retire stale/reset references, never restamp or fall back to last-valid.

        Current vehicle-state, tracking and command-consumer checks remain necessary
        after this preparation stage; a non-null result is not a control permission.
        """
        if not math.isfinite(now_s):
            self._clear('CLOCK_UNKNOWN')
            return None
        if self._last_now is not None and now_s < self._last_now:
            self._clear('CLOCK_RESET')
        self._last_now = now_s
        value = self._prepared
        if value is None:
            return None
        if (clock, epoch) != (value.clock, value.epoch):
            self._clear('CLOCK_RESET')
        elif now_s >= value.expires_s:
            self._clear('PLAN_STALE')
        return self._prepared
