"""Bounded retained-route recovery; no invented waypoints or ROS dependencies.

Routes/poses use SLAM-world metres/radians, speeds m/s, stamps simulation ns.
This selector never authorizes motion. Its route must pass a fresh occupancy
and controller safety check before each command. No reverse manoeuvres.
"""
from __future__ import annotations

import math
from typing import Any
import numpy as np

RECOVERY_POLICY = 'retained_time_path_recovery_v1'
RECOVERY_SPEED_MPS = 3./3.6


def validate_recovery_policy(config: dict[str, Any]) -> str:
    policy = config.get('time_path_recovery_policy', 'off')
    if policy not in ('off', RECOVERY_POLICY) or (policy != 'off' and config.get('slam_mppi_policy', 'off') == 'off'):
        raise ValueError('TIME_RECOVERY_REQUIRES_SLAM_MPPI')
    return policy


def recoverable_geometry(xy: np.ndarray, reason: str) -> bool:
    """Only small finite stopped-prediction jitter, never a large malformed path."""
    xy = np.asarray(xy, dtype=float)
    return (reason in ('TIME_PATH_BACKTRACK', 'TIME_PATH_UNRESOLVED_EXCURSION')
            and xy.shape == (30, 2) and np.isfinite(xy).all()
            and float(np.linalg.norm(xy, axis=1).max()) <= .5)


def route_support(route: np.ndarray, pose: np.ndarray) -> tuple[float, float, float]:
    """Remaining arc metres, lateral distance metres, heading error radians."""
    delta = np.diff(route, axis=0)
    ds = np.linalg.norm(delta, axis=1)
    valid = np.flatnonzero(ds > .01)
    if not len(valid):
        return 0., math.inf, math.inf
    t = np.clip(np.sum((pose[:2]-route[:-1])*delta, axis=1)/np.maximum(ds*ds, 1e-12), 0., 1.)
    projected = route[:-1]+t[:, None]*delta
    distance = np.linalg.norm(projected-pose[:2], axis=1)
    i = int(valid[np.argmin(distance[valid])])
    yaw = math.atan2(delta[i, 1], delta[i, 0])-pose[2]
    return float((1-t[i])*ds[i]+ds[i+1:].sum()), float(distance[i]), abs(math.atan2(math.sin(yaw), math.cos(yaw)))


class RetainedPathRecovery:
    """At most two 6 s / 2.5 m attempts per runtime; healthy handback after .5 s.

    A retained reference is at most 5 s old on entry and 10 s old during an
    attempt. Missing packets suspend motion without refreshing any budget.
    """
    def __init__(self) -> None:
        self.route: np.ndarray | None = None
        self.route_stamp_ns: int | None = None
        self.last_ns: int | None = None
        self.bad_since_ns: int | None = None
        self.good_since_ns: int | None = None
        self.started_ns: int | None = None
        self.previous_xy: np.ndarray | None = None
        self.travel_m = 0.
        self.attempts = 0
        self.active = False
        self.exhausted = False
        self.last_reason = 'NORMAL'

    def select(self, fresh: np.ndarray | None, pose: np.ndarray, stamp_ns: int,
               speed_mps: float, *, avoidance_active: bool = False,
               explicit_stop: bool = False, source_stamp_ns: int | None = None) -> dict[str, Any]:
        pose = np.asarray(pose, dtype=float)
        if (pose.shape != (3,) or not np.isfinite(pose).all() or not math.isfinite(speed_mps)
                or speed_mps < -.03 or type(stamp_ns) is not int or stamp_ns < 0
                or (self.last_ns is not None and stamp_ns <= self.last_ns)):
            raise ValueError('TIME_RECOVERY_POSE_SPEED_CLOCK')
        if self.last_ns is not None and stamp_ns-self.last_ns > 350_000_000:
            self.good_since_ns = self.bad_since_ns = None
        self.last_ns = stamp_ns
        source_stamp_ns = stamp_ns if source_stamp_ns is None else source_stamp_ns
        if type(source_stamp_ns) is not int or not 0 <= stamp_ns-source_stamp_ns <= 500_000_000:
            raise ValueError('TIME_RECOVERY_SOURCE_STALE')
        if self.active and self.previous_xy is not None:
            self.travel_m += float(np.linalg.norm(pose[:2]-self.previous_xy))
        self.previous_xy = pose[:2].copy()
        if fresh is not None:
            fresh = np.asarray(fresh, dtype=float)
            if fresh.ndim != 2 or fresh.shape[1:] != (2,) or not 2 <= len(fresh) <= 200 or not np.isfinite(fresh).all():
                raise ValueError('TIME_RECOVERY_REFERENCE_SHAPE_FINITE')

        def result(mode: str, reason: str, route: np.ndarray | None) -> dict[str, Any]:
            self.last_reason = reason
            return dict(mode=mode, reason=reason, reference_world=route,
                        policy=RECOVERY_POLICY, stamp_ns=stamp_ns, attempts=self.attempts, travelled_m=self.travel_m,
                        started_ns=self.started_ns, reference_source_stamp_ns=self.route_stamp_ns)

        if explicit_stop or fresh is None:
            self.good_since_ns = self.bad_since_ns = None
            return result('STOP', 'EXPLICIT_STOP' if explicit_stop else 'FRESH_MODEL_REQUIRED', None)
        remaining, lateral, heading = route_support(fresh, pose)
        healthy = remaining >= 3.5 and lateral <= .3 and heading <= .3
        if healthy:
            if self.good_since_ns is None:
                self.good_since_ns = stamp_ns
        else:
            self.good_since_ns = None
        if self.active:
            if healthy and stamp_ns-self.good_since_ns >= 500_000_000:
                self.active = False
                self.bad_since_ns = None
                self.route = fresh.copy(); self.route_stamp_ns = source_stamp_ns
                return result('NOMINAL', 'RECOVERY_HAND_BACK', fresh)
            if (stamp_ns-self.started_ns >= 6_000_000_000 or self.travel_m >= 2.5
                    or stamp_ns-self.route_stamp_ns > 10_000_000_000):
                self.active = False; self.exhausted = True
                return result('STOP', 'RECOVERY_BUDGET_EXHAUSTED', None)
            available, distance, angle = route_support(self.route, pose)
            if available < 1.6 or distance > .75 or angle > .6:
                self.active = False; self.exhausted = True
                return result('STOP', 'RECOVERY_REFERENCE_EXHAUSTED', None)
            return result('RECOVER', 'RETAINED_REFERENCE', self.route)
        if self.exhausted:
            if healthy and stamp_ns-self.good_since_ns >= 500_000_000:
                self.exhausted = False
            else:
                return result('STOP', 'RECOVERY_EXHAUSTED_WAIT_HEALTHY_MODEL', None)
        if remaining >= 4. and lateral <= .3 and heading <= .4:
            self.route = fresh.copy(); self.route_stamp_ns = source_stamp_ns
        if avoidance_active or remaining >= 2.5:
            self.bad_since_ns = None
            return result('NOMINAL', 'NORMAL', fresh)
        if self.bad_since_ns is None:
            self.bad_since_ns = stamp_ns
        if stamp_ns-self.bad_since_ns < 300_000_000 or speed_mps > 2.:
            return result('NOMINAL', 'SHORT_REFERENCE_CONFIRMING', fresh)
        if self.attempts >= 2:
            return result('STOP', 'RECOVERY_ATTEMPT_LIMIT', None)
        if self.route is None or stamp_ns-self.route_stamp_ns > 5_000_000_000:
            return result('NOMINAL', 'NO_RECENT_RECOVERY_REFERENCE', fresh)
        available, distance, angle = route_support(self.route, pose)
        if available < 2. or distance > .75 or angle > .6:
            return result('NOMINAL', 'RECOVERY_REFERENCE_UNUSABLE', fresh)
        self.attempts += 1; self.active = True; self.started_ns = stamp_ns; self.travel_m = 0.
        return result('RECOVER', 'RECOVERY_ENTER', self.route)
