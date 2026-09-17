"""Longitudinal-only limiter over fresh local-SLAM occupied-path observations.

All distances are metres, speeds m/s, acceleration m/s^2, timestamps ns.
No path generation, steering output, object classification or global pose.
Braking deceleration is a simulation assumption, not a physical guarantee.
"""
from __future__ import annotations

import math
from typing import Any

SLAM_SLOWDOWN_POLICY = 'slam_path_slowdown_v1'


def validate_slam_slowdown_policy(config: dict[str, Any]) -> str:
    policy = config.get('slam_slowdown_policy', 'off')
    if policy not in ('off', SLAM_SLOWDOWN_POLICY):
        raise ValueError('SLAM_SLOWDOWN_POLICY')
    return policy


class SlamSlowdown:
    """Cap the existing target only; release at 0.5 m/s^2 after fresh clearance.

    Stop 1 m before observed occupancy, allowing 1.8 m from base to front,
    measured observation age, 0.5 s reaction allowance and 1 m/s^2 braking.
    Missing/invalid inputs request zero speed; recovery never latches steering.
    """
    def __init__(self) -> None:
        self.cap_mps: float | None = None

    def update(self, packet: dict[str, Any] | None, *, run_id: str,
               source_valid: bool, now_sim_ns: int, now_wall_ns: int,
               receipt_ns: int | None, speed_mps: float, target_mps: float,
               acceleration_mps2: float, dt_s: float) -> dict[str, Any]:
        if (not all(math.isfinite(x) for x in (speed_mps, target_mps, acceleration_mps2, dt_s))
                or speed_mps < -.03 or target_mps < 0 or not 0 <= dt_s <= .1):
            raise ValueError('SLAM_LONGITUDINAL_COMMAND_CONTRACT')
        valid = False; gap = None; age = None; stamp = None
        desired = 0.; reason = 'SLAM_INPUT_MISSING'
        try:
            if not source_valid:
                raise ValueError('SLAM_SOURCE_INVALID')
            if not isinstance(packet, dict):
                raise ValueError('SLAM_INPUT_MISSING')
            if (packet.get('event') != 'OBSTACLES' or packet.get('run_id') != run_id
                    or packet.get('frame') != 'time_slam_map'
                    or packet.get('gnss_imu_map_inputs') is not False):
                raise ValueError('SLAM_INPUT_IDENTITY')
            stamp = packet.get('stamp_ns')
            plan_stamp = packet.get('plan_observation_ns')
            if (type(stamp) is not int or type(plan_stamp) is not int
                    or type(receipt_ns) is not int
                    or not 0 <= now_sim_ns-stamp <= 350_000_000
                    or not 0 <= now_wall_ns-receipt_ns <= 350_000_000
                    or not 0 <= now_sim_ns-plan_stamp <= 500_000_000
                    or plan_stamp > stamp):
                raise ValueError('SLAM_INPUT_STALE_OR_UNMATCHED')
            if packet.get('input_valid') is not True or packet.get('path_valid') is not True:
                raise ValueError('SLAM_PATH_UNAVAILABLE')
            blocked = packet.get('path_blocked'); distance = packet.get('nearest_path_obstacle_m')
            if type(blocked) is not bool or (not blocked and distance is not None):
                raise ValueError('SLAM_OBSTACLE_CONTRACT')
            age = (now_sim_ns-stamp)/1e9
            desired = target_mps; reason = 'CLEAR'
            if blocked:
                if type(distance) not in (int, float) or not math.isfinite(distance) or distance < 0:
                    raise ValueError('SLAM_OBSTACLE_DISTANCE')
                gap = max(0., distance-1.8-1.-max(0., speed_mps)*age)
                desired = min(target_mps, math.sqrt(.5**2+2.*gap)-.5)
                if desired <= .05:
                    desired = 0.  # Avoid creeping indefinitely at the stop margin.
                reason = 'OBSTACLE_STOP' if desired <= 1e-6 else 'OBSTACLE_SPEED_CAP'
            valid = True
        except ValueError as exc:
            desired = 0.; reason = str(exc)
            if type(stamp) is not int:
                stamp = None
        # Immediate reduction; bounded release also covers a single clear flicker.
        if self.cap_mps is not None and desired > self.cap_mps:
            desired = min(desired, self.cap_mps+.5*dt_s)
            if reason == 'CLEAR' and desired < target_mps:
                reason = 'CLEAR_RELEASE'
        self.cap_mps = max(0., min(target_mps, desired))
        target = self.cap_mps
        acceleration = acceleration_mps2
        if target < target_mps-1e-9 or not valid:
            # Track the falling cap as distance closes, not only its current
            # speed error: d(v_cap)/dt = -v / sqrt(0.25 + 2*gap).
            feedforward = (-max(0., speed_mps)/math.sqrt(.25+2.*gap)
                           if valid and gap is not None else 0.)
            acceleration = min(acceleration, -1. if target <= 1e-6 else
                               max(-1., min(1., 2.*(target-max(0., speed_mps))+feedforward)))
        return dict(policy=SLAM_SLOWDOWN_POLICY, valid=valid, reason=reason,
                    scan_stamp_ns=stamp, scan_age_s=age, remaining_stop_gap_m=gap,
                    nominal_target_mps=target_mps, target_speed_mps=target,
                    nominal_acceleration_mps2=acceleration_mps2, acceleration_mps2=acceleration,
                    limited=target < target_mps-1e-9 or not valid,
                    steering_modified=False)
