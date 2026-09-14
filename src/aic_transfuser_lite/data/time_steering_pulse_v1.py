"""Finite teacher-only steering perturbation; immutable proposals commit on publish.

Angles are ROS steering-input rad and body-heading rad, never CARLA controls.
The caller must apply its actuator limiter and stopping-sweep guard afterwards.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
import math

import numpy as np


@dataclass(frozen=True)
class SteeringPulseConfig:
    start_s_m: float
    amplitude_rad: float
    duration_s: float = 1.0
    start_window_m: float = 1.0
    release_ramp_s: float = .15
    recovery_s: float = 10.0
    max_lateral_m: float = .25
    max_heading_rad: float = math.radians(4.)
    goal_lateral_m: float = .05
    goal_heading_rad: float = math.radians(2.)
    entry_lateral_m: float = .05
    entry_heading_rad: float = math.radians(1.)
    min_speed_mps: float = 1.15
    max_speed_mps: float = 1.4
    plateau_s: float = 0.0

    def __post_init__(self) -> None:
        if not all(type(v) in (float, int) and math.isfinite(v) for v in self.__dict__.values()):
            raise ValueError('PULSE_CONFIG_FINITE_SI')
        if (not 10. <= self.start_s_m <= 300. or not 0. < abs(self.amplitude_rad) <= .1
                or not .25 <= self.duration_s <= 2. or not .25 <= self.start_window_m <= 2.
                or not 0. <= self.plateau_s <= max(0., self.duration_s-.5)
                or not .05 <= self.release_ramp_s <= min(.25, self.duration_s/2.)
                or not 4. <= self.recovery_s <= 15.
                or not 0. < self.goal_lateral_m < self.max_lateral_m <= .3
                or not 0. < self.goal_heading_rad < self.max_heading_rad <= math.radians(6.)
                or not 0. < self.entry_lateral_m <= .1
                or not 0. < self.entry_heading_rad <= math.radians(2.)
                or not 1. <= self.min_speed_mps < self.max_speed_mps <= 5/3.6+.1):
            raise ValueError('PULSE_CONFIG_BOUNDS')


@dataclass(frozen=True)
class SteeringPulseState:
    stage: str = 'waiting'
    approach_seen: bool = False
    start_ns: int | None = None
    start_wall_ns: int | None = None
    release_ns: int | None = None
    release_value_rad: float = 0.
    zero_ns: int | None = None
    reason: str | None = None
    last_sim_ns: int | None = None
    last_wall_ns: int | None = None


@dataclass(frozen=True)
class SteeringPulseDecision:
    state: SteeringPulseState
    perturbation_rad: float
    phase: str


def nominal_recovery_errors(guide: np.ndarray, *, s_m: float, offset_m: float,
                            yaw_rad: float) -> tuple[float, float]:
    """Guide [N,3] = base progress m, observed lateral m, observed body yaw rad."""
    guide = np.asarray(guide, dtype=float)
    if (guide.ndim != 2 or guide.shape[1] != 3 or len(guide) < 2 or not np.isfinite(guide).all()
            or np.any(np.diff(guide[:, 0]) <= 0) or not np.isfinite([s_m, offset_m, yaw_rad]).all()
            or not guide[0, 0] <= s_m <= guide[-1, 0]):
        raise ValueError('PULSE_NOMINAL_GUIDE_SUPPORT')
    lateral = offset_m - float(np.interp(s_m, guide[:, 0], guide[:, 1]))
    heading = yaw_rad - float(np.interp(s_m, guide[:, 0], np.unwrap(guide[:, 2])))
    return lateral, math.atan2(math.sin(heading), math.cos(heading))


def propose_steering_pulse(config: SteeringPulseConfig, state: SteeringPulseState, *,
                           sim_ns: int, wall_ns: int, s_m: float, speed_mps: float,
                           lateral_m: float, heading_rad: float) -> SteeringPulseDecision:
    """Propose one command; retries must reuse the last successfully published state.

One bump, then unperturbed recovery. Reaching any bound releases the bump;
failure to reach the requested state never extends or amplifies it.
"""
    if (type(sim_ns) is not int or type(wall_ns) is not int or min(sim_ns, wall_ns) < 0
            or (state.last_sim_ns is not None and sim_ns < state.last_sim_ns)
            or (state.last_wall_ns is not None and wall_ns < state.last_wall_ns)
            or not np.isfinite([s_m, speed_mps, lateral_m, heading_rad]).all()):
        raise ValueError('PULSE_CLOCK_OR_STATE')
    if state.stage not in ('waiting', 'active', 'releasing', 'recovery', 'complete', 'skipped'):
        raise ValueError('PULSE_STAGE')
    next_state = replace(state, last_sim_ns=sim_ns, last_wall_ns=wall_ns)
    if next_state.stage == 'waiting':
        # A closed-course spawn can project just before the lap seam. Require
        # a genuine approach before treating a high progress value as a miss.
        if s_m < config.start_s_m:
            next_state = replace(next_state, approach_seen=True)
        if not next_state.approach_seen:
            return SteeringPulseDecision(next_state, 0., 'baseline')
        if s_m > config.start_s_m + config.start_window_m:
            next_state = replace(next_state, stage='skipped', reason='START_WINDOW_MISSED')
        elif s_m >= config.start_s_m:
            ready = (config.min_speed_mps <= speed_mps <= config.max_speed_mps
                and abs(lateral_m) <= config.entry_lateral_m and abs(heading_rad) <= config.entry_heading_rad)
            if ready:
                next_state = replace(next_state, stage='active', start_ns=sim_ns, start_wall_ns=wall_ns)
    if next_state.stage in ('waiting', 'complete', 'skipped'):
        return SteeringPulseDecision(next_state, 0., 'baseline')
    if next_state.start_ns is None or next_state.start_wall_ns is None:
        raise ValueError('PULSE_START_MISSING')
    elapsed = (sim_ns-next_state.start_ns)/1e9
    if next_state.stage in ('active', 'releasing'):
        if elapsed >= config.duration_s or wall_ns-next_state.start_wall_ns >= 2*config.duration_s*1e9:
            next_state = replace(next_state, stage='recovery', zero_ns=sim_ns,
                                 reason=next_state.reason or 'DURATION_LIMIT')
        elif next_state.stage == 'active':
            if config.plateau_s == 0.:
                value = config.amplitude_rad*math.sin(math.pi*elapsed/config.duration_s)**2
            else:
                # The total deadline includes both ramps and the plateau.
                # A nonzero plateau leaves at least .25s for each cosine ramp.
                ramp_s = (config.duration_s-config.plateau_s)/2.
                ramp_fraction = min(1., elapsed/ramp_s, (config.duration_s-elapsed)/ramp_s)
                value = config.amplitude_rad*.5*(1.-math.cos(math.pi*ramp_fraction))
            sign = math.copysign(1., config.amplitude_rad)
            reason = ('STATE_GOAL' if sign*lateral_m >= config.goal_lateral_m and sign*heading_rad >= config.goal_heading_rad
                else 'LATERAL_LIMIT' if abs(lateral_m) >= config.max_lateral_m
                else 'HEADING_LIMIT' if abs(heading_rad) >= config.max_heading_rad
                else 'PROGRESS_LIMIT' if s_m >= config.start_s_m+config.start_window_m+3.
                else 'SPEED_LIMIT' if not config.min_speed_mps <= speed_mps <= config.max_speed_mps else None)
            if reason:
                next_state = replace(next_state, stage='releasing', release_ns=sim_ns,
                                     release_value_rad=value, reason=reason)
            else:
                return SteeringPulseDecision(next_state, value, 'hold')
        if next_state.stage == 'releasing':
            if next_state.release_ns is None:
                raise ValueError('PULSE_RELEASE_MISSING')
            fraction = (sim_ns-next_state.release_ns)/1e9/config.release_ramp_s
            if fraction < 1.:
                value = next_state.release_value_rad*.5*(1.+math.cos(math.pi*fraction))
                return SteeringPulseDecision(next_state, value, 'hold')
            next_state = replace(next_state, stage='recovery', zero_ns=sim_ns)
    if next_state.zero_ns is None:
        raise ValueError('PULSE_ZERO_BOUNDARY_MISSING')
    if sim_ns-next_state.zero_ns >= config.recovery_s*1e9:
        next_state = replace(next_state, stage='complete')
    return SteeringPulseDecision(next_state, 0., 'recovery' if next_state.stage == 'recovery' else 'baseline')
