"""Bounded, seeded teacher perturbations, committed only after ROS publication.

This schedules at most three existing single pulses. SI units are m, s and rad;
course/nominal errors and seed/event metadata are teacher/debug-only information.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
import math
import random
from typing import Any, Mapping, Sequence

from .time_steering_pulse_v1 import SteeringPulseConfig, SteeringPulseState, propose_steering_pulse

SCHEMA = 'measured_random_steering_pulse_v1'


@dataclass(frozen=True)
class RandomPulseConfig:
    seed: int
    max_events: int = 3
    start_min_m: float = 65.
    start_max_m: float = 111.
    jitter_min_s: float = .5
    jitter_max_s: float = 1.5
    stable_hold_s: float = 1.
    future_tail_s: float = 3.
    clearance_margin_m: float = .5

    def __post_init__(self) -> None:
        if (type(self.seed) is not int or not 0 <= self.seed <= 2**32-1
                or type(self.max_events) is not int or not 2 <= self.max_events <= 3):
            raise ValueError('RANDOM_SEED_OR_EVENT_LIMIT')
        values = [v for k, v in self.__dict__.items() if k not in ('seed', 'max_events')]
        if (not all(type(v) in (int, float) and math.isfinite(v) for v in values)
                or not 60. <= self.start_min_m < self.start_max_m <= 111.
                or not .5 <= self.jitter_min_s < self.jitter_max_s <= 3.
                or self.stable_hold_s != 1. or self.future_tail_s != 3.
                or self.clearance_margin_m != .5):
            raise ValueError('RANDOM_CONFIG_BOUNDS')


def random_schedule(config: RandomPulseConfig) -> tuple[tuple[int, float], ...]:
    """Return (left-positive sign, delay s); both signs occur in the finite plan."""
    rng = random.Random(config.seed)
    signs = [-1, 1] + ([rng.choice((-1, 1))] if config.max_events == 3 else [])
    rng.shuffle(signs)
    return tuple((sign, rng.uniform(config.jitter_min_s, config.jitter_max_s)) for sign in signs)


@dataclass(frozen=True)
class RandomPulseState:
    stage: str = 'waiting'
    event_id: int = 0
    completed_events: int = 0
    pulse: SteeringPulseState = SteeringPulseState()
    start_s_m: float | None = None
    candidate_ns: int | None = None
    stable_since_ns: int | None = None
    recovery_since_ns: int | None = None
    confirmed_ns: int | None = None
    last_sim_ns: int | None = None
    last_wall_ns: int | None = None
    approach_seen: bool = False
    reason: str | None = None


@dataclass(frozen=True)
class RandomPulseDecision:
    state: RandomPulseState
    perturbation_rad: float
    phase: str


def propose_random_pulse(config: RandomPulseConfig, template: SteeringPulseConfig,
                         state: RandomPulseState, *, sim_ns: int, wall_ns: int,
                         s_m: float, speed_mps: float, lateral_m: float,
                         heading_rad: float, entry_clear: bool) -> RandomPulseDecision:
    """Pure proposal; callers discard it if any final guard/publish check fails."""
    if (any(type(t) is not int or t < 0 for t in (sim_ns, wall_ns))
            or state.last_sim_ns is not None and sim_ns < state.last_sim_ns
            or state.last_wall_ns is not None and wall_ns < state.last_wall_ns
            or not all(math.isfinite(v) for v in (s_m, speed_mps, lateral_m, heading_rad))
            or type(entry_clear) is not bool):
        raise ValueError('RANDOM_CLOCK_OR_STATE')
    if (state.stage not in ('waiting', 'pulse', 'cooldown', 'complete', 'aborted')
            or not 0 <= state.completed_events <= state.event_id <= config.max_events):
        raise ValueError('RANDOM_STAGE_OR_EVENT_ORDER')
    if (template.duration_s != 2. or template.plateau_s != 1.5 or abs(template.amplitude_rad) != .1
            or template.recovery_s != 10. or template.max_lateral_m != .25
            or template.max_heading_rad != math.radians(4.)):
        raise ValueError('RANDOM_PRESERVES_PROVEN_PULSE')
    gap = state.last_sim_ns is None or sim_ns-state.last_sim_ns > 150_000_000
    st = replace(state, last_sim_ns=sim_ns, last_wall_ns=wall_ns,
                 stable_since_ns=None if gap else state.stable_since_ns,
                 recovery_since_ns=None if gap else state.recovery_since_ns,
                 confirmed_ns=None if gap else state.confirmed_ns)
    schedule = random_schedule(config)
    if st.stage in ('complete', 'aborted'):
        return RandomPulseDecision(st, 0., 'baseline')
    if st.stage in ('waiting', 'cooldown'):
        if s_m < config.start_min_m:
            st = replace(st, approach_seen=True)
        if not st.approach_seen:
            return RandomPulseDecision(st, 0., 'baseline')
        if s_m > config.start_max_m:
            return RandomPulseDecision(replace(st, stage='complete', reason='START_REGION_FINISHED'), 0., 'baseline')
        ready = (entry_clear and template.min_speed_mps <= speed_mps <= template.max_speed_mps
                 and abs(lateral_m) <= template.entry_lateral_m and abs(heading_rad) <= template.entry_heading_rad)
        st = replace(st, stable_since_ns=(st.stable_since_ns if st.stable_since_ns is not None else sim_ns) if ready else None)
        if s_m >= config.start_min_m and st.candidate_ns is None:
            st = replace(st, candidate_ns=sim_ns+round(schedule[st.event_id][1]*1e9))
        if (s_m < config.start_min_m or st.candidate_ns is None or sim_ns < st.candidate_ns
                or st.stable_since_ns is None or sim_ns-st.stable_since_ns < round(config.stable_hold_s*1e9)):
            return RandomPulseDecision(st, 0., 'baseline')
        st = replace(st, stage='pulse', event_id=st.event_id+1, start_s_m=s_m,
                     pulse=SteeringPulseState(approach_seen=True), stable_since_ns=None,
                     recovery_since_ns=None, confirmed_ns=None, reason=None)
    pulse_config = replace(template, start_s_m=st.start_s_m,
                           amplitude_rad=abs(template.amplitude_rad)*schedule[st.event_id-1][0])
    proposal = propose_steering_pulse(pulse_config, st.pulse, sim_ns=sim_ns, wall_ns=wall_ns,
        s_m=s_m, speed_mps=speed_mps, lateral_m=lateral_m, heading_rad=heading_rad)
    st = replace(st, pulse=proposal.state)
    if proposal.state.stage in ('recovery', 'complete'):
        settled = (abs(lateral_m) <= .05 and abs(heading_rad) <= math.radians(2.)
                   and template.min_speed_mps <= speed_mps <= template.max_speed_mps)
        since = (st.recovery_since_ns if st.recovery_since_ns is not None else sim_ns) if settled else None
        confirmed = sim_ns if since is not None and sim_ns-since >= round(config.stable_hold_s*1e9) else None
        st = replace(st, recovery_since_ns=since, confirmed_ns=confirmed)
        if proposal.state.stage == 'complete':
            if confirmed is None:
                st = replace(st, stage='aborted', reason='RECOVERY_NOT_CONFIRMED')
            elif st.event_id == config.max_events:
                st = replace(st, stage='complete', completed_events=st.event_id, reason='EVENT_BUDGET_COMPLETE')
            else:
                st = replace(st, stage='cooldown', completed_events=st.event_id, stable_since_ns=None,
                    candidate_ns=sim_ns+round((config.future_tail_s+schedule[st.event_id][1])*1e9))
    return RandomPulseDecision(st, proposal.perturbation_rad, proposal.phase)


def random_pulse_events(rows: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    """Audit emitted event boundaries and recovery, retaining unsuccessful events.

    Input is control telemetry; output timestamps are ROS publication sim ns,
    not simulator actuator-application times. Sensor/future audits remain required.
    """
    if not rows or any(r.get('annotation_schema') != SCHEMA for r in rows):
        raise ValueError('RANDOM_ANNOTATION_SCHEMA')
    groups: dict[int, list[Mapping[str, Any]]] = {}
    seed = None; limit = None; previous = 0; fixed_config = None
    for row in rows:
        meta = row.get('random_pulse')
        if not isinstance(meta, dict):
            raise ValueError('RANDOM_METADATA_MISSING')
        config = RandomPulseConfig(**meta['config'])
        if seed is None:
            seed, limit = config.seed, config.max_events
            fixed_config = config
        if config != fixed_config:
            raise ValueError('RANDOM_CONFIG_CHANGED')
        st = meta['state']; event_id = st['event_id']
        if type(event_id) is not int or not previous <= event_id <= min(previous+1, limit):
            raise ValueError('RANDOM_EVENT_ORDER')
        previous = event_id
        if event_id and row.get('publication') is not None:
            groups.setdefault(event_id, []).append(row)
    events = []
    for event_id, group in groups.items():
        active = [r for r in group if r['phase'] == 'hold' and r['pulse']['applied']]
        recovery = [r for r in group if r['phase'] == 'recovery' and r['pulse']['applied']]
        completed = [r for r in group if r['random_pulse']['state']['completed_events'] >= event_id
                     and r['pulse']['applied'] and r['phase'] == 'baseline']
        if not active:
            raise ValueError('RANDOM_EVENT_WITHOUT_PUBLISHED_START')
        start = active[0]['publication']['sim_ns']
        zero = recovery[0]['publication']['sim_ns'] if recovery else None
        end = completed[0]['publication']['sim_ns'] if completed else None
        since = None; confirmed = None; prior = None
        for r in recovery:
            p = r['pulse']; t = r['publication']['sim_ns']
            values = [p['lateral_error_m'], p['heading_error_rad'], r['speed_mps']]
            if not all(type(v) in (float, int) and math.isfinite(v) for v in values):
                raise ValueError('RANDOM_RECOVERY_STATE_MISSING')
            valid = (p['requested_rad'] == p['effective_rad'] == 0. and abs(values[0]) <= .05
                     and abs(values[1]) <= math.radians(2.) and 1.15 <= values[2] <= 1.4)
            if prior is None or not 0 <= t-prior <= 150_000_000:
                since = None; confirmed = None
            since = (t if since is None else since) if valid else None
            confirmed = t if since is not None and t-since >= 1_000_000_000 else None
            prior = t
        success = bool(end is not None and zero is not None and confirmed is not None
                       and prior is not None and 0 <= end-prior <= 150_000_000
                       and 9_800_000_000 <= end-zero <= 10_200_000_000)
        event = dict(event_id=event_id, seed=seed, start_publication_ns=start,
                     zero_publication_ns=zero, end_publication_ns=end,
                     recovery_confirmed=success, confirmation_publication_ns=confirmed,
                     start_s_m=active[0]['random_pulse']['state']['start_s_m'],
                     sign=random_schedule(config)[event_id-1][0])
        if events:
            last = events[-1]
            if (not last['recovery_confirmed'] or start < last['end_publication_ns']+3_000_000_000):
                raise ValueError('RANDOM_REINJECTION_BEFORE_RECOVERY_AND_FUTURE')
        events.append(event)
    return events


def event_at(stamp_ns: int, events: Sequence[Mapping[str, Any]]) -> Mapping[str, Any] | None:
    """Find the successful recovery event containing a camera capture stamp."""
    matches = [e for e in events if e['recovery_confirmed']
               and e['zero_publication_ns'] <= stamp_ns < e['end_publication_ns']]
    if len(matches) > 1:
        raise ValueError('RANDOM_EVENT_OVERLAP')
    return matches[0] if matches else None
