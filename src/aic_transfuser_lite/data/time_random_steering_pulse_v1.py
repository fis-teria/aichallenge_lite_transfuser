"""Bounded, seeded teacher perturbations, committed only after ROS publication.

This schedules at most three existing single pulses. SI units are m, s and rad;
course/nominal errors and seed/event metadata are teacher/debug-only information.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
import math
import random
import re
from typing import Any, Mapping, Sequence

from .time_steering_pulse_v1 import SteeringPulseConfig, SteeringPulseState, propose_steering_pulse

SCHEMA = 'measured_random_steering_pulse_v1'


@dataclass(frozen=True)
class PulseSite:
    """One named global-reference start [m] and ROS-input steering sign."""
    site_id: str
    start_s_m: float
    sign: int

    def __post_init__(self) -> None:
        if (not isinstance(self.site_id, str) or not re.fullmatch(r'(S00|R(?:0[1-9]|10))', self.site_id)
                or type(self.start_s_m) not in (int, float) or not math.isfinite(self.start_s_m)
                or not 10. <= self.start_s_m <= 1000.
                or type(self.sign) is not int or self.sign not in (-1, 1)):
            raise ValueError('PULSE_SITE_ID_PROGRESS_OR_SIGN')


@dataclass(frozen=True)
class RandomPulseConfig:
    seed: int
    max_events: int = 3
    start_min_m: float = 66.
    start_max_m: float = 111.
    jitter_min_s: float = .5
    jitter_max_s: float = 1.5
    stable_hold_s: float = 1.
    future_tail_s: float = 3.
    clearance_margin_m: float = .5
    sites: tuple[PulseSite, ...] = ()
    control_policy: str = 'pp_additive_v1'

    def __post_init__(self) -> None:
        if not isinstance(self.sites, (list, tuple)):
            raise ValueError('PULSE_SITES_SEQUENCE')
        sites = tuple(PulseSite(**s) if isinstance(s, dict) else s for s in self.sites)
        if any(not isinstance(s, PulseSite) for s in sites):
            raise ValueError('PULSE_SITES_SEQUENCE')
        object.__setattr__(self, 'sites', sites)
        if (self.control_policy not in ('pp_additive_v1', 'nominal_guide_then_pp_v1')
                or self.control_policy == 'nominal_guide_then_pp_v1' and not sites):
            raise ValueError('RANDOM_CONTROL_POLICY')
        if (type(self.seed) is not int or not 0 <= self.seed <= 2**32-1
                or type(self.max_events) is not int or not (1 if sites else 2) <= self.max_events <= 3):
            raise ValueError('RANDOM_SEED_OR_EVENT_LIMIT')
        values = [v for k, v in self.__dict__.items() if k not in ('seed', 'max_events', 'sites', 'control_policy')]
        spatial_bounds = (len(sites) == self.max_events and len({s.site_id for s in sites}) == len(sites)
                          and all(b.start_s_m-a.start_s_m >= 28. for a, b in zip(sites, sites[1:]))
                          and self.start_min_m == sites[0].start_s_m and self.start_max_m == sites[-1].start_s_m) if sites else False
        if (not all(type(v) in (int, float) and math.isfinite(v) for v in values)
                or not (spatial_bounds if sites else 60. <= self.start_min_m < self.start_max_m <= 111.)
                or not .5 <= self.jitter_min_s < self.jitter_max_s <= 3.
                or self.stable_hold_s != 1. or self.future_tail_s != 3.
                or self.clearance_margin_m != .5):
            raise ValueError('RANDOM_CONFIG_BOUNDS')


def random_schedule(config: RandomPulseConfig) -> tuple[tuple[int, float], ...]:
    """Return (left-positive sign, delay s); both signs occur in the finite plan."""
    if config.sites:
        # These positions were already drawn with a recorded selection seed.
        # No second time jitter is added to the bounded spatial start windows.
        return tuple((site.sign, 0.) for site in config.sites)
    rng = random.Random(config.seed)
    signs = [-1, 1] + ([rng.choice((-1, 1))] if config.max_events == 3 else [])
    rng.shuffle(signs)
    return tuple((sign, rng.uniform(config.jitter_min_s, config.jitter_max_s)) for sign in signs)


def validate_random_guide(config: RandomPulseConfig, template: SteeringPulseConfig, guide: Any) -> None:
    """Require measured [N,3] (progress m, lateral m, yaw rad) and full margins."""
    import numpy as np
    values = np.asarray(guide, dtype=float)
    if (values.ndim != 2 or values.shape[1] != 3 or len(values) < 2 or not np.isfinite(values).all()
            or not np.all(np.diff(values[:,0]) > 0)
            or values[0,0] > config.start_min_m-5.
            or values[-1,0] < config.start_max_m+template.start_window_m+3.
                +1.7*(template.recovery_s+(config.future_tail_s if config.sites else 0.))):
        raise ValueError('RANDOM_GUIDE_RECOVERY_COVERAGE')


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
    site_cursor: int = 0
    active_site_index: int | None = None
    skipped_sites: tuple[int, ...] = ()
    last_progress_m: float | None = None


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
    profile = (template.duration_s, template.plateau_s, template.goal_min_elapsed_s)
    expected = (1.5, 1., 1.25) if config.control_policy == 'nominal_guide_then_pp_v1' else (2., 1.5, 0.)
    if (profile != expected or abs(template.amplitude_rad) != .1
            or template.recovery_s != 10. or template.max_lateral_m != .25
            or template.max_heading_rad != math.radians(4.)):
        raise ValueError('RANDOM_PRESERVES_PROVEN_PULSE')
    if config.sites:
        return _propose_site_pulse(config, template, state, sim_ns=sim_ns, wall_ns=wall_ns,
            s_m=s_m, speed_mps=speed_mps, lateral_m=lateral_m, heading_rad=heading_rad,
            entry_clear=entry_clear)
    gap = state.last_sim_ns is None or sim_ns-state.last_sim_ns > 150_000_000
    st = replace(state, last_sim_ns=sim_ns, last_wall_ns=wall_ns,
                 stable_since_ns=None if gap else state.stable_since_ns,
                 recovery_since_ns=None if gap else state.recovery_since_ns)
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
        # Keep the evidence of a completed continuous hold. A later telemetry
        # gap resets the CURRENT hold; it cannot erase an already observed
        # recovery. A separate fresh entry hold still gates every reinjection.
        confirmed = st.confirmed_ns
        if confirmed is None and since is not None and sim_ns-since >= round(config.stable_hold_s*1e9):
            confirmed = sim_ns
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


def _propose_site_pulse(config: RandomPulseConfig, template: SteeringPulseConfig,
                        state: RandomPulseState, *, sim_ns: int, wall_ns: int, s_m: float,
                        speed_mps: float, lateral_m: float, heading_rad: float,
                        entry_clear: bool) -> RandomPulseDecision:
    """Finite named windows; use the proven pulse in an explicit local m axis.

    Global progress is kept in metadata. Internally the pulse starts at local
    10 m: local_s = global_s - actual_global_start + 10 m. Its progress, state,
    duration and steering limits therefore stay unchanged above global 300 m.
    validate_random_guide must check the full global support before use.
    """
    if (type(state.site_cursor) is not int or not 0 <= state.site_cursor <= len(config.sites)
            or any(type(i) is not int or not 0 <= i < state.site_cursor for i in state.skipped_sites)
            or len(set(state.skipped_sites)) != len(state.skipped_sites)):
        raise ValueError('PULSE_SITE_STATE_ORDER')
    gap = state.last_sim_ns is None or sim_ns-state.last_sim_ns > 150_000_000
    st = replace(state, last_sim_ns=sim_ns, last_wall_ns=wall_ns, last_progress_m=s_m,
        stable_since_ns=None if gap else state.stable_since_ns,
        recovery_since_ns=None if gap else state.recovery_since_ns)
    if st.stage in ('complete', 'aborted'):
        return RandomPulseDecision(st, 0., 'baseline')
    if state.approach_seen and state.last_progress_m is not None and s_m < state.last_progress_m-5.:
        # A completed plan ignores the lap seam; an unfinished one cannot fire
        # its remaining sites in the next lap or after a localization jump.
        return RandomPulseDecision(replace(st, stage='aborted', reason='SITE_PROGRESS_REGRESSION'), 0., 'baseline')
    if st.stage in ('waiting', 'cooldown'):
        if s_m < config.start_min_m:
            st = replace(st, approach_seen=True)
        if not st.approach_seen:
            return RandomPulseDecision(st, 0., 'baseline')
        while st.site_cursor < len(config.sites) and s_m > config.sites[st.site_cursor].start_s_m+template.start_window_m:
            st = replace(st, skipped_sites=(*st.skipped_sites, st.site_cursor), site_cursor=st.site_cursor+1)
        if st.site_cursor == len(config.sites):
            return RandomPulseDecision(replace(st, stage='complete', reason='SITE_WINDOWS_FINISHED'), 0., 'baseline')
        ready = (entry_clear and template.min_speed_mps <= speed_mps <= template.max_speed_mps
                 and abs(lateral_m) <= template.entry_lateral_m and abs(heading_rad) <= template.entry_heading_rad)
        st = replace(st, stable_since_ns=(st.stable_since_ns if st.stable_since_ns is not None else sim_ns) if ready else None)
        if (s_m < config.sites[st.site_cursor].start_s_m or st.stable_since_ns is None
                or sim_ns-st.stable_since_ns < 1_000_000_000
                or st.candidate_ns is not None and sim_ns < st.candidate_ns):
            return RandomPulseDecision(st, 0., 'baseline')
        st = replace(st, stage='pulse', event_id=st.event_id+1, active_site_index=st.site_cursor,
            start_s_m=s_m, pulse=SteeringPulseState(approach_seen=True), stable_since_ns=None,
            recovery_since_ns=None, confirmed_ns=None, reason=None)
    if st.active_site_index is None or not 0 <= st.active_site_index < len(config.sites) or st.start_s_m is None:
        raise ValueError('PULSE_ACTIVE_SITE_MISSING')
    pulse_config = replace(template, start_s_m=10.,
                           amplitude_rad=abs(template.amplitude_rad)*config.sites[st.active_site_index].sign)
    proposal = propose_steering_pulse(pulse_config, st.pulse, sim_ns=sim_ns, wall_ns=wall_ns,
        s_m=s_m-st.start_s_m+10., speed_mps=speed_mps, lateral_m=lateral_m, heading_rad=heading_rad)
    st = replace(st, pulse=proposal.state)
    if proposal.state.stage in ('recovery', 'complete'):
        settled = (abs(lateral_m) <= .05 and abs(heading_rad) <= math.radians(2.)
                   and template.min_speed_mps <= speed_mps <= template.max_speed_mps)
        since = (st.recovery_since_ns if st.recovery_since_ns is not None else sim_ns) if settled else None
        confirmed = st.confirmed_ns
        if confirmed is None and since is not None and sim_ns-since >= 1_000_000_000:
            confirmed = sim_ns
        st = replace(st, recovery_since_ns=since, confirmed_ns=confirmed)
        if proposal.state.stage == 'complete':
            if confirmed is None:
                st = replace(st, stage='aborted', reason='RECOVERY_NOT_CONFIRMED')
            else:
                cursor = st.active_site_index+1
                st = replace(st, site_cursor=cursor, completed_events=st.event_id,
                    stage='complete' if cursor == len(config.sites) else 'cooldown', stable_since_ns=None,
                    candidate_ns=sim_ns+round(config.future_tail_s*1e9), reason='SITE_EVENT_COMPLETE')
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
        if config.control_policy == 'nominal_guide_then_pp_v1':
            from .time_nominal_steering_guide_v1 import validate_guide_control_row
            validate_guide_control_row(row)
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
                since = None
            since = (t if since is None else since) if valid else None
            if confirmed is None and since is not None and t-since >= 1_000_000_000:
                confirmed = t
            prior = t
        success = bool(end is not None and zero is not None and confirmed is not None
                       and prior is not None and 0 <= end-prior <= 150_000_000
                       and 9_800_000_000 <= end-zero <= 10_200_000_000)
        site_index = active[0]['random_pulse']['state'].get('active_site_index') if config.sites else event_id-1
        if config.sites and (type(site_index) is not int or not 0 <= site_index < len(config.sites)
                or any(r['random_pulse']['state'].get('active_site_index') != site_index for r in group)):
            raise ValueError('RANDOM_EVENT_SITE_CHANGED')
        actual_start = active[0]['random_pulse']['state']['start_s_m']
        if config.sites and not config.sites[site_index].start_s_m <= actual_start <= config.sites[site_index].start_s_m+2.:
            raise ValueError('RANDOM_EVENT_OUTSIDE_SITE_WINDOW')
        event = dict(event_id=event_id, seed=seed, start_publication_ns=start,
                     zero_publication_ns=zero, end_publication_ns=end,
                     recovery_confirmed=success, confirmation_publication_ns=confirmed,
                     start_s_m=active[0]['random_pulse']['state']['start_s_m'],
                     sign=random_schedule(config)[site_index][0])
        if config.sites:
            event.update(site_id=config.sites[site_index].site_id,
                         planned_start_s_m=config.sites[site_index].start_s_m)
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
