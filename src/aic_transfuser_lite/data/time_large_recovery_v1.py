"""Measured lateral-state collection, independent of the small steering pulse.

All distances are metres, angles body radians, velocities m/s, clocks ns.
This module selects between two continuously running teacher PP commands; it
never bypasses the final actuator, sensor, clock or stopping-sweep checks.
Proposals have no effect until commit_large_recovery confirms publication.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, replace
import math
import random
import re
from typing import Any, Mapping, Sequence

SCHEMA = 'measured_large_recovery_v1'
MAX_EVENTS = 7
SPEED_POLICIES = ('bounded_5kmh_v1', 'record_actual_v1')


def collection_speed_eligible(speed_mps: float, policy: str) -> bool:
    """Acquisition gate, independent of measured-speed stopping-sweep safety.

    New collections can retain overspeed observations instead of discarding an
    event. The legacy default keeps historical recording audits reproducible.
    Both policies require forward motion; neither changes the 5 km/h target.
    """
    if policy not in SPEED_POLICIES:
        raise ValueError('LARGE_SPEED_POLICY')
    return (type(speed_mps) in (int, float) and math.isfinite(speed_mps)
            and speed_mps >= 1.15 and (policy == 'record_actual_v1' or speed_mps <= 1.4))


@dataclass(frozen=True)
class LargeRecoverySite:
    site_id: str
    release_s_m: float
    target_offset_m: float
    return_length_m: float = 6.
    settle_distance_m: float = 2.
    preparation_origin: str = 'measured_normal'
    target_heading_rad: float = 0.
    heading_tolerance_rad: float = math.radians(2.)
    corner_id: str | None = None
    approach_distance_m: float = 8.

    def __post_init__(self) -> None:
        if (not isinstance(self.site_id, str) or not re.fullmatch(r'[A-Z][A-Z0-9_]{0,23}', self.site_id)
                or any(type(v) not in (int, float) or not math.isfinite(v)
                       for v in (self.release_s_m, self.target_offset_m, self.return_length_m, self.settle_distance_m,
                                 self.target_heading_rad, self.heading_tolerance_rad, self.approach_distance_m))
                or not 20. <= self.release_s_m <= 325.
                or abs(self.target_offset_m) not in (.2, .4, .6)
                or self.return_length_m not in (4., 6., 10.) or self.settle_distance_m not in (2., 4.)
                or self.preparation_origin not in ('measured_normal', 'nominal_path')
                or abs(self.target_heading_rad) > math.radians(7.)
                or not math.radians(.5) <= self.heading_tolerance_rad <= math.radians(2.)
                or self.approach_distance_m not in (4., 6., 8.)
                or self.corner_id is not None and (not isinstance(self.corner_id, str)
                    or not re.fullmatch(r'C[0-9]{2}[A-Z]?', self.corner_id))):
            raise ValueError('LARGE_SITE_CONTRACT')

    @property
    def start_s_m(self) -> float:
        return self.release_s_m - self.approach_distance_m - self.settle_distance_m

    def at_goal(self, lateral_m: float, heading_rad: float) -> bool:
        """Measured error in the unchanged nominal-lap frame, metres/radians."""
        return (abs(lateral_m-self.target_offset_m) <= .05
                and abs(heading_rad-self.target_heading_rad) <= self.heading_tolerance_rad)


@dataclass(frozen=True)
class LargeRecoveryConfig:
    sites: tuple[LargeRecoverySite, ...]
    seed: int = 0
    event_cap: int = 1
    speed_policy: str = 'bounded_5kmh_v1'
    entry_heading_tolerance_rad: float = math.radians(1.)
    recovery_duration_s: float = 10.

    def __post_init__(self) -> None:
        if self.speed_policy not in SPEED_POLICIES:
            raise ValueError('LARGE_SPEED_POLICY')
        if (type(self.recovery_duration_s) not in (int, float)
                or not math.isfinite(self.recovery_duration_s)
                or not 10. <= self.recovery_duration_s <= 15.):
            raise ValueError('LARGE_RECOVERY_DURATION')
        if (type(self.entry_heading_tolerance_rad) not in (int, float)
                or not math.isfinite(self.entry_heading_tolerance_rad)
                or not math.radians(.5) <= self.entry_heading_tolerance_rad <= math.radians(2.)):
            raise ValueError('LARGE_ENTRY_HEADING_TOLERANCE')
        if not isinstance(self.sites, (tuple, list)):
            raise ValueError('LARGE_SITES_SEQUENCE')
        sites = tuple(LargeRecoverySite(**s) if isinstance(s, dict) else s for s in self.sites)
        if (any(not isinstance(s, LargeRecoverySite) for s in sites)
                or type(self.seed) is not int or not 0 <= self.seed < 2**32
                or type(self.event_cap) is not int or not 1 <= self.event_cap <= MAX_EVENTS
                or not 1 <= len(sites) <= self.event_cap
                or len({s.site_id for s in sites}) != len(sites)
                or any(b.start_s_m-a.start_s_m < 40. for a, b in zip(sites, sites[1:]))):
            raise ValueError('LARGE_SITE_BUDGET_OR_SPACING')
        object.__setattr__(self, 'sites', sites)

    @property
    def recovery_duration_ns(self) -> int:
        """Finite observed recovery window, independent of sensor freshness."""
        return round(self.recovery_duration_s * 1_000_000_000)


def select_large_sites(candidates: Sequence[LargeRecoverySite], *, seed: int,
                       event_cap: int, required_site_ids: Sequence[str] = ()) -> LargeRecoveryConfig:
    """Seeded, bounded site selection; chronological driving order is preserved.

    Candidates must subsequently pass the actual map/reference checks. This
    function neither certifies geometry nor automatically increases a run cap.
    """
    if (type(seed) is not int or not 0 <= seed < 2**32
            or type(event_cap) is not int or not 1 <= event_cap <= MAX_EVENTS
            or any(not isinstance(s, LargeRecoverySite) for s in candidates)
            or len(set(required_site_ids)) != len(required_site_ids)
            or len(required_site_ids) > event_cap):
        raise ValueError('LARGE_SELECTION_CONTRACT')
    pool = list(candidates)
    random.Random(seed).shuffle(pool)
    selected: list[LargeRecoverySite] = []
    for required in required_site_ids:
        match = next((s for s in pool if s.site_id == required), None)
        if match is None:
            raise ValueError('LARGE_REQUIRED_SITE_MISSING')
        selected.append(match)
    # Required windows must themselves be independent.
    if selected:
        LargeRecoveryConfig(tuple(sorted(selected, key=lambda s: s.start_s_m)), seed, event_cap)
    for site in pool:
        if len(selected) == event_cap:
            break
        if all(site.site_id != s.site_id and abs(site.start_s_m-s.start_s_m) >= 40. for s in selected):
            selected.append(site)
    if len(selected) != event_cap:
        raise ValueError('LARGE_INSUFFICIENT_SEPARATED_SITES')
    return LargeRecoveryConfig(tuple(sorted(selected, key=lambda s: s.start_s_m)), seed, event_cap)


@dataclass(frozen=True)
class LargeRecoveryState:
    stage: str = 'waiting'
    site_cursor: int = 0
    event_id: int = 0
    completed_events: int = 0
    skipped_sites: tuple[int, ...] = ()
    approach_seen: bool = False
    last_progress_m: float | None = None
    last_sim_ns: int | None = None
    last_wall_ns: int | None = None
    last_sequence: int = 0
    start_ns: int | None = None
    start_wall_ns: int | None = None
    request_ns: int | None = None
    release_ns: int | None = None
    confirmed_ns: int | None = None
    stable_since_ns: int | None = None
    target_since_ns: int | None = None
    next_allowed_ns: int | None = None
    goal_reached: bool = False
    reason: str | None = None


@dataclass(frozen=True)
class LargeRecoveryDecision:
    state: LargeRecoveryState
    command_source: str  # 'nominal' or 'preparation'; longitudinal stays nominal.
    phase: str
    transition: str | None = None


def propose_large_recovery(config: LargeRecoveryConfig, state: LargeRecoveryState, *,
                           sim_ns: int, wall_ns: int, s_m: float, speed_mps: float,
                           lateral_m: float, heading_rad: float,
                           nominal_stamp_ns: int, entry_clear: bool) -> LargeRecoveryDecision:
    """One immutable selection; the caller validates both PP sources and guards.

    Preparation: 4/6/8 m approach + 2/4 m settling, target +/-5 cm and configured heading tolerance
    continuously for 0.25 s. Request inside [release, release+1] m only.
    Recovery: nominal PP, settle within 10 s, then reserve >=3 s future tail.
    A missed/failed event never increases amplitude, event count or deadlines.
    """
    if (any(type(t) is not int or t < 0 for t in (sim_ns, wall_ns, nominal_stamp_ns))
            or state.last_sim_ns is not None and sim_ns < state.last_sim_ns
            or state.last_wall_ns is not None and wall_ns < state.last_wall_ns
            or not all(type(v) in (int, float) and math.isfinite(v)
                       for v in (s_m, speed_mps, lateral_m, heading_rad))
            or type(entry_clear) is not bool):
        raise ValueError('LARGE_CLOCK_OR_OBSERVATION')
    if (state.stage not in ('waiting', 'preparing', 'handover', 'recovery', 'cooldown', 'complete', 'aborted')
            or not 0 <= state.completed_events <= state.event_id <= len(config.sites)
            or not 0 <= state.site_cursor <= len(config.sites)):
        raise ValueError('LARGE_STATE_ORDER')
    gap = (state.last_sim_ns is None or sim_ns-state.last_sim_ns > 150_000_000
           or state.last_wall_ns is None or wall_ns-state.last_wall_ns > 300_000_000)
    st = replace(state, last_sim_ns=sim_ns, last_wall_ns=wall_ns, last_progress_m=s_m,
                 stable_since_ns=None if gap else state.stable_since_ns,
                 target_since_ns=None if gap else state.target_since_ns)
    if st.stage in ('complete', 'aborted'):
        return LargeRecoveryDecision(st, 'nominal', 'invalid' if st.stage == 'aborted' else 'baseline')
    if state.approach_seen and state.last_progress_m is not None and s_m < state.last_progress_m-5.:
        raise ValueError('LARGE_PROGRESS_REGRESSION')
    speed_ok = collection_speed_eligible(speed_mps, config.speed_policy)
    if st.stage in ('waiting', 'cooldown'):
        if s_m < config.sites[0].start_s_m:
            st = replace(st, approach_seen=True)
        if not st.approach_seen:
            return LargeRecoveryDecision(st, 'nominal', 'baseline')
        while st.site_cursor < len(config.sites) and s_m > config.sites[st.site_cursor].start_s_m+1.:
            st = replace(st, skipped_sites=(*st.skipped_sites, st.site_cursor), site_cursor=st.site_cursor+1)
        if st.site_cursor == len(config.sites):
            return LargeRecoveryDecision(replace(st, stage='complete', reason='WINDOWS_FINISHED'), 'nominal', 'baseline')
        ready = (entry_clear and speed_ok and abs(lateral_m) <= .05
                 and abs(heading_rad) <= config.entry_heading_tolerance_rad)
        st = replace(st, stable_since_ns=(st.stable_since_ns if st.stable_since_ns is not None else sim_ns) if ready else None)
        if (s_m < config.sites[st.site_cursor].start_s_m or st.stable_since_ns is None
                or sim_ns-st.stable_since_ns < 1_000_000_000
                or st.next_allowed_ns is not None and sim_ns < st.next_allowed_ns):
            return LargeRecoveryDecision(st, 'nominal', 'baseline')
        return LargeRecoveryDecision(replace(st, stage='preparing', event_id=st.event_id+1,
            start_ns=None, start_wall_ns=None, request_ns=None, release_ns=None, confirmed_ns=None,
            stable_since_ns=None, target_since_ns=None, goal_reached=False, reason=None),
            'preparation', 'approach', 'start')
    site = config.sites[st.site_cursor]
    if st.start_ns is None or st.start_wall_ns is None:
        raise ValueError('LARGE_UNPUBLISHED_START')
    if st.stage in ('preparing', 'handover'):
        bound = ('LATERAL_LIMIT' if abs(lateral_m) >= .75
                 else 'HEADING_LIMIT' if abs(heading_rad) >= math.radians(12.)
                 else 'PREPARATION_TIMEOUT' if sim_ns-st.start_ns >= 12_000_000_000
                     or wall_ns-st.start_wall_ns >= 24_000_000_000
                 else 'PREPARATION_PROGRESS_LIMIT' if s_m > site.release_s_m+2.
                 else 'PREPARATION_SPEED_LIMIT' if not speed_ok else None)
        if bound:
            # Fresh nominal remains the teacher fallback, with an invalid event.
            return LargeRecoveryDecision(replace(st, stage='aborted', reason=bound), 'nominal', 'invalid')
    if st.stage == 'preparing':
        goal = site.at_goal(lateral_m, heading_rad)
        since = (st.target_since_ns if st.target_since_ns is not None else sim_ns) if goal else None
        st = replace(st, target_since_ns=since)
        if s_m > site.release_s_m+1.:
            return LargeRecoveryDecision(replace(st, stage='aborted', reason='TARGET_NOT_REACHED'), 'nominal', 'invalid')
        if s_m >= site.release_s_m and since is not None and sim_ns-since >= 250_000_000:
            return LargeRecoveryDecision(replace(st, stage='handover', goal_reached=True), 'preparation', 'hold', 'request')
        return LargeRecoveryDecision(st, 'preparation', 'approach' if s_m < site.release_s_m-site.settle_distance_m else 'hold')
    if st.stage == 'handover':
        if st.request_ns is None:
            raise ValueError('LARGE_UNPUBLISHED_REQUEST')
        if sim_ns-st.request_ns > 300_000_000:
            raise ValueError('LARGE_HANDOVER_TIMEOUT')
        if st.request_ns < nominal_stamp_ns <= sim_ns:
            # The normal PP always has the normal reference. No CSV/parameter
            # acknowledgement is used as evidence of a control switch.
            return LargeRecoveryDecision(replace(st, stage='recovery', stable_since_ns=None), 'nominal', 'recovery', 'release')
        return LargeRecoveryDecision(st, 'preparation', 'hold')
    if st.release_ns is None:
        raise ValueError('LARGE_UNPUBLISHED_RELEASE')
    settled = speed_ok and abs(lateral_m) <= .1 and abs(heading_rad) <= math.radians(2.)
    since = (st.stable_since_ns if st.stable_since_ns is not None else sim_ns) if settled else None
    st = replace(st, stable_since_ns=since)
    transition = None
    if st.confirmed_ns is None and since is not None and sim_ns-since >= 1_000_000_000:
        st = replace(st, confirmed_ns=sim_ns)
        transition = 'confirm'
    if sim_ns-st.release_ns >= config.recovery_duration_ns:
        if st.confirmed_ns is None or st.confirmed_ns-st.release_ns > config.recovery_duration_ns:
            return LargeRecoveryDecision(replace(st, stage='aborted', reason='RECOVERY_NOT_CONFIRMED'), 'nominal', 'invalid')
        if since is None or sim_ns-since < 1_000_000_000:
            return LargeRecoveryDecision(replace(st, stage='aborted', reason='RECOVERY_NOT_STABLE_AT_END'), 'nominal', 'invalid')
        cursor = st.site_cursor+1
        st = replace(st, site_cursor=cursor, completed_events=st.completed_events+1,
                     stage='complete' if cursor == len(config.sites) else 'cooldown',
                     stable_since_ns=None, reason='EVENT_COMPLETE')
        return LargeRecoveryDecision(st, 'nominal', 'baseline', 'complete')
    return LargeRecoveryDecision(st, 'nominal', 'recovery', transition)


def commit_large_recovery(decision: LargeRecoveryDecision, *, sim_ns: int,
                          wall_ns: int, sequence: int) -> LargeRecoveryState:
    """Commit only AFTER a validated go command is actually published.

    Transition timestamps are publication times, never proposal or parameter
    request times. The simulator's later actuator application is not asserted.
    """
    st = decision.state
    if (any(type(v) is not int or v < 0 for v in (sim_ns, wall_ns, sequence))
            or st.last_sim_ns is None or sim_ns < st.last_sim_ns
            or st.last_wall_ns is None or wall_ns < st.last_wall_ns
            or sequence <= st.last_sequence):
        raise ValueError('LARGE_COMMIT_ORDER')
    st = replace(st, last_sim_ns=sim_ns, last_wall_ns=wall_ns, last_sequence=sequence)
    if decision.transition == 'start':
        st = replace(st, start_ns=sim_ns, start_wall_ns=wall_ns)
    elif decision.transition == 'request':
        st = replace(st, request_ns=sim_ns)
    elif decision.transition == 'release':
        st = replace(st, release_ns=sim_ns)
    elif decision.transition == 'confirm':
        st = replace(st, confirmed_ns=sim_ns)
    elif decision.transition == 'complete':
        st = replace(st, next_allowed_ns=sim_ns+3_000_000_000)
    return st


def large_recovery_events(rows: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    """Audit publication switches and observed recovery; no sensor-label claim."""
    groups: dict[int, list[Mapping[str, Any]]] = {}
    for row in rows:
        if row.get('annotation_schema') != SCHEMA:
            raise ValueError('LARGE_AUDIT_SCHEMA')
        data = row.get('large_recovery', {})
        if data.get('applied') and row.get('publication') and data['state']['event_id']:
            groups.setdefault(data['state']['event_id'], []).append(row)
    result = []
    for event_id, group in groups.items():
        preparations = [r for r in group if r['large_recovery']['command_source'] == 'preparation']
        recoveries = [r for r in group if r['phase'] == 'recovery']
        if not preparations:
            raise ValueError('LARGE_AUDIT_MISSING_PREPARATION')
        first = preparations[0]['large_recovery']
        config = LargeRecoveryConfig(**first['config'])
        site = config.sites[first['state']['site_cursor']]
        requests = [r for r in preparations if r['large_recovery']['state']['stage'] == 'handover']
        boundary = bool(requests and recoveries)
        if boundary:
            request = requests[0]['publication']['sim_ns']
            release = recoveries[0]['publication']['sim_ns']
            d = recoveries[0]['large_recovery']
            boundary = (d['state']['request_ns'] == request and d['state']['release_ns'] == release
                        and d['nominal_stamp_ns'] > request and release >= d['nominal_stamp_ns']
                        and requests[0]['publication']['sequence'] < recoveries[0]['publication']['sequence']
                        and site.at_goal(requests[0]['large_recovery']['lateral_error_m'],
                                         requests[0]['large_recovery']['heading_error_rad']))
        else:
            request = release = None
        confirmed = False; since = None; previous = None; previous_wall = None
        for row in recoveries:
            data = row['large_recovery']; pub = row['publication']; t = pub['sim_ns']
            if data['command_source'] != 'nominal':
                raise ValueError('LARGE_PREPARATION_IN_RECOVERY')
            if previous is not None and (t-previous > 150_000_000 or t < previous
                    or pub['monotonic_ns']-previous_wall > 300_000_000):
                since = None
            settled = (abs(data['lateral_error_m']) <= .1 and abs(data['heading_error_rad']) <= math.radians(2.)
                       and collection_speed_eligible(row['speed_mps'], config.speed_policy))
            since = (since if since is not None else t) if settled else None
            if since is not None and t-since >= 1_000_000_000 and release is not None and t-release <= config.recovery_duration_ns:
                confirmed = True
            previous = t; previous_wall = pub['monotonic_ns']
        completions = [r for r in group if r['large_recovery']['state']['completed_events'] >= event_id]
        completed = bool(completions)
        stable_end = False
        if completed and since is not None and previous is not None:
            end = completions[0]; data = end['large_recovery']; t = end['publication']['sim_ns']
            stable_end = (0 <= t-previous <= 150_000_000 and t-since >= 1_000_000_000
                          and abs(data['lateral_error_m']) <= .1 and abs(data['heading_error_rad']) <= math.radians(2.)
                          and collection_speed_eligible(end['speed_mps'], config.speed_policy))
        target_samples = sum(abs(r['large_recovery']['lateral_error_m']-site.target_offset_m) <= .1 for r in recoveries)
        result.append(dict(event_id=event_id, site_id=site.site_id, target_offset_m=site.target_offset_m,
            speed_policy=config.speed_policy,
            recovery_duration_s=config.recovery_duration_s,
            peak_observed_speed_mps=max(r['speed_mps'] for r in group),
            above_legacy_speed_samples=sum(r['speed_mps'] > 1.4 for r in group),
            corner_id=site.corner_id, target_heading_rad=site.target_heading_rad,
            heading_tolerance_rad=site.heading_tolerance_rad,
            preparation_start_ns=preparations[0]['publication']['sim_ns'], request_ns=request, release_ns=release,
            end_publication_ns=completions[0]['publication']['sim_ns'] if completions else None,
            publication_switch_verified=boundary, recovery_confirmed=bool(boundary and confirmed and completed and stable_end),
            stable_at_end=stable_end,
            completed=completed, recovery_sample_count=len(recoveries),
            peak_observed_lateral_m=max(abs(r['large_recovery']['lateral_error_m']) for r in group
                                        if r['large_recovery']['lateral_error_m'] is not None),
            target_band_recovery_samples=target_samples, target_band_observed=target_samples >= 2))
    return result


def large_event_at(stamp_ns: int, events: Sequence[Mapping[str, Any]]) -> Mapping[str, Any] | None:
    """Associate camera capture with its independently confirmed recovery event."""
    matches = [e for e in events if e['recovery_confirmed']
               and e['release_ns'] <= stamp_ns < e['end_publication_ns']]
    if len(matches) > 1:
        raise ValueError('LARGE_EVENT_OVERLAP')
    return matches[0] if matches else None
