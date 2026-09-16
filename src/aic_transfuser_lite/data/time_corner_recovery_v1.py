"""Finite corner coverage across laps, using validated observed camera anchors.

All coordinates are base-course progress m, lateral m and heading rad relative
to the measured nominal guide. Coverage is per whole-run split, never frames.
"""
from __future__ import annotations

from dataclasses import asdict
import math
from typing import Any, Mapping, Sequence

from .time_large_recovery_v1 import LargeRecoveryConfig, LargeRecoverySite, MAX_EVENTS, collection_speed_eligible


def partition_corner_sites(sites: Sequence[LargeRecoverySite], *, event_cap: int = 3
                           ) -> tuple[LargeRecoveryConfig, ...]:
    """Cover every named corner exactly once, separating close sites into laps.

    No site is randomly dropped to fill a lap; each resulting config still
    passes the ordinary 40 m spacing and bounded event-count contracts.
    """
    if (type(event_cap) is not int or not 1 <= event_cap <= MAX_EVENTS or not sites
            or any(not isinstance(s, LargeRecoverySite) or s.corner_id is None for s in sites)
            or len({s.corner_id for s in sites}) != len(sites)
            or len({s.site_id for s in sites}) != len(sites)):
        raise ValueError('CORNER_CATALOG_CONTRACT')
    laps: list[list[LargeRecoverySite]] = []
    for site in sorted(sites, key=lambda s: (s.start_s_m, s.corner_id)):
        lap = next((group for group in laps if len(group) < event_cap
                    and site.start_s_m-group[-1].start_s_m >= 40.), None)
        if lap is None:
            lap = []
            laps.append(lap)
        lap.append(site)
    return tuple(LargeRecoveryConfig(tuple(lap), event_cap=event_cap) for lap in laps)


def corner_coverage(sites: Sequence[LargeRecoverySite], audits: Sequence[Mapping[str, Any]], *,
                    minimum_state_anchors: int = 3, minimum_event_anchors: int = 60
                    ) -> dict[str, Any]:
    """Require causal-input/full-future anchors at entry AND observed recovery.

    Caller supplies completed collection summaries plus their independently
    audited anchor_states. A complete lap or reached command goal alone cannot
    close a target. Repeated correlated frames do not count as distinct runs.
    """
    partition_corner_sites(sites)
    if (type(minimum_state_anchors) is not int or minimum_state_anchors < 1
            or type(minimum_event_anchors) is not int or minimum_event_anchors < 1):
        raise ValueError('CORNER_COVERAGE_THRESHOLDS')
    run_ids: set[str] = set()
    by_corner = {s.corner_id: {'target': asdict(s), 'train': [], 'validation': []} for s in sites}
    for audit in audits:
        name, split = audit['run_id'], audit['split']
        if name in run_ids or split not in ('train', 'validation'):
            raise ValueError('CORNER_RUN_SPLIT_IDENTITY')
        run_ids.add(name)
        if (audit['result_status'] != 'COMPLETE_LAP' or audit['fault'] is not None
                or not audit['stop_confirmed'] or not audit['closed_bag'] or audit['accepted'] <= 0):
            continue
        states = audit['anchor_states']
        if len(states) != audit['accepted'] or len({a['anchor_id'] for a in states}) != len(states):
            raise ValueError('CORNER_ACCEPTED_ANCHOR_IDENTITY')
        for site in sites:
            events = [e for e in audit['events'] if e.get('corner_id') == site.corner_id
                      and e['recovery_confirmed'] and e['completed'] and e['stable_at_end']]
            # Only anchors bound to this confirmed event and matching target
            # can satisfy it. Earlier nearby recovery cannot substitute.
            ids = {e['event_id'] for e in events if e['target_offset_m'] == site.target_offset_m
                   and e['target_heading_rad'] == site.target_heading_rad}
            policies = {e['event_id']: e.get('speed_policy', 'bounded_5kmh_v1') for e in events}
            anchors = [a for a in states if a['event_id'] in ids and a['site_id'] == site.site_id]
            matched = []
            for a in anchors:
                values = [a[k] for k in ('base_s_m', 'lateral_m', 'heading_rad', 'speed_mps')]
                if not all(type(v) in (int, float) and math.isfinite(v) for v in values):
                    raise ValueError('CORNER_ANCHOR_FINITE')
                if (site.release_s_m-.5 <= a['base_s_m'] <= site.release_s_m+3.
                        and site.at_goal(a['lateral_m'], a['heading_rad'])
                        and collection_speed_eligible(a['speed_mps'], policies[a['event_id']])):
                    matched.append(a['anchor_id'])
            if len(anchors) >= minimum_event_anchors and len(matched) >= minimum_state_anchors:
                by_corner[site.corner_id][split].append(dict(run_id=name, event_anchors=len(anchors),
                    entry_state_anchors=len(matched), entry_anchor_ids=matched))
    missing = {split: [s.corner_id for s in sites if not by_corner[s.corner_id][split]]
               for split in ('train', 'validation')}
    return dict(schema='corner_recovery_coverage_v1', corners=by_corner, missing=missing,
                complete=not any(missing.values()), minimum_state_anchors=minimum_state_anchors,
                minimum_event_anchors=minimum_event_anchors,
                scope='same_corner_independent_runs_not_held_out_corner_generalization')


def pending_corner_laps(sites: Sequence[LargeRecoverySite], coverage: Mapping[str, Any], *,
                        split: str, event_cap: int = 3) -> tuple[LargeRecoveryConfig, ...]:
    """Carry missing targets to later laps; caller enforces finite attempt budget."""
    if coverage.get('schema') != 'corner_recovery_coverage_v1' or split not in ('train', 'validation'):
        raise ValueError('CORNER_PENDING_COVERAGE')
    if set(coverage['corners']) != {s.corner_id for s in sites} or any(
            LargeRecoverySite(**coverage['corners'][s.corner_id]['target']) != s for s in sites):
        raise ValueError('CORNER_TARGET_IDENTITY')
    missing = coverage['missing'][split]
    expected = [s.corner_id for s in sites if not coverage['corners'][s.corner_id][split]]
    if missing != expected:
        raise ValueError('CORNER_PENDING_IDENTITY')
    selected = [s for s in sites if s.corner_id in missing]
    return partition_corner_sites(selected, event_cap=event_cap) if selected else ()
