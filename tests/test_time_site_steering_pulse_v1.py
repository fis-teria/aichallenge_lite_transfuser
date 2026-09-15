from dataclasses import asdict, replace
import json
import math

import numpy as np
import pytest

from aic_transfuser_lite.data.time_random_steering_pulse_v1 import (
    SCHEMA, PulseSite, RandomPulseConfig, RandomPulseState, propose_random_pulse,
    random_pulse_events, validate_random_guide,
)
from aic_transfuser_lite.data.time_steering_pulse_v1 import SteeringPulseConfig
from aic_transfuser_lite.data.time_recovery_collection_v1 import collection_phase_windows, recovery_teacher_mask

TEMPLATE = SteeringPulseConfig(10., .1, duration_s=2., plateau_s=1.5, start_window_m=2.)


def config(sites=None):
    sites = sites or (PulseSite('R01', 15., 1), PulseSite('S00', 45., -1), PulseSite('R02', 75., 1))
    return RandomPulseConfig(915110, max_events=len(sites), start_min_m=sites[0].start_s_m,
                             start_max_m=sites[-1].start_s_m, sites=sites)


def record(cfg=None, *, missing_first=False, recover=True, gap=False):
    cfg = cfg or config()
    state = RandomPulseState(); rows = []
    origin = cfg.start_min_m-5.
    for i in range(round((cfg.start_max_m-origin+25.)/1.27/.05)):
        t = i*.05; s = origin+t*1.27
        if (gap and state.event_id == 1 and state.pulse.zero_ns is not None
                and 9. < t-state.pulse.zero_ns/1e9 < 9.25):
            continue
        lateral = .15 if not recover and state.stage == 'pulse' else 0.
        d = propose_random_pulse(cfg, TEMPLATE, state, sim_ns=round(t*1e9), wall_ns=round((t+10)*1e9),
            s_m=s, speed_mps=1.27, lateral_m=lateral, heading_rad=0.,
            entry_clear=not (missing_first and s < cfg.sites[0].start_s_m+3.))
        state = d.state
        rows.append(dict(annotation_schema=SCHEMA, sim_ns=round(t*1e9), phase=d.phase,
            publication=dict(sim_ns=round(t*1e9)+5_000_000, monotonic_ns=round((t+10)*1e9), sequence=len(rows)+1),
            target_speed_mps=5/3.6, speed_mps=1.27,
            pulse=dict(applied=True, state=asdict(state.pulse), requested_rad=d.perturbation_rad,
                       effective_rad=d.perturbation_rad, lateral_error_m=lateral, heading_error_rad=0.),
            random_pulse=dict(config=asdict(cfg), state=asdict(state))))
    return rows, state


def test_sites_json_round_trip_and_global_guide_support():
    cfg = config((PulseSite('R10', 315., -1),))
    assert RandomPulseConfig(**json.loads(json.dumps(asdict(cfg)))) == cfg
    validate_random_guide(cfg, TEMPLATE, [[.1, 0., 0.], [353., 0., 0.]])
    with pytest.raises(ValueError, match='COVERAGE'):
        validate_random_guide(cfg, TEMPLATE, [[.1, 0., 0.], [342., 0., 0.]])
    rows, state = record(cfg)
    events = random_pulse_events(rows)
    assert state.completed_events == 1 and events[0]['site_id'] == 'R10'
    assert 315. <= events[0]['start_s_m'] <= 317.
    assert events[0]['sign'] == -1 and events[0]['recovery_confirmed']


def test_sites_are_published_once_and_teacher_future_never_crosses_pulse():
    rows, state = record()
    events = random_pulse_events(rows)
    assert state.completed_events == 3 and state.site_cursor == 3 and not state.skipped_sites
    assert [e['site_id'] for e in events] == ['R01', 'S00', 'R02']
    assert [e['sign'] for e in events] == [1, -1, 1]
    windows = collection_phase_windows(rows)
    for e in events:
        assert e['recovery_confirmed'] and recovery_teacher_mask(e['zero_publication_ns']+150_000_000, windows).all()
        assert not recovery_teacher_mask(e['start_publication_ns'], windows).any()
    for a, b in zip(events, events[1:]):
        assert b['start_publication_ns'] >= a['end_publication_ns']+3_000_000_000


def test_missed_site_is_logged_without_delayed_or_wrong_direction_injection():
    rows, state = record(missing_first=True)
    events = random_pulse_events(rows)
    assert state.skipped_sites == (0,) and state.completed_events == 2
    assert [e['site_id'] for e in events] == ['S00', 'R02']
    assert [e['sign'] for e in events] == [-1, 1]
    assert 45. <= events[0]['start_s_m'] <= 47.


def test_failed_recovery_aborts_remaining_sites_and_is_not_eligible_teacher():
    rows, state = record(recover=False)
    events = random_pulse_events(rows)
    assert state.stage == 'aborted' and state.completed_events == 0 and len(events) == 1
    assert not events[0]['recovery_confirmed']
    assert not any(w.phase == 'recovery' for w in collection_phase_windows(rows))


def test_late_gap_does_not_erase_recovery_but_invalidates_cross_gap_labels():
    rows, state = record(gap=True)
    events = random_pulse_events(rows)
    assert state.completed_events == 3 and all(e['recovery_confirmed'] for e in events)
    windows = collection_phase_windows(rows)
    assert not recovery_teacher_mask(events[0]['zero_publication_ns']+8_000_000_000, windows).all()


def test_rejected_proposal_is_repeatable_and_unfinished_lap_wrap_aborts():
    cfg = config(); st = RandomPulseState()
    for i in range(101):
        kwargs = dict(sim_ns=i*50_000_000, wall_ns=(i+200)*50_000_000, s_m=10.+i*.0635,
                      speed_mps=1.27, lateral_m=0., heading_rad=0., entry_clear=True)
        d = propose_random_pulse(cfg, TEMPLATE, st, **kwargs)
        if d.state.event_id:
            assert st.event_id == 0
            assert d == propose_random_pulse(cfg, TEMPLATE, st, **kwargs)
            break
        st = d.state
    else:
        pytest.fail('No proposed start')
    seam = propose_random_pulse(cfg, TEMPLATE, d.state, **{**kwargs, 'sim_ns':kwargs['sim_ns']+50_000_000,
        'wall_ns':kwargs['wall_ns']+50_000_000, 's_m':0.})
    assert seam.state.stage == 'aborted' and seam.perturbation_rad == 0.
    assert seam.state.reason == 'SITE_PROGRESS_REGRESSION'


@pytest.mark.parametrize('sites', [
    [dict(site_id='S01', start_s_m=15., sign=1)],
    [dict(site_id='R01', start_s_m=float('nan'), sign=1)],
    [dict(site_id='R01', start_s_m=15., sign=True)],
    [dict(site_id='R01', start_s_m=15., sign=1), dict(site_id='R01', start_s_m=50., sign=-1)],
    [dict(site_id='R01', start_s_m=15., sign=1), dict(site_id='R02', start_s_m=30., sign=-1)],
])
def test_invalid_sites_are_rejected(sites):
    with pytest.raises(ValueError):
        RandomPulseConfig(1, max_events=len(sites), start_min_m=15., start_max_m=50., sites=sites)


def test_stop_command_cannot_commit_or_create_a_recovery_boundary():
    rows, _ = record()
    first = next(i for i, r in enumerate(rows) if r['phase'] == 'recovery')
    rows = rows[:first+1]
    rows[-1]['pulse']['applied'] = False
    rows[-1]['target_speed_mps'] = 0.
    rows[-1]['phase'] = 'invalid'
    events = random_pulse_events(rows)
    assert not events[0]['recovery_confirmed'] and events[0]['zero_publication_ns'] is None
