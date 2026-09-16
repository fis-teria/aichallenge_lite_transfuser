from copy import deepcopy
from dataclasses import asdict, replace
import math

import numpy as np
import pytest

from aic_transfuser_lite.data.time_large_recovery_v1 import (
    SCHEMA, LargeRecoverySite, LargeRecoveryConfig, LargeRecoveryState,
    propose_large_recovery, commit_large_recovery, large_recovery_events, large_event_at, select_large_sites,
    collection_speed_eligible,
)
from aic_transfuser_lite.data.time_recovery_collection_v1 import collection_phase_windows, recovery_teacher_mask, check_collection_input_time
from aic_transfuser_lite.runtime.recovery_disturbance_markers import DisturbanceLocations


def config(count=1, offset=.6):
    return LargeRecoveryConfig(tuple(LargeRecoverySite('P'+str(i), 60.+50*i, offset*(-1)**i)
                                    for i in range(count)), seed=42, event_cap=count)


def proposal(state=LargeRecoveryState(), cfg=None, **changes):
    values = dict(sim_ns=1_000_000_000, wall_ns=11_000_000_000, s_m=49., speed_mps=1.25,
                  lateral_m=0., heading_rad=0., nominal_stamp_ns=1_000_000_000, entry_clear=True)
    values.update(changes)
    return propose_large_recovery(cfg or config(), state, **values)


def synthetic_run(cfg=None, *, reach=True, speed_mps=1.25, recovery_decay_s=4., failed_site_ids=()):
    """Synthetic observed-state replay; not an AWSIM/dynamic vehicle result."""
    cfg = cfg or config(2)
    state = LargeRecoveryState(); rows = []
    start_s = cfg.sites[0].start_s_m-3.
    finish = 1+(cfg.sites[-1].release_s_m-start_s)/1.25+cfg.recovery_duration_s+5.
    for index in range(round((finish-1)/.05)):
        t = 1+index*.05; ns = round(t*1e9); s = start_s+(t-1)*1.25
        lat = 0.
        if (state.stage in ('preparing', 'handover') and reach
                and cfg.sites[state.site_cursor].site_id not in failed_site_ids):
            site = cfg.sites[state.site_cursor]; f = np.clip((s-site.start_s_m)/8., 0., 1.)
            lat = site.target_offset_m*f*f*(3-2*f)
        if state.stage == 'recovery':
            lat = cfg.sites[state.site_cursor].target_offset_m*max(0., 1-(ns-state.release_ns)/(recovery_decay_s*1e9))
        decision = proposal(state, cfg, sim_ns=ns, wall_ns=ns+10_000_000_000,
                            s_m=s, nominal_stamp_ns=ns, lateral_m=float(lat), speed_mps=speed_mps)
        state = commit_large_recovery(decision, sim_ns=ns, wall_ns=ns+10_000_000_001, sequence=index+1)
        rows.append(dict(annotation_schema=SCHEMA, phase=decision.phase, target_speed_mps=5/3.6, speed_mps=speed_mps,
            publication=dict(sim_ns=ns, monotonic_ns=ns+10_000_000_001, sequence=index+1),
            large_recovery=dict(config=asdict(cfg), state=asdict(state), applied=True,
                command_source=decision.command_source, lateral_error_m=float(lat), heading_error_rad=0., nominal_stamp_ns=ns),
            current_pose=dict(x_m=s, y_m=float(lat), yaw_rad=0., stamp_ns=ns), projection=dict(s_m=s)))
    return state, rows


def test_two_events_recover_with_publication_boundary_and_future_tail():
    state, rows = synthetic_run()
    events = large_recovery_events(rows)
    assert state.stage == 'complete' and state.completed_events == 2
    assert all(e['recovery_confirmed'] and e['target_band_observed'] for e in events)
    assert events[1]['preparation_start_ns'] >= events[0]['end_publication_ns']+3_000_000_000
    windows = collection_phase_windows(rows)
    for event in events:
        assert not recovery_teacher_mask(event['preparation_start_ns'], windows).any()
        assert recovery_teacher_mask(event['release_ns']+150_000_000, windows).all()
        assert large_event_at(event['release_ns'], events)['event_id'] == event['event_id']
    assert large_event_at(events[0]['request_ns'], events) is None


@pytest.mark.parametrize('speed', [1.40056765, 1.43136, 1.8])
def test_record_actual_speed_reaches_all_events_and_survives_teacher_audit(speed):
    cfg = replace(config(2), speed_policy='record_actual_v1')
    state, rows = synthetic_run(cfg, speed_mps=speed)
    events = large_recovery_events(rows)
    assert state.completed_events == 2
    assert all(e['recovery_confirmed'] and e['stable_at_end'] for e in events)
    assert all(e['above_legacy_speed_samples'] > 0 and e['peak_observed_speed_mps'] == speed for e in events)
    assert recovery_teacher_mask(events[0]['release_ns']+150_000_000, collection_phase_windows(rows)).all()
    # Config recorded before this policy existed keeps the original meaning.
    for row in rows:
        del row['large_recovery']['config']['speed_policy']
    assert not any(e['recovery_confirmed'] for e in large_recovery_events(rows))


def test_record_actual_policy_rejects_unknown_nonfinite_and_nonforward_observations():
    with pytest.raises(ValueError, match='SPEED_POLICY'):
        replace(config(), speed_policy='anything')
    for speed in (float('nan'), float('inf'), -.1, 0., 1.14):
        assert not collection_speed_eligible(speed, 'record_actual_v1')
    assert not collection_speed_eligible(1.5, 'bounded_5kmh_v1')
    assert collection_speed_eligible(1.5, 'record_actual_v1')


def test_entry_heading_can_match_recovered_state_without_bypassing_current_clearance():
    cfg = replace(config(), speed_policy='record_actual_v1')
    st = LargeRecoveryState(approach_seen=True, stable_since_ns=0,
        last_sim_ns=950_000_000, last_wall_ns=10_950_000_000)
    measured = dict(s_m=50., speed_mps=1.4025565, lateral_m=.01195,
                    heading_rad=math.radians(1.0764366))
    assert proposal(st, cfg, **measured).state.event_id == 0
    aligned = replace(cfg, entry_heading_tolerance_rad=math.radians(2.))
    assert proposal(st, aligned, **measured).transition == 'start'
    assert proposal(st, aligned, **measured, entry_clear=False).state.event_id == 0
    assert proposal(st, aligned, **{**measured, 'heading_rad':math.radians(2.01)}).state.event_id == 0
    assert proposal(st, aligned, **{**measured, 'lateral_m':.051}).state.event_id == 0
    for invalid in (float('nan'), 0., math.radians(2.01), True):
        with pytest.raises(ValueError, match='ENTRY_HEADING_TOLERANCE'):
            replace(cfg, entry_heading_tolerance_rad=invalid)


def test_optional_entry_lead_retains_clearance_stability_and_cooldown():
    cfg = config()
    early = replace(cfg, sites=(replace(cfg.sites[0], entry_window_lead_m=.5),))
    st = LargeRecoveryState(approach_seen=True, stable_since_ns=0,
        last_sim_ns=950_000_000, last_wall_ns=10_950_000_000)
    assert cfg.sites[0].entry_start_s_m == 50.
    assert early.sites[0].entry_start_s_m == 49.5
    assert early.sites[0].start_s_m == 50. and early.sites[0].release_s_m == 60.
    assert proposal(st, cfg, s_m=49.5).transition is None
    assert proposal(st, early, s_m=49.49).transition is None
    assert proposal(st, early, s_m=49.5).transition == 'start'
    for changes in ({'entry_clear':False}, {'lateral_m':.051}, {'heading_rad':math.radians(1.01)},
                    {'speed_mps':1.14}):
        assert proposal(st, early, s_m=49.5, **changes).transition is None
    assert proposal(replace(st, stable_since_ns=1), early, s_m=49.5).transition is None
    assert proposal(replace(st, next_allowed_ns=1_000_000_001), early, s_m=49.5).transition is None
    for invalid in (True, -.01, .5001, float('nan'), float('inf')):
        with pytest.raises(ValueError, match='LARGE_SITE_CONTRACT'):
            replace(cfg.sites[0], entry_window_lead_m=invalid)


@pytest.mark.parametrize('offset', [-.6, -.4, -.2, .2, .4, .6])
def test_parallel_displacement_goals_are_measured_with_either_sign(offset):
    state, rows = synthetic_run(config(1, offset))
    event, = large_recovery_events(rows)
    assert state.completed_events == 1 and event['recovery_confirmed']
    assert event['peak_observed_lateral_m'] == pytest.approx(abs(offset))


def test_explicit_longer_recovery_records_slow_return_without_weakening_stability_or_old_audit():
    original = config(1)
    state, rows = synthetic_run(original, recovery_decay_s=12.)
    assert state.stage == 'aborted' and state.reason == 'RECOVERY_NOT_CONFIRMED'
    assert not large_recovery_events(rows)[0]['recovery_confirmed']
    longer = replace(original, recovery_duration_s=15.)
    state, rows = synthetic_run(longer, recovery_decay_s=12.)
    event, = large_recovery_events(rows)
    assert state.stage == 'complete' and state.completed_events == 1
    assert event['recovery_confirmed'] and event['stable_at_end']
    assert event['recovery_duration_s'] == 15.
    assert event['end_publication_ns']-event['release_ns'] == 15_000_000_000
    assert recovery_teacher_mask(event['release_ns']+150_000_000, collection_phase_windows(rows)).all()
    changed = deepcopy(rows)
    completion = next(r for r in changed if r['large_recovery']['state']['completed_events'] == 1)
    completion['large_recovery']['lateral_error_m'] = .101
    assert not large_recovery_events(changed)[0]['recovery_confirmed']
    for row in rows:
        del row['large_recovery']['config']['recovery_duration_s']
    assert not large_recovery_events(rows)[0]['recovery_confirmed']
    for invalid in (9.99, 15.01, True, float('nan'), float('inf')):
        with pytest.raises(ValueError, match='LARGE_RECOVERY_DURATION'):
            replace(original, recovery_duration_s=invalid)


def test_failed_target_prevents_later_injection_and_labels():
    state, rows = synthetic_run(reach=False)
    assert state.stage == 'aborted' and state.reason == 'TARGET_NOT_REACHED'
    assert state.event_id == 1 and state.completed_events == 0
    assert not any(w.phase == 'recovery' for w in collection_phase_windows(rows))


def test_explicit_target_miss_can_continue_without_labeling_failure_as_recovery():
    cfg=replace(config(2),failed_site_policy='continue_after_target_miss_v1',recovery_duration_s=15.)
    state,rows=synthetic_run(cfg,failed_site_ids=('P0',))
    events=large_recovery_events(rows)
    assert state.stage=='complete' and state.event_id==2 and state.completed_events==1
    assert state.skipped_sites==(0,)
    assert not events[0]['completed'] and not events[0]['recovery_confirmed']
    assert events[1]['completed'] and events[1]['recovery_confirmed']
    failed=next(r for r in rows if r['large_recovery']['state']['reason']=='TARGET_NOT_REACHED')
    assert events[1]['preparation_start_ns']>=failed['publication']['sim_ns']+18_000_000_000
    windows=collection_phase_windows(rows)
    assert not recovery_teacher_mask(failed['publication']['sim_ns'],windows).any()
    assert recovery_teacher_mask(events[1]['release_ns']+150_000_000,windows).all()
    changed=deepcopy(rows)
    changed[-1]['large_recovery']['state']['completed_events']=2
    with pytest.raises(ValueError,match='COMPLETION_COUNTER'):large_recovery_events(changed)


def test_continue_policy_keeps_entry_stability_and_other_preparation_bounds():
    cfg=replace(config(2),failed_site_policy='continue_after_target_miss_v1')
    st=LargeRecoveryState(stage='cooldown',site_cursor=1,event_id=1,approach_seen=True,
        last_sim_ns=950_000_000,last_wall_ns=10_950_000_000,stable_since_ns=0,next_allowed_ns=2_000_000_000)
    assert proposal(st,cfg,s_m=100.).transition is None
    st=replace(st,next_allowed_ns=0)
    assert proposal(st,cfg,s_m=100.,entry_clear=False).transition is None
    assert proposal(st,cfg,s_m=100.,lateral_m=.06).transition is None
    assert proposal(st,cfg,s_m=100.).transition=='start'
    st=LargeRecoveryState(stage='preparing',event_id=1,start_ns=0,start_wall_ns=10_000_000_000)
    assert proposal(st,cfg,lateral_m=.76).state.stage=='aborted'
    with pytest.raises(ValueError,match='LARGE_FAILED_SITE_POLICY'):
        replace(cfg,failed_site_policy='continue_after_sensor_fault')


def test_longer_settling_moves_preparation_earlier_without_releasing_early():
    cfg=LargeRecoveryConfig((LargeRecoverySite('P00',60.,.2,settle_distance_m=4.),))
    assert cfg.sites[0].start_s_m==48.
    state,rows=synthetic_run(cfg)
    event,=large_recovery_events(rows)
    request=next(r for r in rows if r['publication']['sim_ns']==event['request_ns'])
    assert state.completed_events==1 and event['recovery_confirmed']
    assert 60.<=request['projection']['s_m']<=61.
    assert any(r['phase']=='hold' and r['projection']['s_m']<58. for r in rows)
    with pytest.raises(ValueError,match='LARGE_SITE_CONTRACT'):
        LargeRecoverySite('P00',60.,.2,settle_distance_m=6.)


def test_rejected_proposal_never_consumes_event_or_release():
    prior = LargeRecoveryState(approach_seen=True, stable_since_ns=0,
        last_sim_ns=950_000_000, last_wall_ns=10_950_000_000)
    a = proposal(prior, s_m=50.)
    b = proposal(prior, s_m=50.)
    assert a == b and prior.event_id == 0 and a.state.start_ns is None
    published = commit_large_recovery(a, sim_ns=1_010_000_000, wall_ns=11_010_000_000, sequence=10)
    assert published.event_id == 1 and published.start_ns == 1_010_000_000
    with pytest.raises(ValueError, match='COMMIT_ORDER'):
        commit_large_recovery(a, sim_ns=999_999_999, wall_ns=11_010_000_000, sequence=10)


def test_handover_requires_new_nonfuture_nominal_command():
    st = LargeRecoveryState(stage='handover', event_id=1, approach_seen=True,
        last_sim_ns=900_000_000, last_wall_ns=10_900_000_000,
        start_ns=0, start_wall_ns=10_000_000_000, request_ns=900_000_000, goal_reached=True)
    assert proposal(st, s_m=60., lateral_m=.6, nominal_stamp_ns=900_000_000).command_source == 'preparation'
    assert proposal(st, s_m=60., lateral_m=.6, nominal_stamp_ns=1_010_000_000).command_source == 'preparation'
    decision = proposal(st, s_m=60., lateral_m=.6)
    assert decision.transition == 'release' and decision.state.release_ns is None
    assert commit_large_recovery(decision, sim_ns=1_020_000_000, wall_ns=11_020_000_000, sequence=1).release_ns == 1_020_000_000
    with pytest.raises(ValueError, match='HANDOVER_TIMEOUT'):
        proposal(st, s_m=60., lateral_m=.6, sim_ns=1_200_000_001)


@pytest.mark.parametrize('change,reason', [({'lateral_m':.75}, 'LATERAL_LIMIT'),
    ({'heading_rad':math.radians(12.)}, 'HEADING_LIMIT'), ({'speed_mps':1.5}, 'PREPARATION_SPEED_LIMIT'),
    ({'s_m':63.}, 'PREPARATION_PROGRESS_LIMIT'), ({'sim_ns':12_000_000_000}, 'PREPARATION_TIMEOUT')])
def test_dedicated_bounds_fall_back_without_increasing_disturbance(change, reason):
    st = LargeRecoveryState(stage='preparing', event_id=1, start_ns=0, start_wall_ns=10_000_000_000)
    decision = proposal(st, **change)
    assert decision.command_source == 'nominal' and decision.phase == 'invalid'
    assert decision.state.stage == 'aborted' and decision.state.reason == reason


def test_missing_recovery_blocks_next_event_even_when_a_later_site_is_available():
    st = LargeRecoveryState(stage='recovery', event_id=1, start_ns=0,
        start_wall_ns=0, request_ns=1, release_ns=2)
    decision = proposal(st, config(2), sim_ns=10_000_000_002, lateral_m=.5, s_m=74.)
    assert decision.state.stage == 'aborted' and decision.state.reason == 'RECOVERY_NOT_CONFIRMED'


def test_transient_return_followed_by_drift_cannot_advance_the_event_count():
    st=LargeRecoveryState(stage='recovery',event_id=1,start_ns=0,start_wall_ns=0,
        request_ns=1,release_ns=2,confirmed_ns=5_000_000_002,
        last_sim_ns=9_950_000_002,last_wall_ns=10_950_000_002)
    decision=proposal(st,config(2),sim_ns=10_000_000_002,wall_ns=11_000_000_002,s_m=74.,lateral_m=.3)
    assert decision.state.stage=='aborted' and decision.state.reason=='RECOVERY_NOT_STABLE_AT_END'
    assert decision.state.completed_events==0


@pytest.mark.parametrize('count', [3, 5, 6, 7])
def test_event_budget_is_finite_and_all_events_are_separated(count):
    cfg=LargeRecoveryConfig(tuple(LargeRecoverySite('P'+str(i),40.+40*i,.2*(-1)**i) for i in range(count)),event_cap=count)
    state,rows=synthetic_run(cfg)
    events=large_recovery_events(rows)
    assert state.completed_events==count and state.event_id==count and state.stage=='complete'
    assert len(events)==count and all(e['recovery_confirmed'] for e in events)
    assert all(b['preparation_start_ns']>=a['end_publication_ns']+3_000_000_000 for a,b in zip(events,events[1:]))
    with pytest.raises(ValueError):LargeRecoveryConfig(cfg.sites,event_cap=8)


def test_clock_gaps_reset_entry_and_goal_holds_and_regression_fails():
    st = LargeRecoveryState(approach_seen=True, stable_since_ns=0,
                           last_sim_ns=1, last_wall_ns=1, last_progress_m=49.)
    assert proposal(st, s_m=50.).state.event_id == 0
    with pytest.raises(ValueError, match='PROGRESS_REGRESSION'):
        proposal(st, s_m=43.)
    with pytest.raises(ValueError, match='CLOCK_OR_OBSERVATION'):
        proposal(st, sim_ns=0)
    preparing = replace(st, stage='preparing', event_id=1, start_ns=0,
                        start_wall_ns=0, target_since_ns=1, last_progress_m=60.)
    assert proposal(preparing, s_m=60., lateral_m=.6).state.stage == 'preparing'


def test_random_plan_is_reproducible_and_retains_required_stop_site():
    pool = [LargeRecoverySite('S00', 118., .6), LargeRecoverySite('R01', 60., -.4),
            LargeRecoverySite('R02', 180., .2), LargeRecoverySite('R03', 235., -.6)]
    first = select_large_sites(pool, seed=14, event_cap=3, required_site_ids=['S00'])
    assert first == select_large_sites(pool, seed=14, event_cap=3, required_site_ids=['S00'])
    assert len(first.sites) == 3 and any(s.site_id == 'S00' for s in first.sites)
    assert all(b.start_s_m-a.start_s_m >= 40 for a,b in zip(first.sites, first.sites[1:]))
    with pytest.raises(ValueError, match='INSUFFICIENT'):
        select_large_sites(pool[:1], seed=1, event_cap=2)
    with pytest.raises(ValueError):
        LargeRecoveryConfig(tuple(pool[:2]), event_cap=2)  # Unsorted plan.
    with pytest.raises(ValueError):
        LargeRecoveryConfig((LargeRecoverySite('P', 60., .8),))


def test_preparation_commands_cannot_be_relabelled_as_teachers():
    _, rows = synthetic_run(config())
    tampered = deepcopy(rows)
    item = next(r for r in tampered if r['phase'] == 'recovery')
    item['large_recovery']['command_source'] = 'preparation'
    with pytest.raises(ValueError, match='PREPARATION_IN_RECOVERY|PREPARATION_IN_TEACHER'):
        collection_phase_windows(tampered)
    unconfirmed = deepcopy(rows)
    for r in unconfirmed:
        if r['phase'] == 'recovery':
            r['large_recovery']['lateral_error_m'] = .4
    assert not any(w.phase == 'recovery' for w in collection_phase_windows(unconfirmed))


def test_two_rviz_markers_are_measured_positions_with_explicit_preparation_labels():
    _, rows = synthetic_run()
    markers = DisturbanceLocations()
    for row in rows:
        markers.add(row)
    assert len(markers.events) == 2
    assert markers.events[1].label == 'P0 LEFT PREP 60cm'
    assert markers.events[2].label == 'P1 RIGHT PREP 60cm'
    assert markers.events[1].x_m >= 50 and markers.events[1].publication_sequence > 0


@pytest.mark.parametrize('role,limit', [('preparation_nominal', 150_000_000), ('preparation_trajectory', 1_500_000_000)])
def test_added_sources_use_existing_time_budgets(role, limit):
    values = dict(capture_ns=0, receipt_ns=limit, now_sim_ns=limit, now_wall_ns=limit)
    check_collection_input_time(role, **values)
    with pytest.raises(ValueError, match='STALE'):
        check_collection_input_time(role, **{**values, 'now_sim_ns':limit+1})
