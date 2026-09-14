from dataclasses import asdict, replace
import math

import numpy as np
import pytest

from aic_transfuser_lite.data.time_steering_pulse_v1 import SteeringPulseConfig
from aic_transfuser_lite.data.time_random_steering_pulse_v1 import (
    SCHEMA, RandomPulseConfig, RandomPulseState, propose_random_pulse, random_schedule,
    random_pulse_events, event_at, validate_random_guide,
)
from aic_transfuser_lite.data.time_recovery_collection_v1 import collection_phase_windows, recovery_teacher_mask


TEMPLATE = SteeringPulseConfig(88., .1, duration_s=2., plateau_s=1.5)


def tick(config, state, t, **overrides):
    values = dict(sim_ns=round(t*1e9), wall_ns=round((t+10)*1e9), s_m=55.+1.27*t,
                  speed_mps=1.27, lateral_m=0., heading_rad=0., entry_clear=True)
    return propose_random_pulse(config, TEMPLATE, state, **{**values, **overrides})


def recorded_run(*, recover=True, seed=915062):
    config = RandomPulseConfig(seed)
    state = RandomPulseState(); rows = []
    schedule = random_schedule(config)
    for step in range(1501):
        t = step*.05; lateral = 0.; heading = 0.
        if state.stage == 'pulse':
            sign = schedule[state.event_id-1][0]
            age = (round(t*1e9)-state.pulse.start_ns)/1e9
            if age >= .6:
                lateral, heading = sign*.06, sign*.04
            if state.pulse.zero_ns is not None and round(t*1e9)-state.pulse.zero_ns >= 2_000_000_000:
                lateral, heading = (0., 0.) if recover else (sign*.15, sign*.04)
        decision = tick(config, state, t, lateral_m=lateral, heading_rad=heading)
        state = decision.state
        rows.append(dict(annotation_schema=SCHEMA, sim_ns=round(t*1e9), phase=decision.phase,
            publication=dict(sim_ns=round(t*1e9)+5_000_000, monotonic_ns=round((t+10)*1e9), sequence=step+1),
            target_speed_mps=5/3.6, speed_mps=1.27,
            pulse=dict(applied=True, state=asdict(state.pulse), requested_rad=decision.perturbation_rad,
                       effective_rad=decision.perturbation_rad, lateral_error_m=lateral, heading_error_rad=heading),
            random_pulse=dict(config=asdict(config), state=asdict(state))))
    return rows, state


def test_seeded_plan_is_reproducible_bounded_and_contains_both_sides():
    a = random_schedule(RandomPulseConfig(915062))
    assert a == random_schedule(RandomPulseConfig(915062))
    assert a != random_schedule(RandomPulseConfig(915063))
    assert len(a) == 3 and {s for s, _ in a} == {-1, 1}
    assert all(.5 <= delay <= 1.5 for _, delay in a)


def test_fractional_measured_guide_endpoints_preserve_full_start_and_future_margins():
    guide = [[60.051293691840584,0.,0.],[134.9410027178904,0.,0.]]
    validate_random_guide(RandomPulseConfig(1), TEMPLATE, guide)
    with pytest.raises(ValueError,match='COVERAGE'):
        validate_random_guide(RandomPulseConfig(1,start_min_m=65.), TEMPLATE, guide)
    for invalid in ([[60.,0.],[135.,0.]], [[60.,0.,0.],[132.,0.,0.]],
                    [[60.,0.,0.],[60.,0.,0.]], [[60.,0.,float('nan')],[135.,0.,0.]]):
        with pytest.raises(ValueError,match='COVERAGE'):
            validate_random_guide(RandomPulseConfig(1), TEMPLATE, invalid)


@pytest.mark.parametrize('kwargs', [dict(seed=True), dict(seed=-1), dict(max_events=4), dict(start_max_m=135.),
    dict(jitter_min_s=0.), dict(stable_hold_s=.1), dict(future_tail_s=2.9), dict(clearance_margin_m=0.),
    dict(start_min_m=float('nan'))])
def test_invalid_configuration_is_rejected(kwargs):
    with pytest.raises(ValueError):
        RandomPulseConfig(**{**dict(seed=1), **kwargs})


def test_three_events_have_actual_publication_boundaries_and_no_future_overlap():
    rows, state = recorded_run()
    events = random_pulse_events(rows)
    assert state.stage == 'complete' and state.event_id == state.completed_events == len(events) == 3
    windows = collection_phase_windows(rows)
    for e in events:
        assert e['recovery_confirmed']
        assert e['zero_publication_ns'] > e['start_publication_ns']
        assert event_at(e['zero_publication_ns']+150_000_000, events)['event_id'] == e['event_id']
        assert recovery_teacher_mask(e['zero_publication_ns']+150_000_000, windows).all()
        assert e['zero_publication_ns'] % 50_000_000 == 5_000_000
    for a, b in zip(events, events[1:]):
        assert b['start_publication_ns'] >= a['end_publication_ns']+3_000_000_000
        assert not recovery_teacher_mask(b['start_publication_ns']-2_000_000_000, windows).all()
    assert event_at(events[-1]['end_publication_ns'], events) is None


def test_unconfirmed_recovery_stops_repetition_and_is_not_teacher():
    rows, state = recorded_run(recover=False)
    assert state.stage == 'aborted' and state.event_id == 1 and state.completed_events == 0
    events = random_pulse_events(rows)
    assert len(events) == 1 and not events[0]['recovery_confirmed']
    windows = collection_phase_windows(rows)
    assert not any(w.phase == 'recovery' for w in windows)
    assert not recovery_teacher_mask(events[0]['zero_publication_ns']+200_000_000, windows).any()


def test_rejected_proposals_do_not_consume_rng_or_events():
    config = RandomPulseConfig(12)
    state = tick(config, RandomPulseState(), 0.).state
    for i in range(1, 201):
        proposal = tick(config, state, i*.05)
        if proposal.state.event_id:
            rejected = proposal
            break
        state = proposal.state
    assert state.event_id == 0
    retried = tick(config, state, i*.05)
    assert rejected == retried and retried.state.event_id == 1


def test_missing_clearance_or_steering_headroom_prevents_entry():
    config = RandomPulseConfig(12); state = RandomPulseState()
    for i in range(1101):
        decision = tick(config, state, i*.05, entry_clear=False)
        state = decision.state
        assert decision.perturbation_rad == 0.
    assert state.event_id == 0 and state.reason == 'START_REGION_FINISHED'


def test_spawn_at_seam_does_not_consume_events_and_clock_regression_fails():
    config = RandomPulseConfig(12)
    state = tick(config, RandomPulseState(), 0., s_m=349.).state
    assert not state.approach_seen and state.stage == 'waiting'
    state = tick(config, state, 1., s_m=0.).state
    assert state.approach_seen
    with pytest.raises(ValueError, match='CLOCK'):
        tick(config, state, .5)
    with pytest.raises(ValueError, match='CLOCK'):
        tick(config, state, 2., heading_rad=float('nan'))


def test_same_stamp_conflict_and_missing_telemetry_cannot_form_labels():
    rows, _ = recorded_run()
    events = random_pulse_events(rows)
    stamp = events[0]['zero_publication_ns']+500_000_000
    selected = next(i for i, r in enumerate(rows) if r['publication']['sim_ns'] == stamp)
    rows[selected]['publication']['sim_ns'] = rows[selected-1]['publication']['sim_ns']
    rows[selected]['phase'] = 'invalid'
    windows = collection_phase_windows(rows)
    assert not recovery_teacher_mask(stamp-100_000_000, windows).all()
    rows, _ = recorded_run()
    # Keep publication sequence honest after a dropped timestamp span.
    rows = [r for r in rows if not stamp <= r['publication']['sim_ns'] < stamp+500_000_000]
    for i, r in enumerate(rows):
        r['publication']['sequence'] = i+1
    assert not recovery_teacher_mask(stamp-100_000_000, collection_phase_windows(rows)).all()


@pytest.mark.parametrize('kind', ['seed', 'order', 'amplitude', 'early_next'])
def test_corrupt_event_metadata_or_teacher_commands_fail_closed(kind):
    rows, _ = recorded_run()
    first = next(r for r in rows if r['phase'] == 'recovery')
    if kind == 'seed':
        first['random_pulse']['config']['seed'] += 1
    elif kind == 'order':
        first['random_pulse']['state']['event_id'] = 3
    elif kind == 'amplitude':
        first['pulse']['effective_rad'] = .01
    else:
        previous_end = random_pulse_events(rows)[0]['end_publication_ns']
        second = next(r for r in rows if r['phase'] == 'hold' and r['random_pulse']['state']['event_id'] == 2)
        second['publication']['sim_ns'] = previous_end+1_000_000_000
    with pytest.raises(ValueError):
        collection_phase_windows(rows)


def test_whole_future_mask_shape_units_and_existing_single_pulse_compatibility():
    rows, _ = recorded_run()
    mask = recovery_teacher_mask(random_pulse_events(rows)[0]['zero_publication_ns']+200_000_000,
                                 collection_phase_windows(rows))
    assert mask.shape == (30,) and mask.dtype == np.bool_ and mask.all()
    with pytest.raises(ValueError):
        tick(RandomPulseConfig(1), RandomPulseState(), 0., entry_clear=1)
    with pytest.raises(ValueError, match='PRESERVES'):
        propose_random_pulse(RandomPulseConfig(1), replace(TEMPLATE, amplitude_rad=.05), RandomPulseState(),
            sim_ns=0, wall_ns=0, s_m=0., speed_mps=1.27, lateral_m=0., heading_rad=0., entry_clear=True)
