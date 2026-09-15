from dataclasses import asdict, replace
import copy
import math

import numpy as np
import pytest

from aic_transfuser_lite.data.time_nominal_steering_guide_v1 import (
    POLICY, NominalSteeringGuide, guide_control, validate_guide_control_row,
)
from aic_transfuser_lite.data.time_random_steering_pulse_v1 import (
    SCHEMA, PulseSite, RandomPulseConfig, RandomPulseState, propose_random_pulse, random_pulse_events,
)
from aic_transfuser_lite.data.time_steering_pulse_v1 import (
    SteeringPulseConfig, SteeringPulseState, propose_steering_pulse,
)
from aic_transfuser_lite.data.time_recovery_collection_v1 import collection_phase_windows, recovery_teacher_mask


TEMPLATE = SteeringPulseConfig(10., .1, duration_s=1.5, plateau_s=1., start_window_m=2., goal_min_elapsed_s=1.25)


def test_curved_nominal_steering_is_interpolated_at_current_progress():
    values = np.array([[10., .1], [10.2, .2], [10.4, .3]])
    guide = NominalSteeringGuide(values)
    values[1, 1] = -.5
    assert guide.at(10.1) == pytest.approx(.15)
    assert guide.at(10.3) == pytest.approx(.25)
    with pytest.raises(ValueError, match='SUPPORT'):
        guide.at(9.9)
    with pytest.raises(ValueError):
        guide.values[0, 0] = 0.


@pytest.mark.parametrize('values', [[], [[10., 0., 0.]], [[10., 0.], [10.3, 0.]],
    [[10., 0.], [10., .1]], [[10., 0.], [10.1, float('nan')]], [[10., .6], [10.1, .1]]])
def test_invalid_steering_guide_cannot_be_used(values):
    with pytest.raises(ValueError, match='SHAPE_FINITE_GAPS_OR_BOUNDS'):
        NominalSteeringGuide(values)


@pytest.mark.parametrize('sign', [-1, 1])
def test_countersteering_cannot_cancel_full_plateau_and_release_returns_to_pp(sign):
    args = dict(pp_rad=-sign*.09, guide_rad=sign*.01, amplitude_rad=.1)
    full = guide_control(**args, pulse_rad=sign*.1, phase='hold')
    half = guide_control(**args, pulse_rad=sign*.05, phase='hold')
    released = guide_control(**args, pulse_rad=0., phase='recovery')
    assert full['requested_angle_rad'] == pytest.approx(sign*.11)
    assert full['guide_weight'] == 1.
    assert half['guide_weight'] == .5 and half['phase'] == 'hold'
    assert released['requested_angle_rad'] == args['pp_rad'] and released['guide_weight'] == 0.


def tick(st, t, lateral=.07, heading=math.radians(3.)):
    return propose_steering_pulse(TEMPLATE, st, sim_ns=round(t*1e9), wall_ns=round((t+10)*1e9),
        s_m=10.+1.27*t, speed_mps=1.27, lateral_m=lateral, heading_rad=heading)


def test_soft_goal_does_not_shorten_one_second_plateau_but_hard_limit_does():
    st = tick(SteeringPulseState(approach_seen=True), 0., 0., 0.).state
    for t in (.25, .5, .75, 1., 1.249):
        decision = tick(st, t)
        assert decision.perturbation_rad == pytest.approx(.1) and decision.state.stage == 'active'
        st = decision.state
    proposal = tick(st, 1.25)
    assert proposal.state.stage == 'releasing' and proposal.state.reason == 'STATE_GOAL'
    assert st.stage == 'active' and proposal == tick(st, 1.25)  # failed publish must not commit
    early = tick(replace(st, last_sim_ns=0, last_wall_ns=10_000_000_000), .5, heading=math.radians(4.))
    assert early.state.reason == 'HEADING_LIMIT' and early.state.stage == 'releasing'


def separated_record():
    cfg = RandomPulseConfig(915110, max_events=1, start_min_m=15., start_max_m=15.,
        sites=(PulseSite('S00', 15., 1),), control_policy=POLICY)
    st = RandomPulseState(); rows = []
    for i in range(390):
        t = i*.05
        d = propose_random_pulse(cfg, TEMPLATE, st, sim_ns=round(t*1e9), wall_ns=round((t+10)*1e9),
            s_m=10.+1.27*t, speed_mps=1.27, lateral_m=0., heading_rad=0., entry_clear=True)
        st = d.state
        data = guide_control(pp_rad=-.08, guide_rad=.02, pulse_rad=d.perturbation_rad,
                             amplitude_rad=.1, phase=d.phase)
        rows.append(dict(annotation_schema=SCHEMA, sim_ns=round(t*1e9), phase=d.phase,
            nominal_angle_rad=-.08, guide_control=data,
            publication=dict(sim_ns=round(t*1e9)+5_000_000, monotonic_ns=round((t+10)*1e9), sequence=i+1),
            target_speed_mps=5/3.6, speed_mps=1.27,
            pulse=dict(applied=True, state=asdict(st.pulse), requested_rad=d.perturbation_rad,
                       effective_rad=data['requested_angle_rad']+.08, lateral_error_m=0., heading_error_rad=0.),
            random_pulse=dict(config=asdict(cfg), state=asdict(st))))
    return rows


def test_event_parser_accepts_separated_profile_and_excludes_entire_return_ramp():
    rows = separated_record()
    event, = random_pulse_events(rows)
    assert event['recovery_confirmed']
    windows = collection_phase_windows(rows)
    ramp = next(r for r in reversed(rows) if r['phase'] == 'hold')
    assert ramp['guide_control']['guide_weight'] > 0.
    assert not recovery_teacher_mask(ramp['publication']['sim_ns'], windows).any()
    assert recovery_teacher_mask(event['zero_publication_ns']+150_000_000, windows).all()


@pytest.mark.parametrize('change', ['weight', 'phase', 'missing'])
def test_assisted_control_cannot_masquerade_as_pure_pp_recovery(change):
    row = copy.deepcopy(next(r for r in separated_record() if r['phase'] == 'recovery'))
    if change == 'missing':
        del row['guide_control']
    elif change == 'weight':
        row['guide_control']['guide_weight'] = .01
    else:
        row['guide_control']['phase'] = 'hold'
    with pytest.raises(ValueError, match='GUIDE_CONTROL_METADATA'):
        validate_guide_control_row(row)


def test_separated_template_requires_explicit_named_policy():
    cfg = RandomPulseConfig(1)
    with pytest.raises(ValueError, match='PRESERVES_PROVEN_PULSE'):
        propose_random_pulse(cfg, TEMPLATE, RandomPulseState(), sim_ns=0, wall_ns=0, s_m=10.,
                             speed_mps=1.27, lateral_m=0., heading_rad=0., entry_clear=True)
    with pytest.raises(ValueError, match='CONTROL_POLICY'):
        replace(cfg, control_policy=POLICY)


def test_official_pp_input_contract_survives_unperturbed_sharp_corner():
    from aic_transfuser_lite.data.time_recovery_collection_v1 import bounded_collection_command
    for phase in ('baseline', 'recovery'):
        command = guide_control(pp_rad=.64, guide_rad=.64, pulse_rad=0., amplitude_rad=.1, phase=phase)
        assert command['requested_angle_rad'] == .64 and command['guide_weight'] == 0.
        angle, _ = bounded_collection_command(command['requested_angle_rad'], 0., .5, .05)
        assert angle == .5
    with pytest.raises(ValueError, match='GUIDE_CONTROL_INPUT'):
        guide_control(pp_rad=.641, guide_rad=0., pulse_rad=0., amplitude_rad=.1, phase='baseline')
