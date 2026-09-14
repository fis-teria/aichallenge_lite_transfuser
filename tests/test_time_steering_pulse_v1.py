import math

import numpy as np
import pytest

from aic_transfuser_lite.data.time_steering_pulse_v1 import (
    SteeringPulseConfig, SteeringPulseState, propose_steering_pulse, nominal_recovery_errors,
)


def tick(config, state, t, **kwargs):
    values = dict(sim_ns=round(t*1e9), wall_ns=round((10+t)*1e9), s_m=88.+t,
                  speed_mps=1.27, lateral_m=0., heading_rad=0.)
    return propose_steering_pulse(config, state, **{**values, **kwargs})


def approached():
    return SteeringPulseState(approach_seen=True)


@pytest.mark.parametrize('amplitude', [-.05, .05])
def test_single_smooth_pulse_cannot_extend_or_restart(amplitude):
    config = SteeringPulseConfig(88., amplitude)
    state = approached(); values=[]
    for t in np.arange(0., 12.01, .05):
        decision=tick(config,state,float(t));state=decision.state;values.append(decision.perturbation_rad)
        if t >= 1.:
            assert decision.perturbation_rad == 0.
    assert values[0] == 0. and max(map(abs,values)) == pytest.approx(.05)
    assert max(abs(b-a)/.05 for a,b in zip(values,values[1:])) < .16
    assert state.stage == 'complete'
    assert tick(config,state,13.,s_m=88.).perturbation_rad == 0.


def test_rejected_proposal_does_not_commit_start_or_release():
    config=SteeringPulseConfig(88.,.05); initial=approached()
    first=tick(config,initial,0.)
    retry=tick(config,initial,.05)
    assert initial.start_ns is None and retry.state.start_ns == 50_000_000
    active=tick(config,first.state,.25)
    rejected=tick(config,active.state,.3,lateral_m=.1,heading_rad=.05)
    assert rejected.state.stage == 'releasing'
    retried=tick(config,active.state,.3)
    assert retried.state.stage == 'active'


@pytest.mark.parametrize('changes,reason', [
    ({'lateral_m':.25},'LATERAL_LIMIT'), ({'heading_rad':-.08},'HEADING_LIMIT'),
    ({'s_m':93.},'PROGRESS_LIMIT'), ({'speed_mps':1.5},'SPEED_LIMIT'),
    ({'lateral_m':.06,'heading_rad':.04},'STATE_GOAL'),
])
def test_any_bound_requests_release_without_waiting_for_goal(changes,reason):
    config=SteeringPulseConfig(88.,.05); state=tick(config,approached(),0.).state
    decision=tick(config,state,.4,**changes)
    assert decision.state.stage=='releasing' and decision.state.reason==reason and decision.phase=='hold'
    final=tick(config,decision.state,.6)
    assert final.perturbation_rad==0. and final.phase=='recovery'


def test_no_start_with_wrong_entry_state_and_no_late_retry():
    config=SteeringPulseConfig(88.,.05)
    decision=tick(config,approached(),0.,heading_rad=.1)
    assert decision.state.stage=='waiting'
    decision=tick(config,decision.state,2.)
    assert decision.state.stage=='skipped'
    assert tick(config,decision.state,3.,s_m=88.).state.stage=='skipped'


def test_wall_cap_and_clock_reset_are_explicit():
    config=SteeringPulseConfig(88.,.05); state=tick(config,approached(),0.).state
    assert tick(config,state,.1,wall_ns=12_000_000_000).phase=='recovery'
    later=tick(config,state,.2).state
    with pytest.raises(ValueError,match='CLOCK'):
        tick(config,later,.1)
    with pytest.raises(ValueError,match='CLOCK'):
        tick(config,state,.1,lateral_m=float('nan'))


@pytest.mark.parametrize('kwargs', [{'amplitude_rad':.11},{'duration_s':3.},{'max_heading_rad':.2},
    {'max_lateral_m':1.},{'start_s_m':float('inf')},{'amplitude_rad':True},
    {'plateau_s':float('nan')},{'plateau_s':-.1},{'duration_s':2.,'plateau_s':1.6}])
def test_unsafe_or_non_si_configuration_is_rejected(kwargs):
    with pytest.raises(ValueError,match='CONFIG'):
        SteeringPulseConfig(**{'start_s_m':88.,'amplitude_rad':.05,**kwargs})


def test_nominal_guide_interpolation_wraps_heading_and_rejects_extrapolation():
    guide=np.array([[80.,.2,math.pi-.1],[90.,.3,-math.pi+.1]])
    lateral,heading=nominal_recovery_errors(guide,s_m=85.,offset_m=.3,yaw_rad=-math.pi+.05)
    assert lateral==pytest.approx(.05) and heading==pytest.approx(.05)
    with pytest.raises(ValueError,match='GUIDE'):
        nominal_recovery_errors(guide,s_m=95.,offset_m=0.,yaw_rad=0.)
    with pytest.raises(ValueError,match='GUIDE'):
        nominal_recovery_errors(guide[::-1],s_m=85.,offset_m=0.,yaw_rad=0.)


def test_initial_lap_seam_does_not_skip_the_first_real_approach():
    config=SteeringPulseConfig(88.,.05)
    spawn=tick(config,SteeringPulseState(),0.,s_m=349.)
    assert spawn.state.stage=='waiting' and not spawn.state.approach_seen
    approach=tick(config,spawn.state,1.,s_m=5.)
    assert approach.state.approach_seen
    assert tick(config,approach.state,70.,s_m=88.).state.stage=='active'


@pytest.mark.parametrize('amplitude',[-.08,.08])
def test_plateau_changes_area_without_extending_duration_or_amplitude(amplitude):
    config=SteeringPulseConfig(88.,amplitude,duration_s=2.,plateau_s=1.5)
    state=approached(); values=[]; times=np.linspace(0.,2.,2001)
    for t in times:
        decision=tick(config,state,float(t)); state=decision.state
        values.append(decision.perturbation_rad)
        if .25<=t<=1.75:
            assert decision.perturbation_rad==pytest.approx(amplitude)
    assert values[0]==values[-1]==0. and state.stage=='recovery'
    assert max(map(abs,values))<=abs(amplitude)
    assert max(abs(b-a)/.001 for a,b in zip(values,values[1:])) < .503
    assert np.sum((np.asarray(values[:-1])+np.asarray(values[1:]))*.0005)==pytest.approx(amplitude*1.75)
    assert tick(config,state,2.1).perturbation_rad==0.


@pytest.mark.parametrize('amplitude',[-.08,.08])
def test_plateau_state_goal_still_releases_before_total_deadline(amplitude):
    config=SteeringPulseConfig(88.,amplitude,duration_s=2.,plateau_s=1.5)
    state=tick(config,approached(),0.).state
    active=tick(config,state,.5); sign=math.copysign(1.,amplitude)
    release=tick(config,active.state,.8,lateral_m=sign*.06,heading_rad=sign*.04)
    assert release.state.stage=='releasing' and release.state.reason=='STATE_GOAL'
    assert release.perturbation_rad==pytest.approx(amplitude)
    halfway=tick(config,release.state,.875)
    assert halfway.perturbation_rad==pytest.approx(amplitude*.5)
    assert halfway.phase=='hold'
    end=tick(config,halfway.state,1.)
    assert end.phase=='recovery' and end.perturbation_rad==0.
