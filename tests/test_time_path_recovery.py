"""Bounded runtime recovery admission and fresh-map checks, not real-car proof."""
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest

from aic_transfuser_lite.control.time_path_recovery import RetainedPathRecovery, RECOVERY_POLICY, recoverable_geometry
from aic_transfuser_lite.control.slam_mppi import AvoidancePlanner, FAST_POLICY, mppi_nominal_control
from aic_transfuser_lite.control.time_reference_v1 import TimePlan, TimedBodyPose
from aic_transfuser_lite.control.time_trial_v1 import validate_trial_config
from test_slam_mppi import command, packet, scene


def route(length=6., offset=0.):
    return np.column_stack((np.linspace(.1, length, 30)+offset, np.zeros(30)))


def enter(r, offset=0.):
    r.select(route(), np.zeros(3), 1_000_000_000, 1.)
    r.select(route(1.), np.array([offset,0,0]), 1_100_000_000, .5)
    return r.select(route(1.), np.array([offset,0,0]), 1_400_000_000, .5)


def test_enter_tracks_retained_world_path_and_hands_back_only_after_stable_model():
    r = RetainedPathRecovery()
    selected = enter(r, 1.)
    assert selected['mode'] == 'RECOVER' and selected['attempts'] == 1
    np.testing.assert_array_equal(selected['reference_world'], route())
    assert selected['reference_source_stamp_ns'] == 1_000_000_000
    for t in [1_500_000_000,1_700_000_000,1_900_000_000]:
        assert r.select(route(6., 1.), np.array([1.,0,0]), t, .5)['mode'] == 'RECOVER'
    out = r.select(route(6., 1.), np.array([1.,0,0]), 2_000_000_000, .5)
    assert out['mode'] == 'NOMINAL' and out['reason'] == 'RECOVERY_HAND_BACK'


def test_missing_source_explicit_stop_and_stale_reference_cannot_start_motion():
    r=RetainedPathRecovery(); enter(r)
    assert r.select(None, np.zeros(3), 1_500_000_000, 0.)['mode']=='STOP'
    assert r.select(route(), np.zeros(3), 1_600_000_000, 0., explicit_stop=True)['mode']=='STOP'
    r=RetainedPathRecovery()
    assert r.select(route(1.), np.zeros(3), 1_000_000_000, 0.)['mode']=='NOMINAL'
    assert r.select(route(1.), np.zeros(3), 1_300_000_000, 0.)['reason']=='NO_RECENT_RECOVERY_REFERENCE'


@pytest.mark.parametrize('budget', ['time','distance'])
def test_recovery_budget_stops_and_does_not_reset_on_short_prediction(budget):
    r=RetainedPathRecovery(); enter(r)
    t=7_400_000_000 if budget=='time' else 1_500_000_000
    pose=np.array([2.5,0,0]) if budget=='distance' else np.zeros(3)
    assert r.select(route(1.),pose,t,0.)['reason']=='RECOVERY_BUDGET_EXHAUSTED'
    assert r.select(route(1.),pose,t+100_000_000,0.)['mode']=='STOP'


def test_nan_large_backtracking_and_replayed_clock_are_not_recoverable():
    assert recoverable_geometry(route(.15), 'TIME_PATH_UNRESOLVED_EXCURSION')
    assert not recoverable_geometry(route(6.), 'TIME_PATH_BACKTRACK')
    assert not recoverable_geometry(np.full((30,2),np.nan), 'TIME_PATH_BACKTRACK')
    r=RetainedPathRecovery(); enter(r)
    with pytest.raises(ValueError,match='CLOCK'):
        r.select(route(),np.zeros(3),1_400_000_000,0.)
    with pytest.raises(ValueError,match='SHAPE_FINITE'):
        r.select(np.zeros((30,3)),np.zeros(3),1_500_000_000,0.)


def recovery_packet():
    _, values, origin=scene(False)
    g=SimpleNamespace(values=values,origin=origin/.2,resolution_m=.2)
    planner=AvoidancePlanner(policy=FAST_POLICY,recovery_enabled=True)
    for stamp, fresh in [(1_000_000_000,route()),(1_100_000_000,route(1.)),(1_400_000_000,route(1.))]:
        planner.prepare_reference(fresh,np.zeros(3),stamp,0.)
        p=packet(False,stamp)
        p['mppi']=planner.update(p,fresh,g)
    return p,planner,g


def test_planned_recovery_overrides_short_nominal_only_with_fresh_free_map():
    p,planner,g=recovery_packet()
    assert p['mppi']['mode']=='RECOVER' and p['mppi']['target_speed_mps']<=3/3.6
    args=dict(now_sim_ns=1_400_000_000,recovery_enabled=True,mppi_policy=FAST_POLICY,
              vehicle_model_policy='awsim_understeer_20kmh_trial_v1',speed_mps=0.,target_mps=0.,acceleration_mps2=-1.)
    out=command(p,**args)
    assert out['mode']=='RECOVER' and out['acceleration_mps2']>0
    assert command(p,**dict(args,recovery_enabled=False))['mode']=='STOP'
    assert command(p,**dict(args,now_sim_ns=1_800_000_000))['mode']=='STOP'
    bad=deepcopy(p);bad['path_blocked']=True;bad['nearest_path_obstacle_m']=2.
    assert command(bad,**args)['mode']=='STOP'
    # Newly unknown space invalidates the same retained reference at planning.
    g.values[:,:]=-1
    planner.prepare_reference(route(1.),np.zeros(3),1_500_000_000,0.)
    stopped=planner.update(packet(False,1_500_000_000),route(1.),g)
    assert stopped['mode']=='STOP' and stopped['reason']=='MPPI_NO_FEASIBLE_PATH'


def test_config_and_small_jitter_require_explicit_recovery_profile():
    c=json.loads(Path('configs/control/time_path_slam_mppi_20_15_15_recovery.json').read_text())
    validate_trial_config(c)
    with pytest.raises(ValueError,match='RECOVERY'):
        validate_trial_config(dict(c,slam_mppi_policy='off'))
    pose=TimedBodyPose(1_000_000_000,'sim','0','time_wheel_odom','base_link',0.,0.,0.)
    jitter=np.column_stack((.1*np.sin(np.linspace(0.,np.pi,30)),np.zeros(30)))
    plan=TimePlan('short',pose,jitter)
    args=dict(mppi_policy=FAST_POLICY,speed_mps=0.,rear_axle_offset_m=(.001,0.),
              speed_policy='curvature_time_preview_20kmh_v1',lookahead_policy='velocity_time_preview_v1',
              vehicle_model_policy='awsim_understeer_20kmh_trial_v1',speed_cap_mps=20/3.6,corner_max_speed_mps=15/3.6)
    with pytest.raises(ValueError,match='UNRESOLVED_EXCURSION'):
        mppi_nominal_control(plan,pose,**args)
    admitted=mppi_nominal_control(plan,pose,recovery_enabled=True,**args)
    assert admitted['recovery_only'] and admitted['nominal_tracking_unavailable']=='TIME_PATH_UNRESOLVED_EXCURSION'


def test_attempt_budget_is_total_and_unknown_policy_is_rejected():
    r=RetainedPathRecovery();enter(r)
    stamp=1_500_000_000
    for _ in range(2):
        for i in range(6):
            result=r.select(route(),np.zeros(3),stamp,0.);stamp+=100_000_000
        assert result['mode']=='NOMINAL'
        for i in range(4):
            result=r.select(route(1.),np.zeros(3),stamp,0.);stamp+=100_000_000
    assert result['reason']=='RECOVERY_ATTEMPT_LIMIT' and r.attempts==2


def test_recovery_only_admission_cannot_accept_nominal_or_avoid_mode():
    p,_,_=recovery_packet()
    for mode in ['NOMINAL','AVOID']:
        q=deepcopy(p);q['mppi']['mode']=mode
        out=command(q,now_sim_ns=1_400_000_000,mppi_policy=FAST_POLICY,recovery_enabled=True,
                    recovery_only=True,vehicle_model_policy='awsim_understeer_20kmh_trial_v1')
        assert out['mode']=='STOP' and out['reason']=='MPPI_REJECTED:RECOVERY_PLAN_REQUIRED'


def test_new_three_metre_reference_replaces_consumed_four_metre_reference():
    r=RetainedPathRecovery()
    r.select(route(4.),np.zeros(3),1_000_000_000,1.)
    pose=np.array([3.,0,0])
    r.select(route(3.2,3.),pose,1_100_000_000,1.)
    r.select(route(1.,3.),pose,1_200_000_000,0.)
    out=r.select(route(1.,3.),pose,1_500_000_000,0.)
    assert out['mode']=='RECOVER'
    assert out['reference_source_stamp_ns']==1_100_000_000
    assert out['retained_remaining_m']>3.


def test_each_recovery_attempt_resets_sampling_side():
    _,planner,_=recovery_packet()
    assert planner.recovery_decision['reason']=='RECOVERY_ENTER'
    # End the first attempt with a healthy model, then enter a second episode.
    planner.recovery_solver.side=1
    for i in range(6):
        planner.prepare_reference(route(),np.zeros(3),1_500_000_000+i*100_000_000,0.)
    for i in range(4):
        planner.prepare_reference(route(1.),np.zeros(3),2_100_000_000+i*100_000_000,0.)
    assert planner.recovery_decision['reason']=='RECOVERY_ENTER'
    assert planner.recovery_solver.side==0
