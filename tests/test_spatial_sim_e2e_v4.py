from pathlib import Path
from dataclasses import replace
import numpy as np
import pytest
import yaml

from aic_transfuser_lite.control.constrained_reference_v4 import constrained_reference, reference_to_world
from aic_transfuser_lite.control.spatial_tracking_contracts_v4 import SpatialPathCandidate
from aic_transfuser_lite.control.spatial_speed_profile_v4 import rolling_horizon, plan
from aic_transfuser_lite.control.sim_dispatch_v4 import SnapshotKey, send_rejection, AckermannDispatch, sent_history_sample


def cfg():
    return yaml.safe_load((Path(__file__).parents[1]/'configs/control/spatial_sim_e2e_v4.yaml').read_text())


def raw_curve(curvature=0.):
    s=np.arange(1,21)/10
    xy=np.c_[s,np.zeros(20)] if curvature==0 else np.c_[np.sin(s*curvature)/curvature,(1-np.cos(s*curvature))/curvature]
    return SpatialPathCandidate(xy.astype('float32'),s,'SYNTHETIC_NOT_SIM')


def test_raw_identity_order_bound_and_initial_steering():
    c=raw_curve(.15); before=c.raw_xy.tobytes()
    p=constrained_reference(c,cfg(),np.zeros(5))
    assert p.reason is None and c.raw_xy.tobytes()==before and len(before)==160
    assert p.diagnostics['maximum_deviation_bound_m']<=.1
    assert np.all(np.diff(p.diagnostics['raw_correspondence_s_m'])>0)
    assert p.diagnostics['steering_knots_rad'][0]==0
    assert p.diagnostics['initial_connection_length_m']>0
    assert p.diagnostics['clearance_verified'] is False
    assert p.diagnostics['runtime_permission'] is False


@pytest.mark.parametrize('kind',['nan','inf','short','cross','far','steep','tail','duplicate'])
def test_rejections_and_frozen_tail(kind):
    c=raw_curve(); xy=c.raw_xy.copy()
    if kind=='nan': xy[2,1]=np.nan
    if kind=='inf': xy[2,1]=np.inf
    if kind=='short': xy*=.01
    if kind=='cross': xy[8:12]=np.array([[.7,.2],[.7,-.2],[.8,-.2],[.8,.2]])
    if kind=='far': xy[:,1]+=1.
    if kind=='steep': xy[:,1]=xy[:,0]
    if kind=='duplicate': xy[3]=xy[2]
    if kind=='tail': xy[-1]=xy[-2]+np.array([-.03,.1])
    candidate=SpatialPathCandidate(xy,c.nominal_s,'synthetic-'+kind)
    before=xy.tobytes(); p=constrained_reference(candidate,cfg(),np.zeros(5))
    assert xy.tobytes()==before
    if kind=='tail':
        assert p.diagnostics['unused_tail_indices']==[19]
    else:
        assert p.reason is not None


def test_no_deviation_relaxation():
    config=cfg(); config['reference_max_deviation_m']=.101
    assert constrained_reference(raw_curve(),config,np.zeros(5)).reason=='DEVIATION_POLICY'


def test_world_transform_epoch_and_rear_offset():
    p=constrained_reference(raw_curve(),cfg(),np.array([-.03,0,0,0,0]))
    assert p.reason is None
    world=reference_to_world(p,np.array([10,20,np.pi/2]),observation_epoch='1',control_epoch='1')
    assert np.allclose(world.world_xy[0],[10,19.97])
    with pytest.raises(ValueError):
        reference_to_world(p,np.zeros(3),observation_epoch='1',control_epoch='2')


def test_rolling_launch_update_and_hold_cap():
    config=cfg(); p=constrained_reference(raw_curve(),config,np.zeros(5))
    first=rolling_horizon(p,config,progress_s=0,current_v=0,previous_a=0,delay_s=.1,permission='RUN',safety_cap_mps=.3)
    assert first['speed'][0]>0 and first['initial_v_mps']==0
    updated=rolling_horizon(p,config,progress_s=.05,current_v=.18,previous_a=.1,delay_s=.1,permission='RUN',safety_cap_mps=.3)
    assert updated['speed'][0]>=.17 and updated['initial_v_mps']==.18
    hold=rolling_horizon(p,config,progress_s=.05,current_v=.18,previous_a=0,delay_s=.1,permission='HOLD',safety_cap_mps=.3)
    assert hold['caps']['final']==0 and hold['speed'][0]>.03
    assert np.all(plan(p,config,'HOLD').caps['final_spatial_cap']==0)
    end=rolling_horizon(p,config,progress_s=p.actual_s[-1],current_v=.2,previous_a=.1,delay_s=.5,permission='RUN',safety_cap_mps=.3)
    assert end['reason']=='STOPPING_DISTANCE_INSUFFICIENT'
    assert end['stop_target_overshoot_m']>0 and end['usable_end_overshoot_m']==0


def gate_args():
    return dict(now_monotonic_ns=10,now_sim_ns=5,max_age_ns=10,isolation_verified=True,interface_verified=True,
                frame_verified=True,clearance_verified=True,watchdog_healthy=True,logger_healthy=True,
                communication_healthy=True,solver_accepted=True,enabled=True)


@pytest.mark.parametrize('field',['isolation_verified','interface_verified','frame_verified','clearance_verified',
                                'watchdog_healthy','logger_healthy','communication_healthy','solver_accepted','enabled'])
def test_fail_closed_no_dispatch_on_bad_evidence(field):
    k=SnapshotKey('1','i','p','s',0,100); args=gate_args(); args[field]=False
    assert send_rejection(k,k,**args) is not None


def test_expired_solver_state_path_epoch():
    k=SnapshotKey('1','i','p','s',0,100); args=gate_args()
    assert send_rejection(k,k,**args) is None
    for changed in [replace(k,epoch='2'),replace(k,path_id='new'),replace(k,state_id='new')]:
        assert send_rejection(k,changed,**args)=='STALE_SNAPSHOT_OR_RESET'
    args['now_monotonic_ns']=100
    assert send_rejection(k,k,**args)=='EXPIRED_SOLVER_RESULT'
    args['now_monotonic_ns']=10; args['now_sim_ns']=-1
    assert send_rejection(k,k,**args)=='STALE_STATE_OR_CLOCK_RESET'


def test_ackermann_units_once_only_and_causal_sent_history():
    dispatch=AckermannDispatch(cfg())
    request=dispatch.request('op1',np.array([.2,.4]),measured_delta_rad=.1,control_dt_s=.1,target_speed_mps=.2)
    assert request['steering_tire_angle_rad']==pytest.approx(.14)
    assert request['acceleration_mps2']==.2 and request['applied'] is None
    with pytest.raises(ValueError): sent_history_sample(request,current_observation_ns=20,available_cutoff_ns=30)
    sent=dispatch.acknowledge_sent(request,sent_sim_ns=10,sent_monotonic_ns=11)
    assert sent_history_sample(sent,current_observation_ns=20,available_cutoff_ns=30)==pytest.approx((.14,.2,.2))
    for t,cutoff in [(10,30),(9,30),(20,10)]:
        with pytest.raises(ValueError): sent_history_sample(sent,current_observation_ns=t,available_cutoff_ns=cutoff)
    with pytest.raises(ValueError):
        dispatch.request('op1',np.zeros(2),measured_delta_rad=.14,control_dt_s=.1,target_speed_mps=0)
