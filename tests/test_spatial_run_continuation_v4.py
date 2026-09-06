"""Bounded synthetic regressions; no ROS, checkpoint, sensors or asset reads."""
from dataclasses import replace
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import numpy as np
import pytest
import yaml
from aic_transfuser_lite.runtime.spatial_input_v4 import SpatialInputV4, GridObservation, Stamp, PassiveCommand
from aic_transfuser_lite.runtime.spatial_sim_adapter_v4 import Sample, await_control_join, stopped_forward_initial_speed
from aic_transfuser_lite.control.sim_dispatch_v4 import SimControlSchedule, AckermannDispatch, StopObservation
from aic_transfuser_lite.control.spatial_sim_guard_v4 import scene_aabb_evidence

ROOT=Path(__file__).parents[1]
spec=importlib.util.spec_from_file_location('host_continuation',ROOT/'tools/spatial_dev_host_v4.py')
host=importlib.util.module_from_spec(spec);spec.loader.exec_module(host)

def cfg(): return yaml.safe_load((ROOT/'configs/control/spatial_sim_e2e_v4.yaml').read_text())
def observation(ns,grid,epoch='0'):
    t=Stamp(ns,ns,ns,epoch=epoch)
    return GridObservation(grid,t,t,t,np.zeros((8,8,3),np.uint8),np.ones(750), (0.,0.,0.,0.))
def builder():return SpatialInputV4(command_binding_known=True,command_policy='SIM_ONLY_POLICY_CHANGED',history_policy='SIM_GRID_MISSING_V2')

def test_105ms_grid_missing_does_not_reset_receipts_or_fake_slots():
    b=builder();rejected=0
    for ns in range(0,2_100_000_001,105_000_000):
        grid=round(ns/100_000_000)*100_000_000
        if abs(ns-grid)>40_000_000:rejected+=1;continue
        for cmd_ns in range(max(0,ns-105_000_000),ns+1,25_000_000):
            b.add_command(PassiveCommand(Stamp(cmd_ns,cmd_ns,cmd_ns),0.,0.,0.,source='sim_sent'))
        b.append(observation(ns,grid),ns)
    batch,proof=b.build(2_100_000_000)
    assert rejected>0 and b.reset_count==0 and proof['stable_grid_history']
    assert len(proof['slots'])==11 and any(not s['valid'] for s in proof['slots'])
    assert all(s['source_camera_ns'] is None for s in proof['slots'] if not s['valid'])
    assert len(b.commands)>0 and batch.command_mask.any()
    assert batch.image.shape==(1,4,3,224,384) and batch.command_history.shape==(1,10,3)
    assert not all(proof['ego_masks'])

@pytest.mark.parametrize('kind',['true_gap','epoch','regression'])
def test_real_reset_invalidates_both_streams(kind):
    b=builder();b.append(observation(100_000_000,100_000_000),100_000_000)
    b.add_command(PassiveCommand(Stamp(50_000_000,50_000_000,50_000_000),0.,0.,0.,source='sim_sent'))
    ns=1_200_000_000 if kind=='true_gap' else 50_000_000 if kind=='regression' else 200_000_000
    b.append(observation(ns,ns,epoch='1' if kind=='epoch' else '0'),ns)
    assert b.reset_count==1 and not b.commands

def test_future_or_late_command_does_not_erase_eligible_past():
    b=builder();base=Stamp(100_000_000,100,100)
    past=PassiveCommand(base,0.,.2,0.,source='sim_sent')
    b.add_command(past)
    b.add_command(replace(past,stamp=replace(base,header_ns=200_000_000)))
    b.add_command(replace(past,stamp=replace(base,header_ns=110_000_000,available_ns=500)))
    chosen,record=b._command_for(replace(base,header_ns=120_000_000),replace(base,header_ns=130_000_000),200)
    assert chosen and record is past
    assert b._command_for(replace(base,header_ns=170_000_000),replace(base,header_ns=180_000_000),200)[0] is None

def control_bundle(right=False,epoch='0'):
    stamps=[100_000_000,150_000_000] if right else [100_000_000]
    d={role:[Sample(ns,ns,[0.,0.,0.] if role in ('pose','velocity') else [0.],role,epoch) for ns in stamps]
       for role in ('pose','velocity','steering')}
    return dict(d,epoch=epoch,cutoff_ns=150_000_000,previous_acceleration=0.,camera=['ORIGINAL'])

def test_late_pose_bracket_does_not_rebuild_input_or_extend_expiry():
    original=control_bundle();update=control_bundle(True);update['camera']=['FORBIDDEN_LATE_IMAGE']
    times=iter([160_000_000,170_000_000,180_000_000])
    after,joined=await_control_join(original,125_000_000,125_000_000,200_000_000,
                                   lambda:update,lambda:False,clock=lambda:next(times))
    assert original['camera']==after['camera']==['ORIGINAL'] and len(original['pose'])==1
    assert joined['pose_provenance']['source_stamps_ns']==[100_000_000,150_000_000]

@pytest.mark.parametrize('mode',['expired','reset','stop'])
def test_pose_join_rejects_without_second_forward(mode):
    times=iter([160_000_000,210_000_000])
    update=control_bundle(True,epoch='1' if mode=='reset' else '0')
    with pytest.raises(ValueError,match='POSE_JOIN'):
        await_control_join(control_bundle(),125_000_000,125_000_000,200_000_000,
            lambda:update,lambda:mode=='stop',clock=lambda:next(times))

def result(obs=100_000_000,op='p'):
    return dict(epoch='0',operation_id=op,observed_sim_ns=obs,state_source_ns=100_000_000,
        deadline_monotonic_ns=300_000_000,current_state=np.zeros(5),first_control=[.02,0.],
        solver_accepted=True,motion_rejection=None,input_id='OLD_CAMERA')
def arguments():return dict(epoch='0',now_ns=150_000_000,sim_ns=150_000_000,state=np.zeros(5),state_ns=150_000_000)

def test_same_tick_wait_and_new_camera_does_not_invalidate_result():
    s=SimControlSchedule(cfg());s.reset('0');s.sent(100_000_000,0.,operation_id='HOLD')
    assert not s.due(100_000_000) and not s.due(150_000_000)
    assert s.rejection(result(),**arguments())=='WAIT_POSITIVE_CONTROL_TICK'
    args=dict(arguments(),sim_ns=200_000_000,state_ns=200_000_000)
    assert s.rejection(result(),**args) is None # no latest-received-ID equality
    t=s.timing(200_000_000)
    assert t['actual_send_interval_s']==t['control_interval_s']==.1
    d=AckermannDispatch(cfg());r=d.request('p',np.array([.02,.1]),measured_delta_rad=0.,control_dt_s=t['control_interval_s'],target_speed_mps=.01)
    assert r['steering_tire_angle_rad']==pytest.approx(.01)
    # A failed publisher does NOT consume the operation. Successful send does.
    assert s.rejection(result(),**args) is None
    s.sent(200_000_000,.02,operation_id='p',observation_ns=100_000_000)
    assert s.rejection(result(),**args)=='ALREADY_SENT'

@pytest.mark.parametrize('kind,expected',[('reverse_order','OLDER_THAN_ADOPTED'),('epoch','EPOCH_MISMATCH'),
    ('expired','EXPIRED'),('state','STATE_TIME_MISMATCH'),('speed','STATE_SPEED_MISMATCH'),('position','STATE_POSITION_MISMATCH')])
def test_result_adoption_constraints(kind,expected):
    s=SimControlSchedule(cfg());s.reset('0');s.sent(0,0.,operation_id='HOLD')
    r=result();a=arguments()
    if kind=='reverse_order':s.adopted_observation_ns=110_000_000
    if kind=='epoch':r['epoch']='1'
    if kind=='expired':a['now_ns']=300_000_000
    if kind=='state':r['state_source_ns']=160_000_000
    if kind=='speed':a['state'][3]=.2
    if kind=='position':a['state'][0]=1.
    assert s.rejection(r,**a)==expected

def test_raw_negative_velocity_only_numerical_stopped_drive_policy():
    samples=[Sample(i*50_000_000,i*50_000_000,[-.0001,0.,0.],'base','0') for i in range(22)]
    v,p=stopped_forward_initial_speed(-.0001,samples,drive_gear_sent=True)
    assert v==0 and p['raw_signed_mps']==-.0001 and p['changed']
    assert stopped_forward_initial_speed(-.02,samples,drive_gear_sent=True)[0]==-.02
    assert stopped_forward_initial_speed(-.0001,samples,drive_gear_sent=False)[0]==-.0001
    assert stopped_forward_initial_speed(-.0001,samples[-2:],drive_gear_sent=True)[0]==-.0001

def test_stop_requires_all_source_samples_and_no_clock_stall():
    s=StopObservation()
    for ns in range(0,600_000_001,50_000_000): assert not s.add(ns,0.,fresh=True)
    for _ in range(50):assert not s.add(600_000_000,0.,fresh=True)
    assert not s.add(900_000_000,0.,fresh=True)
    for ns in range(950_000_000,1_950_000_001,50_000_000):s.add(ns,0.,fresh=True)
    assert s.confirmed
    assert not s.add(2_000_000_000,.04,fresh=True)

def test_host_is_armed_even_when_heartbeat_says_not_powered():
    w=host.HostWatch('session')
    h=dict(token='session',monotonic_ns=100,powered=False,logger_ok=True)
    assert w.check(h,101) is None and w.armed
    assert w.check(h,800_000_101)=='HEARTBEAT_STALE'
    assert w.check(None,800_000_101)=='HEARTBEAT_MISSING'

@pytest.mark.parametrize('fault',['logs','runtime_stop','inspect','none'])
def test_frozen_cleanup_never_unpauses_and_other_failures_do_not_skip(fault):
    calls=[]
    def run(argv,timeout):
        calls.append(argv)
        if (fault=='logs' and 'logs' in argv) or (fault=='runtime_stop' and 'stop' in argv):raise OSError(fault)
        if 'inspect' in argv:
            return SimpleNamespace(stdout='bad' if fault=='inspect' else json.dumps(dict(Running=True,Paused=True)))
        return SimpleNamespace(stdout='ok')
    out=host.cleanup_owned(run,'a'*64,'b'*64,paused_kill_verified=True)
    assert not any('unpause' in c for c in calls)
    assert any('kill' in c and 'a'*64 in c for c in calls)
    assert any('stop' in c and 'b'*64 in c for c in calls)
    assert any('logs' in c for c in calls)
    if fault!='none':assert out['errors']

def test_unknown_paused_exit_method_keeps_sim_frozen():
    calls=[]
    def run(argv,timeout):
        calls.append(argv);return SimpleNamespace(stdout=json.dumps(dict(Running=True,Paused=True)))
    out=host.cleanup_owned(run,'a'*64,None,paused_kill_verified=False)
    assert not any('unpause' in c or 'kill' in c or 'stop' in c for c in calls)
    assert out['errors'][0]['error']=='UNVERIFIED_KEEP_PAUSED'

def test_budget_reservation_is_not_reset_by_a_restart(tmp_path):
    pytest.importorskip('fcntl')
    path=tmp_path/'budget.json';path.write_text(json.dumps(dict(used=dict(wall_s=1039.9,forward=60,mpc=1,snapshots=1,powered=0,powered_s=0,log_bytes=100),active=None)))
    budget=host.AttemptBudget(path)
    reservation=dict(wall_s=170,forward=40,mpc=40,snapshots=2,powered=0,powered_s=0,log_bytes=1000)
    budget.reserve('one',reservation)
    with pytest.raises(ValueError,match='UNRESOLVED'):budget.reserve('two',reservation)
    budget.finish(dict(wall_s=20,forward=2,mpc=0,snapshots=1,powered=0,powered_s=0,log_bytes=500),exact=True)
    assert budget.value['used']['forward']==62 and budget.value['used']['wall_s']==1059.9
    budget.close()


def test_aabb_interior_and_unmonitored_actors_remain_unknown():
    b=dict(scene_metadata_sha256='synthetic',pose={'assumed_acquisition_bound_s':.055},scene_space=dict(
        obstacle_local_world_xy_bounds_m=[[[-10,-10],[10,10]]],dynamic_coverage_verified=True))
    a=scene_aabb_evidence(np.zeros((1,5)),cfg(),b,state_ns=1,now_sim_ns=1,epoch='0')
    assert not a['verified'] and a['reason']=='STATIC_MESH_INTERIOR_UNRESOLVED'
    b['scene_space']['obstacle_local_world_xy_bounds_m']=[[[10,10],[20,20]]]
    b['scene_space']['dynamic_coverage_verified']=False
    assert scene_aabb_evidence(np.zeros((1,5)),cfg(),b,state_ns=1,now_sim_ns=1,epoch='0')['reason']=='MOVABLE_ACTOR_COVERAGE_UNVERIFIED'
