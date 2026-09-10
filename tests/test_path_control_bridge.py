"""Fixture-only bridge evaluation. Does not infer, publish or read bags."""
from dataclasses import replace
import json
import math
import os
from pathlib import Path
import numpy as np
import pytest
from aic_transfuser_lite.control.path_control_bridge import Limits, Plan, Vehicle, PathPose, ShadowBridge, from_v4_record


def setup(xy=None):
    limits=Limits('ARTIFICIAL_TEST_ONLY',True,1.,-.2,.6,1.,.8,.5,1.,.3,
                  .5,.1,.5,1.,.6,.8,1.,2.,.2,.01,.05,.1)
    if xy is None: xy=np.c_[np.linspace(0,4,41),np.zeros(41)]
    plan=Plan('p1','FIXTURE_E2E_GEOMETRY',1.,50.,3.,'sim','0','base_link','BASE_LINK_ORIGIN',
              tuple(map(tuple,xy)))
    pose=PathPose('p1',1.,'sim','0',(0.,0.,0.),'SYNTHETIC_POSE')
    state=Vehicle('s1',1.,'sim','0',(.2,0.,0.),0.,0.)
    return ShadowBridge(limits,fixture_mode=True),plan,pose,state


def tick(b,s,t=1.):
    return b.tick(now_s=t,clock='sim',epoch='0',state=replace(s,stamp_s=t))


def test_shadow_curvature_warning_preserves_pp_steering_rejection():
    angle=np.linspace(0.,1.5,31)
    xy=np.c_[np.sin(angle),1-np.cos(angle)]
    b,p,tf,s=setup(xy)
    assert not b.accept(p,tf,now_s=1.)
    assert b.reason=='CURVATURE_INFEASIBLE'
    b=ShadowBridge(b.limits,fixture_mode=True,shadow_curvature_log_only=True)
    assert b.accept(p,tf,now_s=1.)
    assert b.audit['curvature_limit_exceeded']
    assert b.audit['curvature_steer_excess_rad']>0.
    assert b.audit['curvature_plan_id']==p.id
    out=tick(b,s)
    assert not out['valid'] and out['reason']=='PP_STEER_INFEASIBLE'
    assert out['curvature_policy']=='SHADOW_LOG_ONLY'


def test_curvature_warning_cannot_enable_nonfixture_or_leak_to_next_plan():
    b,p,tf,s=setup()
    with pytest.raises(ValueError,match='SHADOW_ONLY_CURVATURE_POLICY'):
        ShadowBridge(b.limits,shadow_curvature_log_only=True)
    b=ShadowBridge(b.limits,fixture_mode=True,shadow_curvature_log_only=True)
    assert b.accept(p,tf,now_s=1.)
    assert not b.audit['curvature_limit_exceeded']
    assert not b.accept(replace(p,id='bad',xy_m=()),tf,now_s=1.1)
    assert 'curvature_plan_id' not in b.audit


def test_no_fixed_speed_cap_keeps_terminal_braking_and_explicit_stop():
    b,p,tf,s=setup()
    b.limits=replace(b.limits,speed_cap_mps=None)
    assert b.accept(p,tf,now_s=1.)
    out=tick(b,replace(s,speed_mps=4.))
    expected=math.sqrt(.15**2+2*4)-.15
    assert out['valid']
    assert out['target_speed_mps']==pytest.approx(expected)
    assert out['command']['acceleration_mps2']<0
    assert out['fixed_speed_cap_mps'] is None
    b,p,tf,s=setup()
    b.limits=replace(b.limits,speed_cap_mps=None)
    p=replace(p,speed_mps=0.,speed_plan_id=p.id,speed_source='EXPLICIT_STOP')
    assert b.accept(p,tf,now_s=1.)
    assert tick(b,s)['target_speed_mps']==0.


@pytest.mark.parametrize('cap',[float('nan'),float('inf'),-1.,0.])
def test_disabled_cap_requires_none_not_invalid_number(cap):
    b,p,tf,s=setup()
    b.limits=replace(b.limits,speed_cap_mps=cap)
    assert not b.accept(p,tf,now_s=1.)


@pytest.fixture
def trace(request,tmp_path):
    def save(data):
        root=Path(os.environ.get('PATH_BRIDGE_TRACE_DIR',str(tmp_path)))
        root.mkdir(parents=True,exist_ok=True)
        (root/(request.node.name.replace('/','_')+'.json')).write_text(json.dumps(data,indent=2),encoding='utf8')
    return save


@pytest.mark.parametrize('shape',['straight','left','right','s'])
def test_normal_shapes(shape,trace):
    x=np.linspace(0,4,41)
    if shape in ('left','right'):
        a=x/8; xy=np.c_[8*np.sin(a),8*(1-np.cos(a))*(1 if shape=='left' else -1)]
    else: xy=np.c_[x,.08*np.sin(x) if shape=='s' else x*0]
    b,p,tf,s=setup(xy); raw=p.xy_m
    assert b.accept(p,tf,now_s=1.)
    out=tick(b,s); trace(out)
    assert out['valid'] and p.xy_m==raw
    assert abs(out['command']['tire_steering_rad'])<=b.limits.steer_rad
    assert out['source']==p.source and out['speed_source']=='EXPLICIT_TRIAL_POLICY'


def test_old_body_transform_and_control_between_updates(trace):
    b,p,tf,s=setup()
    tf=replace(tf,base_in_local=(10.,20.,math.pi/2))
    assert b.accept(p,tf,now_s=1.)
    s=replace(s,pose=(10.,20.2,math.pi/2))
    out=[tick(b,s,1.+i*.05) for i in range(5)]
    assert all(x['valid'] for x in out)
    assert all(abs(x['command']['tire_steering_rad'])<1e-5 for x in out)
    trace(out)


def test_initial_offsets_and_model_closed_loop(trace):
    b,p,tf,s=setup(); b.limits=replace(b.limits,path_ttl_s=20.)
    p=replace(p,expires_s=20.)
    assert b.accept(p,tf,now_s=1.)
    s=replace(s,pose=(.2,.05,.02))
    output=[]
    for i in range(120):
        t=1+i*.05; out=tick(b,s,t); output.append(out)
        assert out['valid'],out['reason']
        cmd=out['command']; v=max(0,s.speed_mps+cmd['acceleration_mps2']*.05)
        # Bicycle at rear axle; state.pose remains base origin.
        yaw=s.pose[2]; rear=np.array(s.pose[:2])+b.limits.rear_x_in_base_m*np.array([math.cos(yaw),math.sin(yaw)])
        rear+=v*.05*np.array([math.cos(yaw),math.sin(yaw)])
        yaw+=v*math.tan(cmd['tire_steering_rad'])/b.limits.wheelbase_m*.05
        base=rear-b.limits.rear_x_in_base_m*np.array([math.cos(yaw),math.sin(yaw)])
        s=replace(s,id=f's{i+2}',pose=(*base,yaw),speed_mps=v,tire_steer_rad=cmd['tire_steering_rad'])
    assert output[-1]['cross_track_m']<output[0]['cross_track_m']
    assert max(abs(x['steering_rate_rps']) for x in output)<=b.limits.steer_rate_rps+1e-10
    trace(dict(scope='KINEMATIC_MODEL_CLOSED_LOOP_NOT_REAL_TRAJECTORY',results=output))


def test_initial_offset_requiring_excess_steer_is_rejected(trace):
    b,p,tf,s=setup();assert b.accept(p,tf,now_s=1.)
    out=tick(b,replace(s,pose=(.2,.15,.05)))
    trace(out);assert not out['valid'] and out['reason']=='PP_STEER_INFEASIBLE'


def test_past_path_after_vehicle_translation_and_rotation(trace):
    b,p,tf,s=setup();assert b.accept(p,tf,now_s=1.)
    old=b.reference.copy()
    s=replace(s,pose=(.4,.01,.03))
    out=tick(b,s,1.05);trace(out)
    assert out['valid'] and np.array_equal(old,b.reference)
    # Path remains y=0 in fixed local frame, not reattached to current ego.
    assert out['command']['tire_steering_rad']<0


def test_explicit_stop(trace):
    b,p,tf,s=setup(); p=replace(p,speed_plan_id='p1',speed_mps=0.,speed_source='EXPLICIT_STOP')
    assert b.accept(p,tf,now_s=1.)
    out=tick(b,replace(s,speed_mps=.2));trace(out)
    assert out['valid'] and out['target_speed_mps']==0 and out['command']['acceleration_mps2']<0


@pytest.mark.parametrize('case,reason',[
 ('empty','PATH_SHAPE'),('nan','PATH_NONFINITE'),('short','HORIZON_SHORT'),
 ('stale','PLAN_STALE'),('future','PLAN_FUTURE'),('tf','OBSERVATION_TF_MISSING'),
 ('frame','FRAME_CONTRACT_UNKNOWN'),('speed','SPEED_PLAN_MISMATCH'),
 ('timed','TIMED_TRAJECTORY_UNSUPPORTED'),('fold','PATH_FOLDBACK'),('curve','CURVATURE_INFEASIBLE')])
def test_bad_input_clears_reference(case,reason,trace):
    b,p,tf,s=setup()
    if case=='empty': p=replace(p,xy_m=())
    elif case=='nan': p=replace(p,xy_m=((0.,0.),(.1,float('nan'))))
    elif case=='short': p=replace(p,xy_m=((0.,0.),(.1,0.)))
    elif case=='stale': p=replace(p,source_s=-2.)
    elif case=='future': p=replace(p,source_s=2.)
    elif case=='tf': tf=None
    elif case=='frame': p=replace(p,frame='UNKNOWN')
    elif case=='speed': p=replace(p,speed_mps=.2,speed_plan_id='other',speed_source='model')
    elif case=='timed': p=replace(p,kind='TIMED',xy_m=((0.,0.),(0.,0.),(.2,0.)))
    elif case=='fold': p=replace(p,xy_m=((0.,0.),(.2,0.),(0.,0.)))
    else: p=replace(p,xy_m=((0.,0.),(.1,0.),(.2,.1)))
    assert not b.accept(p,tf,now_s=1.)
    out=tick(b,s);trace(out)
    assert out['reason']==reason and out['command'] is None


def test_expiry_reset_order_and_missing_state(trace):
    rows=[]
    for case in ('expiry','reset','order','state','state_stale','jump'):
        b,p,tf,s=setup();assert b.accept(p,tf,now_s=1.)
        assert tick(b,s)['valid']
        if case=='expiry': out=tick(b,s,3.)
        elif case=='reset': out=tick(b,s,.5)
        elif case=='order':
            assert not b.accept(p,tf,now_s=1.05);out=tick(b,s,1.05)
        elif case=='state': out=b.tick(now_s=1.05,clock='sim',epoch='0',state=None)
        elif case=='state_stale': out=b.tick(now_s=1.3,clock='sim',epoch='0',state=s)
        else:
            new=replace(p,id='p2',source_s=1.05); newtf=replace(tf,plan_id='p2',stamp_s=1.05,base_in_local=(5.,0.,0.))
            assert not b.accept(new,newtf,now_s=1.05);out=tick(b,s,1.05)
        assert not out['valid'] and out['command'] is None
        assert not tick(b,s,3.1)['valid']
        rows.append(out)
    trace(rows)


def test_vehicle_unknown_no_pass(trace):
    b,p,tf,s=setup();b.fixture_mode=False
    assert not b.accept(p,tf,now_s=1.)
    out=tick(b,s); trace(out);assert out['reason']=='VEHICLE_CONTRACT_UNKNOWN'


def test_v4_record_adapter_raw_preservation(trace):
    _,p,_,_=setup()
    record=dict(output=dict(output_id='v4:output',model_xy_m=p.xy_m[:20],frame='base_link',
                            pose_reference_point='BASE_LINK_ORIGIN',status='SHAPE_FINITE_ONLY',
                            t_obs=dict(status='KNOWN',ns='1000000000',domain='ROS_SIM',clock_id='clock',epoch_id='0')),
                timing=dict(snapshot_ready=dict(status='KNOWN',ns='50000000000',domain='MONOTONIC')))
    before=repr(record); plan=from_v4_record(record,expires_s=2.)
    assert repr(record)==before and plan.speed_mps is None
    assert plan.source_s==1. and plan.generated_mono_s==50.
    trace(dict(source=plan.source,points=len(plan.xy_m),speed=plan.speed_mps))
