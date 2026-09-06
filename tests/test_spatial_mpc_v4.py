from pathlib import Path
import numpy as np
import pytest
import yaml
from aic_transfuser_lite.control.spatial_tracking_contracts_v4 import SpatialPathCandidate
from aic_transfuser_lite.control.spatial_path_adapter_v4 import prepare, transform, wrap
from aic_transfuser_lite.control.spatial_speed_profile_v4 import plan, horizon, stopping_distance
from aic_transfuser_lite.control.spatial_mpc_v4 import SpatialMPC
from aic_transfuser_lite.evaluation.spatial_tracking_sim_v4 import synthetic_scenes

def config():
    return yaml.safe_load((Path(__file__).parents[1]/'configs/control/spatial_mpc_virtual_v4.yaml').read_text())

def prepared(scene=None):
    cfg=config(); scene=scene or synthetic_scenes()[0]
    c=SpatialPathCandidate(scene['raw_xy'],scene['nominal_s'],scene['id'])
    return cfg,scene,prepare(c,cfg,scene)

def test_polyline_time_rest_launch_stop_and_identity():
    cfg,scene,path=prepared(); raw=scene['raw_xy'].tobytes(); ref=plan(path,cfg)
    assert path.reason is None and ref.speed.max()>0.1 and ref.speed[-1]==0
    assert np.isclose(path.actual_s[-1],2.5) and np.isclose(ref.endpoint_s,2.4)
    assert raw==scene['raw_xy'].tobytes() and path.source_indices[0]==-1
    assert np.max(np.abs(ref.acceleration))<=cfg['acceleration_max_mps2']
    h=horizon(ref,path,100.,cfg)
    assert np.all(h['endpoint_hold']) and np.all(h['speed']==0) and h['spatial_extension_m']==0
    assert np.allclose(plan(path,cfg,'HOLD').s,0)

@pytest.mark.parametrize('kind',['nan','inf','origin','gap','duplicate','cusp'])
def test_bad_or_duplicate_geometry(kind):
    cfg,scene,path=prepared(); xy=scene['raw_xy'].copy()
    if kind=='nan': xy[5,0]=np.nan
    if kind=='inf': xy[5,0]=np.inf
    if kind=='origin': xy[:]=0
    if kind=='gap': xy[5,0]=3
    if kind=='duplicate': xy[5]=xy[4]
    if kind=='cusp': xy[5]=xy[3]
    got=prepare(SpatialPathCandidate(xy,scene['nominal_s'],'test'),cfg,scene)
    assert (got.reason is None)==(kind=='duplicate')

def test_wrap_frame_roundtrip_large_turn_not_x_monotonic():
    cfg=config(); s=np.linspace(.05,8.,161); xy=np.c_[3*np.sin(s/3),3*(1-np.cos(s/3))].astype('float32')
    path=prepare(SpatialPathCandidate(xy,s,'bigturn'),cfg,dict(saved=True))
    assert path.reason is None and np.any(np.diff(xy[:,0])<0)
    pose=np.array([1.,2.,2.9]); assert np.allclose(transform(transform(xy,pose),pose,True),xy)
    assert abs(float(wrap(2*np.pi+.1))-.1)<1e-9

@pytest.mark.parametrize('fault',[None,'failure','timeout','nonfinite'])
def test_real_solver_and_rejection(fault):
    cfg,scene,path=prepared(); ref=plan(path,cfg); mpc=SpatialMPC(cfg)
    result=mpc.solve(np.zeros(5),horizon(ref,path,0.,cfg),np.zeros(2),scene,fault=fault)
    assert mpc.calls==1 and result.diagnostics['actual_solver_called']
    assert result.accepted==(fault is None)
    if fault is None: assert result.controls.shape==(15,2) and result.first_control[0]>0
    else: assert result.diagnostics['rejection_reason'] and np.isfinite(result.first_control).all()

def test_stop_distance_jerk_delay_not_constant_deceleration_claim():
    cfg=config()
    assert stopping_distance(.5,0,cfg)>.5**2/2
    assert stopping_distance(.5,0,cfg,.3)>stopping_distance(.5,0,cfg)

def test_cli_help_without_pythonpath():
    import os,subprocess,sys
    env=dict(os.environ); env.pop('PYTHONPATH',None)
    result=subprocess.run([sys.executable,str(Path(__file__).parents[1]/'tools/evaluate_spatial_mpc_v4.py'),'--help'],
        env=env,capture_output=True,text=True)
    assert result.returncode==0, result.stderr

def test_missing_input_and_self_intersection_reject():
    cfg=config()
    assert prepare(SpatialPathCandidate(np.empty((0,2),dtype='float32'),np.empty(0),'missing'),cfg,{}).reason=='SHAPE_DTYPE'
    # Dense crossing polyline, loose steering/cusp checks only to isolate intersection detection.
    vertices=np.array([[0,0],[1,1],[0,1],[1,0]],dtype=float)
    xy=np.concatenate([np.linspace(a,b,11)[1:] for a,b in zip(vertices[:-1],vertices[1:])]).astype('float32')
    cfg['cusp_angle_rad']=3.2; cfg['steering_limit_rad']=1.56
    assert prepare(SpatialPathCandidate(xy,np.arange(len(xy)), 'cross'),cfg,{}).reason=='SELF_INTERSECTION'
