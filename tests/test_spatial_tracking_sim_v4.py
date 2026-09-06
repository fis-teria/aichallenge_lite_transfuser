import numpy as np
from test_spatial_mpc_v4 import config, prepared
from aic_transfuser_lite.evaluation.spatial_tracking_sim_v4 import plant_step, execute_scene, synthetic_scenes
from aic_transfuser_lite.control.spatial_mpc_v4 import prediction

def test_plant_independent_rk4_and_no_projection():
    cfg=config(); z=np.array([0.,.2,.1,.3,.2]); u=np.array([.1,.1])
    nxt,applied=plant_step(z,u,0.,cfg)
    pred=prediction(z,u[None],cfg)[-1]
    assert nxt[1]>.2 and not np.array_equal(nxt,pred)
    assert applied['substep_states'].shape==(6,5)

def test_stop_saturation_explicit():
    cfg=config(); z=np.array([0.,0.,0.,.005,0.])
    nxt,applied=plant_step(z,np.array([-1.,0.]),-.8,cfg)
    assert nxt[3]==0 and applied['unilateral_stop_substeps']>0
    assert nxt[0]>0 and nxt[0]<.001

def test_hold_stale_and_short_path_do_not_invent_success():
    cfg=config(); cfg['budgets']['scene_cycles']=2
    for kind in ('HOLD','stale','short'):
        scene=synthetic_scenes()[0]; scene['initial_state']=[0,0,0,.5,0]
        if kind=='HOLD': scene['permission']='HOLD'
        elif kind=='stale': scene['stale']=True
        else: scene.update(switch_time_s=0.,switch_remaining_m=.02)
        events=[]
        result=execute_scene(scene,cfg,lambda:True,lambda k,v:events.append((k,v)))
        assert result['fallback_cycles']==2 and result['solver_calls']==0 and not result['tracking_success']
        assert any(k=='cycle' and v['next_state'][0]>0 for k,v in events)

def test_known_footprint_trims_before_node_collision():
    scene=synthetic_scenes()[-1]; cfg,scene,path=prepared(scene)
    assert path.diagnostics['trim_reason']=='KNOWN_FOOTPRINT_BOUNDARY'
    assert path.diagnostics['usable_prefix_s_m']<1.0  # 1.2 m body front, not node center.
