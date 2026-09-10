import json
from pathlib import Path
from copy import deepcopy
import numpy as np
from aic_transfuser_lite.control.path_control_bridge import Limits, Vehicle
from aic_transfuser_lite.control.v4_pp_connection import ShadowPPConnection, pp_center_pose


def setup():
    cfg=json.loads(Path('ros2_ws/src/aic_e2e_runtime/config/v4_pp_shadow_connection.json').read_text())
    c=ShadowPPConnection(Limits(**cfg['limits']), cfg['fixed_frame'])
    r=dict(event='PLAN',session_id='s',clock='sim',epoch='0',source='FIXED_V4_UNCORRECTED',
        source_s=1.,generated_monotonic_s=99.,expires_s=1.,accepted=False,
        frame='base_link',reference_point='BASE_LINK_ORIGIN',output_id='p',
        raw_xy_m=[[float(x),0.] for x in np.linspace(0,2,20)],
        observation_pose_xyyaw=[0.,0.,0.],observation_pose_evidence='SYNTHETIC',
        pose_frame=cfg['fixed_frame'],transport_kind='DIAGNOSTIC_NOT_CONTROL')
    return c,r,Vehicle('v',1.,'sim','0',(0.,0.,0.),0.,0.)


def test_connection_and_expiry():
    c,r,s=setup(); raw=deepcopy(r)
    assert c.accept(r,1.)
    ref=c.tick(1.,s)
    assert ref is not None and ref.speed_mps[0]>0 and ref.speed_mps[-1]==0
    assert r==raw and ref.expires_s==1.5
    assert c.tick(1.5,s) is None
    assert c.tick(1.1,s) is None


def test_default_shadow_has_no_fixed_half_metre_speed_cap():
    from dataclasses import replace
    c,r,s=setup()
    assert c.limits.speed_cap_mps is None
    assert c.accept(r,1.)
    ref=c.tick(1.,replace(s,speed_mps=4.))
    assert ref is not None
    assert .5 < ref.speed_mps[0] < 2.
    assert ref.speed_mps[-1]==0.
    assert np.isfinite(ref.speed_mps).all()
    assert ref.speed_source=='CONSTRAINT_DERIVED_TRIAL_POLICY_NOT_MODEL_SPEED'


def test_missing_state_and_packet_clear():
    c,r,s=setup(); assert c.accept(r,1.)
    assert c.tick(1.,None) is None
    assert c.tick(1.05,s) is None
    assert not c.accept({},1.1)


def test_pp_midpoint_produces_correct_rear():
    x,y,yaw=pp_center_pose((10.,20.,np.pi/2),1.087,-.484)
    assert abs(x-10)<1e-12
    assert abs((y-1.087/2)-(20-.484))<1e-12


def test_context_reset_rejects_first_new_context():
    c,r,s=setup(); assert c.accept(r,1.)
    r['epoch']='1'
    assert not c.accept(r,1.1)
    assert c.reason=='CONTEXT_RESET'


def test_shadow_output_only_source():
    text=Path('ros2_ws/src/aic_e2e_runtime/aic_e2e_runtime/v4_pp_connection_node.py').read_text()
    assert 'AckermannControlCommand' not in text
    assert "'/control/command" not in text
    assert "'/shadow/v4/pp/trajectory'" in text
