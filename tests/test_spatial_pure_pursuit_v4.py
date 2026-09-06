from pathlib import Path
import json
import numpy as np
import pytest
import yaml
from aic_transfuser_lite.control.spatial_tracking_contracts_v4 import PreparedPath
from aic_transfuser_lite.evaluation.spatial_pure_pursuit_v4 import evaluate
from aic_transfuser_lite.control.waypoint_controller import control_from_waypoints


def fixture():
    cfg=yaml.safe_load((Path(__file__).parents[1]/'configs/control/spatial_sim_e2e_v4.yaml').read_text())
    s=np.linspace(0,1.5,76);xy=np.c_[s,s*0]
    return PreparedPath(xy,s,np.arange(len(s)),np.zeros(len(s)-1),np.zeros(len(s)),{},None),cfg


def test_straight_moves_and_brakes_with_limits():
    p,c=fixture();r=evaluate(p,np.zeros(5),c)
    assert r['tracking_stop_pass'] and r['negative_brake_cycles']>0
    assert r['max_steering_rad']==0 and r['max_speed_mps']<=.3
    assert r['max_jerk_mps3']<=2.+1e-9 and not r['violations']
    assert json.loads(json.dumps(r,allow_nan=False))['tracking_stop_pass'] is True


def test_explicit_stop_not_called_endpoint_success():
    p,c=fixture();r=evaluate(p,np.zeros(5),c,force_stop_s=2.)
    assert r['stopped_after_motion'] and r['stop_reason']=='EXPLICIT_OFFLINE_STOP'
    assert not r['tracking_stop_pass'] and r['negative_brake_cycles']>0


def test_invalid_input_rejected():
    p,c=fixture()
    with pytest.raises(ValueError):evaluate(p,np.full(5,np.nan),c)
    p.reason='REJECTED'
    with pytest.raises(ValueError):evaluate(p,np.zeros(5),c)


def test_pp_left_right_sign():
    assert control_from_waypoints(np.array([[1.,.2]]),.2,0).steering_rad>0
    assert control_from_waypoints(np.array([[1.,-.2]]),.2,0).steering_rad<0
