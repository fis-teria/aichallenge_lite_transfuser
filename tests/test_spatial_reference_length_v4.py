"""Synthetic endpoint/parameter/constraint tests; no model or sensor data."""
from pathlib import Path
import numpy as np
import pytest
import yaml
from aic_transfuser_lite.control.constrained_reference_v4 import constrained_reference, ordered_polyline_error
from aic_transfuser_lite.control.spatial_tracking_contracts_v4 import SpatialPathCandidate

POLICY='ENDPOINT_NORMALIZED_LENGTH_V1'


def setup():
    cfg=yaml.safe_load((Path(__file__).parents[1]/'configs/control/spatial_sim_e2e_v4.yaml').read_text())
    s=np.arange(1,21)/10
    xy=np.c_[s,.035*np.where(np.arange(20)%2,1.,-1.)].astype('float32')
    return cfg,SpatialPathCandidate(xy,s,'SYNTHETIC_WAVY')


def test_length_is_variable_but_raw_endpoint_coverage_and_limits_are_not():
    cfg,c=setup(); raw=c.raw_xy.tobytes()
    p=constrained_reference(c,cfg,np.zeros(5),length_policy=POLICY)
    assert p.reason is None
    d=p.diagnostics
    assert d['length_scale']<.95 and d['maximum_deviation_bound_m']<=.1
    assert d['used_raw_indices']==list(range(20)) and d['unused_tail_indices']==[]
    assert np.array_equal(d['selected_raw_endpoint_xy_m'],c.raw_xy[-1])
    assert c.raw_xy.tobytes()==raw and d['runtime_permission'] is False
    q=np.asarray(d['ordered_parameter_m']); conn=d['initial_connection_length_m']
    ts=np.r_[0.,conn+np.asarray(d['raw_actual_s_m'])];target=np.vstack([np.zeros(2),c.raw_xy])
    _,diff=ordered_polyline_error(q,p.world_xy,ts,target)
    assert abs(np.linalg.norm(diff,axis=1).max()+1e-9-d['maximum_deviation_bound_m'])<1e-12
    # Rate must be checked per physical length, not the longer target parameter.
    rate=np.diff(d['steering_knots_rad'])/(d['integration_length_m']/6)*cfg['maximum_speed_mps']
    assert abs(rate).max()<=cfg['steering_rate_limit_rad_s']+1e-8
    assert np.all(np.diff(d['integration_parameter_s_m'])>0)


def test_fixed_default_and_invalid_policy():
    cfg,c=setup();s=c.nominal_s;c=SpatialPathCandidate(np.c_[s,s*0].astype('float32'),s,'straight')
    fixed=constrained_reference(c,cfg,np.zeros(5))
    assert fixed.reason is None and fixed.diagnostics['length_scale']==1
    assert constrained_reference(c,cfg,np.zeros(5),length_policy='unknown').reason=='LENGTH_POLICY'


@pytest.mark.parametrize('kind',['nan','duplicate','far','deviation'])
def test_no_invalid_input_or_deviation_bypass(kind):
    cfg,c=setup()
    if kind=='nan':c.raw_xy[0,0]=np.nan
    if kind=='duplicate':c.raw_xy[3]=c.raw_xy[2]
    if kind=='far':c.raw_xy[:,1]+=1.
    if kind=='deviation':cfg['reference_max_deviation_m']=.101
    assert constrained_reference(c,cfg,np.zeros(5),length_policy=POLICY).reason is not None
