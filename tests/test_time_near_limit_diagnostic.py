"""Opt-in AWSIM proximity diagnosis preserves the standard guard by default."""
import json
import math
from pathlib import Path

import numpy as np
import pytest

from aic_transfuser_lite.control.curvature_support_v2 import (
    clearance_dimensions, curvature_support_envelope, NEAR_LIMIT_CLEARANCE)
from aic_transfuser_lite.control.time_trial_v1 import validate_trial_config
from aic_transfuser_lite.control.turning_scan_guard import check_turning_scan


def config():
    value = json.loads(Path('configs/control/time_path_random_lap_20260915.json').read_text())
    return dict(value, diagnostic_clearance_profile=NEAR_LIMIT_CLEARANCE,
                diagnostic_only=True, maximum_diagnostic_trials=1)


@pytest.mark.parametrize('field,value', [('diagnostic_only',False),('maximum_diagnostic_trials',2),
    ('maximum_diagnostic_trials',True),('host','different-host'),('execution_profile','bounded_10s'),
    ('obstacle_policy','steering_sweep_v1'),('vehicle_model_policy','ideal_bicycle_v1')])
def test_smaller_reserve_requires_explicit_single_awsim_diagnostic(field,value):
    cfg=config();cfg[field]=value
    with pytest.raises(ValueError):validate_trial_config(cfg)


def test_standard_metadata_and_exact_default_geometry_are_preserved():
    assert validate_trial_config(config())=='fixed_5kmh'
    assert clearance_dimensions('standard_v1')==(.4,.85)
    args=(-.1,.2,1.8,.004,np.array([1.649,0.]))
    implicit=curvature_support_envelope(*args)
    explicit=curvature_support_envelope(*args,clearance_profile='standard_v1')
    for a,b in zip(implicit[:2],explicit[:2]):np.testing.assert_array_equal(a,b)
    assert implicit[2]==explicit[2] and 'diagnostic_only' not in implicit[2]
    with pytest.raises(ValueError,match='CLEARANCE_PROFILE'):
        curvature_support_envelope(*args,clearance_profile='disabled')


def test_diagnostic_envelope_still_encloses_physical_body_during_varying_curvature():
    k_min,k_max=-.12,.23;travel=.1+1.3*.5+1.3**2/2
    n,h,meta=curvature_support_envelope(k_min,k_max,travel,.004,np.array([1.649,0.]),clearance_profile=NEAR_LIMIT_CLEARANCE)
    assert n.shape==(64,2) and h.shape==(64,) and meta['diagnostic_only'] is True
    assert meta['body_half_width_m']==.70 and meta['fixed_stopping_reserve_m']==.1
    corners=np.array([[-.510,-.65],[1.984,-.65],[1.984,.65],[-.510,.65]])
    position=np.zeros(2);yaw=0.;steps=1800;ds=travel/steps
    for i in range(steps+1):
        c,s=math.cos(yaw),math.sin(yaw)
        world=corners@np.array([[c,s],[-s,c]])+position
        assert np.max(world@n.T-h)<=1e-8
        k=k_min if (i//43)%2 else k_max
        position+=ds*np.array([math.cos(yaw+k*ds/2),math.sin(yaw+k*ds/2)]);yaw+=k*ds


def test_diagnostic_keeps_obstacle_unknown_scan_and_vehicle_model_gates():
    angles=np.linspace(-1.5,1.5,751);ranges=np.full(751,np.inf)
    options=dict(speed_mps=1.3,measured_steer_rad=0.,issued_steer_rad=0.,previous_steer_rad=0.,
        scan_in_current_rear=(1.649,0.,0.),vehicle_model_policy='awsim_understeer_v1',
        heading_rate_radps=0.,reported_lateral_mps=0.,envelope_policy='curvature_support_v2',clearance_profile=NEAR_LIMIT_CLEARANCE)
    args=(angles[0],angles[1]-angles[0],.1,25.)
    result=check_turning_scan(ranges,*args,**options)
    assert result['stopping_travel_m']==pytest.approx(.1+1.3*.5+1.3**2/2)
    ranges[375]=.3
    with pytest.raises(ValueError,match='SWEEP_OCCUPIED'):check_turning_scan(ranges,*args,**options)
    ranges[375]=np.nan
    with pytest.raises(ValueError,match='SCAN_UNKNOWN'):check_turning_scan(ranges,*args,**options)
    with pytest.raises(ValueError,match='REQUIRES_AWSIM'):
        check_turning_scan(np.full(751,np.inf),*args,**dict(options,vehicle_model_policy='ideal_bicycle_v1'))
