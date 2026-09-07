import numpy as np
import pytest
from aic_transfuser_lite.control.static_course_map_v4 import StaticCourseMap
from aic_transfuser_lite.control.map_observation_v4 import compare_observation
from test_spatial_live_pp_v4 import budget


def fixture():
    grid=np.zeros((40,40),np.int8);grid[:,20:]=100
    course=StaticCourseMap(grid,.1,(10.,10.,0.),{})
    scan=dict(ranges=np.ones(750),angle_min=-.001,angle_increment=.000002,range_min=.05,range_max=10.)
    return course,scan


def test_world_translation_hits_and_nonpromotion():
    m,s=fixture()
    r=compare_observation(m,np.array([300011.5,3900011.5,0]),s,lidar_x_m=0.,lidar_y_m=0.)
    assert r['centre_class']==0 and r['exact_hit_counts']['occupied']==750
    assert r['hits_near_occupied_020m']==750 and not r['runtime_permission']
    assert not r['geometry_binding_verified']
    assert r['map_xy_m']==[11.5,11.5]


def test_rotation_and_invalid_rays():
    m,s=fixture();s['ranges'][:3]=[np.nan,np.inf,10.]
    r=compare_observation(m,np.array([300011.5,3900011.5,np.pi]),s,lidar_x_m=0.,lidar_y_m=0.)
    assert r['valid_hit_count']==747 and r['exact_hit_counts']['free']==747


def test_bad_inputs():
    m,s=fixture()
    with pytest.raises(ValueError):compare_observation(m,np.zeros(3),s,lidar_x_m=0.,lidar_y_m=0.)
    s['angle_increment']=0
    with pytest.raises(ValueError):compare_observation(m,np.array([300011.,3900011.,0]),s,lidar_x_m=0.,lidar_y_m=0.)


def test_deadline_renewal_once_preserves_prior_record(tmp_path,monkeypatch):
    b=budget(tmp_path)
    try:
        monkeypatch.setattr('spatial_pp_budget_v4.time.time',lambda:100.)
        a=b.authorize();before=dict(b.value['used'])
        monkeypatch.setattr('spatial_pp_budget_v4.time.time',lambda:2000.)
        r=b.renew_map_test_deadline()
        assert a['driving_cutoff_unix_s']==1900. and r['new_cutoff_unix_s']==3800.
        assert b.value['used']==before and len(b.value['authorization_changes'])==2
        monkeypatch.setattr('spatial_pp_budget_v4.time.time',lambda:3000.)
        assert b.renew_map_test_deadline()==r
        assert b.value['pp_authorization']['driving_cutoff_unix_s']==3800.
    finally:b.close()
