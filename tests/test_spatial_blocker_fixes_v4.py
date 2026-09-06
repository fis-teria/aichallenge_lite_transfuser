"""Pure geometry and missing-evidence regressions; no data/model/ROS."""
import numpy as np
import pytest
from aic_transfuser_lite.control.constrained_reference_v4 import ordered_polyline_error
from aic_transfuser_lite.control.spatial_sim_guard_v4 import scene_aabb_evidence


def test_target_corner_not_on_reference_grid_is_in_certificate():
    q = np.array([0., 1.]); xy = np.zeros((2, 2))
    s = np.array([0., .37, 1.]); target = np.array([[0.,0.], [0.,.09], [0.,0.]])
    check, error = ordered_polyline_error(q, xy, s, target)
    assert .37 in check
    assert np.linalg.norm(error,axis=1).max() == .09


def test_union_certificate_bounds_dense_samples_without_grid_penalty():
    rng = np.random.default_rng(42)
    for _ in range(30):
        a = np.r_[0., np.sort(rng.uniform(0,1,6)), 1.]
        b = np.r_[0., np.sort(rng.uniform(0,1,9)), 1.]
        x, y = rng.normal(size=(len(a),2)), rng.normal(size=(len(b),2))
        _, error = ordered_polyline_error(a,x,b,y)
        dense = np.unique(np.r_[np.linspace(0,1,10001),a,b])
        difference = np.column_stack([np.interp(dense,a,x[:,k])-np.interp(dense,b,y[:,k]) for k in (0,1)])
        assert np.max(np.linalg.norm(difference,axis=1)) <= np.max(np.linalg.norm(error,axis=1))+1e-12


@pytest.mark.parametrize('q', [[0.,0.], [1.,0.], [0.,float('nan')], [0.,2.]])
def test_bad_polyline_contract(q):
    with pytest.raises(ValueError): ordered_polyline_error(q,np.zeros((2,2)),[0.,1.],np.zeros((2,2)))


def binding():
    return dict(scene_metadata_sha256='SYNTHETIC',pose={'assumed_acquisition_bound_s':.055},
        scene_space=dict(obstacle_local_world_xy_bounds_m=[[[10.,10.],[20.,20.]]],dynamic_coverage_verified=True))


def config():
    return dict(rear_overhang_m=.51,body_width_m=1.54,wheelbase_m=1.087,front_overhang_m=.53,maximum_speed_mps=.3)


def test_empty_footprint_cannot_be_certified_clear():
    with pytest.raises(ValueError): scene_aabb_evidence(np.empty((0,5)),config(),binding(),state_ns=1,now_sim_ns=1,epoch='0')


@pytest.mark.parametrize('boxes', [[[[float('nan'),0],[1,1]]], [[[2,2],[1,1]]], [[0,1]]])
def test_invalid_bounds_cannot_become_free(boxes):
    b=binding(); b['scene_space']['obstacle_local_world_xy_bounds_m']=boxes
    with pytest.raises(ValueError): scene_aabb_evidence(np.zeros((1,5)),config(),b,state_ns=1,now_sim_ns=1,epoch='0')


@pytest.mark.parametrize('flag', ['false',1,None,False])
def test_non_boolean_dynamic_evidence_is_unknown(flag):
    b=binding(); b['scene_space']['dynamic_coverage_verified']=flag
    result=scene_aabb_evidence(np.zeros((1,5)),config(),b,state_ns=1,now_sim_ns=1,epoch='0')
    assert not result['verified'] and result['reason']=='MOVABLE_ACTOR_COVERAGE_UNVERIFIED'
