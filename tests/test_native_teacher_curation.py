from dataclasses import replace
import math

import numpy as np
import pytest

from aic_transfuser_lite.data.native_teacher_curation import (
    NativeCurationConfig, classify_anchor, convex_point_distance, spaced_indices,
    teacher_command_is_usable, window_indices,
)


def good_evidence():
    return dict(coverage_ok=True,teacher_ok=True,perception_ok=True,pose_geometry_ok=True,
        box_distance_m=9.,cone_gap_m=2.,cone_distance_m=8.,map_median_max_m=.05,
        map_inlier_fraction_min=.9,wall_points_min=150,scan_span_min_rad=2.,all_free_run=True)


@pytest.mark.parametrize('mode,reason',[
    ('AVOID','mppi_brain:return_unavailable:infeasible_braking_fallback'),
    ('AVOID','mppi_brain:alongside_hold:infeasible_braking_fallback'),
    ('FREE_RUN','mppi_brain:ordinary_hold:mppi_brain:infeasible_braking_fallback'),
])
def test_infeasible_braking_is_not_an_expert_command_even_without_emergency(mode,reason):
    assert not teacher_command_is_usable(mode,False,reason)


def test_command_validation_keeps_valid_motion_and_does_not_infer_stop_intent():
    assert teacher_command_is_usable('AVOID',False,'mppi_brain:retimed_continuation')
    assert teacher_command_is_usable('FREE_RUN',False,'mppi_brain:longitudinal_follow')
    assert teacher_command_is_usable('AVOID',False,'mppi_brain:validated_braking_fallback')
    # A feasible command is not sufficient for a stop label; the speed gate still applies.
    assert not decide(velocity_mps=np.zeros(30))['selected']
    assert not teacher_command_is_usable('AVOID',True,'mppi_brain:avoid_left')
    assert not teacher_command_is_usable('RECOVERY',False,'mppi_brain:avoid_left')
    assert not teacher_command_is_usable('AVOID',False,' ')
    with pytest.raises(ValueError,match='TEACHER_COMMAND_TYPES'):
        teacher_command_is_usable('AVOID',False,None)
    with pytest.raises(ValueError,match='TEACHER_COMMAND_TYPES'):
        teacher_command_is_usable('AVOID',0,'mppi_brain:avoid_left')


def decide(**changes):
    args=dict(xy_m=np.column_stack([np.arange(1,31)*.1,np.zeros(30)]),
        xy_mask=np.ones(30,bool),velocity_mps=np.ones(30),velocity_mask=np.ones(30,bool),
        findings=['NATIVE_PHYSICAL_CLEARANCE_NOT_FULLY_OBSERVED'],evidence=good_evidence())
    args.update(changes)
    return classify_anchor(**args)


def test_nominal_motion_can_be_selected_without_claiming_native_contact_certification():
    result=decide()
    assert result['selected'] and result['use']=='nominal_xy_speed'
    assert result['reasons']==[]


def test_static_cone_context_is_observed_motion_not_an_inferred_stop_or_behaviour_label():
    e=good_evidence();e.update(cone_distance_m=3.,cone_gap_m=.61,all_free_run=False)
    result=decide(evidence=e)
    assert result['selected'] and result['use']=='static_cone_xy_speed'
    e['cone_gap_m']=.59
    result=decide(evidence=e)
    assert not result['selected'] and 'STATIC_CONE_PROJECTION_MARGIN' in result['reasons']


@pytest.mark.parametrize('finding',[
    'RECORDED_POSE_PROJECTED_CONE_GAP_BELOW_030','NOMINAL_BOX_GAP_BELOW_030',
    'RECORDED_POSE_WALL_OVERLAP','LIDAR_OBSERVED_POINT_GAP_BELOW_030'])
def test_prior_adverse_clearance_evidence_excludes_even_otherwise_good_motion(finding):
    assert decide(findings=[finding])['disposition']=='exclude'


def test_stationary_unknown_stop_and_one_future_creep_point_are_held():
    for speeds in (np.zeros(30),np.r_[np.ones(29),.2]):
        result=decide(velocity_mps=speeds)
        assert result['disposition']=='hold' and 'STOP_OR_CREEP_INTENT_UNVERIFIED' in result['reasons']


def test_nearby_dynamic_box_or_prior_box_proximity_is_not_admitted():
    e=good_evidence();e['box_distance_m']=6.
    for result in (decide(evidence=e),decide(findings=['DYNAMIC_BOX_NEARBY_UNVERIFIED'])):
        assert not result['selected'] and 'DYNAMIC_BOX_PROXIMITY_UNVERIFIED' in result['reasons']


def test_nonfree_motion_without_a_supported_static_obstacle_stays_on_hold():
    e=good_evidence();e['all_free_run']=False
    assert 'NONFREE_MODE_WITHOUT_STATIC_CONE_CONTEXT' in decide(evidence=e)['reasons']


@pytest.mark.parametrize('field,value',[
    ('coverage_ok',False),('teacher_ok',False),('perception_ok',False),('pose_geometry_ok',False),
    ('map_median_max_m',.151),('map_inlier_fraction_min',.69),('wall_points_min',79),
    ('scan_span_min_rad',1.),('map_median_max_m',float('nan')),('map_inlier_fraction_min',1.1),
    ('scan_span_min_rad',8.),('map_median_max_m',-.1)])
def test_missing_or_bad_window_evidence_cannot_be_treated_as_clear(field,value):
    e=good_evidence();e[field]=value
    assert not decide(evidence=e)['selected']
    assert not decide(evidence={})['selected']


def test_nonfinite_incomplete_reverse_and_wrong_unit_shapes_are_rejected():
    assert decide(velocity_mask=np.zeros(30,bool))['disposition']=='exclude'
    assert decide(velocity_mps=-np.ones(30))['disposition']=='exclude'
    assert decide(xy_m=np.full((30,2),np.nan))['disposition']=='exclude'
    with pytest.raises(ValueError,match='SHAPE'):decide(xy_m=np.zeros((2,30)))
    with pytest.raises(ValueError,match='DTYPE'):decide(xy_mask=np.ones(30))


def test_closed_window_includes_adjacent_support_and_does_not_bridge_missing_frames():
    t=np.arange(0,1_000_000_001,50_000_000,dtype=np.int64)
    assert window_indices(t,100_000_000,200_000_000,150_000_000)==(2,5)
    assert window_indices(t,110_000_000,190_000_000,150_000_000)==(2,5)
    assert window_indices(t,0,1_100_000_000,150_000_000) is None
    assert window_indices(t[t!=150_000_000],110_000_000,190_000_000,50_000_000) is None
    with pytest.raises(ValueError):window_indices(np.array([1,1],np.int64),1,1,10)
    with pytest.raises(ValueError):window_indices(np.array([1.,2.]),1,2,10)
    with pytest.raises(ValueError):window_indices(t,2,1,10)


def test_future_only_bad_scan_or_command_is_included_in_the_anchor_window():
    from tools.curate_native_teacher_data import summarize_window
    cfg=NativeCurationConfig()
    t=np.arange(0,6_000_000_001,50_000_000,dtype=np.int64)
    n=len(t)
    series=dict(pose_t=t,scan_t=t,command_t=t,perception_t=t,
        box_distance_m=np.full(n,9.),cone_distance_m=np.full(n,8.),cone_gap_m=np.full(n,2.),
        registration=np.tile([1.,.05,.9,150.,2.],(n,1)),command=np.ones((n,2),int),ready=np.ones(n,bool))
    lo,hi=950_000_000,5_050_000_000
    assert decide(evidence=summarize_window(series,lo,hi,cfg))['selected']
    series['registration'][80,1]=.4  # 4 s: two seconds after the hypothetical camera anchor.
    assert not decide(evidence=summarize_window(series,lo,hi,cfg))['selected']
    series['registration'][80,1]=.05
    series['command'][80,0]=int(teacher_command_is_usable(
        'FREE_RUN',False,'mppi_brain:ordinary_hold:mppi_brain:infeasible_braking_fallback'))
    result=decide(evidence=summarize_window(series,lo,hi,cfg))
    assert not result['selected'] and 'WINDOW_TEACHER_OK' in result['reasons']


def test_point_polygon_distance_handles_boundary_winding_and_invalid_geometry():
    square=np.array([[-1.,-1.],[1.,-1.],[1.,1.],[-1.,1.]])
    points=np.array([[0.,0.],[1.,0.],[1.3,0.],[2.,2.]])
    for poly in (square,square[::-1]):
        np.testing.assert_allclose(convex_point_distance(points,poly),[0.,0.,.3,math.sqrt(2)])
    with pytest.raises(ValueError):convex_point_distance(np.ones((2,3)),square)
    with pytest.raises(ValueError):convex_point_distance(points,np.zeros((4,2)))
    with pytest.raises(ValueError):convex_point_distance(np.array([[np.nan,0.]]),square)
    with pytest.raises(ValueError):convex_point_distance(points,np.array([[0.,0.],[2.,0.],[.5,.5],[0.,2.]]))


def test_thinning_keeps_source_indices_and_retains_eligibility_of_skipped_frames():
    t=np.arange(6,dtype=np.int64)*105_000_000;mask=np.array([False,True,True,True,True,True])
    np.testing.assert_array_equal(spaced_indices(t,mask,200_000_000),[1,3,5])
    assert mask.sum()==5
    assert len(spaced_indices(t,np.zeros(6,bool),200_000_000))==0
    with pytest.raises(ValueError):spaced_indices(t.astype(float),mask,200_000_000)


@pytest.mark.parametrize('changes',[
    {'max_evidence_gap_ns':0},{'minimum_future_speed_mps':float('nan')},
    {'minimum_anchor_spacing_ns':.2},{'minimum_map_inlier_fraction':1.1},
    {'minimum_scan_span_rad':7.},{'minimum_wall_points':True},
    {'horizon_ns':1_000_000_000},{'history_ns':100_000_000},{'endpoint_guard_ns':1},
    {'minimum_clearance_m':.1}])
def test_curation_units_and_configuration_are_explicit(changes):
    with pytest.raises(ValueError):replace(NativeCurationConfig(),**changes)


def test_documented_direct_cli_entrypoint_imports_with_only_src_on_pythonpath():
    import os
    from pathlib import Path
    import subprocess
    import sys
    root=Path(__file__).resolve().parents[1]
    env=dict(os.environ,PYTHONPATH=str(root/'src'))
    result=subprocess.run([sys.executable,str(root/'tools/curate_native_teacher_data.py'),'--help'],
                          cwd=root,env=env,text=True,capture_output=True,timeout=30)
    assert result.returncode==0,result.stderr
    assert '--root' in result.stdout and '--output' in result.stdout
