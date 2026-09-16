import csv
from dataclasses import asdict
import hashlib
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest

from aic_transfuser_lite.data.recovery_reference_v3 import MpcReferencePointV3, OccupancyMapV3, _recompute_geometry
from aic_transfuser_lite.data.time_recovery_collection_v1 import reference_rows_with_wrap
from aic_transfuser_lite.data.time_large_recovery_v1 import SCHEMA, LargeRecoveryConfig, LargeRecoverySite
from aic_transfuser_lite.data.time_large_recovery_reference_v1 import preparation_course, validate_large_reference


def fixture():
    xy = np.r_[np.column_stack((np.arange(0.,160.,.5),np.zeros(320))),
               np.column_stack((np.full(200,160.),np.arange(0.,100.,.5))),
               np.column_stack((np.arange(160.,0.,-.5),np.full(320,100.))),
               np.column_stack((np.zeros(200),np.arange(100.,0.,-.5)))]
    s,yaw,kappa = _recompute_geometry(*xy.T)
    base = tuple(MpcReferencePointV3(float(a),float(b),float(c),float(d),float(e),5/3.6,0.)
                 for a,b,c,d,e in zip(s,*xy.T,yaw,kappa))
    ns = np.arange(0.,s[-1],.1)
    normal = np.column_stack((ns,np.interp(ns,s,xy[:,0]),np.interp(ns,s,xy[:,1]),
                              np.interp(ns,s,np.unwrap(yaw)),np.full(len(ns),1.25),np.zeros(len(ns))))
    occupancy = OccupancyMapV3(np.ones((1300,1900),dtype=bool),.1,-10.,-10.)
    cfg = LargeRecoveryConfig((LargeRecoverySite('P00',60.,.6),))
    return base,normal,occupancy,cfg


def write_bundle(tmp_path):
    base,normal,occupancy,cfg = fixture()
    points,evidence = preparation_course(base,normal,cfg,occupancy)
    path = tmp_path/'left_preparation.csv'
    with path.open('w',newline='') as f:
        writer=csv.writer(f);writer.writerow(['s_m','x_m','y_m','psi_rad','kappa_radpm','vx_mps','ax_mps2'])
        writer.writerows(reference_rows_with_wrap(points))
    reference=dict(intervals=[],signed_offset_m=0.,baseline_xy_m=[[p.x_m,p.y_m] for p in base])
    reference['reference_xy_m']=reference['baseline_xy_m']
    reference['large_recovery']=dict(schema=SCHEMA,config=asdict(cfg),nominal_guide=normal[:,[0,5,3]].tolist(),
        preparation_csv=path.name,preparation_xy_m=[[p.x_m,p.y_m] for p in points],map_screen_pass=True,
        preparation_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    (tmp_path/'left.json').write_text(json.dumps(reference))
    return reference


def test_preparation_holds_offset_beyond_handover_without_previewing_return():
    base,normal,occupancy,cfg = fixture()
    points,evidence = preparation_course(base,normal,cfg,occupancy)
    xy=np.asarray([[p.x_m,p.y_m] for p in points])
    bottom=xy[xy[:,1]<1.]
    for x in (58.,60.,62.,70.,72.):
        nearest=bottom[np.argmin(abs(bottom[:,0]-x))]
        assert nearest == pytest.approx([x,.6],abs=1e-6)
    assert all(r['preparation_map_pass'] and r['candidate_return_map_pass'] for r in evidence)
    assert all(r['hidden_return_preview_arc_m'] >= 10. for r in evidence)
    assert len(points) < len(base)+500  # Only the changed patch is densified.
    assert all(p.vx_mps == 5/3.6 for p in points)


def test_actual_map_obstacle_blocks_preparation_generation():
    base,normal,occupancy,cfg=fixture()
    occupancy.free[1299-100,650]=False  # x=55 m, y=0 m within driven preparation footprint.
    with pytest.raises(ValueError,match='LARGE_SITE_MAP_REJECTED'):
        preparation_course(base,normal,cfg,occupancy)


def test_four_metre_settling_patch_holds_before_the_same_release_location():
    from dataclasses import replace
    base,normal,occupancy,cfg=fixture()
    cfg=replace(cfg,sites=(replace(cfg.sites[0],settle_distance_m=4.),))
    points,evidence=preparation_course(base,normal,cfg,occupancy)
    xy=np.asarray([[p.x_m,p.y_m] for p in points]);bottom=xy[xy[:,1]<1.]
    for s in (56.,58.,60.,62.,70.):
        np.testing.assert_allclose(bottom[np.argmin(abs(bottom[:,0]-s))],[s,.6],atol=1e-6,rtol=0.)
    assert all(e['preparation_map_pass'] and e['candidate_return_map_pass'] for e in evidence)


@pytest.mark.parametrize('offset', [-.2, .2])
def test_command_origin_does_not_command_measured_tracking_error_again(offset):
    from dataclasses import replace
    base,normal,occupancy,cfg=fixture()
    normal=normal.copy();normal[:,2]+=.3;normal[:,5]=.3
    site=replace(cfg.sites[0],target_offset_m=offset,preparation_origin='nominal_path')
    cfg=replace(cfg,sites=(site,))
    points,evidence=preparation_course(base,normal,cfg,occupancy)
    xy=np.asarray([[p.x_m,p.y_m] for p in points]);bottom=xy[xy[:,1]<1.]
    for s,y in ((site.start_s_m,0.),(58.,offset),(60.,offset),(70.,offset)):
        np.testing.assert_allclose(bottom[np.argmin(abs(bottom[:,0]-s))],[s,y],atol=1e-6,rtol=0.)
    assert all(e['preparation_map_pass'] and e['candidate_return_map_pass'] for e in evidence)
    with pytest.raises(ValueError,match='LARGE_SITE_CONTRACT'):
        replace(site,preparation_origin='unverified')


def test_shape_and_missing_measured_support_are_explicit():
    base,normal,occupancy,cfg=fixture()
    with pytest.raises(ValueError,match='TRACE_SHAPE'):
        preparation_course(base,normal[:,:5],cfg,occupancy)
    with pytest.raises(ValueError,match='PATCH_COVERAGE'):
        preparation_course(base,normal[600:],cfg,occupancy)


def test_static_bundle_validates_geometry_hash_and_continuous_measured_guide(tmp_path):
    ref=write_bundle(tmp_path)
    cfg,guide=validate_large_reference(ref,tmp_path)
    assert cfg.event_cap==1 and guide.shape[1]==3
    bad=json.loads(json.dumps(ref));bad['large_recovery']['nominal_guide'][2][0]=bad['large_recovery']['nominal_guide'][1][0]
    with pytest.raises(ValueError,match='GUIDE_COVERAGE'):
        validate_large_reference(bad,tmp_path)
    (tmp_path/'left_preparation.csv').write_text('changed')
    with pytest.raises(ValueError,match='SHA_MISMATCH'):
        validate_large_reference(ref,tmp_path)


def test_preparation_pp_has_separate_topics_and_retains_same_speed_policy(tmp_path):
    write_bundle(tmp_path)
    path=Path(__file__).resolve().parents[1]/'tools/run_time_recovery_nodes.py'
    spec=importlib.util.spec_from_file_location('large_nodes_test',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    commands=dict(generator=['ros2','run','simple_trajectory_generator','simple_trajectory_generator_node',
        '__node:=recovery_teacher_trajectory','trajectory:=/recovery_teacher/trajectory','csv_path:='+str(tmp_path/'left.csv')],
        pure_pursuit=['ros2','launch','pure_pursuit.launch.xml','node_name:=recovery_teacher_pure_pursuit',
            'input_trajectory:=/recovery_teacher/trajectory','output_control_cmd:=/recovery_teacher/nominal_control_cmd',
            'output_raw_control_cmd:=/recovery_teacher/raw_control_cmd','external_target_vel:=1.3888888888888888',
            'speed_proportional_gain:=4.0'])
    result=module.preparation_commands(tmp_path,'left',commands)
    pp=result['preparation_pure_pursuit']
    assert 'output_control_cmd:=/recovery_teacher/preparation_control_cmd' in pp
    assert 'input_trajectory:=/recovery_teacher/preparation_trajectory' in pp
    assert 'speed_proportional_gain:=4.0' in pp
    assert 'external_target_vel:=1.3888888888888888' in pp
    assert not any('/control/command/control_cmd' in a for command in result.values() for a in command)
    assert 'csv_path:='+str(tmp_path/'left_preparation.csv') in result['preparation_generator']


def test_preparation_bias_changes_command_geometry_but_not_the_observed_goal():
    from dataclasses import replace
    import math
    from aic_transfuser_lite.data.time_large_recovery_reference_v1 import preparation_lateral
    site=LargeRecoverySite('C01',60.,.6,target_heading_rad=math.radians(4.),
        heading_tolerance_rad=math.radians(1.),corner_id='C01')
    tuned=replace(site,preparation_offset_bias_m=-.05,preparation_heading_bias_rad=math.radians(1.))
    x=np.array([site.start_s_m,60.-1e-4,60.,60.+1e-4])
    y=preparation_lateral(x,tuned)
    assert y[0]==0. and y[2]==pytest.approx(.55)
    assert (y[3]-y[1])/2e-4==pytest.approx(math.tan(math.radians(5.)),abs=1e-5)
    assert tuned.at_goal(.6,math.radians(4.)) == site.at_goal(.6,math.radians(4.))
    assert not tuned.at_goal(.50,math.radians(4.))
    for change in ({'preparation_offset_bias_m':.11},{'preparation_heading_bias_rad':float('nan')},
                   {'preparation_heading_bias_rad':math.radians(3.)}):
        with pytest.raises(ValueError,match='LARGE_SITE_CONTRACT'):replace(site,**change)


def test_oriented_map_screen_is_explicit_and_legacy_default_remains_circle():
    from dataclasses import replace
    base,normal,occupancy,cfg=fixture()
    assert cfg.map_screen_policy=='circle_1p4_v1'
    points,evidence=preparation_course(base,normal,replace(cfg,map_screen_policy='oriented_body_v1'),occupancy)
    assert points and evidence[0]['map_screen_policy']=='oriented_body_v1'
    assert evidence[0]['body_in_base_link_m']['half_width']==.85
    assert not evidence[0]['physical_dynamic_recovery_proven']
    occupancy.free[1299-100,650]=False
    with pytest.raises(ValueError,match='LARGE_SITE_MAP_REJECTED'):
        preparation_course(base,normal,replace(cfg,map_screen_policy='oriented_body_v1'),occupancy)
    with pytest.raises(ValueError,match='LARGE_MAP_SCREEN_POLICY'):
        replace(cfg,map_screen_policy='ignore_obstacles')
