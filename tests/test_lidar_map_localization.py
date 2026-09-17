import math

import numpy as np
import pytest

from aic_transfuser_lite.runtime.lidar_map_localization import (
    BoundaryMap, MapLocalizer, compose, inverse, match_scan, scan_points, transform,
)


def fixture():
    segments = np.array([[[0., -3.], [12., -3.]], [[12., -3.], [12., 4.]],
                         [[12., 4.], [0., 4.]], [[0., 4.], [0., -3.]]])
    world = np.concatenate([a+np.linspace(.05, .95, 70)[:, None]*(b-a) for a,b in segments])
    return BoundaryMap(segments+np.array([89600., 43100.])), world+np.array([89600., 43100.])


def observed(world, pose):
    return transform(world, inverse(pose))-[1.65, 0.]


def test_pose_composition_and_sensor_mount():
    a = np.array([89631., 43128., 2.1]); b = np.array([2., -.3, -.4])
    np.testing.assert_allclose(compose(a, inverse(a)), 0., atol=1e-10)
    np.testing.assert_allclose(transform(transform(np.array([[1., 2.]]), b), a),
                               transform(np.array([[1., 2.]]), compose(a,b)))


def test_known_xy_yaw_registration_with_large_map_coordinates_and_outliers():
    course, world = fixture(); true = np.array([89603., 43101., .15])
    points = np.r_[observed(world, true), np.full((12, 2), 50.)]
    result = match_scan(course, points, true+[.25, -.2, .035], initializing=True)
    assert result.accepted and result.rank == 3
    np.testing.assert_allclose(result.pose, true, atol=.02)
    assert result.mean_distance_m < .01


def test_parallel_walls_do_not_invent_longitudinal_correction():
    course = BoundaryMap(np.array([[[-100., -3.], [100., -3.]], [[-100., 3.], [100., 3.]]]))
    points = np.concatenate([np.column_stack((np.linspace(2., 12., 100), np.full(100,y))) for y in [-3.,3.]])
    result = match_scan(course, points-[1.65,0], np.array([.4,.2,.02]), initializing=True)
    assert result.accepted and result.reason == 'DEGRADED'
    assert result.pose[0] == pytest.approx(.4, abs=.002)
    assert abs(result.pose[1]) < .01 and abs(result.pose[2]) < .002


def test_wheel_prior_is_continuous_while_map_to_odom_corrects_drift():
    course, world = fixture(); tracker = MapLocalizer(course)
    start = np.array([89602., 43100., 0.])
    tracker.initialize(start+[.1,-.1,.01])
    for i in range(20):
        truth = start + [i*.15, 0, 0]
        wheel = np.array([i*.14, 0., 0.])
        wheel_before = wheel.copy()
        result = tracker.update(1_000_000_000+i*200_000_000, wheel, observed(world,truth))
        assert result.accepted
        np.testing.assert_equal(wheel, wheel_before)
        np.testing.assert_allclose(result.pose, truth, atol=.03)
    assert tracker.valid(4_900_000_000)
    assert tracker.map_to_odom[0]-start[0] == pytest.approx(.19, abs=.03)
    assert not tracker.valid(5_400_000_000)
    assert tracker.update(6_000_000_000,wheel,observed(world,truth)) is None
    assert tracker.status == 'LOST_REINITIALIZE' and tracker.map_to_odom is None


def test_missing_seed_bad_scan_and_clock_reset_do_not_publish_valid_pose():
    course, world = fixture(); tracker = MapLocalizer(course)
    p = np.array([89603.,43100.,0.]); cloud = observed(world,p)
    assert tracker.update(100,np.zeros(3),cloud) is None
    tracker.initialize(p)
    for i in range(3): tracker.update(100+i,np.zeros(3),cloud)
    assert tracker.valid(102)
    result = tracker.update(103,np.zeros(3),np.zeros((2,2)))
    assert not result.accepted and not tracker.valid(103)
    assert tracker.update(100,np.zeros(3),cloud) is None
    assert tracker.status == 'CLOCK_RESET' and tracker.map_to_odom is None


def test_filtering_shapes_units_and_map_validation():
    points = scan_points(np.array([1., np.nan, np.inf, 0., -1., 30., 2., 3., 4.]),0.,.1,0.,25.)
    assert points.shape == (2,2) and np.isfinite(points).all()
    np.testing.assert_allclose(points[0],[1.,0.])
    with pytest.raises(ValueError): scan_points(np.zeros((3,2)),0.,.1,0.,25.)
    with pytest.raises(ValueError): scan_points(np.ones(4),0.,-.1,0.,25.)
    with pytest.raises(ValueError): transform(np.ones((3,3)),np.zeros(3))
    with pytest.raises(ValueError): compose([0,0,math.nan],np.zeros(3))
    with pytest.raises(ValueError): BoundaryMap(np.zeros((2,2,2)))


def test_minority_wall_support_cannot_validate_a_cluttered_scan():
    course, world = fixture(); p = np.array([89603.,43101.,.1])
    cloud = np.r_[observed(world,p),np.full((400,2),50.)]
    result = match_scan(course,cloud,p)
    assert not result.accepted and result.reason == 'INSUFFICIENT_SUPPORT'
    assert result.fraction < .55
    assert result.all_mean_distance_m > result.mean_distance_m


def test_gentle_bend_retains_weak_longitudinal_evidence():
    xs=np.linspace(0,18,19)
    segments=[]
    for y in [-3.,3.]:
        line=np.column_stack((xs,y+.001*(xs-9)**3))
        segments.extend(zip(line[:-1],line[1:]))
    course=BoundaryMap(np.array(segments))
    world=np.concatenate([a+np.linspace(.1,.9,8)[:,None]*(b-a) for a,b in segments])
    truth=np.array([1.,0.,0.])
    result=match_scan(course,observed(world,truth),truth+[.3,.1,.01],initializing=True)
    assert result.accepted
    assert abs(result.pose[0]-truth[0])<.08


def test_lanelet_loader_uses_only_explicit_boundary_nodes(tmp_path):
    path = tmp_path/'map.osm'
    path.write_text('''<osm><node id="1"><tag k="local_x" v="1"/><tag k="local_y" v="2"/></node>
    <node id="2"><tag k="local_x" v="3"/><tag k="local_y" v="2"/></node>
    <node id="3"><tag k="local_x" v="3"/><tag k="local_y" v="4"/></node>
    <way id="10"><nd ref="1"/><nd ref="2"/><nd ref="3"/></way>
    <relation><member type="way" role="left" ref="10"/></relation></osm>''')
    course = BoundaryMap.from_lanelet(path)
    assert course.segments.shape == (2,2,2)
    np.testing.assert_allclose(course.origin,[1,2])


def test_bounded_relocalization_requires_confirmation_before_output():
    course, world=fixture();tracker=MapLocalizer(course);p=np.array([89603.,43101.,.1])
    tracker.initialize(p)
    for i in range(3):tracker.update(1_000_000_000+i*200_000_000,np.zeros(3),observed(world,p))
    assert tracker.valid(1_400_000_000)
    # A corner reveals 0.6m accumulated dead-reckoning error.
    bad_wheel=np.array([.6,0,0])
    result=tracker.update(1_600_000_000,bad_wheel,observed(world,p))
    assert result.accepted and not tracker.valid(1_600_000_000)
    assert tracker.status=='INITIALIZING'
    tracker.update(1_800_000_000,bad_wheel,observed(world,p))
    assert not tracker.valid(1_800_000_000)
    tracker.update(2_000_000_000,bad_wheel,observed(world,p))
    assert tracker.valid(2_000_000_000)


def test_unlimited_accepts_large_correction_without_changing_fit():
    course,world=fixture(); truth=np.array([89603.,43101.,.1])
    cloud=observed(world,truth);predicted=truth+[.7,0.,0.]
    bounded=match_scan(course,cloud,predicted)
    unlimited=match_scan(course,cloud,predicted,correction_mode='unlimited')
    assert bounded.reason=='CORRECTION_LIMIT'
    assert unlimited.accepted and unlimited.correction_m>.45
    np.testing.assert_allclose(bounded.pose,unlimited.pose)


def test_aggressive_large_map_offset_and_insufficient_support():
    course,world=fixture();truth=np.array([89603.,43101.,.1]);cloud=observed(world,truth)
    result=match_scan(course,cloud,truth+[1.5,.7,.12],correction_mode='simulation_aggressive')
    assert result.accepted and result.correction_m>1.
    np.testing.assert_allclose(result.pose,truth,atol=.03)
    bad=match_scan(course,np.r_[cloud,np.full((500,2),50.)],truth,correction_mode='simulation_aggressive')
    assert not bad.accepted and bad.reason=='INSUFFICIENT_SUPPORT'


def test_aggressive_auto_recovery_requires_consecutive_matches():
    course,world=fixture();truth=np.array([89603.,43101.,.1]);cloud=observed(world,truth)
    tracker=MapLocalizer(course,correction_mode='simulation_aggressive');tracker.initialize(truth)
    for t in [1_000_000_000,1_200_000_000,1_400_000_000]:
        tracker.update(t,np.zeros(3),cloud)
    assert tracker.valid(1_400_000_000)
    for t in [2_600_000_000,2_800_000_000]:
        tracker.update(t,np.array([1.5,.1,0.]),cloud)
        assert not tracker.valid(t)
    tracker.update(3_000_000_000,np.array([1.5,.1,0.]),np.zeros((2,2)))
    for t in [3_200_000_000,3_400_000_000]:
        tracker.update(t,np.array([1.5,.1,0.]),cloud)
        assert not tracker.valid(t)
    tracker.update(3_600_000_000,np.array([1.5,.1,0.]),cloud)
    assert tracker.valid(3_600_000_000)
    assert tracker.update(1_000_000_000,np.zeros(3),cloud) is None
    assert tracker.status=='CLOCK_RESET'


def test_unknown_correction_mode_is_rejected():
    course,world=fixture()
    with pytest.raises(ValueError,match='CORRECTION_MODE'):
        MapLocalizer(course,correction_mode='anything')
    with pytest.raises(ValueError,match='CORRECTION_MODE'):
        match_scan(course,np.zeros((40,2)),np.zeros(3),correction_mode='anything')
