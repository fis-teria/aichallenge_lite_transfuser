import numpy as np
import pytest
import csv

from aic_transfuser_lite.data.time_recovery_collection_v1 import (
    PhaseWindow, TARGET_MPS, phase_at_s, project_course, recovery_teacher_mask, validate_nominal, load_pose_course,
    bounded_collection_command, reference_rows_with_wrap, check_collection_input_time, select_collection_input,
    select_collection_motion,
    check_collection_decision_age, collection_snapshot_retry_allowed,
    validate_collection_imu_axes, collection_imu_yaw_rate,
    collection_speed_gain, validate_collection_speed_parameters,
)


@pytest.mark.parametrize('policy,gain', [('legacy_gain1_v1', 1.), ('aligned_gain4_v1', 4.)])
def test_collection_loaded_speed_contract_before_authority(policy, gain):
    params = dict(use_external_target_vel=True, external_target_vel=5/3.6, speed_proportional_gain=gain)
    validate_collection_speed_parameters(policy, params)
    assert collection_speed_gain(policy) == gain
    for field, value in [('use_external_target_vel', False), ('external_target_vel', 5.),
                         ('external_target_vel', float('nan')), ('speed_proportional_gain', 2.),
                         ('speed_proportional_gain', True), ('speed_proportional_gain', None)]:
        with pytest.raises(ValueError, match='COLLECTION_'):
            validate_collection_speed_parameters(policy, {**params, field: value})
    with pytest.raises(ValueError, match='SPEED_POLICY'):
        collection_speed_gain('unbounded')


def test_aligned_collection_and_e2e_longitudinal_command_use_same_si_limits():
    from aic_transfuser_lite.control.waypoint_controller import ControllerConfig, control_from_waypoints
    # The same measured speed isolates the gain/units/clamp from sensor timing.
    for speed in (0., .5, 1., 1.27, TARGET_MPS, 1.5):
        _, actual = bounded_collection_command(.1, collection_speed_gain('aligned_gain4_v1')*(TARGET_MPS-speed), 0., .05)
        expected = control_from_waypoints(np.array([[1., 0.], [2., 0.]]), TARGET_MPS, speed,
            ControllerConfig(speed_kp=4., min_accel_mps2=-1., max_accel_mps2=1.))
        assert actual == pytest.approx(expected.acceleration_mps2)


def test_phase_mask_excludes_hold_and_preserves_future_barrier():
    windows = [PhaseWindow(0, 2_000_000_000, "baseline"),
               PhaseWindow(2_000_000_000, 3_000_000_000, "approach"),
               PhaseWindow(3_000_000_000, 5_000_000_000, "hold"),
               PhaseWindow(5_000_000_000, 9_000_000_000, "recovery"),
               PhaseWindow(9_000_000_000, 12_000_000_000, "baseline"),
               PhaseWindow(12_000_000_000, 13_000_000_000, "braking")]
    mask = recovery_teacher_mask(1_000_000_000, windows)
    assert mask.dtype == np.bool_ and mask.shape == (30,)
    assert mask[:9].all() and not mask[9:].any()
    assert not recovery_teacher_mask(3_000_000_000, windows).any()
    assert recovery_teacher_mask(5_000_000_000, windows).all()
    assert recovery_teacher_mask(8_000_000_000, windows).all()
    assert not recovery_teacher_mask(12_000_000_000, windows).any()


def test_missing_phase_support_and_overlaps_fail_closed():
    assert not recovery_teacher_mask(0, []).any()
    windows = [PhaseWindow(0, 500_000_000, "recovery"), PhaseWindow(600_000_000, 4_000_000_000, "baseline")]
    mask = recovery_teacher_mask(0, windows)
    assert mask[:4].all() and not mask[4:].any()
    with pytest.raises(ValueError, match="OVERLAP"):
        recovery_teacher_mask(0, [PhaseWindow(0, 20, "baseline"), PhaseWindow(10, 30, "recovery")])


def test_projection_sign_direction_and_spatial_boundaries():
    course = np.array([[0., 0.], [10., 0.], [10., 10.], [0., 10.]])
    assert project_course(course, [4., .2], 0.)["offset_m"] == pytest.approx(.2)
    assert project_course(course, [4., -.2], 0.)["offset_m"] == pytest.approx(-.2)
    with pytest.raises(ValueError, match="TOO_FAR"):
        project_course(course, [4., 50.], 0.)
    intervals = [dict(start_s_m=2., end_s_m=4., phase="approach"),
                 dict(start_s_m=4., end_s_m=6., phase="hold"),
                 dict(start_s_m=6., end_s_m=8., phase="recovery")]
    assert phase_at_s(4., intervals) == "hold"
    assert phase_at_s(6., intervals) == "recovery"
    assert phase_at_s(8., intervals) == "baseline"


def nominal():
    return dict(stamp_ns=1_000_000_000, now_ns=1_050_000_000, received_ns=2_000_000_000,
                now_wall_ns=2_100_000_000, target_mps=TARGET_MPS, acceleration_mps2=.5,
                steering_input_rad=.1, measured_speed_mps=1.2)


@pytest.mark.parametrize("field,value,reason", [
    ("stamp_ns", 800_000_000, "STALE"), ("stamp_ns", 1_071_000_000, "FUTURE"),
    ("now_wall_ns", 2_300_000_001, "STALE"), ("received_ns", 2_100_000_001, "FUTURE"),
    ("target_mps", .75, "FIXED_SPEED"), ("steering_input_rad", .641, "STEERING"),
    ("measured_speed_mps", 1.7, "OVERSPEED"), ("acceleration_mps2", float("nan"), "NONFINITE"),
])
def test_nominal_freshness_speed_and_angle(field, value, reason):
    settings = nominal()
    validate_nominal(**settings)
    settings[field] = value
    with pytest.raises(ValueError, match=reason):
        validate_nominal(**settings)


def test_pose_course_keeps_positions_and_replaces_speed(tmp_path):
    path = tmp_path/'course.csv'
    def write(qw=1.):
        with path.open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['x','y','z','x_quat','y_quat','z_quat','w_quat','speed'])
            writer.writerows([10*np.cos(t),10*np.sin(t),0,0,0,0,qw,4.1666666667]
                             for t in np.linspace(0,2*np.pi,21))
    write()
    points = load_pose_course(path)
    assert len(points) == 20  # Closing duplicate is not a zero-length segment.
    assert points[0].x_m == 10.
    assert all(p.vx_mps == TARGET_MPS and p.ax_mps2 == 0. for p in points)
    assert all(b.s_m > a.s_m for a,b in zip(points,points[1:]))
    wrapped = np.asarray(reference_rows_with_wrap(points))
    assert wrapped.ndim == 2 and wrapped.shape[1] == 7
    np.testing.assert_allclose(wrapped[20,1:3], wrapped[0,1:3])
    assert (np.diff(wrapped[:,0]) > 0).all()
    assert wrapped[-1,0]-wrapped[20,0] >= 12.
    assert np.allclose(np.linalg.norm(np.diff(wrapped[:,1:3],axis=0),axis=1), 20*np.sin(np.pi/20))
    assert np.all(wrapped[:,5] == TARGET_MPS)
    write(qw=.5)
    with pytest.raises(ValueError, match='QUATERNION'):
        load_pose_course(path)


def test_official_nominal_bound_keeps_existing_final_actuator_limits():
    settings = nominal(); settings['steering_input_rad'] = -.64
    validate_nominal(**settings)
    assert bounded_collection_command(-.64, 1.3889, 0., .05) == pytest.approx((-.04, 1.))
    assert bounded_collection_command(-.64, -1.5, -.48, .05) == pytest.approx((-.5, -1.))
    assert bounded_collection_command(.64, .3, .48, .05) == pytest.approx((.5, .3))
    with pytest.raises(ValueError, match='CONTRACT'):
        bounded_collection_command(.1, .2, .51, .05)


def test_camera_uses_timepath_budget_while_control_state_keeps_150ms():
    clocks=dict(capture_ns=1_000_000_000,receipt_ns=2_000_000_000,
                now_sim_ns=1_180_000_000,now_wall_ns=2_180_000_000)
    check_collection_input_time('camera', **clocks)
    for role in ('pose','velocity','steering','scan','nominal'):
        with pytest.raises(ValueError,match='STALE_'+role):
            check_collection_input_time(role,**clocks)
    for field,value in [('now_sim_ns',1_500_000_001),('now_wall_ns',2_500_000_001),
                        ('now_sim_ns',979_999_999)]:
        with pytest.raises(ValueError,match='STALE_camera'):
            check_collection_input_time('camera',**dict(clocks,**{field:value}))


def test_inflight_future_nominal_keeps_fresh_original_sample_until_clock_catches_up():
    # r06: publisher's 139.225 s command arrived before collector /clock=139.200 s.
    rows=[(139_195_000_000,100_000_000), (139_225_000_000,166_000_000)]
    assert select_collection_input('nominal',rows,now_sim_ns=139_200_000_000,now_wall_ns=167_000_000)==0
    assert select_collection_input('nominal',rows,now_sim_ns=139_230_000_000,now_wall_ns=200_000_000)==1
    assert select_collection_input('nominal',rows,now_sim_ns=139_400_000_000,now_wall_ns=250_000_000) is None
    assert select_collection_input('nominal',rows,now_sim_ns=139_230_000_000,now_wall_ns=500_000_000) is None
    assert select_collection_input('pose',[],now_sim_ns=1,now_wall_ns=1) is None
    with pytest.raises(ValueError,match='CONTRACT'):
        select_collection_input('nominal',[(-1,0)],now_sim_ns=1,now_wall_ns=1)


def test_motion_snapshot_pairs_history_without_relaxing_capture_or_skew():
    # Independently newest pose and steering disagree; previous measured pose
    # still satisfies both the original capture limit and the velocity skew.
    history = {'pose': [(960_000_000, 1_980_000_000), (1_015_000_000, 1_990_000_000)],
               'velocity': [(950_000_000, 1_985_000_000)],
               'steering': [(955_000_000, 1_985_000_000)]}
    clocks = dict(now_sim_ns=1_000_000_000, now_wall_ns=2_000_000_000)
    assert select_collection_motion(history, **clocks) == {'pose': 0, 'velocity': 0, 'steering': 0}
    history['velocity'].append((995_000_000, 1_995_000_000))
    assert select_collection_motion(history, **clocks) == {'pose': 1, 'velocity': 1, 'steering': 0}
    assert select_collection_motion({}, **clocks) is None
    assert select_collection_motion(history, **dict(clocks, now_sim_ns=1_200_000_000)) is None
    assert select_collection_motion(history, **dict(clocks, now_wall_ns=2_300_000_001)) is None
    with pytest.raises(ValueError, match='CONTRACT'):
        select_collection_motion({'velocity': [(-1, 0)]}, **clocks)


def test_r07_delayed_velocity_does_not_allow_expired_pose_to_mask_skew():
    # Exact logged r07 stamps: latest velocity is nearly 150 ms old after a
    # callback stall. The previously selected pose is now expired. Merely using
    # the previous control row is unsafe; without another aligned sample stop.
    history = {'pose': [(199_629_995_537, 36_672_731_339_470),
                        (199_714_995_536, 36_672_895_675_317)],
               'velocity': [(199_639_995_537, 36_672_896_074_905)],
               'steering': [(199_639_995_537, 36_672_896_245_582)]}
    clocks = dict(now_sim_ns=199_789_995_534, now_wall_ns=36_672_897_912_439)
    assert select_collection_motion(history, **clocks) is None
    # A synthetic, actually received aligned pose would be admissible, without
    # altering any of the original stamps or admitting the expired pose.
    history['pose'].append((199_674_995_537, 36_672_896_000_000))
    assert select_collection_motion(history, **clocks) == {'pose': 2, 'velocity': 0, 'steering': 0}


def test_scan_alignment_needs_pose_timeline_even_when_latest_motion_is_fresh():
    from aic_transfuser_lite.control.time_reference_v1 import TimedBodyPose
    from aic_transfuser_lite.control.turning_scan_guard import select_aligned_scan
    def pose(t):
        return TimedBodyPose(t, 'sim', '0', 'map', 'base_link', 0., 0., 0.)
    # r08 scan=22.036611470 s lies just over 50 ms after 21.984999508 s.
    # Dropping the intermediate pose breaks bracketing although newest pose
    # and state are fresh. A future scan must not be used instead.
    poses = [pose(21_984_999_508), pose(22_044_999_507), pose(22_084_999_506)]
    scans = [(22_036_611_470, 1_000_000_000), (22_086_616_755, 1_050_000_000)]
    clocks = dict(now_sim_ns=22_099_999_506, now_receipt_ns=1_075_000_000)
    with pytest.raises(ValueError, match='FRESH_ALIGNED_SCAN_MISSING'):
        select_aligned_scan(scans, poses, poses[-1], **clocks)
    # Synthetic intermediate measured sample demonstrates the required queue
    # semantics; this does not fabricate a missing pose during collection.
    poses.insert(1, pose(22_024_999_507))
    index, captured = select_aligned_scan(scans, poses, poses[-1], **clocks)
    assert index == 0 and captured.stamp_ns == scans[0][0]


def test_expired_snapshot_retry_is_bounded_and_does_not_retry_physical_or_reset_faults():
    assert collection_snapshot_retry_allowed('STALE_scan', attempt=0, elapsed_ns=59_000_000)
    assert not collection_snapshot_retry_allowed('STALE_scan', attempt=1, elapsed_ns=60_000_000)
    assert not collection_snapshot_retry_allowed('STALE_scan', attempt=0, elapsed_ns=80_000_001)
    for reason in ('STOPPING_SWEEP_OCCUPIED', 'CLOCK_RESET', 'SOURCE_pose', 'OVERSPEED_OR_REVERSE',
                   'COLLECTION_COMPUTATION_TIMEOUT', 'SCAN_POSE_ALIGNMENT'):
        assert not collection_snapshot_retry_allowed(reason, attempt=0, elapsed_ns=1)
    check_collection_decision_age(started_ns=100, now_ns=100_000_100)
    with pytest.raises(ValueError, match='COMPUTATION_TIMEOUT'):
        check_collection_decision_age(started_ns=100, now_ns=100_000_101)
    with pytest.raises(ValueError, match='COMPUTATION_TIMEOUT'):
        check_collection_decision_age(started_ns=100, now_ns=99)


def test_retry_allows_real_arrival_time_without_extending_computation_budget():
    from aic_transfuser_lite.data.time_recovery_collection_v1 import collection_snapshot_retry_wait_ns
    for elapsed, expected in [(0,20_000_000),(49_000_000,20_000_000),
                              (75_000_000,5_000_000),(80_000_000,0)]:
        assert collection_snapshot_retry_wait_ns(elapsed) == expected
        assert elapsed+expected+20_000_000 <= 100_000_000
    for invalid in (-1,80_000_001,1.5):
        with pytest.raises(ValueError,match='RETRY_WAIT_CONTRACT'):
            collection_snapshot_retry_wait_ns(invalid)


def test_imu_must_be_fresh_and_aligned_with_selected_measured_velocity():
    clocks = dict(now_sim_ns=1_000_000_000, now_wall_ns=2_000_000_000, include_imu=True)
    history = {r:[(990_000_000,1_990_000_000)] for r in ('pose','velocity','steering')}
    assert select_collection_motion(history, **clocks) is None
    history['imu'] = [(939_999_999,1_950_000_000)]
    assert select_collection_motion(history, **clocks) is None
    history['imu'].append((950_000_000,1_950_000_000))
    assert select_collection_motion(history, **clocks) == {'pose':0,'velocity':0,'steering':0,'imu':1}
    history['imu'] = [(990_000_000,1_699_999_999)]
    assert select_collection_motion(history, **clocks) is None


def test_imu_yaw_source_requires_verified_vertical_axes_and_finite_radps():
    transforms = {'imu_link': ('sensor_kit_base_link',(0.,0.,-2**-.5,2**-.5)),
                  'sensor_kit_base_link': ('base_link',(0.,0.,0.,1.))}
    validate_collection_imu_axes(transforms)
    assert collection_imu_yaw_rate([-.00002,-.01362,.1361621916294098], 'imu_link') == .1361621916294098
    with pytest.raises(ValueError, match='AXES_MISSING'):
        validate_collection_imu_axes({})
    for bad in [('wrong_parent',(0.,0.,0.,1.)), ('sensor_kit_base_link',(2**-.5,0.,0.,2**-.5)),
                ('sensor_kit_base_link',(0.,0.,0.,2.))]:
        with pytest.raises(ValueError, match='AXES_NOT_VERTICAL'):
            validate_collection_imu_axes(dict(transforms,imu_link=bad))
    for angular,frame in [([0.,0.,float('nan')],'imu_link'),([0.,0.,.1],'other'),([0.,.1],'imu_link')]:
        with pytest.raises(ValueError, match='RATE_CONTRACT'):
            collection_imu_yaw_rate(angular,frame)
