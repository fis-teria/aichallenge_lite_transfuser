import numpy as np
import pytest
import csv

from aic_transfuser_lite.data.time_recovery_collection_v1 import (
    PhaseWindow, TARGET_MPS, phase_at_s, project_course, recovery_teacher_mask, validate_nominal, load_pose_course,
)


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
    ("target_mps", .75, "FIXED_SPEED"), ("steering_input_rad", .501, "STEERING"),
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
    write(qw=.5)
    with pytest.raises(ValueError, match='QUATERNION'):
        load_pose_course(path)
