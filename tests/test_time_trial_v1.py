from dataclasses import replace
import json
import math
from pathlib import Path

import numpy as np
import pytest

from aic_transfuser_lite.control.time_reference_v1 import TimePlan, TimedBodyPose, prepare_time_reference
from aic_transfuser_lite.control.time_trial_v1 import interpolate_body_pose, time_trial_control


def pose(stamp=0, x=0., y=0., yaw=0.):
    return TimedBodyPose(stamp, "sim", "0", "map", "base_link", x, y, yaw)


def plan(speed=.15):
    return TimePlan("p", pose(), np.column_stack((np.arange(1,31) * .1 * speed, np.zeros(30))))


def test_calibrated_origin_changes_only_tracking_coordinates_not_source_speed():
    p = plan(1.)
    current = pose(230_000_000, .18)
    a = prepare_time_reference(p, current, rear_axle_offset_m=(0., 0.))
    b = prepare_time_reference(p, current, rear_axle_offset_m=(.001, 0.))
    np.testing.assert_allclose(b.xy_current_m, a.xy_current_m - [.001, 0.])
    assert a.target_speed_mps == pytest.approx(1.) == b.target_speed_mps
    assert b.source_body_frame == "base_link" and b.tracking_frame == "rear_axle"
    with pytest.raises(ValueError, match="EXPLICIT_REAR"):
        prepare_time_reference(p, current)


def test_short_launch_stationary_brake_cap_and_fractional_age():
    launch = time_trial_control(plan(), pose(), speed_mps=0., rear_axle_offset_m=(.001, 0.))
    assert launch["acceleration_mps2"] > 0 and launch["steer_rad"] == 0
    assert launch["target_speed_mps"] == pytest.approx(.15)
    stop = time_trial_control(plan(0.), pose(), speed_mps=.1, rear_axle_offset_m=(.001, 0.))
    assert stop["target_speed_mps"] == 0 and stop["acceleration_mps2"] < 0
    capped = time_trial_control(plan(1.), pose(230_000_000, .2), speed_mps=.1, rear_axle_offset_m=(.001, 0.))
    assert capped["target_speed_mps"] == .25 and capped["predicted_source_speed_mps"] == pytest.approx(1.)
    assert capped["plan_age_sec"] == pytest.approx(.23)


def test_pose_interpolation_wrap_and_no_nearest_relabeling():
    a = pose(100_000_000, 0., yaw=math.pi-.1)
    b = pose(150_000_000, .1, yaw=-math.pi+.1)
    middle = interpolate_body_pose([b, a], 125_000_000)
    assert middle.x_m == pytest.approx(.05) and abs(middle.yaw_rad) == pytest.approx(math.pi)
    with pytest.raises(ValueError, match="MISSING"):
        interpolate_body_pose([a], 125_000_000)
    with pytest.raises(ValueError, match="IDENTITY"):
        interpolate_body_pose([a, replace(b, epoch="1")], 125_000_000)


@pytest.mark.parametrize("age, speed, offset, reason", [
    (600_000_000, .1, .001, "STALE"), (0, .5, .001, "SPEED"),
    (0, .1, .484, "FUTURE_HEADING"), (0, float("nan"), .001, "SPEED")])
def test_trial_rejects_stale_overspeed_unresolved_body_offset(age, speed, offset, reason):
    with pytest.raises(ValueError, match=reason):
        time_trial_control(plan(), pose(age), speed_mps=speed, rear_axle_offset_m=(offset, 0.))


def test_trial_rejects_discontinuous_path_without_cutting_it():
    xy = plan().xy_m.copy(); xy[15] += [4., 0.]
    p = TimePlan("bad", pose(), xy)
    with pytest.raises(ValueError, match="DISCONTINUITY"):
        time_trial_control(p, pose(), speed_mps=.1, rear_axle_offset_m=(.001, 0.))
    np.testing.assert_array_equal(p.xy_m, xy)


def test_recorded_awsim_near_origin_prediction_uses_resolved_geometry_without_correction():
    """Trial 03 failed on a centimetre-scale first step, preserved verbatim."""
    fixture = json.loads((Path(__file__).parent / "fixtures/time_path/near_origin_foldback.json").read_text())
    xy = np.array(fixture["raw_xy_m"], dtype=np.float64)
    observed = pose(fixture["observation_ns"])
    p = TimePlan(fixture["plan_id"], observed, xy)
    current = replace(observed, stamp_ns=observed.stamp_ns + 150_000_000)
    command = time_trial_control(p, current, speed_mps=0., rear_axle_offset_m=(.0010000169277191162, 0.))
    assert 0 < command["target_speed_mps"] <= .25
    assert command["acceleration_mps2"] > 0
    assert command["geometry"]["raw_points_checked"] == 31
    np.testing.assert_array_equal(p.xy_m, xy)


def test_stationary_prediction_noise_never_authorizes_motion():
    xy = np.column_stack((np.full(30, .012), np.tile([.01, -.01], 15)))
    with pytest.raises(ValueError, match="TIME_PATH_MOTION_UNRESOLVED"):
        time_trial_control(TimePlan("noise", pose(), xy), pose(), speed_mps=0., rear_axle_offset_m=(.001, 0.))
