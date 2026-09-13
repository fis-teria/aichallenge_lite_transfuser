from dataclasses import replace
import json
import math
from pathlib import Path

import numpy as np
import pytest

from aic_transfuser_lite.control.time_reference_v1 import TimePlan, TimedBodyPose, prepare_time_reference
from aic_transfuser_lite.control.time_trial_v1 import (
    interpolate_body_pose, time_trial_control, trial_speed_limits, validate_trial_config,
)


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


@pytest.mark.parametrize("source_speed", [.15, .8, 1.8])
def test_fixed_5kmh_target_does_not_use_source_point_spacing(source_speed):
    p = plan(source_speed)
    result = time_trial_control(p, pose(), speed_mps=0., rear_axle_offset_m=(.001, 0.), speed_policy="fixed_5kmh")
    assert result["target_speed_mps"] * 3.6 == pytest.approx(5.)
    assert result["predicted_source_speed_mps"] == pytest.approx(source_speed)
    assert result["acceleration_mps2"] == 1.
    assert result["steer_rad"] == 0.
    assert result["speed_policy"] == "fixed_5kmh"
    np.testing.assert_array_equal(p.xy_m, plan(source_speed).xy_m)


def test_fixed_speed_brakes_on_overshoot_and_rejects_overspeed_stationary_short_path():
    def control(p=plan(1.), speed=1.5):
        return time_trial_control(p, pose(), speed_mps=speed, rear_axle_offset_m=(.001, 0.), speed_policy="fixed_5kmh")
    assert control()["acceleration_mps2"] < 0
    with pytest.raises(ValueError, match="TRIAL_SPEED_CONTRACT"):
        control(speed=6./3.6 + 1e-6)
    with pytest.raises(ValueError, match="MOTION_UNRESOLVED"):
        control(plan(0.), speed=0.)
    with pytest.raises(ValueError, match="REFERENCE_STOPPING_DISTANCE"):
        control(plan(.5), speed=5./3.6)
    assert control(plan(.6), speed=5./3.6)["target_speed_mps"] == pytest.approx(5./3.6)
    with pytest.raises(ValueError, match="STALE"):
        time_trial_control(plan(1.), pose(600_000_000), speed_mps=1., rear_axle_offset_m=(.001, 0.), speed_policy="fixed_5kmh")


def test_fixed_speed_scan_stopping_distance_scales_with_measured_speed():
    from aic_transfuser_lite.control.long_sim_tracking_v4 import check_scan
    # A finite 1.8 m front clearance permits the old crawl, not 5 km/h.
    ranges = np.full(750, np.inf); ranges[375] = 1.8 + 1.5 - 1.165
    parameters = (ranges, -np.pi, 2*np.pi/750, .1, 25.)
    assert check_scan(*parameters, .25) == pytest.approx(1.8)
    with pytest.raises(ValueError, match="STOPPING_CORRIDOR_OCCUPIED"):
        check_scan(*parameters, 5./3.6)


def test_speed_profiles_reject_mismatched_runtime_configuration():
    root = Path(__file__).resolve().parents[1] / "configs/control"
    old = json.loads((root/"time_path_awsim_trial_20260913.json").read_text())
    fixed = json.loads((root/"time_path_fixed_5kmh_awsim_20260913.json").read_text())
    assert validate_trial_config(old) == "source_capped_0p25"
    assert validate_trial_config(fixed) == "fixed_5kmh"
    assert trial_speed_limits("fixed_5kmh") == pytest.approx((5./3.6, 6./3.6))
    for key, bad in (("speed_cap_mps", .25), ("overspeed_limit_mps", .45),
                     ("drive_limit_sim_s", 100.), ("speed_cap_mps", float("nan"))):
        with pytest.raises(ValueError, match="TRIAL_CONFIG_MISMATCH"):
            validate_trial_config({**fixed, key: bad})
    with pytest.raises(ValueError, match="TRIAL_SPEED_POLICY"):
        validate_trial_config({**fixed, "speed_policy": "fixed_50kmh"})
    with pytest.raises(ValueError, match="TRIAL_SPEED_CONTRACT"):
        time_trial_control(plan(), pose(), speed_mps=0., rear_axle_offset_m=(.001, 0.), speed_policy="fixed_5kmh", speed_cap_mps=.25)


def test_recorded_startup_selects_feasible_original_point_without_changing_path():
    fixture = json.loads((Path(__file__).parent/"fixtures/time_path/startup_actuator_limit.json").read_text())
    observed = TimedBodyPose(**fixture["observation_pose"])
    current = TimedBodyPose(**fixture["current_pose"])
    xy = np.array(fixture["raw_xy_m"])
    p = TimePlan(fixture["plan_id"], observed, xy)
    options = dict(speed_mps=fixture["speed_mps"], rear_axle_offset_m=(.0010000169277191162, 0.), speed_policy="fixed_5kmh")
    old = time_trial_control(p, current, **options)
    assert old["steer_rad"] == pytest.approx(fixture["recorded_required_tire_rad"])
    assert abs(old["steer_rad"]) > .3
    new = time_trial_control(p, current, **options, lookahead_policy="feasible_1_to_1p5m_v1")
    assert abs(new["steer_rad"]) <= .3
    assert 1. <= new["selected_lookahead_distance_m"] <= 1.5
    assert new["target_speed_mps"] == 5./3.6
    np.testing.assert_array_equal(new["reference_xy_rear_m"], old["reference_xy_rear_m"])
    np.testing.assert_array_equal(p.xy_m, xy)
    assert any(np.array_equal(new["lookahead_rear_m"], np.asarray(point, dtype=np.float32))
               for point in new["reference_xy_rear_m"][1:])


@pytest.mark.parametrize("curvature", [-.4, .4])
def test_feasible_lookahead_rejects_unreachable_curve_instead_of_clipping(curvature):
    s = np.arange(1, 31)*.08
    xy = np.column_stack((np.sin(curvature*s)/curvature, (1-np.cos(curvature*s))/curvature))
    with pytest.raises(ValueError, match="FEASIBLE_LOOKAHEAD_MISSING"):
        time_trial_control(TimePlan("curve", pose(), xy), pose(), speed_mps=.1,
            rear_axle_offset_m=(.001, 0.), speed_policy="fixed_5kmh", lookahead_policy="feasible_1_to_1p5m_v1")


def test_feasible_lookahead_never_extends_short_path_and_requires_calibrated_config():
    with pytest.raises(ValueError, match="FEASIBLE_LOOKAHEAD_MISSING"):
        time_trial_control(plan(.25), pose(), speed_mps=0., rear_axle_offset_m=(.001, 0.),
                           speed_policy="fixed_5kmh", lookahead_policy="feasible_1_to_1p5m_v1")
    root = Path(__file__).parents[1]/"configs/control"
    config = json.loads((root/"time_path_feasible_turning_5kmh_20260913.json").read_text())
    assert validate_trial_config(config) == "fixed_5kmh"
    config.pop("steering_policy"); config.pop("steering_asset_sha256")
    with pytest.raises(ValueError, match="REQUIRES_CALIBRATION"):
        validate_trial_config(config)
    with pytest.raises(ValueError, match="LOOKAHEAD_POLICY"):
        validate_trial_config({**config, "lookahead_policy": "extend_past_horizon"})


def test_recorded_corner_preview_keeps_turning_and_passes_unchanged_support_guard():
    from aic_transfuser_lite.control.awsim_steering import command_steering, CALIBRATED_POLICY
    from aic_transfuser_lite.control.turning_scan_guard import check_turning_scan
    f = json.loads((Path(__file__).parent/'fixtures/time_path/turn12_preview_rejection.json').read_text())
    p = TimePlan(f['plan_id'], TimedBodyPose(**f['observation_pose']), np.array(f['raw_xy_m']))
    current = TimedBodyPose(**f['current_pose'])
    options = dict(speed_mps=f['speed_mps'], rear_axle_offset_m=(.0010000169277191162, 0.), speed_policy='fixed_5kmh')
    old = time_trial_control(p, current, **options, lookahead_policy='feasible_1_to_1p5m_v1')
    new = time_trial_control(p, current, **options, lookahead_policy='stopping_preview_v1')
    assert old['steer_rad'] == pytest.approx(f['recorded_required_tire_rad'])
    assert new['steer_rad'] < f['measured_steer_rad'] < old['steer_rad']
    distance = .4+f['speed_mps']*.5+f['speed_mps']**2/2
    assert distance <= new['selected_lookahead_distance_m'] <= distance+.5
    np.testing.assert_array_equal(new['reference_xy_rear_m'], old['reference_xy_rear_m'])
    np.testing.assert_array_equal(p.xy_m, f['raw_xy_m'])
    scan = f['scan']
    for index, result in enumerate((old, new)):
        mapped = command_steering(result['steer_rad'], f['previous_input_steer_rad'], f['dt_s'], policy=CALIBRATED_POLICY)
        def guard():
            return check_turning_scan(np.asarray(scan['ranges'], dtype=float), scan['angle_min'], scan['angle_increment'],
                scan['range_min'], scan['range_max'], speed_mps=f['speed_mps'], measured_steer_rad=f['measured_steer_rad'],
                issued_steer_rad=mapped['issued_tire_target_rad'], previous_steer_rad=mapped['previous_tire_target_rad'],
                scan_in_current_rear=f['scan_in_current_rear'], envelope_policy='curvature_support_v2')
        if index == 0:
            with pytest.raises(ValueError, match='SWEEP_OCCUPIED'):
                guard()
        else:
            assert guard()['minimum_ray_margin_m'] > 0


@pytest.mark.parametrize('speed', [0., .5, 1., 1.3, 6/3.6])
def test_stopping_preview_uses_speed_and_never_extends_raw_reference(speed):
    p = plan(1.5)
    result = time_trial_control(p, pose(), speed_mps=speed, rear_axle_offset_m=(.001, 0.),
                               speed_policy='fixed_5kmh', lookahead_policy='stopping_preview_v1')
    minimum = max(1., .4+speed*.5+speed**2/2)
    assert result['minimum_preview_distance_m'] == pytest.approx(minimum)
    assert minimum <= result['selected_lookahead_distance_m'] <= minimum+.5
    assert result['steer_rad'] == 0.
