"""Regression coverage for bounded interpolation and unchanged existing targets."""
import json
from pathlib import Path

import numpy as np
import pytest

from aic_transfuser_lite.control.polyline_lookahead_v1 import select_polyline_lookahead
from aic_transfuser_lite.control.time_reference_v1 import TimePlan, TimedBodyPose
from aic_transfuser_lite.control.time_trial_v1 import time_trial_control, validate_trial_config


def select(points: np.ndarray, times: np.ndarray | None = None) -> dict:
    if times is None:
        times = np.arange(len(points))*.1
    return select_polyline_lookahead(points, times, minimum_m=1., maximum_m=1.5, response_length_m=1.087)


def test_preserve_first_admitted_vertex_before_any_earlier_interpolated_point() -> None:
    points = np.array([[0., 0.], [.9, 0.], [1.4, .05], [1.6, .05]])
    saved = points.copy()
    target = select(points)
    assert target['kind'] == 'vertex' and target['reference_indices'] == [2, 2]
    np.testing.assert_array_equal(target['xy_m'], points[2].astype(np.float32))
    np.testing.assert_array_equal(points, saved)


def test_missing_vertex_uses_original_segment_and_interpolates_remaining_time() -> None:
    points = np.array([[0., 0.], [.9, 0.], [1.6, 0.]])
    target = select(points, np.array([0., .1, .2]))
    assert target['kind'] == 'segment' and target['reference_indices'] == [1, 2]
    assert target['xy_m'] == pytest.approx([1.25, 0.])
    assert target['remaining_s'] == pytest.approx(.15)
    assert target['steering_margin_rad'] == .3


@pytest.mark.parametrize('reflection', [-1., 1.])
def test_narrow_angle_crossing_uses_strict_float32_limits(reflection: float) -> None:
    points = np.array([[0., 0.], [1.43, -.305], [1.50, -.333]])*[1., reflection]
    target = select(points)
    assert target['kind'] == 'segment'
    assert target['distance_margin_m'] > 0 and target['steering_margin_rad'] > 0
    assert 1. <= target['distance_m'] <= 1.5
    assert abs(target['required_tire_rad']) <= .3
    a, b = target['reference_indices']
    np.testing.assert_array_equal(target['xy_m'], (points[a]+target['fraction']*(points[b]-points[a])).astype(np.float32))


@pytest.mark.parametrize('points', [
    [[0., 0.], [.9, 0.]],
    [[0., 0.], [-1.2, 0.], [-2., 0.]],
    [[0., 0.], [.9, 0.], [-1., 100.], [1.6, 0.]],
])
def test_no_extrapolation_backward_target_or_shortcut_over_rejected_point(points: list) -> None:
    with pytest.raises(ValueError, match='FEASIBLE_LOOKAHEAD_MISSING'):
        select(np.array(points))


@pytest.mark.parametrize('times', [[0., 0.], [0., float('nan')], [.1, .2], [0., 3.1]])
def test_invalid_time_contract_fails_explicitly(times: list) -> None:
    with pytest.raises(ValueError, match='INVALID_LOOKAHEAD_REFERENCE'):
        select(np.array([[0., 0.], [2., 0.]]), np.array(times))


def test_shape_and_nonfinite_geometry_are_rejected() -> None:
    for points in (np.ones((2, 3)), np.array([[0., 0.], [float('inf'), 0.]])):
        with pytest.raises(ValueError, match='INVALID_LOOKAHEAD_REFERENCE'):
            select(points)
    with pytest.raises(ValueError, match='INVALID_LOOKAHEAD_BOUNDS'):
        select_polyline_lookahead(np.zeros((2, 2)), np.array([0., .1]), minimum_m=1.5,
                                 maximum_m=1., response_length_m=1.087)


def test_recorded_recovery_launch_passes_actual_pp_and_preserves_raw_path() -> None:
    fixture = json.loads((Path(__file__).parent/'fixtures/time_path/recovery_startup_segment.json').read_text())
    xy = np.array(fixture['raw_xy_m'])
    plan = TimePlan(fixture['plan_id'], TimedBodyPose(**fixture['observation_pose']), xy.copy())
    current = TimedBodyPose(**fixture['current_pose'])
    options = dict(speed_mps=fixture['speed_mps'], speed_policy='fixed_5kmh',
                   rear_axle_offset_m=(fixture['rear_axle_forward_m'], 0.), vehicle_model_policy='awsim_understeer_v1')
    with pytest.raises(ValueError, match=fixture['recorded_reason']):
        time_trial_control(plan, current, **options, lookahead_policy='stopping_preview_v1')
    result = time_trial_control(plan, current, **options, lookahead_policy='stopping_preview_segment_v1')
    selection = result['lookahead_selection']
    assert selection['kind'] == 'segment'
    assert selection['observation_horizon_s'] == pytest.approx(selection['remaining_s']+result['plan_age_sec'])
    assert result['steer_rad'] == pytest.approx(-.2996376962421034, abs=1e-12)
    assert result['target_speed_mps'] == 5/3.6 and result['acceleration_mps2'] == 1.
    assert selection['required_tire_rad'] == pytest.approx(result['steer_rad'], abs=1e-12)
    a, b = selection['reference_indices']
    reference = np.array(result['reference_xy_rear_m'])
    np.testing.assert_array_equal(result['lookahead_rear_m'],
        (reference[a]+selection['fraction']*(reference[b]-reference[a])).astype(np.float32))
    np.testing.assert_array_equal(plan.xy_m, xy)
    for speed_kmh in (1., 3., 5.):
        with pytest.raises(ValueError, match='FEASIBLE_LOOKAHEAD_MISSING'):
            time_trial_control(plan, current, **{**options, 'speed_mps': speed_kmh/3.6},
                               lookahead_policy='stopping_preview_segment_v1')


def test_recorded_original_vertex_command_is_identical_with_new_policy() -> None:
    fixture = json.loads((Path(__file__).parent/'fixtures/time_path/startup_actuator_limit.json').read_text())
    plan = TimePlan(fixture['plan_id'], TimedBodyPose(**fixture['observation_pose']), np.array(fixture['raw_xy_m']))
    options = dict(speed_mps=fixture['speed_mps'], speed_policy='fixed_5kmh', rear_axle_offset_m=(.0010000169277191162, 0.))
    current = TimedBodyPose(**fixture['current_pose'])
    old = time_trial_control(plan, current, **options, lookahead_policy='stopping_preview_v1')
    new = time_trial_control(plan, current, **options, lookahead_policy='stopping_preview_segment_v1')
    assert new['lookahead_selection']['kind'] == 'vertex'
    for key in ('lookahead_rear_m', 'steer_rad', 'acceleration_mps2', 'target_speed_mps', 'reference_xy_rear_m'):
        assert new[key] == old[key]


def test_new_config_changes_only_lookahead_and_keeps_calibration_requirement() -> None:
    root = Path(__file__).parents[1]/'configs/control'
    old = json.loads((root/'time_path_recovery_5kmh_20260914.json').read_text())
    new = json.loads((root/'time_path_segment_5kmh_20260914.json').read_text())
    assert new == {**old, 'lookahead_policy': 'stopping_preview_segment_v1'}
    assert validate_trial_config(new) == 'fixed_5kmh'
    new.pop('steering_policy'); new.pop('steering_asset_sha256')
    with pytest.raises(ValueError, match='REQUIRES_CALIBRATION|VEHICLE_MODEL'):
        validate_trial_config(new)
