import json
from pathlib import Path

import numpy as np
import pytest

from aic_transfuser_lite.control.time_reference_v1 import TimedBodyPose
from aic_transfuser_lite.evaluation.time_recovery_comparison_v1 import pp_probe, summarize_pp, component_errors, recorded_pose_for_pp
from aic_transfuser_lite.evaluation.time_clearance_v1 import PoseIndex


def config():
    return json.loads((Path(__file__).resolve().parents[1]/'configs/control/time_path_segment_5kmh_20260914.json').read_text())


def test_straight_observed_motion_is_age_compensated_in_pp():
    observation = TimedBodyPose(1_000_000_000, 'sim', '0', 'map', 'base_link', 10., 20., 0.)
    current = TimedBodyPose(1_200_000_000, 'sim', '0', 'map', 'base_link', 10.25, 20., 0.)
    xy = np.column_stack((np.arange(1, 31)*.125, np.zeros(30)))
    probe = pp_probe(xy, observation, current, 1.25, config())
    assert probe['accepted'] and probe['age_s'] == pytest.approx(.2)
    assert probe['steer_rad'] == pytest.approx(0.)
    assert probe['observation_horizon_s'] > .2


def test_nonfinite_prediction_and_speed_ineligibility_keep_distinct_denominators():
    pose = TimedBodyPose(1, 'sim', '0', 'map', 'base_link', 0., 0., 0.)
    bad = pp_probe(np.full((30, 2), np.nan), pose, pose, 1.2, config())
    fast = pp_probe(np.zeros((30, 2)), pose, pose, 2.2, config())
    assert bad['applicable'] and not bad['accepted']
    assert not fast['applicable']
    summary = summarize_pp([bad, fast])
    assert (summary['attempted'], summary['applicable'], summary['accepted']) == (2, 1, 0)
    assert summary['accepted_fraction'] == 0.
    assert summarize_pp([fast])['accepted_fraction'] is None


def test_component_errors_use_run_weights_and_explicit_support():
    predicted = np.tile(np.array([[1., 2.], [1., 2.], [1., 2.], [3., -4.]])[:, None], (1, 30, 1))
    mask = np.ones((4, 30), bool)
    result = component_errors(predicted, np.zeros_like(predicted), mask, np.array([True, True, False, True]), ['a', 'a', 'a', 'b'])
    assert result['3s'] == {'forward_mae_m': 2., 'left_mae_m': 3., 'left_bias_m': -1.,
                            'supported_anchors': 3, 'supported_runs': 2, 'total_anchors': 4}
    mask[:] = False
    assert component_errors(predicted, np.zeros_like(predicted), mask, np.ones(4, bool), ['a']*4)['3s']['left_mae_m'] is None


def test_component_errors_reject_shape_mismatch():
    with pytest.raises(ValueError, match='N,30,2'):
        component_errors(np.zeros((2, 6, 2)), np.zeros((2, 6, 2)), np.ones((2, 6)), np.ones(2), ['a', 'b'])


def test_recorded_pp_state_preserves_frozen_observation_and_excludes_ambiguous_future():
    from dataclasses import replace
    obs = TimedBodyPose(1_000_000_000, 'sim', '0', 'map', 'base_link', 10., 20., 0.)
    future = replace(obs, stamp_ns=1_100_000_000, x_m=10.125)
    index = PoseIndex([obs, replace(obs, x_m=10.02), future, replace(future, x_m=10.15)])
    current, reason = recorded_pose_for_pp(index, obs, 0.)
    assert current is obs and reason == 'FROZEN_OBSERVATION_POSE'
    current, reason = recorded_pose_for_pp(index, obs, .1)
    assert current is None and reason == 'RECORDED_STATE_AMBIGUOUS_POSE_STAMP'
    summary = summarize_pp([{'applicable': False, 'accepted': False, 'reason': reason}])
    assert summary['attempted'] == 1 and summary['applicable'] == 0
    assert summary['accepted_fraction'] is None


def test_recorded_pp_state_does_not_hide_unsupported_age_or_other_pose_errors():
    obs = TimedBodyPose(1_000_000_000, 'sim', '0', 'map', 'base_link', 0., 0., 0.)
    index = PoseIndex([obs])
    with pytest.raises(ValueError, match='expected recorded PP age'):
        recorded_pose_for_pp(index, obs, .3)
    with pytest.raises(ValueError):
        recorded_pose_for_pp(index, obs, .2)
