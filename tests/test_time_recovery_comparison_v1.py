import json
from pathlib import Path

import numpy as np
import pytest

from aic_transfuser_lite.control.time_reference_v1 import TimedBodyPose
from aic_transfuser_lite.evaluation.time_recovery_comparison_v1 import pp_probe, summarize_pp


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
