"""Offline smoothing preserves contracts and cannot certify PP executability."""
import json
from pathlib import Path

import numpy as np
import pytest

from aic_transfuser_lite.evaluation.time_smoothing_v1 import (
    METHODS, path_change_metrics, probe_recorded_pp, smooth_time_paths,
)


@pytest.mark.parametrize("method", METHODS)
def test_stationary_linear_origin_endpoint_and_no_mutation(method: str) -> None:
    times = np.arange(1, 31) * .1
    raw = np.stack((np.zeros((30, 2)), np.column_stack((times*1.2, times*.2))))
    saved = raw.copy()
    out = smooth_time_paths(raw, method)
    assert out.shape == (2, 30, 2)
    np.testing.assert_allclose(out, raw, atol=2e-12)
    np.testing.assert_array_equal(raw, saved)
    np.testing.assert_array_equal(out[:, -1], raw[:, -1])
    assert not np.shares_memory(out, raw)


def test_d2_reduces_known_alternating_noise_without_endpoint_change() -> None:
    raw = np.column_stack((np.arange(1, 31)*.1, .02*(-1.)**np.arange(30)))
    raw[-1, 1] = 0.
    filtered = smooth_time_paths(raw, "d2_lambda10")
    metrics = path_change_metrics(raw, filtered)
    assert metrics["endpoint_max_displacement_m"] == 0
    assert metrics["changed_discrete_acceleration_rms_mps2"]["mean"] < metrics["raw_discrete_acceleration_rms_mps2"]["mean"] / 3
    assert metrics["same_time_point_displacement_m"]["max"] > .01


@pytest.mark.parametrize("raw", [np.zeros((29, 2)), np.zeros((0, 30, 2)), np.full((30, 2), np.nan)])
def test_invalid_shape_or_nonfinite_rejected(raw: np.ndarray) -> None:
    with pytest.raises(ValueError, match="FINITE_30X2"):
        smooth_time_paths(raw, "mean3")


def test_unknown_method_rejected() -> None:
    with pytest.raises(ValueError, match="UNKNOWN"):
        smooth_time_paths(np.zeros((30, 2)), "invented")


def test_pp_probe_units_margin_and_curvature_limit() -> None:
    root = Path(__file__).resolve().parents[1]
    cfg = json.loads((root/'configs/control/time_path_recovery_5kmh_20260914.json').read_text())
    pose = dict(stamp_ns=1_000_000_000, clock='sim', epoch='0', world_frame='map',
                body_frame='base_link', x_m=0., y_m=0., yaw_rad=0.)
    command = dict(plan_id='synthetic', speed_mps=0.,
                   details=dict(observation_pose=pose, current_pose=pose))
    straight = np.column_stack((np.arange(1, 31)*.1, np.zeros(30)))
    accepted = probe_recorded_pp(straight, command, cfg)
    assert accepted['reason'] == 'PP_OK'
    assert accepted['best_steering_margin_rad'] == pytest.approx(.3)
    assert accepted['selected_steer_rad'] == pytest.approx(0.)
    # A smooth circular curve still needs too much steering: smooth != feasible.
    angle = np.arange(1, 31)*.1 / 3.
    circle = np.column_stack((3*np.sin(angle), 3*(1-np.cos(angle))))
    rejected = probe_recorded_pp(circle, command, cfg)
    assert rejected['reason'] == 'STEERING_FEASIBLE_LOOKAHEAD_MISSING'
    assert rejected['best_steering_margin_rad'] < 0
