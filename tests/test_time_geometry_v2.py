import math

import numpy as np
import pytest

from aic_transfuser_lite.control.time_geometry_v2 import TimeGeometryConfig, validate_time_geometry


def arc(speed_mps: float, curvature_per_m: float) -> np.ndarray:
    distance = np.arange(1, 31) * .1 * speed_mps
    if curvature_per_m == 0:
        return np.column_stack((distance, np.zeros(30)))
    angle = distance * curvature_per_m
    return np.column_stack((np.sin(angle), 1 - np.cos(angle))) / curvature_per_m


@pytest.mark.parametrize("speed", [.05, .15, .25, 1.])
@pytest.mark.parametrize("curvature", [0., -.49, .2, .49])
def test_stationary_slow_and_steering_feasible_arcs(speed, curvature):
    xy = arc(speed, curvature)
    before = xy.copy()
    result = validate_time_geometry(xy)
    assert result["motion_resolved"]
    assert result["raw_points_checked"] == 31
    np.testing.assert_array_equal(xy, before)


def test_stationary_path_has_no_invented_heading():
    assert not validate_time_geometry(np.zeros((30, 2)))["motion_resolved"]
    noise = np.column_stack((np.full(30, .015), np.tile([.01, -.01], 15)))
    assert not validate_time_geometry(noise)["motion_resolved"]


def test_smooth_s_bend_is_not_forced_to_constant_curvature():
    ds = .05
    heading = np.cumsum(.35 * np.sin(np.arange(30) / 6) * ds)
    xy = np.cumsum(np.column_stack((np.cos(heading), np.sin(heading))) * ds, axis=0)
    assert validate_time_geometry(xy)["motion_resolved"]


@pytest.mark.parametrize("kind", ["reverse", "u_turn", "right_angle", "lateral_jump", "zigzag", "tail_backtrack"])
def test_significant_invalid_shapes_are_still_rejected(kind):
    xy = arc(.5, 0)
    if kind == "reverse":
        xy[:, 0] *= -1
    elif kind == "u_turn":
        xy[15:, 0] = xy[14, 0] - np.arange(1, 16) * .05
    elif kind == "right_angle":
        xy[15:, 0] = xy[14, 0]
        xy[15:, 1] = np.arange(1, 16) * .05
    elif kind == "lateral_jump":
        xy[:, 1] = .12
    elif kind == "zigzag":
        xy[:, 1] = np.tile([.07, -.07], 15)
    else:
        xy[22:, 0] = xy[21, 0] - np.arange(1, 9) * .008
    with pytest.raises(ValueError, match="TIME_PATH_"):
        validate_time_geometry(xy)


def test_sub_centimetre_reverse_steps_cannot_hide_large_backtracking():
    xy = np.column_stack((np.arange(1, 31) * -.009, np.zeros(30)))
    with pytest.raises(ValueError, match="TIME_PATH_INITIAL_DIRECTION"):
        validate_time_geometry(xy)


@pytest.mark.parametrize("xy", [np.zeros((29, 2)), np.zeros((30, 3)), np.full((30, 2), np.nan), np.full((30, 2), np.inf)])
def test_invalid_shape_and_nonfinite_predictions(xy):
    with pytest.raises(ValueError, match="TIME_PATH_SHAPE_FINITE"):
        validate_time_geometry(xy)


@pytest.mark.parametrize("change", [{"position_budget_m": .04}, {"heading_baseline_m": .02}, {"max_step_m": 1.},
                                     {"max_steer_rad": math.nan}, {"wheelbase_m": 0.}])
def test_invalid_or_expanded_trial_geometry_configuration(change):
    with pytest.raises(ValueError, match="TIME_GEOMETRY_CONFIG"):
        TimeGeometryConfig(**change)
