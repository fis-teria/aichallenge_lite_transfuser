"""Exact segment existence tests, including cases missed by vertex selection."""
import math

import numpy as np
import pytest

from aic_transfuser_lite.evaluation.time_lookahead_audit_v1 import (
    audit_polyline, segment_intervals, unit_roots,
)


@pytest.mark.parametrize('coefficients,expected', [
    ((0., 2., -1.), [.5]), ((1., -1., .25), [.5]),
    ((1., 0., 1.), []), ((0., 0., 1.), []), ((1., -1., 0.), [0., 1.]),
])
def test_real_unit_roots(coefficients: tuple, expected: list) -> None:
    assert unit_roots(coefficients) == pytest.approx(expected)


def test_no_vertex_but_same_segment_contains_admissible_target() -> None:
    points = np.array([[0., 0.], [.9, 0.], [1.6, 0.]])
    saved = points.copy()
    result = audit_polyline(points, np.array([0., .1, .2]), minimum_m=1., maximum_m=1.5, response_length_m=1.087)
    assert result['discrete_selected'] is None
    midpoint = result['continuous_midpoint']
    assert midpoint['midpoint_distance_m'] == pytest.approx(1.25)
    assert midpoint['remaining_s_interval'] == pytest.approx([.1+.1/7, .1+.6/7])
    assert midpoint['midpoint_angle_rad'] == 0.
    np.testing.assert_array_equal(points, saved)


def test_angle_constraint_crossing_and_reflection_preserve_interval() -> None:
    points = np.array([[0., 0.], [1.43, -.305], [1.50, -.333]])
    result = audit_polyline(points, np.array([0., 2.4, 2.5]), minimum_m=1., maximum_m=1.5, response_length_m=1.087)
    assert result['discrete_selected'] is None
    p = result['continuous_midpoint']
    assert p and p['segment_index'] == 1 and p['midpoint_margin_rad'] > 0
    reflection = audit_polyline(points*[1., -1.], np.array([0., 2.4, 2.5]), minimum_m=1., maximum_m=1.5, response_length_m=1.087)
    assert reflection['continuous_midpoint']['fraction_interval'] == pytest.approx(p['fraction_interval'])
    assert reflection['continuous_midpoint']['midpoint_angle_rad'] == pytest.approx(-p['midpoint_angle_rad'])


def test_smooth_unreachable_circle_remains_rejected_without_angle_relaxation() -> None:
    angle = np.arange(31)*.03
    points = np.column_stack((3*np.sin(angle), 3*(1-np.cos(angle))))
    result = audit_polyline(points, np.arange(31)*.1, minimum_m=1., maximum_m=1.5, response_length_m=1.087)
    assert math.atan(1.087/3) > .3
    assert result['continuous_midpoint'] is None


def test_tangent_is_zero_width_and_backward_path_is_rejected() -> None:
    tangent = segment_intervals(np.array([1.5, -.1]), np.array([1.5, .1]), 1., 1.5, 1., None)
    assert tangent == [(.5, .5)]
    assert segment_intervals(np.array([-1., 0.]), np.array([-2., 0.]), 1., 1.5, 1., .3) == []


def test_invalid_time_grid_and_units_fail() -> None:
    with pytest.raises(ValueError, match='TIMED_POLYLINE'):
        audit_polyline(np.zeros((2, 2)), np.array([0., 0.]), minimum_m=1., maximum_m=1.5, response_length_m=1.)
    with pytest.raises(ValueError, match='INVALID_SEGMENT'):
        segment_intervals(np.zeros(2), np.ones(2), 1.5, 1., 1., .3)
