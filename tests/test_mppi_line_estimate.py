"""Check the geometric approximation without running AWSIM or changing CSVs."""
import math

import numpy as np
import pytest

from tools.mppi_cma.estimate_35_report import approximate_line, polygon_separation
from tools.mppi_cma.geometry import convex_overlap, vehicle_polygon


def test_separating_gap_matches_existing_vehicle_ot_overlap_contract():
    polygon = np.array([[0., 0.], [10., 0.], [10., 2.], [0., 2.]])
    rng = np.random.default_rng(12)
    xy = rng.uniform([-3., -3.], [13., 5.], size=(80, 2))
    yaw = rng.uniform(-math.pi, math.pi, size=80)
    separation = polygon_separation(xy, yaw, polygon)
    expected = np.array([convex_overlap(vehicle_polygon(*point, angle), polygon, .1)
                         for point, angle in zip(xy, yaw)])
    assert np.array_equal(separation <= .1, expected)


def test_closed_circle_approximation_preserves_inputs_and_physical_units():
    angle = np.linspace(0., 2 * math.pi, 129)
    xy = np.column_stack((10 * np.cos(angle), 10 * np.sin(angle)))
    original = xy.copy()
    polygon = np.array([[30., 30.], [31., 30.], [31., 32.], [30., 32.]])
    result = approximate_line(xy, polygon, points=64, spacing_m=.3)
    assert np.array_equal(xy, original)
    assert result['audit']['optimizer_success']
    assert result['audit']['removed_near_duplicate_closing_point']
    assert result['audit']['ot_overlap_samples_margin_0p1m'] == 0
    assert result['audit']['approximate_length_m'] == pytest.approx(20 * math.pi, rel=.02)
    assert result['audit']['maximum_required_tire_angle_deg'] < 18.
    assert result['xy_m'].shape == (len(result['curvature_1pm']), 2)
    assert np.median(result['curvature_1pm']) == pytest.approx(.1, rel=.03)


def test_failed_optimizer_does_not_return_an_unverified_line(monkeypatch):
    from types import SimpleNamespace
    from tools.mppi_cma import estimate_35_report as report
    angle = np.linspace(0., 2 * math.pi, 65)
    xy = np.column_stack((10 * np.cos(angle), 10 * np.sin(angle)))
    polygon = np.array([[30., 30.], [31., 30.], [31., 32.], [30., 32.]])
    monkeypatch.setattr(report, 'minimize', lambda *args, **kwargs:
                        SimpleNamespace(success=False, message='did not converge'))
    with pytest.raises(RuntimeError, match='did not converge'):
        report.approximate_line(xy, polygon, points=64)


def test_invalid_geometry_is_reported():
    polygon = np.array([[0., 0.], [1., 0.], [1., 1.]])
    with pytest.raises(ValueError, match='xy_m'):
        approximate_line(np.zeros((8, 3)), polygon)
    with pytest.raises(ValueError, match='Duplicate'):
        approximate_line(np.zeros((8, 2)), polygon)
