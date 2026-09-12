"""Analytic curves and periodic constraints for the offline speed estimate."""
from dataclasses import replace
import math

import numpy as np
import pytest

from tools.mppi_cma.speed_envelope import KartLimits, drive_cap, estimate_closed_profile, profile_residuals


def test_constant_radius_matches_analytic_lateral_limit_and_lap_time():
    radius = 10.0
    n = 180
    cfg = replace(KartLimits(), rolling_resistance_mps2=0., linear_drag_per_s=0.)
    k = np.full(n, 1 / radius)
    ds = np.full(n, 2 * math.pi * radius / n)
    result = estimate_closed_profile(k, ds, cfg, 6.)
    expected_speed = math.sqrt(6 * radius)
    assert np.allclose(result['speed_mps'], expected_speed, atol=1e-7)
    assert result['lap_time_s'] == pytest.approx(2 * math.pi * radius / expected_speed)
    assert max(result['constraint_residuals'].values()) < 1e-7


def test_speed_ceiling_is_35_kmh_not_10_mps():
    result = estimate_closed_profile(np.zeros(32), np.ones(32), KartLimits(), 12.)
    assert np.allclose(result['speed_mps'] * 3.6, 35.)
    assert result['lap_time_s'] == pytest.approx(32 / (35 / 3.6))


def test_closed_lap_brakes_before_curve_and_is_invariant_to_start_index():
    phase = np.linspace(-math.pi, math.pi, 100, endpoint=False)
    k = 0.02 + 0.2 * np.exp(-(phase / .4) ** 2)
    ds = np.full(len(k), .7)
    cfg = KartLimits()
    result = estimate_closed_profile(k, ds, cfg, 12.)
    speed = result['speed_mps']
    assert speed[45] < speed[35]  # braking starts before the maximum at 50
    assert speed[55] < speed[65]  # limited acceleration after the corner
    rotated = estimate_closed_profile(np.roll(k, 53), np.roll(ds, 53), cfg, 12.)
    assert np.allclose(np.roll(speed, 53), rotated['speed_mps'], atol=2e-6)
    assert max(profile_residuals(k, ds, speed, cfg, 12.).values()) < 2e-5
    assert result['lap_time_s'] == pytest.approx(rotated['lap_time_s'], abs=1e-6)


def test_stronger_brakes_cannot_make_estimated_lap_slower():
    phase = np.linspace(-math.pi, math.pi, 120, endpoint=False)
    k = .04 + .2 * np.exp(-(phase / .4) ** 2)
    ds = np.ones(len(k))
    ordinary = estimate_closed_profile(k, ds, KartLimits(), 12.)
    strong = estimate_closed_profile(k, ds, replace(KartLimits(), brake_command_cap_mps2=8.), 12.)
    assert strong['lap_time_s'] <= ordinary['lap_time_s']


def test_fixed_angle_violation_cannot_be_repaired_by_reducing_speed():
    with pytest.raises(ValueError, match='slowing down cannot fix'):
        estimate_closed_profile(np.full(20, .5), np.ones(20), KartLimits(), 6.)


def test_motor_fade_uses_rolling_resistance_floor():
    cfg = KartLimits()
    assert drive_cap(0., cfg) == pytest.approx(1.37)
    assert drive_cap(10., cfg) == pytest.approx(.37)
    assert drive_cap(12., cfg) == pytest.approx(.37)
    assert drive_cap(9., cfg) > drive_cap(9.7, cfg) > .37


@pytest.mark.parametrize('k,ds', [(np.zeros((8, 1)), np.ones(8)), (np.zeros(8), np.ones(9)),
                                 (np.zeros(8), np.zeros(8)), (np.full(8, np.nan), np.ones(8))])
def test_invalid_shape_or_units_are_rejected(k, ds):
    with pytest.raises(ValueError):
        estimate_closed_profile(k, ds, KartLimits(), 10.)


@pytest.mark.parametrize('key,value', [('tire_angle_rad', math.pi / 2), ('speed_cap_mps', float('nan')),
                                      ('tire_rate_radps', 0.), ('linear_drag_per_s', -1.)])
def test_invalid_vehicle_limits_are_rejected(key, value):
    with pytest.raises(ValueError):
        estimate_closed_profile(np.zeros(8), np.ones(8), replace(KartLimits(), **{key: value}), 10.)
