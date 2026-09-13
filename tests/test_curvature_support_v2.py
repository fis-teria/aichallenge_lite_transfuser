import json
import math
from pathlib import Path

import numpy as np
import pytest

from aic_transfuser_lite.control.curvature_support_v2 import curvature_support_envelope
from aic_transfuser_lite.control.turning_scan_guard import check_turning_scan


@pytest.mark.parametrize('steers', [(-.5, .5), (-.3, -.05), (.05, .3), (-.5, -.5), (.5, .5), (0., 0.)])
def test_independent_variable_curvature_integration_stays_inside_support_planes(steers):
    k_min, k_max = np.tan(steers)/1.087
    travel = .4+(6/3.6)*.5+(6/3.6)**2/2
    normals, support, metadata = curvature_support_envelope(k_min, k_max, travel, .004, np.array([1.649, 0.]))
    assert normals.shape == (64, 2) and support.shape == (64,)
    np.testing.assert_allclose(np.linalg.norm(normals, axis=1), 1., atol=1e-14)
    assert metadata['integration_step_m'] <= .005
    rng = np.random.default_rng(47)
    corners = np.array([[-.510, -.85], [1.984, -.85], [1.984, .85], [-.510, .85]])
    for mode in ('lower', 'upper', 'switch', 'random'):
        position = np.zeros(2); yaw = 0.
        steps = 3000; ds = travel/steps
        for i in range(steps+1):
            c, s = math.cos(yaw), math.sin(yaw)
            world = corners@np.array([[c, s], [-s, c]])+position
            assert np.max(world@normals.T-support) <= 1e-8
            k = k_min if mode == 'lower' else k_max if mode == 'upper' else (
                k_min if (i//53)%2 else k_max) if mode == 'switch' else rng.uniform(k_min, k_max)
            position += ds*np.array([math.cos(yaw+k*ds/2), math.sin(yaw+k*ds/2)])
            yaw += k*ds


def test_recorded_v1_overapproximation_is_cleared_without_fixing_actual_steering():
    fixture = json.loads((Path(__file__).parent/'fixtures/time_path/turn11_scan_guard_rejection.json').read_text())
    scan = fixture.pop('scan')
    options = {k: fixture[k] for k in ('speed_mps', 'measured_steer_rad', 'issued_steer_rad',
                                       'previous_steer_rad', 'scan_in_current_rear')}
    args = (np.asarray(scan['ranges'], dtype=float), scan['angle_min'], scan['angle_increment'], scan['range_min'], scan['range_max'])
    with pytest.raises(ValueError, match='SWEEP_OCCUPIED'):
        check_turning_scan(*args, **options)
    result = check_turning_scan(*args, **options, envelope_policy='curvature_support_v2')
    assert result['minimum_ray_margin_m'] > 0
    assert result['policy'] == 'CURVATURE_INTERVAL_SUPPORT_V2'
    assert result['full_body_free_space_verified'] is False
    assert result['issued_steer_rad'] != result['measured_steer_rad']


@pytest.mark.parametrize('bad', [(0., 0., .3, .004), (.2, -.2, 1., .004), (-1., 1., 1., .004),
                                (0., float('nan'), 1., .004), (0., 0., 1., .03)])
def test_support_domain_is_explicit(bad):
    with pytest.raises(ValueError, match='SUPPORT_ENVELOPE_CONTRACT'):
        curvature_support_envelope(*bad, np.array([1.649, 0.]))
