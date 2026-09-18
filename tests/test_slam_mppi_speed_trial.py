"""20/15/15 km/h configuration and physical limits, not AWSIM calibration."""
from copy import deepcopy
import json
from pathlib import Path

import numpy as np
import pytest

from aic_transfuser_lite.control.slam_mppi import FAST_POLICY, path_speed_limit, ReferenceMppi
from aic_transfuser_lite.control.time_trial_v1 import validate_trial_config
from aic_transfuser_lite.control.time_dev_v1 import TimeDevSpeeds
from aic_transfuser_lite.control.vehicle_motion_v1 import effective_response_length
from tools.time_dev_runner import make_command
from test_slam_mppi import command, packet, scene


def config():
    return json.loads(Path('configs/control/time_path_slam_mppi_20_15_15.json').read_text())


def fast_packet(path):
    p = packet(False)
    p['mppi'] = dict(policy=FAST_POLICY, stamp_ns=p['stamp_ns'], frame='time_slam_map',
                    mode='AVOID', target_speed_mps=path_speed_limit(path, FAST_POLICY),
                    path_world_xy_m=path.tolist(), weighted_path_rechecked=True)
    return p


def test_configuration_and_launcher_keep_real_stop_guard():
    c = config()
    validate_trial_config(c)
    args = make_command(source=Path('/src'), deployment=Path('/runtime'), run_id='codex-time-fast',
                        display=':1', speeds=TimeDevSpeeds(20., 15.), record_video=True, slam_mppi=True)
    assert 'configs/control/time_path_slam_mppi_20_15_15_recovery.json' in args
    assert '--record-video' in args
    for key, value in [('scan_occupancy_policy', 'log_only_awsim_v1'),
                       ('stopping_distance_policy', 'awsim_cap_1m_diagnostic_v1'),
                       ('mppi_max_speed_kmh', 20.), ('mppi_max_speed_kmh', float('nan')),
                       ('speed_parameters', dict(max_speed_kmh=20., corner_max_speed_kmh=20.))]:
        bad = deepcopy(c); bad[key] = value
        with pytest.raises(ValueError, match='SLAM_MPPI'):
            validate_trial_config(bad)


def test_long_straight_path_permits_15_kmh_avoidance_and_brakes_from_20():
    path = np.column_stack((np.linspace(0., 16., 161), np.zeros(161)))
    p = fast_packet(path)
    assert p['mppi']['target_speed_mps'] == pytest.approx(15./3.6)
    kwargs = dict(mppi_policy=FAST_POLICY, vehicle_model_policy='awsim_understeer_20kmh_trial_v1',
                  target_mps=20./3.6)
    r = command(p, speed_mps=15./3.6, **kwargs)
    assert r['mode'] == 'AVOID' and r['target_speed_mps'] == pytest.approx(15./3.6)
    assert abs(r['steer_rad']) < 1e-10
    r = command(p, speed_mps=20./3.6, **kwargs)
    assert r['mode'] == 'STOP' and r['reason'] == 'MPPI_DECELERATE_BEFORE_AVOIDANCE'
    assert r['acceleration_mps2'] == -1.
    # A packet from a different profile cannot acquire control authority.
    assert command(p)['reason'] == 'MPPI_REJECTED:MPPI_PLAN_IDENTITY'


def test_short_horizon_and_turn_reduce_target_using_physical_units():
    for length, curvature in [(3., 0.), (16., .22)]:
        s = np.linspace(0., length, int(length*10)+1)
        path = (np.column_stack((s, np.zeros_like(s))) if curvature == 0 else
                np.column_stack((np.sin(curvature*s)/curvature, (1-np.cos(curvature*s))/curvature)))
        speed = path_speed_limit(path, FAST_POLICY)
        assert 0 < speed < 15./3.6
        assert .4+.5*speed+speed**2/2+1. <= length+1e-6
        assert speed**2*curvature <= 2.5+1e-6
        tire = np.arctan(curvature*effective_response_length(speed, 'awsim_understeer_20kmh_trial_v1'))
        assert tire <= .3+1e-6


def test_fast_planner_rechecks_speed_and_unknown_space():
    ref, grid, origin = scene()
    solver = ReferenceMppi(policy=FAST_POLICY)
    result = solver.solve(ref, np.zeros(3), grid, origin, .2)
    assert 0 < result['target_speed_mps'] <= 15./3.6
    path = np.asarray(result['path_world_xy_m'])
    assert np.min(np.linalg.norm(path-[6., 0.], axis=1)) > 1.5
    grid[:, 65:75] = -1
    with pytest.raises(ValueError, match='NO_FEASIBLE_PATH'):
        solver.solve(ref, np.zeros(3), grid, origin, .2)


def test_fast_packet_cannot_overstate_curved_path_speed():
    s = np.linspace(0., 12., 121)
    path = np.column_stack((np.sin(.2*s)/.2, (1-np.cos(.2*s))/.2))
    p = fast_packet(path)
    p['mppi']['target_speed_mps'] = 15./3.6
    r = command(p, mppi_policy=FAST_POLICY, vehicle_model_policy='awsim_understeer_20kmh_trial_v1')
    assert r['mode'] == 'STOP' and r['reason'] == 'MPPI_REJECTED:MPPI_PATH_SPEED_LIMIT'


@pytest.mark.parametrize('path', [np.zeros((2, 2)), np.zeros((3, 3)), np.full((3, 2), np.nan)])
def test_path_speed_rejects_invalid_shape_or_nonfinite(path):
    with pytest.raises(ValueError, match='MPPI_PATH_SPEED_SHAPE_OR_FINITE'):
        path_speed_limit(path, FAST_POLICY)
