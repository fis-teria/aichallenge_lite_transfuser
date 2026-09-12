"""Four-car start gating and measured-speed identity must remain independent."""
from copy import deepcopy
import json

import pytest

from tools.mppi_cma.shared_course_state import all_vehicles_ready
from tools.mppi_cma.dashboard import snapshot


def ready_cars():
    return [dict(ego={'x_m': i, 'y_m': 2., 'speed_mps': 0., 'arrival_s': 1.},
                 gnss={'x_m': i, 'y_m': 2., 'arrival_s': 1.},
                 reference={'x_m': 10., 'y_m': 20.}) for i in range(4)]


def test_all_four_must_be_fresh_stationary_and_on_requested_reference():
    cars = ready_cars()
    assert all_vehicles_ready(cars, 1.1, [10., 20.])
    for field, value in [('arrival_s', .5), ('speed_mps', .2), ('x_m', 99.)]:
        changed = deepcopy(cars); changed[3]['ego'][field] = value
        assert not all_vehicles_ready(changed, 1.1, [10., 20.])
    cars[2]['reference']['x_m'] = 11.
    with pytest.raises(ValueError, match='different reference'):
        all_vehicles_ready(cars, 1.1, [10., 20.])
    with pytest.raises(ValueError, match='Exactly four'):
        all_vehicles_ready(cars[:3], 1.1, [10., 20.])


def test_nonfinite_vehicle_pose_cannot_start_race():
    cars = ready_cars(); cars[0]['ego']['speed_mps'] = float('nan')
    with pytest.raises(ValueError, match='Non-finite'):
        all_vehicles_ready(cars, 1.1, [10., 20.])


def test_shared_dashboard_displays_four_domains_with_distinct_speeds_and_ranks(tmp_path):
    for name in ['shared_course', 'snapshot', 'episodes/shared-normal-ghost01']:
        (tmp_path / name).mkdir(parents=True)
    path = tmp_path / 'shared_course/state.json'
    path.write_text(json.dumps({'mode': 'shared_course', 'completed': False, 'phase': 'validation',
                               'new_episodes_started': 1, 'maximum_new_episodes': 2,
                               'active_episode': 'shared-normal-ghost01', 'conditions': {}}))
    (tmp_path / 'snapshot/base_reference.csv').write_text('x_m,y_m\n1,2\n3,4\n')
    (tmp_path / 'calibration.json').write_text(json.dumps({'ot_lane_polygon_map_m': [[0,0],[1,0],[1,1]]}))
    episode = tmp_path / 'episodes/shared-normal-ghost01'
    (episode / 'config.json').write_text(json.dumps({'target_mps': 10., 'handicap': False}))
    cars = [dict(vehicle_number=i, ego={'speed_mps': 5.+i}, status=[200,1,20,3,5-i]) for i in range(1,5)]
    (episode / 'samples.jsonl').write_text(json.dumps({'vehicles': cars}) + '\n')
    before = path.read_bytes()
    result = snapshot(tmp_path, 'shared_course')
    assert result['mode'] == 'shared_course'
    assert [s['live']['ego']['speed_mps'] for s in result['simulations']] == [6.,7.,8.,9.]
    assert [s['live']['status'][4] for s in result['simulations']] == [4,3,2,1]
    assert path.read_bytes() == before
