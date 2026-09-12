"""Candidate identity, mixed-reference readiness, and conservative promotion."""
from pathlib import Path
import json

import pytest

from tools.mppi_cma.refinement_selection import compare
from tools.mppi_cma.run_shared_candidates import controller_commands, REFERENCE_TARGET
from tools.mppi_cma.shared_course_state import all_vehicles_ready
from tools.mppi_cma.dashboard import snapshot


def test_four_controller_mounts_and_ros_domains_are_separate_in_the_same_network():
    references = [Path(f'/reference/candidate-{number}.csv') for number in range(4)]
    commands = controller_commands(Path('/eval/race'), references, 'sha256:frozen', 'cma-mppi-owned', 10.)
    for number, command in enumerate(commands, 1):
        assert command[command.index('--network')+1] == 'container:cma-mppi-owned'
        assert f'ROS_DOMAIN_ID={number}' in command and f'VEHICLE_ID=d{number}' in command
        mounts = [arg for arg in command if ':' + REFERENCE_TARGET + ':' in arg]
        assert mounts == [str(references[number-1]) + ':' + REFERENCE_TARGET + ':ro']
        assert 'domain_id:=' + str(number) in command
    with pytest.raises(ValueError, match='Exactly four'):
        controller_commands(Path('/eval/race'), references[:3], 'image', 'owned', 10.)


def test_readiness_rejects_swapped_candidates_even_with_all_four_vehicles_stationary():
    expected = [[float(i), 2.] for i in range(4)]
    cars = [{'ego': {'x_m': 0., 'y_m': 0., 'speed_mps': 0., 'arrival_s': 1.},
             'gnss': {'x_m': 0., 'y_m': 0., 'arrival_s': 1.},
             'reference': {'x_m': xy[0], 'y_m': xy[1]}} for xy in expected]
    assert all_vehicles_ready(cars, 1.1, expected[0], expected)
    cars[0]['reference'], cars[1]['reference'] = cars[1]['reference'], cars[0]['reference']
    with pytest.raises(ValueError, match='different reference'):
        all_vehicles_ready(cars, 1.1, expected[0], expected)
    with pytest.raises(ValueError, match='shape'):
        all_vehicles_ready(cars, 1.1, expected[0], expected[:3])


def runs(times, feasible=True):
    return [{'flying_lap_s': value, 'preferred_feasible': feasible} for value in times]


def test_promotion_requires_four_feasible_runs_dense_clearance_and_repeated_improvement():
    dense = [{'passed': True}] * 4
    old = runs([38., 38., 38., 38.])
    new = runs([37.8, 37.9, 37.7, 38.1])
    result = compare(new, old, dense, dense)
    assert result['promoted'] and result['comparison_wins'] == 3
    assert not compare(runs([37.99]*4), old, dense, dense)['promoted']
    assert not compare(runs([37.5,37.5,38.1,38.1]), old, dense, dense)['promoted']
    assert not compare(new, old, [{'passed': False}]+dense[1:], dense)['promoted']
    assert not compare(new, runs([38.]*4, feasible=False), dense, dense)['promoted']
    with pytest.raises(ValueError, match='Four candidate'):
        compare(new[:3], old, dense, dense)
    with pytest.raises(ValueError, match='finite'):
        compare(runs([float('nan')]*4), old, dense, dense)


def test_refinement_dashboard_maps_each_vehicle_to_its_own_reference(tmp_path):
    for name in ['refinement', 'snapshot', 'episodes/group']:
        (tmp_path/name).mkdir(parents=True)
    (tmp_path/'refinement/state.json').write_text(json.dumps({
        'mode':'shared_course','study_kind':'refinement','completed':False,'phase':'search',
        'active_episode':'group','active_episodes':['group'],'new_episodes_started':4,
        'maximum_new_episodes':64,'candidate_limit_per_condition':24,'conditions':{}}))
    (tmp_path/'snapshot/base_reference.csv').write_text('x_m,y_m\n1,2\n3,4\n')
    (tmp_path/'calibration.json').write_text(json.dumps({'ot_lane_polygon_map_m':[[0,0],[1,0],[1,1]]}))
    references=[]
    for i in range(4):
        directory=tmp_path/f'candidate-{i}';directory.mkdir()
        path=directory/'reference.csv';path.write_text(f'x_m,y_m\n{i},1\n{i},2\n')
        references.append(str(path))
    episode=tmp_path/'episodes/group'
    (episode/'config.json').write_text(json.dumps({'target_mps':10.,'handicap':False,'vehicle_references':references}))
    (episode/'samples.jsonl').write_text(json.dumps({'vehicles':[{'vehicle_number':i} for i in range(1,5)]})+'\n')
    state=snapshot(tmp_path,'refinement')
    assert state['candidate_limit_per_condition']==24
    for i,car in enumerate(state['simulations']):
        assert car['reference']==[[float(i),1.],[float(i),2.]]
        assert car['episode']==f'candidate-{i} · D{i+1}'
