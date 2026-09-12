"""Ensure live reads tolerate unfinished writes and preserve study files."""
import json
import pytest

from tools.mppi_cma.dashboard import last_sample, snapshot, episode_snapshot


def test_latest_complete_jsonl_sample_ignores_partial_write(tmp_path):
    path = tmp_path / 'samples.jsonl'
    assert last_sample(path) == {}
    path.write_bytes(b'{"ego":{"speed_mps":7.5}}\n{"ego":')
    assert last_sample(path)['ego']['speed_mps'] == 7.5
    assert path.read_bytes().endswith(b'{"ego":')


def test_finished_dashboard_has_no_live_vehicle_and_does_not_write(tmp_path):
    (tmp_path / 'search').mkdir()
    (tmp_path / 'snapshot').mkdir()
    state = tmp_path / 'search/state.json'
    state.write_text(json.dumps({'completed': True, 'phase': 'complete', 'new_episodes_started': 43,
                                'maximum_new_episodes': 43, 'conditions': {}}))
    (tmp_path / 'snapshot/base_reference.csv').write_text('x_m,y_m\n1,2\n3,4\n')
    (tmp_path / 'calibration.json').write_text(json.dumps({'ot_lane_polygon_map_m': [[0, 0], [1, 0], [1, 1]]}))
    before = state.read_bytes()
    payload = snapshot(tmp_path)
    assert payload['completed'] and payload['active'] is None
    assert payload['live']['ego'] is None
    assert payload['baseline'] == [[1., 2.], [3., 4.]]
    assert state.read_bytes() == before


def test_isolated_episode_read_keeps_each_vehicle_identity_and_units(tmp_path):
    for name,speed in [('first',7.5),('second',10.)]:
        path=tmp_path/'episodes'/name;path.mkdir(parents=True)
        (path/'samples.jsonl').write_text(json.dumps({'ego':{'speed_mps':speed},'status':[200,1,20,3,1]})+'\n')
        (path/'config.json').write_text(json.dumps({'target_mps':speed,'handicap':name=='first'}))
    first=episode_snapshot(tmp_path,'first');second=episode_snapshot(tmp_path,'second')
    assert first['episode']=='first' and second['episode']=='second'
    assert first['live']['ego']['speed_mps']==7.5
    assert second['live']['ego']['speed_mps']==10.
    assert first['handicap'] and not second['handicap']


def test_finished_refinement_keeps_selected_routes_visible_without_live_episodes(tmp_path):
    (tmp_path / 'refinement').mkdir()
    (tmp_path / 'snapshot').mkdir()
    base = tmp_path / 'snapshot/base_reference.csv'
    base.write_text('x_m,y_m\n1,2\n3,4\n')
    selected = tmp_path / 'selected.csv'
    selected.write_text('x_m,y_m\n1.5,2.5\n3.5,4.5\n')
    (tmp_path / 'calibration.json').write_text(json.dumps({'ot_lane_polygon_map_m': [[0, 0], [1, 0], [1, 1]]}))
    state = tmp_path / 'refinement/state.json'
    state.write_text(json.dumps({'completed': True, 'phase': 'complete', 'new_episodes_started': 64,
                                'maximum_new_episodes': 64, 'conditions': {
        'normal': {'evaluations': [], 'selected': {'reference': str(base), 'metrics': {}}},
        'leader': {'evaluations': [], 'selected': {'reference': str(selected), 'metrics': {}}},
    }}))
    before = {p: p.read_bytes() for p in tmp_path.rglob('*') if p.is_file()}
    payload = snapshot(tmp_path, 'refinement')
    assert payload['simulations'] == []
    assert payload['conditions']['normal']['selected_reference'] == [[1., 2.], [3., 4.]]
    assert payload['conditions']['leader']['selected_reference'] == [[1.5, 2.5], [3.5, 4.5]]
    assert {p: p.read_bytes() for p in tmp_path.rglob('*') if p.is_file()} == before


def test_named_campaign_reports_actual_target_and_rejects_path_traversal(tmp_path):
    (tmp_path/'continuous35_20260913').mkdir()
    (tmp_path/'snapshot').mkdir()
    (tmp_path/'snapshot/base_reference.csv').write_text('x_m,y_m\n1,2\n3,4\n')
    (tmp_path/'calibration.json').write_text('{"ot_lane_polygon_map_m":[[0,0],[1,0],[1,1]]}')
    (tmp_path/'continuous35_20260913/state.json').write_text(json.dumps({
        'completed': False, 'phase': 'rebaseline', 'new_episodes_started': 4, 'maximum_new_episodes': 388,
        'conditions': {'normal': {'target_mps': 35/3.6, 'evaluations': []}}}))
    result = snapshot(tmp_path, 'continuous35_20260913')
    assert result['conditions']['normal']['target_mps'] * 3.6 == pytest.approx(35.)
    assert result['phase'] == 'rebaseline' and result['conditions']['normal']['best'] is None
    with pytest.raises(ValueError):
        snapshot(tmp_path, '../outside')
