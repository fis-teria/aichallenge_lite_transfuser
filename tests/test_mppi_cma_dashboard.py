"""Ensure live reads tolerate unfinished writes and preserve study files."""
import json

from tools.mppi_cma.dashboard import last_sample, snapshot


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
