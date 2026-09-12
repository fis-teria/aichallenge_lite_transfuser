"""Changed-speed scores cannot leak into a restarted CMA search."""
import copy
import hashlib
import importlib
import json
from pathlib import Path
import sys

import pytest

from tools.mppi_cma.continuation_state import campaign_view, condition_targets


def load_restart(monkeypatch):
    pytest.importorskip('fcntl')
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1]/'tools/mppi_cma'))
    return importlib.import_module('tools.mppi_cma.restart35')


def source_fixture(tmp_path):
    source = tmp_path/'old'
    source.mkdir()
    state = {'completed': True, 'optimizer_image_id': 'pinned', 'conditions': {}}
    for condition, target in [('normal', 10.), ('leader', 7.5)]:
        folder = source/condition
        folder.mkdir()
        reference = folder/'selected.csv'
        reference.write_text('x_m,y_m\n1,2\n3,4\n')
        selected = {'name': 'old-'+condition, 'reference': str(reference), 'anchors_m': [0.]*16,
                    'metrics': {'target_mps': target, 'flying_lap_s': 37. if condition == 'normal' else 50.}}
        state['conditions'][condition] = {'selected': selected, 'next_generation': 18,
                                          'validated_preferred_feasible': True, 'evaluations': []}
        (folder/'optimizer.pkl').write_bytes(b'old-generation-18')
        (folder/'optimizer_status.json').write_text(json.dumps({'generation': 18, 'pending': False}))
        (folder/'optimizer_config.json').write_text('{"sigma_m":0.16,"seed":123}')
        (folder/'seed_anchors.json').write_text(json.dumps([0.]*16))
    (source/'state.json').write_text(json.dumps(state))
    return source, state


def baseline_runs():
    return [{'episode': f'baseline-d{i}', 'flying_lap_s': value,
             'target_mps': 35/3.6, 'command_speed_max_mps': 35/3.6,
             'preferred_feasible': True, 'handicap_enabled': False}
            for i, value in enumerate([38., 38.2, 38.1, 38.3], 1)]


def test_restart_remeasures_normal_and_preserves_only_unchanged_leader_distribution(tmp_path, monkeypatch):
    module = load_restart(monkeypatch)
    source, _ = source_fixture(tmp_path)
    before = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in source.rglob('*') if p.is_file()}
    result = module.prepare_seed(source, tmp_path/'seed', baseline_runs(), [{'passed': True}]*4)
    assert condition_targets(result) == {'normal': 35/3.6, 'leader': 7.5}
    normal = result['conditions']['normal']
    assert normal['baseline_median_s'] == pytest.approx(38.15)
    assert normal['selected']['metrics']['flying_lap_s'] != 37.
    assert normal['next_generation'] == 0 and normal['optimizer_restart_required']
    assert not (tmp_path/'seed/normal/optimizer.pkl').exists()
    assert (tmp_path/'seed/leader/optimizer.pkl').read_bytes() == b'old-generation-18'
    assert result['conditions']['leader']['next_generation'] == 18
    assert {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in source.rglob('*') if p.is_file()} == before
    assert result['restart_baseline_evaluations'] == 4


@pytest.mark.parametrize('field,value', [('target_mps', 10.), ('command_speed_max_mps', 10.),
                                       ('handicap_enabled', True), ('preferred_feasible', False),
                                       ('flying_lap_s', float('nan'))])
def test_failed_rebaseline_creates_no_seed(tmp_path, monkeypatch, field, value):
    module = load_restart(monkeypatch)
    source, _ = source_fixture(tmp_path)
    runs = baseline_runs()
    runs[0][field] = value
    with pytest.raises(ValueError, match='baseline failed'):
        module.prepare_seed(source, tmp_path/'seed', runs, [{'passed': True}]*4)
    assert not (tmp_path/'seed').exists()


def test_dense_overlap_blocks_restart_and_short_repeat_set_is_rejected(tmp_path, monkeypatch):
    module = load_restart(monkeypatch)
    source, _ = source_fixture(tmp_path)
    with pytest.raises(ValueError, match='dense OT'):
        module.prepare_seed(source, tmp_path/'seed', baseline_runs(), [{'passed': True}]*3+[{'passed': False}])
    with pytest.raises(ValueError, match='Four normal'):
        module.prepare_seed(source, tmp_path/'seed', baseline_runs()[:3], [{'passed': True}]*4)
    assert not (tmp_path/'seed').exists()


def test_target_contract_rejects_relabeling_old_scores(tmp_path):
    _, state = source_fixture(tmp_path)
    assert condition_targets(state) == {'normal': 10., 'leader': 7.5}
    state['conditions']['normal']['target_mps'] = 35/3.6
    with pytest.raises(ValueError, match='separately measured'):
        condition_targets(state)
    state['conditions']['normal']['target_mps'] = float('inf')
    with pytest.raises(ValueError, match='finite'):
        condition_targets(state)


def test_campaign_counts_baseline_and_keeps_35kmh_measurements(tmp_path, monkeypatch):
    module = load_restart(monkeypatch)
    source, _ = source_fixture(tmp_path)
    seed = module.prepare_seed(source, tmp_path/'seed', baseline_runs(), [{'passed': True}]*4)
    journal = {'maximum_rounds': 6, 'completed': False, 'initial_evaluations': 4}
    view = campaign_view(journal, seed, [])
    assert view['new_episodes_started'] == 4 and view['maximum_new_episodes'] == 388
    assert view['conditions']['normal']['target_mps'] == 35/3.6
    assert view['conditions']['normal']['baseline_median_s'] == pytest.approx(38.15)


def test_fresh_speed_seed_manifest_does_not_require_old_normal_optimizer(tmp_path, monkeypatch):
    module = load_restart(monkeypatch)
    source, _ = source_fixture(tmp_path)
    module.prepare_seed(source, tmp_path/'seed', baseline_runs(), [{'passed': True}]*4)
    continuous = importlib.import_module('continuous')
    monkeypatch.setattr(continuous, 'ROOT', tmp_path)
    manifest = continuous.source_manifest('seed')
    assert len(manifest) == 7
    assert 'seed/normal/optimizer.pkl' not in manifest
    assert 'seed/leader/optimizer.pkl' in manifest


def test_round_passes_35kmh_to_all_four_controllers_before_budget_pause(tmp_path, monkeypatch):
    module = load_restart(monkeypatch)
    source, _ = source_fixture(tmp_path)
    module.prepare_seed(source, tmp_path/'seed', baseline_runs(), [{'passed': True}]*4)
    refine = importlib.import_module('tools.mppi_cma.refine')
    monkeypatch.setattr(refine, 'ROOT', tmp_path)
    now = [100.]
    monkeypatch.setattr(refine.time, 'time', lambda: now[0])
    observed = []

    def optimizer(command, **_kwargs):
        folder = tmp_path/command[-2].removeprefix('/work/')
        assert command[-1] in ('init', 'ask')
        (folder/'optimizer.pkl').write_bytes(b'new-35kmh-pending')
        (folder/'proposals.json').write_text(json.dumps([[i*.01]*16 for i in range(8)]))

    monkeypatch.setattr(refine.subprocess, 'run', optimizer)

    def generate(_base, output, _anchors):
        output.parent.mkdir(parents=True)
        output.write_text('x_m,y_m\n1,2\n3,4\n')

    monkeypatch.setattr(refine, 'generate_reference', generate)

    def launch(name, references, target_mps):
        observed.append(target_mps)
        assert len(set(references)) == 4
        output = tmp_path/'episodes'/name
        output.mkdir(parents=True)
        (output/'runtime_result.json').write_text('{}')
        now[0] = 102.
        return output

    monkeypatch.setattr(refine, 'run_shared_candidates', launch)
    monkeypatch.setattr(refine, 'analyze', lambda output, vehicle: {
        'episode': output.name, 'target_mps': 35/3.6, 'objective': 38.,
        'flying_lap_s': 38., 'preferred_feasible': True})
    monkeypatch.setattr(sys, 'argv', ['refine.py', '--study-subdir', 'new/round-000',
        '--previous-subdir', 'seed', '--episode-prefix', 'c35-test', '--continue-optimizer',
        '--deadline-unix-s', '101'])
    assert refine.main() == 3
    assert observed == [35/3.6]
    state = json.loads((tmp_path/'new/round-000/state.json').read_text())
    assert state['new_episodes_started'] == 4 and state['phase'] == 'paused'
    assert state['conditions']['normal']['starting_generation'] == 0
    assert state['conditions']['leader']['starting_generation'] == 18
