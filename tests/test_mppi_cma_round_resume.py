"""Exercise real round orchestration across a pause and two optimizer continuations."""
import hashlib
import importlib
import json
from pathlib import Path
import sys

import pytest


def test_round_pause_resume_then_continue_without_repeating_completed_evaluations(tmp_path, monkeypatch):
    pytest.importorskip('fcntl')
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1]/'tools/mppi_cma'))
    module = importlib.import_module('tools.mppi_cma.refine')
    monkeypatch.setattr(module, 'ROOT', tmp_path)
    original = tmp_path/'refinement'; original.mkdir()
    initial = {'completed': True, 'optimizer_image_id': 'fixed-image', 'conditions': {}}
    for condition in ('normal', 'leader'):
        folder = original/condition; folder.mkdir()
        reference = folder/'selected.csv'; reference.write_text('100')
        selected = {'reference': str(reference), 'anchors_m': [0.]*16,
                    'metrics': {'episode': 'old-selected', 'objective': 100., 'flying_lap_s': 100., 'preferred_feasible': True}}
        initial['conditions'][condition] = {'selected': selected, 'best': {'unused': True},
                                            'next_generation': 3, 'validated_preferred_feasible': True}
        (folder/'optimizer.pkl').write_bytes(b'checkpoint-generation-3')
        (folder/'optimizer_status.json').write_text(json.dumps({'generation': 3, 'pending': False}))
        (folder/'optimizer_config.json').write_text('{}')
        (folder/'seed_anchors.json').write_text(json.dumps([0.]*16))
    (original/'state.json').write_text(json.dumps(initial))
    before = {str(p.relative_to(original)):hashlib.sha256(p.read_bytes()).hexdigest()
              for p in original.rglob('*') if p.is_file()}
    now = [1000.]; pause_next = [True]; launches = []; prepared = {}
    monkeypatch.setattr(module.time, 'time', lambda: now[0])

    def optimizer(command, **_kwargs):
        folder = tmp_path/command[-2].removeprefix('/work/')
        status_path = folder/'optimizer_status.json'
        status = json.loads(status_path.read_text())
        if command[-1] == 'ask':
            (folder/'proposals.json').write_text(json.dumps([[.1*i+.001*status['generation']]*16 for i in range(8)]))
        else:
            assert command[-1] == 'tell'
            evaluated = json.loads((folder/'evaluated.json').read_text())
            assert evaluated['generation'] == status['generation']
            assert len(evaluated['objectives']) == 8
            status['generation'] += 1
            status_path.write_text(json.dumps(status))
            (folder/'optimizer.pkl').write_bytes(f"checkpoint-generation-{status['generation']}".encode())
    monkeypatch.setattr(module.subprocess, 'run', optimizer)

    def generate(_base, output, _anchors):
        output.parent.mkdir(parents=True, exist_ok=True); output.write_text('95')
    monkeypatch.setattr(module, 'generate_reference', generate)

    def launch(name, references, target_mps=10.):
        output = tmp_path/'episodes'/name
        assert not output.exists(), 'A completed simulator evaluation was launched twice'
        output.mkdir(parents=True)
        prepared[output] = [(float(Path(p).read_text()), target_mps) for p in references]
        (output/'runtime_result.json').write_text('{"ok":true}')
        launches.append((name, len(references)))
        if pause_next[0]:
            now[0] = 1002.; pause_next[0] = False
        return output
    monkeypatch.setattr(module, 'run_shared_candidates', launch)
    monkeypatch.setattr(module, 'run_episode', lambda name, **kw: launch(name, [kw['reference']], kw['target_mps']))
    monkeypatch.setattr(module, 'verify', lambda *a, **k: {'passed': True})
    monkeypatch.setattr(module, 'analyze', lambda output, vehicle=1: {
        'episode': output.name, 'objective': prepared[output][vehicle-1][0],
        'flying_lap_s': prepared[output][vehicle-1][0], 'preferred_feasible': True,
        'target_mps': prepared[output][vehicle-1][1]})

    def run(study, previous, prefix, deadline, resume=False):
        argv = ['refine.py', '--study-subdir', study, '--previous-subdir', previous,
                '--episode-prefix', prefix, '--continue-optimizer', '--deadline-unix-s', str(deadline)]
        if resume:
            argv.append('--resume-budget')
        monkeypatch.setattr(sys, 'argv', argv)
        return module.main()

    assert run('continuous/round-000', 'refinement', 'test-r00', 1001.) == 3
    state_path = tmp_path/'continuous/round-000/state.json'
    paused = json.loads(state_path.read_text())
    assert paused['phase'] == 'paused' and paused['new_episodes_started'] == 4
    assert paused['conditions']['normal']['next_generation'] == 3
    assert run('continuous/round-000', 'refinement', 'test-r00', 2000., True) == 0
    first = json.loads(state_path.read_text())
    assert first['completed'] and first['new_episodes_started'] == 64
    assert all(p['next_generation'] == 6 and p['comparison']['promoted'] for p in first['conditions'].values())
    assert run('continuous/round-001', 'continuous/round-000', 'test-r01', 2000.) == 0
    second = json.loads((tmp_path/'continuous/round-001/state.json').read_text())
    assert second['completed'] and second['new_episodes_started'] == 64
    assert all(p['next_generation'] == 9 and not p['comparison']['promoted'] for p in second['conditions'].values())
    assert sum(count for _, count in launches) == 128
    assert len({name for name, _ in launches}) == len(launches)
    assert {str(p.relative_to(original)):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in original.rglob('*') if p.is_file()} == before
