"""Frozen split/budget, consumed-attempt, and prior-pair quality boundaries."""
from __future__ import annotations

import ast
import copy
import importlib.util
import json
from pathlib import Path, PureWindowsPath

import pytest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('phase_collection_test', ROOT / 'tools/collect_time_recovery_phases.py')
assert SPEC and SPEC.loader
M = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(M)


def plan() -> dict:
    return json.loads((ROOT / 'configs/time_path_p1/recovery_phases_20260914.json').read_bytes())


def test_fixed_budget_and_state_diversity() -> None:
    p = plan()
    M.validate_plan(p)
    assert len({r['run_id'] for r in p['runs']}) == 12
    assert sum(r['split'] == 'train' for r in p['runs']) == 8
    assert sum(r['split'] == 'validation' for r in p['runs']) == 4
    assert {(r['phase'], r['side']) for r in p['runs']} == {(a, b) for a in ('early', 'late') for b in ('left', 'right')}


@pytest.mark.parametrize('key,value', [('maximum_attempts', 13), ('target_speed_mps', 2.),
    ('previous_reserved_test_usage', 'train'), ('model_training', True), ('minimum_valid_anchors_per_run', 0),
    ('pulse_start_s_m', {'early': 88., 'late': 104.}), ('remote_hub', '/tmp/outside'),
    ('analysis_name', '../outside')])
def test_scope_drift_rejected(key: str, value: object) -> None:
    p = plan(); p[key] = value
    with pytest.raises(ValueError): M.validate_plan(p)


@pytest.mark.parametrize('change', ('duplicate', 'split', 'reorder'))
def test_run_membership_is_frozen(change: str) -> None:
    p = plan()
    if change == 'duplicate': p['runs'][1] = copy.deepcopy(p['runs'][0])
    elif change == 'split': p['runs'][8]['split'] = 'train'
    else: p['runs'][0], p['runs'][1] = p['runs'][1], p['runs'][0]
    with pytest.raises(ValueError, match='RUN_ORDER'): M.validate_plan(p)


def test_missing_prior_pair_gate_prevents_dispatch(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    p = plan(); p['remote_hub'] = str(tmp_path)
    monkeypatch.setattr(M.subprocess, 'Popen', lambda *a, **k: pytest.fail('must not dispatch'))
    with pytest.raises(FileNotFoundError):
        M.start(p, tmp_path / 'collection_plan.json', p['runs'][2]['run_id'])


def test_failed_prior_pair_gate_prevents_dispatch(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    p = plan(); p['remote_hub'] = str(tmp_path)
    (tmp_path / 'pair01_20260914_prepared.json').write_text('{"status":"FAIL"}')
    monkeypatch.setattr(M.subprocess, 'Popen', lambda *a, **k: pytest.fail('must not dispatch'))
    with pytest.raises(AssertionError):
        M.start(p, tmp_path / 'collection_plan.json', p['runs'][2]['run_id'])


def test_consumed_campaign_prevents_dispatch(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    p = plan(); p['remote_hub'] = str(tmp_path)
    path = tmp_path / 'collection_plan.json'; M.write(path, p)
    M.write(tmp_path / 'campaign_20260914.json', {'plan_sha256': M.sha(path), 'sealed': True})
    monkeypatch.setattr(M, 'remote_root', lambda *a: tmp_path)
    monkeypatch.setattr(M.subprocess, 'Popen', lambda *a, **k: pytest.fail('must not dispatch'))
    with pytest.raises(AssertionError): M.start(p, path, p['runs'][0]['run_id'])


@pytest.mark.parametrize('path_class', (Path, PureWindowsPath))
def test_shipping_generated_python_is_parseable_and_exactly_scoped(monkeypatch: pytest.MonkeyPatch, path_class: type) -> None:
    scripts = []
    commands = []
    monkeypatch.setattr(M, 'Path', path_class)
    def fake_remote(host: str, code: str, *, lock: bool = False) -> str:
        ast.parse(code)
        scripts.append((host, code, lock))
        if 'm.pack(' in code:
            return json.dumps({'archive_bytes': 123, 'snapshot_sha256': 'a' * 64})
        return '{}'
    monkeypatch.setattr(M, 'remote_python', fake_remote)
    monkeypatch.setattr(M.subprocess, 'run', lambda command, **k: commands.append(command))
    M.ship_pair(plan(), 1)
    assert len(scripts) == 3 and scripts[1][2]
    cleanup = scripts[-1][1]
    assert 'codex-time-recovery-earlyleft-r50' in cleanup and 'codex-time-recovery-earlyright-r51' in cleanup
    assert 'codex-time-recovery-lateleft-r52' not in cleanup
    assert 'all_files_and_directory_structure_identical' in cleanup and 'all_sqlite_quick_checks_passed' in cleanup
    assert all('\\' not in arg for command in commands for arg in command)
    assert all("Path('/home/" in code for _, code, _ in scripts)


@pytest.mark.parametrize('path_class', (Path, PureWindowsPath))
def test_dispatch_and_status_keep_remote_posix_paths(monkeypatch: pytest.MonkeyPatch, path_class: type) -> None:
    commands = []
    scripts = []
    monkeypatch.setattr(M, 'Path', path_class)
    monkeypatch.setattr(M.subprocess, 'run', lambda command, **k: commands.append(command))
    monkeypatch.setattr(M, 'ship_pair', lambda *a: None)
    def complete(host: str, code: str, *, lock: bool = False) -> str:
        ast.parse(code)
        scripts.append(code)
        return json.dumps({'complete': True, 'result': {'status': 'COMPLETE_LAP',
            'nodes': {'closed_bag': True}, 'cleanup_errors': [], 'last_control': {'stop_confirmed': True, 'fault': None}}})
    monkeypatch.setattr(M, 'remote_python', complete)
    M.collect_pair(plan(), 1)
    assert len(scripts) == 2 and all("p=Path('/home/graneple/" in code for code in scripts)
    assert all('\\' not in arg for command in commands for arg in command)


def test_finalizer_refuses_uncollected_runs(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(M, 'paths', lambda p: (tmp_path, tmp_path / 'raw'))
    with pytest.raises(FileNotFoundError): M.finalize(plan(), tmp_path / 'plan.json')
    assert not (tmp_path / 'collection_index.json').exists()
