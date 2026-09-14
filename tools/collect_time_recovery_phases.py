"""Frozen twelve-run collection at two new course positions; reuse proven runtime."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tarfile
import time
from typing import Any


def read(path: Path) -> Any:
    return json.loads(path.read_bytes())


def sha(path: Path) -> str:
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(8 * 1024**2), b''):
            h.update(block)
    return h.hexdigest()


def write(path: Path, data: Any, *, replace: bool = False) -> None:
    payload = (json.dumps(data, indent=2, allow_nan=False) + '\n').encode()
    if replace:
        pending = path.with_suffix(path.suffix + '.pending')
        with pending.open('xb') as stream:
            stream.write(payload)
        pending.replace(path)
    else:
        with path.open('xb') as stream:
            stream.write(payload)


def validate_plan(plan: dict[str, Any]) -> None:
    if (plan['schema'] != 'measured_recovery_phase_collection_v1'
            or plan['maximum_attempts'] != 12 or plan['batch_size'] != 2
            or plan['target_speed_mps'] != 5 / 3.6
            or plan['pulse_start_s_m'] != {'early': 76., 'late': 104.}
            or plan['minimum_dual_nominal_targets_per_run'] != 1
            or plan['minimum_valid_anchors_per_run'] != 60
            or not plan['stop_on_failed_pair_gate'] or not plan['raw_transfer_before_next_pair']
            or plan['model_training'] or plan['model_evaluation']
            or plan['previous_reserved_test_usage'] != 'sealed'):
        raise ValueError('FROZEN_COLLECTION_CONTRACT')
    expected = []
    for pair in range(1, 7):
        phase = 'early' if pair % 2 else 'late'
        for offset, side in enumerate(('left', 'right')):
            expected.append(dict(run_id=f'codex-time-recovery-{phase}{side}-r{48 + 2*pair + offset}',
                                 phase=phase, side=side, split='train' if pair <= 4 else 'validation', pair=pair))
    if plan['runs'] != expected:
        raise ValueError('FROZEN_RUN_ORDER_AND_SPLIT')
    for key, prefix in (('source_campaign', '/home/graneple/e2e_autonomous/time_recovery_'),
                        ('remote_hub', '/home/graneple/e2e_autonomous/time_recovery_')):
        if not re.fullmatch(re.escape(prefix) + r'[a-z0-9_]+', plan[key]):
            raise ValueError('OWNED_CAMPAIGN_PATH')
    if plan['native_root'] != '/home/thistle/e2e_autonomous' or plan['analysis_name'] != 'time_recovery_phase_expansion_20260914':
        raise ValueError('NATIVE_WSL_OUTPUT_REQUIRED')
    if plan['runtime_source_commit'] != 'a1c9e5ad0a5c274826601338355b75d34b268186':
        raise ValueError('PROVEN_RUNTIME_IDENTITY')


def paths(plan: dict[str, Any]) -> tuple[Path, Path]:
    root = Path(plan['native_root'])
    return root / 'runs' / plan['analysis_name'], root / 'raw' / plan['analysis_name']


def remote_root(plan: dict[str, Any], phase: str) -> Path:
    if phase not in plan['pulse_start_s_m']:
        raise ValueError('PHASE_NOT_PLANNED')
    return Path(plan['remote_hub']).parent / f'time_recovery_phase_{phase}_20260914'


def prepare(plan: dict[str, Any], plan_path: Path) -> None:
    """WSL: change only start progress [m] in observed-teacher references."""
    from dataclasses import asdict
    import numpy as np
    from aic_transfuser_lite.data.time_steering_pulse_v1 import SteeringPulseConfig

    out, raw = paths(plan)
    out.mkdir(exist_ok=False)
    raw.mkdir(exist_ok=False)
    refs = out / 'references'
    refs.mkdir()
    root = Path(plan['native_root'])
    manifest: dict[str, Any] = {'plan_sha256': sha(plan_path), 'files': {}, 'changed_config_field': 'start_s_m', 'source_references': {}}
    for phase, start in plan['pulse_start_s_m'].items():
        (refs / phase).mkdir()
        for side, number in (('left', 38), ('right', 39)):
            original = root / f'raw/time_recovery_expansion_20260914/codex-time-recovery-pulse{side}-r{number}/reference.json'
            ref = read(original)
            old = dict(ref['steering_pulse']['config'])
            config = SteeringPulseConfig(**{**old, 'start_s_m': start})
            guide = np.asarray(ref['steering_pulse']['nominal_guide'], dtype=float)
            assert guide.ndim == 2 and guide.shape[1] == 3 and np.isfinite(guide).all()
            assert np.all(np.diff(guide[:, 0]) > 0)
            assert guide[0, 0] <= start - 5 and guide[-1, 0] >= start + config.start_window_m + 3 + 1.7 * config.recovery_s
            assert old == {**asdict(config), 'start_s_m': old['start_s_m']}
            assert ref['intervals'] == [] and ref['signed_offset_m'] == 0 and ref['reference_xy_m'] == ref['baseline_xy_m']
            ref['steering_pulse']['config'] = asdict(config)
            target = refs / phase / f'{side}.json'
            write(target, ref)
            manifest['files'][f'{phase}/{side}.json'] = sha(target)
            manifest['source_references'][str(original)] = sha(original)
    write(out / 'reference_preparation.json', manifest)
    shutil.copy2(plan_path, out / 'collection_plan.json')
    with tarfile.open(out / 'references.tar.gz', 'x:gz') as archive:
        for rel in manifest['files']:
            archive.add(refs / rel, arcname=rel, recursive=False)
    print(json.dumps({'status': 'REFERENCES_PREPARED', 'plan_sha256': sha(plan_path), 'archive_sha256': sha(out / 'references.tar.gz')}), flush=True)


def setup(plan: dict[str, Any], plan_path: Path) -> None:
    """.10: copy verified frozen runtime into two new owned roots."""
    hub = Path(plan['remote_hub'])
    assert hub.resolve() == hub and plan_path.resolve() == hub / 'collection_plan.json'
    source = Path(plan['source_campaign'])
    old = read(source / 'campaign_20260914.json')
    assert old['sealed'] and all(r['state'] == 'WSL_MOVED' for r in old['attempts'])
    assert not subprocess.check_output(['docker', 'ps', '-q'], text=True).strip()
    assert shutil.disk_usage(hub).free >= 12 * 2**30
    deployment = read(source / 'deployment.json')
    assert deployment['source_commit'] == plan['runtime_source_commit']
    for rel, digest in deployment['files'].items():
        assert (source / rel).resolve().is_relative_to(source) and sha(source / rel) == digest, rel
    prep = read(hub / 'reference_preparation.json')
    assert prep['plan_sha256'] == sha(plan_path)
    with tarfile.open(hub / 'references.tar.gz') as archive:
        members = archive.getmembers()
        assert len(members) == 4 and {m.name for m in members} == set(prep['files'])
        values = {}
        for member in members:
            assert member.isfile()
            stream = archive.extractfile(member)
            assert stream is not None
            value = stream.read()
            assert hashlib.sha256(value).hexdigest() == prep['files'][member.name]
            values[member.name] = value
    for phase in plan['pulse_start_s_m']:
        dest = remote_root(plan, phase)
        dest.mkdir(exist_ok=False)
        for rel in deployment['files']:
            target = dest / rel
            target.parent.mkdir(exist_ok=True, parents=True)
            shutil.copy2(source / rel, target)
        for name in ('deployment.json', 'deployed_commit.txt', 'test_gate.json', 'speed_gate.json'):
            shutil.copy2(source / name, dest / name)
        shutil.copy2(hub / 'move_pair.py', dest / 'move_pair.py')
        (dest / 'references').mkdir()
        for side in ('left', 'right'):
            (dest / 'references' / f'{side}.json').write_bytes(values[f'{phase}/{side}.json'])
            shutil.copy2(source / 'references' / f'{side}.csv', dest / 'references' / f'{side}.csv')
        planned = {r['run_id']: r['side'] for r in plan['runs'] if r['phase'] == phase}
        write(dest / 'campaign_20260914.json', dict(scope='FROZEN_PHASE_COLLECTION', source_commit=plan['runtime_source_commit'],
              plan_sha256=sha(plan_path), planned=planned, maximum_attempts=6, batch_size=2, attempts=[], sealed=False))
    repo = Path('/home/graneple/git/autononous_ai/aichallenge-racingkart')
    status = subprocess.run(['git', '-C', str(repo), 'status', '--porcelain'], capture_output=True, check=True)
    write(hub / 'host_before.json', dict(head=subprocess.check_output(['git', '-C', str(repo), 'rev-parse', 'HEAD'], text=True).strip(),
          git_status_sha256=hashlib.sha256(status.stdout).hexdigest(),
          containers=[json.loads(row) for row in subprocess.check_output(['docker', 'ps', '-a', '--format', '{{json .}}'], text=True).splitlines()],
          compose=json.loads(subprocess.check_output(['docker', 'compose', 'ls', '--all', '--format', 'json'], text=True)),
          free_bytes=shutil.disk_usage(hub).free))
    print('SETUP_COMPLETE', flush=True)


def start(plan: dict[str, Any], plan_path: Path, run_id: str) -> None:
    """.10: one consumed attempt; earlier pair gates must already be copied back."""
    assigned = next(r for r in plan['runs'] if r['run_id'] == run_id)
    root, hub = remote_root(plan, assigned['phase']), Path(plan['remote_hub'])
    for number in range(1, assigned['pair']):
        gate = read(hub / f'pair{number:02d}_20260914_prepared.json')
        assert gate['status'] == 'PASS' and gate['plan_sha256'] == sha(plan_path)
    ledger = read(root / 'campaign_20260914.json')
    assert ledger['plan_sha256'] == sha(plan_path) and not ledger['sealed']
    assert len(ledger['attempts']) < ledger['maximum_attempts'] == 6
    assert not (root / run_id).exists() and run_id not in [r['run_id'] for r in ledger['attempts']]
    assert not subprocess.check_output(['docker', 'ps', '-q'], text=True).strip()
    assert shutil.disk_usage(root).free >= 12 * 2**30
    resident = [r for r in ledger['attempts'] if r['state'] != 'WSL_MOVED']
    assert len(resident) < 2
    for previous in resident:
        result = read(root / previous['run_id'] / 'result.json')
        assert result['status'] == 'COMPLETE_LAP' and result['nodes']['closed_bag'] and not result['cleanup_errors']
    earlier = [r for r in plan['runs'] if r['pair'] == assigned['pair']]
    if assigned != earlier[0]:
        assert read(root / earlier[0]['run_id'] / 'result.json')['status'] == 'COMPLETE_LAP'
    deployment = read(root / 'deployment.json')
    assert deployment['source_commit'] == plan['runtime_source_commit'] == (root / 'deployed_commit.txt').read_text().strip()
    for rel, digest in deployment['files'].items():
        assert (root / rel).resolve().is_relative_to(root) and sha(root / rel) == digest, rel
    gate = read(root / 'test_gate.json')
    assert gate['commit'] == plan['runtime_source_commit'] and gate['focused_exit'] == gate['full_exit'] == 0
    assert read(root / 'speed_gate.json')['all_within_005_mps']
    ref_path = root / 'references' / f"{assigned['side']}.json"
    prep = read(hub / 'reference_preparation.json')
    assert sha(ref_path) == prep['files'][f"{assigned['phase']}/{assigned['side']}.json"]
    ref = read(ref_path)
    assert sha(ref_path.with_suffix('.csv')) == ref['reference_sha256']
    command = ['timeout', '--signal=TERM', '--kill-after=20s', '1980s', 'python3', str(root / 'source/tools/run_time_recovery_awsim.py'),
               '--campaign-root', str(root), '--run-id', run_id, '--side', assigned['side'], '--speed-policy', 'aligned_gain4_v1', '--separate-cpus']
    row = dict(**assigned, state='STARTING', started_unix_s=time.time(), command=command, reference_sha256=sha(ref_path))
    ledger['attempts'].append(row)
    write(root / 'campaign_20260914.json', ledger, replace=True)
    env = dict(os.environ, PYTHONPATH=str(root / 'source/src'), DISPLAY=':1', XAUTHORITY='/run/user/1000/gdm/Xauthority')
    with (root / (run_id + '_supervisor.log')).open('x') as stream:
        child = subprocess.Popen(command, env=env, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
    row.update(state='STARTED', pid=child.pid)
    write(root / 'campaign_20260914.json', ledger, replace=True)
    print(json.dumps(row), flush=True)


def audit_pair(plan: dict[str, Any], plan_path: Path, pair: int) -> None:
    """WSL: audit causal observations, both measured guides, then materialize."""
    from aic_transfuser_lite.data.time_dataset_v1 import TimeDatasetConfig
    from aic_transfuser_lite.data.time_recovery_training_v1 import materialize_recovery_run
    from aic_transfuser_lite.data.time_training_cache_v1 import _prepare_run

    out, raw = paths(plan)
    assigned = [r for r in plan['runs'] if r['pair'] == pair]
    prefix = f'pair{pair:02d}_20260914'
    receipt = read(out / (prefix + '_verified.json'))
    assert receipt['all_files_and_directory_structure_identical'] and receipt['all_sqlite_quick_checks_passed']
    assert [r['run_id'] for r in receipt['runs']] == [r['run_id'] for r in assigned]
    types = Path(plan['native_root']) / 'runs/time_recovery_collection_20260913/types'
    suffixes = []
    for row in assigned:
        name = row['run_id']; suffix = name.split('-')[-1]; suffixes.append(suffix)
        commands = [
            ('bag_audit', ['tools/audit_time_recovery_collection.py', '--run', str(raw / name), '--types', str(types)]),
            ('causal_probe', ['docs/evidence/time_recovery_collection_20260913/causal_replay_probe.py', '--run', str(raw / name), '--types', str(types)]),
            ('state_audit', ['docs/evidence/time_steering_pulse_20260914/analyze_pulse.py', '--run', str(raw / name), '--probe', str(out / (suffix + '_causal_probe.json'))]),
        ]
        for kind, command in commands:
            target = out / (suffix + '_' + kind + '.json')
            assert not target.exists()
            with target.with_suffix('.log').open('x') as stream:
                result = subprocess.run([sys.executable, *command, '--output', str(target)], stdout=stream, stderr=subprocess.STDOUT, timeout=600)
            print('AUDIT_STAGE', name, kind, result.returncode, flush=True)
            if result.returncode:
                raise RuntimeError(target.with_suffix('.log').read_text()[-2500:])
    summary_path = out / (prefix + '_summary.json')
    command = [sys.executable, 'docs/evidence/time_recovery_expansion_20260914/summarize_expansion.py',
               '--analysis', str(out), '--raw', str(raw), '--alternate-guide',
               str(Path(plan['native_root']) / 'runs/time_steering_pulse_20260914/alternate_nominal_guide_r31.json'),
               '--runs', *suffixes, '--output', str(summary_path)]
    with summary_path.with_suffix('.log').open('x') as stream:
        subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT, check=True, timeout=120)
    summary = read(summary_path)
    for row in summary['runs']:
        assert row['status'] == 'COMPLETE_LAP' and row['recovery']['confirmed'], 'RECOVERY_NOT_CONFIRMED'
        assert row['target_count_agreeing_with_both_nominals'] >= plan['minimum_dual_nominal_targets_per_run'], 'NO_OUTWARD_TEACHER_AT_NEW_PHASE'
        assert row['accepted_primary']['count'] >= plan['minimum_valid_anchors_per_run'], 'INSUFFICIENT_VALID_RECOVERY_WINDOW'
    prepared = []
    for row in assigned:
        name = row['run_id']; suffix = name.split('-')[-1]
        destination = out / 'materialized' / name
        result = materialize_recovery_run(raw / name, destination, split=row['split'], types=types,
                                          previous_probe=out / (suffix + '_causal_probe.json'))
        cache = out / 'prepared' / row['split'] / name
        values = _prepare_run(destination, cache, name, TimeDatasetConfig())
        assert values['input_valid'] == values['anchors'] == result['accepted']
        assert not values['input_reason_refinements']
        files = {str(p.relative_to(out)): sha(p) for directory in (cache, destination) for p in sorted(directory.iterdir()) if p.is_file()}
        prepared.append(dict(**values, split=row['split'], phase=row['phase'], files=files))
        print('INPUTS_AND_TEACHERS_PREPARED', name, values['anchors'], flush=True)
    write(out / (prefix + '_prepared.json'), dict(status='PASS', plan_sha256=sha(plan_path), runs=prepared,
          total_accepted=summary['total_accepted'], total_targets=summary['total_targets_agreeing_with_both_nominals'],
          model_trained=False, model_evaluated=False, previous_reserved_test_read=False))
    print('PAIR_AUDIT_PASS', pair, flush=True)


def remote_python(host: str, code: str, *, lock: bool = False) -> str:
    command = ('cd /home/thistle/e2e_autonomous/e2e_lite_transfuser && tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -') if lock else 'python3 -'
    result = subprocess.run(['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10', host, command],
                            input=code, text=True, encoding='utf-8', capture_output=True, timeout=1200)
    if result.returncode:
        raise RuntimeError(result.stdout + '\n' + result.stderr)
    return result.stdout


def ship_pair(plan: dict[str, Any], pair: int, *, root: Path | None = None, prefix: str | None = None) -> None:
    """Windows: existing exact inventory/SQLite verification precedes reclamation."""
    host = 'graneple@192.168.3.10'
    out, raw = paths(plan)
    assigned = [r for r in plan['runs'] if r['pair'] == pair]
    names = [r['run_id'] for r in assigned]
    root = root if root is not None else remote_root(plan, assigned[0]['phase'])
    prefix = prefix if prefix is not None else f'pair{pair:02d}_20260914'
    bootstrap = "from pathlib import Path\nimport importlib.util\ns=importlib.util.spec_from_file_location('m'," + repr((root / 'move_pair.py').as_posix()) + ")\nm=importlib.util.module_from_spec(s);s.loader.exec_module(m)\n"
    packed = remote_python(host, bootstrap + f"m.REMOTE=Path({root.as_posix()!r})\nm.pack(m.REMOTE,{prefix!r},{names!r})\n")
    receipt = json.loads(packed)
    print('PAIR_PACKED', pair, receipt['archive_bytes'], flush=True)
    files = [prefix + '.tar.gz', prefix + '_snapshot.json', prefix + '_shipping.json']
    subprocess.run(['scp', '-3', *[host + ':' + (root / f).as_posix() for f in files], 'codex-wsl:' + out.as_posix() + '/'], check=True, timeout=1200)
    checked = remote_python('codex-wsl', f"""
from pathlib import Path
import importlib.util
s=importlib.util.spec_from_file_location('m','docs/evidence/time_recovery_batches_20260914/move_pair.py')
m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
m.ANALYSIS=Path({out.as_posix()!r});m.RAW=Path({raw.as_posix()!r})
m.verify(m.ANALYSIS,{prefix!r},{names!r},{receipt['snapshot_sha256']!r})
""", lock=True)
    print('PAIR_HASH_AND_SQLITE_VERIFIED', pair, checked.strip(), flush=True)
    subprocess.run(['scp', '-3', 'codex-wsl:' + (out / (prefix + '_verified.json')).as_posix(), host + ':' + root.as_posix() + '/'], check=True, timeout=60)
    cleanup_code = rf"""
from pathlib import Path
import json,subprocess,shutil
root=Path({root.as_posix()!r});raw=Path({raw.as_posix()!r});prefix={prefix!r};names={names!r};snapshot={receipt['snapshot_sha256']!r}
assert root.resolve()==root and root.parent==Path('/home/graneple/e2e_autonomous')
assert not subprocess.check_output(['docker','ps','-q'],text=True).strip()
v=json.loads((root/(prefix+'_verified.json')).read_bytes())
assert v['snapshot_sha256']==snapshot and v['all_files_and_directory_structure_identical'] and v['all_sqlite_quick_checks_passed']
ledger=json.loads((root/'campaign_20260914.json').read_bytes())
assert sorted(r['run_id'] for r in ledger['attempts'] if r['state']!='WSL_MOVED')==sorted(names)
for name in names:
 marker=root/(prefix+'_markers')/name;marker.mkdir(parents=True,exist_ok=False)
 result=json.loads((root/name/'result.json').read_bytes())
 (marker/'MOVED_TO_WSL.json').write_bytes((json.dumps(dict(snapshot_sha256=snapshot,raw_path=str(raw/name),result=result),indent=2)+'\n').encode())
with (root/(prefix+'_cleanup.json')).open('x') as f:
 f.write(json.dumps(dict(state='PREPARED',removed_runs=[],before_free_bytes=shutil.disk_usage(root).free),indent=2)+'\n')
code="from pathlib import Path;import importlib.util;s=importlib.util.spec_from_file_location('m','/collection/move_pair.py');m=importlib.util.module_from_spec(s);s.loader.exec_module(m);m.RAW=Path("+repr(str(raw))+");m.cleanup(Path('/collection'),"+repr(prefix)+","+repr(names)+","+repr(snapshot)+")"
subprocess.run(['docker','run','--rm','--network','none','--entrypoint','python3','-v',str(root)+':/collection','codex-cartographer-v4-build:20260910','-c',code],check=True,timeout=600)
for row in ledger['attempts']:
 if row['run_id'] in names:
  row.update(state='WSL_MOVED',raw_path=str(raw/row['run_id']),result_status=json.loads((root/row['run_id']/'MOVED_TO_WSL.json').read_bytes())['result']['status'])
ledger['sealed']=len(ledger['attempts'])==ledger['maximum_attempts'] and all(r['state']=='WSL_MOVED' for r in ledger['attempts'])
(root/'campaign_20260914.json').write_bytes((json.dumps(ledger,indent=2)+'\n').encode())
print('PAIR_MOVED',names)
"""
    print(remote_python(host, cleanup_code), flush=True)
    subprocess.run(['scp', '-3', host + ':' + (root / (prefix + '_cleanup.json')).as_posix(), 'codex-wsl:' + out.as_posix() + '/'], check=True, timeout=60)


def collect_pair(plan: dict[str, Any], pair: int) -> None:
    """Windows: one fixed pair, no retries; classify each run before the next."""
    hub = Path(plan['remote_hub']); host = 'graneple@192.168.3.10'
    assigned = [r for r in plan['runs'] if r['pair'] == pair]
    for row in assigned:
        command = ['ssh', host, 'python3', (hub / 'collect_time_recovery_phases.py').as_posix(), 'start',
                   '--plan', (hub / 'collection_plan.json').as_posix(), '--run-id', row['run_id']]
        subprocess.run(command, check=True, timeout=60)
        run = remote_root(plan, row['phase']) / row['run_id']
        deadline = time.monotonic() + 2030
        while time.monotonic() < deadline:
            code = f"""
import json
from pathlib import Path
p=Path({run.as_posix()!r});r=p/'result.json';c=p/'control_heartbeat.json';g=p/'progress.json'
if r.exists() and (p/'transfer_manifest.json').exists():
 print(json.dumps(dict(complete=True,result=json.loads(r.read_bytes()))))
else:
 d=json.loads(c.read_bytes()) if c.exists() else {{}}
 a=json.loads(g.read_bytes()) if g.exists() else {{}}
 print(json.dumps(dict(complete=False,phase=d.get('phase'),s_m=(d.get('projection') or {{}}).get('s_m'),fault=d.get('fault'),bag_bytes=a.get('bag_bytes'),sim_ns=d.get('sim_ns'))))
"""
            status = json.loads(remote_python(host, code))
            if status['complete']:
                result = status['result']
                print('RUN_FINISHED', row['run_id'], result['status'], flush=True)
                if not (result['status'] == 'COMPLETE_LAP' and result['nodes']['closed_bag']
                        and not result['cleanup_errors'] and result['last_control']['stop_confirmed']
                        and result['last_control']['fault'] is None):
                    raise RuntimeError('RUN_FAILED; preserve evidence and do not retry: ' + str(run / 'result.json'))
                break
            print('RUN_PROGRESS', row['run_id'], json.dumps(status), flush=True)
            time.sleep(30)
        else:
            raise TimeoutError('OUTER_COLLECTION_DEADLINE: ' + str(run))
    ship_pair(plan, pair)
    repo = Path('/home/thistle/e2e_autonomous/e2e_lite_transfuser')
    command = f'cd {repo.as_posix()} && tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -u tools/collect_time_recovery_phases.py audit --plan configs/time_path_p1/recovery_phases_20260914.json --pair {pair}'
    subprocess.run(['ssh', 'codex-wsl', command], check=True, timeout=2400)
    out, _ = paths(plan)
    subprocess.run(['scp', '-3', 'codex-wsl:' + (out / f'pair{pair:02d}_20260914_prepared.json').as_posix(), host + ':' + hub.as_posix() + '/'], check=True, timeout=60)
    print('COLLECTION_PAIR_COMPLETE', pair, flush=True)


def finalize(plan: dict[str, Any], plan_path: Path) -> None:
    """WSL: enumerate only preassigned materialized runs and verify their hashes."""
    out, raw = paths(plan)
    rows = []
    for pair in range(1, 7):
        report = read(out / f'pair{pair:02d}_20260914_prepared.json')
        summary = read(out / f'pair{pair:02d}_20260914_summary.json')
        assert report['status'] == 'PASS' and report['plan_sha256'] == sha(plan_path)
        for item in report['runs']:
            name = item['run_id']
            assigned = next(r for r in plan['runs'] if r['run_id'] == name)
            assert item['split'] == assigned['split'] and item['phase'] == assigned['phase']
            assert item['anchors'] == item['input_valid']
            for rel, digest in item['files'].items():
                assert (out / rel).resolve().is_relative_to(out) and sha(out / rel) == digest, rel
            metrics = next(r for r in summary['runs'] if r['run_id'] == name)
            rows.append(dict(**assigned, raw_path=str(raw / name), accepted_anchors=item['anchors'],
                             target_anchor_ids=metrics['target_anchor_ids_both_nominals'],
                             recovery=metrics['recovery'], lap_seconds=metrics['lap_seconds'], prepared_files=item['files']))
    assert [r['run_id'] for r in rows] == [r['run_id'] for r in plan['runs']]
    totals = {split: dict(independent_runs=sum(r['split'] == split for r in rows),
                         anchors=sum(r['accepted_anchors'] for r in rows if r['split'] == split),
                         target_anchors=sum(len(r['target_anchor_ids']) for r in rows if r['split'] == split))
              for split in ('train', 'validation')}
    write(out / 'collection_index.json', dict(status='COMPLETE_VERIFIED', plan_sha256=sha(plan_path), totals=totals,
          production_runs=rows, model_trained=False, model_evaluated=False, prior_reserved_test_read=False,
          runtime_source_commit=plan['runtime_source_commit'],
          audit_source_commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip()))
    print(json.dumps(totals), flush=True)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('mode', choices=('prepare', 'setup', 'start', 'collect', 'ship', 'audit', 'finalize'))
    ap.add_argument('--plan', type=Path, required=True)
    ap.add_argument('--run-id')
    ap.add_argument('--pair', type=int, choices=range(1, 7))
    args = ap.parse_args()
    plan = read(args.plan); validate_plan(plan)
    if args.mode == 'prepare': prepare(plan, args.plan)
    elif args.mode == 'setup': setup(plan, args.plan)
    elif args.mode == 'start': start(plan, args.plan, args.run_id)
    elif args.mode == 'finalize': finalize(plan, args.plan)
    else:
        if args.pair is None: ap.error('--pair required')
        if args.mode == 'collect': collect_pair(plan, args.pair)
        elif args.mode == 'ship': ship_pair(plan, args.pair)
        else: audit_pair(plan, args.plan, args.pair)


if __name__ == '__main__':
    main()
