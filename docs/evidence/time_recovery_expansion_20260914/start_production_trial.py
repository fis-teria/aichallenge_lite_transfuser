"""Start one of twelve preassigned production runs after both WSL calibration gates pass."""
from pathlib import Path
import argparse
import hashlib
import json
import os
import shutil
import subprocess
import time

ROOT = Path('/home/graneple/e2e_autonomous/time_recovery_outward_20260914')
REFERENCE_HASHES = {
    'left': '0a6fc9fe5cf5ef50d3d578ff66af4c2e15b23778120deae2e640be55ab10bd86',
    'right': '021be33cd82336eef82bb069d298be4738c7cfc98d308165e070aa0f7375e318',
}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', required=True)
    ap.add_argument('--check-only', action='store_true')
    args = ap.parse_args()
    assert ROOT.resolve() == ROOT
    assert not subprocess.check_output(['docker', 'ps', '-q'], text=True).strip()
    assert shutil.disk_usage(ROOT).free >= 12*2**30
    ledger = json.loads((ROOT/'campaign_20260914.json').read_text())
    assert args.run_id in ledger['planned'] and not (ROOT/args.run_id).exists()
    assert len(ledger['attempts']) < ledger['maximum_attempts'] == 12
    old_root = ROOT.parent/'time_recovery_pulse_a010_20260914'
    old = json.loads((old_root/'campaign_20260914.json').read_text())
    assert old['sealed'] and old['maximum_attempts'] == 2 and len(old['attempts']) == 2
    assert all(row['state'] == 'WSL_MOVED' for row in old['attempts'])
    assert not ledger['sealed'] and ledger['scope'] == 'FROZEN_PRODUCTION'
    calibration_path = ROOT/'calibration_summary.json'
    assert hashlib.sha256(calibration_path.read_bytes()).hexdigest() == ledger['calibration_summary_sha256']
    calibration = json.loads(calibration_path.read_text())
    assert calibration['all_calibration_gates_pass'] and len(calibration['runs']) == 2
    assert {row['run_id'] for row in calibration['runs']} == set(old['planned'])
    plan_path = ROOT/'production_plan.json'
    assert hashlib.sha256(plan_path.read_bytes()).hexdigest() == ledger['production_plan_sha256']
    plan = json.loads(plan_path.read_text())
    assert {row['run_id']: row['side'] for row in plan['runs']} == ledger['planned']
    assert len(plan['runs']) == 12 and len(ledger['planned']) == 12
    assert [sum(row['split'] == split for row in plan['runs']) for split in ('train', 'validation', 'evaluation_reserved')] == [8, 2, 2]
    assert not set(ledger['planned']).intersection(old['planned'])
    assigned = next(row for row in plan['runs'] if row['run_id'] == args.run_id)
    assert assigned['reference_sha256'] == REFERENCE_HASHES[assigned['side']]
    assert args.run_id not in [row['run_id'] for row in ledger['attempts']]
    resident = [row for row in ledger['attempts'] if row['state'] != 'WSL_MOVED']
    assert len(resident) < ledger['batch_size'] == 2
    for row in ledger['attempts']:
        if row['state'] == 'WSL_MOVED':
            assert row['result_status'] == 'COMPLETE_LAP'
        else:
            result = json.loads((ROOT/row['run_id']/'result.json').read_text())
            assert result['status'] == 'COMPLETE_LAP' and result['nodes']['closed_bag'] and not result['cleanup_errors']
    speed_root = ROOT.parent/'time_recovery_speed_20260914'
    for name in ('codex-time-recovery-speedbase-r30', 'codex-time-recovery-speedbase-r31'):
        result = json.loads((speed_root/name/'MOVED_TO_WSL.json').read_text())['result']
        assert result['status'] == 'COMPLETE_LAP' and result['nodes']['closed_bag'] and not result['cleanup_errors']
    gate = json.loads((ROOT/'speed_gate.json').read_text())
    assert gate['all_within_005_mps'] and gate['threshold_mps'] == .05
    test_gate = json.loads((ROOT/'test_gate.json').read_text())
    assert test_gate['commit'] == ledger['source_commit']
    assert test_gate['focused_exit'] == test_gate['full_exit'] == 0
    manifest = json.loads((ROOT/'deployment.json').read_text())
    assert (ROOT/'deployed_commit.txt').read_text().strip() == manifest['source_commit'] == ledger['source_commit']
    for relative, digest in manifest['files'].items():
        path = ROOT/relative
        assert path.resolve().is_relative_to(ROOT)
        assert hashlib.sha256(path.read_bytes()).hexdigest() == digest, relative
    side = ledger['planned'][args.run_id]
    reference_path = ROOT/'references'/f'{side}.json'
    assert hashlib.sha256(reference_path.read_bytes()).hexdigest() == REFERENCE_HASHES[side]
    ref = json.loads(reference_path.read_text())
    assert ref['intervals'] == [] and ref['signed_offset_m'] == 0.
    assert ref['reference_xy_m'] == ref['baseline_xy_m']
    assert hashlib.sha256((ROOT/'references'/f'{side}.csv').read_bytes()).hexdigest() == ref['reference_sha256']
    pulse = ref['steering_pulse']
    assert pulse['schema'] == 'measured_steering_pulse_v1'
    assert pulse['config']['amplitude_rad'] == (.10 if side == 'left' else -.10)
    assert pulse['config']['duration_s'] == 2. and pulse['config']['start_s_m'] == 88.
    assert pulse['config']['plateau_s'] == 1.5
    env = dict(os.environ, PYTHONPATH=str(ROOT/'source/src'), DISPLAY=':1', XAUTHORITY='/run/user/1000/gdm/Xauthority')
    command = ['python3', str(ROOT/'source/tools/run_time_recovery_awsim.py'), '--campaign-root', str(ROOT),
               '--run-id', args.run_id, '--side', side, '--speed-policy', 'aligned_gain4_v1', '--separate-cpus']
    if args.check_only:
        print(json.dumps(dict(status='PREFLIGHT_PASS', assigned=assigned)))
        return
    row = dict(run_id=args.run_id, side=side, split=assigned['split'], state='STARTING', started_unix_s=time.time(), command=command,
               reference_sha256=REFERENCE_HASHES[side], pulse_config=pulse['config'])
    ledger['attempts'].append(row)
    (ROOT/'campaign_20260914.json').write_text(json.dumps(ledger, indent=2)+'\n')
    with (ROOT/(args.run_id+'_supervisor.log')).open('x') as stream:
        child = subprocess.Popen(command, env=env, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
    row.update(pid=child.pid, state='STARTED')
    (ROOT/'campaign_20260914.json').write_text(json.dumps(ledger, indent=2)+'\n')
    print(json.dumps(row))


if __name__ == '__main__':
    main()
