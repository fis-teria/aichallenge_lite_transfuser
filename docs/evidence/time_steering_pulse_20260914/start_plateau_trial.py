"""Four finite, individually started calibration attempts; never start a loop."""
from pathlib import Path
import argparse
import hashlib
import json
import os
import shutil
import subprocess
import time

ROOT = Path('/home/graneple/e2e_autonomous/time_recovery_pulse_plateau_20260914')
REFERENCE_HASHES = {
    'left': 'cd4ff5bf4906bacd177a8927b1b9a7f7dbe41900fc90bea8c31dad52730f4e9d',
    'right': '621c2c1059103ff0c7b430db9e3a0b5291ccac1f7a4a3a3a0e83c4e8d792f33c',
}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', required=True)
    args = ap.parse_args()
    assert ROOT.resolve() == ROOT
    assert not subprocess.check_output(['docker', 'ps', '-q'], text=True).strip()
    assert shutil.disk_usage(ROOT).free >= 12*2**30
    ledger = json.loads((ROOT/'campaign_20260914.json').read_text())
    assert args.run_id in ledger['planned'] and not (ROOT/args.run_id).exists()
    assert len(ledger['attempts']) < ledger['maximum_attempts'] == 2
    old_root = ROOT.parent/'time_recovery_pulse_20260914'
    old = json.loads((old_root/'campaign_20260914.json').read_text())
    assert old['sealed'] and old['maximum_attempts'] == 2 and len(old['attempts']) == 2
    assert all(row['state'] == 'WSL_MOVED' for row in old['attempts'])
    assert len(ledger['attempts'])+len(old['attempts']) < ledger['maximum_total_pulse_attempts'] == 4
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
    assert pulse['config']['amplitude_rad'] == (.08 if side == 'left' else -.08)
    assert pulse['config']['duration_s'] == 2. and pulse['config']['start_s_m'] == 88.
    assert pulse['config']['plateau_s'] == 1.5
    env = dict(os.environ, PYTHONPATH=str(ROOT/'source/src'), DISPLAY=':1', XAUTHORITY='/run/user/1000/gdm/Xauthority')
    command = ['python3', str(ROOT/'source/tools/run_time_recovery_awsim.py'), '--campaign-root', str(ROOT),
               '--run-id', args.run_id, '--side', side, '--speed-policy', 'aligned_gain4_v1', '--separate-cpus']
    row = dict(run_id=args.run_id, side=side, state='STARTING', started_unix_s=time.time(), command=command,
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
