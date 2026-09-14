"""Four finite, individually started calibration attempts; never start a loop."""
from pathlib import Path
import argparse
import hashlib
import json
import os
import shutil
import subprocess
import time

ROOT = Path('/home/graneple/e2e_autonomous/time_recovery_pulse_20260914')
REFERENCE_HASHES = {
    'left': '0d8b3a6de18522bd4ff2d66dedf85c71fd27c6776eb5f924ed061fbf0ed43666',
    'right': '400f58ee97637cbce614c00f1d0daab087047eb58762751565fa9384d953a1bc',
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
    assert len(ledger['attempts']) < ledger['maximum_attempts'] == 4
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
