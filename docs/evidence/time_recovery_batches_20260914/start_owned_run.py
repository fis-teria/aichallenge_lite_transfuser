"""Start one explicitly selected simulator recording under the campaign limits."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import time


ROOT = Path('/home/graneple/e2e_autonomous/time_recovery_collection_20260913')


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', required=True)
    ap.add_argument('--profile', choices=('straight040', 'curve020', 'curve040'), required=True)
    ap.add_argument('--side', choices=('left', 'right'), required=True)
    args = ap.parse_args()
    if not re.fullmatch(r'codex-time-recovery-[a-z0-9-]+', args.run_id):
        raise ValueError('OWNED_RUN_ID_REQUIRED')
    assert ROOT.resolve() == ROOT and not (ROOT/args.run_id).exists()
    assert not subprocess.check_output(['docker', 'ps', '-q'], text=True).strip()
    active = subprocess.check_output(['ps', '-eo', 'args'], text=True).splitlines()
    assert not any(str(ROOT/'source/tools/run_time_recovery_awsim.py') in row for row in active)
    ledger_path = ROOT/'campaign_20260914.json'
    ledger = json.loads(ledger_path.read_text())
    assert ledger['max_attempts'] == 8 and ledger['max_bag_bytes'] == 16*2**30
    assert len(ledger['attempts']) < ledger['max_attempts']
    assert [args.profile, args.side] in ledger['planned_conditions']
    assert (ROOT/'deployed_commit.txt').read_text().strip() == ledger['runtime_source_sha']
    for previous in ledger['attempts']:
        if previous['state'] == 'STARTED':
            result = json.loads((ROOT/previous['run_id']/'result.json').read_text())
            assert result.get('nodes', {}).get('closed_bag')
            previous.update(state='RECORDED', status=result['status'],
                            fault=result['last_control'].get('fault'),
                            bag_bytes=sum(p.stat().st_size for p in
                                (ROOT/previous['run_id']/'bag').iterdir() if p.is_file()))
    resident = [p for p in ledger['attempts'] if p['state'] != 'WSL_MOVED']
    assert len(resident) < ledger['batch_size'] == 2
    assert sum(p.get('bag_bytes', 2*2**30) for p in ledger['attempts'])+2*2**30 <= ledger['max_bag_bytes']
    assert shutil.disk_usage(ROOT).free >= 10*2**30+2*2**30
    now = time.time()
    if ledger['started_unix'] is None:
        ledger.update(started_unix=now, deadline_unix=now+4*3600)
    assert now+1980 <= ledger['deadline_unix']
    chosen = ROOT/'reference_sets_20260914'/('references_'+args.profile)
    ref = json.loads((chosen/(args.side+'.json')).read_text())
    assert hashlib.sha256((chosen/(args.side+'.csv')).read_bytes()).hexdigest() == ref['reference_sha256']
    old = ROOT/('references_before_'+args.run_id)
    assert not old.exists() and (ROOT/'references').resolve().parent == ROOT
    (ROOT/'references').rename(old)
    shutil.copytree(chosen, ROOT/'references')
    entry = dict(run_id=args.run_id, profile=args.profile, side=args.side,
                 reference_sha256=ref['reference_sha256'], started_unix=now, state='PREPARED')
    ledger['attempts'].append(entry)
    ledger_path.write_text(json.dumps(ledger, indent=2)+'\n')
    env = dict(os.environ, PYTHONPATH=str(ROOT/'source/src'), DISPLAY=':1',
               XAUTHORITY='/run/user/1000/gdm/Xauthority')
    with (ROOT/(args.run_id+'_supervisor.log')).open('x') as log:
        child = subprocess.Popen(['python3', str(ROOT/'source/tools/run_time_recovery_awsim.py'),
            '--run-id', args.run_id, '--side', args.side], env=env, stdout=log,
            stderr=subprocess.STDOUT, start_new_session=True)
    entry.update(state='STARTED', pid=child.pid)
    ledger_path.write_text(json.dumps(ledger, indent=2)+'\n')
    print(json.dumps(entry))


if __name__ == '__main__':
    main()
