"""Start one finite, explicitly named corner collection with the existing runtime.

Operational evidence helper: run on .10; --check-only performs no mutations.
The source, controller, safety limits and old campaign remain unchanged.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time


ROOT = Path('/home/graneple/e2e_autonomous/time_recovery_collection_20260913')
PLANNED = {'codex-time-recovery-cornerleft020-r27': 'left',
           'codex-time-recovery-cornerright020-r28': 'right'}
SOURCE_SHA = '0484dc369f66282e05faa61dc98c80d687a9c6c4'


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-id', choices=tuple(PLANNED), required=True)
    parser.add_argument('--reference-sha256', required=True)
    parser.add_argument('--check-only', action='store_true')
    args = parser.parse_args()
    assert ROOT.resolve() == ROOT and not (ROOT/args.run_id).exists()
    assert not subprocess.check_output(['docker', 'ps', '-q'], text=True).strip()
    processes = subprocess.check_output(['ps', '-eo', 'args'], text=True).splitlines()
    assert not any(str(ROOT/'source/tools/run_time_recovery_awsim.py') in row for row in processes)
    assert (ROOT/'deployed_commit.txt').read_text().strip() == SOURCE_SHA
    source = json.loads((ROOT/'corner_source_verification_20260914.json').read_text())
    assert source['runtime_source_sha'] == SOURCE_SHA and not source['mismatches']
    for relative, record in source['rows'].items():
        assert sha(ROOT/'source'/relative) == record['expected'], relative
    for relative, expected in json.loads((ROOT/'input_source_sha256.json').read_text()).items():
        assert sha(ROOT/relative) == expected, relative
    assert shutil.disk_usage(ROOT).free >= 12*2**30
    chosen = ROOT/'references_corner020_20260914'
    side = PLANNED[args.run_id]
    reference = json.loads((chosen/(side+'.json')).read_text())
    assert sha(chosen/(side+'.csv')) == reference['reference_sha256'] == args.reference_sha256
    assert reference['config']['requests'][0]['base_start_range_m'] == [76., 82.]
    assert 76 <= reference['selected_segments'][0]['base_start_s_m'] <= 82
    assert abs(reference['signed_offset_m']) == .2
    campaign_path = ROOT/'corner_campaign_20260914.json'
    ledger = json.loads(campaign_path.read_text()) if campaign_path.exists() else dict(
        maximum_attempts=2, maximum_bag_bytes=4*2**30, attempts=[], started_unix=time.time())
    assert len(ledger['attempts']) < ledger['maximum_attempts'] == 2
    assert time.time()+1980 < ledger['started_unix']+4*3600
    for previous in ledger['attempts']:
        result = json.loads((ROOT/previous['run_id']/'result.json').read_text())
        assert result['nodes']['closed_bag'] and not result.get('cleanup_errors')
        previous.update(status=result['status'], state='RECORDED')
    backup = ROOT/('references_before_'+args.run_id)
    assert not backup.exists() and (ROOT/'references').resolve().parent == ROOT
    report = dict(run_id=args.run_id, side=side, reference_sha256=args.reference_sha256,
                  source_sha=SOURCE_SHA, check_passed=True)
    if args.check_only:
        print(json.dumps(report))
        return
    (ROOT/'references').rename(backup)
    shutil.copytree(chosen, ROOT/'references')
    report.update(state='PREPARED', started_unix=time.time())
    ledger['attempts'].append(report)
    campaign_path.write_text(json.dumps(ledger, indent=2)+'\n')
    env = dict(os.environ, PYTHONPATH=str(ROOT/'source/src'), DISPLAY=':1',
               XAUTHORITY='/run/user/1000/gdm/Xauthority')
    with (ROOT/(args.run_id+'_supervisor.log')).open('x') as stream:
        child = subprocess.Popen(['python3', str(ROOT/'source/tools/run_time_recovery_awsim.py'),
                                  '--run-id', args.run_id, '--side', side],
                                 env=env, stdout=stream, stderr=subprocess.STDOUT,
                                 start_new_session=True)
    report.update(state='STARTED', pid=child.pid)
    campaign_path.write_text(json.dumps(ledger, indent=2)+'\n')
    print(json.dumps(report))


if __name__ == '__main__':
    main()
