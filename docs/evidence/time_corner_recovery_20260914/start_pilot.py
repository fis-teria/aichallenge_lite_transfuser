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
import signal
import subprocess
import time


ROOT = Path('/home/graneple/e2e_autonomous/time_recovery_collection_20260913')
PLANNED = {'codex-time-recovery-cornerleft020-r27': 'left',
           'codex-time-recovery-cornerright020-r28': 'right',
           'codex-time-recovery-cornerleft020-r29': 'left'}
SOURCE_SHA = '0484dc369f66282e05faa61dc98c80d687a9c6c4'


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-id', choices=tuple(PLANNED), required=True)
    parser.add_argument('--reference-sha256', required=True)
    parser.add_argument('--check-only', action='store_true')
    parser.add_argument('--separate-cpus', action='store_true',
                        help='Put collection nodes on P cores 2-5; simulator and Autoware on 0-1,6-19 before Start')
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
    # One diagnosed infrastructure retry, within the predeclared four-attempt bound.
    if args.run_id == 'codex-time-recovery-cornerleft020-r29':
        failed = json.loads((ROOT/'codex-time-recovery-cornerleft020-r27/result.json').read_text())
        assert failed['last_control']['fault'] == 'COLLECTION_COMPUTATION_TIMEOUT'
        assert args.separate_cpus
        ledger.update(maximum_attempts=3, maximum_bag_bytes=6*2**30)
    assert len(ledger['attempts']) < ledger['maximum_attempts'] <= 3
    assert time.time()+1980 < ledger['started_unix']+4*3600
    for previous in ledger['attempts']:
        if previous['state'] == 'WSL_MOVED':
            continue
        result = json.loads((ROOT/previous['run_id']/'result.json').read_text())
        assert result['nodes']['closed_bag'] and not result.get('cleanup_errors')
        previous.update(status=result['status'], state='RECORDED')
    assert sum(r['state'] != 'WSL_MOVED' for r in ledger['attempts']) < 2
    backup = ROOT/('references_before_'+args.run_id)
    assert not backup.exists() and (ROOT/'references').resolve().parent == ROOT
    report = dict(run_id=args.run_id, side=side, reference_sha256=args.reference_sha256,
                  source_sha=SOURCE_SHA, check_passed=True, separate_cpus=args.separate_cpus)
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
    if args.separate_cpus:
        requested = {args.run_id+'-nodes':'2-5', args.run_id+'-simulator-1':'0-1,6-19',
                     args.run_id+'-autoware-1':'0-1,6-19'}
        applied = {}
        deadline = time.monotonic()+60.
        try:
            while set(applied) != set(requested):
                assert child.poll() is None and time.monotonic() < deadline
                assert not (ROOT/args.run_id/'drive_authorized.json').exists(), 'CPU_SETUP_AFTER_DRIVE_AUTHORITY'
                active = set(subprocess.check_output(['docker','ps','--format','{{.Names}}'],text=True).splitlines())
                for name in requested.keys()-applied.keys():
                    if name not in active:
                        continue
                    subprocess.run(['docker','update','--cpuset-cpus',requested[name],name],check=True,capture_output=True)
                    state = json.loads(subprocess.check_output(['docker','inspect',name],text=True))[0]
                    assert state['HostConfig']['CpusetCpus'] == requested[name]
                    applied[name] = dict(cpus=requested[name],container_id=state['Id'])
                time.sleep(.1)
            (ROOT/args.run_id/'cpu_assignment.json').write_text(json.dumps(applied,indent=2)+'\n')
        except BaseException:
            os.killpg(child.pid,signal.SIGTERM)
            raise
        report['cpu_assignment'] = applied
        campaign_path.write_text(json.dumps(ledger,indent=2)+'\n')
    print(json.dumps(report))


if __name__ == '__main__':
    main()
