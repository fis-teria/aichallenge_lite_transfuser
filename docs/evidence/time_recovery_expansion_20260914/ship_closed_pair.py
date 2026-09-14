"""Windows orchestration: transfer a packed pair, verify in WSL, reclaim exact originals."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess

HOST = 'graneple@192.168.3.10'
REMOTE = '/home/graneple/e2e_autonomous/time_recovery_outward_20260914'
ANALYSIS = '/home/thistle/e2e_autonomous/runs/time_recovery_expansion_20260914'
RAW = '/home/thistle/e2e_autonomous/raw/time_recovery_expansion_20260914'
PLAN_SHA = '04d757dde75e6b42062a6966d5f9d05bdad1649c7e4d51980f47ded5f68bb462'


def run_python(host: str, code: str, *, wsl_lock: bool = False) -> str:
    command = ('cd /home/thistle/e2e_autonomous/e2e_lite_transfuser && '
        'tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -') if wsl_lock else 'python3 -'
    result = subprocess.run(['ssh', host, command], input=code, text=True, encoding='utf-8',
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if result.returncode:
        raise RuntimeError(result.stdout+'\n'+result.stderr)
    print(result.stdout, flush=True)
    return result.stdout


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--pair', type=int, choices=range(2, 8), required=True)
    args = ap.parse_args()
    evidence = Path(__file__).resolve().parent
    plan_path = evidence/'production_plan.json'
    assert hashlib.sha256(plan_path.read_bytes()).hexdigest() == PLAN_SHA
    plan = json.loads(plan_path.read_text())
    names = [r['run_id'] for r in plan['runs'][2*(args.pair-2):2*(args.pair-1)]]
    assert len(names) == 2 and len(set(names)) == 2
    prefix = f'pair{args.pair:02d}_20260914'
    assert not (evidence/(prefix+'_verified.json')).exists()
    text = run_python(HOST, f"""
from pathlib import Path
import json
root=Path({REMOTE!r});p=root/({prefix!r}+'_shipping.json')
d=json.loads(p.read_text());assert d['run_ids']=={names!r}
assert (root/({prefix!r}+'.tar.gz')).stat().st_size==d['archive_bytes']
print(json.dumps(d))
""")
    receipt = json.loads(text)
    print('TRANSFER_STARTED', names, flush=True)
    files = [prefix+'.tar.gz', prefix+'_snapshot.json', prefix+'_shipping.json']
    subprocess.run(['scp', '-3', *[HOST+':'+REMOTE+'/'+f for f in files], 'codex-wsl:'+ANALYSIS+'/'], check=True)
    run_python('codex-wsl', f"""
from pathlib import Path
import importlib.util
s=importlib.util.spec_from_file_location('m','docs/evidence/time_recovery_batches_20260914/move_pair.py')
m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
m.ANALYSIS=Path({ANALYSIS!r});m.RAW=Path({RAW!r})
m.verify(m.ANALYSIS,{prefix!r},{names!r},{receipt['snapshot_sha256']!r})
""", wsl_lock=True)
    subprocess.run(['scp', '-3', 'codex-wsl:'+ANALYSIS+'/'+prefix+'_verified.json', HOST+':'+REMOTE+'/'], check=True)
    run_python(HOST, f"""
from pathlib import Path
import json,subprocess,shutil
root=Path({REMOTE!r});raw=Path({RAW!r});prefix={prefix!r};names={names!r};snapshot={receipt['snapshot_sha256']!r}
assert not subprocess.check_output(['docker','ps','-q'],text=True).strip()
v=json.loads((root/(prefix+'_verified.json')).read_text())
assert v['snapshot_sha256']==snapshot and v['all_files_and_directory_structure_identical'] and v['all_sqlite_quick_checks_passed']
ledger=json.loads((root/'campaign_20260914.json').read_text())
assert sorted(r['run_id'] for r in ledger['attempts'] if r['state']!='WSL_MOVED')==sorted(names)
for name in names:
 marker=root/(prefix+'_markers')/name;marker.mkdir(parents=True,exist_ok=False)
 result=json.loads((root/name/'result.json').read_text())
 (marker/'MOVED_TO_WSL.json').write_text(json.dumps(dict(snapshot_sha256=snapshot,raw_path=str(raw/name),result=result),indent=2)+'\\n')
with (root/(prefix+'_cleanup.json')).open('x') as f:f.write(json.dumps(dict(state='PREPARED',removed_runs=[],before_free_bytes=shutil.disk_usage(root).free),indent=2)+'\\n')
code="from pathlib import Path;import importlib.util;s=importlib.util.spec_from_file_location('m','/collection/move_pair.py');m=importlib.util.module_from_spec(s);s.loader.exec_module(m);m.RAW=Path("+repr(str(raw))+");m.cleanup(Path('/collection'),"+repr(prefix)+","+repr(names)+","+repr(snapshot)+")"
subprocess.run(['docker','run','--rm','--network','none','--entrypoint','python3','-v',str(root)+':/collection','codex-cartographer-v4-build:20260910','-c',code],check=True)
for row in ledger['attempts']:
 if row['run_id'] in names:row.update(state='WSL_MOVED',raw_path=str(raw/row['run_id']),result_status=json.loads((root/row['run_id']/'MOVED_TO_WSL.json').read_text())['result']['status'])
ledger['sealed']=len(ledger['attempts'])==ledger['maximum_attempts'] and all(r['state']=='WSL_MOVED' for r in ledger['attempts'])
(root/'campaign_20260914.json').write_text(json.dumps(ledger,indent=2)+'\\n')
print('PAIR_SHIPPED',names)
""")
    small = [prefix+'_verified.json', prefix+'_shipping.json', prefix+'_cleanup.json']
    subprocess.run(['scp', *[HOST+':'+REMOTE+'/'+f for f in small], str(evidence)], check=True)


if __name__ == '__main__':
    main()
