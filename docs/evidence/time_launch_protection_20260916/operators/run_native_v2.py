"""Bounded sequential comparison under the native WSL worktree lock."""
from pathlib import Path
import json
import subprocess
import time

repo=Path('/home/thistle/e2e_autonomous/e2e_lite_transfuser')
root=repo.parent/'runs/time_launch_protection_v2_20260916'
expected='7b19a1bab61bb7d30bc1cb0f8fc44b9d1c506136'
assert Path.cwd()==repo
assert subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()==expected
assert not subprocess.check_output(['git','status','--porcelain']).strip()
root.mkdir(exist_ok=True)
python=str(repo/'.venv/bin/python');tool='tools/train_time_launch_protection.py'
started=time.time();status_path=root/'driver_status.json'
assert not status_path.exists()

def status(**values):
    value=dict(source_commit=expected,elapsed_wall_s=time.time()-started,**values)
    tmp=status_path.with_suffix('.tmp');tmp.write_text(json.dumps(value,indent=2)+'\n');tmp.replace(status_path)
    with (root/'driver_history.jsonl').open('a') as stream:stream.write(json.dumps(value)+'\n')
    print(json.dumps(value),flush=True)

commands=[('pytest',[python,'-m','pytest','-q']),
          ('teacher_oracle',[python,'-u',str(root/'oracle_native.py')]),
          ('prepare',[python,'-u',tool,'prepare']),
          ('existing_gate',[python,'-u',tool,'evaluate','--existing-only'])]
for arm in ['protocol_control','launch_balanced']:
    commands.append((arm,['timeout','--signal=TERM','--kill-after=30s','10800s',python,'-u',tool,'train','--arm',arm]))
commands.append(('final_evaluation',[python,'-u',tool,'evaluate']))
for phase,command in commands:
    status(status='RUNNING',phase=phase,command=command)
    with (root/(phase+'.log')).open('xb') as stream:
        result=subprocess.run(command,stdout=stream,stderr=subprocess.STDOUT)
    status(status='PHASE_COMPLETE' if result.returncode==0 else 'FAILED',phase=phase,exit_code=result.returncode)
    if result.returncode:raise SystemExit(result.returncode)
    if phase=='existing_gate':
        selection=json.loads((root/'evaluation_existing/selection.json').read_text())
        bad=next(r for r in selection['decisions'] if r['candidate_id']=='previous_epoch2')
        assert not bad['eligible'] and 'launch:PP_REJECTED' in bad['reasons']
        status(status='KNOWN_REGRESSION_REJECTED',phase=phase,launch_rejected=len(bad['launch_rejected_case_ids']))
status(status='COMPLETE',phase='final_evaluation',exit_code=0)
