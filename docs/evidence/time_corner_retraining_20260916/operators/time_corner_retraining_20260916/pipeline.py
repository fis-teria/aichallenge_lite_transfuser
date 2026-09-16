"""Finite training pipeline; invoked inside the native WSL worktree lock."""
from pathlib import Path
import json,subprocess,sys,time,hashlib
repo=Path('/home/thistle/e2e_autonomous/e2e_lite_transfuser')
checks=repo.parent/'runs/time_corner_retraining_checks_20260916'
head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip()
gate=json.loads((checks/'test_gate.json').read_bytes())
assert gate['source_commit']==head and gate['exit']==0
assert hashlib.sha256((checks/'pytest.log').read_bytes()).hexdigest()==gate['log_sha256']
assert not subprocess.check_output(['git','status','--porcelain'],cwd=repo).strip()
status=dict(source_commit=head,started_unix=time.time(),stages=[],status='RUNNING')
def save():
 (checks/'pipeline_status.json').write_text(json.dumps(status,indent=2))
save()
try:
 for stage,seconds in [('audit',2400),('prepare',2400),('train',10800),('compare',2400)]:
  plan='configs/time_path_p1/recovery_corner_20260916.json' if stage=='audit' else '../runs/time_corner_retraining_20260916/resolved_plan.json'
  command=[sys.executable,'-u','tools/train_time_corner_recovery.py',stage,'--plan',plan]
  entry=dict(stage=stage,command=command,started_unix=time.time(),timeout_s=seconds)
  status['stages'].append(entry);save();print('START',stage,flush=True)
  with (checks/(stage+'.log')).open('x') as log:
   result=subprocess.run(['timeout','--signal=TERM','--kill-after=30s',str(seconds)+'s',*command],cwd=repo,stdout=log,stderr=subprocess.STDOUT)
  entry.update(exit=result.returncode,ended_unix=time.time(),log_sha256=hashlib.sha256((checks/(stage+'.log')).read_bytes()).hexdigest());save()
  print('END',stage,result.returncode,flush=True)
  print('\n'.join((checks/(stage+'.log')).read_text().splitlines()[-3:]),flush=True)
  if result.returncode:raise RuntimeError('stage failed: '+stage)
 status.update(status='COMPLETE',ended_unix=time.time());save()
except Exception as exc:
 status.update(status='FAILED',error=str(exc),ended_unix=time.time());save();raise
