"""One bounded restart with serial input loading; preserve the interrupted run."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess
import sys
import time

repo = Path('/home/thistle/e2e_autonomous/e2e_lite_transfuser')
checks = repo.parent/'runs/time_corner_retraining_serial_checks_20260916'
checks.mkdir(exist_ok=False)
head = subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip()
assert head == '5bd177cd6192b00465be1d902da8999fe7668453'
assert not subprocess.check_output(['git','status','--porcelain'],cwd=repo).strip()
shutil.copy2('/mnt/e/workspace/e2e_lite_transfuser/tmp/time_corner_retraining_20260916/retry_pipeline.py',checks/'pipeline.py')
status = dict(source_commit=head,status='RUNNING',started_unix=time.time(),stages=[],
    prior_attempt='time_corner_retraining_checks_20260916',restart_from_original_initialization=True,
    shared_memory_worker_transfer=False,actual_loader_workers=0)

def save():
    (checks/'pipeline_status.json').write_text(json.dumps(status,indent=2))

save()
try:
    start = time.monotonic()
    with (checks/'pytest.log').open('x') as log:
        result = subprocess.run([sys.executable,'-m','pytest','-q'],cwd=repo,stdout=log,stderr=subprocess.STDOUT,timeout=450)
    gate = dict(source_commit=head,exit=result.returncode,seconds=time.monotonic()-start,
        log_sha256=hashlib.sha256((checks/'pytest.log').read_bytes()).hexdigest())
    (checks/'test_gate.json').write_text(json.dumps(gate,indent=2))
    print('TEST_GATE',json.dumps(gate),flush=True)
    print('\n'.join((checks/'pytest.log').read_text().splitlines()[-6:]),flush=True)
    assert result.returncode == 0
    for stage, seconds in [('train',10800),('compare',2400)]:
        command = [sys.executable,'-u','tools/train_time_corner_recovery.py',stage,
            '--plan','../runs/time_corner_retraining_20260916/resolved_plan.json',
            '--loader-workers','0','--training-output','../runs/time_corner_retraining_20260916/training_serial']
        row = dict(stage=stage,command=command,started_unix=time.time(),timeout_s=seconds)
        status['stages'].append(row);save();print('START',stage,flush=True)
        with (checks/(stage+'.log')).open('x') as log:
            result = subprocess.run(['timeout','--signal=TERM','--kill-after=30s',str(seconds)+'s',*command],
                cwd=repo,stdout=log,stderr=subprocess.STDOUT)
        row.update(exit=result.returncode,ended_unix=time.time(),log_sha256=hashlib.sha256((checks/(stage+'.log')).read_bytes()).hexdigest())
        save();print('END',stage,result.returncode,flush=True)
        print('\n'.join((checks/(stage+'.log')).read_text().splitlines()[-3:]),flush=True)
        if result.returncode:
            raise RuntimeError('stage failed: '+stage)
    status.update(status='COMPLETE',ended_unix=time.time());save()
except Exception as exc:
    status.update(status='FAILED',error=str(exc),ended_unix=time.time());save();raise
