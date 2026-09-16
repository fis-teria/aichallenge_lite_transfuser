"""Checkpoint/config readiness for the user-requested single simulator trial."""
from dataclasses import fields, replace
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import torch
from aic_transfuser_lite.control.time_trial_v1 import validate_trial_config
from aic_transfuser_lite.data.time_training_cache_v1 import TimeTrainingCacheDataset, verify_time_training_cache
from aic_transfuser_lite.runtime.time_runtime_v1 import TimeRuntimeModel
from aic_transfuser_lite.training.time_checkpoint_v1 import TimeCheckpointIdentity, load_time_checkpoint
from aic_transfuser_lite.training.time_config_v1 import TimeModelConfig, build_time_model

repo=Path('/home/thistle/e2e_autonomous/e2e_lite_transfuser');base=repo.parent
out=base/'runs/time_launch_model_lap_20260917';out.mkdir(exist_ok=False)
prior=base/'runs/time_launch_protection_v2_20260916'
proof=json.loads((prior/'preparation/proof.json').read_bytes())
assert not subprocess.check_output(['git','status','--porcelain']).strip()
protected=['src','tools','tests','ros2_ws','schemas']
assert not subprocess.check_output(['git','diff',proof['source_commit'],'HEAD','--',*protected]).strip()
history=[json.loads(r) for r in (prior/'driver_history.jsonl').read_text().splitlines()]
assert any(r['phase']=='pytest' and r['status']=='PHASE_COMPLETE' and r['exit_code']==0 for r in history)
assert '2800 passed, 4 skipped' in (prior/'pytest.log').read_text()
selection=json.loads((prior/'evaluation_final/selection.json').read_bytes())
assert selection['status']=='NO_NEW_CANDIDATE' and not selection['runtime_test_allowed']
checkpoint=prior/'launch_balanced/epoch_03.pt'
verified=json.loads((checkpoint.parent/'verification.json').read_bytes());assert verified['status']=='PASS'
digest=hashlib.sha256(checkpoint.read_bytes()).hexdigest()
assert digest==verified['checkpoints'][checkpoint.name]=='1fe12ca066791d3eb8907c6921120e2cfbb38925c422d5be54940f4acbdd120a'
cfg_path=repo/'configs/control/time_path_launch_probe_20260917.json'
control=json.loads(cfg_path.read_bytes());old=json.loads((repo/'configs/control/time_path_corner_lap_20260916.json').read_bytes())
assert {k for k in control.keys()|old.keys() if control.get(k)!=old.get(k)}=={'checkpoint_sha256','checkpoint_epoch'}
assert validate_trial_config(control)=='fixed_5kmh' and control['checkpoint_sha256']==digest

command=[sys.executable,'-m','pytest','-q','tests/test_time_runtime_v1.py','tests/test_time_trial_v1.py']
with (out/'focused_pytest.log').open('x') as log:
    tested=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,timeout=120)
assert tested.returncode==0,(out/'focused_pytest.log').read_text()
torch.set_num_threads(4)
runtime=TimeRuntimeModel(checkpoint,expected_sha256=digest,device='cuda')
assert runtime.epoch==control['checkpoint_epoch']==3
cache=base/proof['plan']['cache'];identity=verify_time_training_cache(cache)
assert identity['manifest_sha256']==proof['cache_sha256']
ds=TimeTrainingCacheDataset(cache,'validation',verify_hashes=False)
payload=torch.load(checkpoint,map_location='cpu',weights_only=False)
cfg=TimeModelConfig.from_dict(payload['config']);model=build_time_model(cfg).to('cuda').eval()
load_time_checkpoint(checkpoint,config=cfg,identity=TimeCheckpointIdentity(**payload['identity']),model=model,mode='finetune')
valid=np.flatnonzero(ds.input_valid)
indices=sorted(set(map(int,valid[np.linspace(0,len(valid)-1,12,dtype=int)]))|set(proof['validation']['launch_indices']))
samples=[ds[i] for i in indices]
samples+=torch.load(prior/'preparation/baseline_launch_samples.pt',weights_only=False)
rows=[]
for sample in samples:
    batch=sample.inputs;assert batch is not None and batch.targets is None
    moved=replace(batch,**{f.name:getattr(batch,f.name).to('cuda') for f in fields(batch)
        if isinstance(getattr(batch,f.name),torch.Tensor)})
    with torch.inference_mode():expected=model(moved).float().cpu().numpy()[0]
    actual=runtime.predict(batch);np.testing.assert_array_equal(actual,expected)
    rows.append(dict(anchor_id=sample.anchor_id,run_id=sample.run,max_abs_error_m=0.))
head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()
record=dict(status='PASS',exit=0,source_commit=head,tested_source_commit=proof['source_commit'],
    existing_full_pytest_reused=True,full_pytest_log_sha256=hashlib.sha256((prior/'pytest.log').read_bytes()).hexdigest(),
    tested_implementation_unchanged=True,focused_pytest_command=command,focused_pytest_exit=0,
    focused_pytest_log_sha256=hashlib.sha256((out/'focused_pytest.log').read_bytes()).hexdigest(),
    checkpoint_sha256=digest,checkpoint_epoch=3,config_sha256=hashlib.sha256(cfg_path.read_bytes()).hexdigest(),
    runtime_contract_valid=True,predictions=rows,runtime_controls_changed=False,sealed_test_read=False,
    offline_selection_sha256=hashlib.sha256((prior/'evaluation_final/selection.json').read_bytes()).hexdigest(),
    offline_selection_runtime_test_allowed=False,user_authorization='one exploratory AWSIM run requested 2026-09-17',
    formal_promotion=False)
(out/'deployment_gate.json').write_text(json.dumps(record,indent=2)+'\n')
print(json.dumps(dict(status='NATIVE_RUNTIME_READY',source_commit=head,epoch=3,predictions_exact=len(rows),formal_promotion=False)),flush=True)
