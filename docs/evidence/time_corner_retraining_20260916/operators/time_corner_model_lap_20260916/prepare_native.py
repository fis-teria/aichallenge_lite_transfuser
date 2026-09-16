"""Verify the selected checkpoint and unchanged deployment code in native WSL."""
from dataclasses import fields, replace
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np
import torch

from aic_transfuser_lite.control.time_trial_v1 import validate_trial_config
from aic_transfuser_lite.data.time_training_cache_v1 import TimeTrainingCacheDataset, verify_time_training_cache
from aic_transfuser_lite.runtime.time_runtime_v1 import TimeRuntimeModel
from aic_transfuser_lite.training.time_checkpoint_v1 import TimeCheckpointIdentity, load_time_checkpoint
from aic_transfuser_lite.training.time_config_v1 import TimeModelConfig, build_time_model

repo = Path('/home/thistle/e2e_autonomous/e2e_lite_transfuser')
base = repo.parent
out = base/'runs/time_corner_model_lap_20260916'
out.mkdir(exist_ok=False)
checks = base/'runs/time_corner_retraining_serial_checks_20260916'
gate = json.loads((checks/'test_gate.json').read_bytes())
assert gate['exit'] == 0
assert hashlib.sha256((checks/'pytest.log').read_bytes()).hexdigest() == gate['log_sha256']
protected = ['src', 'tools', 'tests', 'ros2_ws', 'schemas']
assert not subprocess.check_output(['git','diff',gate['source_commit'],'HEAD','--',*protected],cwd=repo).strip()
assert json.loads((checks/'pipeline_status.json').read_bytes())['status'] == 'COMPLETE'
interrupted_log = base/'runs/time_corner_retraining_checks_20260916/train.log'
serial_log = checks/'train.log'
def training_prefix(path):
    rows = [json.loads(line) for line in path.read_text().splitlines() if line.startswith('{')]
    return {r['optimizer_steps']:r for r in rows if r.get('phase')=='TRAINING'
            and r.get('epoch')==1 and 'train_l1_m' in r}
old_prefix, new_prefix = training_prefix(interrupted_log), training_prefix(serial_log)
assert old_prefix and set(old_prefix) <= set(new_prefix)
keys = ['epoch_cursor','train_l1_m','learning_rate']
assert all(all(old[k]==new_prefix[step][k] for k in keys) for step,old in old_prefix.items())
(out/'loader_prefix_parity.json').write_text(json.dumps(dict(status='PASS',
    compared_logged_steps=sorted(old_prefix),fields=keys,parameters_or_optimizer_states_compared=False,
    interrupted_log_sha256=hashlib.sha256(interrupted_log.read_bytes()).hexdigest(),
    serial_log_sha256=hashlib.sha256(serial_log.read_bytes()).hexdigest()),indent=2))
checkpoint = base/'runs/time_corner_retraining_20260916/training_serial/best.pt'
proof = json.loads((checkpoint.parent/'verification.json').read_bytes())
assert proof['status'] == 'PASS'
digest = hashlib.sha256(checkpoint.read_bytes()).hexdigest()
assert proof['checkpoint_sha256'] == digest
config_path = repo/'configs/control/time_path_corner_lap_20260916.json'
control = json.loads(config_path.read_bytes())
assert validate_trial_config(control) == 'fixed_5kmh'
old = json.loads((repo/'configs/control/time_path_multiscale_lap_20260916.json').read_bytes())
assert {k for k in control.keys()|old.keys() if control.get(k)!=old.get(k)} <= {'checkpoint_sha256','checkpoint_epoch'}
assert control['checkpoint_sha256'] == digest
torch.set_num_threads(4)
runtime = TimeRuntimeModel(checkpoint, expected_sha256=digest, device='cuda')
assert runtime.epoch == control['checkpoint_epoch']
cache_root = base/'datasets/cache/time_corner_retraining_20260916'
cache = verify_time_training_cache(cache_root)
dataset = TimeTrainingCacheDataset(cache_root, 'validation', verify_hashes=False)
payload = torch.load(checkpoint, map_location='cpu', weights_only=False)
config = TimeModelConfig.from_dict(payload['config'])
model = build_time_model(config).to('cuda').eval()
load_time_checkpoint(checkpoint, config=config, identity=TimeCheckpointIdentity(**payload['identity']),model=model,mode='finetune')
valid = np.flatnonzero(dataset.input_valid)
rows = []
for i in valid[np.linspace(0,len(valid)-1,12,dtype=int)]:
    sample = dataset[int(i)]
    batch = sample.inputs
    assert batch is not None and batch.targets is None
    moved = replace(batch, **{f.name:getattr(batch,f.name).to('cuda') for f in fields(batch)
                             if isinstance(getattr(batch,f.name),torch.Tensor)})
    with torch.inference_mode():
        expected = model(moved).float().cpu().numpy()[0]
    actual = runtime.predict(batch)
    np.testing.assert_array_equal(actual,expected)
    rows.append(dict(anchor_id=sample.anchor_id,run_id=sample.run,max_abs_error_m=0.0))
head = subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip()
proof = dict(status='PASS',exit=0,source_commit=head,tested_source_commit=gate['source_commit'],
    tested_code_unchanged=True,test_log_sha256=gate['log_sha256'],checkpoint_sha256=digest,
    checkpoint_epoch=runtime.epoch,config_sha256=hashlib.sha256(config_path.read_bytes()).hexdigest(),
    cache_sha256=cache['manifest_sha256'],runtime_contract_valid=True,predictions=rows,
    runtime_controls_changed=False,sealed_test_read=False)
(out/'deployment_gate.json').write_text(json.dumps(proof,indent=2))
print(json.dumps(dict(status='NATIVE_RUNTIME_READY',source_commit=head,checkpoint_sha256=digest,epoch=runtime.epoch,predictions_exact=len(rows))),flush=True)
