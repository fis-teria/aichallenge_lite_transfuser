"""Executed in native WSL under the existing worktree lock."""
from dataclasses import fields, replace
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
import torch

from aic_transfuser_lite.data.time_training_cache_v1 import TimeTrainingCacheDataset, verify_time_training_cache
from aic_transfuser_lite.runtime.time_runtime_v1 import TimeRuntimeModel
from aic_transfuser_lite.training.time_checkpoint_v1 import TimeCheckpointIdentity, load_time_checkpoint
from aic_transfuser_lite.training.time_config_v1 import TimeModelConfig, build_time_model
from tools.export_time_recovery_runtime_checkpoint import export_runtime_checkpoint

root = Path('/home/thistle/e2e_autonomous/runs/time_multiscale_model_lap_20260916')
root.mkdir(exist_ok=False)
head = subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip()
source = root.parent/'time_recovery_multiscale_20260916/training/best.pt'
digest = '685f8b8f9936ab7e272be12616982374fd8a8d61fbe4a304b25475dc2a206ba8'
try:
    TimeRuntimeModel(source, expected_sha256=digest, device='cpu')
except ValueError as exc:
    assert str(exc) == 'TEACHER_RUNTIME_CONTRACT_MISMATCH'
    (root/'original_runtime_refusal.json').write_text(json.dumps(dict(reason=str(exc), sha256=digest)))
else:
    raise AssertionError('Expected the preserved original to lack the runtime contract')
begin = time.monotonic()
with (root/'pytest.log').open('x') as log:
    test = subprocess.run([sys.executable, '-m', 'pytest', '-q'], stdout=log, stderr=subprocess.STDOUT, timeout=300)
gate = dict(source_commit=head, exit=test.returncode, seconds=time.monotonic()-begin)
(root/'test_gate.json').write_text(json.dumps(gate, indent=2))
print(json.dumps(gate), flush=True)
print('\n'.join((root/'pytest.log').read_text().splitlines()[-4:]), flush=True)
assert test.returncode == 0
cache_root = Path('/home/thistle/e2e_autonomous/datasets/cache/time_recovery_multiscale_20260916')
cache = verify_time_training_cache(cache_root)
assert cache['manifest_sha256'] == 'f2b463b73f1d1cb7993b151754ca16488fcbd5748887375ee71b30d56316efc9'
exported = root/'multiscale_runtime.pt'
proof = export_runtime_checkpoint(source, expected_sha256=digest, cache=cache, output=exported)
(root/'multiscale_runtime.export.json').write_text(json.dumps(proof, indent=2))
runtime = TimeRuntimeModel(exported, expected_sha256=proof['output_sha256'], device='cuda')
payload = torch.load(source, map_location='cpu', weights_only=False)
cfg = TimeModelConfig.from_dict(payload['config'])
model = build_time_model(cfg)
load_time_checkpoint(source, config=cfg, identity=TimeCheckpointIdentity(**payload['identity']), model=model, mode='finetune')
model.to('cuda').eval()
dataset = TimeTrainingCacheDataset(cache_root, split='validation', verify_hashes=False)
valid = np.flatnonzero(dataset.input_valid)
indices = valid[np.linspace(0, len(valid)-1, 12, dtype=int)]
rows = []
for index in indices:
    sample = dataset[int(index)]
    batch = sample.inputs
    assert batch is not None and batch.targets is None
    moved = replace(batch, **{f.name: getattr(batch, f.name).to('cuda') for f in fields(batch)
                             if isinstance(getattr(batch, f.name), torch.Tensor)})
    with torch.inference_mode():
        expected = model(moved).float().cpu().numpy()[0]
    actual = runtime.predict(batch)
    np.testing.assert_array_equal(actual, expected)
    rows.append(dict(anchor_id=sample.anchor_id, run_id=sample.run, max_error_m=float(np.max(np.abs(actual-expected)))))
assert hashlib.sha256(source.read_bytes()).hexdigest() == digest
result = dict(status='PASS', source_commit=head, original_sha256=digest, runtime_sha256=proof['output_sha256'],
              cache_manifest_sha256=cache['manifest_sha256'], verified_cache=True, samples=rows,
              model_state_entries_equal=proof['state_entries_equal'], precision='float32', device='cuda',
              runtime_gate_unchanged=True, original_checkpoint_preserved=True)
(root/'prediction_identity.json').write_text(json.dumps(result, indent=2))
print(json.dumps(dict(status='NATIVE_READY', runtime_sha256=proof['output_sha256'], predictions_exact=len(rows))), flush=True)
