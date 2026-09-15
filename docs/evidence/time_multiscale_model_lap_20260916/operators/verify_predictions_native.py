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

assert json.loads((root/'test_gate.json').read_text())['exit']==0
source=root.parent/'time_recovery_multiscale_20260916/training/best.pt'
digest='685f8b8f9936ab7e272be12616982374fd8a8d61fbe4a304b25475dc2a206ba8'
head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()
cache_root=Path('/home/thistle/e2e_autonomous/datasets/cache/time_recovery_multiscale_20260916')
cache=json.loads((cache_root/'identity.json').read_text())
proof=json.loads((root/'multiscale_runtime.export.json').read_text())
exported=root/'multiscale_runtime.pt'
assert hashlib.sha256(exported.read_bytes()).hexdigest()==proof['output_sha256']
assert not (root/'prediction_identity.json').exists()
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
