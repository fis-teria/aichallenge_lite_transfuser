"""Finite same-data departure diagnostic: train 162, validation 53 anchors."""
from pathlib import Path
from dataclasses import replace
from collections import Counter
import json
import sys
import numpy as np
import torch
sys.path.insert(0,str(Path.cwd()/'tools'))
from compare_time_training_methods import pp_rows
from aic_transfuser_lite.data.time_training_cache_v1 import TimeTrainingCacheDataset
from aic_transfuser_lite.runtime.time_runtime_v1 import TimeRuntimeModel
from aic_transfuser_lite.training.train_time_v1 import training_batch
from aic_transfuser_lite.evaluation.time_recovery_comparison_v1 import summarize_pp, component_errors

plan=json.loads(Path('configs/control/time_objective_driving_comparison_20260915.json').read_text())
root=Path(plan['native_root']);audit=json.loads((root/'departure_data_audit.json').read_text())
cache=Path('/home/thistle/e2e_autonomous/datasets/cache/time_recovery_random_update_20260915')
controller=json.loads(Path(plan['models']['A']['config']).read_text())
out=root/'departure_comparison';out.mkdir(exist_ok=False)
datasets={s:TimeTrainingCacheDataset(cache,s,verify_hashes=False) for s in ('train','validation')}
indices={}
for split,ds in datasets.items():
 assert ds.identity['manifest_sha256']==audit['cache_manifest_sha256']
 by_run={r['run_id']:set(r['departure_candidate_indices']) for r in audit['splits'][split]['runs']}
 indices[split]=[i for i,(run,local) in enumerate(ds._index) if local in by_run[ds._runs[run]['path'].name]]
 assert len(indices[split])==audit['splits'][split]['totals']['under_0p1_and_teacher_3s_displacement_over_1m']
result=dict(scope='Same recorded departure candidates; batched FP32, PP age zero, no scan or closed-loop claim',models={},teacher={},sealed_test_read=False)
for split,ds in datasets.items():
 ix=indices[split];teacher=pp_rows(ds,ix,ds.targets[ix],controller,teacher=True)
 result['teacher'][split]=summarize_pp(teacher)
for arm,m in plan['models'].items():
 runtime=TimeRuntimeModel(Path(m['checkpoint']),expected_sha256=m['sha256'],device='cuda')
 result['models'][arm]=dict(checkpoint_sha256=m['sha256'],splits={})
 for split,ds in datasets.items():
  ix=indices[split];pred=[]
  for start in range(0,len(ix),16):
   samples=[ds[i] for i in ix[start:start+16]]
   batch=replace(training_batch(samples,torch.device('cuda')),targets=None)
   with torch.inference_mode():values=runtime.model(batch).float().cpu().numpy()
   assert values.shape==(len(samples),30,2) and np.isfinite(values).all();pred.append(values)
  values=np.concatenate(pred);np.save(out/(arm+'_'+split+'_prediction.npy'),values,allow_pickle=False)
  probes=pp_rows(ds,ix,values,controller)
  result['models'][arm]['splits'][split]=dict(pp=summarize_pp(probes),
     errors=component_errors(values,ds.targets[ix],ds.xy_mask[ix],ds.input_valid[ix],[ds.run_ids[i] for i in ix]))
  print(json.dumps(dict(arm=arm,split=split,pp=result['models'][arm]['splits'][split]['pp'])),flush=True)
 del runtime;torch.cuda.empty_cache()
with (out/'summary.json').open('x') as f:json.dump(result,f,indent=2)
