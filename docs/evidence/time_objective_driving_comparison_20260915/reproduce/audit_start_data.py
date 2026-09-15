"""Count recorded stationary/departure teachers; no new training or test access."""
from pathlib import Path
import json
import numpy as np
from aic_transfuser_lite.data.time_training_cache_v1 import TimeTrainingCacheDataset, _sha

cache=Path('/home/thistle/e2e_autonomous/datasets/cache/time_recovery_random_update_20260915')
root=Path('/home/thistle/e2e_autonomous/runs/time_objective_driving_comparison_20260915_r2')
out={}
for split in ('train','validation'):
 ds=TimeTrainingCacheDataset(cache,split,verify_hashes=False)
 assert ds.identity['manifest_sha256']=='cb52a01fae1a492b5d06f1473c6ed773410934fe39a019e91c33097db340c895'
 inventory={r['path']:r for r in ds.identity['cache_files']};counts=[]
 for run in ds._runs:
  for name in ('inputs.npz','labels.npz','anchors.jsonl'):
   p=run['path']/name;entry=inventory[p.relative_to(cache).as_posix()]
   assert p.stat().st_size==entry['bytes'] and _sha(p)==entry['sha256']
  x,y=run['inputs'],run['labels'];v=x['ego'][:,-1,0]
  valid=x['input_valid']&y['xy_mask'].all(axis=1)&x['ego_mask'][:,-1,0]
  near_stop=valid&(np.abs(v)<.1)
  endpoint=np.linalg.norm(y['xy_m'][:,-1,:],axis=1)
  starts=near_stop&(endpoint>1.)
  departure_indices=np.flatnonzero(starts).tolist()
  counts.append(dict(run_id=run['path'].name,total_anchors=len(v),fully_supported=int(valid.sum()),
      current_speed_abs_under_0p1_mps=int(near_stop.sum()),
      under_0p1_and_teacher_3s_displacement_over_1m=int(starts.sum()),
      departure_candidate_indices=departure_indices,
      departure_teacher_endpoint_m=[float(a) for a in endpoint[starts]],
      departure_teacher_lateral_3s_m=[float(a) for a in y['xy_m'][starts,-1,1]]))
 out[split]=dict(runs=counts,totals={k:sum(r[k] for r in counts) for k in
    ['total_anchors','fully_supported','current_speed_abs_under_0p1_mps','under_0p1_and_teacher_3s_displacement_over_1m']},
    independent_runs_with_departure_candidates=sum(r['under_0p1_and_teacher_3s_displacement_over_1m']>0 for r in counts))
result=dict(scope='Observed dataset support; thresholds define candidate states, not a performance guarantee',
    cache_manifest_sha256=ds.identity['manifest_sha256'],source_files_hashed=True,
    current_velocity_feature='ego[-1,0] longitudinal m/s',future_displacement='Euclidean norm of observed 3s endpoint in anchor base_link',splits=out,sealed_test_read=False)
with (root/'departure_data_audit.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps({s:{k:v for k,v in row.items() if k!='runs'} for s,row in out.items()}),flush=True)
