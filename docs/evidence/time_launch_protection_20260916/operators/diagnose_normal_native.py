"""Post-hoc run/speed description; does not alter the frozen selection gate."""
from pathlib import Path
import hashlib
import json
import subprocess
import numpy as np
from aic_transfuser_lite.data.time_training_cache_v1 import TimeTrainingCacheDataset

root=Path('/home/thistle/e2e_autonomous/runs/time_launch_protection_v2_20260916')
proof=json.loads((root/'preparation/proof.json').read_bytes())
assert subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()==proof['source_commit']
assert json.loads((root/'driver_status.json').read_bytes())['status']=='COMPLETE'
ds=TimeTrainingCacheDataset(Path('/home/thistle/e2e_autonomous')/proof['plan']['cache'],'validation',verify_hashes=False)
assert ds.identity['manifest_sha256']==proof['cache_sha256']
selected=proof['validation']['selected_indices'];lookup={i:j for j,i in enumerate(selected)}
nominal=proof['validation']['stages']['nominal'];runs=sorted({ds.run_ids[i] for i in nominal})
assert len(runs)==4
result={};inputs={}
for name in ['initial','protocol_control_epoch3','launch_balanced_epoch3']:
    path=root/'evaluation_final'/(name+'_predictions.npy')
    inputs[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
    p=np.load(path,allow_pickle=False)
    assert p.shape==(len(selected),30,2) and np.isfinite(p).all()
    rows=[]
    for rid in runs:
        ids=[i for i in nominal if ds.run_ids[i]==rid]
        delta=p[[lookup[i] for i in ids]].astype(float)-ds.targets[ids]
        error=np.linalg.norm(delta,axis=2)
        rows.append(dict(run_id=rid,anchors=len(ids),collection_speed_kmh=int(rid.split('kmh_')[0]),
            ade_m=float(error.mean()),endpoint_3s_m=float(error[:,-1].mean()),
            mean_signed_end_x_error_m=float(delta[:,-1,0].mean()),
            mean_signed_end_y_error_m=float(delta[:,-1,1].mean())))
    by_speed={}
    for speed in [5,8]:
        group=[r for r in rows if r['collection_speed_kmh']==speed]
        assert len(group)==2
        by_speed[str(speed)]={k:float(np.mean([r[k] for r in group])) for k in ['ade_m','endpoint_3s_m']}
    result[name]=dict(runs=rows,run_equal_by_collection_speed_kmh=by_speed)
report=dict(source_commit=proof['source_commit'],population_sha256=proof['population_sha256'],
    purpose='POST_HOC_DESCRIPTION_ONLY_NOT_NEW_SELECTION_RULE',nominal_launch_excluded=True,
    new_awsim_runs=0,test_read=False,inputs=inputs,models=result)
with (root/'summary/nominal_run_diagnosis.json').open('x') as f:json.dump(report,f,indent=2)
print(json.dumps({name:r['run_equal_by_collection_speed_kmh'] for name,r in result.items()}))
