from pathlib import Path
import json,gc
import numpy as np
import torch
from aic_transfuser_lite.contracts.model_batch_v3 import ModelBatchV3
from aic_transfuser_lite.runtime.time_runtime_v1 import TimeRuntimeModel
ROOT=Path('/home/thistle/e2e_autonomous');out=ROOT/'runs/native_fit_factorial_20260918'
cap=ROOT/'runs/avoidance_diagnosis_20260918/capture_r3';torch.set_num_threads(2)
torch.backends.cuda.matmul.allow_tf32=False;torch.backends.cudnn.allow_tf32=False
torch.backends.cudnn.benchmark=False;torch.backends.cudnn.deterministic=True
data=np.load(cap/'replayed_batch_arrays.npz');rows=json.loads((cap/'input_replay.json').read_text())['replayed'];results=[]
for name in ['initial','control','focused','geometry','focused_geometry']:
    info=json.loads((out/(name+'.json')).read_text())
    model=TimeRuntimeModel(Path(info['checkpoint']),expected_sha256=info['checkpoint_sha256'],device='cuda')
    cases=[]
    for i,row in enumerate(rows):
        tensors={key[len(str(i))+1:]:torch.from_numpy(data[key].copy()) for key in data.files if key.startswith(str(i)+'_')}
        batch=ModelBatchV3(**tensors,targets=None,requested_outputs=frozenset({'trajectory'}));p=model.predict(batch)
        if name=='initial':np.testing.assert_allclose(p[-1],row['endpoint_xy_m'],atol=2e-5,rtol=0)
        cases.append(dict(observation_s=row['observation_s'],speed_kmh=row['actual_speed_kmh'],endpoint_m=p[-1].tolist(),
            maximum_abs_lateral_m=float(abs(p[:,1]).max()),xy_m=p.tolist()))
    results.append(dict(arm=name,checkpoint_sha256=model.sha256,cases=cases));del model;gc.collect();torch.cuda.empty_cache()
report=dict(scope='Weight-only replay on 10 previously reconstructed actual failure inputs; development diagnostic, not a new closed-loop avoidance test',results=results)
(out/'actual_failure_replay.json').write_text(json.dumps(report,indent=2))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
i=min(range(len(rows)),key=lambda j:abs(rows[j]['observation_s']-31.914999286))
fig,axes=plt.subplots(1,2,figsize=(10,4))
rgb=data[f'{i}_image'][0,-1].transpose(1,2,0)
rgb=np.uint8(np.clip((rgb*np.array([.229,.224,.225])+np.array([.485,.456,.406]))*255,0,255))
axes[0].imshow(rgb);axes[0].axis('off');axes[0].set_title('Actual recorded model image')
scan=data[f'{i}_lidar'][0,-1];theta=-np.pi+np.arange(750)*2*np.pi/750;valid=scan[1]>.5;r=scan[0]*25
axes[1].scatter(r[valid]*np.cos(theta[valid])+1.65,r[valid]*np.sin(theta[valid]),s=4,c='gray',label='Recorded LiDAR input')
for item in results:
    xy=np.array(item['cases'][i]['xy_m']);axes[1].plot(xy[:,0],xy[:,1],label=item['arm'])
axes[1].set(xlim=(-.5,5),ylim=(-2,2),aspect='equal',xlabel='Forward [m]',ylabel='Left [m]')
axes[1].grid(alpha=.3);axes[1].legend(fontsize=7)
fig.suptitle(f"Same input at {rows[i]['observation_s']:.3f} s: offline predictions, no closed-loop claim")
fig.tight_layout();fig.savefig(out/'actual_failure_replay.png',dpi=150,bbox_inches='tight');plt.close(fig)
print(json.dumps([dict(arm=r['arm'],cases=[{k:v for k,v in c.items() if k!='xy_m'} for c in r['cases'] if 31.8<c['observation_s']<32.0]) for r in results],indent=2))
