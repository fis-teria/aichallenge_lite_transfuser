"""Post-selection diagnostic on the actual observed 55--65 cm validation band."""
import ops as m

m.remote(r'''
from pathlib import Path
import json,hashlib,sys
import numpy as np
import torch
torch.set_num_threads(4)
sys.path.insert(0,'tools')
from compare_time_training_methods import pp_rows
from aic_transfuser_lite.data.time_training_cache_v1 import TimeTrainingCacheDataset
from aic_transfuser_lite.evaluation.time_metrics_v1 import time_horizon_metrics
from aic_transfuser_lite.evaluation.time_method_selection_v1 import pp_agreement_score
from aic_transfuser_lite.evaluation.time_recovery_comparison_v1 import component_errors
root=Path('/home/thistle/e2e_autonomous');out=root/'runs/time_recovery_multiscale_20260916/training/comparison'
read=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
summary=read(out/'summary.json');assert summary['status']=='COMPLETE'
plan=read(Path('configs/time_path_p1/recovery_multiscale_20260916.json'))
ds=TimeTrainingCacheDataset(root/plan['cache'],'validation',verify_hashes=False)
assert len(ds)==plan['expected']['validation_total']
index={a:i for i,a in enumerate(ds.anchor_ids)};assert len(index)==len(ds)
manifest_path=Path('docs/evidence/time_recovery_60cm_20260916/manifest.json')
manifest=read(manifest_path)['files'];states={};sources={}
for spec in plan['additions']:
 if spec['split']!='validation' or spec['analysis']!='runs/time_recovery_60cm_20260916':continue
 filename=spec['run_id']+'_anchor_states.json';p=root/spec['analysis']/filename
 assert sha(p)==manifest[filename]['sha256'];sources[filename]=sha(p)
 rows=read(p);assert len(rows)==spec['anchors']
 for row in rows:
  i=index[row['anchor_id']];assert ds.run_ids[i]==spec['run_id'] and i not in states
  states[i]=row
assert len(states)==summary['models']['after']['groups']['new_0.6m']['anchors']
band=[i for i,r in states.items() if .55<=abs(r['lateral_m'])<=.65]
groups={'observed_abs_55_to_65cm':band,
 'observed_left_55_to_65cm':[i for i in band if states[i]['lateral_m']>0],
 'observed_right_55_to_65cm':[i for i in band if states[i]['lateral_m']<0]}
assert all(groups.values())
reports={}
for label in ('before','after'):
 values=np.load(out/(label+'_predictions.npy'),mmap_mode='r',allow_pickle=False)
 assert values.shape==(len(ds),30,2)
 report={}
 for group,ids in groups.items():
  runs=[ds.run_ids[i] for i in ids];pred=np.asarray(values[ids])
  truth=pp_rows(ds,ids,ds.targets[ids],summary['controller'],teacher=True)
  proposed=pp_rows(ds,ids,pred,summary['controller'])
  report[group]=dict(anchors=len(ids),runs=len(set(runs)),
   xy=time_horizon_metrics(torch.from_numpy(pred),torch.from_numpy(ds.targets[ids]),torch.from_numpy(ds.xy_mask[ids]),input_valid=torch.from_numpy(ds.input_valid[ids]),run_ids=runs),
   components=component_errors(pred,ds.targets[ids],ds.xy_mask[ids],ds.input_valid[ids],runs),
   pp=pp_agreement_score(truth,proposed,runs))
 reports[label]=report
proof=dict(status='COMPLETE',scope='POST_SELECTION_OBSERVED_DEVIATION_DIAGNOSTIC_SAME_COURSE_NOT_CLOSED_LOOP',
 selection_changed=False,sealed_test_read=False,summary_sha256=sha(out/'summary.json'),
 frozen_state_manifest_sha256=sha(manifest_path),state_files=sources,
 groups={k:[ds.anchor_ids[i] for i in ids] for k,ids in groups.items()},
 prediction_sha256={k:sha(out/(k+'_predictions.npy')) for k in reports},models=reports)
with (out/'observed_60cm_band.json').open('x') as f:json.dump(proof,f,indent=2,allow_nan=False)
print(json.dumps({label:{g:dict(anchors=v['anchors'],runs=v['runs'],xy3s_m=v['xy']['run_macro_mean']['3s']['raw_error_m'],left3s_m=v['components']['3s']['left_mae_m'],pp_rad=v['pp']['run_macro_penalized_rad'],pp_rejected=v['pp']['candidate_rejected']) for g,v in rows.items()} for label,rows in reports.items()}))
''',native=True,lock=True,timeout=300)
