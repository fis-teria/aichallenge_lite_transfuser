from pathlib import Path
import json, hashlib
import numpy as np
root=Path('/home/thistle/e2e_autonomous')
experiment=root/'runs/time_native_obstacle10_replay_20260918'
collection=root/'runs/mppi_v45_pc10_20260918'
output=root/'runs/native10_teacher_fit_20260919'
output.mkdir(exist_ok=False)
def read(p):return json.loads(p.read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
proof=read(experiment/'preparation/proof.json')
manifest_path=experiment/'preparation/native_manifest.json'
assert sha(manifest_path)==proof['native_manifest_sha256']
manifest=read(manifest_path)
labels={}; source_of={}; uses={}; sources=[]
for source in proof['plan']['native_sources']:
    directory=collection/source['native_selection']
    m=read(directory/'selection_manifest.json')
    assert sha(directory/'selection_manifest.json')==source['selection_manifest_sha256']
    for run in m['runs']:
        if not run['selected']:continue
        name=run['run_id']
        for rel in (name+'/selected_anchors.jsonl',name+'/selected_teachers.npz'):
            assert sha(directory/rel)==m['output_sha256'][rel]
        rows=[json.loads(s) for s in (directory/name/'selected_anchors.jsonl').read_text().splitlines()]
        with np.load(directory/name/'selected_teachers.npz',allow_pickle=False) as z:
            xy=z['xy_m'].copy(); stamps=z['observation_ns'].copy()
        assert xy.shape==(len(rows),30,2) and np.isfinite(xy).all()
        assert np.array_equal(stamps,[row['observation_ns'] for row in rows])
        for row,target in zip(rows,xy):
            a=row['anchor_id'];assert a not in labels
            labels[a]=target;source_of[a]=source['native_selection'];uses[a]=row['selection_use']
    sources.append(dict(selection=source['native_selection'],expected=source['native_anchors']))
anchors=[a['anchor_id'] for shard in manifest['shards'] for a in shard['anchors']]
assert set(anchors)==set(labels) and len(anchors)==442
teacher=np.stack([labels[a] for a in anchors])
predictions={name:np.load(experiment/'evaluation'/(name+'_native_predictions.npy'),allow_pickle=False) for name in ('initial','epoch_01','epoch_02','epoch_03')}
assert all(x.shape==teacher.shape and np.isfinite(x).all() for x in predictions.values())
geom={}
old_geometry=root/'runs/avoidance_diagnosis_20260918/training_windows.jsonl'
for row in map(json.loads,old_geometry.read_text().splitlines()):geom[row['anchor_id']]=row['cone_body_xy_m']
for path in (collection/'collect10_validation_v1').glob('*-geometry.json'):
    for row in read(path):geom[row['anchor_id']]=row['object_body_xy_m']
assert all(a in geom for a in anchors)
front=[i for i,a in enumerate(anchors) if uses[a]=='static_cone_xy_speed' and geom[a][0]>0]
substantial=[i for i in front if abs(teacher[i,-1,1])>.2]
groups={'all_442':list(range(442)),'front_cone_context':front,'front_teacher_abs_endpoint_y_gt_020m':substantial}
for s in sources:groups[s['selection']]=[i for i,a in enumerate(anchors) if source_of[a]==s['selection']]
metrics={}
for name,pred in predictions.items():
    metrics[name]={}
    for group,idx in groups.items():
        t,p=teacher[idx],pred[idx]
        metrics[name][group]=dict(anchors=len(idx),ade_sample_mean_m=float(np.linalg.norm(p-t,axis=-1).mean()),
            endpoint_sample_mean_m=float(np.linalg.norm(p[:,-1]-t[:,-1],axis=-1).mean()),
            endpoint_y_mae_m=float(np.abs(p[:,-1,1]-t[:,-1,1]).mean()),
            endpoint_y_abs_less_than_half_teacher=int((np.abs(p[:,-1,1])<.5*np.abs(t[:,-1,1])).sum()),
            endpoint_y_opposite_sign=int((p[:,-1,1]*t[:,-1,1]<0).sum()))
unused=read(collection/'front_validation_v1/summary.json')
unused_runs=unused['runs']
example=anchors.index('lidar-v45-pc10-corners-native-a03:epoch0000:10494999765')
report=dict(checkpoint_sha256=read(experiment/'evaluation/epoch_03.json')['checkpoint_sha256'],
    inference_scope='Saved predictions on matched training inputs, no new training or runtime change',
    metrics=metrics, averaging='sample mean, unlike run macro in final evaluation',
    front_definition='selected static cone context and associated cone x>0; historical old geometry uses nearest selected cone, not every physical obstacle',
    lateral_caveat='Teacher endpoint lateral displacement includes road curvature; these counts alone do not label an avoidance maneuver.',
    presentations=dict(old=60608,native=4420,cone_context=3260,total=65028,native_fraction=4420/65028,cone_fraction=3260/65028),
    example=dict(anchor_id=anchors[example],teacher_endpoint_xy_m=teacher[example,-1].tolist(),new_endpoint_xy_m=predictions['epoch_03'][example,-1].tolist()),
    unused_selection=dict(path=str(collection/'front_validation_v1/summary.json'),sha256=sha(collection/'front_validation_v1/summary.json'),
        selected_total=sum(r['selected'] for r in unused_runs),run_ids=[r['run_id'] for r in unused_runs],
        any_run_in_training=any(r['run_id'] in {s['run_id'] for s in manifest['shards']} for r in unused_runs)),
    hashes=dict(native_manifest=sha(manifest_path),teacher_selections=[s['selection_manifest_sha256'] for s in proof['plan']['native_sources']],
        predictions={n:sha(experiment/'evaluation'/(n+'_native_predictions.npy')) for n in predictions}))
(output/'summary.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
