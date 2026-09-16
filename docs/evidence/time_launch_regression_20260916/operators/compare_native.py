"""Finite read-only checkpoint diagnosis on existing nominal/launch recordings.

Run inside the native WSL repository under with_wsl_training_lock.sh. Predictions
and evaluator-only future poses never feed the model. No optimizer or ROS writer.
"""
from __future__ import annotations
import argparse
from collections import Counter
from dataclasses import fields, replace
from functools import lru_cache
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
import torch

sys.path.insert(0, str(Path.cwd()/'tools'))
from compare_time_recovery_expansion import motion, speed_at
from aic_transfuser_lite.control.time_reference_v1 import TimePlan, TimedBodyPose, prepare_time_reference
from aic_transfuser_lite.control.polyline_lookahead_v1 import segment_intervals
from aic_transfuser_lite.control.vehicle_motion_v1 import effective_response_length
from aic_transfuser_lite.data.time_training_cache_v1 import TimeTrainingCacheDataset, verify_time_training_cache
from aic_transfuser_lite.data.time_corpus_v1 import EventWindows, audit_anchor
from aic_transfuser_lite.data.time_dataset_v1 import TimeDatasetConfig, assemble_time_sample
from aic_transfuser_lite.data.time_sqlite_reader_v1 import read_time_sqlite_run, load_event
from aic_transfuser_lite.evaluation.time_corner_comparison_v1 import replay_observation_pose, world_points
from aic_transfuser_lite.evaluation.time_recovery_comparison_v1 import pp_probe, summarize_pp
from aic_transfuser_lite.evaluation.time_metrics_v1 import time_horizon_metrics
from aic_transfuser_lite.training.time_checkpoint_v1 import TimeCheckpointIdentity, load_time_checkpoint
from aic_transfuser_lite.training.time_config_v1 import TimeModelConfig, build_time_model

BASE=Path('/home/thistle/e2e_autonomous')
ROOT=BASE/'runs/time_launch_regression_20260916'
TRAIN=BASE/'runs/time_corner_retraining_20260916'
CACHE=BASE/'datasets/cache/time_corner_retraining_20260916'
CONTROL=json.loads(Path('configs/control/time_path_corner_lap_20260916.json').read_bytes())
PLAN=json.loads((TRAIN/'resolved_plan.json').read_bytes())
AGES=(0.,.1,.2,.3,.4,.5)
MODELS={'old':BASE/PLAN['initialization'],**{f'epoch{i}':TRAIN/f'training_serial/epoch_{i:02d}.pt' for i in (1,2,3)}}
SOURCES: dict[str,dict]={}

def sha(path: Path) -> str:
    h=hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()

def pin(path: Path, expected: str|None=None) -> str:
    digest=sha(path)
    if expected is not None and digest!=expected:raise ValueError(f'source changed: {path}')
    SOURCES[str(path)]=dict(bytes=path.stat().st_size,sha256=digest)
    return digest

def write(path: Path, value: object) -> None:
    path.write_bytes((json.dumps(value,indent=2,allow_nan=False)+'\n').encode())

def quantile(v: list[float]|np.ndarray) -> dict:
    values=np.asarray(v,float)
    return dict(zip(('min','median','max'),map(float,np.quantile(values,[0,.5,1])))) if len(values) else {}

def probe(xy: np.ndarray, obs: TimedBodyPose, current: TimedBodyPose, speed: float) -> dict:
    """Existing PP plus reason attribution on the unchanged [30,2] metre path."""
    value=pp_probe(xy,obs,current,speed,CONTROL)
    if not value['applicable']:return value
    if value['accepted']:
        value['selected_steering_margin_rad']=.3-abs(value['steer_rad'])
    if value['reason'] not in ('PP_ACCEPTED','STEERING_FEASIBLE_LOOKAHEAD_MISSING'):return value
    ref=prepare_time_reference(TimePlan('diagnostic',obs,xy),current,
        rear_axle_offset_m=(CONTROL['geometry']['rear_axle_forward_in_base_link_m'],0.))
    minimum=max(1.,.4+max(0.,speed)*.5+speed*speed/2);maximum=minimum+1.
    length=effective_response_length(max(0.,speed),CONTROL['vehicle_model_policy'])
    points=ref.xy_current_m
    radial=sum(len(segment_intervals(a,b,minimum,maximum,length,None)) for a,b in zip(points,points[1:]))
    feasible=sum(len(segment_intervals(a,b,minimum,maximum,length,.3)) for a,b in zip(points,points[1:]))
    value.update(radial_intervals=radial,feasible_intervals=feasible,
        failure_geometry=None if value['accepted'] else ('NO_POINT_IN_DISTANCE_BAND' if radial==0 else 'TIRE_LIMIT'),
        endpoint_rear_m=points[-1].tolist())
    return value

def category(speed: float, endpoint: np.ndarray) -> str:
    distance=float(np.linalg.norm(endpoint))
    if abs(speed)<=.03:
        return 'stationary_wait' if distance<=.1 else ('stationary_transition' if distance<1. else 'stationary_future_launch')
    return 'creep' if abs(speed)<=.25 else 'moving'

def summarize(rows: list[dict], names: list[str]) -> dict:
    result={}
    for group in ['all',*sorted({r['group'] for r in rows})]:
        chosen=[r for r in rows if group=='all' or r['group']==group]
        by_age={}
        for age in AGES:
            current=[r for r in chosen if r['age_s']==age]
            if not current:continue
            by_age[str(age)]={}
            for name in names:
                values=[r['models'][name] for r in current]
                teacher_supported=[r for r in current if r['models']['teacher']['accepted']]
                row=summarize_pp(values)
                row.update(teacher_supported=len(teacher_supported),
                    accepted_on_teacher_support=sum(r['models'][name]['accepted'] for r in teacher_supported),
                    failure_geometry=dict(Counter(v['failure_geometry'] for v in values if v.get('failure_geometry'))),
                    selected_margin_rad=quantile([v['selected_steering_margin_rad'] for v in values if 'selected_steering_margin_rad' in v]))
                by_age[str(age)][name]=row
        result[group]=by_age
    return result

def model_predict(name: str, samples: list, out: Path) -> np.ndarray:
    path=MODELS[name]
    digest=pin(path,PLAN['initialization_sha256'] if name=='old' else None)
    payload=torch.load(path,map_location='cpu',weights_only=False)
    cfg=TimeModelConfig.from_dict(payload['config'])
    assert not cfg.use_command_history
    model=build_time_model(cfg).to('cuda').eval()
    load_time_checkpoint(path,config=cfg,identity=TimeCheckpointIdentity(**payload['identity']),model=model,mode='finetune')
    outputs=[]
    with torch.inference_mode():
        for sample in samples:
            batch=sample.inputs
            assert batch is not None and batch.targets is None
            moved=replace(batch,**{f.name:getattr(batch,f.name).to('cuda') for f in fields(batch)
                                   if isinstance(getattr(batch,f.name),torch.Tensor)})
            value=model(moved).float().cpu().numpy()
            assert value.shape==(1,30,2) and np.isfinite(value).all()
            outputs.append(value[0])
    result=np.stack(outputs)
    np.save(out/(name+'_predictions.npy'),result,allow_pickle=False)
    write(out/(name+'_identity.json'),dict(checkpoint=str(path),sha256=digest,epoch=payload['epoch'],
        batch_size=1,use_command_history=False,source_identity=payload['identity']))
    del model,payload;torch.cuda.empty_cache()
    print('PREDICTED',name,len(samples),flush=True)
    return result

def nominal(out: Path) -> None:
    print('VERIFY_CACHE',flush=True)
    cache=verify_time_training_cache(CACHE)
    assert cache['manifest_sha256']=='9662ddbfd37d5829ba9282f0e65f4c07e964eb1b683c007e95e4da9e606e86e7'
    inventory={};datasets={};windows={};window_info={}
    for split in ('train','validation'):
        ds=TimeTrainingCacheDataset(CACHE,split,verify_hashes=False);datasets[split]=ds
        inv=[];chosen=[]
        for run in ds._runs:
            name=run['path'].name;z=run['inputs'];lab=run['labels'];speed=z['ego'][:,-1,0]
            good=z['input_valid']&lab['xy_mask'].all(axis=1)
            groups=Counter(category(float(v),xy[-1]) for v,xy,ok in zip(speed,lab['xy_m'],good) if ok)
            inv.append(dict(run_id=name,anchors=len(speed),groups=dict(groups),auxiliary_loss=name.startswith('codex-time-recovery-')))
            if name.startswith('codex-time-recovery-'):continue
            indices=[i for i,rid in enumerate(ds.run_ids) if rid==name]
            local=np.flatnonzero(good&(speed>.25))
            assert len(local)
            start=int(ds._anchors[indices[int(local[0])]]['observation_ns'])
            assert any(good[j] and abs(speed[j])<=.03 for j in range(int(local[0])))
            window_info[name]=dict(first_above_0p25mps_ns=start,split=split)
            chosen += [i for i in indices if ds.input_valid[i] and ds.xy_mask[i].all()
                       and start-6_000_000_000<=ds._anchors[i]['observation_ns']<=start+3_000_000_000]
        inventory[split]=inv;windows[split]=chosen
    write(out/'inventory.json',dict(cache_sha256=cache['manifest_sha256'],splits=inventory,windows=window_info,
        window_definition='first measured v>0.25 m/s: -6 through +3 s; input-valid, full teacher; fixed before model inspection'))
    val=datasets['validation'];indices=windows['validation'];sel=[i for i,r in enumerate(val.run_ids) if r in PLAN['selection_run_ids']]
    local={i:j for j,i in enumerate(sel)}
    old_path=BASE/'runs/time_recovery_multiscale_20260916/training/validation_epoch_03.npy'
    pin(old_path);old=np.load(old_path,allow_pickle=False)
    assert old.shape==(len(sel),30,2)
    pin(TRAIN/'comparison/before_predictions.npy')
    np.testing.assert_array_equal(old,np.load(TRAIN/'comparison/before_predictions.npy')[sel])
    predictions={'old':old[[local[i] for i in indices]]}
    for epoch in (1,2,3):
        path=TRAIN/f'training_serial/validation_epoch_{epoch:02d}.npy';pin(path)
        pred=np.load(path,allow_pickle=False);assert pred.shape==old.shape
        metrics=time_horizon_metrics(torch.from_numpy(pred),torch.from_numpy(val.targets[sel]),torch.from_numpy(val.xy_mask[sel]),
            input_valid=torch.from_numpy(val.input_valid[sel]),run_ids=[val.run_ids[i] for i in sel])
        expected=json.loads((TRAIN/f'training_serial/validation_epoch_{epoch:02d}.json').read_bytes())
        for key in ('horizons','all_point_ade_m','run_macro_mean'):assert metrics[key]==expected[key],key
        predictions[f'epoch{epoch}']=pred[[local[i] for i in indices]]
    np.testing.assert_array_equal(predictions['epoch2'],np.load(TRAIN/'comparison/after_predictions.npy')[indices])
    predictions['teacher']=val.targets[indices]
    metadata=[];controls=[];missing=Counter();pose_checks=[];poses={}
    for rid in sorted({val.run_ids[i] for i in indices}):
        raw=BASE/'datasets/raw/time_teacher_20laps_20260911/runs'/rid
        source=next(r for r in val.split_manifest['runs'] if r['run_id']==rid)['sources'][0]
        pin(raw/'bag/bag_0.db3',source['sha256'])
        poses[rid]=motion(raw)
        print('MOTION_READY',rid,flush=True)
    for at,i in enumerate(indices):
        a=val._anchors[i];r,j=val._index[i];z=val._runs[r]['inputs'];v=float(z['ego'][j,-1,0])
        idx,source,ts,vs=poses[a['run_id']]
        obs=replay_observation_pose([source[k] for k in a['observation_pose_row_ids']],
            observation_ns=a['observation_ns'],freeze_receipt_ns=a['freeze_ns'])
        group=category(v,val.targets[i,-1])
        metadata.append(dict(anchor_id=a['anchor_id'],run_id=a['run_id'],group=group,speed_mps=v,
            observation_ns=a['observation_ns'],ego=z['ego'][j,-1].tolist(),teacher_endpoint=val.targets[i,-1].tolist(),
            observation_pose=vars(obs),predicted_endpoints={k:p[at,-1].tolist() for k,p in predictions.items()}))
        for age in AGES:
            try:current=obs if age==0 else idx.at(obs.stamp_ns+round(age*1e9))
            except ValueError as exc:
                if str(exc)!='AMBIGUOUS_POSE_STAMP':raise
                missing[str(age)]+=1;continue
            speed=v if age==0 else speed_at(ts,vs,current.stamp_ns)
            if age:
                delta=float(np.linalg.norm(world_points(val.targets[i],obs)[round(age*10)-1]-[current.x_m,current.y_m]))
                assert delta<1e-5;pose_checks.append(delta)
            controls.append(dict(anchor_id=a['anchor_id'],run_id=a['run_id'],group=group,age_s=age,speed_mps=speed,
                models={name:probe(data[at],obs,current,speed) for name,data in predictions.items()}))
    errors={}
    for group in sorted({r['group'] for r in metadata}):
        ix=[i for i,r in enumerate(metadata) if r['group']==group]
        errors[group]={name:dict(anchors=len(ix),ade_m=float(np.linalg.norm(p[ix]-predictions['teacher'][ix],axis=2).mean()),
            endpoint_norm_m=quantile(np.linalg.norm(p[ix,-1],axis=1))) for name,p in predictions.items()}
    write(out/'anchors.json',metadata);write(out/'controls.json',controls)
    summary=dict(status='PASS',scope='FIXED_VALIDATION_STARTUP_INPUTS_NO_TRAINING',validation_anchors=len(indices),
        pp=summarize(controls,list(predictions)),errors=errors,missing_future_pose=dict(missing),
        future_pose_teacher_max_error_m=max(pose_checks),saved_prediction_metrics_exact=True,test_read=False)
    write(out/'summary.json',summary)
    print('NOMINAL_COMPLETE',json.dumps(summary['pp']['stationary_future_launch']),flush=True)

def baseline(out: Path) -> None:
    samples=[];metadata=[];truth_poses={};source_frames=[]
    types=BASE/'runs/time_recovery_collection_20260913/types'
    for p in sorted(types.rglob('*.idl')):pin(p)
    for rid in ('codex-time-recovery-speedbase-r30','codex-time-recovery-speedbase-r31'):
        raw=BASE/'raw/time_recovery_speed_20260914'/rid
        inventory=json.loads((raw/'transfer_manifest.json').read_bytes())
        inventory={r['path']:r for r in inventory} if isinstance(inventory,list) else inventory
        for name in ('bag/bag_0.db3','control.jsonl','result.json'):pin(raw/name,inventory[name]['sha256'])
        result=json.loads((raw/'result.json').read_bytes())
        assert result['status']=='COMPLETE_LAP' and result['fixed_target_mps']==5/3.6
        assert result['simulator_assets']=={'AWSIM_Data/level1':CONTROL['geometry']['scene_sha256'],**CONTROL['steering_asset_sha256']}
        control=[json.loads(line) for line in (raw/'control.jsonl').read_bytes().splitlines()]
        start=next(r['sim_ns'] for r in control if r.get('reason')=='RECOVERY_TEACHER_TRACKING' and r.get('current_pose'))
        view=out/rid;view.mkdir();(view/'bag').mkdir()
        os.link(raw/'bag/bag_0.db3',view/'bag/bag_0.db3')
        (view/'types').symlink_to(types,target_is_directory=True)
        print('READ_BASELINE',rid,flush=True)
        index=read_time_sqlite_run(view,rid);windows=EventWindows(index.events)
        bounds={e.epoch_id:(e.first_sim_stamp_ns,e.last_sim_stamp_ns) for e in index.epochs}
        cameras={}
        for e in sorted(index.events,key=lambda x:(x.available_ns,x.sequence)):
            if e.role=='camera' and start-2_000_000_000<=e.capture_ns<=start+3_000_000_000:
                cameras.setdefault((e.epoch,e.capture_ns),e)
        cfg=TimeDatasetConfig();excluded=Counter()
        @lru_cache(maxsize=64)
        def loaded(sequence: int):return load_event(view,by_id[sequence])
        by_id={e.sequence:e for e in index.events}
        for anchor in sorted(cameras.values(),key=lambda x:x.capture_ns):
            events=windows.at(anchor)
            teacher,audit=audit_anchor(events,anchor,config=cfg,bounds=bounds[anchor.epoch],
                freeze_ns=anchor.available_ns+50_000_000,intervention_ns=None)
            if not audit['usable_full']:
                excluded[audit['input_invalid_reason'] or str(audit['teacher_reasons'])]+=1;continue
            ids={k for role in ('camera','lidar') for slot in audit['history_row_ids'][role] for k in slot}
            real=tuple(loaded(e.sequence) if e.role in ('camera','lidar') else e for e in events
                       if e.role not in ('camera','lidar') or e.sequence in ids)
            current_anchor=next(e for e in real if e.sequence==anchor.sequence)
            sample=assemble_time_sample(real,current_anchor,config=cfg,epoch_start_ns=bounds[anchor.epoch][0],
                epoch_end_ns=bounds[anchor.epoch][1],freeze_ns=audit['freeze_ns'])
            assert sample.inputs is not None and sample.teacher is not None
            np.testing.assert_array_equal(sample.teacher.xy_m,teacher.xy_m)
            samples.append(sample)
            metadata.append(dict(anchor_id=sample.anchor_id,run_id=rid,group=category(float(sample.inputs.ego[0,-1,0]),teacher.xy_m[-1]),
                observation_ns=sample.observation_ns,after_teacher_arm_s=(sample.observation_ns-start)/1e9,
                ego=sample.inputs.ego[0,-1].tolist(),history_row_ids=audit['history_row_ids'],
                pose_row_ids=audit['observation_pose_row_ids'],freeze_ns=audit['freeze_ns']))
        truth_poses[rid]=motion(view)
        source_frames.append(dict(run_id=rid,anchors=len(cameras),excluded=dict(excluded),armed_ns=start,
            sensor_metadata=index.sensor_metadata,scope='EXISTING_CALIBRATION_BASELINE_DIAGNOSTIC_NOT_UNTOUCHED_TEST'))
        loaded.cache_clear()
        del index,windows,by_id
        print('BASELINE_SAMPLES_READY',rid,len(samples),flush=True)
    assert samples and all(not r['run_id'].startswith(('5kmh','8kmh')) for r in metadata)
    predictions={name:model_predict(name,samples,out) for name in MODELS}
    predictions['teacher']=np.stack([s.teacher.xy_m for s in samples])
    np.save(out/'teachers.npy',predictions['teacher'],allow_pickle=False)
    controls=[];missing=Counter();checks=[]
    for at,(a,sample) in enumerate(zip(metadata,samples)):
        idx,source,ts,vs=truth_poses[a['run_id']]
        obs=replay_observation_pose([source[k] for k in a['pose_row_ids']],observation_ns=a['observation_ns'],freeze_receipt_ns=a['freeze_ns'])
        a['observation_pose']=vars(obs);a['predicted_endpoints']={name:p[at,-1].tolist() for name,p in predictions.items()}
        for age in AGES:
            try:current=obs if age==0 else idx.at(obs.stamp_ns+round(age*1e9))
            except ValueError as exc:
                if str(exc)!='AMBIGUOUS_POSE_STAMP':raise
                missing[str(age)]+=1;continue
            speed=float(sample.inputs.ego[0,-1,0]) if age==0 else speed_at(ts,vs,current.stamp_ns)
            if age:
                delta=float(np.linalg.norm(world_points(sample.teacher.xy_m,obs)[round(age*10)-1]-[current.x_m,current.y_m]))
                assert delta<1e-5;checks.append(delta)
            values={name:probe(p[at],obs,current,speed) for name,p in predictions.items()}
            controls.append(dict(anchor_id=a['anchor_id'],run_id=a['run_id'],group=a['group'],age_s=age,
                after_teacher_arm_s=a['after_teacher_arm_s'],models=values))
    write(out/'anchors.json',metadata);write(out/'controls.json',controls)
    around=[r for r in controls if -.25<=r['after_teacher_arm_s']<=.5]
    summary=dict(status='PASS',scope='PAIRED_EXISTING_SAME_ASSET_BASELINE_INPUTS_NOT_NEW_DRIVING',anchors=len(samples),
        sources=source_frames,pp=summarize(controls,list(predictions)),near_arm_pp=summarize(around,list(predictions)),
        missing_future_pose=dict(missing),future_pose_teacher_max_error_m=max(checks),batch_size=1,
        no_teacher_or_future_pose_model_inputs=True,test_read=False)
    write(out/'summary.json',summary)
    print('BASELINE_COMPLETE',json.dumps(summary['near_arm_pp']['all']),flush=True)

def main() -> None:
    ap=argparse.ArgumentParser();ap.add_argument('mode',choices=('nominal','baseline'));args=ap.parse_args()
    assert not subprocess.check_output(['git','status','--porcelain']).strip()
    assert Path.cwd().resolve()==BASE/'e2e_lite_transfuser'
    torch.set_num_threads(4)
    torch.backends.cuda.matmul.allow_tf32=False;torch.backends.cudnn.allow_tf32=False
    torch.backends.cudnn.benchmark=False;torch.backends.cudnn.deterministic=True
    out=ROOT/args.mode;out.mkdir(parents=True,exist_ok=False)
    pose=TimedBodyPose(1_000_000_000,'sim','0','map','base_link',0.,0.,0.)
    straight=np.column_stack((np.arange(1,31)*.1*1.4,np.zeros(30)))
    assert probe(straight,pose,pose,0.)['accepted']
    assert not probe(straight*.01,pose,pose,0.)['accepted']
    (nominal if args.mode=='nominal' else baseline)(out)
    write(out/'provenance.json',dict(status='PASS',git_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        sources=SOURCES,ages_s=AGES,age_grid='RECORDED_FUTURE_STATE_SENSITIVITY_NOT_CLOSED_LOOP',
        optimizer_steps=0,awsim_executions=0,sealed_test_read=False,pp_smoke_checks=2))

if __name__=='__main__':main()
