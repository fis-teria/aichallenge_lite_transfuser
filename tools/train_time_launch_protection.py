"""Audit simulator-ready teachers, compare fixed-budget launch sampling, gate promotion."""
from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict, fields, replace
from functools import lru_cache
import json
from pathlib import Path
import resource
import sqlite3
import subprocess
from typing import Any

import numpy as np
import torch
from torch.utils.data import Subset

from compare_time_recovery_expansion import motion, speed_at
from compare_time_recovery_objectives import make_objective
from train_time_corner_recovery import build_sampler
from aic_transfuser_lite.control.time_reference_v1 import TimedBodyPose
from aic_transfuser_lite.data.time_corpus_v1 import EventWindows, audit_anchor
from aic_transfuser_lite.data.time_dataset_v1 import TimeDatasetConfig, assemble_time_sample
from aic_transfuser_lite.data.time_launch_training_v1 import DriveWindow, LaunchQuotaDataset
from aic_transfuser_lite.data.time_split_v1 import content_sha256
from aic_transfuser_lite.data.time_sqlite_reader_v1 import read_time_sqlite_run, load_event, _store
from aic_transfuser_lite.data.time_training_cache_v1 import TimeTrainingCacheDataset, verify_time_training_cache, _sha
from aic_transfuser_lite.evaluation.time_batched_v1 import evaluate_time_batched
from aic_transfuser_lite.evaluation.time_corner_comparison_v1 import replay_observation_pose, world_points
from aic_transfuser_lite.evaluation.time_recovery_comparison_v1 import pp_probe
from aic_transfuser_lite.evaluation.time_stage_selection_v1 import StageSelectionPolicy, select_stage_candidate
from aic_transfuser_lite.training.time_checkpoint_v1 import TimeCheckpointIdentity, load_time_checkpoint
from aic_transfuser_lite.training.time_config_v1 import TimeModelConfig, build_time_model
from aic_transfuser_lite.training.time_corpus_runner_v1 import CorpusTrainingPlan, run_training_arm


def read(path: Path) -> Any:
    return json.loads(path.read_bytes())


def write(path: Path, value: Any) -> None:
    path.write_bytes((json.dumps(value,indent=2,allow_nan=False)+'\n').encode())


def source_guard(repo: Path, plan: dict[str,Any]) -> str:
    if subprocess.check_output(['git','status','--porcelain'],cwd=repo).strip():
        raise ValueError('clean committed Windows-synchronized source required')
    protected=['src/aic_transfuser_lite/models','src/aic_transfuser_lite/training',
        'src/aic_transfuser_lite/contracts','src/aic_transfuser_lite/control',
        'src/aic_transfuser_lite/data/time_dataset_v1.py','src/aic_transfuser_lite/data/time_training_cache_v1.py',
        'src/aic_transfuser_lite/data/time_teacher_v1.py','src/aic_transfuser_lite/evaluation/time_batched_v1.py']
    if subprocess.check_output(['git','diff',plan['source_guard_commit'],'HEAD','--',*protected],cwd=repo):
        raise ValueError('loss, model, input, controller or training implementation changed')
    if plan['test_usage']!='sealed' or plan['loss_changes'] or plan['model_input_changes'] or plan['awsim_modifications']:
        raise ValueError('fixed objective/model/runtime and sealed test required')
    return subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip()


def ready_window(raw: Path) -> tuple[DriveWindow,dict[str,Any]]:
    """Bound Ready by its receipt and next /clock, without guessing start+3 s."""
    store=_store(raw);states=[];sim=None;available=None;upper=None
    db=raw/'bag/bag_0.db3'
    with sqlite3.connect(f'file:{db}?mode=ro',uri=True) as connection:
        topics={i:(n,t) for i,n,t in connection.execute(
            "SELECT id,name,type FROM topics WHERE name IN ('/clock','/awsim/state')")}
        if {n for n,_ in topics.values()}!={'/clock','/awsim/state'}:
            raise ValueError('clock and explicit simulator state required')
        ids=list(topics)
        rows=connection.execute('SELECT id,topic_id,timestamp,data FROM messages WHERE topic_id IN ('+
            ','.join('?' for _ in ids)+') ORDER BY timestamp,id',ids)
        for row_id,topic,receipt,blob in rows:
            name,kind=topics[topic];message=store.deserialize_cdr(blob,kind)
            if name=='/clock':
                stamp=int(message.clock.sec)*1_000_000_000+int(message.clock.nanosec)
                if sim is not None and stamp<sim:raise ValueError('clock reset before Ready')
                sim=stamp
                if available is not None:
                    upper=stamp;break
            elif not states or states[-1]['state']!=message.data:
                states.append(dict(row_id=int(row_id),state=message.data,sim_lower_ns=sim,available_ns=int(receipt)))
                if message.data=='Ready':available=int(receipt)
    if (upper is None or available is None or len(states)<2
            or [r['state'] for r in states[-2:]]!=['Start','Ready']
            or states[-1]['sim_lower_ns'] is None or not 0<=upper-states[-1]['sim_lower_ns']<=50_000_000):
        raise ValueError('bounded recorded Start-to-Ready transition required')
    summary=read(raw/'probe_summary.json')
    if summary['fault'] is not None or not summary['stop_confirmed'] or summary.get('lap_completed_sim') is None:
        raise ValueError('completed nominal teacher run required')
    brake=round(summary['brake_sim']*1e9)
    window=DriveWindow(int(upper),available,int(brake))
    return window,dict(window=asdict(window),state_transitions=states,
        probe_summary_sha256=_sha(raw/'probe_summary.json'),
        ready_bound='first Ready receipt, conservatively bounded by the next received clock')


def sampler_context(root: Path,plan: dict[str,Any],proof: dict[str,Any],full: Any) -> tuple[dict[str,Any],Any]:
    parent=root/plan['parent_experiment'];old=read(parent/'resolved_plan.json')
    recovery=sorted(r for r in set(full.run_ids) if r.startswith('codex-time-recovery-'))
    targets=read(parent/'data_and_budget_verification.json')['target_anchor_ids']
    reference=build_sampler(full,recovery,targets,repeats=old['recovery_repeats'],
        fraction=old['balanced_target_fraction'],seed=plan['training']['seed'])
    if len(reference)!=plan['presentations_per_epoch']:
        raise ValueError('reference presentation budget changed')
    samplers={name:LaunchQuotaDataset(full,reference.indices,
        eligible_indices=proof['train']['eligible_indices'],launch_indices=proof['train']['launch_indices'],
        recovery_run_ids=recovery,launch_fraction=fraction,seed=plan['training']['seed'])
        for name,fraction in [('protocol_control',None),('launch_balanced',plan['launch_fraction'])]}
    return samplers,old


def prepare(root: Path,repo: Path,plan: dict[str,Any]) -> None:
    head=source_guard(repo,plan);out=root/plan['output']/'preparation';out.mkdir(parents=True,exist_ok=False)
    identity=verify_time_training_cache(root/plan['cache'])
    if identity['manifest_sha256']!=plan['cache_sha256']:raise ValueError('cache changed')
    if _sha(root/plan['initialization'])!=plan['initialization_sha256']:raise ValueError('initial weights changed')
    proof=dict(source_commit=head,plan=plan,cache_sha256=identity['manifest_sha256'],raw_sources={},runs=[])
    datasets={}
    for split in ['train','validation']:
        ds=TimeTrainingCacheDataset(root/plan['cache'],split,verify_hashes=False);datasets[split]=ds
        eligible=[];launch=[];excluded=[]
        by_run={r:[] for r in set(ds.run_ids)}
        for i,r in enumerate(ds.run_ids):by_run[r].append(i)
        for rid,indices in sorted(by_run.items()):
            if rid.startswith('codex-time-recovery-'):
                eligible.extend(indices);continue
            raw=root/'datasets/raw/time_teacher_20laps_20260911/runs'/rid
            digest=_sha(raw/'bag/bag_0.db3')
            declared=next(r for r in ds.split_manifest['runs'] if r['run_id']==rid)['sources'][0]['sha256']
            if digest!=declared:raise ValueError('nominal raw bag changed')
            proof['raw_sources'][str(raw/'bag/bag_0.db3')]=digest
            window,audit=ready_window(raw);run_launch=[];reasons=Counter()
            for i in indices:
                anchor=ds._anchors[i]
                reason=window.exclusion(anchor['observation_ns'],anchor['freeze_ns'],teacher_span_ns=plan['teacher_span_ns'])
                if reason:
                    excluded.append(dict(index=i,anchor_id=ds.anchor_ids[i],reason=reason));reasons[reason]+=1;continue
                eligible.append(i);ri,j=ds._index[i];speed=float(ds._runs[ri]['inputs']['ego'][j,-1,0])
                if (anchor['observation_ns']<=window.ready_sim_ns+plan['launch_window_ns']
                        and -.03<=speed<=plan['launch_speed_max_mps'] and ds.input_valid[i] and ds.xy_mask[i].all()):
                    launch.append(i);run_launch.append(i)
            proof['runs'].append(dict(split=split,run_id=rid,**audit,total=len(indices),excluded=dict(reasons),
                launch_anchors=len(run_launch),launch_anchor_ids=[ds.anchor_ids[i] for i in run_launch]))
            print('AUDITED_READY',split,rid,len(run_launch),dict(reasons),flush=True)
        proof[split]=dict(eligible_indices=sorted(eligible),launch_indices=sorted(launch),excluded=excluded)
    val=datasets['validation'];active=proof['validation']['eligible_indices'];launch=set(proof['validation']['launch_indices'])
    selected=[i for i in active if val.input_valid[i] and val.xy_mask[i].all()]
    proof['validation']['selected_indices']=selected
    proof['validation']['unsupported_active_anchors']=len(active)-len(selected)
    proof['validation']['stages']={
        'nominal':[i for i in selected if not val.run_ids[i].startswith('codex-time-recovery-') and i not in launch],
        'recovery':[i for i in selected if val.run_ids[i].startswith('codex-time-recovery-')]}
    samplers,old=sampler_context(root,plan,proof,datasets['train'])
    proof['samplers']={name:s.audit for name,s in samplers.items()}
    controller=read(repo/plan['controller']);proof['controller']=controller
    proof['controller_sha256']=_sha(repo/plan['controller'])
    auxiliary=read(root/plan['parent_experiment']/'recovery_geometry_targets.json')
    objective=make_objective(dict(cache=identity,controller=controller),old,auxiliary)
    # Geometry identity includes controller metadata: compare numerical policies,
    # while the frozen old objective itself remains byte-for-byte identical.
    old_identity=read(root/plan['parent_experiment']/'geometry_identity.json')
    old_controller=old_identity['controller']
    for name in ['geometry','speed_policy','lookahead_policy','vehicle_model_policy']:
        if controller[name]!=old_controller[name]:raise ValueError('controller numerical contract changed')
    objective=make_objective(dict(cache=identity,controller=old_controller),old,auxiliary)
    if objective.identity!=old_identity:raise ValueError('recovery objective changed')
    proof['auxiliary_identity']=old_identity
    proof['auxiliary_targets_sha256']=_sha(root/plan['parent_experiment']/'recovery_geometry_targets.json')
    write(out/'proof.json',proof)
    prepare_launch_cases(root,plan,proof,val,out)
    write(out/'proof.json',proof)
    print('PREPARATION_COMPLETE',json.dumps({k:v for k,v in proof['samplers'].items()}),flush=True)


def prepare_launch_cases(root: Path,plan: dict[str,Any],proof: dict[str,Any],val: Any,out: Path) -> None:
    prior=root/'runs/time_launch_regression_20260916/baseline'
    metadata=[r for r in read(prior/'anchors.json') if 0<=r['after_teacher_arm_s']<=.5]
    samples=[];baseline_rows=[];pose_sources={}
    for rid in sorted({r['run_id'] for r in metadata}):
        raw=root/'raw/time_recovery_speed_20260914'/rid
        transfer=read(raw/'transfer_manifest.json')
        if isinstance(transfer,list):transfer={r['path']:r for r in transfer}
        bag=raw/'bag/bag_0.db3'
        if _sha(bag)!=transfer['bag/bag_0.db3']['sha256']:raise ValueError('baseline bag changed')
        proof['raw_sources'][str(bag)]=transfer['bag/bag_0.db3']['sha256']
        result=read(raw/'result.json')
        if result['status']!='COMPLETE_LAP' or result['fixed_target_mps']!=5/3.6:
            raise ValueError('complete fixed-five baseline required')
        if result['simulator_assets']!={'AWSIM_Data/level1':proof['controller']['geometry']['scene_sha256'],**proof['controller']['steering_asset_sha256']}:
            raise ValueError('baseline simulator assets differ')
        view=prior/rid
        if not (view/'bag/bag_0.db3').samefile(bag):raise ValueError('baseline reader view is not original raw')
        index=read_time_sqlite_run(view,rid);windows=EventWindows(index.events)
        events={e.sequence:e for e in index.events};bounds={e.epoch_id:(e.first_sim_stamp_ns,e.last_sim_stamp_ns) for e in index.epochs}
        @lru_cache(maxsize=64)
        def loaded(sequence: int):return load_event(view,events[sequence])
        for record in [r for r in metadata if r['run_id']==rid]:
            camera_id=record['history_row_ids']['camera'][-1][0];anchor=events[camera_id]
            if anchor.capture_ns!=record['observation_ns']:raise ValueError('baseline camera timestamp changed')
            selected=windows.at(anchor)
            _,audit=audit_anchor(selected,anchor,config=TimeDatasetConfig(),bounds=bounds[anchor.epoch],
                freeze_ns=record['freeze_ns'],intervention_ns=None)
            if (audit['history_row_ids']!=record['history_row_ids']
                    or audit['observation_pose_row_ids']!=record['pose_row_ids']):
                raise ValueError('baseline causal input or observation-pose history changed')
            needed={i for role in ['camera','lidar'] for slot in record['history_row_ids'][role] for i in slot}
            real=tuple(loaded(e.sequence) if e.role in ['camera','lidar'] else e for e in selected
                if e.role not in ['camera','lidar'] or e.sequence in needed)
            sample=assemble_time_sample(real,next(e for e in real if e.sequence==camera_id),config=TimeDatasetConfig(),
                epoch_start_ns=bounds[anchor.epoch][0],epoch_end_ns=bounds[anchor.epoch][1],freeze_ns=record['freeze_ns'])
            if sample.inputs is None or sample.teacher is None or not sample.teacher.xy_mask.all():
                raise ValueError('baseline launch sample lost support')
            if sample.anchor_id!=record['anchor_id']:raise ValueError('baseline anchor identity changed')
            assert sample.inputs.targets is None
            np.testing.assert_allclose(sample.teacher.xy_m[-1],record['predicted_endpoints']['teacher'],rtol=0,atol=0)
            baseline_rows.append(dict(record,sample_index=len(samples)))
            samples.append(sample)
        pose_sources[rid]=motion(view);loaded.cache_clear()
        del index,windows,events
    torch.save(samples,out/'baseline_launch_samples.pt')
    proof['baseline_samples_sha256']=_sha(out/'baseline_launch_samples.pt')
    proof['baseline_samples']=len(samples)
    original_metadata=read(prior/'anchors.json')
    original_indices={r['anchor_id']:i for i,r in enumerate(original_metadata)}
    old_prediction_file=prior/'old_predictions.npy'
    old_proof=read(root/'runs/time_launch_regression_20260916/supplement/provenance.json')
    if _sha(old_prediction_file)!=old_proof['sources'][str(old_prediction_file)]['sha256']:
        raise ValueError('prior baseline prediction evidence changed')
    frozen_prediction=np.load(old_prediction_file,allow_pickle=False)[[original_indices[r['anchor_id']] for r in baseline_rows]]
    np.save(out/'initial_baseline_predictions.npy',frozen_prediction,allow_pickle=False)
    proof['initial_baseline_predictions_sha256']=_sha(out/'initial_baseline_predictions.npy')
    records=[];missing=Counter()
    for index in proof['validation']['launch_indices']:
        row=val._anchors[index];rid=row['run_id']
        if rid not in pose_sources:
            pose_sources[rid]=motion(root/'datasets/raw/time_teacher_20laps_20260911/runs'/rid)
        poses,by_id,ts,vs=pose_sources[rid]
        obs=replay_observation_pose([by_id[k] for k in row['observation_pose_row_ids']],
            observation_ns=row['observation_ns'],freeze_receipt_ns=row['freeze_ns'])
        ri,j=val._index[index];v=float(val._runs[ri]['inputs']['ego'][j,-1,0])
        records.append(dict(origin='nominal_validation',sample_index=index,anchor_id=row['anchor_id'],run_id=rid,
            observation=obs,speed=v,teacher=val.targets[index],poses=pose_sources[rid]))
    for record,sample in zip(baseline_rows,samples,strict=True):
        records.append(dict(origin='awsim_calibration',sample_index=record['sample_index'],anchor_id=record['anchor_id'],
            run_id=record['run_id'],observation=TimedBodyPose(**record['observation_pose']),
            speed=float(sample.inputs.ego[0,-1,0]),teacher=sample.teacher.xy_m,poses=pose_sources[record['run_id']]))
    cases=[]
    for row in records:
        obs=row['observation'];poses,_,ts,vs=row['poses']
        for age in plan['launch_ages_s']:
            try:current=obs if age==0 else poses.at(obs.stamp_ns+round(age*1e9))
            except ValueError as exc:
                if str(exc)!='AMBIGUOUS_POSE_STAMP':raise
                missing[f"{row['run_id']}:{age}"]+=1;continue
            speed=row['speed'] if age==0 else speed_at(ts,vs,current.stamp_ns)
            if age:
                error=np.linalg.norm(world_points(row['teacher'],obs)[round(age*10)-1]-[current.x_m,current.y_m])
                if error>1e-5:raise ValueError('future pose does not match measured teacher')
            teacher=pp_probe(row['teacher'],obs,current,speed,proof['controller'])
            cases.append(dict(case_id=f"{row['anchor_id']}@{age}",origin=row['origin'],sample_index=row['sample_index'],
                run_id=row['run_id'],age_s=age,observation_pose=asdict(obs),current_pose=asdict(current),
                speed_mps=speed,teacher=teacher))
    write(out/'launch_cases.json',cases)
    write(out/'baseline_launch_metadata.json',baseline_rows)
    proof['launch_cases_sha256']=_sha(out/'launch_cases.json')
    proof['launch_missing_future_pose']=dict(missing)
    proof['population_sha256']=content_sha256(dict(validation_anchor_ids=[val.anchor_ids[i] for i in proof['validation']['selected_indices']],
        stages=proof['validation']['stages'],launch_cases=cases,baseline_samples_sha256=proof['baseline_samples_sha256']))


def context(root: Path,repo: Path,plan: dict[str,Any]) -> tuple[dict[str,Any],Any,Any]:
    head=source_guard(repo,plan);prep=root/plan['output']/'preparation';proof=read(prep/'proof.json')
    if proof['plan']!=plan or proof['source_commit']!=head:raise ValueError('frozen experiment/source changed')
    identity=verify_time_training_cache(root/plan['cache'])
    if identity['manifest_sha256']!=plan['cache_sha256'] or _sha(root/plan['initialization'])!=plan['initialization_sha256']:
        raise ValueError('cache or initial checkpoint changed')
    for name,key in [('launch_cases.json','launch_cases_sha256'),('baseline_launch_samples.pt','baseline_samples_sha256'),
                     ('initial_baseline_predictions.npy','initial_baseline_predictions_sha256')]:
        if _sha(prep/name)!=proof[key]:raise ValueError('frozen launch evaluation changed')
    return proof,TimeTrainingCacheDataset(root/plan['cache'],'train',verify_hashes=False),TimeTrainingCacheDataset(root/plan['cache'],'validation',verify_hashes=False)


def train(root: Path,repo: Path,plan: dict[str,Any],arm: str,resume: bool) -> None:
    proof,full,val=context(root,repo,plan);samplers,old=sampler_context(root,plan,proof,full);sampler=samplers[arm]
    if sampler.audit!=proof['samplers'][arm]:raise ValueError('presentation order changed')
    parent=root/plan['parent_experiment'];rows=read(parent/'recovery_geometry_targets.json')
    if _sha(parent/'recovery_geometry_targets.json')!=proof['auxiliary_targets_sha256']:raise ValueError('auxiliary labels changed')
    objective=make_objective(dict(cache=full.identity,controller=proof['auxiliary_identity']['controller']),old,rows)
    if objective.identity!=proof['auxiliary_identity']:raise ValueError('loss changed')
    source=root/plan['initialization'];payload=torch.load(source,map_location='cpu',weights_only=False)
    cfg=TimeModelConfig.from_dict(payload['config']);initial_identity=TimeCheckpointIdentity(**payload['identity'])
    if cfg.use_command_history or cfg.dataset_config()!=full.config:raise ValueError('input contract changed')
    selected=proof['validation']['selected_indices'];split=full.split_manifest
    teacher=dict(format='ready_nominal_launch_quota_v1',experiment_plan=plan,arm=arm,
        contract=full.identity['contract'],
        preparation_sha256=_sha(root/plan['output']/'preparation/proof.json'),
        cache_manifest_sha256=plan['cache_sha256'],train_anchor_order_sha256=content_sha256(sampler.anchor_ids),
        selection_validation_anchor_ids_sha256=content_sha256([val.anchor_ids[i] for i in selected]),
        auxiliary_identity=objective.identity,source_git_commit=proof['source_commit'],sampler=sampler.audit)
    teacher['manifest_sha256']=content_sha256(teacher)
    identity=TimeCheckpointIdentity(split['manifest_sha256'],teacher['manifest_sha256'],
        content_sha256([r for r in split['runs'] if r['split']=='train']),arm,proof['source_commit'])
    output=root/plan['output']/arm
    result=run_training_arm(sampler,Subset(val,selected),train_run_ids=sampler.run_ids,
        validation_run_ids=[val.run_ids[i] for i in selected],split_manifest=split,teacher_manifest=teacher,
        identity=identity,config=cfg,plan=CorpusTrainingPlan(**plan['training']),output=output,
        resume=resume,initialization=source,initialization_sha256=plan['initialization_sha256'],
        initialization_identity=initial_identity,retain_epoch_checkpoints=True,recovery_objective=objective)
    if not result['reload_predictions_exact'] or result['optimizer_steps']!=plan['optimizer_steps_per_arm']:
        raise ValueError('training budget or reload verification failed')
    write(output/'verification.json',dict(status='PASS',source_commit=proof['source_commit'],
        optimizer_steps=result['optimizer_steps'],anchors_visited=result['anchors_visited'],
        initial_checkpoint_sha256=plan['initialization_sha256'],
        checkpoints={f'epoch_{i:02d}.pt':_sha(output/f'epoch_{i:02d}.pt') for i in range(1,plan['training']['epochs']+1)},
        runtime_promotion_requires_separate_stage_gate=True))


def stage_errors(prediction: np.ndarray,val: Any,selected: list[int],stages: dict[str,list[int]]) -> dict[str,Any]:
    if prediction.shape!=(len(selected),30,2):raise ValueError('expected complete validation [N,30,2] metre predictions')
    lookup={i:j for j,i in enumerate(selected)};result={}
    for name,indices in stages.items():
        if not indices:raise ValueError('empty stage')
        p=prediction[[lookup[i] for i in indices]];truth=val.targets[indices]
        ids=[val.run_ids[i] for i in indices];runs=sorted(set(ids))
        values=[]
        if np.isfinite(p).all():
            error=np.linalg.norm(p.astype(float)-truth,axis=2)
            for rid in runs:
                e=error[np.array(ids)==rid];values.append((float(e.mean()),float(e[:,-1].mean())))
        result[name]=dict(anchors=len(indices),runs=len(runs),
            ade_m=float(np.mean([v[0] for v in values])) if values else None,
            endpoint_3s_m=float(np.mean([v[1] for v in values])) if values else None,
            invalid_predictions=int((~np.isfinite(p).all(axis=(1,2))).sum()))
    return result


def evaluate(root: Path,repo: Path,plan: dict[str,Any],existing_only: bool) -> None:
    proof,_,val=context(root,repo,plan);base=root/plan['output'];prep=base/'preparation'
    out=base/('evaluation_existing' if existing_only else 'evaluation_final');out.mkdir(exist_ok=False)
    cases=read(prep/'launch_cases.json');samples=torch.load(prep/'baseline_launch_samples.pt',weights_only=False)
    selected=proof['validation']['selected_indices'];lookup={i:j for j,i in enumerate(selected)}
    candidates={'initial':root/plan['initialization']}
    if existing_only:candidates['previous_epoch2']=root/plan['parent_experiment']/'training_serial/epoch_02.pt'
    else:
        for arm in ['protocol_control','launch_balanced']:
            check=read(base/arm/'verification.json')
            if check['status']!='PASS':raise ValueError('completed arm required')
            for i in range(1,plan['training']['epochs']+1):
                name=f'epoch_{i:02d}.pt';path=base/arm/name
                if _sha(path)!=check['checkpoints'][name]:raise ValueError('retained checkpoint changed')
                candidates[f'{arm}_epoch{i}']=path
    reports=[]
    for name,path in candidates.items():
        payload=torch.load(path,map_location='cpu',weights_only=False);cfg=TimeModelConfig.from_dict(payload['config'])
        model=build_time_model(cfg).to('cuda').eval()
        load_time_checkpoint(path,config=cfg,identity=TimeCheckpointIdentity(**payload['identity']),model=model,mode='finetune')
        # Retained predictions are valid only for this frozen validation order.
        if name.startswith(('protocol_control_','launch_balanced_')):
            if payload['teacher_manifest']['selection_validation_anchor_ids_sha256']!=content_sha256([val.anchor_ids[i] for i in selected]):
                raise ValueError('saved validation order changed')
            prediction=np.load(path.parent/f"validation_epoch_{payload['epoch']:02d}.npy",allow_pickle=False)
        else:
            _,values=evaluate_time_batched(model,Subset(val,selected),run_ids=[val.run_ids[i] for i in selected],
                split_manifest=val.split_manifest,batch_size=32,workers=0,precision='float32')
            prediction=values.numpy()
        sensor_predictions=[]
        with torch.inference_mode():
            for sample in samples:
                batch=sample.inputs
                if batch is None or batch.targets is not None:raise ValueError('teacher must not enter inference inputs')
                moved=replace(batch,**{f.name:getattr(batch,f.name).to('cuda') for f in fields(batch)
                    if isinstance(getattr(batch,f.name),torch.Tensor)})
                sensor_predictions.append(model(moved).float().cpu().numpy()[0])
        sensor_predictions=np.stack(sensor_predictions)
        if sensor_predictions.shape!=(len(samples),30,2):raise ValueError('baseline prediction shape changed')
        if name=='initial':
            np.testing.assert_array_equal(sensor_predictions,np.load(prep/'initial_baseline_predictions.npy',allow_pickle=False))
        launch=[]
        for row in cases:
            xy=(prediction[lookup[row['sample_index']]] if row['origin']=='nominal_validation'
                else sensor_predictions[row['sample_index']])
            p=pp_probe(xy,TimedBodyPose(**row['observation_pose']),TimedBodyPose(**row['current_pose']),row['speed_mps'],proof['controller'])
            launch.append(dict(case_id=row['case_id'],run_id=row['run_id'],origin=row['origin'],age_s=row['age_s'],
                teacher_accepted=row['teacher']['accepted'],teacher_steer_rad=row['teacher'].get('steer_rad'),
                accepted=p['accepted'],steer_rad=p.get('steer_rad'),reason=p['reason']))
        report=dict(candidate_id=name,checkpoint=str(path),checkpoint_sha256=_sha(path),
            population_sha256=proof['population_sha256'],xy=stage_errors(prediction,val,selected,proof['validation']['stages']),launch=launch)
        write(out/(name+'.json'),report);np.save(out/(name+'_predictions.npy'),prediction,allow_pickle=False)
        np.save(out/(name+'_baseline_predictions.npy'),sensor_predictions,allow_pickle=False)
        reports.append(report);del model,payload;torch.cuda.empty_cache()
        print('CANDIDATE_EVALUATED',name,json.dumps(dict(xy=report['xy'],launch_accepted=sum(r['accepted'] for r in launch),launch_cases=len(launch))),flush=True)
    selection=select_stage_candidate(reports,baseline_id='initial',policy=StageSelectionPolicy(**plan['selection']))
    chosen=next(r for r in reports if r['candidate_id']==selection['selected_candidate_id'])
    selection.update(checkpoint=chosen['checkpoint'],checkpoint_sha256=chosen['checkpoint_sha256'],
        source_commit=proof['source_commit'],population_sha256=proof['population_sha256'],
        validation_role=plan['validation_role'],test_usage='sealed',new_awsim_runs=0)
    write(out/'selection.json',selection)
    print('SELECTION',json.dumps(selection),flush=True)


def main() -> None:
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command',choices=['prepare','train','evaluate'])
    parser.add_argument('--plan',type=Path,default=Path('configs/time_path_p1/launch_protection_20260916.json'))
    parser.add_argument('--root',type=Path,default=Path('..'))
    parser.add_argument('--arm',choices=['protocol_control','launch_balanced'])
    parser.add_argument('--resume',action='store_true')
    parser.add_argument('--existing-only',action='store_true')
    args=parser.parse_args();root=args.root.resolve();repo=Path(__file__).resolve().parents[1]
    if root.as_posix().startswith('/mnt/') or not torch.cuda.is_available():raise ValueError('native WSL CUDA required')
    torch.set_num_threads(4);torch.backends.cuda.matmul.allow_tf32=False;torch.backends.cudnn.allow_tf32=False
    torch.backends.cudnn.benchmark=False;torch.backends.cudnn.deterministic=True
    soft,hard=resource.getrlimit(resource.RLIMIT_NOFILE);resource.setrlimit(resource.RLIMIT_NOFILE,(max(soft,min(hard,8192)),hard))
    plan=read(args.plan)
    if args.command=='train':
        if args.arm is None:parser.error('--arm is required for train')
        train(root,repo,plan,args.arm,args.resume)
    elif args.command=='prepare':prepare(root,repo,plan)
    else:evaluate(root,repo,plan,args.existing_only)


if __name__=='__main__':main()
