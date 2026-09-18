"""701-window 2x2 diagnostic with identical old presentations in every arm."""
from __future__ import annotations
import argparse
import gc
import json
from pathlib import Path
import random
import resource
import subprocess
import time
import numpy as np
import torch
from torch.utils.data import Subset
from train_time_launch_protection import read, write, sampler_context, stage_errors
from train_time_native_replay import predict_samples, recovery_run_errors, select_retained_native
from compare_time_recovery_objectives import make_objective
from aic_transfuser_lite.control.time_reference_v1 import TimedBodyPose, TimePlan
from aic_transfuser_lite.control.time_trial_v1 import time_trial_control
from aic_transfuser_lite.control.vehicle_motion_v1 import effective_response_length
from aic_transfuser_lite.data.time_native_replay_v1 import NativeReplayDataset
from aic_transfuser_lite.data.time_training_cache_v1 import TimeTrainingCacheDataset, verify_time_training_cache, _sha
from aic_transfuser_lite.data.time_split_v1 import content_sha256, assert_split_membership
from aic_transfuser_lite.evaluation.time_batched_v1 import evaluate_time_batched
from aic_transfuser_lite.evaluation.time_recovery_comparison_v1 import pp_probe
from aic_transfuser_lite.training.native_fit_v1 import NativeGeometryObjective, AddNativeObjective
from aic_transfuser_lite.training.time_config_v1 import TimeModelConfig, build_time_model
from aic_transfuser_lite.training.time_checkpoint_v1 import TimeCheckpointIdentity, load_time_checkpoint, save_time_checkpoint
from aic_transfuser_lite.training.time_corpus_runner_v1 import train_corpus_batch


def comparison_schedule(*, old_count: int, native_count: int, front: list[int],
                        steps: int, focused: bool, seed: int = 42) -> list[list[tuple[str, int]]]:
    """32 old + 8 native per update, with identical old indices across arms.

    Native uniform permutations visit every anchor. Focus replaces four of
    eight native slots with front-cone anchors; old presentations never change.
    Counts are presentations, not independent events or a full corpus epoch.
    """
    if (any(type(n) is not int or n <= 0 for n in (old_count, native_count, steps))
            or type(focused) is not bool or not front or len(set(front)) != len(front)
            or any(type(i) is not int or not 0 <= i < native_count for i in front)):
        raise ValueError('positive counts and unique in-range front indices required')
    def stream(count: int, rng_seed: int):
        rng = np.random.default_rng(rng_seed)
        while True:
            yield from map(int, rng.permutation(count))
    old = stream(old_count, seed)
    uniform = stream(native_count, seed + 1)
    approach = stream(len(front), seed + 2)
    return [[('old', next(old)) for _ in range(32)] +
            [('native', front[next(approach)] if focused and j < 4 else next(uniform))
             for j in range(8)] for _ in range(steps)]


def main() -> None:
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--output',default='runs/native701_retention_20260919')
    args=parser.parse_args();root=args.root.resolve();repo=Path(__file__).resolve().parents[1]
    if str(root).startswith('/mnt/') or not torch.cuda.is_available():raise ValueError('native WSL CUDA required')
    if subprocess.check_output(['git','status','--porcelain'],cwd=repo).strip():raise ValueError('clean synchronized source required')
    head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip()
    torch.set_num_threads(4);torch.backends.cuda.matmul.allow_tf32=False
    torch.backends.cudnn.allow_tf32=False;torch.backends.cudnn.benchmark=False;torch.backends.cudnn.deterministic=True
    soft,hard=resource.getrlimit(resource.RLIMIT_NOFILE);resource.setrlimit(resource.RLIMIT_NOFILE,(max(soft,min(hard,8192)),hard))
    out=root/args.output;out.mkdir(parents=True,exist_ok=False)
    base=root/'runs/time_native701_prepare_20260919';prep=base/'preparation'
    proof=read(prep/'proof.json');plan=read(repo/'configs/time_path_p1/native701_prepare_20260919.json')
    previous=read(repo/plan['previous_plan']);old_prep=root/plan['previous_experiment']/'preparation';old_proof=read(old_prep/'proof.json')
    for name,digest in proof['previous_artifacts'].items():
        if _sha(root/name)!=digest:raise ValueError('old evidence changed: '+name)
    if _sha(prep/'split_manifest.json')!=proof['split_file_sha256']:raise ValueError('split changed')
    cache=verify_time_training_cache(root/previous['cache'])
    if cache['manifest_sha256']!=previous['cache_sha256']:raise ValueError('cache changed')
    full=TimeTrainingCacheDataset(root/previous['cache'],'train',verify_hashes=False)
    val=TimeTrainingCacheDataset(root/previous['cache'],'validation',verify_hashes=False)
    samplers,old=sampler_context(root,previous,old_proof,full);replay=samplers['launch_balanced']
    native=NativeReplayDataset(prep,manifest_sha256=proof['native_manifest_sha256']);split=read(prep/'split_manifest.json')
    if samplers['launch_balanced'].audit!=proof['previous_sampler']:raise ValueError('old replay changed')
    source=root/plan['initialization'];initial_sha=plan['initialization_sha256']
    if _sha(source)!=initial_sha:raise ValueError('initializer changed')
    diagnostic=root/'runs/avoidance_diagnosis_20260918/training_windows.jsonl'
    if _sha(diagnostic)!='25673a909c39d7c77b6385a8d127a9bb7efcc63c0f2abc00dc8ab1684925771c':raise ValueError('front membership changed')
    geometry={r['anchor_id']:r['cone_body_xy_m'] for r in map(json.loads,diagnostic.read_text().splitlines())}
    collection=root/'runs/mppi_v45_pc10_20260918'
    for folder in ('front_validation_v1','collect10_validation_v1'):
        for path in (collection/folder).glob('*-geometry.json'):
            for r in read(path):
                if r['anchor_id'] in geometry:raise ValueError('duplicate geometry anchor')
                geometry[r['anchor_id']]=r['object_body_xy_m']
    rows=[dict(anchor_id=a,use=native.uses[i],cone_body_xy_m=geometry[a],
               ego_mps=float(native[i].inputs.ego[0,-1,0])) for i,a in enumerate(native.anchor_ids)]
    front=[i for i,r in enumerate(rows) if r['use']=='static_cone_xy_speed' and r['cone_body_xy_m'][0]>0]
    if len(native)!=701 or not front:raise ValueError('population changed')
    targets=read(root/previous['parent_experiment']/'recovery_geometry_targets.json')
    recovery=make_objective(dict(cache=dict(split_manifest=split),controller=old_proof['auxiliary_identity']['controller']),old,targets)
    if any(recovery.identity[k]!=v for k,v in old_proof['auxiliary_identity'].items() if k!='split_manifest_sha256'):raise ValueError('old recovery loss changed')
    config=read(repo/'configs/control/time_path_dev.json')
    def control(i: int,xy: np.ndarray) -> dict:
        sample=native[i];pose=TimedBodyPose(sample.observation_ns,'sim','0','diagnostic_local','base_link',0.,0.,0.)
        return time_trial_control(TimePlan('fit',pose,xy),pose,speed_mps=rows[i]['ego_mps'],
            rear_axle_offset_m=(config['geometry']['rear_axle_forward_in_base_link_m'],0.),
            speed_policy=config['speed_policy'],lookahead_policy=config['lookahead_policy'],
            vehicle_model_policy=config['vehicle_model_policy'],speed_cap_mps=10/3.6,corner_max_speed_mps=10/3.6)
    teacher_controls={}; teacher_rejected=[]
    for i in front:
        try:teacher_controls[i]=control(i,native[i].teacher.xy_m)
        except ValueError as exc:teacher_rejected.append(dict(index=i,reason=str(exc)))
    geometry_rows=[dict(anchor_id=native[i].anchor_id,run_id=native[i].run,
        horizon_s=teacher_controls[i]['lookahead_selection']['observation_horizon_s'],
        response_length_m=effective_response_length(rows[i]['ego_mps'],config['vehicle_model_policy'])) for i in teacher_controls]
    native_objective=NativeGeometryObjective(geometry_rows,split_manifest=split)
    write(out/'native_geometry_targets.json',geometry_rows)
    experiment=dict(source_git_commit=head,steps_per_arm=512,batch_size=40,seed=42,learning_rate=1e-6,
        weight_decay=1e-4,precision='float32',initialization_sha256=initial_sha,
        native_manifest_sha256=proof['native_manifest_sha256'],front_indices=front,
        old_recovery_objective_unchanged=True,old_presentations_per_arm=16384,native_presentations_per_arm=4096,
        teacher_pp_rejected=teacher_rejected,automatic_runtime_promotion=False,test_usage='sealed',
        scope='training-fit diagnostic; validation retention only; not heldout avoidance or closed-loop proof')
    write(out/'resolved_plan.json',experiment)
    selected=old_proof['validation']['selected_indices'];lookup={i:j for j,i in enumerate(selected)}
    launch_samples=torch.load(old_prep/'baseline_launch_samples.pt',weights_only=False);cases=read(old_prep/'launch_cases.json')
    native.targets=np.stack([native[i].teacher.xy_m for i in range(len(native))])
    stages={'all':list(range(len(native)))}
    stages.update({use:[i for i,v in enumerate(native.uses) if v==use] for use in sorted(set(native.uses))})
    reports=[]
    for name,focused,geometry in [('initial',False,False),('control',False,False),('focused',True,False),('geometry',False,True),('focused_geometry',True,True)]:
        torch.manual_seed(42);np.random.seed(42);random.seed(42)
        payload=torch.load(source,map_location='cpu',weights_only=False);cfg=TimeModelConfig.from_dict(payload['config'])
        model=build_time_model(cfg).cuda();load_time_checkpoint(source,config=cfg,identity=TimeCheckpointIdentity(**payload['identity']),model=model,mode='finetune')
        path=source;started=time.monotonic()
        if name!='initial':
            schedule=comparison_schedule(old_count=len(replay),native_count=len(native),front=front,steps=512,focused=focused)
            write(out/(name+'_schedule.json'),schedule)
            objective=AddNativeObjective(recovery,native_objective if geometry else None)
            optimizer=torch.optim.AdamW(model.parameters(),lr=1e-6,weight_decay=1e-4);model.train()
            with (out/(name+'_training.jsonl')).open('x') as stream:
                for step,batch in enumerate(schedule,1):
                    samples=[replay[i] if source_name=='old' else native[i] for source_name,i in batch]
                    assert_split_membership(split,[s.run for s in samples],split='train')
                    result=train_corpus_batch(model,samples,optimizer,precision='float32',max_grad_norm=1.,recovery_objective=objective)
                    if not result['updated']:raise ValueError('fixed update budget lost support')
                    stream.write(json.dumps(dict(step=step,**result))+'\n')
                    if step%32==0:stream.flush();print('TRAIN',name,step,round(time.monotonic()-started,1),flush=True)
            teacher=dict(contract=payload['teacher_manifest']['contract'],experiment=experiment,arm=name,
                schedule_sha256=content_sha256(schedule),native_geometry=geometry)
            teacher['manifest_sha256']=content_sha256(teacher)
            identity=TimeCheckpointIdentity(split['manifest_sha256'],teacher['manifest_sha256'],
                content_sha256([r for r in split['runs'] if r['split']=='train']),args.output+'/'+name,head)
            path=out/(name+'.pt')
            save_time_checkpoint(path,config=cfg,identity=identity,model=model,optimizer=optimizer,scheduler=None,
                epoch=1,global_step=512,split_manifest=split,teacher_manifest=teacher)
            model.eval();before=predict_samples(model,[native[i] for i in front[:8]])
            load_time_checkpoint(path,config=cfg,identity=identity,model=model,mode='finetune')
            np.testing.assert_array_equal(before,predict_samples(model,[native[i] for i in front[:8]]))
            del optimizer
        model.eval()
        print('EVALUATING',name,flush=True)
        _,values=evaluate_time_batched(model,Subset(val,selected),run_ids=[val.run_ids[i] for i in selected],
            split_manifest=split,batch_size=32,workers=0,precision='float32')
        prediction=values.numpy();native_predictions=predict_samples(model,native);lp=predict_samples(model,launch_samples,batch_size=1)
        launch=[]
        for row in cases:
            xy=prediction[lookup[row['sample_index']]] if row['origin']=='nominal_validation' else lp[row['sample_index']]
            p=pp_probe(xy,TimedBodyPose(**row['observation_pose']),TimedBodyPose(**row['current_pose']),row['speed_mps'],old_proof['controller'])
            launch.append(dict(case_id=row['case_id'],run_id=row['run_id'],origin=row['origin'],age_s=row['age_s'],
                teacher_accepted=row['teacher']['accepted'],teacher_steer_rad=row['teacher'].get('steer_rad'),accepted=p['accepted'],steer_rad=p.get('steer_rad'),reason=p['reason']))
        front_cases=[]
        for i in teacher_controls:
            try:p=control(i,native_predictions[i]);front_cases.append(dict(index=i,accepted=True,teacher_rad=teacher_controls[i]['steer_rad'],prediction_rad=p['steer_rad']))
            except ValueError as exc:front_cases.append(dict(index=i,accepted=False,reason=str(exc)))
        curved=[r for r in front_cases if r['accepted'] and abs(r['teacher_rad'])>.02]
        fit=dict(anchors=len(front),teacher_pp_rejected=teacher_rejected,accepted=sum(r['accepted'] for r in front_cases),curved=len(curved),
            opposite_sign=sum(r['teacher_rad']*r['prediction_rad']<0 for r in curved),
            below_half=sum(abs(r['prediction_rad'])<.5*abs(r['teacher_rad']) for r in curved),
            ade_m=float(np.linalg.norm(native_predictions[front]-native.targets[front],axis=2).mean()),cases=front_cases)
        report=dict(candidate_id=name,checkpoint=str(path),checkpoint_sha256=_sha(path),population_sha256=old_proof['population_sha256'],
            launch=launch,xy=stage_errors(prediction,val,selected,old_proof['validation']['stages']),
            recovery_by_run=recovery_run_errors(prediction,val,selected,old_proof['validation']['stages']['recovery']),
            native_fit=stage_errors(native_predictions,native,list(range(len(native))),stages),front_fit=fit,wall_s=time.monotonic()-started)
        write(out/(name+'.json'),report);np.save(out/(name+'_native_predictions.npy'),native_predictions);np.save(out/(name+'_validation_predictions.npy'),prediction)
        reports.append(report);print('EVALUATED',name,json.dumps(dict(xy=report['xy'],front_fit={k:v for k,v in fit.items() if k!='cases'})),flush=True)
        del model,payload;gc.collect();torch.cuda.empty_cache()
    write(out/'selection_vs_initializer.json',select_retained_native(reports,plan))
    write(out/'selection_vs_deployed.json',select_retained_native(reports,plan))
    print('COMPARISON_COMPLETE',flush=True)


if __name__=='__main__':main()
