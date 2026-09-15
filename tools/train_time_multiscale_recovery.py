"""Audit, append and retrain on observed 12/20/40/60 cm recovery collections."""
from __future__ import annotations

import argparse
from copy import deepcopy
import json
from pathlib import Path
import resource
import subprocess
from typing import Any

import numpy as np
import torch
from torch.utils.data import Subset

from analyze_time_recovery_fit import outward_ids
from compare_time_recovery_objectives import geometry_targets, make_objective
from compare_time_training_methods import pp_rows
from train_time_recovery_update import context as data_context, read_json, write
from aic_transfuser_lite.data.time_multiscale_recovery_v1 import audit_collections
from aic_transfuser_lite.data.time_outward_balanced_v1 import OutwardBalancedMixDataset
from aic_transfuser_lite.data.time_recovery_append_v1 import prepare_append_cache
from aic_transfuser_lite.data.time_split_v1 import content_sha256
from aic_transfuser_lite.data.time_training_cache_v1 import _sha
from aic_transfuser_lite.evaluation.time_batched_v1 import evaluate_time_batched
from aic_transfuser_lite.evaluation.time_metrics_v1 import time_horizon_metrics
from aic_transfuser_lite.evaluation.time_method_selection_v1 import pp_agreement_score
from aic_transfuser_lite.evaluation.time_recovery_comparison_v1 import component_errors, summarize_pp
from aic_transfuser_lite.training.time_checkpoint_v1 import TimeCheckpointIdentity, load_time_checkpoint
from aic_transfuser_lite.training.time_config_v1 import TimeModelConfig, build_time_model
from aic_transfuser_lite.training.time_corpus_runner_v1 import CorpusTrainingPlan, run_training_arm
from aic_transfuser_lite.training.time_recovery_geometry_v1 import RecoveryGeometryPlan


def audit(root: Path, repo: Path, sources: dict[str, Any]) -> None:
    out = root/sources['audit_output']
    out.mkdir(parents=True, exist_ok=False)
    proof, additions = audit_collections(root, sources['collections'], out)
    write(out/'source_verification.json', proof)
    plan = deepcopy(read_json(repo/sources['previous_data_plan']))
    previous = read_json(root/plan['output']/'matched_budget_verification.json')
    plan.update(experiment='observed_multiscale_recovery_20260916',
        parent_cache=plan['cache'], parent_cache_sha256=sources['parent_cache_sha256'],
        cache=sources['cache'], output=sources['output'], audit_output=sources['audit_output'],
        additions=additions, collections=sources['collections'],
        previous_data_plan=sources['previous_data_plan'],
        reference_presentation_order_sha256=previous['train_anchor_order_sha256'],
        balanced_target_fraction=0.25, balanced_target_rule='prior_outward_plus_first_second_of_each_new_accepted_event',
        geometry=read_json(repo/'configs/time_path_p1/recovery_objective_comparison_20260915.json')['geometry'],
        comparison_model=sources['comparison_model'], comparison_model_sha256=sources['comparison_model_sha256'],
        source_guard_commit=sources['source_guard_commit'],
        comparison='same_initialization_optimizer_update_budget_and_selection; expanded_recovery_pool_and_early_state_sampling')
    for spec in plan['additions']:
        spec['proof_sha256'] = _sha(out/'source_verification.json')
    plan['expected']['unique_train'] += proof['totals']['train']['anchors']
    plan['expected']['unique_recovery_train'] += proof['totals']['train']['anchors']
    plan['expected']['validation_total'] += proof['totals']['validation']['anchors']
    write(out/'resolved_plan.json', plan)
    print(json.dumps(dict(status='SOURCES_AUDITED', additions=proof['totals'], expected=plan['expected'])), flush=True)


def context(root: Path, repo: Path, plan: dict[str, Any]) -> dict[str, Any]:
    protected = ['src/aic_transfuser_lite/models', 'src/aic_transfuser_lite/training',
        'src/aic_transfuser_lite/contracts', 'src/aic_transfuser_lite/evaluation',
        'src/aic_transfuser_lite/data/time_training_cache_v1.py',
        'src/aic_transfuser_lite/data/time_dataset_v1.py', 'src/aic_transfuser_lite/data/time_teacher_v1.py']
    if subprocess.check_output(['git','diff',plan['source_guard_commit'],'HEAD','--',*protected], cwd=repo):
        raise ValueError('model, objective, trainer or evaluation implementation changed')
    c = data_context(root, plan)
    old_targets, old_hashes = outward_ids(root, repo, read_json(repo/plan['previous_data_plan']))
    source = read_json(root/plan['audit_output']/'source_verification.json')
    for spec in plan['additions']:
        if _sha(root/spec['analysis']/spec['proof']) != spec['proof_sha256']:
            raise ValueError('audited source proof changed')
    targets = old_targets & set(c['full'].anchor_ids)
    targets.update(a for r in source['runs'] if r['split']=='train' for a in r['early_train_anchor_ids'])
    recovery_ids = sorted(r for r in set(c['full'].run_ids) if r.startswith('codex-time-recovery-'))
    sampler = OutwardBalancedMixDataset(c['train'], recovery_ids, sorted(targets),
        target_fraction=plan['balanced_target_fraction'], seed=plan['training']['seed'])
    controller = {**read_json(repo/plan['offline_controller_config']), 'lookahead_policy':'stopping_preview_extended_v1'}
    if _sha(root/plan['comparison_model']/'best.pt') != plan['comparison_model_sha256']:
        raise ValueError('comparison checkpoint changed')
    proof = dict(data=c['data_proof'], source_verification_sha256=_sha(root/plan['audit_output']/'source_verification.json'),
        cache_sha256=c['cache']['manifest_sha256'], sampler=sampler.audit,
        sampler_order_sha256=content_sha256(sampler.anchor_ids), prior_collection_hashes=old_hashes,
        target_anchor_ids=sorted(targets), geometry=plan['geometry'], controller=controller,
        sealed_test_read=False)
    return dict(**c, sampler=sampler, proof=proof, controller=controller)


def prepare(root: Path, repo: Path, plan: dict[str, Any]) -> None:
    prepare_append_cache(root, root/plan['cache'], plan)
    c = context(root, repo, plan)
    out = root/plan['audit_output']
    geometry = {**plan['geometry'], 'rear_axle_offset_m':tuple(plan['geometry']['rear_axle_offset_m'])}
    rows = geometry_targets(c, RecoveryGeometryPlan(**geometry))
    objective = make_objective(c, plan, rows)
    write(out/'recovery_geometry_targets.json', rows)
    write(out/'geometry_identity.json', objective.identity)
    write(out/'data_and_budget_verification.json', c['proof'])
    print(json.dumps(dict(status='PREPARED_VERIFIED', cache_sha256=c['cache']['manifest_sha256'],
        target_anchors=len(c['proof']['target_anchor_ids']), expected=plan['expected'])), flush=True)


def train(root: Path, repo: Path, plan: dict[str, Any], *, resume: bool) -> None:
    if not torch.cuda.is_available():
        raise RuntimeError('native WSL CUDA required')
    c = context(root, repo, plan)
    out = root/plan['audit_output']
    saved = read_json(out/'data_and_budget_verification.json')
    if any(saved.get(k) != json.loads(json.dumps(v)) for k,v in c['proof'].items()):
        raise ValueError('prepared experiment drift')
    objective = make_objective(c, plan, read_json(out/'recovery_geometry_targets.json'))
    if objective.identity != read_json(out/'geometry_identity.json'):
        raise ValueError('auxiliary teacher identity changed')
    source = root/plan['initialization']
    payload = torch.load(source, map_location='cpu', weights_only=False)
    config = TimeModelConfig.from_dict(payload['config'])
    initial_identity = TimeCheckpointIdentity(**payload['identity'])
    if config.use_command_history or config.dataset_config() != c['full'].config:
        raise ValueError('model input contract changed')
    split = c['cache']['split_manifest']
    selected, val = c['selection'], c['validation']
    selected_ids = [val.run_ids[i] for i in selected]
    baseline = build_time_model(config).to('cuda')
    load_time_checkpoint(source, config=config, identity=initial_identity, model=baseline, mode='finetune')
    metrics, _ = evaluate_time_batched(baseline, Subset(val, selected), run_ids=selected_ids,
        split_manifest=split, batch_size=32, workers=4, precision='float32')
    if metrics != read_json(root/plan['comparison_model']/'initial_validation.json'):
        raise ValueError('same-initial-weight validation did not replay')
    del baseline, payload
    torch.cuda.empty_cache()
    head = subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip()
    teacher = dict(format='observed_multiscale_recovery_training_v1', plan=plan,
        cache_sha256=c['cache']['manifest_sha256'], auxiliary_identity=objective.identity,
        preparation_sha256=_sha(out/'data_and_budget_verification.json'), source=head)
    teacher['manifest_sha256'] = content_sha256(teacher)
    identity = TimeCheckpointIdentity(split['manifest_sha256'], teacher['manifest_sha256'],
        content_sha256([r for r in split['runs'] if r['split']=='train']), 'multiscale_balanced_geometry', head)
    print('INITIAL_VALIDATION_EXACT_TRAINING_START', flush=True)
    result = run_training_arm(c['sampler'], Subset(val, selected), train_run_ids=c['sampler'].run_ids,
        validation_run_ids=selected_ids, split_manifest=split, teacher_manifest=teacher, identity=identity,
        config=config, plan=CorpusTrainingPlan(**plan['training']), output=root/plan['output'], resume=resume,
        initialization=source, initialization_sha256=plan['initialization_sha256'], initialization_identity=initial_identity,
        retain_epoch_checkpoints=True, recovery_objective=objective)
    before = read_json(root/plan['comparison_model']/'result.json')
    if (result['optimizer_steps'] != plan['expected']['optimizer_steps']
            or result['initial_weights_sha256'] != before['initial_weights_sha256']
            or not result['reload_predictions_exact']):
        raise ValueError('training budget or reload verification failed')
    write(root/plan['output']/'multiscale_verification.json', dict(status='PASS', source=head,
        best_checkpoint_sha256=_sha(root/plan['output']/'best.pt'), optimizer_steps=result['optimizer_steps'],
        initial_validation_exact=True, reload_predictions_exact=True, expected=plan['expected']))
    print(json.dumps(result), flush=True)


def compare(root: Path, repo: Path, plan: dict[str, Any]) -> None:
    c = context(root, repo, plan)
    check = read_json(root/plan['output']/'multiscale_verification.json')
    if check['status'] != 'PASS' or _sha(root/plan['output']/'best.pt') != check['best_checkpoint_sha256']:
        raise ValueError('verified completed training required')
    ds = c['validation']; n = len(ds)
    selected = c['selection']; selected_set = set(selected)
    remainder = [i for i in range(n) if i not in selected_set]
    source = read_json(root/plan['audit_output']/'source_verification.json')
    amplitudes = {r['run_id']:r['amplitude_group'] for r in source['runs']}
    groups = dict(nominal=[i for i,r in enumerate(ds.run_ids) if not r.startswith('codex-time-recovery-')],
        prior_recovery=[i for i,r in enumerate(ds.run_ids) if r.startswith('codex-time-recovery-') and r not in amplitudes])
    for amplitude in sorted(set(amplitudes.values())):
        groups['new_'+str(amplitude)+'m'] = [i for i,r in enumerate(ds.run_ids) if amplitudes.get(r)==amplitude]
    out = root/plan['output']/'comparison'; out.mkdir(exist_ok=False)
    truth = pp_rows(ds, list(range(n)), ds.targets, c['controller'], teacher=True)
    reports = {}
    for label, directory in [('before',root/plan['comparison_model']), ('after',root/plan['output'])]:
        checkpoint = directory/'best.pt'
        payload = torch.load(checkpoint, map_location='cpu', weights_only=False)
        config = TimeModelConfig.from_dict(payload['config']); identity = TimeCheckpointIdentity(**payload['identity'])
        model = build_time_model(config).to('cuda')
        load_time_checkpoint(checkpoint, config=config, identity=identity, model=model, mode='finetune')
        values = torch.full((n,30,2), float('nan'))
        for part in (selected,remainder):
            metrics, tensor = evaluate_time_batched(model, Subset(ds,part), run_ids=[ds.run_ids[i] for i in part],
                split_manifest=c['cache']['split_manifest'], batch_size=32, workers=4, precision='float32')
            values[part] = tensor
            if part is selected:
                if metrics != read_json(directory/'best_validation.json'):
                    raise ValueError('selection metrics did not exactly replay')
                np.testing.assert_allclose(tensor.numpy(), np.load(directory/f"validation_epoch_{payload['epoch']:02d}.npy"),
                                           rtol=0, atol=0, equal_nan=True)
        predictions = values.numpy(); np.save(out/(label+'_predictions.npy'), predictions)
        predicted_pp = pp_rows(ds,list(range(n)),predictions,c['controller'])
        report = dict(checkpoint=str(checkpoint), sha256=_sha(checkpoint), epoch=payload['epoch'], groups={})
        for group, indices in groups.items():
            runs = [ds.run_ids[i] for i in indices]
            report['groups'][group] = dict(anchors=len(indices), runs=len(set(runs)),
                xy=time_horizon_metrics(values[indices],torch.from_numpy(ds.targets[indices]),torch.from_numpy(ds.xy_mask[indices]),
                    input_valid=torch.from_numpy(ds.input_valid[indices]),run_ids=runs),
                components=component_errors(predictions[indices],ds.targets[indices],ds.xy_mask[indices],ds.input_valid[indices],runs),
                pp=pp_agreement_score([truth[i] for i in indices],[predicted_pp[i] for i in indices],runs),
                pp_applicability=summarize_pp([predicted_pp[i] for i in indices]))
        reports[label] = report; write(out/(label+'_metrics.json'),report)
        del model,payload; torch.cuda.empty_cache()
        print('MULTISCALE_COMPARISON_COMPLETE',label,flush=True)
    write(out/'summary.json',dict(status='COMPLETE',scope='OFFLINE_SAME_COURSE_VALIDATION_NOT_CLOSED_LOOP',
        models=reports, new_validation_used_for_selection=False, sealed_test_read=False, controller=c['controller']))


def main() -> None:
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('command',choices=['audit','prepare','train','compare'])
    ap.add_argument('--plan',type=Path,required=True); ap.add_argument('--root',type=Path,default=Path('..'))
    ap.add_argument('--resume',action='store_true'); args=ap.parse_args()
    root=args.root.resolve(); repo=Path(__file__).resolve().parents[1]
    if root.as_posix().startswith('/mnt/'):
        raise ValueError('native WSL root required')
    torch.set_num_threads(4)
    soft,hard=resource.getrlimit(resource.RLIMIT_NOFILE)
    resource.setrlimit(resource.RLIMIT_NOFILE,(max(soft,min(hard,8192)),hard))
    torch.backends.cuda.matmul.allow_tf32=False; torch.backends.cudnn.allow_tf32=False
    torch.backends.cudnn.benchmark=False; torch.backends.cudnn.deterministic=True
    plan=read_json(args.plan)
    if args.command=='train': train(root,repo,plan,resume=args.resume)
    else: {'audit':audit,'prepare':prepare,'compare':compare}[args.command](root,repo,plan)


if __name__=='__main__':
    main()
