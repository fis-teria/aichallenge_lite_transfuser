"""Append verified corner collections and finetune with complete train coverage."""
from __future__ import annotations

import argparse
from collections import Counter
from copy import deepcopy
from dataclasses import replace
import json
from pathlib import Path
import resource
import subprocess
from typing import Any

import numpy as np
import torch
from torch.utils.data import Subset

from compare_time_recovery_objectives import geometry_targets, make_objective
from compare_time_training_methods import pp_rows
from train_time_multiscale_recovery import training_teacher_manifest
from train_time_recovery_update import read_json, write
from aic_transfuser_lite.data.time_multiscale_recovery_v1 import audit_collections
from aic_transfuser_lite.data.time_outward_balanced_v1 import OutwardBalancedMixDataset
from aic_transfuser_lite.data.time_recovery_append_v1 import prepare_append_cache
from aic_transfuser_lite.data.time_recovery_training_v1 import RecoveryMixDataset, MatchedRecoveryMixDataset
from aic_transfuser_lite.data.time_split_v1 import content_sha256
from aic_transfuser_lite.data.time_training_cache_v1 import TimeTrainingCacheDataset, verify_time_training_cache, _sha
from aic_transfuser_lite.evaluation.time_batched_v1 import evaluate_time_batched
from aic_transfuser_lite.evaluation.time_metrics_v1 import time_horizon_metrics
from aic_transfuser_lite.evaluation.time_method_selection_v1 import pp_agreement_score
from aic_transfuser_lite.evaluation.time_recovery_comparison_v1 import component_errors, summarize_pp
from aic_transfuser_lite.training.time_checkpoint_v1 import TimeCheckpointIdentity, load_time_checkpoint
from aic_transfuser_lite.training.time_config_v1 import TimeModelConfig, build_time_model
from aic_transfuser_lite.training.time_corpus_runner_v1 import CorpusTrainingPlan, run_training_arm
from aic_transfuser_lite.training.time_recovery_geometry_v1 import RecoveryGeometryPlan


def require_completed_collection(summary: dict[str, Any]) -> None:
    """Only independently completed, stopped, closed recordings may be appended."""
    if (summary.get('result_status') != 'COMPLETE_LAP' or summary.get('fault') is not None
            or summary.get('stop_confirmed') is not True or summary.get('closed_bag') is not True):
        raise ValueError('completed lap, no fault, confirmed stop and closed bag required')


def build_sampler(full: Any, recovery_ids: list[str], targets: list[str],
                  *, repeats: int, fraction: float, seed: int) -> OutwardBalancedMixDataset:
    """Present every unique [30,2] metre teacher; preserve nominal anchors once."""
    reference = RecoveryMixDataset(full, recovery_ids, repeats=repeats)
    uniform = MatchedRecoveryMixDataset(full, reference, recovery_ids, seed=seed)
    sampler = OutwardBalancedMixDataset(uniform, recovery_ids, targets,
                                        target_fraction=fraction, seed=seed)
    if set(sampler.anchor_ids) != set(full.anchor_ids):
        raise ValueError('expanded presentation budget dropped an anchor')
    nominal = [a for a, r in zip(sampler.anchor_ids, sampler.run_ids) if r not in set(recovery_ids)]
    if len(nominal) != len(set(nominal)):
        raise ValueError('nominal anchors must be presented exactly once')
    return sampler


def audit(root: Path, repo: Path, plan: dict[str, Any]) -> None:
    out = root/plan['audit_output']
    out.mkdir(parents=True, exist_ok=False)
    for spec in plan['collections']:
        analysis = root/spec['analysis']
        index = analysis/'collection_index.json'
        if _sha(index) != spec['index_sha256']:
            raise ValueError('collection index changed')
        for row in read_json(index)['runs']:
            if row.get('accepted', row.get('accepted_camera_anchors', 0)):
                require_completed_collection(read_json(analysis/(row['run_id']+'_collection_summary.json')))
    proof, additions = audit_collections(root, plan['collections'], out)
    if {s: proof['totals'][s]['anchors'] for s in ('train', 'validation')} != plan['expected_additions']:
        raise ValueError('audited addition counts differ from frozen inventory')
    write(out/'source_verification.json', proof)
    resolved = deepcopy(plan)
    resolved['additions'] = additions
    for row in additions:
        row['proof_sha256'] = _sha(out/'source_verification.json')
    write(out/'resolved_plan.json', resolved)
    print(json.dumps(dict(status='AUDITED', totals=proof['totals'])), flush=True)


def context(root: Path, repo: Path, plan: dict[str, Any]) -> dict[str, Any]:
    if plan['test_usage'] != 'sealed' or plan['new_validation_usage'] != 'post_selection_diagnostic_only':
        raise ValueError('sealed test and independent diagnostic validation required')
    protected = ['src/aic_transfuser_lite/models', 'src/aic_transfuser_lite/training',
        'src/aic_transfuser_lite/contracts', 'src/aic_transfuser_lite/evaluation',
        'src/aic_transfuser_lite/control', 'src/aic_transfuser_lite/data/time_dataset_v1.py',
        'src/aic_transfuser_lite/data/time_teacher_v1.py', 'src/aic_transfuser_lite/data/time_training_cache_v1.py']
    if subprocess.check_output(['git', 'diff', plan['source_guard_commit'], 'HEAD', '--', *protected], cwd=repo):
        raise ValueError('model, objective, controller or input implementation changed')
    cache = verify_time_training_cache(root/plan['cache'])
    if cache['plan'] != plan or cache['parent_cache_sha256'] != plan['parent_cache_sha256']:
        raise ValueError('cache plan or parent identity changed')
    source = read_json(root/plan['audit_output']/'source_verification.json')
    for row in plan['additions']:
        if _sha(root/row['analysis']/row['proof']) != row['proof_sha256']:
            raise ValueError('audited source proof changed')
    prior = root/plan['prior_preparation']
    if _sha(prior) != plan['prior_preparation_sha256']:
        raise ValueError('prior train sampling evidence changed')
    full = TimeTrainingCacheDataset(root/plan['cache'], 'train', verify_hashes=False)
    val = TimeTrainingCacheDataset(root/plan['cache'], 'validation', verify_hashes=False)
    recovery_ids = sorted(r for r in set(full.run_ids) if r.startswith('codex-time-recovery-'))
    targets = set(read_json(prior)['target_anchor_ids'])
    targets.update(a for r in source['runs'] if r['split'] == 'train' for a in r['early_train_anchor_ids'])
    sampler = build_sampler(full, recovery_ids, sorted(targets), repeats=plan['recovery_repeats'],
                            fraction=plan['balanced_target_fraction'], seed=plan['training']['seed'])
    selected = [i for i, r in enumerate(val.run_ids) if r in plan['selection_run_ids']]
    actual = dict(unique_train=len(full), unique_recovery_train=sampler.unique_recovery_anchors,
        validation_total=len(val), selection_validation=len(selected), presentations_per_epoch=len(sampler))
    if actual != plan['expected'] or set(val.run_ids[i] for i in selected) != set(plan['selection_run_ids']):
        raise ValueError('frozen population changed: '+str(actual))
    if _sha(root/plan['initialization']) != plan['initialization_sha256']:
        raise ValueError('initial checkpoint changed')
    controller = {**read_json(repo/plan['offline_controller_config']), 'lookahead_policy':'stopping_preview_extended_v1'}
    proof = dict(counts=actual, cache_sha256=cache['manifest_sha256'], sampler=sampler.audit,
        sampler_order_sha256=content_sha256(sampler.anchor_ids), target_anchor_ids=sorted(targets),
        source_verification_sha256=_sha(root/plan['audit_output']/'source_verification.json'),
        nominal_unique_anchors=len(full)-sampler.unique_recovery_anchors, sealed_test_read=False,
        new_validation_used_for_selection=False, controller=controller)
    return dict(full=full, validation=val, sampler=sampler, selection=selected, source=source,
                cache=cache, controller=controller, proof=proof)


def prepare(root: Path, repo: Path, plan: dict[str, Any]) -> None:
    prepare_append_cache(root, root/plan['cache'], plan)
    c = context(root, repo, plan)
    out = root/plan['audit_output']
    geometry = {**plan['geometry'], 'rear_axle_offset_m':tuple(plan['geometry']['rear_axle_offset_m'])}
    rows = geometry_targets(c, RecoveryGeometryPlan(**geometry))
    write(out/'recovery_geometry_targets.json', rows)
    write(out/'geometry_identity.json', make_objective(c, plan, rows).identity)
    write(out/'data_and_budget_verification.json', c['proof'])
    print(json.dumps(dict(status='PREPARED', counts=c['proof']['counts'],
                         target_anchors=len(c['proof']['target_anchor_ids']))), flush=True)


def execution_plan(plan: dict[str, Any], loader_workers: int | None) -> CorpusTrainingPlan:
    """Change only input-loading concurrency, retaining the mathematical plan."""
    result = CorpusTrainingPlan(**plan['training'])
    if loader_workers is not None:
        result = replace(result, workers=loader_workers)
    result.validate()
    return result


def train(root: Path, repo: Path, plan: dict[str, Any], *, resume: bool,
          loader_workers: int | None = None, output: Path | None = None) -> None:
    if not torch.cuda.is_available():
        raise RuntimeError('native WSL CUDA required')
    c = context(root, repo, plan)
    out = root/plan['audit_output']
    saved = read_json(out/'data_and_budget_verification.json')
    if any(saved.get(k) != json.loads(json.dumps(v)) for k, v in c['proof'].items()):
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
    head = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=repo, text=True).strip()
    teacher = training_teacher_manifest(plan=plan, cache=c['cache'], auxiliary_identity=objective.identity,
        preparation_sha256=_sha(out/'data_and_budget_verification.json'), source=head)
    actual_plan = execution_plan(plan, loader_workers)
    output = output or root/plan['output']
    teacher['actual_loader_workers'] = actual_plan.workers
    teacher.pop('manifest_sha256')
    teacher['manifest_sha256'] = content_sha256(teacher)
    split, val, selected = c['cache']['split_manifest'], c['validation'], c['selection']
    identity = TimeCheckpointIdentity(split['manifest_sha256'], teacher['manifest_sha256'],
        content_sha256([r for r in split['runs'] if r['split']=='train']), 'corner_recovery_finetune', head)
    result = run_training_arm(c['sampler'], Subset(val, selected), train_run_ids=c['sampler'].run_ids,
        validation_run_ids=[val.run_ids[i] for i in selected], split_manifest=split, teacher_manifest=teacher,
        identity=identity, config=config, plan=actual_plan, output=output,
        resume=resume, initialization=source, initialization_sha256=plan['initialization_sha256'],
        initialization_identity=initial_identity, retain_epoch_checkpoints=True, recovery_objective=objective)
    if (not result['reload_predictions_exact'] or result['optimizer_steps'] != plan['expected_optimizer_steps']
            or read_json(output/'initial_validation.json') != read_json(source.parent/'best_validation.json')):
        raise ValueError('reload, training budget or initial validation replay failed')
    write(output/'verification.json', dict(status='PASS', source_commit=head,
        checkpoint_sha256=_sha(output/'best.pt'), optimizer_steps=result['optimizer_steps'],
        reload_predictions_exact=True, initial_validation_exact=True, sealed_test_read=False,
        actual_loader_workers=actual_plan.workers))
    print(json.dumps(dict(status='TRAINING_COMPLETE', best_epoch=result['best_epoch'])), flush=True)


def compare(root: Path, repo: Path, plan: dict[str, Any], *, loader_workers: int | None = None,
            output: Path | None = None) -> None:
    c = context(root, repo, plan)
    output = output or root/plan['output']
    workers = execution_plan(plan, loader_workers).workers
    check = read_json(output/'verification.json')
    if check['status'] != 'PASS' or _sha(output/'best.pt') != check['checkpoint_sha256']:
        raise ValueError('verified completed training required')
    out = root/plan['audit_output']/'comparison'
    out.mkdir(exist_ok=False)
    ds = c['validation']; selected = c['selection']; selection_set = set(selected)
    remainder = [i for i in range(len(ds)) if i not in selection_set]
    new_runs = {r['run_id'] for r in c['source']['runs']}
    groups = dict(selection=selected,
        nominal=[i for i, r in enumerate(ds.run_ids) if not r.startswith('codex-time-recovery-')],
        prior_recovery=[i for i, r in enumerate(ds.run_ids) if r.startswith('codex-time-recovery-') and r not in new_runs],
        added_recovery=[i for i, r in enumerate(ds.run_ids) if r in new_runs])
    for group in plan['collections']:
        ids = {r['run_id'] for r in read_json(root/group['analysis']/'collection_index.json')['runs']}
        groups[Path(group['analysis']).name] = [i for i, r in enumerate(ds.run_ids) if r in ids]
    truth = pp_rows(ds, list(range(len(ds))), ds.targets, c['controller'], teacher=True)
    reports = {}
    for label, checkpoint in [('before', root/plan['initialization']), ('after', output/'best.pt')]:
        payload = torch.load(checkpoint, map_location='cpu', weights_only=False)
        config = TimeModelConfig.from_dict(payload['config'])
        model = build_time_model(config).to('cuda')
        load_time_checkpoint(checkpoint, config=config, identity=TimeCheckpointIdentity(**payload['identity']),
                             model=model, mode='finetune')
        values = torch.full((len(ds), 30, 2), float('nan'))
        for indices in (selected, remainder):
            metrics, pred = evaluate_time_batched(model, Subset(ds, indices), run_ids=[ds.run_ids[i] for i in indices],
                split_manifest=c['cache']['split_manifest'], batch_size=32, workers=workers, precision='float32')
            values[indices] = pred
            if indices is selected and metrics != read_json(checkpoint.parent/'best_validation.json'):
                raise ValueError('selected validation did not exactly replay')
        np.save(out/(label+'_predictions.npy'), values.numpy())
        pp = pp_rows(ds, list(range(len(ds))), values.numpy(), c['controller'])
        report = dict(checkpoint_sha256=_sha(checkpoint), epoch=payload['epoch'], groups={})
        for group, indices in groups.items():
            runs = [ds.run_ids[i] for i in indices]
            report['groups'][group] = dict(anchors=len(indices), runs=len(set(runs)),
                xy=time_horizon_metrics(values[indices], torch.from_numpy(ds.targets[indices]),
                    torch.from_numpy(ds.xy_mask[indices]), input_valid=torch.from_numpy(ds.input_valid[indices]), run_ids=runs),
                components=component_errors(values.numpy()[indices], ds.targets[indices], ds.xy_mask[indices], ds.input_valid[indices], runs),
                pp=pp_agreement_score([truth[i] for i in indices], [pp[i] for i in indices], runs),
                pp_applicability=summarize_pp([pp[i] for i in indices]))
        reports[label] = report
        write(out/(label+'_metrics.json'), report)
        del model, payload
        torch.cuda.empty_cache()
        print('COMPARISON_COMPLETE', label, flush=True)
    write(out/'summary.json', dict(status='COMPLETE', models=reports,
        scope='OFFLINE_SAME_COURSE_INDEPENDENT_RUNS_NOT_CLOSED_LOOP', sealed_test_read=False,
        new_validation_used_for_selection=False, controller=c['controller']))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('command', choices=('audit', 'prepare', 'train', 'compare'))
    ap.add_argument('--plan', type=Path, required=True)
    ap.add_argument('--root', type=Path, default=Path('..'))
    ap.add_argument('--resume', action='store_true')
    ap.add_argument('--loader-workers', type=int)
    ap.add_argument('--training-output', type=Path)
    args = ap.parse_args()
    root, repo = args.root.resolve(), Path(__file__).resolve().parents[1]
    if root.as_posix().startswith('/mnt/') or subprocess.check_output(['git', 'status', '--porcelain'], cwd=repo).strip():
        raise ValueError('clean source and native WSL data root required')
    torch.set_num_threads(4)
    soft, hard = resource.getrlimit(resource.RLIMIT_NOFILE)
    resource.setrlimit(resource.RLIMIT_NOFILE, (max(soft, min(hard, 8192)), hard))
    torch.backends.cuda.matmul.allow_tf32 = False
    torch.backends.cudnn.allow_tf32 = False
    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.deterministic = True
    plan = read_json(args.plan)
    output = args.training_output.resolve() if args.training_output else None
    if output is not None and not output.is_relative_to(root/'runs'):
        raise ValueError('training output must stay in the native runs directory')
    if args.command not in ('train', 'compare') and (output is not None or args.loader_workers is not None):
        raise ValueError('execution overrides apply only to training and comparison')
    if args.command == 'train':
        train(root, repo, plan, resume=args.resume, loader_workers=args.loader_workers, output=output)
    elif args.command == 'compare':
        compare(root, repo, plan, loader_workers=args.loader_workers, output=output)
    else:
        {'audit':audit, 'prepare':prepare, 'compare':compare}[args.command](root, repo, plan)


if __name__ == '__main__':
    main()
