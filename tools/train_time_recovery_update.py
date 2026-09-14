"""Append audited recovery data and retrain with the frozen historical budget."""
from __future__ import annotations

import argparse
import ast
from collections import Counter
from dataclasses import asdict
import json
from pathlib import Path
import resource
import subprocess
from typing import Any

import torch
from torch.utils.data import Subset

from aic_transfuser_lite.data.time_recovery_append_v1 import prepare_append_cache, read_json
from aic_transfuser_lite.data.time_recovery_training_v1 import MatchedRecoveryMixDataset, RecoveryMixDataset
from aic_transfuser_lite.data.time_split_v1 import content_sha256
from aic_transfuser_lite.data.time_training_cache_v1 import TimeTrainingCacheDataset, verify_time_training_cache, _sha
from aic_transfuser_lite.evaluation.time_batched_v1 import evaluate_time_batched
from aic_transfuser_lite.training.time_checkpoint_v1 import TimeCheckpointIdentity, load_time_checkpoint
from aic_transfuser_lite.training.time_config_v1 import TimeModelConfig, build_time_model
from aic_transfuser_lite.training.time_corpus_runner_v1 import CorpusTrainingPlan, run_training_arm


def write(path: Path, value: Any) -> None:
    path.write_bytes((json.dumps(value, indent=2, allow_nan=False)+'\n').encode())


def reference_proof(root: Path, repo: Path, plan: dict[str, Any]) -> dict[str, Any]:
    """Check old optimizer/model code and fixed run budget before using CUDA."""
    if plan['test_usage'] != 'sealed' or plan['new_validation_usage'] != 'post_selection_diagnostic_only':
        raise ValueError('sealed test and separate new validation required')
    if subprocess.check_output(['git', 'status', '--porcelain'], cwd=repo, text=True).strip():
        raise ValueError('committed clean source required')
    for path, digest in ((root/plan['initialization'], plan['initialization_sha256']),
                         (root/plan['reference_training']/'best.pt', plan['reference_checkpoint_sha256']),
                         (root/plan['historical_source_proof'], plan['historical_source_proof_sha256'])):
        if _sha(path) != digest:
            raise ValueError('pinned historical artifact changed: '+str(path))
    prior = read_json(root/plan['historical_source_proof'])
    if (prior['source']['historical_source_commit'] != plan['historical_source_commit']
            or not prior['source']['training_ast_equal_after_removing_epoch_archive']):
        raise ValueError('historical optimizer parity proof missing')
    paths = ['src/aic_transfuser_lite/models', 'src/aic_transfuser_lite/training',
        'src/aic_transfuser_lite/contracts', 'src/aic_transfuser_lite/evaluation/time_batched_v1.py',
        'src/aic_transfuser_lite/evaluation/time_metrics_v1.py',
        'src/aic_transfuser_lite/data/time_training_cache_v1.py',
        'src/aic_transfuser_lite/data/time_dataset_v1.py', 'src/aic_transfuser_lite/data/time_teacher_v1.py',
        'src/aic_transfuser_lite/data/image_preprocess.py']
    changed = subprocess.check_output(['git', 'diff', '--name-only', plan['dependency_source_commit'],
        'HEAD', '--', *paths], cwd=repo, text=True).splitlines()
    if changed:
        raise ValueError(f'training dependency changed: {changed}')
    mix_path = 'src/aic_transfuser_lite/data/time_recovery_training_v1.py'
    old = ast.parse(subprocess.check_output(['git', 'show', plan['dependency_source_commit']+':'+mix_path],
                                          cwd=repo, text=True))
    current = ast.parse((repo/mix_path).read_text())
    for name in ('RecoveryMixDataset', 'MatchedRecoveryMixDataset'):
        def find(tree: ast.Module) -> str:
            return ast.dump(next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == name))
        if find(old) != find(current):
            raise ValueError('historical sampler changed: '+name)
    old_plan = read_json(root/plan['reference_training']/'plan.json')
    result = read_json(root/plan['reference_training']/'result.json')
    training = CorpusTrainingPlan(**plan['training'])
    if (any(old_plan[k] != v for k, v in asdict(training).items())
            or old_plan['initialization']['sha256'] != plan['initialization_sha256']
            or old_plan['max_anchors'] != plan['expected']['presentations_per_epoch']*training.epochs
            or old_plan['torch_version'] != torch.__version__
            or result['optimizer_steps'] != plan['expected']['optimizer_steps']
            or result['status'] != 'COMPLETE' or not result['reload_predictions_exact']):
        raise ValueError('historical training conditions or completion differ')
    return dict(source_commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=repo, text=True).strip(),
                historical_source=prior['source'], dependency_source_commit=plan['dependency_source_commit'],
                training_dependencies_unchanged=paths, sampler_classes_ast_equal=True,
                torch_version=torch.__version__, historical_control_reused=True)


def context(root: Path, plan: dict[str, Any]) -> dict[str, Any]:
    caches = {}
    for key in ('parent_cache', 'reference_cache', 'cache'):
        caches[key] = verify_time_training_cache(root/plan[key])
        if key != 'cache' and caches[key]['manifest_sha256'] != plan[key+'_sha256']:
            raise ValueError('pinned cache identity changed: '+key)
    cache, parent = caches['cache'], caches['parent_cache']
    if cache['plan'] != plan:
        raise ValueError('prepared cache uses another experiment plan')
    files = {r['path']: r for r in cache['cache_files']}
    if any(files.get(r['path']) != r for r in parent['cache_files']):
        raise ValueError('original cache bytes changed')
    full = TimeTrainingCacheDataset(root/plan['cache'], 'train', verify_hashes=False)
    val = TimeTrainingCacheDataset(root/plan['cache'], 'validation', verify_hashes=False)
    previous = TimeTrainingCacheDataset(root/plan['reference_cache'], 'train', verify_hashes=False)
    parent_data = TimeTrainingCacheDataset(root/plan['parent_cache'], 'train', verify_hashes=False)
    prior_ids = [r['run_id'] for r in caches['reference_cache']['plan']['runs'] if r['split'] == 'train']
    reference = RecoveryMixDataset(previous, prior_ids, repeats=plan['reference_recovery_repeats'])
    recovery_ids = [r['run_id'] for r in cache['split_manifest']['additional_runs'] if r['split'] == 'train']
    old_ids = [r['run_id'] for r in parent['split_manifest']['additional_runs'] if r['split'] == 'train']
    old_mix = MatchedRecoveryMixDataset(parent_data, reference, old_ids, seed=plan['training']['seed'])
    if content_sha256(old_mix.anchor_ids) != plan['reference_presentation_order_sha256']:
        raise ValueError('old presentation reference did not replay')
    train = MatchedRecoveryMixDataset(full, reference, recovery_ids, seed=plan['training']['seed'])
    selection = [i for i, rid in enumerate(val.run_ids) if rid in plan['selection_run_ids']]
    expected = plan['expected']
    if (len(full) != expected['unique_train'] or len(val) != expected['validation_total']
            or len(train) != expected['presentations_per_epoch'] or len(selection) != expected['selection_validation']
            or train.unique_recovery_anchors != expected['unique_recovery_train']
            or train.recovery_presentations != expected['recovery_presentations_per_epoch']
            or set(val.run_ids[i] for i in selection) != set(plan['selection_run_ids'])):
        raise ValueError('frozen counts do not match materialized population')
    nominal = [(i, a) for i, (a, r) in enumerate(zip(train.anchor_ids, train.run_ids)) if r not in recovery_ids]
    old_nominal = [(i, a) for i, (a, r) in enumerate(zip(old_mix.anchor_ids, old_mix.run_ids)) if r not in old_ids]
    if nominal != old_nominal or len(nominal) != expected['nominal_presentations_per_epoch']:
        raise ValueError('normal training slots changed')
    histogram = Counter(Counter(a for a, r in zip(train.anchor_ids, train.run_ids) if r in recovery_ids).values())
    return dict(cache=cache, full=full, validation=val, train=train, selection=selection,
                data_proof=dict(expected=expected, original_cache_files_byte_identical=len(parent['cache_files']),
                    original_split_preserved=True, nominal_slots_preserved=len(nominal),
                    reference_presentation_order_replayed=True, recovery_repeat_histogram=dict(histogram),
                    train_anchor_order_sha256=content_sha256(train.anchor_ids),
                    selection_anchor_order_sha256=content_sha256([val.anchor_ids[i] for i in selection]),
                    train_run_ids=sorted(set(train.run_ids)), selection_run_ids=plan['selection_run_ids'],
                    reserved_test_read=False))


def train_update(root: Path, repo: Path, plan: dict[str, Any], proof: dict[str, Any], *, resume: bool) -> None:
    if not torch.cuda.is_available():
        raise RuntimeError('native WSL CUDA required')
    c = context(root, plan)
    full, val, train, selected = (c[k] for k in ('full', 'validation', 'train', 'selection'))
    selected_ids = [val.run_ids[i] for i in selected]
    source, output = root/plan['initialization'], root/plan['output']
    payload = torch.load(source, map_location='cpu', weights_only=False)
    config = TimeModelConfig.from_dict(payload['config'])
    source_identity = TimeCheckpointIdentity(**payload['identity'])
    cache, split = c['cache'], c['cache']['split_manifest']
    if (config.use_command_history or config.dataset_config() != full.config
            or source_identity.split_manifest_sha256 != split['base_manifest']['manifest_sha256']):
        raise ValueError('inherited model/input/split contract changed')
    teacher = dict(format='appended_recovery_training_identity_v1', cache_sha256=cache['manifest_sha256'],
        contract=cache['contract'], experiment_plan=plan, proof=proof, data_proof=c['data_proof'],
        new_validation_used_for_selection=False)
    teacher['manifest_sha256'] = content_sha256(teacher)
    identity = TimeCheckpointIdentity(split['manifest_sha256'], teacher['manifest_sha256'],
        content_sha256([r for r in split['runs'] if r['split'] == 'train']),
        'matched_budget_random_recovery_update', proof['source_commit'])
    baseline = build_time_model(config).to('cuda')
    load_time_checkpoint(source, config=config, identity=source_identity, model=baseline, mode='finetune')
    metrics, _ = evaluate_time_batched(baseline, Subset(val, selected), run_ids=selected_ids, split_manifest=split,
        batch_size=plan['training']['batch_size'], workers=plan['training']['workers'], precision=plan['training']['precision'])
    if metrics != read_json(root/plan['reference_training']/'initial_validation.json'):
        raise ValueError('same-weight initial validation does not exactly replay; training not started')
    del baseline, payload
    torch.cuda.empty_cache()
    print('HISTORICAL_CONTROL_INITIAL_VALIDATION_EXACT', json.dumps(c['data_proof']), flush=True)
    result = run_training_arm(train, Subset(val, selected), train_run_ids=train.run_ids,
        validation_run_ids=selected_ids, split_manifest=split, teacher_manifest=teacher, identity=identity,
        config=config, plan=CorpusTrainingPlan(**plan['training']), output=output, resume=resume,
        initialization=source, initialization_sha256=plan['initialization_sha256'], initialization_identity=source_identity,
        retain_epoch_checkpoints=True)
    old_result = read_json(root/plan['reference_training']/'result.json')
    if (result['optimizer_steps'] != plan['expected']['optimizer_steps']
            or result['initial_weights_sha256'] != old_result['initial_weights_sha256']
            or read_json(output/'initial_validation.json') != read_json(root/plan['reference_training']/'initial_validation.json')
            or _sha(root/plan['reference_training']/'best.pt') != plan['reference_checkpoint_sha256']):
        raise ValueError('historical control weights or budget drift after training')
    write(output/'matched_budget_verification.json', dict(status='PASS', **proof, **c['data_proof'],
        initial_validation_exact=True, initial_weights_match=True, optimizer_steps=result['optimizer_steps'],
        best_checkpoint_sha256=_sha(output/'best.pt'), reload_predictions_exact=result['reload_predictions_exact']))
    print(json.dumps(result), flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('prepare', 'train'))
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--root', type=Path, default=Path('..'))
    parser.add_argument('--resume', action='store_true')
    args = parser.parse_args()
    root, repo, plan = args.root.resolve(), Path(__file__).resolve().parents[1], read_json(args.plan)
    if root.as_posix().startswith('/mnt/'):
        raise ValueError('native WSL dataset/output root required')
    torch.set_num_threads(4)
    soft, hard = resource.getrlimit(resource.RLIMIT_NOFILE)
    resource.setrlimit(resource.RLIMIT_NOFILE, (max(soft, min(hard, 8192)), hard))
    torch.backends.cuda.matmul.allow_tf32 = False
    torch.backends.cudnn.allow_tf32 = False
    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.deterministic = True
    proof = reference_proof(root, repo, plan)
    if args.command == 'prepare':
        cache = prepare_append_cache(root, root/plan['cache'], plan)
        print(json.dumps(dict(status='PREPARED_VERIFIED', cache_sha256=cache['manifest_sha256'], **proof)), flush=True)
    else:
        train_update(root, repo, plan, proof, resume=args.resume)


if __name__ == '__main__':
    main()
