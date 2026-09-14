"""Integrate verified recovery runs and train with the historical control budget."""
from __future__ import annotations

import argparse
import ast
from dataclasses import asdict
import json
from pathlib import Path
import resource
import subprocess

import torch
from torch.utils.data import Subset

from aic_transfuser_lite.data.time_recovery_training_v1 import (
    MatchedRecoveryMixDataset, RecoveryMixDataset, prepare_recovery_cache)
from aic_transfuser_lite.data.time_split_v1 import content_sha256
from aic_transfuser_lite.data.time_training_cache_v1 import (
    TimeTrainingCacheDataset, verify_time_training_cache, _sha)
from aic_transfuser_lite.training.time_checkpoint_v1 import TimeCheckpointIdentity, load_time_checkpoint
from aic_transfuser_lite.training.time_config_v1 import TimeModelConfig, build_time_model
from aic_transfuser_lite.training.time_corpus_runner_v1 import CorpusTrainingPlan, run_training_arm
from aic_transfuser_lite.evaluation.time_batched_v1 import evaluate_time_batched


def read(path: Path) -> dict:
    return json.loads(path.read_text())


def check_reference(root: Path, repo: Path, plan: dict) -> dict:
    collection = root / plan['collection_index']
    if _sha(collection) != plan['collection_index_sha256']:
        raise ValueError('collection index changed')
    index = read(collection)
    if index['status'] != 'COMPLETE_VERIFIED':
        raise ValueError('collection not completely verified')
    expected = {(r['run_id'], r['split']) for r in index['production_runs'] if r['split'] != 'evaluation_reserved'}
    assigned = {(r['run_id'], r['split']) for r in plan['runs'] if r['root'] == 'expansion'}
    if assigned != expected or len(assigned) != 10:
        raise ValueError('frozen expansion assignments changed')
    excluded = set(index['calibration_run_ids']) | {r['run_id'] for r in index['production_runs'] if r['split'] == 'evaluation_reserved'}
    if excluded & {r['run_id'] for r in plan['runs']}:
        raise ValueError('calibration or evaluation reservation leaked')
    old = root / plan['reference_training']
    if (_sha(old/'best.pt') != plan['reference_checkpoint_sha256']
            or _sha(root/plan['initialization']) != plan['initialization_sha256']):
        raise ValueError('pinned checkpoint changed')
    old_plan, result = read(old/'plan.json'), read(old/'result.json')
    if result['status'] != 'COMPLETE' or not result['reload_predictions_exact']:
        raise ValueError('historical control is incomplete')
    training = CorpusTrainingPlan(**plan['training'])
    if any(old_plan[k] != v for k, v in asdict(training).items()):
        raise ValueError('training hyperparameters differ from control')
    if (old_plan['initialization']['sha256'] != plan['initialization_sha256']
            or old_plan['max_anchors'] != plan['expected']['presentations_per_epoch'] * training.epochs
            or result['optimizer_steps'] != plan['expected']['optimizer_steps']
            or old_plan['torch_version'] != torch.__version__):
        raise ValueError('historical control initialization, budget or torch differs')
    # Cached-data training dependencies. Recovery teacher phase generation is
    # checked by exact old cache byte equality after preparation, not by this list.
    paths = ['src/aic_transfuser_lite/models', 'src/aic_transfuser_lite/training',
        'src/aic_transfuser_lite/contracts', 'src/aic_transfuser_lite/evaluation/time_batched_v1.py',
        'src/aic_transfuser_lite/evaluation/time_metrics_v1.py',
        'src/aic_transfuser_lite/data/time_training_cache_v1.py',
        'src/aic_transfuser_lite/data/time_dataset_v1.py',
        'src/aic_transfuser_lite/data/time_teacher_v1.py',
        'src/aic_transfuser_lite/data/image_preprocess.py']
    changed = subprocess.check_output(['git', 'diff', '--name-only', plan['reference_source_commit'], 'HEAD', '--', *paths],
                                     cwd=repo, text=True).splitlines()
    if changed:
        raise ValueError(f'historical control training dependency changed: {changed}')
    mix_path = 'src/aic_transfuser_lite/data/time_recovery_training_v1.py'
    old_text = subprocess.check_output(['git', 'show', plan['reference_source_commit']+':'+mix_path], cwd=repo, text=True)
    def mix_ast(source: str) -> str:
        return ast.dump(next(n for n in ast.parse(source).body if isinstance(n, ast.ClassDef) and n.name == 'RecoveryMixDataset'))
    if mix_ast(old_text) != mix_ast((repo/mix_path).read_text()):
        raise ValueError('historical control recovery sampler changed')
    return {'collection_sha256': plan['collection_index_sha256'], 'excluded_run_ids': sorted(excluded),
        'historical_control_reused': True, 'reference_checkpoint_sha256': plan['reference_checkpoint_sha256'],
        'unchanged_training_dependency_paths': paths, 'torch_version': torch.__version__}


def check_old_cache(original: dict, combined: dict, plan: dict) -> None:
    if original['manifest_sha256'] != plan['reference_cache_sha256'] or combined['plan'] != plan:
        raise ValueError('cache/plan identity mismatch')
    before = {r['path']: (r['bytes'], r['sha256']) for r in original['cache_files']}
    after = {r['path']: (r['bytes'], r['sha256']) for r in combined['cache_files']}
    if any(after.get(k) != v for k, v in before.items()):
        raise ValueError('original training or validation cache bytes changed')
    prior_runs = {r['run_id']: r for r in original['split_manifest']['runs']}
    actual_runs = {r['run_id']: r for r in combined['split_manifest']['runs']}
    if any(actual_runs.get(k) != v for k, v in prior_runs.items()):
        raise ValueError('original split or source hashes changed')


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('prepare', 'train'))
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--root', type=Path, default=Path('..'))
    parser.add_argument('--cache', type=Path, required=True)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--resume', action='store_true')
    args = parser.parse_args()
    root, repo = args.root.resolve(), Path(__file__).resolve().parents[1]
    plan = read(args.plan)
    torch.set_num_threads(4)
    soft, hard = resource.getrlimit(resource.RLIMIT_NOFILE)
    resource.setrlimit(resource.RLIMIT_NOFILE, (max(soft, min(hard, 8192)), hard))
    proof = check_reference(root, repo, plan)
    original = verify_time_training_cache(root/plan['reference_cache'])
    if args.command == 'prepare':
        cache = prepare_recovery_cache(root/plan['base_cache'], args.cache, plan=plan,
            raw_roots={k: root/v for k, v in plan['raw_roots'].items()}, types=root/plan['types'],
            evidence_root=repo/'docs/evidence')
        check_old_cache(original, cache, plan)
        print(json.dumps({'status': 'PREPARED_VERIFIED', 'cache_sha256': cache['manifest_sha256'],
            'original_cache_files_byte_identical': len(original['cache_files']), **proof}), flush=True)
        return
    if args.output is None:
        parser.error('--output is required for train')
    if not torch.cuda.is_available():
        raise RuntimeError('native WSL CUDA required')
    cache = verify_time_training_cache(args.cache)
    check_old_cache(original, cache, plan)
    full = TimeTrainingCacheDataset(args.cache, 'train', verify_hashes=False)
    validation = TimeTrainingCacheDataset(args.cache, 'validation', verify_hashes=False)
    previous = TimeTrainingCacheDataset(root/plan['reference_cache'], 'train', verify_hashes=False)
    prior_ids = [r['run_id'] for r in original['plan']['runs'] if r['split'] == 'train']
    reference = RecoveryMixDataset(previous, prior_ids, repeats=plan['reference_recovery_repeats'])
    recovery_ids = [r['run_id'] for r in plan['runs'] if r['split'] == 'train']
    train = MatchedRecoveryMixDataset(full, reference, recovery_ids, seed=plan['training']['seed'])
    selected = [i for i, rid in enumerate(validation.run_ids) if rid in plan['selection_run_ids']]
    selected_ids = [validation.run_ids[i] for i in selected]
    expected = plan['expected']
    if (len(full) != expected['unique_train'] or len(train) != expected['presentations_per_epoch']
            or len(validation) != expected['validation_total'] or len(selected) != expected['selection_validation']
            or train.recovery_presentations != expected['recovery_presentations_per_epoch']
            or train.unique_recovery_anchors != expected['unique_recovery_train']
            or set(selected_ids) != set(plan['selection_run_ids'])):
        raise ValueError('frozen counts differ from materialized population')
    source = root/plan['initialization']
    payload = torch.load(source, map_location='cpu', weights_only=False)
    config = TimeModelConfig.from_dict(payload['config'])
    source_identity = TimeCheckpointIdentity(**payload['identity'])
    if (config.use_command_history or config.dataset_config() != full.config
            or source_identity.split_manifest_sha256 != cache['split_manifest']['base_manifest']['manifest_sha256']):
        raise ValueError('input or inherited split contract differs')
    teacher = {'format': 'matched_recovery_training_identity_v1', 'cache_sha256': cache['manifest_sha256'],
        'contract': cache['contract'], 'experiment_plan': plan, 'proof': proof,
        'unique_train_anchors': len(full), 'presented_train_anchors_per_epoch': len(train),
        'train_anchor_order_sha256': content_sha256(train.anchor_ids),
        'selection_validation_anchor_ids_sha256': content_sha256([validation.anchor_ids[i] for i in selected]),
        'selection_run_ids': plan['selection_run_ids'], 'new_validation_used_for_selection': False}
    teacher['manifest_sha256'] = content_sha256(teacher)
    split = cache['split_manifest']
    commit = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=repo, text=True).strip()
    identity = TimeCheckpointIdentity(split['manifest_sha256'], teacher['manifest_sha256'],
        content_sha256([r for r in split['runs'] if r['split'] == 'train']),
        'matched_budget_outward_recovery_finetune', commit)
    if args.resume and (args.output/'plan.json').is_file():
        identity = TimeCheckpointIdentity(**{**asdict(identity),
            'source_git_commit': read(args.output/'plan.json')['identity']['source_git_commit']})
    torch.backends.cuda.matmul.allow_tf32 = False
    torch.backends.cudnn.allow_tf32 = False
    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.deterministic = True
    baseline = build_time_model(config).to('cuda')
    load_time_checkpoint(source, config=config, identity=source_identity, model=baseline, mode='finetune')
    initial_metrics, _ = evaluate_time_batched(baseline, Subset(validation, selected), run_ids=selected_ids,
        split_manifest=split, batch_size=plan['training']['batch_size'], workers=plan['training']['workers'],
        precision=plan['training']['precision'])
    if initial_metrics != read(root/plan['reference_training']/'initial_validation.json'):
        raise ValueError('initial validation does not exactly replay historical control; train not started')
    del baseline
    torch.cuda.empty_cache()
    print('HISTORICAL_CONTROL_INITIAL_VALIDATION_EXACT', flush=True)
    result = run_training_arm(train, Subset(validation, selected), train_run_ids=train.run_ids,
        validation_run_ids=selected_ids, split_manifest=split, teacher_manifest=teacher, identity=identity,
        config=config, plan=CorpusTrainingPlan(**plan['training']), output=args.output, resume=args.resume,
        initialization=source, initialization_sha256=plan['initialization_sha256'], initialization_identity=source_identity)
    if result['optimizer_steps'] != expected['optimizer_steps']:
        raise ValueError('actual optimizer update count differs from control')
    initial = read(args.output/'initial_validation.json')
    prior_initial = read(root/plan['reference_training']/'initial_validation.json')
    if initial != prior_initial:
        raise ValueError('same-weight same-data initial validation does not exactly replay historical control')
    with (args.output/'matched_budget_verification.json').open('x') as stream:
        json.dump({**proof, 'initial_validation_exact': True, 'original_cache_files_byte_identical': len(original['cache_files']),
            'nominal_slots_preserved': True, 'optimizer_steps': result['optimizer_steps'],
            'initial_weights_match': result['initial_weights_sha256'] == read(root/plan['reference_training']/'result.json')['initial_weights_sha256'],
            'best_checkpoint_sha256': _sha(args.output/'best.pt'), 'expected': expected}, stream, indent=2)
    print(json.dumps(result), flush=True)


if __name__ == '__main__':
    main()
