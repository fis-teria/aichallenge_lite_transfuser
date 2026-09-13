"""Native WSL preparation and finite OFF-model recovery finetuning."""
from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import resource
import subprocess

import torch
from aic_transfuser_lite.data.time_recovery_training_v1 import prepare_recovery_cache, RecoveryMixDataset
from aic_transfuser_lite.data.time_training_cache_v1 import TimeTrainingCacheDataset, verify_time_training_cache, _sha
from aic_transfuser_lite.data.time_split_v1 import content_sha256
from aic_transfuser_lite.training.time_config_v1 import TimeModelConfig
from aic_transfuser_lite.training.time_checkpoint_v1 import TimeCheckpointIdentity
from aic_transfuser_lite.training.time_corpus_runner_v1 import CorpusTrainingPlan, run_training_arm


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('prepare','train'))
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--data-root', type=Path, default=Path('../'))
    parser.add_argument('--cache', type=Path, required=True)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--resume', action='store_true')
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[1]
    root = args.data_root.resolve()
    plan = json.loads(args.plan.read_text())
    torch.set_num_threads(4)
    soft, hard = resource.getrlimit(resource.RLIMIT_NOFILE)
    resource.setrlimit(resource.RLIMIT_NOFILE, (max(soft, min(hard,8192)), hard))
    if args.command == 'prepare':
        result = prepare_recovery_cache(root/plan['base_cache'], args.cache, plan=plan,
            raw_roots={name:root/path for name,path in plan['raw_roots'].items()},
            types=root/plan['types'], evidence_root=repo/'docs/evidence')
        print(json.dumps({'status':'COMPLETE','cache_sha256':result['manifest_sha256'],
                          'runs':result['recovery_audits']}), flush=True)
        return
    if args.output is None:
        parser.error('--output is required for training')
    if not torch.cuda.is_available():
        raise RuntimeError('real recovery training requires native WSL CUDA')
    cache = verify_time_training_cache(args.cache)
    if cache['plan'] != plan:
        raise ValueError('frozen preparation/training plans differ')
    source = root/plan['initialization']
    if _sha(source) != plan['initialization_sha256']:
        raise ValueError('initialization file hash mismatch')
    payload = torch.load(source, map_location='cpu', weights_only=False)
    config = TimeModelConfig.from_dict(payload['config'])
    if config.use_command_history:
        raise ValueError('this experiment is command OFF only')
    source_identity = TimeCheckpointIdentity(**payload['identity'])
    split = cache['split_manifest']
    if source_identity.split_manifest_sha256 != split['base_manifest']['manifest_sha256']:
        raise ValueError('inherited checkpoint does not match the original split')
    dataset = TimeTrainingCacheDataset(args.cache, 'train', verify_hashes=False)
    validation = TimeTrainingCacheDataset(args.cache, 'validation', verify_hashes=False)
    if dataset.config != config.dataset_config() or validation.config != config.dataset_config():
        raise ValueError('input preprocessing changed')
    recovery_ids = [r['run_id'] for r in plan['runs'] if r['split']=='train']
    train = RecoveryMixDataset(dataset, recovery_ids, repeats=plan['recovery_repeats'])
    teacher = {'format':'time_recovery_training_identity_v1','cache_sha256':cache['manifest_sha256'],
        'contract':cache['contract'], 'experiment_plan':plan,
        'unique_train_anchors':len(dataset), 'presented_train_anchors_per_epoch':len(train),
        'validation_anchors':len(validation),
        'train_anchor_order_sha256':content_sha256(train.anchor_ids),
        'validation_anchor_order_sha256':content_sha256(validation.anchor_ids)}
    teacher['manifest_sha256'] = content_sha256(teacher)
    git_sha = subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip()
    identity = TimeCheckpointIdentity(split['manifest_sha256'],teacher['manifest_sha256'],
        content_sha256([r for r in split['runs'] if r['split']=='train']),
        'finetune_command_off_verified_recovery_'+plan['initialization_sha256'], git_sha)
    training = CorpusTrainingPlan(**plan['training'])
    if args.resume and (args.output/'plan.json').is_file():
        original = json.loads((args.output/'plan.json').read_text())['identity']
        identity = TimeCheckpointIdentity(**{**asdict(identity),'source_git_commit':original['source_git_commit']})
    result = run_training_arm(train, validation, train_run_ids=train.run_ids,
        validation_run_ids=validation.run_ids, split_manifest=split, teacher_manifest=teacher,
        identity=identity, config=config, plan=training, output=args.output, resume=args.resume,
        initialization=source, initialization_sha256=plan['initialization_sha256'],
        initialization_identity=source_identity)
    initial = json.loads((args.output/'initial_validation.json').read_text())
    final = json.loads((args.output/'best_validation.json').read_text())
    heldout = {r['run_id'] for r in plan['runs'] if r['split']=='validation'}
    comparison = {'result':result,'initialization_sha256':plan['initialization_sha256'],
                  'best_checkpoint_sha256':_sha(args.output/'best.pt'),'test_evaluated':False,'groups':{}}
    for name, ids in [('nominal',set(validation.run_ids)-heldout),('recovery',heldout)]:
        comparison['groups'][name] = {'run_ids':sorted(ids),
            'initial_3s_run_macro_m':sum(initial['run_macro'][rid]['horizons']['3s']['raw_error_m'] for rid in ids)/len(ids),
            'candidate_3s_run_macro_m':sum(final['run_macro'][rid]['horizons']['3s']['raw_error_m'] for rid in ids)/len(ids)}
    (args.output/'comparison.json').write_text(json.dumps(comparison,indent=2,allow_nan=False)+'\n')
    print(json.dumps(comparison),flush=True)


if __name__ == '__main__':
    main()
