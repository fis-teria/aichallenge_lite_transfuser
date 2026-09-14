"""Compare frozen old/new weights on run-separated validation after selection."""
from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
import resource
import subprocess

import numpy as np
import torch
from torch.utils.data import Subset

from compare_time_training_methods import pp_rows
from train_time_recovery_update import read_json, write, reference_proof
from aic_transfuser_lite.data.time_training_cache_v1 import TimeTrainingCacheDataset, verify_time_training_cache, _sha
from aic_transfuser_lite.evaluation.time_batched_v1 import evaluate_time_batched
from aic_transfuser_lite.evaluation.time_method_selection_v1 import pp_agreement_score
from aic_transfuser_lite.evaluation.time_metrics_v1 import time_horizon_metrics
from aic_transfuser_lite.evaluation.time_recovery_comparison_v1 import component_errors, summarize_pp
from aic_transfuser_lite.training.time_checkpoint_v1 import TimeCheckpointIdentity, load_time_checkpoint
from aic_transfuser_lite.training.time_config_v1 import TimeModelConfig, build_time_model


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--root', type=Path, default=Path('..'))
    args = parser.parse_args()
    root, repo, plan = args.root.resolve(), Path(__file__).resolve().parents[1], read_json(args.plan)
    torch.set_num_threads(4)
    torch.backends.cuda.matmul.allow_tf32 = False
    torch.backends.cudnn.allow_tf32 = False
    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.deterministic = True
    soft, hard = resource.getrlimit(resource.RLIMIT_NOFILE)
    resource.setrlimit(resource.RLIMIT_NOFILE, (max(soft, min(hard, 8192)), hard))
    proof = reference_proof(root, repo, plan)
    training = root/plan['output']
    verified = read_json(training/'matched_budget_verification.json')
    if (verified['status'] != 'PASS' or not verified['reload_predictions_exact']
            or _sha(training/'best.pt') != verified['best_checkpoint_sha256']):
        raise ValueError('completed verified training required')
    cache = verify_time_training_cache(root/plan['cache'])
    if cache['plan'] != plan:
        raise ValueError('frozen comparison dataset changed')
    ds = TimeTrainingCacheDataset(root/plan['cache'], 'validation', verify_hashes=False)
    original_ids = set(plan['selection_run_ids'])
    expanded_ids = {'codex-time-recovery-pulseleft-r46', 'codex-time-recovery-pulseright-r47'}
    spec = next(r for r in plan['additions'] if r['split'] == 'validation')
    random_id = spec['run_id']
    collection_file = root/spec['analysis']/spec['proof']
    if _sha(collection_file) != spec['proof_sha256']:
        raise ValueError('collection event assignments changed')
    collection = next(r for r in read_json(collection_file)['runs'] if r['run_id'] == random_id)
    targets = {a for e in collection['events'] for a in e['target_anchor_ids']}
    partitions = [[i for i, rid in enumerate(ds.run_ids) if rid in ids]
                  for ids in (original_ids, expanded_ids, {random_id})]
    if (list(map(len, partitions)) != [plan['expected']['selection_validation'], 186, spec['anchors']]
            or sorted(i for part in partitions for i in part) != list(range(len(ds)))):
        raise ValueError('comparison membership or historical batch shapes changed')
    old_recovery = {rid for rid in original_ids if rid.startswith('codex-time-recovery-')}
    groups = dict(nominal=[i for i in partitions[0] if ds.run_ids[i] not in old_recovery],
                  old_recovery=[i for i in partitions[0] if ds.run_ids[i] in old_recovery],
                  expanded_recovery=partitions[1], random_recovery=partitions[2],
                  random_outward=[i for i in partitions[2] if ds.anchor_ids[i] in targets])
    if len(groups['random_outward']) != 6 or len(targets) != 6:
        raise ValueError('strict outward diagnostic population changed')
    for event in collection['events']:
        indices = [i for i in partitions[2] if ds._anchors[i]['recovery_event_id'] == event['event_id']]
        if len(indices) != event['accepted']['count']:
            raise ValueError('event population differs from collection audit')
        groups['random_event_'+str(event['event_id'])] = indices
    controller = read_json(repo/plan['offline_controller_config'])
    if plan['offline_controller_age_s'] != 0.0:
        raise ValueError('only frozen observed-state PP is budgeted')
    all_indices = list(range(len(ds)))
    truth = pp_rows(ds, all_indices, ds.targets, controller, teacher=True)
    # Training has already selected both checkpoints using the original six runs.
    out = training/'comparison'
    out.mkdir(exist_ok=False)
    reports = {}
    for name, folder in (('before', root/plan['reference_training']), ('after', training)):
        checkpoint = folder/'best.pt'
        result = read_json(folder/'result.json')
        epoch = result['best_epoch']
        payload = torch.load(checkpoint, map_location='cpu', weights_only=False)
        if payload['epoch'] != epoch or result['status'] != 'COMPLETE':
            raise ValueError('selected checkpoint identity mismatch')
        config, identity = TimeModelConfig.from_dict(payload['config']), TimeCheckpointIdentity(**payload['identity'])
        model = build_time_model(config).to('cuda')
        load_time_checkpoint(checkpoint, config=config, identity=identity, model=model, mode='finetune')
        tensor = torch.full((len(ds), 30, 2), float('nan'))
        for part in partitions:
            metrics, values = evaluate_time_batched(model, Subset(ds, part), run_ids=[ds.run_ids[i] for i in part],
                split_manifest=cache['split_manifest'], batch_size=32, workers=4, precision='float32')
            tensor[part] = values
            if part == partitions[0] and metrics != read_json(folder/'best_validation.json'):
                raise ValueError('selected metrics did not replay exactly')
        predictions = tensor.numpy()
        np.testing.assert_allclose(predictions[partitions[0]], np.load(folder/f'validation_epoch_{epoch:02d}.npy'),
                                   rtol=0, atol=0, equal_nan=True)
        np.save(out/(name+'_predictions.npy'), predictions, allow_pickle=False)
        pps = pp_rows(ds, all_indices, predictions, controller)
        report = dict(epoch=epoch, checkpoint=str(checkpoint), sha256=_sha(checkpoint),
                      training_source_commit=identity.source_git_commit, selected_prediction_replay_exact=True, groups={})
        for group, ix in groups.items():
            runs = [ds.run_ids[i] for i in ix]
            report['groups'][group] = dict(anchor_count=len(ix), run_count=len(set(runs)),
                xy=time_horizon_metrics(tensor[ix], torch.from_numpy(ds.targets[ix]), torch.from_numpy(ds.xy_mask[ix]),
                    input_valid=torch.from_numpy(ds.input_valid[ix]), run_ids=runs),
                components=component_errors(predictions[ix], ds.targets[ix], ds.xy_mask[ix], ds.input_valid[ix], runs),
                pp=pp_agreement_score([truth[i] for i in ix], [pps[i] for i in ix], runs),
                pp_applicability=summarize_pp([pps[i] for i in ix]))
        reports[name] = report
        write(out/(name+'_metrics.json'), report)
        del model, payload
        torch.cuda.empty_cache()
        print('COMPARISON_MODEL_COMPLETE', name, flush=True)
    old_history = read_json(root/plan['reference_training']/'history.json')
    new_history = read_json(training/'history.json')
    for old, new in zip(old_history, new_history, strict=True):
        if any(old['train_counts'][k] != new['train_counts'][k]
               for k in ('visited', 'input_invalid', 'teacher_unsupported', 'supported')):
            raise ValueError('train support count budget changed')
    summary = dict(status='COMPLETE', scope='OFFLINE_VALIDATION_AND_PP_CALCULATION_NOT_CLOSED_LOOP',
        source=proof, models=reports, comparison_batches_separated_by_role=True,
        training_support_and_update_budget_equal=True, selection_run_ids=plan['selection_run_ids'],
        held_out_random_run=random_id, held_out_random_events=3,
        event_anchor_counts=Counter(str(ds._anchors[i]['recovery_event_id']) for i in partitions[2]),
        random_validation_used_for_selection=False, comparison_runs_not_independent_events=True,
        reserved_test_read=False, new_awsim_trials=0, controller=controller,
        controller_config_sha256=_sha(repo/plan['offline_controller_config']))
    write(out/'summary.json', summary)
    print('RECOVERY_UPDATE_COMPARISON_COMPLETE', flush=True)


if __name__ == '__main__':
    main()
