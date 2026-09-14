"""Diagnose frozen recovery train fit and run-separated validation in native WSL."""
from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
import resource
from typing import Any

import numpy as np
import torch
from torch.utils.data import Subset

from compare_time_training_methods import pp_rows
from train_time_recovery_update import read_json, reference_proof, write
from aic_transfuser_lite.data.time_split_v1 import content_sha256
from aic_transfuser_lite.data.time_training_cache_v1 import TimeTrainingCacheDataset, verify_time_training_cache, _sha
from aic_transfuser_lite.evaluation.time_metrics_v1 import time_horizon_metrics
from aic_transfuser_lite.evaluation.time_method_selection_v1 import pp_agreement_score
from aic_transfuser_lite.evaluation.time_recovery_comparison_v1 import component_errors, summarize_pp
from aic_transfuser_lite.evaluation.time_recovery_fit_v1 import predict_recovery_fit
from aic_transfuser_lite.training.time_checkpoint_v1 import TimeCheckpointIdentity, load_time_checkpoint
from aic_transfuser_lite.training.time_config_v1 import TimeModelConfig, build_time_model


def outward_ids(root: Path, repo: Path, plan: dict[str, Any]) -> tuple[set[str], dict[str, str]]:
    """Use collection-time assignments, never select subsets from model errors."""
    parent = read_json(root/plan['parent_cache']/'identity.json')
    if parent['manifest_sha256'] != plan['parent_cache_sha256']:
        raise ValueError('parent identity changed')
    files = {root/parent['plan']['collection_index']: parent['plan']['collection_index_sha256']}
    for spec in plan['additions']:
        if spec['event_ids']:
            files[root/spec['analysis']/spec['proof']] = spec['proof_sha256']
    targets: set[str] = set()
    hashes = {}
    for path, expected in files.items():
        if _sha(path) != expected:
            raise ValueError('frozen collection changed: '+str(path))
        hashes[str(path)] = expected
        collection = read_json(path)
        for row in collection.get('production_runs', collection.get('runs', [])):
            if row['split'] not in {'train', 'validation'}:
                continue
            targets.update(row.get('target_anchor_ids', []))
            for event in row.get('events', []):
                targets.update(event['target_anchor_ids'])
    early = repo/'docs/evidence/time_recovery_phase_expansion_20260914/pair01_20260914_summary.json'
    hashes[str(early)] = _sha(early)
    for row in read_json(early)['runs']:
        if row['run_id'] not in {s['run_id'] for s in plan['additions'] if s['split'] == 'train'}:
            raise ValueError('early-phase run not in frozen training population')
        targets.update(row['target_anchor_ids_both_nominals'])
    return targets, hashes


def groups(dataset: TimeTrainingCacheDataset, indices: list[int], targets: set[str]) -> dict[str, list[int]]:
    """Indices here are local to the unique recovery-only diagnostic sequence."""
    rows = [dataset._anchors[i] for i in indices]
    result = {'all': list(range(len(rows))),
              'strict_outward': [j for j, row in enumerate(rows) if row['anchor_id'] in targets]}
    for family, token in [('original', None), ('pulse', '-pulse'), ('early', '-early'), ('random', '-random')]:
        chosen = [j for j, row in enumerate(rows) if
                  (token in row['run_id'] if token else not any(t in row['run_id'] for t in ('-pulse', '-early', '-random')))]
        if chosen:
            result[family] = chosen
            strict = [j for j in chosen if rows[j]['anchor_id'] in targets]
            if strict:
                result[family+'_outward'] = strict
    for run in sorted({row['run_id'] for row in rows}):
        result['run:'+run] = [j for j, row in enumerate(rows) if row['run_id'] == run]
    for run, event in sorted({(row['run_id'], row['recovery_event_id']) for row in rows if 'recovery_event_id' in row}):
        result[f'event:{run}:{event}'] = [j for j, row in enumerate(rows)
                                       if row['run_id'] == run and row.get('recovery_event_id') == event]
    return result


def report_group(prediction: np.ndarray, target: np.ndarray, ids: list[str],
                 teacher_pp: list[dict[str, Any]], model_pp: list[dict[str, Any]]) -> dict[str, Any]:
    n = len(ids)
    mask, valid = np.ones((n, 30), bool), np.ones(n, bool)
    errors = np.linalg.norm(prediction-target, axis=-1)
    return dict(anchor_count=n, run_count=len(set(ids)),
        xy=time_horizon_metrics(torch.from_numpy(prediction), torch.from_numpy(target),
                               torch.from_numpy(mask), input_valid=torch.from_numpy(valid), run_ids=ids),
        components=component_errors(prediction, target, mask, valid, ids),
        endpoint_pooled_cm={str(p): float(np.percentile(errors[:, -1]*100, p)) for p in (50, 95, 100)},
        pp=pp_agreement_score(teacher_pp, model_pp, ids),
        teacher_pp_applicability=summarize_pp(teacher_pp), pp_applicability=summarize_pp(model_pp))


def plot_results(out: Path, summary: dict[str, Any], data: dict[str, Any], predictions: dict[str, Any]) -> None:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    stages = ['initial', 'epoch_01', 'epoch_02', 'selected_epoch_03']
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5), constrained_layout=True)
    for split, color in [('train', '#1769aa'), ('validation', '#c77420')]:
        for group, style in [('all', '-'), ('strict_outward', '--')]:
            rows = [summary['models'][s][split]['groups'][group] for s in stages]
            label = f'{split} / {group} ({rows[0]["anchor_count"]} anchors)'
            axes[0].plot(range(4), [r['xy']['run_macro_mean']['3s']['raw_error_m']*100 for r in rows],
                         style+'o', color=color, label=label)
            axes[1].plot(range(4), [r['pp']['run_macro_penalized_rad'] for r in rows], style+'o', color=color, label=label)
    for ax in axes:
        ax.set_xticks(range(4), ['Before', 'Epoch 1', 'Epoch 2', 'Epoch 3'])
        ax.set_ylim(bottom=0)
        ax.grid(alpha=.25)
        ax.legend(fontsize=7)
    axes[0].set_ylabel('3 s position error [cm], run-equal mean')
    axes[1].set_ylabel('Teacher PP angle error [rad], run-equal mean')
    fig.suptitle('Frozen recovery fit: recorded examples, no training or driving\nTrain and validation states differ; frames within runs are correlated')
    fig.savefig(out/'recovery_fit_progress.png', dpi=160)
    plt.close(fig)
    fig, axes = plt.subplots(2, 3, figsize=(12, 7), constrained_layout=True)
    chosen = []
    for rowno, split in enumerate(('train', 'validation')):
        context = data[split]
        for event in (1, 2, 3):
            j = next(j for j, row in enumerate(context['anchors'])
                     if '-random-' in row['run_id'] and row.get('recovery_event_id') == event
                     and j in context['groups']['strict_outward'])
            anchor = context['anchors'][j]
            chosen.append({'split': split, 'anchor_id': anchor['anchor_id']})
            ax = axes[rowno, event-1]
            teacher = context['targets'][j]
            ax.plot(teacher[:, 0], teacher[:, 1], 'k-', label='Observed teacher')
            for name, color in [('initial', '#c77420'), ('selected_epoch_03', '#1769aa')]:
                xy = predictions[name][split][j]
                ax.plot(xy[:, 0], xy[:, 1], '--', color=color, label=name)
            ax.set(title=f'{split}, random event {event}', xlabel='Forward [m]', ylabel='Left [m]')
            ax.grid(alpha=.25)
            ax.legend(fontsize=7)
    fig.suptitle('First pre-audited outward anchor per random event\nObservation body coordinates; lateral scale enlarged; these are predictions, not driven paths')
    fig.savefig(out/'recovery_fit_examples.png', dpi=160)
    plt.close(fig)
    write(out/'plot_manifest.json', dict(selection='first_collection_audited_outward_anchor_per_random_event',
        anchors=chosen, files={p.name: _sha(p) for p in sorted(out.glob('*.png'))}))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--root', type=Path, default=Path('..'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root, repo, out = args.root.resolve(), Path(__file__).resolve().parents[1], args.output.resolve()
    if root.as_posix().startswith('/mnt/') or out.as_posix().startswith('/mnt/'):
        raise ValueError('native WSL data and output required')
    plan = read_json(args.plan)
    torch.set_num_threads(4)
    torch.backends.cuda.matmul.allow_tf32 = False
    torch.backends.cudnn.allow_tf32 = False
    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.deterministic = True
    soft, hard = resource.getrlimit(resource.RLIMIT_NOFILE)
    resource.setrlimit(resource.RLIMIT_NOFILE, (max(soft, min(hard, 8192)), hard))
    if not torch.cuda.is_available():
        raise RuntimeError('native WSL CUDA required')
    proof = reference_proof(root, repo, plan)
    training = root/plan['output']
    verification = read_json(training/'matched_budget_verification.json')
    if verification['status'] != 'PASS' or not verification['reload_predictions_exact']:
        raise ValueError('verified completed training required')
    cache = verify_time_training_cache(root/plan['cache'])
    if cache['plan'] != plan:
        raise ValueError('frozen recovery dataset changed')
    target_ids, collection_hashes = outward_ids(root, repo, plan)
    datasets, data = {}, {}
    controller = {**read_json(repo/plan['offline_controller_config']), 'lookahead_policy': 'stopping_preview_extended_v1'}
    for split in ('train', 'validation'):
        ds = TimeTrainingCacheDataset(root/plan['cache'], split, verify_hashes=False)
        ix = [i for i, rid in enumerate(ds.run_ids) if rid.startswith('codex-time-recovery-')]
        if not ds.input_valid[ix].all() or not ds.xy_mask[ix].all() or not np.isfinite(ds.targets[ix]).all():
            raise ValueError('recovery fit requires all audited inputs and 30 teacher points')
        datasets[split] = ds
        data[split] = dict(indices=ix, targets=ds.targets[ix], run_ids=[ds.run_ids[i] for i in ix],
            anchor_ids=[ds.anchor_ids[i] for i in ix], anchors=[ds._anchors[i] for i in ix],
            groups=groups(ds, ix, target_ids), teacher_pp=pp_rows(ds, ix, ds.targets[ix], controller, teacher=True))
    if (len(data['train']['indices']) != 1410 or len(data['validation']['indices']) != 538
            or len(data['train']['groups']['strict_outward']) != 35
            or len(data['validation']['groups']['strict_outward']) != 12
            or set(data['train']['run_ids']) & set(data['validation']['run_ids'])):
        raise ValueError('frozen recovery populations changed')
    assigned = {d['anchor_ids'][i] for d in data.values() for i in d['groups']['strict_outward']}
    if assigned != target_ids:
        raise ValueError('collection outward IDs missing from train/validation cache')
    checkpoints = {'initial': root/plan['initialization'], 'epoch_01': training/'epoch_01.pt',
                   'epoch_02': training/'epoch_02.pt', 'selected_epoch_03': training/'best.pt'}
    checkpoint_hashes = {name: _sha(path) for name, path in checkpoints.items()}
    if (checkpoint_hashes['initial'] != plan['initialization_sha256']
            or checkpoint_hashes['selected_epoch_03'] != verification['best_checkpoint_sha256']
            or checkpoint_hashes['selected_epoch_03'] != '53e1962b97cfaa47acae3e2ad4687fac96c80672905406fdbe1514abd9563da2'):
        raise ValueError('pinned initialization/current checkpoint changed')
    audit = {split: dict(anchor_count=len(d['indices']), runs=dict(Counter(d['run_ids'])),
        group_counts={k: len(v) for k, v in d['groups'].items()},
        anchor_order_sha256=content_sha256(d['anchor_ids']), all_inputs_and_30_point_teachers_supported=True)
        for split, d in data.items()}
    out.mkdir(parents=True, exist_ok=False)
    frozen = dict(source=proof, cache_sha256=cache['manifest_sha256'], collection_hashes=collection_hashes,
        checkpoints={n: dict(path=str(p), sha256=checkpoint_hashes[n]) for n, p in checkpoints.items()},
        data_audit=audit, batch_size=32, workers=4, precision='float32', controller=controller, controller_age_s=0.,
        optimizer_updates=0, new_awsim_trials=0, reserved_test_raw_or_cache_read=False,
        selection_validation_runs=plan['selection_run_ids'], validation_used_for_new_selection=False)
    write(out/'diagnostic_plan.json', frozen)
    print('RECOVERY_FIT_AUDIT_PASS', json.dumps(audit), flush=True)
    summary = dict(status='RUNNING', scope='FROZEN_TRAIN_FIT_AND_RUN_VALIDATION_NOT_CLOSED_LOOP', **frozen, models={})
    predictions = {}
    for name, checkpoint in checkpoints.items():
        payload = torch.load(checkpoint, map_location='cpu', weights_only=False)
        config = TimeModelConfig.from_dict(payload['config'])
        identity = TimeCheckpointIdentity(**payload['identity'])
        if config.use_command_history or config.dataset_config() != datasets['train'].config:
            raise ValueError('model/input contract changed')
        if name != 'initial' and (identity.split_manifest_sha256 != cache['split_manifest']['manifest_sha256']
                or payload['epoch'] != {'epoch_01': 1, 'epoch_02': 2, 'selected_epoch_03': 3}[name]):
            raise ValueError('epoch lineage or split mismatch')
        model = build_time_model(config).to('cuda')
        load_time_checkpoint(checkpoint, config=config, identity=identity, model=model, mode='finetune')
        summary['models'][name] = dict(epoch=payload['epoch'], checkpoint_sha256=checkpoint_hashes[name])
        predictions[name] = {}
        for split, d in data.items():
            tensor = predict_recovery_fit(model, Subset(datasets[split], d['indices']), run_ids=d['run_ids'],
                anchor_ids=d['anchor_ids'], split_manifest=cache['split_manifest'], split=split, batch_size=32, workers=4)
            values = tensor.numpy()
            predictions[name][split] = values
            np.save(out/f'{name}_{split}_predictions.npy', values, allow_pickle=False)
            probes = pp_rows(datasets[split], d['indices'], values, controller)
            results = {}
            for group, ix in d['groups'].items():
                results[group] = report_group(values[ix], d['targets'][ix], [d['run_ids'][i] for i in ix],
                    [d['teacher_pp'][i] for i in ix], [probes[i] for i in ix])
            rows = [dict(anchor_id=aid, run_id=d['run_ids'][i], observation_ns=d['anchors'][i]['observation_ns'],
                         strict_outward=i in d['groups']['strict_outward'], teacher_pp=d['teacher_pp'][i], prediction_pp=probes[i],
                         endpoint_error_m=float(np.linalg.norm(values[i, -1]-d['targets'][i, -1])))
                    for i, aid in enumerate(d['anchor_ids'])]
            write(out/f'{name}_{split}_rows.json', rows)
            summary['models'][name][split] = dict(groups=results)
            print('RECOVERY_FIT_STAGE', name, split, '3s_m', results['all']['xy']['run_macro_mean']['3s']['raw_error_m'],
                  'pp_rad', results['all']['pp']['run_macro_penalized_rad'], flush=True)
        del model, payload
        torch.cuda.empty_cache()
        write(out/'progress.json', summary)
    # Existing validation inference used different batch boundaries. A tight
    # floating-point tolerance checks the unchanged preprocessing/weights route.
    saved = training/'comparison/after_predictions.npy'
    earlier = np.load(saved, allow_pickle=False)[data['validation']['indices']]
    fresh = predictions['selected_epoch_03']['validation']
    np.testing.assert_allclose(fresh, earlier, rtol=1e-5, atol=2e-6)
    summary['selected_validation_replay'] = dict(max_abs_difference_m=float(np.max(np.abs(fresh-earlier))),
        prior_predictions_sha256=_sha(saved), rtol=1e-5, atol_m=2e-6, batch_boundaries_differ=True)
    if any(_sha(p) != checkpoint_hashes[n] for n, p in checkpoints.items()):
        raise ValueError('checkpoint changed during read-only diagnostic')
    summary['status'] = 'COMPLETE'
    write(out/'summary.json', summary)
    plot_results(out, summary, data, predictions)
    write(out/'artifact_manifest.json', {'files': [dict(path=p.name, bytes=p.stat().st_size, sha256=_sha(p))
        for p in sorted(out.iterdir()) if p.is_file()]})
    print('RECOVERY_FIT_COMPLETE', flush=True)


if __name__ == '__main__':
    main()
