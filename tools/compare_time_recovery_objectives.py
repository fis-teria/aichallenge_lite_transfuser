"""Frozen 2x2 recovery sampling/objective comparison on the existing WSL corpus."""
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

import numpy as np
import torch
from torch.utils.data import Subset

from analyze_time_recovery_fit import groups, outward_ids, report_group
from compare_time_training_methods import pp_rows
from train_time_recovery_update import context as data_context, read_json, write
from aic_transfuser_lite.control.time_reference_v1 import TimePlan, TimedBodyPose
from aic_transfuser_lite.control.time_trial_v1 import time_trial_control
from aic_transfuser_lite.data.time_outward_balanced_v1 import OutwardBalancedMixDataset
from aic_transfuser_lite.data.time_split_v1 import content_sha256
from aic_transfuser_lite.data.time_training_cache_v1 import _sha
from aic_transfuser_lite.evaluation.time_batched_v1 import evaluate_time_batched
from aic_transfuser_lite.evaluation.time_metrics_v1 import time_horizon_metrics
from aic_transfuser_lite.evaluation.time_recovery_fit_v1 import predict_recovery_fit
from aic_transfuser_lite.training.time_checkpoint_v1 import TimeCheckpointIdentity, load_time_checkpoint
from aic_transfuser_lite.training.time_config_v1 import TimeModelConfig, build_time_model
from aic_transfuser_lite.training.time_corpus_runner_v1 import CorpusTrainingPlan, run_training_arm
from aic_transfuser_lite.training.time_recovery_geometry_v1 import (
    RecoveryGeometryObjective, RecoveryGeometryPlan, fixed_time_pp_angle)


def default_runner_parity(old: str, current: str) -> None:
    """Remove exactly the reviewed opt-in hook, then require historical AST equality."""
    tree = ast.parse(current)
    counts: Counter[str] = Counter()
    guarded = [ast.dump(ast.parse(s).body[0]) for s in (
        'if recovery_objective is not None:\n result["auxiliary_loss_sum_m"] = 0.0\n recovery_objective.validate_samples(samples)',
        'if recovery_objective is not None:\n auxiliary = recovery_objective(prediction.float(), batch.targets.trajectory_xy_m, batch.targets.trajectory_mask, eligible)\n result["auxiliary_loss_sum_m"] = float(auxiliary.detach().cpu())\n objective_sum = objective_sum + auxiliary',
        'if recovery_objective is not None:\n frozen_plan["recovery_objective"] = recovery_objective.identity',
        'if recovery_objective is not None:\n state["epoch_totals"]["auxiliary_loss_sum_m"] = 0.0',
    )]
    remaining = set(guarded)

    class Normalize(ast.NodeTransformer):
        def visit_ImportFrom(self, node: ast.ImportFrom) -> ast.AST | None:
            if ast.dump(node) == ast.dump(ast.parse('from .time_recovery_geometry_v1 import RecoveryGeometryObjective').body[0]):
                counts['import'] += 1
                return None
            return node

        def visit_FunctionDef(self, node: ast.FunctionDef) -> ast.AST:
            if node.name in {'run_training_arm', 'train_corpus_batch'}:
                at = next(i for i, arg in enumerate(node.args.kwonlyargs) if arg.arg == 'recovery_objective')
                if ast.dump(node.args.kw_defaults[at]) != ast.dump(ast.Constant(None)):
                    raise ValueError('auxiliary default must be None')
                node.args.kwonlyargs.pop(at)
                node.args.kw_defaults.pop(at)
                counts['parameter'] += 1
            return self.generic_visit(node)

        def visit_If(self, node: ast.If) -> ast.AST | None:
            key = ast.dump(node)
            if key in remaining:
                remaining.remove(key)
                return None
            return self.generic_visit(node)

        def visit_Assign(self, node: ast.Assign) -> ast.AST | None:
            if ast.dump(node) == ast.dump(ast.parse('objective_sum = summed').body[0]):
                counts['assignment'] += 1
                return None
            return self.generic_visit(node)

        def visit_Name(self, node: ast.Name) -> ast.AST:
            if node.id == 'objective_sum':
                counts['loss_name'] += 1
                node.id = 'summed'
            return node

        def visit_Call(self, node: ast.Call) -> ast.AST:
            if isinstance(node.func, ast.Name) and node.func.id == 'train_corpus_batch':
                kw = next(k for k in node.keywords if k.arg == 'recovery_objective')
                if ast.dump(kw.value) != ast.dump(ast.Name('recovery_objective', ast.Load())):
                    raise ValueError('auxiliary forwarding changed')
                node.keywords.remove(kw)
                counts['forwarding'] += 1
            return self.generic_visit(node)

        def visit_Compare(self, node: ast.Compare) -> ast.AST:
            expected = ast.parse('key in ("loss_sum_m", "auxiliary_loss_sum_m")', mode='eval').body
            if ast.dump(node) == ast.dump(expected):
                counts['reset'] += 1
                return ast.parse('key == "loss_sum_m"', mode='eval').body
            return self.generic_visit(node)

    actual = Normalize().visit(tree)
    if (remaining or counts != {'import': 1, 'parameter': 2, 'assignment': 1, 'loss_name': 1, 'forwarding': 1, 'reset': 1}
            or ast.dump(actual) != ast.dump(ast.parse(old))):
        raise ValueError('historical runner differs outside the exact opt-in auxiliary hook')


def source_proof(repo: Path, plan: dict[str, Any]) -> dict[str, Any]:
    if subprocess.check_output(['git', 'status', '--porcelain'], cwd=repo, text=True).strip():
        raise ValueError('clean committed source required')
    runner = 'src/aic_transfuser_lite/training/time_corpus_runner_v1.py'
    module = 'src/aic_transfuser_lite/training/time_recovery_geometry_v1.py'
    historical = plan['baseline_source_commit']
    old = subprocess.check_output(['git', 'show', historical+':'+runner], cwd=repo, text=True)
    default_runner_parity(old, (repo/runner).read_text())
    paths = ['src/aic_transfuser_lite/models', 'src/aic_transfuser_lite/training',
        'src/aic_transfuser_lite/contracts', 'src/aic_transfuser_lite/control',
        'src/aic_transfuser_lite/data', 'src/aic_transfuser_lite/evaluation',
        'tools/train_time_recovery_update.py', 'tools/analyze_time_recovery_fit.py',
        'tools/compare_time_training_methods.py']
    changed = subprocess.check_output(['git', 'diff', '--name-only', historical, 'HEAD', '--', *paths],
                                      cwd=repo, text=True).splitlines()
    if sorted(changed) != sorted([runner, module]):
        raise ValueError('unexpected baseline dependency changes: '+str(changed))
    return dict(source_commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=repo, text=True).strip(),
        baseline_source_commit=historical, default_runner_ast_equal=True,
        only_dependency_changes=changed, torch_version=torch.__version__)


def context(root: Path, repo: Path, plan: dict[str, Any]) -> dict[str, Any]:
    if (plan['test_usage'] != 'sealed' or plan['new_data'] or plan['new_awsim_trials'] != 0
            or plan['comparison_validation_usage'] != 'post_selection_diagnostic_only_no_weight_or_epoch_tuning'):
        raise ValueError('fixed data, diagnostic-only comparison validation and sealed test required')
    proof = source_proof(repo, plan)
    data_plan = read_json(repo/plan['data_plan'])
    c = data_context(root, data_plan)
    if c['cache']['manifest_sha256'] != plan['cache_sha256']:
        raise ValueError('frozen corpus changed')
    full, uniform = c['full'], c['train']
    targets, collection_hashes = outward_ids(root, repo, data_plan)
    train_targets = sorted(targets & set(full.anchor_ids))
    recovery_ids = sorted(r for r in set(full.run_ids) if r.startswith('codex-time-recovery-'))
    balanced = OutwardBalancedMixDataset(uniform, recovery_ids, train_targets,
        target_fraction=plan['outward_fraction_within_recovery'], seed=data_plan['training']['seed'])
    if (len(train_targets) != plan['expected_outward_train_anchors']
            or len(balanced.audit['target_run_presentations']) != plan['expected_outward_train_runs']
            or balanced.audit['target_presentations'] != plan['expected_outward_presentations']):
        raise ValueError('outward population/budget changed')
    historical = root/data_plan['output']
    old_plan, old_result = read_json(historical/'plan.json'), read_json(historical/'result.json')
    old_proof = read_json(historical/'matched_budget_verification.json')
    if (_sha(historical/'best.pt') != plan['baseline_checkpoint_sha256']
            or old_proof['status'] != 'PASS' or old_result['status'] != 'COMPLETE'
            or old_proof['train_anchor_order_sha256'] != content_sha256(uniform.anchor_ids)
            or old_plan['torch_version'] != torch.__version__
            or any(old_plan[k] != v for k, v in asdict(CorpusTrainingPlan(**data_plan['training'])).items())
            or old_result['optimizer_steps'] != data_plan['expected']['optimizer_steps']
            or not old_result['reload_predictions_exact']
            or old_plan['initialization']['sha256'] != data_plan['initialization_sha256']
            or _sha(root/data_plan['initialization']) != data_plan['initialization_sha256']):
        raise ValueError('baseline initialization, source, sampler, training settings or completion changed')
    controller = {**read_json(repo/data_plan['offline_controller_config']),
                  'lookahead_policy': 'stopping_preview_extended_v1'}
    proof.update(data=c['data_proof'], cache_sha256=c['cache']['manifest_sha256'],
        collection_hashes=collection_hashes, experiment_plan=plan, data_plan_sha256=_sha(repo/plan['data_plan']),
        controller=controller, train_targets=train_targets,
        uniform_outward_presentations=sum(a in set(train_targets) for a in uniform.anchor_ids),
        balanced_audit=balanced.audit, uniform_anchor_order_sha256=content_sha256(uniform.anchor_ids),
        balanced_anchor_order_sha256=content_sha256(balanced.anchor_ids),
        historical_control_reused=True, reserved_test_raw_or_cache_read=False)
    return dict(**c, uniform=uniform, balanced=balanced, proof=proof, targets=targets,
                data_plan=data_plan, controller=controller, historical=historical)


def geometry_targets(c: dict[str, Any], plan: RecoveryGeometryPlan) -> list[dict[str, Any]]:
    ds, controller = c['full'], c['controller']
    indices = [i for i, rid in enumerate(ds.run_ids) if rid.startswith('codex-time-recovery-')]
    if not ds.input_valid[indices].all() or not ds.xy_mask[indices].all():
        raise ValueError('all recovery training inputs and teachers must be supported')
    rows = []
    for i in indices:
        run, local = ds._index[i]
        speed = float(ds._runs[run]['inputs']['ego'][local, -1, 0])
        pose = TimedBodyPose(ds._anchors[i]['observation_ns'], 'sim', '0', 'map', 'base_link', 0., 0., 0.)
        result = time_trial_control(TimePlan(ds.anchor_ids[i], pose, ds.targets[i]), pose, speed_mps=speed,
            rear_axle_offset_m=plan.rear_axle_offset_m, speed_policy=controller['speed_policy'],
            lookahead_policy=controller['lookahead_policy'], vehicle_model_policy=controller['vehicle_model_policy'])
        rows.append(dict(anchor_id=ds.anchor_ids[i], run_id=ds.run_ids[i],
            horizon_s=result['lookahead_selection']['observation_horizon_s'],
            response_length_m=result['nominal_response_length_m'], actual_teacher_steer_rad=result['steer_rad']))
    surrogate = fixed_time_pp_angle(torch.from_numpy(ds.targets[indices]).double(),
        torch.tensor([r['horizon_s'] for r in rows], dtype=torch.float64),
        torch.tensor([r['response_length_m'] for r in rows], dtype=torch.float64), plan.rear_axle_offset_m)
    error = np.abs(surrogate.numpy() - np.array([r['actual_teacher_steer_rad'] for r in rows]))
    if float(error.max()) > 1e-6:
        raise ValueError('teacher fixed-time geometry does not match actual PP')
    c['proof']['teacher_geometry_parity_max_abs_rad'] = float(error.max())
    c['proof']['teacher_geometry_count'] = len(rows)
    return rows


def make_objective(c: dict[str, Any], plan: dict[str, Any], rows: list[dict[str, Any]]) -> RecoveryGeometryObjective:
    geometry = dict(plan['geometry'])
    geometry['rear_axle_offset_m'] = tuple(geometry['rear_axle_offset_m'])
    return RecoveryGeometryObjective(rows, split_manifest=c['cache']['split_manifest'],
        plan=RecoveryGeometryPlan(**geometry), controller=c['controller'])


def train(root: Path, repo: Path, plan: dict[str, Any], arm: str, *, resume: bool) -> None:
    if plan['arms'][arm]['reuse_baseline']:
        raise ValueError('baseline is reused, not retrained')
    c = context(root, repo, plan)
    out = root/plan['output']
    prepared = read_json(out/'data_and_budget_verification.json')
    # The saved proof additionally contains geometry parity, prepared once.
    if any(prepared.get(k) != json.loads(json.dumps(v)) for k, v in c['proof'].items()):
        raise ValueError('prepared experiment differs from current source/data')
    rows = read_json(out/'recovery_geometry_targets.json')
    objective = make_objective(c, plan, rows)
    if objective.identity != read_json(out/'geometry_identity.json'):
        raise ValueError('frozen auxiliary targets changed')
    if plan['arms'][arm]['objective'] == 'l1':
        objective = None
    sampler = c[plan['arms'][arm]['sampling']]
    dp, val, selected = c['data_plan'], c['validation'], c['selection']
    payload = torch.load(root/dp['initialization'], map_location='cpu', weights_only=False)
    config = TimeModelConfig.from_dict(payload['config'])
    source_identity = TimeCheckpointIdentity(**payload['identity'])
    split = c['cache']['split_manifest']
    if config.use_command_history or config.dataset_config() != c['full'].config:
        raise ValueError('input/model contract changed')
    teacher = dict(format='recovery_sampling_geometry_comparison_v1', experiment=plan, arm=arm,
        cache_sha256=plan['cache_sha256'], source=c['proof']['source_commit'],
        contract=c['cache']['contract'],
        train_anchor_order_sha256=content_sha256(sampler.anchor_ids),
        selection_anchor_order_sha256=content_sha256([val.anchor_ids[i] for i in selected]),
        auxiliary_identity=objective.identity if objective else None,
        preparation_sha256=_sha(out/'data_and_budget_verification.json'))
    teacher['manifest_sha256'] = content_sha256(teacher)
    identity = TimeCheckpointIdentity(split['manifest_sha256'], teacher['manifest_sha256'],
        content_sha256([r for r in split['runs'] if r['split'] == 'train']), arm, c['proof']['source_commit'])
    result = run_training_arm(sampler, Subset(val, selected), train_run_ids=sampler.run_ids,
        validation_run_ids=[val.run_ids[i] for i in selected], split_manifest=split,
        teacher_manifest=teacher, identity=identity, config=config, plan=CorpusTrainingPlan(**dp['training']),
        output=out/arm, resume=resume, initialization=root/dp['initialization'],
        initialization_sha256=dp['initialization_sha256'], initialization_identity=source_identity,
        retain_epoch_checkpoints=True, recovery_objective=objective)
    old = read_json(c['historical']/'result.json')
    if (result['optimizer_steps'] != old['optimizer_steps'] or result['anchors_visited'] != old['anchors_visited']
            or result['initial_weights_sha256'] != old['initial_weights_sha256']
            or read_json(out/arm/'initial_validation.json') != read_json(c['historical']/'initial_validation.json')):
        raise ValueError('matched budget/initial validation verification failed')
    write(out/arm/'comparison_verification.json', dict(status='PASS', arm=arm,
        best_checkpoint_sha256=_sha(out/arm/'best.pt'), initialization_validation_exact=True,
        initial_weights_match=True, optimizer_steps=result['optimizer_steps'],
        anchors_visited=result['anchors_visited'], reload_predictions_exact=result['reload_predictions_exact'],
        preparation_sha256=_sha(out/'data_and_budget_verification.json')))


def compare(root: Path, repo: Path, plan: dict[str, Any]) -> None:
    c = context(root, repo, plan)
    out = root/plan['output']/'comparison'
    out.mkdir(parents=True, exist_ok=False)
    datasets = {'train': c['full'], 'validation': c['validation']}
    data = {}
    for split, ds in datasets.items():
        ix = [i for i, rid in enumerate(ds.run_ids) if rid.startswith('codex-time-recovery-')]
        data[split] = dict(indices=ix, targets=ds.targets[ix], anchor_ids=[ds.anchor_ids[i] for i in ix],
            run_ids=[ds.run_ids[i] for i in ix], groups=groups(ds, ix, c['targets']),
            teacher_pp=pp_rows(ds, ix, ds.targets[ix], c['controller'], teacher=True))
    summary = dict(status='RUNNING', experiment_plan=plan, source=c['proof']['source_commit'],
        scope='OFFLINE_FIXED_DATA_SINGLE_SEED_NOT_CLOSED_LOOP', test_evaluated=False,
        reserved_test_raw_or_cache_read=False, controller=c['controller'], controller_age_s=0., arms={})
    predictions = {}
    for arm, spec in plan['arms'].items():
        directory = c['historical'] if spec['reuse_baseline'] else root/plan['output']/arm
        result = read_json(directory/'result.json')
        if result['status'] != 'COMPLETE' or not result['reload_predictions_exact']:
            raise ValueError('complete reload-verified arm required')
        checkpoint = directory/'best.pt'
        digest = _sha(checkpoint)
        expected = plan['baseline_checkpoint_sha256'] if spec['reuse_baseline'] else read_json(directory/'comparison_verification.json')['best_checkpoint_sha256']
        if digest != expected:
            raise ValueError('selected checkpoint changed')
        payload = torch.load(checkpoint, map_location='cpu', weights_only=False)
        config, identity = TimeModelConfig.from_dict(payload['config']), TimeCheckpointIdentity(**payload['identity'])
        model = build_time_model(config).to('cuda')
        load_time_checkpoint(checkpoint, config=config, identity=identity, model=model, mode='finetune')
        entry = dict(checkpoint=str(checkpoint), checkpoint_sha256=digest, best_epoch=result['best_epoch'],
            history=read_json(directory/'history.json'), selection_metrics=read_json(directory/'best_validation.json'))
        summary['arms'][arm] = entry
        predictions[arm] = {}
        for split, d in data.items():
            ds = datasets[split]
            tensor = predict_recovery_fit(model, Subset(ds, d['indices']), run_ids=d['run_ids'],
                anchor_ids=d['anchor_ids'], split_manifest=ds.split_manifest, split=split, batch_size=32, workers=4)
            values = tensor.numpy()
            np.save(out/f'{arm}_{split}_predictions.npy', values, allow_pickle=False)
            predictions[arm][split] = values
            probes = pp_rows(ds, d['indices'], values, c['controller'])
            entry[split] = {}
            for group, ix in d['groups'].items():
                entry[split][group] = report_group(values[ix], d['targets'][ix], [d['run_ids'][i] for i in ix],
                    [d['teacher_pp'][i] for i in ix], [probes[i] for i in ix])
            write(out/f'{arm}_{split}_rows.json', [dict(anchor_id=aid, run_id=d['run_ids'][i],
                strict_outward=i in d['groups']['strict_outward'], teacher_pp=d['teacher_pp'][i], prediction_pp=probes[i],
                endpoint_error_m=float(np.linalg.norm(values[i, -1]-d['targets'][i, -1])),
                endpoint_left_error_m=float(values[i, -1, 1]-d['targets'][i, -1, 1])) for i, aid in enumerate(d['anchor_ids'])])
        selected = c['selection']
        saved = np.load(directory/f'validation_epoch_{result["best_epoch"]:02d}.npy', allow_pickle=False)
        ds = c['validation']
        normal = [j for j, i in enumerate(selected) if not ds.run_ids[i].startswith('codex-time-recovery-')]
        source_indices = [selected[j] for j in normal]
        entry['normal_validation'] = time_horizon_metrics(torch.from_numpy(saved[normal]),
            torch.from_numpy(ds.targets[source_indices]), torch.from_numpy(ds.xy_mask[source_indices]),
            input_valid=torch.from_numpy(ds.input_valid[source_indices]), run_ids=[ds.run_ids[i] for i in source_indices])
        recovery_lookup = {i: j for j, i in enumerate(data['validation']['indices'])}
        pairs = [(j, recovery_lookup[i]) for j, i in enumerate(selected) if i in recovery_lookup]
        earlier = saved[[p[0] for p in pairs]]
        fresh = predictions[arm]['validation'][[p[1] for p in pairs]]
        np.testing.assert_allclose(earlier, fresh, rtol=1e-5, atol=2e-6)
        entry['archived_selection_recovery_prediction_max_abs_m'] = float(np.abs(earlier-fresh).max())
        entry['selection_predictions_sha256'] = _sha(directory/f'validation_epoch_{result["best_epoch"]:02d}.npy')
        if _sha(checkpoint) != digest:
            raise ValueError('checkpoint changed during read-only evaluation')
        del model, payload
        torch.cuda.empty_cache()
        write(out/'progress.json', summary)
        print('COMPARISON_ARM_COMPLETE', arm, 'validation_outward_3s_m',
              entry['validation']['strict_outward']['xy']['run_macro_mean']['3s']['raw_error_m'], flush=True)
    summary['status'] = 'COMPLETE'
    summary['data_counts'] = {s: dict(anchors=len(d['indices']), runs=len(set(d['run_ids'])),
        strict_outward=len(d['groups']['strict_outward']), anchor_order_sha256=content_sha256(d['anchor_ids'])) for s, d in data.items()}
    write(out/'summary.json', summary)
    plot(out, summary)
    write(out/'artifact_manifest.json', {p.name: dict(bytes=p.stat().st_size, sha256=_sha(p))
                                       for p in sorted(out.iterdir()) if p.is_file()})


def plot(out: Path, summary: dict[str, Any]) -> None:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    names = list(summary['arms'])
    labels = ['Uniform / L1', 'Balanced / L1', 'Uniform / geometry', 'Balanced / geometry']
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    for column, metric in enumerate(('3s position error [cm]', 'PP angle error [rad]')):
        for row, split in enumerate(('train', 'validation')):
            ax = axes[row, column]
            for offset, group, color in ((-.18, 'all', '#1769aa'), (.18, 'strict_outward', '#c77420')):
                records = [summary['arms'][name][split][group] for name in names]
                values = [r['xy']['run_macro_mean']['3s']['raw_error_m']*100 if column == 0
                          else r['pp']['run_macro_penalized_rad'] for r in records]
                bars = ax.bar(np.arange(4)+offset, values, .35, color=color,
                              label=f'{group} ({records[0]["anchor_count"]} anchors)')
                ax.bar_label(bars, fmt='%.2f' if column == 0 else '%.4f', fontsize=7, padding=3)
            ax.set_xticks(range(4), labels, rotation=12, fontsize=8)
            ax.set_ylabel(metric+'; run-equal mean')
            ax.set_title(split)
            ax.set_ylim(0, ax.get_ylim()[1]*1.18)
            ax.grid(axis='y', alpha=.2)
            ax.legend(fontsize=8)
    fig.suptitle('Existing recovery data: fixed budget, one seed, original validation checkpoint rule\nTeacher-relative offline prediction; no new closed-loop driving')
    fig.savefig(out/'recovery_objective_comparison.png', dpi=160)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('prepare', 'train', 'compare'))
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--root', type=Path, default=Path('..'))
    parser.add_argument('--arm', choices=('balanced_l1', 'uniform_geometry', 'balanced_geometry'))
    parser.add_argument('--resume', action='store_true')
    args = parser.parse_args()
    repo, root, plan = Path(__file__).resolve().parents[1], args.root.resolve(), read_json(args.plan)
    if root.as_posix().startswith('/mnt/') or not torch.cuda.is_available():
        raise ValueError('native WSL data and CUDA required')
    torch.set_num_threads(4)
    torch.backends.cuda.matmul.allow_tf32 = False
    torch.backends.cudnn.allow_tf32 = False
    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.deterministic = True
    soft, hard = resource.getrlimit(resource.RLIMIT_NOFILE)
    resource.setrlimit(resource.RLIMIT_NOFILE, (max(soft, min(hard, 8192)), hard))
    if args.command == 'prepare':
        c = context(root, repo, plan)
        geometry = dict(plan['geometry'])
        geometry['rear_axle_offset_m'] = tuple(geometry['rear_axle_offset_m'])
        rows = geometry_targets(c, RecoveryGeometryPlan(**geometry))
        objective = make_objective(c, plan, rows)
        # Fail before any training if same-weight initial inference cannot replay.
        dp = c['data_plan']
        payload = torch.load(root/dp['initialization'], map_location='cpu', weights_only=False)
        config, identity = TimeModelConfig.from_dict(payload['config']), TimeCheckpointIdentity(**payload['identity'])
        model = build_time_model(config).to('cuda')
        load_time_checkpoint(root/dp['initialization'], config=config, identity=identity, model=model, mode='finetune')
        selected, val = c['selection'], c['validation']
        metrics, _ = evaluate_time_batched(model, Subset(val, selected), run_ids=[val.run_ids[i] for i in selected],
            split_manifest=c['cache']['split_manifest'], batch_size=32, workers=4, precision='float32')
        if metrics != read_json(c['historical']/'initial_validation.json'):
            raise ValueError('same initialization validation did not exactly replay')
        c['proof']['initial_validation_exact'] = True
        out = root/plan['output']
        out.mkdir(parents=True, exist_ok=False)
        write(out/'data_and_budget_verification.json', c['proof'])
        write(out/'recovery_geometry_targets.json', rows)
        # JSON normalization ensures exact resume identity equality.
        write(out/'geometry_identity.json', objective.identity)
        print('PREPARED_PARITY_AND_BUDGET_VERIFIED', flush=True)
    elif args.command == 'train':
        if args.arm is None:
            parser.error('--arm is required for training')
        train(root, repo, plan, args.arm, resume=args.resume)
    else:
        compare(root, repo, plan)


if __name__ == '__main__':
    main()
