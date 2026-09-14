"""Compare old/new checkpoints on identical held-out inputs in native WSL."""
from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import resource
import sqlite3
import subprocess

import numpy as np
import torch
from torch.utils.data import Subset

from aic_transfuser_lite.control.time_reference_v1 import TimedBodyPose
from aic_transfuser_lite.control.time_trial_v1 import validate_trial_config
from aic_transfuser_lite.data.time_training_cache_v1 import TimeTrainingCacheDataset, verify_time_training_cache, _sha
from aic_transfuser_lite.data.time_sqlite_reader_v1 import _store
from aic_transfuser_lite.evaluation.time_batched_v1 import evaluate_time_batched
from aic_transfuser_lite.evaluation.time_clearance_v1 import PoseIndex
from aic_transfuser_lite.evaluation.time_corner_comparison_v1 import replay_observation_pose, world_points
from aic_transfuser_lite.evaluation.time_metrics_v1 import time_horizon_metrics
from aic_transfuser_lite.evaluation.time_recovery_comparison_v1 import pp_probe, summarize_pp
from aic_transfuser_lite.training.time_checkpoint_v1 import TimeCheckpointIdentity, load_time_checkpoint
from aic_transfuser_lite.training.time_config_v1 import TimeModelConfig, build_time_model
from train_time_recovery_expansion import check_reference, read


def write(path: Path, data: dict) -> None:
    with path.open('x') as stream:
        json.dump(data, stream, indent=2, allow_nan=False)


def motion(run: Path) -> tuple[PoseIndex, dict, np.ndarray, np.ndarray]:
    """Read recorded poses/velocity; future states are evaluator-only information."""
    store = _store(run)
    dbs = list((run/'bag').glob('*.db3'))
    if len(dbs) != 1:
        raise ValueError('one closed bag required')
    poses, pose_rows, velocities = [], {}, {}
    with sqlite3.connect(dbs[0].resolve().as_uri()+'?mode=ro&immutable=1', uri=True) as con:
        for name in ('/localization/kinematic_state', '/vehicle/status/velocity_status'):
            topic, kind = con.execute('select id,type from topics where name=?', (name,)).fetchone()
            for rid, receipt, blob in con.execute('select id,timestamp,data from messages where topic_id=? order by id', (topic,)):
                msg = store.deserialize_cdr(blob, kind)
                header = msg.header.stamp if hasattr(msg, 'header') else msg.stamp
                t = int(header.sec)*10**9 + int(header.nanosec)
                if name.endswith('kinematic_state'):
                    if kind != 'nav_msgs/msg/Odometry':
                        raise ValueError('pose type changed')
                    p, q = msg.pose.pose.position, msg.pose.pose.orientation
                    pose = TimedBodyPose(t, 'sim', '0', msg.header.frame_id, msg.child_frame_id,
                        p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
                    poses.append(pose); pose_rows[rid] = pose, receipt
                else:
                    v = float(msg.longitudinal_velocity)
                    if t in velocities and velocities[t] != v:
                        raise ValueError('ambiguous recorded velocity')
                    velocities[t] = v
    stamps = np.asarray(sorted(velocities), dtype=np.int64)
    return PoseIndex(poses), pose_rows, stamps, np.asarray([velocities[t] for t in stamps])


def speed_at(stamps: np.ndarray, values: np.ndarray, target: int) -> float:
    index = int(np.searchsorted(stamps, target))
    if index < len(stamps) and stamps[index] == target:
        return float(values[index])
    if index == 0 or index == len(stamps) or max(target-stamps[index-1], stamps[index]-target) > 50_000_000:
        raise ValueError('observed velocity interpolation unsupported')
    fraction = (target-stamps[index-1])/(stamps[index]-stamps[index-1])
    return float(values[index-1]+fraction*(values[index]-values[index-1]))


def plot(out: Path, report: dict, dataset: TimeTrainingCacheDataset, groups: dict, predictions: dict) -> None:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    colors = {'before': '#64748b', 'after': '#c73586'}
    figure, axes = plt.subplots(1, 3, figsize=(13, 4))
    for ax, group in zip(axes, ('nominal', 'old_recovery', 'new_recovery')):
        for name in predictions:
            means = report['models'][name]['groups'][group]['run_macro_mean']
            ax.plot([.5, 1., 2., 3.], [means[k]['raw_error_m'] for k in ('0.5s', '1s', '2s', '3s')],
                marker='o', label=name, color=colors[name])
        ax.set(title=group.replace('_', ' '), xlabel='Future time [s]', ylabel='Run-macro XY error [m]')
        ax.grid(alpha=.3); ax.legend()
    figure.suptitle('Same-input validation comparison; one seed, not closed-loop performance')
    figure.tight_layout(); figure.savefig(out/'error_comparison.png', dpi=160); plt.close(figure)
    targets = groups['new_outward_target']
    figure, axes = plt.subplots(2, 3, figsize=(12, 7), squeeze=False)
    for ax, index in zip(axes.flat, targets):
        xy = dataset.targets[index]
        ax.plot(*np.vstack(([0., 0.], xy)).T, label='Observed teacher', color='#238253')
        for name, data in predictions.items():
            ax.plot(*np.vstack(([0., 0.], data[index])).T, label=name, color=colors[name])
        row = dataset._anchors[index]
        ax.set(title=f"{row['run_id'].split('-')[-1]}  {row['observation_ns']/1e9:.3f}s", xlabel='Forward [m]', ylabel='Left [m]')
        ax.set_aspect('equal', adjustable='datalim'); ax.grid(alpha=.3); ax.legend(fontsize=7)
    figure.suptitle('All 6 outward-target validation anchors (2 recovery events; correlated frames)')
    figure.tight_layout(); figure.savefig(out/'outward_paths.png', dpi=160); plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--root', type=Path, default=Path('..'))
    parser.add_argument('--cache', type=Path, required=True)
    parser.add_argument('--training', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root, repo, out = args.root.resolve(), Path(__file__).resolve().parents[1], args.output
    plan = read(args.plan)
    proof = check_reference(root, repo, plan)
    matched = read(args.training/'matched_budget_verification.json')
    if not matched['initial_validation_exact'] or not matched['initial_weights_match']:
        raise ValueError('training comparison parity not established')
    trained = read(args.training/'result.json')
    if trained['status'] != 'COMPLETE' or not trained['reload_predictions_exact']:
        raise ValueError('training incomplete')
    torch.set_num_threads(4)
    soft, hard = resource.getrlimit(resource.RLIMIT_NOFILE)
    resource.setrlimit(resource.RLIMIT_NOFILE, (max(soft, min(hard, 8192)), hard))
    torch.backends.cuda.matmul.allow_tf32 = False; torch.backends.cudnn.allow_tf32 = False
    torch.backends.cudnn.benchmark = False; torch.backends.cudnn.deterministic = True
    cache = verify_time_training_cache(args.cache)
    if cache['plan'] != plan:
        raise ValueError('evaluation plan/cache mismatch')
    dataset = TimeTrainingCacheDataset(args.cache, 'validation', verify_hashes=False)
    if len(dataset) != plan['expected']['validation_total']:
        raise ValueError('validation population changed')
    new_ids = {r['run_id'] for r in plan['runs'] if r['root'] == 'expansion' and r['split'] == 'validation'}
    old_ids = {r['run_id'] for r in plan['runs'] if r['root'] != 'expansion' and r['split'] == 'validation'}
    collected = read(root/plan['collection_index'])
    target_ids = {a for r in collected['production_runs'] if r['run_id'] in new_ids for a in r['target_anchor_ids']}
    groups = {
        'nominal': [i for i, rid in enumerate(dataset.run_ids) if rid not in new_ids | old_ids],
        'old_recovery': [i for i, rid in enumerate(dataset.run_ids) if rid in old_ids],
        'new_recovery': [i for i, rid in enumerate(dataset.run_ids) if rid in new_ids],
        'new_outward_target': [i for i, aid in enumerate(dataset.anchor_ids) if aid in target_ids]}
    for rid in sorted(new_ids):
        groups[rid.split('-')[-1]] = [i for i, r in enumerate(dataset.run_ids) if r == rid]
    if len(groups['new_outward_target']) != 6 or len(groups['new_recovery']) != 186:
        raise ValueError('new validation or outward target population changed')
    original_indices = [i for i, rid in enumerate(dataset.run_ids) if rid in plan['selection_run_ids']]
    out.mkdir(parents=True, exist_ok=False)
    report = {'scope': 'ONE_SEED_SAME_INPUT_RUN_HOLDOUT_NOT_CLOSED_LOOP', 'test_evaluated': False,
        'evaluation_reserved_opened': False, 'calibration_used': False, 'cache_sha256': cache['manifest_sha256'],
        'source_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=repo, text=True).strip(),
        'training_commit': read(args.training/'plan.json')['identity']['source_git_commit'],
        'new_validation_used_for_checkpoint_selection': False, 'historical_control_proof': proof,
        'models': {}, 'controller_config_sha256': _sha(repo/plan['offline_controller_config'])}
    predictions = {}
    for name, checkpoint in [('before', root/plan['reference_training']/'best.pt'), ('after', args.training/'best.pt')]:
        expected_sha = plan['reference_checkpoint_sha256'] if name == 'before' else matched['best_checkpoint_sha256']
        if _sha(checkpoint) != expected_sha:
            raise ValueError('selected checkpoint changed')
        payload = torch.load(checkpoint, map_location='cpu', weights_only=False)
        config = TimeModelConfig.from_dict(payload['config'])
        if config.dataset_config() != dataset.config or config.use_command_history:
            raise ValueError('inference input contract changed')
        model = build_time_model(config).to('cuda')
        load_time_checkpoint(checkpoint, config=config, identity=TimeCheckpointIdentity(**payload['identity']), model=model, mode='finetune')
        data = torch.full((len(dataset), 30, 2), float('nan'))
        for indices in (original_indices, groups['new_recovery']):
            metrics, values = evaluate_time_batched(model, Subset(dataset, indices), run_ids=[dataset.run_ids[i] for i in indices],
                split_manifest=dataset.split_manifest, batch_size=32, workers=4, precision='float32')
            data[indices] = values
            if indices is original_indices:
                source = root/plan['reference_training'] if name == 'before' else args.training
                result = read(source/'result.json')
                np.testing.assert_allclose(values.numpy(), np.load(source/f"validation_epoch_{result['best_epoch']:02d}.npy"), rtol=0, atol=0, equal_nan=True)
        arm = {'checkpoint_sha256': expected_sha, 'checkpoint_epoch': payload['epoch'], 'saved_validation_predictions_exact': True, 'groups': {}}
        for group, indices in groups.items():
            arm['groups'][group] = time_horizon_metrics(data[indices], torch.from_numpy(dataset.targets[indices].copy()),
                torch.from_numpy(dataset.xy_mask[indices].copy()), input_valid=torch.from_numpy(dataset.input_valid[indices].copy()),
                run_ids=[dataset.run_ids[i] for i in indices])
        report['models'][name] = arm; predictions[name] = data.numpy()
        np.save(out/(name+'_predictions.npy'), data.numpy(), allow_pickle=False)
        del model, payload; torch.cuda.empty_cache()
        print('MODEL_EVALUATED', name, flush=True)
    controller = read(repo/plan['offline_controller_config']); validate_trial_config(controller)
    pp = {name: [] for name in ('before', 'after', 'teacher')}
    for i, row in enumerate(dataset._anchors):
        run, local = dataset._index[i]; inputs = dataset._runs[run]['inputs']
        speed = float(inputs['ego'][local, -1, 0])
        pose = TimedBodyPose(int(row['observation_ns']), 'sim', '0', 'map', 'base_link', 0., 0., 0.)
        for name in pp:
            if not dataset.input_valid[i]:
                result = {'applicable': False, 'accepted': False, 'reason': 'INPUT_INVALID'}
            elif name == 'teacher' and not dataset.xy_mask[i].all():
                result = {'applicable': False, 'accepted': False, 'reason': 'TEACHER_FUTURE_INCOMPLETE'}
            else:
                result = pp_probe(dataset.targets[i] if name == 'teacher' else predictions[name][i], pose, pose, speed, controller)
            pp[name].append(result)
    report['pp_at_observation'] = {name: {g: summarize_pp([rows[i] for i in ix]) for g, ix in groups.items()} for name, rows in pp.items()}
    print('PP_AT_OBSERVATION_COMPLETE', flush=True)
    delayed, correspondence_errors = [], []
    for rid in sorted(new_ids):
        pose_index, pose_rows, stamps, speeds = motion(args.cache/'materialized'/rid/'raw')
        for i in [j for j in groups['new_recovery'] if dataset.run_ids[j] == rid]:
            row = dataset._anchors[i]
            obs = replay_observation_pose([pose_rows[k] for k in row['observation_pose_row_ids']],
                observation_ns=row['observation_ns'], freeze_receipt_ns=row['freeze_ns'])
            for age in plan['offline_controller_age_s']:
                current = pose_index.at(obs.stamp_ns+int(round(age*1e9)))
                speed = speed_at(stamps, speeds, current.stamp_ns)
                if age:
                    teacher_world = world_points(dataset.targets[i], obs)[int(round(age*10))-1]
                    error = float(np.linalg.norm(teacher_world - [current.x_m, current.y_m]))
                    correspondence_errors.append(error)
                    if error > 1e-5:
                        raise ValueError('observed motion does not match teacher coordinates')
                entry = {'anchor_id': row['anchor_id'], 'run_id': rid, 'age_s': age, 'speed_mps': speed,
                    'outward_target': row['anchor_id'] in target_ids, 'observation': asdict(obs), 'current': asdict(current), 'models': {}}
                for name in pp:
                    entry['models'][name] = pp_probe(dataset.targets[i] if name == 'teacher' else predictions[name][i], obs, current, speed, controller)
                delayed.append(entry)
    report['delayed_pp'] = {str(age): {name: {group: summarize_pp([r['models'][name] for r in delayed
        if r['age_s'] == age and (group == 'all_new' or r['outward_target'])])
        for group in ('all_new', 'outward_target')} for name in pp} for age in plan['offline_controller_age_s']}
    report['observed_pose_teacher_max_difference_m'] = max(correspondence_errors)
    report['controller_scope'] = 'RECORDED_TEACHER_STATES_0_100_200MS_AGE_SENSITIVITY_NO_SCAN_GUARD_OR_ACTUATOR_ROLLOUT'
    report['age_grid_scope'] = 'ASSUMED_PLAN_AGE_GRID_NOT_MEASURED_INFERENCE_LATENCY'
    write(out/'pp_details.json', {'at_observation': pp, 'delayed_new_recovery': delayed})
    write(out/'comparison.json', report)
    plot(out, report, dataset, groups, predictions)
    print(json.dumps({name: {g: m['run_macro_mean']['3s']['raw_error_m'] for g, m in arm['groups'].items()}
        for name, arm in report['models'].items()}, indent=2), flush=True)


if __name__ == '__main__':
    main()
