"""Separate native obstacle fine-tuning with unchanged launch/recovery replay."""
from __future__ import annotations

import argparse
from bisect import bisect_right
from dataclasses import replace
import gc
import json
import math
from pathlib import Path
import resource
import subprocess
from typing import Any

import numpy as np
import torch
from rosbags.highlevel import AnyReader
from torch.utils.data import Subset

from filter_teacher_pose_prefix import verified_bag
from train_time_launch_protection import read, write, sampler_context, stage_errors
from compare_time_recovery_objectives import make_objective
from aic_transfuser_lite.control.time_reference_v1 import TimedBodyPose
from aic_transfuser_lite.data.mcap_converter_v2 import (
    TimedImage, TimedLidar, TimedPose, TimedVelocity, TimedSteering, TimedCommand,
    TimedGear, _stamp_ns, _yaw_from_quaternion, message_image_to_rgb,
)
from aic_transfuser_lite.data.topic_contract_v2 import TOPIC_BY_NAME
from aic_transfuser_lite.data.time_corpus_v1 import audit_anchor
from aic_transfuser_lite.data.time_dataset_v1 import TimeDatasetConfig, assemble_time_sample
from aic_transfuser_lite.data.time_history_v1 import TimeEvent
from aic_transfuser_lite.data.time_native_replay_v1 import (
    AdditiveReplayDataset, NativeReplayDataset, check_native_replay, extend_train_split,
)
from aic_transfuser_lite.data.time_split_v1 import content_sha256
from aic_transfuser_lite.data.time_training_cache_v1 import (
    TimeTrainingCacheDataset, verify_time_training_cache, _sha,
)
from aic_transfuser_lite.evaluation.time_batched_v1 import evaluate_time_batched
from aic_transfuser_lite.evaluation.time_recovery_comparison_v1 import pp_probe
from aic_transfuser_lite.evaluation.time_stage_selection_v1 import StageSelectionPolicy, select_stage_candidate
from aic_transfuser_lite.training.time_checkpoint_v1 import TimeCheckpointIdentity, load_time_checkpoint
from aic_transfuser_lite.training.time_config_v1 import TimeModelConfig, build_time_model
from aic_transfuser_lite.training.time_corpus_runner_v1 import CorpusTrainingPlan, run_training_arm
from aic_transfuser_lite.training.train_time_v1 import training_batch


def records(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def source_guard(repo: Path, plan: dict[str, Any]) -> str:
    if subprocess.check_output(['git', 'status', '--porcelain'], cwd=repo).strip():
        raise ValueError('clean Windows-committed synchronized source required')
    protected = ['src/aic_transfuser_lite/models', 'src/aic_transfuser_lite/training',
        'src/aic_transfuser_lite/contracts', 'src/aic_transfuser_lite/control',
        'src/aic_transfuser_lite/data/time_dataset_v1.py',
        'src/aic_transfuser_lite/data/time_teacher_v1.py',
        'src/aic_transfuser_lite/data/time_training_cache_v1.py',
        'src/aic_transfuser_lite/evaluation/time_batched_v1.py', plan['previous_plan']]
    if subprocess.check_output(['git', 'diff', plan['source_guard_commit'], 'HEAD', '--', *protected], cwd=repo):
        raise ValueError('old recipe, model, objective or input implementation changed')
    if (plan['test_usage'] != 'sealed' or plan['native_split'] != 'train'
            or any(plan[k] for k in ('loss_changes', 'model_input_changes', 'awsim_modifications',
                                     'automatic_runtime_promotion'))):
        raise ValueError('fixed model/objective, sealed test and separate train-only experiment required')
    return subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=repo, text=True).strip()


def decode(role: str, message: Any, receipt_ns: int) -> Any:
    stamp, source = _stamp_ns(message, receipt_ns)
    common = dict(timestamp_ns=stamp, bag_timestamp_ns=receipt_ns, timestamp_source=source)
    if role == 'camera':
        return TimedImage(image_rgb=message_image_to_rgb(message), **common)
    if role == 'lidar':
        return TimedLidar(ranges_m=np.asarray(message.ranges, np.float32),
            angle_min_rad=float(message.angle_min), angle_increment_rad=float(message.angle_increment),
            range_min_m=float(message.range_min), range_max_m=float(message.range_max),
            frame_id=str(message.header.frame_id), **common)
    if role == 'pose':
        return TimedPose(x_world_m=message.pose.pose.position.x, y_world_m=message.pose.pose.position.y,
            yaw_world_rad=_yaw_from_quaternion(message.pose.pose.orientation),
            frame_id=message.header.frame_id, child_frame_id=message.child_frame_id, **common)
    if role == 'velocity':
        return TimedVelocity(longitudinal_mps=message.longitudinal_velocity,
            lateral_mps=message.lateral_velocity, yaw_rate_rps=message.heading_rate, **common)
    if role == 'actual_steering':
        return TimedSteering(steering_rad=message.steering_tire_angle, **common)
    if role == 'gear':
        return TimedGear(gear=int(message.report), **common)
    if role not in {'final_command', 'nominal_command'}:
        raise ValueError(f'unsupported contract role: {role}')
    return TimedCommand(speed_mps=message.longitudinal.speed,
        acceleration_mps2=message.longitudinal.acceleration,
        steering_rad=message.lateral.steering_tire_angle, **common)


def replay_run(bag: Path, rows: list[dict[str, Any]], labels: dict[str, np.ndarray]) -> list[Any]:
    """Replay every selected window using the original all-topic message sequence."""
    intervals: list[list[int]] = []
    for row in sorted(rows, key=lambda r: r['observation_ns']):
        lo, hi = row['observation_ns'] - 1_150_000_000, row['observation_ns'] + 3_300_000_000
        if intervals and lo <= intervals[-1][1]:
            intervals[-1][1] = max(intervals[-1][1], hi)
        else:
            intervals.append([lo, hi])
    starts = [r[0] for r in intervals]
    sensor_refs = {role: {i for row in rows for slot in row['history_row_ids'][role] for i in slot}
                   for role in ('camera', 'lidar')}
    events = []
    with AnyReader([bag]) as reader:
        connections = [c for c in reader.connections if c.topic in TOPIC_BY_NAME]
        for sequence, (connection, receipt, blob) in enumerate(reader.messages(connections=connections)):
            interval = bisect_right(starts, receipt) - 1
            if interval < 0 or receipt > intervals[interval][1]:
                continue
            role = TOPIC_BY_NAME[connection.topic].role
            # All chosen histories are checked below; only large unreferenced sensor payloads are omitted.
            if role in sensor_refs and sequence not in sensor_refs[role]:
                continue
            payload = decode(role, reader.deserialize(blob, connection.msgtype), int(receipt))
            events.append(TimeEvent(role, rows[0]['run_id'], rows[0]['epoch'], 'sim', 'bag_receipt',
                int(payload.timestamp_ns), int(receipt), sequence, payload, 'bag_receipt_proxy'))
    cameras = {e.sequence: e for e in events if e.role == 'camera'}
    samples = []
    for i, row in enumerate(rows):
        anchor = cameras[row['camera_row_id']]
        bounds = tuple(row['epoch_bounds_ns'])
        window = [e for e in events if row['observation_ns'] - 1_150_000_000 <= e.available_ns
                  <= row['observation_ns'] + 3_300_000_000]
        kwargs = dict(config=TimeDatasetConfig(), freeze_ns=row['freeze_ns'],
                      intervention_ns=bounds[1] - 2_000_000_000)
        _, replay = audit_anchor(window, anchor, bounds=bounds, **kwargs)
        sample = assemble_time_sample(window, anchor, epoch_start_ns=bounds[0], epoch_end_ns=bounds[1], **kwargs)
        check_native_replay(row, replay, sample, labels['xy_m'][i], labels['velocity_mps'][i])
        samples.append(sample)
    return samples


def prepare_native(root: Path, plan: dict[str, Any], output: Path) -> list[dict[str, Any]]:
    collection = root / plan['native_collection']
    selection = collection / plan['native_selection']
    if _sha(selection / 'selection_manifest.json') != plan['selection_manifest_sha256']:
        raise ValueError('curated source selection changed')
    manifest = read(selection / 'selection_manifest.json')
    for name, digest in manifest['output_sha256'].items():
        if _sha(selection / name) != digest:
            raise ValueError(f'curated selection artifact changed: {name}')
    shards, additions = [], []
    for summary in manifest['runs']:
        if not summary['selected']:
            continue
        run = summary['run_id']
        rows = records(selection / run / 'selected_anchors.jsonl')
        if len(rows) != summary['selected'] or {r['split_group'] for r in rows} != {plan['native_split_group']}:
            raise ValueError('selected count or common scenario group changed')
        collected = collection / 'collected' / run
        bag, checked = verified_bag(collected, run)
        if checked != summary['identity']['source_bag_sha256'] or str(bag) != summary['identity']['source_bag']:
            raise ValueError('raw source identity changed')
        contract_path = collected / 'raw' / run / 'teacher-contract.json'
        item = read(collected / 'export_manifest.json')['files'][str(contract_path.relative_to(collected))]
        if _sha(contract_path) != item['sha256']:
            raise ValueError('teacher contract changed')
        cap = read(contract_path)['cap_mps'] * 3.6
        if not math.isfinite(cap) or abs(cap - round(cap)) > 1e-6 or cap <= 0:
            raise ValueError('integer configured speed cap in km/h required')
        sources = [dict(path=str((collected / name).relative_to(root)), sha256=digest)
                   for name, digest in checked.items() if name.endswith(('.mcap.zstd', '.mcap', '.db3'))]
        additions.append(dict(run_id=run, speed_cap_kmh=round(cap), split='train', sources=sources))
        with np.load(selection / run / 'selected_teachers.npz', allow_pickle=False) as chosen:
            labels = {key: chosen[key].copy() for key in chosen.files}
        if (not labels['xy_mask'].all() or not labels['velocity_mask'].all()
                or any(labels[k].any() for k in ('stop_mask', 'mode_mask', 'forward_avoidance_eligible'))):
            raise ValueError('selected label masks changed')
        np.testing.assert_array_equal(labels['observation_ns'], [r['observation_ns'] for r in rows])
        samples = replay_run(bag, rows, labels)
        path = output / (run + '.pt')
        torch.save(samples, path)
        shards.append(dict(path=path.name, sha256=_sha(path), run_id=run,
            anchors=[dict(anchor_id=r['anchor_id'], selection_use=r['selection_use']) for r in rows],
            all_raw_histories_and_30_xy_velocity_targets_reproduced=True,
            contract_sha256=item['sha256']))
        print('NATIVE_REPLAY_VERIFIED', run, len(samples), flush=True)
        del samples
        gc.collect()
    if sum(len(s['anchors']) for s in shards) != plan['native_anchors']:
        raise ValueError('native training population changed')
    write(output / 'native_manifest.json', dict(format='native_time_sample_replay_v1',
        selection_manifest_sha256=plan['selection_manifest_sha256'], shards=shards,
        split_group=plan['native_split_group'], split='train', stop_mode_training=False))
    return additions


def prepare(root: Path, repo: Path, plan: dict[str, Any]) -> None:
    head = source_guard(repo, plan)
    output = root / plan['output']
    output.mkdir(exist_ok=False)
    prep = output / 'preparation'
    prep.mkdir()
    previous = read(repo / plan['previous_plan'])
    old_prep = root / plan['previous_experiment'] / 'preparation'
    old_proof = read(old_prep / 'proof.json')
    if previous != old_proof['plan'] or _sha(root / plan['initialization']) != plan['initialization_sha256']:
        raise ValueError('previous recipe or initialization changed')
    cache = verify_time_training_cache(root / previous['cache'])
    if cache['manifest_sha256'] != previous['cache_sha256']:
        raise ValueError('old cache changed')
    protected = {str((old_prep / 'proof.json').relative_to(root)): _sha(old_prep / 'proof.json')}
    for name, key in [('launch_cases.json', 'launch_cases_sha256'),
                      ('baseline_launch_samples.pt', 'baseline_samples_sha256')]:
        path = old_prep / name
        if _sha(path) != old_proof[key]:
            raise ValueError('old launch evaluation changed')
        protected[str(path.relative_to(root))] = _sha(path)
    geometry = root / previous['parent_experiment'] / 'recovery_geometry_targets.json'
    if _sha(geometry) != old_proof['auxiliary_targets_sha256']:
        raise ValueError('old recovery auxiliary targets changed')
    protected[str(geometry.relative_to(root))] = _sha(geometry)
    additions = prepare_native(root, plan, prep)
    split = extend_train_split(cache['split_manifest'], additions, group=plan['native_split_group'])
    write(prep / 'split_manifest.json', split)
    native_hash = _sha(prep / 'native_manifest.json')
    full = TimeTrainingCacheDataset(root / previous['cache'], 'train', verify_hashes=False)
    samplers, _ = sampler_context(root, previous, old_proof, full)
    replay = samplers['launch_balanced']
    if replay.audit != old_proof['samplers']['launch_balanced']:
        raise ValueError('old launch/recovery replay changed')
    native = NativeReplayDataset(prep, manifest_sha256=native_hash)
    combined = AdditiveReplayDataset(replay, native, repeats=plan['native_repeats'], split_manifest=split)
    if (len(combined) != plan['presentations_per_epoch'] or
            math.ceil(len(combined) / plan['training']['batch_size']) * plan['training']['epochs'] != plan['optimizer_steps']):
        raise ValueError('finite presentation/update budget mismatch')
    write(prep / 'proof.json', dict(plan=plan, source_commit=head, previous_artifacts=protected,
        cache_manifest_sha256=cache['manifest_sha256'], native_manifest_sha256=native_hash,
        split_file_sha256=_sha(prep / 'split_manifest.json'), previous_sampler=replay.audit,
        sampler=combined.audit, all_native_windows_replayed=len(native),
        previous_validation_unchanged=True, test_usage='sealed'))
    print('PREPARATION_PASS', json.dumps(combined.audit), flush=True)


def context(root: Path, repo: Path, plan: dict[str, Any]) -> tuple[Any, ...]:
    prep = root / plan['output'] / 'preparation'
    proof = read(prep / 'proof.json')
    if proof['plan'] != plan or proof['source_commit'] != source_guard(repo, plan):
        raise ValueError('frozen plan/source changed')
    if _sha(root / plan['initialization']) != plan['initialization_sha256']:
        raise ValueError('initial model changed')
    for name, digest in proof['previous_artifacts'].items():
        if _sha(root / name) != digest:
            raise ValueError('previous evidence changed')
    if _sha(prep / 'split_manifest.json') != proof['split_file_sha256']:
        raise ValueError('split changed')
    previous = read(repo / plan['previous_plan'])
    old_proof = read(root / plan['previous_experiment'] / 'preparation/proof.json')
    # Full cache verification precedes training, once per process context.
    identity = verify_time_training_cache(root / previous['cache'])
    if identity['manifest_sha256'] != proof['cache_manifest_sha256']:
        raise ValueError('old cache changed')
    full = TimeTrainingCacheDataset(root / previous['cache'], 'train', verify_hashes=False)
    val = TimeTrainingCacheDataset(root / previous['cache'], 'validation', verify_hashes=False)
    samplers, old = sampler_context(root, previous, old_proof, full)
    native = NativeReplayDataset(prep, manifest_sha256=proof['native_manifest_sha256'])
    split = read(prep / 'split_manifest.json')
    combined = AdditiveReplayDataset(samplers['launch_balanced'], native,
                                    repeats=plan['native_repeats'], split_manifest=split)
    if combined.audit != proof['sampler'] or samplers['launch_balanced'].audit != proof['previous_sampler']:
        raise ValueError('frozen replay budget/order changed')
    return proof, old_proof, previous, old, full, val, native, combined, split


def train(root: Path, repo: Path, plan: dict[str, Any], *, resume: bool) -> None:
    proof, old_proof, previous, old, full, val, _, combined, split = context(root, repo, plan)
    targets = read(root / previous['parent_experiment'] / 'recovery_geometry_targets.json')
    objective = make_objective(dict(cache=dict(split_manifest=split),
                                   controller=old_proof['auxiliary_identity']['controller']), old, targets)
    before = old_proof['auxiliary_identity']
    if any(objective.identity[k] != before[k] for k in before if k != 'split_manifest_sha256'):
        raise ValueError('recovery loss or targets changed')
    source = root / plan['initialization']
    payload = torch.load(source, map_location='cpu', weights_only=False)
    cfg = TimeModelConfig.from_dict(payload['config'])
    if cfg.use_command_history or cfg.dataset_config() != full.config or cfg.dataset_config() != TimeDatasetConfig():
        raise ValueError('model input contract changed')
    initial_identity = TimeCheckpointIdentity(**payload['identity'])
    del payload
    selected = old_proof['validation']['selected_indices']
    teacher = dict(format='native_obstacle_additive_replay_v1', contract=full.identity['contract'],
        plan=plan, preparation_sha256=_sha(root / plan['output'] / 'preparation/proof.json'),
        sampler=combined.audit, auxiliary_identity=objective.identity,
        native_manifest_sha256=proof['native_manifest_sha256'],
        selection_validation_anchor_ids_sha256=content_sha256([val.anchor_ids[i] for i in selected]),
        source_git_commit=proof['source_commit'])
    teacher['manifest_sha256'] = content_sha256(teacher)
    identity = TimeCheckpointIdentity(split['manifest_sha256'], teacher['manifest_sha256'],
        content_sha256([r for r in split['runs'] if r['split'] == 'train']),
        plan['experiment'], proof['source_commit'])
    output = root / plan['output'] / 'training'
    result = run_training_arm(combined, Subset(val, selected), train_run_ids=combined.run_ids,
        validation_run_ids=[val.run_ids[i] for i in selected], split_manifest=split,
        teacher_manifest=teacher, identity=identity, config=cfg, plan=CorpusTrainingPlan(**plan['training']),
        output=output, resume=resume, initialization=source,
        initialization_sha256=plan['initialization_sha256'], initialization_identity=initial_identity,
        retain_epoch_checkpoints=True, recovery_objective=objective)
    if not result['reload_predictions_exact'] or result['optimizer_steps'] != plan['optimizer_steps']:
        raise ValueError('training reload or update budget verification failed')
    write(output / 'verification.json', dict(status='PASS', optimizer_steps=result['optimizer_steps'],
        anchors_visited=result['anchors_visited'], initial_checkpoint_sha256=plan['initialization_sha256'],
        checkpoints={f'epoch_{i:02d}.pt': _sha(output / f'epoch_{i:02d}.pt')
                     for i in range(1, plan['training']['epochs'] + 1)}))


def predict_samples(model: Any, samples: Any, batch_size: int = 32) -> np.ndarray:
    predictions = []
    with torch.inference_mode():
        for start in range(0, len(samples), batch_size):
            batch = training_batch([samples[i] for i in range(start, min(len(samples), start + batch_size))],
                                   torch.device('cuda'))
            # Explicitly remove all labels before inference.
            predictions.append(model(replace(batch, targets=None)).float().cpu().numpy())
    return np.concatenate(predictions)


def recovery_run_errors(prediction: np.ndarray, val: Any, selected: list[int], indices: list[int]) -> dict[str, Any]:
    return stage_errors(prediction, val, selected,
        {run: [i for i in indices if val.run_ids[i] == run] for run in sorted({val.run_ids[i] for i in indices})})


def select_retained_native(reports: list[dict[str, Any]], plan: dict[str, Any]) -> dict[str, Any]:
    """Retention is a hard gate; native training fit only ranks eligible candidates."""
    gate = select_stage_candidate(reports, baseline_id='initial', policy=StageSelectionPolicy(**plan['retention_gate']))
    baseline = next(r for r in reports if r['candidate_id'] == 'initial')
    index = {r['candidate_id']: r for r in reports}
    for decision in gate['decisions']:
        report = index[decision['candidate_id']]
        if report['recovery_by_run'].keys() != baseline['recovery_by_run'].keys():
            raise ValueError('fixed recovery run population required')
        for run, before in baseline['recovery_by_run'].items():
            after = report['recovery_by_run'][run]
            if after['anchors'] != before['anchors']:
                raise ValueError('recovery run support changed')
            for metric, key in [('ade_m', 'per_recovery_run_ade_tolerance_m'),
                                ('endpoint_3s_m', 'per_recovery_run_endpoint_tolerance_m')]:
                limit = before[metric] + max(plan[key], before[metric] * plan['per_recovery_run_relative_tolerance'])
                if after[metric] is None or not math.isfinite(after[metric]) or after[metric] > limit:
                    decision['reasons'].append(f'recovery_run:{run}:{metric}:REGRESSED')
        fit = report['native_fit']
        if fit.keys() != baseline['native_fit'].keys():
            raise ValueError('fixed native fit population required')
        for name, row in fit.items():
            before = baseline['native_fit'][name]
            if (row['anchors'], row['runs']) != (before['anchors'], before['runs']):
                raise ValueError('native fit support changed')
            if row['ade_m'] is None or not math.isfinite(row['ade_m']):
                decision['reasons'].append(f'native_fit:{name}:NONFINITE')
        if report['candidate_id'] != 'initial':
            for name in ('all', 'static_cone_xy_speed'):
                value = fit[name]['ade_m']
                if value is None or value > baseline['native_fit'][name]['ade_m'] * (1 - plan['minimum_native_fit_relative_improvement']):
                    decision['reasons'].append(f'native_fit:{name}:NO_IMPROVEMENT')
        decision['eligible'] = not decision['reasons']
    eligible = [d for d in gate['decisions'] if d['eligible'] and d['candidate_id'] != 'initial']
    chosen = min(eligible, key=lambda d: (index[d['candidate_id']]['native_fit']['static_cone_xy_speed']['ade_m'],
                                       d['candidate_id'])) if eligible else None
    name = chosen['candidate_id'] if chosen else 'initial'
    return dict(status='CANDIDATE_FOR_AWSIM_TEST' if chosen else 'KEEP_EXISTING_MODEL',
        selected_candidate_id=name, checkpoint=index[name]['checkpoint'],
        checkpoint_sha256=index[name]['checkpoint_sha256'], decisions=gate['decisions'],
        retention_policy=plan['retention_gate'], native_fit_role=plan['native_fit_role'],
        automatic_runtime_promotion=False, new_awsim_runs=0, test_usage='sealed')


def evaluate(root: Path, repo: Path, plan: dict[str, Any]) -> None:
    _, old_proof, _, _, _, val, native, _, split = context(root, repo, plan)
    base = root / plan['output']
    out = base / 'evaluation'
    out.mkdir(exist_ok=False)
    verification = read(base / 'training/verification.json')
    if verification['status'] != 'PASS':
        raise ValueError('completed training required')
    candidates = {'initial': root / plan['initialization']}
    for i in range(1, plan['training']['epochs'] + 1):
        name = f'epoch_{i:02d}.pt'
        path = base / 'training' / name
        if _sha(path) != verification['checkpoints'][name]:
            raise ValueError('epoch checkpoint changed')
        candidates[f'epoch_{i:02d}'] = path
    selected = old_proof['validation']['selected_indices']
    lookup = {i: j for j, i in enumerate(selected)}
    old_prep = root / plan['previous_experiment'] / 'preparation'
    launch_samples = torch.load(old_prep / 'baseline_launch_samples.pt', weights_only=False)
    cases = read(old_prep / 'launch_cases.json')
    native.targets = np.stack([native[i].teacher.xy_m for i in range(len(native))])
    native_stages = {'all': list(range(len(native)))}
    native_stages.update({use: [i for i, value in enumerate(native.uses) if value == use] for use in sorted(set(native.uses))})
    reports = []
    for name, path in candidates.items():
        payload = torch.load(path, map_location='cpu', weights_only=False)
        cfg = TimeModelConfig.from_dict(payload['config'])
        model = build_time_model(cfg).to('cuda').eval()
        load_time_checkpoint(path, config=cfg, identity=TimeCheckpointIdentity(**payload['identity']), model=model, mode='finetune')
        if name == 'initial':
            _, values = evaluate_time_batched(model, Subset(val, selected), run_ids=[val.run_ids[i] for i in selected],
                split_manifest=split, batch_size=32, workers=0, precision='float32')
            prediction = values.numpy()
        else:
            if payload['teacher_manifest']['selection_validation_anchor_ids_sha256'] != content_sha256([val.anchor_ids[i] for i in selected]):
                raise ValueError('saved validation order changed')
            prediction = np.load(path.parent / f"validation_epoch_{payload['epoch']:02d}.npy", allow_pickle=False)
        sensor_predictions = predict_samples(model, launch_samples, batch_size=1)
        native_predictions = predict_samples(model, native)
        launch = []
        for row in cases:
            xy = prediction[lookup[row['sample_index']]] if row['origin'] == 'nominal_validation' else sensor_predictions[row['sample_index']]
            p = pp_probe(xy, TimedBodyPose(**row['observation_pose']), TimedBodyPose(**row['current_pose']),
                         row['speed_mps'], old_proof['controller'])
            launch.append(dict(case_id=row['case_id'], run_id=row['run_id'], origin=row['origin'], age_s=row['age_s'],
                teacher_accepted=row['teacher']['accepted'], teacher_steer_rad=row['teacher'].get('steer_rad'),
                accepted=p['accepted'], steer_rad=p.get('steer_rad'), reason=p['reason']))
        report = dict(candidate_id=name, checkpoint=str(path), checkpoint_sha256=_sha(path),
            population_sha256=old_proof['population_sha256'], launch=launch,
            xy=stage_errors(prediction, val, selected, old_proof['validation']['stages']),
            recovery_by_run=recovery_run_errors(prediction, val, selected, old_proof['validation']['stages']['recovery']),
            native_fit=stage_errors(native_predictions, native, list(range(len(native))), native_stages))
        write(out / (name + '.json'), report)
        np.save(out / (name + '_predictions.npy'), prediction, allow_pickle=False)
        np.save(out / (name + '_native_predictions.npy'), native_predictions, allow_pickle=False)
        reports.append(report)
        print('CANDIDATE_EVALUATED', name, json.dumps(dict(xy=report['xy'], native_fit=report['native_fit'],
            launch_accepted=sum(r['accepted'] for r in launch), launch_cases=len(launch))), flush=True)
        del model, payload
        torch.cuda.empty_cache()
    selection = select_retained_native(reports, plan)
    write(out / 'selection.json', selection)
    print('SELECTION', json.dumps(selection), flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['prepare', 'train', 'evaluate', 'run'])
    parser.add_argument('--root', type=Path, default=Path('..'))
    parser.add_argument('--plan', type=Path, default=Path('configs/time_path_p1/native_obstacle_replay_20260918.json'))
    parser.add_argument('--resume', action='store_true')
    args = parser.parse_args()
    root = args.root.resolve()
    if root.as_posix().startswith('/mnt/') or not torch.cuda.is_available():
        raise ValueError('native WSL CUDA environment required')
    torch.set_num_threads(4)
    torch.backends.cuda.matmul.allow_tf32 = False
    torch.backends.cudnn.allow_tf32 = False
    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.deterministic = True
    soft, hard = resource.getrlimit(resource.RLIMIT_NOFILE)
    resource.setrlimit(resource.RLIMIT_NOFILE, (max(soft, min(hard, 8192)), hard))
    repo = Path(__file__).resolve().parents[1]
    plan = read(args.plan)
    if args.command in ('prepare', 'run'):
        prepare(root, repo, plan)
    if args.command in ('train', 'run'):
        train(root, repo, plan, resume=args.resume)
    if args.command in ('evaluate', 'run'):
        gc.collect()
        evaluate(root, repo, plan)


if __name__ == '__main__':
    main()
