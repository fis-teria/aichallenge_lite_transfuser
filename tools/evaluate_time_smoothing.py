"""Compare a frozen set of path filters on saved AWSIM plans and validation predictions."""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import subprocess
import time
from typing import Any

import numpy as np
import torch

from aic_transfuser_lite.control.time_trial_v1 import validate_trial_config
from aic_transfuser_lite.data.time_split_v1 import content_sha256
from aic_transfuser_lite.data.time_training_cache_v1 import TimeTrainingCacheDataset
from aic_transfuser_lite.evaluation.time_metrics_v1 import time_horizon_metrics
from aic_transfuser_lite.evaluation.time_smoothing_v1 import (
    METHODS, distribution, path_change_metrics, probe_recorded_pp, smooth_time_paths,
)
from evaluate_time_awsim_trial import replay_recorded_control


def sha(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding='utf-8'))


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n', encoding='utf-8')


def summarize_probes(probes: list[dict[str, Any]], raw: list[dict[str, Any]]) -> dict[str, Any]:
    assert len(probes) == len(raw)
    ok = np.array([p['reason'] == 'PP_OK' for p in probes])
    old_ok = np.array([p['reason'] == 'PP_OK' for p in raw])
    return {'commands': len(probes), 'pp_pass': int(ok.sum()),
        'reasons': dict(Counter(p['reason'] for p in probes)),
        'gained_pp_pass': int((ok & ~old_ok).sum()), 'lost_pp_pass': int((~ok & old_ok).sum()),
        'no_candidate_commands': sum(p['candidate_count'] == 0 for p in probes),
        'best_steering_margin_rad': distribution([p['best_steering_margin_rad'] for p in probes
                                                 if p['best_steering_margin_rad'] is not None]),
        'selected_steer_rad': distribution([p['selected_steer_rad'] for p in probes
                                           if p['selected_steer_rad'] is not None])}


def evaluate_trial(run: Path, expected: dict[str, Any], output: Path) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    hashes = {name: sha(run/name) for name in ('host_result.json', 'control.jsonl', 'inference.jsonl', 'trial_config.json')}
    for name, digest in expected['source_sha256'].items():
        if hashes[name] != digest:
            raise ValueError('recorded AWSIM source changed: '+name)
    host = read_json(run/'host_result.json'); config = read_json(run/'trial_config.json')
    validate_trial_config(config)
    if host['trial_config_sha256'] != hashes['trial_config.json']:
        raise ValueError('recorded configuration hash mismatch')
    control = [json.loads(line) for line in (run/'control.jsonl').read_text().splitlines()]
    plans = [json.loads(line) for line in (run/'inference.jsonl').read_text().splitlines()]
    plans = [p for p in plans if p.get('event') == 'PLAN']
    by_id = {p['plan_id']: p for p in plans}
    if len(by_id) != len(plans) or any(p['checkpoint_sha256'] != config['checkpoint_sha256'] for p in plans):
        raise ValueError('plan identity/checkpoint mismatch')
    starts = [r['sim_ns'] for r in control if r.get('event') == 'ARMED']
    if len(starts) != 1:
        raise ValueError('exactly one drive authorization required')
    start = starts[0]
    commands = [r for r in control if r.get('event') == 'COMMAND_SENT']
    end = min(start+round(config['drive_limit_sim_s']*1e9), max(c['sim_ns'] for c in commands)+1)
    stop = [r['sim_ns'] for r in control if r.get('event') == 'STOP_REQUESTED']
    if stop:
        end = min(end, min(stop))
    active = [r for r in commands if start <= r['sim_ns'] < end]
    replay = replay_recorded_control(active, plans, config['geometry']['rear_axle_forward_in_base_link_m'],
        **{key: config[key] for key in ('speed_policy', 'obstacle_policy', 'steering_policy',
                                        'lookahead_policy', 'vehicle_model_policy')})
    eligible = [c for c in active if c.get('plan_id') and c.get('speed_mps') is not None
                and all(k in c.get('details', {}) for k in ('observation_pose', 'current_pose'))]
    if len(eligible) != expected['control_replay']['matched_commands'] or not eligible:
        raise ValueError('replay population differs from saved evaluation')
    used_ids = list(dict.fromkeys(c['plan_id'] for c in eligible))
    used_index = {pid: i for i, pid in enumerate(used_ids)}
    paths = np.array([by_id[pid]['raw_xy_m'] for pid in used_ids], dtype=float)
    launch = [i for i, c in enumerate(eligible) if c['sim_ns'] < start+5_200_000_000]
    full_probes: dict[str, list[dict[str, Any]]] = {}
    methods = {}; examples = {}
    for method in METHODS:
        changed = smooth_time_paths(paths, method)
        probes = [probe_recorded_pp(changed[used_index[c['plan_id']]], c, config) for c in eligible]
        full_probes[method] = probes
        if method == 'raw':
            for c, p in zip(eligible, probes):
                if p['reason'] != 'PP_OK' and p['reason'] != c['reason']:
                    raise ValueError('raw PP rejection differs from recorded reason')
                if p['reason'] == 'PP_OK' and not np.isclose(p['selected_steer_rad'], c['details']['steer_rad'], atol=1e-9, rtol=0.):
                    raise ValueError('raw PP steering differs from recorded calculation')
        methods[method] = {'full_recorded_interval': summarize_probes(probes, full_probes['raw']),
            'launch_first_5p2s': summarize_probes([probes[i] for i in launch], [full_probes['raw'][i] for i in launch]),
            'used_unique_predictions': path_change_metrics(paths, changed)}
        examples[method] = changed[used_index[eligible[0]['plan_id']]]
    with (output/(run.name+'_commands.jsonl')).open('x', encoding='utf-8') as stream:
        for i, c in enumerate(eligible):
            row = {'sim_ns': c['sim_ns'], 'plan_id': c['plan_id'], 'recorded_reason': c['reason'],
                   'methods': {method: full_probes[method][i] for method in METHODS}}
            stream.write(json.dumps(row, allow_nan=False)+'\n')
    return {'run_id': run.name, 'source_sha256': hashes, 'checkpoint_sha256': config['checkpoint_sha256'],
        'active_commands': len(active), 'eligible_commands': len(eligible),
        'unavailable_commands': len(active)-len(eligible), 'used_unique_plans': len(paths),
        'launch_eligible_commands': len(launch), 'raw_control_replay': replay, 'methods': methods,
        'scope': 'paired counterfactual geometry/PP at recorded poses, not actuator/scan replay or closed loop'}, examples


def evaluate_validation(cache: Path, training: Path, output: Path) -> dict[str, Any]:
    identity = read_json(cache/'identity.json')
    verified = []
    for item in identity['cache_files']:
        relative = Path(item['path'])
        if relative.parts[0] != 'validation' or relative.name not in ('anchors.jsonl', 'inputs.npz', 'labels.npz'):
            continue
        path = (cache/relative).resolve(strict=True)
        if not path.is_relative_to(cache.resolve()) or sha(path) != item['sha256'] or path.stat().st_size != item['bytes']:
            raise ValueError('validation cache metadata/labels changed: '+str(relative))
        verified.append(item)
    dataset = TimeTrainingCacheDataset(cache, 'validation', verify_hashes=False)
    if len(verified) != 3*len(set(dataset.run_ids)):
        raise ValueError('incomplete validation source verification')
    comparison = read_json(training/'comparison.json'); result = read_json(training/'result.json')
    checkpoint_hash = sha(training/'best.pt')
    if checkpoint_hash != comparison['best_checkpoint_sha256']:
        raise ValueError('selected checkpoint changed')
    # Trusted local checkpoint generated in this workspace; never untrusted input.
    checkpoint = torch.load(training/'best.pt', map_location='cpu', weights_only=False)
    teacher = checkpoint['teacher_manifest']
    if teacher['cache_sha256'] != identity['manifest_sha256'] or teacher['validation_anchor_order_sha256'] != content_sha256(dataset.anchor_ids):
        raise ValueError('validation cache/order differs from the trained checkpoint')
    del checkpoint
    prediction_file = training/f"validation_epoch_{result['best_epoch']:02d}.npy"
    raw = np.load(prediction_file, allow_pickle=False)
    valid = np.isfinite(raw).all(axis=(1, 2))
    if raw.shape != dataset.targets.shape or not np.array_equal(valid, dataset.input_valid):
        raise ValueError('prediction shape/input support changed')
    original = read_json(training/'best_validation.json')
    groups = {name: values['run_ids'] for name, values in comparison['groups'].items()}
    methods = {}
    for method in METHODS:
        changed = raw.copy()
        changed[valid] = smooth_time_paths(raw[valid], method).astype(np.float32)
        metrics = time_horizon_metrics(torch.from_numpy(changed), torch.from_numpy(dataset.targets),
            torch.from_numpy(dataset.xy_mask), input_valid=torch.from_numpy(dataset.input_valid), run_ids=dataset.run_ids)
        if method == 'raw' and (not np.isclose(metrics['all_point_ade_m'], original['all_point_ade_m'], atol=1e-9, rtol=0.)
                                or metrics['all_point_count'] != original['all_point_count']):
            raise ValueError('raw validation score/count could not be reproduced')
        assert metrics['horizons']['3s'] == original['horizons']['3s']
        write_json(output/('validation_'+method+'.json'), metrics)
        methods[method] = {'all_point_ade_m': metrics['all_point_ade_m'], 'all_point_count': metrics['all_point_count'],
            'horizon_run_macro': metrics['run_macro_mean'],
            'group_run_macro_ade_m': {name: float(np.mean([metrics['run_macro'][rid]['all_point_ade_m'] for rid in ids]))
                                     for name, ids in groups.items()},
            'input_valid_prediction_change': path_change_metrics(raw[valid], changed[valid])}
    return {'anchor_count': len(dataset), 'input_valid_count': int(valid.sum()),
        'run_ids': list(dict.fromkeys(dataset.run_ids)), 'groups': groups, 'test_evaluated': False,
        'source_sha256': {'cache_identity': sha(cache/'identity.json'), 'predictions': sha(prediction_file),
                          'best_validation': sha(training/'best_validation.json'), 'checkpoint': checkpoint_hash},
        'verified_cache_files': verified, 'methods': methods,
        'scope': 'saved candidate validation predictions, paired at unchanged supported times; no new model inference'}


def plot_summary(trials: dict[str, Any], examples: dict[str, Any], validation: dict[str, Any], output: Path) -> None:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    colors = ['#252525', '#0072b2', '#009e73', '#e69f00', '#cc79a7', '#d55e00']
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    for ax, name in zip(axes[0], ('baseline', 'candidate')):
        for method, color in zip(METHODS, colors):
            xy = examples[name][method]
            ax.plot(xy[:, 0], xy[:, 1], '.-', color=color, markersize=2, linewidth=1, label=method)
        ax.scatter([0], [0], marker='>', color='black')
        ax.set(title=name+' first launch prediction', xlabel='Forward x (m)', ylabel='Lateral y (m)')
        ax.set_aspect('equal', adjustable='datalim'); ax.grid(alpha=.2)
    axes[0, 1].legend(fontsize=8)
    margins = [trials['candidate']['methods'][m]['launch_first_5p2s']['best_steering_margin_rad'] for m in METHODS]
    centers = np.array([m['mean'] for m in margins])*1000
    error = np.array([[m['mean']-m['min'] for m in margins], [m['max']-m['mean'] for m in margins]])*1000
    axes[1, 0].bar(METHODS, centers, yerr=error, color=colors, capsize=3)
    axes[1, 0].axhline(0, color='black', linewidth=.7)
    axes[1, 0].set(title='Candidate launch: best PP steering margin', ylabel='Margin (mrad); positive = within angle limit')
    axes[1, 1].bar(METHODS, [validation['methods'][m]['all_point_ade_m']*100 for m in METHODS], color=colors)
    axes[1, 1].set(title='Candidate validation: all supported points', ylabel='Teacher position ADE (cm)')
    for ax in axes[1]:
        ax.tick_params(axis='x', rotation=25); ax.grid(axis='y', alpha=.2)
    fig.suptitle('Frozen filters on saved predictions; fixed endpoints/time grid; no closed-loop replay')
    fig.tight_layout(); fig.savefig(output/'comparison.png', dpi=160); plt.close(fig)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--records', type=Path, required=True)
    ap.add_argument('--cache', type=Path, required=True)
    ap.add_argument('--training', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args(); start = time.monotonic()
    args.output.mkdir(parents=True, exist_ok=False)
    repo = Path(__file__).resolve().parents[1]
    source = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=repo, text=True).strip()
    write_json(args.output/'plan.json', {'methods': METHODS, 'source_commit': source,
        'records': str(args.records), 'cache': str(args.cache), 'training': str(args.training),
        'fixed_time_dt_s': .1, 'fixed_origin': [0., 0.], 'fixed_3s_endpoint': True,
        'runtime_modified': False, 'awsim_started': False, 'test_evaluated': False})
    trials = {}; examples = {}
    for name, rid in [('baseline', 'codex-time-recovery-model-base01'), ('candidate', 'codex-time-recovery-model-candidate01')]:
        expected = read_json(repo/'docs/evidence/time_recovery_finetune_20260914'/f'{name}_evaluation/summary.json')
        trials[name], examples[name] = evaluate_trial(args.records/name/rid, expected, args.output)
        print(json.dumps({'trial': name, 'pp_launch': {m: trials[name]['methods'][m]['launch_first_5p2s']['pp_pass'] for m in METHODS}}), flush=True)
    validation = evaluate_validation(args.cache, args.training, args.output)
    summary = {'status': 'COMPLETE_OFFLINE_EVALUATION', 'source_commit': source, 'methods': METHODS,
        'trials': trials, 'validation': validation, 'elapsed_s': time.monotonic()-start,
        'scope': 'same saved path before/after smoothing; recorded poses frozen, not simulated future state',
        'not_evaluated': ['closed_loop_driving', 'scan_clearance_after_smoothing', 'hypothetical_actuator_response',
                          'cross_prediction_temporal_filtering', 'test_split', 'unknown_courses'],
        'runtime_modified': False}
    plot_summary(trials, examples, validation, args.output)
    write_json(args.output/'summary.json', summary)
    print(json.dumps({'status': summary['status'], 'elapsed_s': summary['elapsed_s'],
        'validation_ade_m': {m: validation['methods'][m]['all_point_ade_m'] for m in METHODS}}), flush=True)


if __name__ == '__main__':
    main()
