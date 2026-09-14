"""Audit new recovery runs against two measured nominals, in native WSL."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'time_steering_pulse_20260914'))
from summarize_pilot import goal, recovery_time
from analyze_pulse import bins


def alternate_state(row: dict, guide_a: np.ndarray, guide_b: np.ndarray) -> dict:
    """Convert an observed state between guides shaped [N, 3]: m, m, rad."""
    for guide in (guide_a, guide_b):
        assert guide.ndim == 2 and guide.shape[1] == 3 and len(guide) >= 2
        assert np.isfinite(guide).all() and np.all(np.diff(guide[:, 0]) > 0.)
        assert guide[0, 0] <= row['base_s_m'] <= guide[-1, 0]
    s = row['base_s_m']
    lateral = row['left_m'] + np.interp(s, guide_a[:, 0], guide_a[:, 1]) - np.interp(s, guide_b[:, 0], guide_b[:, 1])
    heading = row['heading_rad'] + np.interp(s, guide_a[:, 0], np.unwrap(guide_a[:, 2])) - np.interp(s, guide_b[:, 0], np.unwrap(guide_b[:, 2]))
    return {**row, 'left_m': float(lateral), 'heading_rad': math.atan2(math.sin(heading), math.cos(heading))}


def smoke() -> None:
    a = np.array([[0., 0., math.pi-.01], [1., 0., -math.pi+.01]])
    b = np.array([[0., 0., math.pi], [1., 0., math.pi]])
    row = dict(base_s_m=.5, left_m=.08, heading_rad=math.radians(3.))
    assert goal(alternate_state(row, a, b))
    assert math.isclose(alternate_state(row, a, b)['heading_rad'], row['heading_rad'], abs_tol=1e-12)
    b[:, 1] = .10
    assert goal(row) and not goal(alternate_state(row, a, b))
    for bad in (np.zeros((2, 2)), np.zeros((2, 3)), np.full((2, 3), float('nan'))):
        try:
            alternate_state(row, bad, b)
        except AssertionError:
            pass
        else:
            raise AssertionError('Malformed guide accepted')
    print('EXPANSION_SUMMARY_SMOKE_PASS', flush=True)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--analysis', type=Path)
    ap.add_argument('--raw', type=Path)
    ap.add_argument('--alternate-guide', type=Path)
    ap.add_argument('--runs', nargs='+')
    ap.add_argument('--output', type=Path)
    ap.add_argument('--smoke-only', action='store_true')
    args = ap.parse_args(); smoke()
    if args.smoke_only:
        return
    if not all((args.analysis, args.raw, args.alternate_guide, args.runs, args.output)):
        ap.error('All paths and --runs are required')
    assert not args.output.exists() and len(set(args.runs)) == len(args.runs)
    guide_b = np.asarray(json.loads(args.alternate_guide.read_text())['guide'])
    source_hashes = {str(args.alternate_guide): hashlib.sha256(args.alternate_guide.read_bytes()).hexdigest()}
    summaries = []
    for suffix in args.runs:
        path = args.analysis / (suffix+'_state_audit.json')
        d = json.loads(path.read_text())
        source_hashes[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
        raw = args.raw / d['run_id']
        ref = json.loads((raw/'reference.json').read_text())
        guide_a = np.asarray(ref['steering_pulse']['nominal_guide'])
        accepted = [r for r in d['records'] if r['accepted']]
        alternate = [alternate_state(r, guide_a, guide_b) for r in accepted]
        targets = [a['anchor_id'] for a, b in zip(accepted, alternate) if goal(a) and goal(b)]
        result = json.loads((raw/'result.json').read_text())
        controls = [json.loads(s) for s in (raw/'control.jsonl').read_text().splitlines()]
        pub = d['first_zero_publication_ns']
        active = [r for r in controls if r.get('publication') and pub <= r['publication']['sim_ns'] < pub+10_000_000_000 and r['reason'] == 'RECOVERY_TEACHER_TRACKING']
        recovered = recovery_time(d['control_states'])
        completed = result['status'] == 'COMPLETE_LAP' and result['nodes']['closed_bag'] and not result['cleanup_errors']
        summaries.append(dict(run_id=d['run_id'], config=d['config'], status=result['status'],
            lap_seconds=result['judge_laps'][0]['lap_seconds'] if result.get('judge_laps') else None,
            accepted_primary=d['accepted'], accepted_alternate=bins(alternate),
            target_count_agreeing_with_both_nominals=len(targets), target_anchor_ids_both_nominals=targets,
            first_05s_accepted=d['first_05s_accepted'], first_015s_otherwise_usable=d['first_015s_otherwise_usable'],
            recovery=recovered, minimum_guard_ray_margin_m=min(r['guard']['minimum_ray_margin_m'] for r in active),
            max_abs_issued_steering_rad=max(abs(r['issued_angle_rad']) for r in active),
            calibration_gate_pass=completed and recovered['confirmed'] and len(targets) >= 3))
    report = dict(scope='RECOVERY_TEACHER_DATA_QUALITY; NO_MODEL_PERFORMANCE_MEASURED', runs=summaries,
        input_hashes=source_hashes, independent_runs=len(summaries),
        all_calibration_gates_pass=all(r['calibration_gate_pass'] for r in summaries),
        total_accepted=sum(r['accepted_primary']['count'] for r in summaries),
        total_targets_agreeing_with_both_nominals=sum(r['target_count_agreeing_with_both_nominals'] for r in summaries),
        calibration_runs_final_test_eligible=False)
    with args.output.open('x') as stream:
        stream.write(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
