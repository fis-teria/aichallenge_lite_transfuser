"""Build static normal/preparation CSVs from a hash-verified measured normal lap."""
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, replace
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from aic_transfuser_lite.data.recovery_reference_v3 import load_occupancy_map_v3
from aic_transfuser_lite.data.time_recovery_collection_v1 import load_pose_course, reference_rows_with_wrap
from aic_transfuser_lite.data.time_large_recovery_v1 import SCHEMA, LargeRecoverySite, select_large_sites
from aic_transfuser_lite.data.time_large_recovery_reference_v1 import preparation_course, validate_large_reference


def measured_normal_trace(run: Path, expected: dict[str, str]) -> np.ndarray:
    """Read verified actual poses [N,6]: s/x/y/yaw/speed/base-offset in SI."""
    for name in ('control.jsonl', 'reference.json', 'result.json'):
        if hashlib.sha256((run/name).read_bytes()).hexdigest() != expected[name]:
            raise ValueError('LARGE_NORMAL_SOURCE_HASH:'+name)
    result = json.loads((run/'result.json').read_bytes())
    reference = json.loads((run/'reference.json').read_bytes())
    if (result['status'] != 'COMPLETE_LAP' or result['last_control']['fault']
            or not result['last_control']['stop_confirmed'] or not result['nodes']['closed_bag']
            or reference.get('steering_pulse') or reference.get('large_recovery') or reference['intervals']
            or reference['reference_xy_m'] != reference['baseline_xy_m']):
        raise ValueError('LARGE_REQUIRES_VERIFIED_NORMAL_LAP')
    segments: list[list[list[float]]] = [[]]
    for line in (run/'control.jsonl').read_bytes().splitlines():
        row = json.loads(line)
        if row.get('reason') != 'RECOVERY_TEACHER_TRACKING' or not row.get('publication') or not row.get('projection'):
            continue
        p = row['current_pose']; s = row['projection']['s_m']
        if segments[-1] and s < segments[-1][-1][0]-5.:
            segments.append([])
        segments[-1].append([s, p['x_m'], p['y_m'], p['yaw_rad'], row['speed_mps'], row['projection']['offset_m']])
    unique = {}
    for row in max(segments, key=len):
        unique.setdefault(row[0], row)
    values = np.array([unique[s] for s in sorted(unique)], dtype=float)
    if values.ndim != 2 or values.shape[1] != 6 or len(values) < 1000 or not np.isfinite(values).all():
        raise ValueError('LARGE_NORMAL_LAP_SUPPORT')
    return values


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--inputs', type=Path, required=True)
    ap.add_argument('--normal-run', type=Path, required=True)
    ap.add_argument('--normal-proof', type=Path, required=True, help='Verified selection JSON containing source_hashes by normal run ID')
    ap.add_argument('--plan', type=Path, required=True)
    ap.add_argument('--side', choices=('left', 'right'), required=True, help='Output slot name; actual signs are in the plan')
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    proof = json.loads(args.normal_proof.read_bytes())
    normal = measured_normal_trace(args.normal_run, proof['source_hashes'][args.normal_run.name])
    plan = json.loads(args.plan.read_bytes())
    config = select_large_sites([LargeRecoverySite(**s) for s in plan['candidates']], seed=plan['seed'],
        event_cap=plan['event_cap'], required_site_ids=plan.get('required_site_ids', ()))
    config = replace(config, speed_policy=plan.get('speed_policy', 'bounded_5kmh_v1'),
                     entry_heading_tolerance_rad=plan.get('entry_heading_tolerance_rad', math.radians(1.)),
                     recovery_duration_s=plan.get('recovery_duration_s', 10.))
    base = load_pose_course(args.inputs/'base.csv')
    baseline = [[p.x_m, p.y_m] for p in base]
    old = json.loads((args.normal_run/'reference.json').read_bytes())
    if not np.array_equal(np.asarray(baseline), np.asarray(old['baseline_xy_m'])):
        raise ValueError('LARGE_BASE_COORDINATE_IDENTITY')
    prepared, screen = preparation_course(base, normal, config, load_occupancy_map_v3(args.inputs/'occupancy_grid_map.yaml'))
    spec = dict(schema=SCHEMA, config=asdict(config), nominal_guide=normal[:, [0, 5, 3]].tolist(),
        preparation_csv=args.side+'_preparation.csv', preparation_xy_m=[[p.x_m, p.y_m] for p in prepared],
        map_screen_pass=True, map_screen=screen,
        source_hashes=proof['source_hashes'][args.normal_run.name],
        normal_run_id=args.normal_run.name,
        input_hashes={name: hashlib.sha256((args.inputs/name).read_bytes()).hexdigest()
                      for name in ('base.csv', 'occupancy_grid_map.yaml', 'occupancy_grid_map.pgm')},
        plan_sha256=hashlib.sha256(args.plan.read_bytes()).hexdigest(),
        artificial_reference_is_teacher_label=False, aws_sim_pilot_verified=False)
    reference = dict(side=args.side, signed_offset_m=0., intervals=[], baseline_xy_m=baseline,
        reference_xy_m=baseline, large_recovery=spec, full_body_free_space_verified=False)
    validate_large_reference(reference)  # Fail before writing on malformed guide/plan.
    args.output.mkdir(parents=True, exist_ok=False)
    for name, points in ((args.side+'.csv', base), (spec['preparation_csv'], prepared)):
        with (args.output/name).open('x', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['s_m', 'x_m', 'y_m', 'psi_rad', 'kappa_radpm', 'vx_mps', 'ax_mps2'])
            writer.writerows(reference_rows_with_wrap(points))
    reference['reference_sha256'] = hashlib.sha256((args.output/(args.side+'.csv')).read_bytes()).hexdigest()
    spec['preparation_sha256'] = hashlib.sha256((args.output/spec['preparation_csv']).read_bytes()).hexdigest()
    validate_large_reference(reference, args.output)
    (args.output/(args.side+'.json')).write_text(json.dumps(reference, indent=2, allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(args.output), sites=asdict(config), map_screen=screen,
                          new_teacher_samples=0, runtime_pilot_verified=False)))


if __name__ == '__main__':
    main()
