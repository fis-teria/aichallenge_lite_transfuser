"""Generate bounded teacher-only recovery references from a pinned pose CSV."""
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict
import hashlib
import json
from pathlib import Path

import numpy as np

from aic_transfuser_lite.data.recovery_reference_v3 import (
    RecoveryReferenceConfigV3, RecoverySegmentRequestV3, generate_recovery_reference_v3,
    load_occupancy_map_v3,
)
from aic_transfuser_lite.data.time_recovery_collection_v1 import load_pose_course, reference_rows_with_wrap


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--inputs', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--offset-m', type=float, choices=(.2, .4), default=.2)
    ap.add_argument('--geometry', choices=('mixed', 'left_curve', 'right_curve'), default='mixed')
    ap.add_argument('--maximum-base-curvature-inv-m', type=float, default=.01)
    ap.add_argument('--preferred-base-curvature-inv-m', type=float, default=.015)
    ap.add_argument('--base-start-range-m', type=float, nargs=2, metavar=('MIN', 'MAX'),
                    help='Inclusive original-course progress bounds for the approach start [m]; no fallback outside them')
    args = ap.parse_args()
    configs = {side: RecoveryReferenceConfigV3(4., 6., 6., 8., 1.4, .015,
                    args.preferred_base_curvature_inv_m,
                    (RecoverySegmentRequestV3(f'{side}_{round(args.offset_m*100):03d}',
                                             side, args.offset_m, args.geometry,
                                             tuple(args.base_start_range_m) if args.base_start_range_m is not None else None),),
                    maximum_abs_base_curvature_inv_m=args.maximum_base_curvature_inv_m)
               for side in ('left', 'right')}
    for config in configs.values():
        config.validate()
    args.output.mkdir(parents=True, exist_ok=False)
    points = load_pose_course(args.inputs / 'base.csv')
    occupancy = load_occupancy_map_v3(args.inputs / 'occupancy_grid_map.yaml')
    manifest = {'schema': 'measured_time_recovery_collection_v1', 'target_speed_mps': 5/3.6,
                'teacher_source': 'MEASURED_FUTURE_POSE_ONLY', 'model_input': False,
                'sources_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                  for p in args.inputs.iterdir() if p.is_file()}, 'references': {}}
    for side in ('left', 'right'):
        config = configs[side]
        generated = generate_recovery_reference_v3(points, occupancy, config)
        ref = args.output / (side+'.csv')
        with ref.open('x', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['s_m', 'x_m', 'y_m', 'psi_rad', 'kappa_radpm', 'vx_mps', 'ax_mps2'])
            writer.writerows(reference_rows_with_wrap(generated.points))
        # Spatial phase boundaries use the ORIGINAL course progress, continuously.
        # V3 point-index-rounded intervals and its hold=True are intentionally not
        # imported as time-teacher eligibility.
        start = generated.selected_segments[0]['base_start_s_m']
        if start < 5.:
            raise ValueError('RECOVERY_INTERVAL_TOO_CLOSE_TO_INITIAL_SETTLING')
        intervals = [dict(phase=phase, start_s_m=start+a, end_s_m=start+b,
                          training_eligible=phase == 'recovery')
                     for phase, a, b in [('approach', 0, 4), ('hold', 4, 10), ('recovery', 10, 16)]]
        info = {'side': side, 'signed_offset_m': config.requests[0].signed_offset_m,
                'config': asdict(config), 'intervals': intervals,
                'baseline_xy_m': [[p.x_m, p.y_m] for p in points],
                'reference_xy_m': [[p.x_m, p.y_m] for p in generated.points],
                'selected_segments': list(generated.selected_segments),
                'reference_sha256': hashlib.sha256(ref.read_bytes()).hexdigest(),
                'full_body_free_space_verified': False}
        info['controller_wrap_tail_minimum_m'] = 12.
        info['selection_policy'] = (
            f'MAXIMUM_ABSOLUTE_BASE_CURVATURE_AT_MOST_{args.maximum_base_curvature_inv_m:g}_PER_M')
        (args.output / (side+'.json')).write_text(json.dumps(info, indent=2))
        manifest['references'][side] = {k: info[k] for k in ('signed_offset_m', 'intervals', 'reference_sha256')}
    (args.output/'manifest.json').write_text(json.dumps(manifest, indent=2))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(10, 9))
    xy = np.array([[p.x_m, p.y_m] for p in points])
    origin = xy[0]
    ax.plot(*(xy-origin).T, color='black', label='Baseline', linewidth=1)
    for side, color in [('left', 'tab:orange'), ('right', 'tab:blue')]:
        info = json.loads((args.output/(side+'.json')).read_text())
        ref = np.array(info['reference_xy_m'])
        ax.plot(*(ref-origin).T, color=color, label=f'{side} {args.offset_m:.2f} m', linewidth=.8)
    ax.set_aspect('equal'); ax.grid(); ax.legend()
    ax.set(xlabel='Map x relative to start [m]', ylabel='Map y relative to start [m]',
           title='Teacher-only recovery references; fixed target 5 km/h')
    fig.savefig(args.output/'references.png', dpi=140); plt.close(fig)
    print(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    main()
