"""Generate teacher-only +/-20 cm references from a pinned official pose CSV."""
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
    args = ap.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    points = load_pose_course(args.inputs / 'base.csv')
    occupancy = load_occupancy_map_v3(args.inputs / 'occupancy_grid_map.yaml')
    manifest = {'schema': 'measured_time_recovery_collection_v1', 'target_speed_mps': 5/3.6,
                'teacher_source': 'MEASURED_FUTURE_POSE_ONLY', 'model_input': False,
                'sources_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                  for p in args.inputs.iterdir() if p.is_file()}, 'references': {}}
    for side in ('left', 'right'):
        config = RecoveryReferenceConfigV3(8., 6., 10., 8., 1.4, .015, .015,
                    (RecoverySegmentRequestV3(side+'_020', side, .20, 'mixed'),))
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
        intervals = [dict(phase=phase, start_s_m=start+a, end_s_m=start+b,
                          training_eligible=phase == 'recovery')
                     for phase, a, b in [('approach', 0, 8), ('hold', 8, 14), ('recovery', 14, 24)]]
        info = {'side': side, 'signed_offset_m': .2 if side == 'left' else -.2,
                'config': asdict(config), 'intervals': intervals,
                'baseline_xy_m': [[p.x_m, p.y_m] for p in points],
                'reference_xy_m': [[p.x_m, p.y_m] for p in generated.points],
                'selected_segments': list(generated.selected_segments),
                'reference_sha256': hashlib.sha256(ref.read_bytes()).hexdigest(),
                'full_body_free_space_verified': False}
        info['controller_wrap_tail_minimum_m'] = 12.
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
        ax.plot(*(ref-origin).T, color=color, label=side+' 0.20 m', linewidth=.8)
    ax.set_aspect('equal'); ax.grid(); ax.legend()
    ax.set(xlabel='Map x relative to start [m]', ylabel='Map y relative to start [m]',
           title='Teacher-only recovery references; fixed target 5 km/h')
    fig.savefig(args.output/'references.png', dpi=140); plt.close(fig)
    print(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    main()
