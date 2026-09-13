"""Read-only native WSL diagnosis of turn16's exact rejected scan."""
import argparse
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from aic_transfuser_lite.control.curvature_support_v2 import curvature_support_envelope
from aic_transfuser_lite.control.vehicle_motion_v1 import stopping_motion, AWSIM_POLICY, IDEAL_POLICY

ap = argparse.ArgumentParser()
ap.add_argument('--run', type=Path, required=True)
ap.add_argument('--output', type=Path, required=True)
args = ap.parse_args()
args.output.mkdir(exist_ok=False)
control_bytes = (args.run/'control.jsonl').read_bytes()
rows = [json.loads(line) for line in control_bytes.splitlines()]
rejected = next(r for r in rows if r['event'] == 'SCAN_GUARD_REJECTED')
command = next(r for r in rows if r.get('event') == 'COMMAND_SENT' and r.get('sim_ns') == rejected['sim_ns']
               and r.get('details', {}).get('steering_actuator'))
f = {k: rejected[k] for k in ('scan', 'scan_in_current_rear', 'speed_mps', 'measured_steer_rad',
    'issued_steer_rad', 'previous_steer_rad', 'motion_observation')}
f['provenance'] = {'run_id': args.run.name, 'control_sha256': hashlib.sha256(control_bytes).hexdigest(),
                   'rejected_sim_ns': rejected['sim_ns'], 'scope': 'RECORDED_REJECTION_NOT_COUNTERFACTUAL_DRIVE'}
(args.output/'turn16_side_margin_rejection.json').write_text(json.dumps(f, indent=2))
sensor = np.array(f['scan_in_current_rear']); scan = f['scan']; speed = f['speed_mps']
ranges = np.array(scan['ranges'], float)
angles = scan['angle_min']+np.arange(len(ranges))*scan['angle_increment']
directions = np.column_stack([np.cos(angles+sensor[2]), np.sin(angles+sensor[2])])
points = sensor[:2]+ranges[:, None]*directions
summary = {'scope': 'FROZEN_SCAN_ONLY_NOT_NEW_DRIVE_OR_FREE_SPACE_CERTIFICATION', 'policies': {}}
fig, axes = plt.subplots(1, 2, figsize=(12, 5))
for ax, policy in zip(axes, (IDEAL_POLICY, AWSIM_POLICY)):
    motion = stopping_motion(speed, f['measured_steer_rad'], f['issued_steer_rad'], f['previous_steer_rad'],
        policy=policy, heading_rate_radps=f['motion_observation']['heading_rate_radps'],
        reported_lateral_mps=f['motion_observation']['reported_lateral_mps'])
    travel = .4+speed*.5+speed*speed/2
    n, h, _ = curvature_support_envelope(*motion['curvature_interval_per_m'], travel, scan['angle_increment'],
                                        sensor[:2], lateral_padding_m=motion['lateral_displacement_bound_m'])
    projected = n@directions.T
    remaining = h-n@sensor[:2]
    a = remaining[:, None]/np.where(abs(projected)<1e-12, 1., projected)
    near = np.where(projected < -1e-12, a, -np.inf).max(axis=0)
    far = np.where(projected > 1e-12, a, np.inf).min(axis=0)
    outside = ((abs(projected)<1e-12)&(remaining[:, None]<0)).any(axis=0)
    required = np.where(~outside & (far >= np.maximum(near,0.)), far, 0.)
    margins = np.where(required>0, np.minimum(ranges, scan['range_max'])-required, np.inf)
    index = int(np.argmin(margins))
    summary['policies'][policy] = {'minimum_ray_margin_m': float(margins[index]), 'ray_index': index,
        'observed_range_m': float(ranges[index]), 'required_range_m': float(required[index]),
        'hit_rear_xy_m': points[index].tolist(), 'motion': motion}
    x, y = np.meshgrid(np.linspace(-.8,4.5,251),np.linspace(-1.5,1.5,151))
    inside = np.all(np.stack([x,y],axis=-1)@n.T <= h,axis=-1)
    ax.contour(x,y,inside.astype(float),levels=[.5],colors=['#d46b00'])
    ax.plot(points[:,0],points[:,1],'.',markersize=2,color='gray',label='LiDAR hits')
    path = np.array(command['details']['reference_xy_rear_m'])
    ax.plot(path[:,0],path[:,1],color='#d02caa',label='Age-aligned raw prediction')
    ax.plot(*points[index],'rx',label='Minimum ray margin')
    ax.plot([-.510,1.984,1.984,-.510,-.510],[-.85,-.85,.85,.85,-.85],'k--',label='Existing inflated body')
    ax.set_title(policy+'\nray margin %.4f m'%margins[index])
    ax.set_xlim(-.8,4.5);ax.set_ylim(-1.5,1.5);ax.set_aspect('equal');ax.grid(alpha=.3)
    ax.set_xlabel('Rear-frame forward [m]');ax.set_ylabel('Rear-frame left [m]')
axes[0].legend(fontsize=7,loc='lower left')
fig.suptitle('Same rejected capture; orange = conditional stopping enclosure')
fig.tight_layout();fig.savefig(args.output/'stopping_margin.png',dpi=150)
(args.output/'summary.json').write_text(json.dumps(summary,indent=2))
print(json.dumps(summary))
