"""Plot verified recorded positions, with a completed teacher line as context."""
from pathlib import Path
import hashlib
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root = Path('/home/thistle/e2e_autonomous/runs/time_curvature_preview_20260917')
raw = root / 'raw/codex-time-curve15-lap01'
out = root / 'evaluation'
sources = {}


def verified(directory: Path, name: str) -> bytes:
    manifest = json.loads((directory / 'transfer_manifest.json').read_bytes())
    if isinstance(manifest, list):
        manifest = {row['path']: row for row in manifest}
    data = (directory / name).read_bytes()
    assert len(data) == manifest[name]['bytes']
    assert hashlib.sha256(data).hexdigest() == manifest[name]['sha256']
    sources[str(directory / name)] = manifest[name]
    return data


control = [json.loads(line) for line in verified(raw, 'control.jsonl').splitlines()]
summary = json.loads((out / 'summary.json').read_bytes())
start = summary['armed_sim_ns']
end = start + round(summary['active_duration_sim_s'] * 1e9)
commands = [r for r in control if r['event'] == 'COMMAND_SENT' and start <= r['sim_ns'] < end]
poses = {r['details']['current_pose']['stamp_ns']: r['details']['current_pose']
         for r in commands if r.get('details', {}).get('current_pose')}
xy = np.array([[p['x_m'], p['y_m']] for _, p in sorted(poses.items())])
travel = float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum())
assert abs(travel - summary['recorded_pose_travel_m']) < 1e-6
origin = xy[0]
teacher = Path('/home/thistle/e2e_autonomous/raw/time_recovery_speed_20260914/codex-time-recovery-speedbase-r30')
teacher_result = json.loads(verified(teacher, 'result.json'))
config = json.loads(verified(raw, 'trial_config.json'))
assert teacher_result['status'] == 'COMPLETE_LAP'
assert teacher_result['simulator_assets'] == {'AWSIM_Data/level1': config['geometry']['scene_sha256'], **config['steering_asset_sha256']}
teacher_rows = [json.loads(line) for line in verified(teacher, 'control.jsonl').splitlines()]
teacher_rows = [r for r in teacher_rows if r.get('reason') == 'RECOVERY_TEACHER_TRACKING' and r.get('current_pose')]
assert teacher_rows and all(r['phase'] == 'baseline' for r in teacher_rows)
teacher_xy = np.array([[r['current_pose']['x_m'], r['current_pose']['y_m']] for r in teacher_rows])
old = root.parent / 'time_launch_15kmh_stop1m_20260917/raw/codex-time-launch15-stop1m-lap01'
old_rows = [json.loads(line) for line in verified(old, 'control.jsonl').splitlines()]
old_stop = next(r['details']['current_pose'] for r in old_rows
                if r['event'] == 'COMMAND_SENT' and r['reason'] == 'STOPPING_SWEEP_OCCUPIED')
old_xy = np.array([old_stop['x_m'], old_stop['y_m']])
result = dict(status='PASS', record_status=summary['status'], travel_m=travel,
              last_active_pose=poses[max(poses)], previous_guard_pose=old_stop,
              teacher_scope='Completed measured nominal line for context; not surveyed road center',
              sources=sources)
(out / 'route_context.json').write_text(json.dumps(result, indent=2))
fig, ax = plt.subplots(figsize=(8, 7))
ax.plot(*(teacher_xy-origin).T, color='#bac4cc', linewidth=4, label='Completed teacher trajectory (context)')
ax.plot(*(xy-origin).T, color='#246cac', linewidth=2, label='Curvature preview, ceiling 15 km/h: actual trace')
ax.scatter(*(old_xy-origin), marker='X', color='#d48a20', s=100, zorder=5, label='Previous fixed 15 km/h guard at 315.20 m')
ax.scatter(0, 0, marker='o', color='#248749', s=75, zorder=6, label='Drive authorization')
ax.scatter(*(xy[-1]-origin), marker='s', color='#9c3679', s=70, zorder=6, label='Last active recorded position')
ax.set_aspect('equal'); ax.grid(alpha=.2)
ax.set(xlabel='Map X relative to launch [m]', ylabel='Map Y relative to launch [m]',
       title=f"{summary['status']} | Recorded travel {travel:.1f} m | Adaptive speed, ceiling 15 km/h")
ax.legend(fontsize=8); fig.tight_layout()
fig.savefig(out / 'route_progress.png', dpi=160); plt.close(fig)
print(json.dumps(result))
