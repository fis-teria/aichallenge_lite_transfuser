"""Observed acceleration command versus response; not a force calibration."""
import json
from pathlib import Path
import numpy as np
base=Path('/home/thistle/e2e_autonomous/runs')
entries=[('first_preview','time_curvature_preview_20260917','codex-time-curve15-lap01'),
         ('completed_fixed10','time_launch_10kmh_stop1m_20260917','codex-time-launch10-stop1m-lap01')]
results=[]
for label,folder,run in entries:
    raw=base/folder/'raw'/run
    log=[json.loads(line) for line in (raw/'control.jsonl').read_bytes().splitlines()]
    start=next(r['sim_ns'] for r in log if r['event']=='ARMED')
    tracking=[r for r in log if r.get('reason')=='TIME_PATH_TRACKING' and r['sim_ns']>=start]
    steady=[r for r in tracking if 9.<r['speed_mps']*3.6<10.]
    results.append(dict(label=label,tracking_commands=len(tracking),
        maximum_speed_kmh=max(r['speed_mps'] for r in tracking)*3.6,
        initial_acceleration_commands=[r['acceleration_mps2'] for r in tracking[:35]],
        steady_9_to_10kmh_samples=len(steady),
        steady_acceleration_command_median_mps2=float(np.median([r['acceleration_mps2'] for r in steady])) if steady else None,
        target_min_mps=min(r['target_speed_mps'] for r in tracking)))
result=dict(scope='OBSERVED_COMMAND_AND_SPEED_NOT_GENERAL_NET_ACCELERATION_CALIBRATION',trials=results)
(base/'time_curvature_preview_20260917/evaluation/startup_diagnosis.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result))
