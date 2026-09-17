from pathlib import Path
from collections import Counter
import json
import numpy as np
root=Path('/home/thistle/e2e_autonomous/runs/time_path_dev_20260917')
run=root/'raw/codex-time-dev-lap01'
host=json.loads((run/'host_result.json').read_text())
evaluation=json.loads((root/'evaluation/summary.json').read_text())
rows=[json.loads(line) for line in (run/'control.jsonl').read_text().splitlines()]
tracking=[r for r in rows if r.get('reason')=='TIME_PATH_TRACKING']
publisher=next(r for r in rows if r.get('event')=='PUBLISHER_CREATED')
speed=[r['speed_mps']*3.6 for r in tracking if r['speed_mps']>.1]
corner=[r for r in tracking if abs(r['details']['longitudinal_preview']['tracking_curvature_per_m'])>=.05]
assert publisher['speed_parameters']==dict(max_speed_kmh=20.,corner_max_speed_kmh=10.)
assert all(r['target_speed_mps'] <= 10/3.6+1e-9 for r in corner)
assert all(r['target_speed_mps'] <= 20/3.6+1e-9 for r in tracking)
result=dict(status=evaluation['status'],source_commit='f2351125a1b0f17db70f211a8cf7413349aa96cd',
 speed_parameters=publisher['speed_parameters'],judge_laps=host['judge_laps'],
 moving_speed_median_kmh=float(np.median(speed)) if speed else None,
 max_measured_speed_kmh=evaluation['max_measured_speed_mps']*3.6,
 max_tracking_target_kmh=max((r['target_speed_mps']*3.6 for r in tracking),default=None),
 tracking_commands=len(tracking),full_tracking_corner_commands=len(corner),
 max_full_tracking_corner_target_kmh=max((r['target_speed_mps']*3.6 for r in corner),default=None),
 limits=dict(Counter(r['details']['longitudinal_preview']['limiting_reason'] for r in tracking)),
 active_command_reasons=evaluation['active_command_reasons'],
 scan_would_stop_commands=evaluation['scan_would_stop_commands'],
 stop_confirmed=evaluation['stop_confirmed_before_cleanup'],control_replay=evaluation['control_replay'],
 normal_rviz_raw_path_subscribers=host.get('rviz_path_subscribers'),
 geometry_scope='Curvature is from the predicted path and tracking target, not surveyed road curvature')
(root/'dev_analysis.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
