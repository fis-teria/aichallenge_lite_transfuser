"""Map recorded runtime phases to the existing teacher controller progress axis."""
from pathlib import Path
import json,math,sys
import numpy as np
sys.path.insert(0,str(Path.cwd()/'tools'))
from analyze_time_normal_lap_attribution import verify_files
root=Path('/home/thistle/e2e_autonomous/runs/time_objective_driving_comparison_20260915_r2')
def rows(p):return [json.loads(x) for x in p.read_text().splitlines()]
reference=Path('/home/thistle/e2e_autonomous/raw/time_recovery_speed_20260914/codex-time-recovery-speedbase-r30')
proof=verify_files(reference,('control.jsonl',))
guide=[r for r in rows(reference/'control.jsonl') if r.get('reason')=='RECOVERY_TEACHER_TRACKING' and r.get('current_pose') and 55<=r['projection']['s_m']<=142]
xy=np.array([[r['current_pose']['x_m'],r['current_pose']['y_m']] for r in guide])
result=dict(scope='Nearest verified normal guide control pose; progress is teacher guide s_m, not runtime traveled distance',reference=str(reference),hashes=proof,runs={})
for rid in ('codex-time-obj-a-01','codex-time-obj-b-03','codex-time-obj-b-04','codex-time-obj-a-06'):
    raw=root/rid/'raw'/rid;verify_files(raw,('control.jsonl',));cs=rows(raw/'control.jsonl')
    arm=next(r for r in cs if r['event']=='ARMED')['sim_ns'];firstfault=next(r['sim_ns'] for r in cs if r['event']=='SCAN_GUARD_REJECTED')
    commands=[r for r in cs if r['event']=='COMMAND_SENT' and arm<=r['sim_ns']<=firstfault and r.get('details',{}).get('current_pose')]
    selected=[min(commands,key=lambda r:abs((r['sim_ns']-arm)/1e9-t)) for t in (65.,75.,85.,92.,98.,(firstfault-arm)/1e9)]
    values=[]
    for r in selected:
        p=r['details']['current_pose'];distance=np.linalg.norm(xy-[p['x_m'],p['y_m']],axis=1);i=int(distance.argmin());g=guide[i]
        values.append(dict(runtime_seconds=(r['sim_ns']-arm)/1e9,map_xy_m=[p['x_m'],p['y_m']],
            nearest_teacher_progress_m=g['projection']['s_m'],distance_m=float(distance[i]),
            heading_difference_deg=math.degrees(math.atan2(math.sin(p['yaw_rad']-g['current_pose']['yaw_rad']),math.cos(p['yaw_rad']-g['current_pose']['yaw_rad'])))))
    result['runs'][rid]=values
with (root/'collection_locations.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result['runs']['codex-time-obj-a-01']))
