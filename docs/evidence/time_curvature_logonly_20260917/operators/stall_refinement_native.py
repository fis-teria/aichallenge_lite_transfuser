"""Separate the observed speed decay from the subsequent PP rejection."""
import json
from pathlib import Path
import numpy as np
root=Path('/home/thistle/e2e_autonomous/runs/time_curvature_logonly_20260917')
raw=root/'raw/codex-time-curve15-logonly-lap03';out=root/'evaluation'
rows=[json.loads(line) for line in (raw/'control.jsonl').read_bytes().splitlines()]
start=next(r['sim_ns'] for r in rows if r.get('event')=='ARMED')
commands=[r for r in rows if r.get('event')=='COMMAND_SENT' and r['sim_ns']>=start]
first=next(r for r in commands if r['reason']=='STEERING_FEASIBLE_LOOKAHEAD_MISSING')
before=[r for r in commands if first['sim_ns']-3_000_000_000<=r['sim_ns']<first['sim_ns']]
traces=[]
for r in before[::10]:
    p=r.get('details',{}).get('longitudinal_preview',{})
    traces.append(dict(t_s=(r['sim_ns']-start)/1e9,speed_kmh=r['speed_mps']*3.6,
        target_kmh=r['target_speed_mps']*3.6,acceleration_command_mps2=r['acceleration_mps2'],
        measured_tire_rad=r['measured_steer_rad'],reason=r['reason'],limit=p.get('limiting_reason'),
        endpoint_distance_m=p.get('endpoint_distance_m')))
result=dict(scope='OBSERVED_RESPONSE_BEFORE_ADMISSION_FAILURE_NOT_NET_FORCE_CALIBRATION',
    all_preceding_3s_commands_tracking=all(r['reason']=='TIME_PATH_TRACKING' for r in before),
    all_preceding_3s_acceleration_commands_positive=all(r['acceleration_mps2']>0 for r in before),
    prior_3s_command_acceleration_range_mps2=[min(r['acceleration_mps2'] for r in before),max(r['acceleration_mps2'] for r in before)],
    prior_3s_speed_start_end_kmh=[before[0]['speed_mps']*3.6,before[-1]['speed_mps']*3.6],traces=traces,
    cause_boundary='Speed declined while positive longitudinal commands were sent; low-speed actuation and shrinking-horizon feedback need alignment. No training-data shortage conclusion follows.',
    video_observation='Extracted stop frames show the kart on the course with visible wall separation; this is not a full-lap no-contact certificate.')
(out/'stall_refinement.json').write_text(json.dumps(result,indent=2)+'\n')
summary=json.loads((out/'summary.json').read_bytes())
print(json.dumps(dict(summary={k:summary[k] for k in ('status','recorded_pose_travel_m','control_replay','active_duration_sim_s','scan_would_stop_commands','stop_confirmed_before_cleanup')},refinement=result)))
