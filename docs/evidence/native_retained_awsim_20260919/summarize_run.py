from pathlib import Path
import json, sys, math
from collections import Counter
root=Path('/home/thistle/e2e_autonomous/runs/time_native_retained_awsim_20260919')
kind=sys.argv[1]
assert kind in ('cone','box')
run=root/kind/('codex-time-retained-'+kind+'-01')
host=json.loads((run/'host_result.json').read_text())
rows=[json.loads(x) for x in (run/'control.jsonl').read_text().splitlines()]
armed=host['last_control']['armed_ns']
commands=[r for r in rows if r.get('event')=='COMMAND_SENT' and r.get('sim_ns',0)>=armed and r.get('speed_mps') is not None]
plans={r['plan_id']:r for r in map(json.loads,(run/'inference.jsonl').read_text().splitlines()) if r.get('event')=='PLAN'}
video=json.loads((run/'video_recording.json').read_text())
index=min(range(1,len(commands)),key=lambda i: commands[i]['speed_mps']-commands[i-1]['speed_mps'])
a,b=commands[index-1:index+1]
near=[]
for offset in (-2.,-1.,-.5,-.1,0.,.5,1.):
    r=min(commands,key=lambda r:abs(r['sim_ns']-(b['sim_ns']+offset*1e9)))
    p=plans.get(r.get('plan_id'),{})
    near.append(dict(relative_to_drop_s=(r['sim_ns']-b['sim_ns'])/1e9,
        since_armed_s=(r['sim_ns']-armed)/1e9,
        video_wall_s=(r['monotonic_ns']-video['start_monotonic_ns'])/1e9,
        speed_kmh=r['speed_mps']*3.6,reason=r['reason'],steer_rad=r.get('steer_rad'),
        measured_steer_rad=r.get('measured_steer_rad'),acceleration_mps2=r.get('acceleration_mps2'),
        target_speed_kmh=r.get('target_speed_mps',0)*3.6,
        raw_xy_m=p.get('raw_xy_m'),plan_age_s=r.get('details',{}).get('plan_age_sec')))
report=dict(kind=kind,run_id=host['run_id'],status=host['status'],error=host.get('error'),
    checkpoint_sha256=json.loads((run/'trial_config.json').read_text())['checkpoint_sha256'],
    lap_confirmed=host['judge_lap_confirmed'],laps=host['judge_laps'],sections=host['judge_section_events'],
    speed_max_kmh=max(r['speed_mps'] for r in commands)*3.6,
    requested_stop=host['last_control']['requested_stop_reason'],commands=len(commands),
    command_reasons=dict(Counter(r['reason'] for r in commands)),
    first_reason_s={reason:min((r['sim_ns']-armed)/1e9 for r in commands if r['reason']==reason) for reason in {r['reason'] for r in commands}},
    maximum_speed_drop=dict(from_kmh=a['speed_mps']*3.6,to_kmh=b['speed_mps']*3.6,
        interval_s=(b['sim_ns']-a['sim_ns'])/1e9,since_armed_s=(b['sim_ns']-armed)/1e9,
        video_wall_s=(b['monotonic_ns']-video['start_monotonic_ns'])/1e9),
    around_maximum_drop=near, cleanup_errors=host['cleanup_errors'],
    static_spawn=host['static_scenario_startup'],rviz_path_subscribers=host['rviz_path_subscribers'],
    simulator_assets_unchanged=json.loads((root/kind/(kind+'_trial_postflight.json')).read_text())['assets_unchanged'],
    limits='Speed discontinuity alone is not a contact detector; correlate with recorded video. One trial per scenario, no prior-model same-speed comparison.')
with (root/(kind+'_analysis.json')).open('x') as f:json.dump(report,f,indent=2)
print(json.dumps({k:v for k,v in report.items() if k not in ('around_maximum_drop','sections','static_spawn')},indent=2))
