import manage as m
import sys
name=sys.argv[1]
assert name.startswith('codex-time-recovery-separated-')
m.remote((m.REPO/'src/aic_transfuser_lite/data/time_published_hold_v1.py').read_text()+'\n'+rf'''
from pathlib import Path
import json,math
p=Path({m.ROOT!r})/{name!r}
rows=[json.loads(s) for s in (p/'control.jsonl').read_text().splitlines() if s.strip().endswith('}}')]
rows=[r for r in rows if r.get('publication') and r.get('pulse',{{}}).get('applied') and r['phase'] in ('hold','recovery')]
events=[]
for eid in sorted(set(r['random_pulse']['state']['event_id'] for r in rows)):
 group=[r for r in rows if r['random_pulse']['state']['event_id']==eid]
 holds=[r for r in group if r['phase']=='hold'];rec=[r for r in group if r['phase']=='recovery']
 st=group[-1]['random_pulse'];site=st['config']['sites'][st['state']['active_site_index']]
 hold_samples=[]
 for i,r in enumerate(group):
  g=r['guide_control'];t=r['publication']['sim_ns'];delta=site['sign']*(r['issued_angle_rad']-g['guide_angle_rad'])
  good=r['phase']=='hold' and g['guide_weight']>=1.-1e-9 and delta>=.09
  hold_samples.append((t,r['publication']['monotonic_ns'],r['publication']['sequence'],good))
 events.append(dict(site=site['site_id'],hold_s=(rec[0]['publication']['sim_ns']-holds[0]['publication']['sim_ns'])/1e9 if rec else None,
  plateau_s=longest_published_hold_s(hold_samples),start_s=holds[0]['projection']['s_m'],zero_s=rec[0]['projection']['s_m'] if rec else None,
  max_signed_lateral_m=max(site['sign']*r['pulse']['lateral_error_m'] for r in group),
  max_abs_heading_deg=max(abs(math.degrees(r['pulse']['heading_error_rad'])) for r in group),
  stop_region_lateral_m=max([site['sign']*r['pulse']['lateral_error_m'] for r in group if 119<=r['projection']['s_m']<=120],default=0),
  reason=group[-1]['pulse']['state']['reason'],state=st['state']['stage']))
h=json.loads((p/'control_heartbeat.json').read_text())
current=h.get('random_pulse',{{}}).get('state',{{}})
print(json.dumps(dict(run_id=p.name,events=events,progress=(h.get('projection') or {{}}).get('s_m'),fault=h.get('fault'),phase=h.get('phase'),
 completed_events=current.get('completed_events'),current_schedule_stage=current.get('stage'),skipped=current.get('skipped_sites'))))
''')
