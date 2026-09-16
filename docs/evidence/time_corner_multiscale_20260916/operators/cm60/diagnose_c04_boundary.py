import ops_corner as m
m.remote('OUT='+repr(m.OUT)+'\nRAW='+repr(m.RAW)+'\n'+r'''
from pathlib import Path
from collections import Counter
import hashlib,json,math
out=Path(OUT);raw=Path(RAW);reports=[];retry_reports=[]
for domain in (1,2):
 name=f'codex-time-recovery-corner60-p03-d{domain}';p=raw/name/'control.jsonl'
 rows=[json.loads(s) for s in p.read_text().splitlines()]
 summary=json.loads((out/(name+'_collection_summary.json')).read_text())
 event=next(e for e in summary['events'] if e['site_id']=='C04');release=event['release_ns']
 selected=[]
 for r in rows:
  d=r.get('large_recovery') or {};s=d.get('state') or {};pub=r.get('publication')
  if s.get('event_id')!=1 or not d.get('applied') or not pub or not 8.8e9<=pub['sim_ns']-release<=10.15e9:continue
  selected.append(dict(observation_clock_after_release_s=(r['sim_ns']-release)/1e9,publication_after_release_s=(pub['sim_ns']-release)/1e9,
   confirmed_after_release_s=(s['confirmed_ns']-release)/1e9 if s.get('confirmed_ns') is not None else None,
   stable_since_after_release_s=(s['stable_since_ns']-release)/1e9 if s.get('stable_since_ns') is not None else None,
   pose_stamp_ns=(r.get('current_pose') or {}).get('stamp_ns'),lateral_m=d['lateral_error_m'],heading_deg=math.degrees(d['heading_error_rad']),stage=s['stage']))
 reports.append(dict(run_id=name,control_sha256=hashlib.sha256(p.read_bytes()).hexdigest(),event=event,rows=selected))
 good=[r for r in rows if r.get('reason')=='RECOVERY_TEACHER_TRACKING' and r.get('target_speed_mps',0)>0 and r.get('publication')]
 counts=Counter(len(r.get('snapshot_retry_reasons',[])) for r in good)
 retry_reports.append(dict(run_id=name,histogram={str(k):v for k,v in sorted(counts.items())},multiple_retry_examples=[{k:r.get(k) for k in ('sim_ns','reason','publication','snapshot_retry_reasons','decision_wall_ms')} for r in good if len(r.get('snapshot_retry_reasons',[]))>=2][:5]))
with (out/'c04_confirmation_boundary_diagnosis.json').open('x') as f:json.dump(dict(runs=reports,adopted_C04_anchors=0,note='Runtime completion and independent publication-based confirmation disagree near the 10s boundary. These two C04 events remain excluded; no deadline or teacher quality threshold was relaxed.'),f,indent=2)
with (out/'pair03_retry_diagnosis.json').open('x') as f:json.dump(retry_reports,f,indent=2)
print(json.dumps(retry_reports));print(json.dumps([dict(run=r['run_id'],last_rows=r['rows'][-3:]) for r in reports]))
''',native=True,lock=True,timeout=120)
