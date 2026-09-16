from pathlib import Path
from collections import Counter
import hashlib,json
from aic_transfuser_lite.data.time_corner_recovery_v1 import corner_coverage
from aic_transfuser_lite.data.time_large_recovery_v1 import LargeRecoverySite
ROOT=Path('/home/thistle/e2e_autonomous')
def read(p):return json.loads(p.read_bytes())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
phases=[];audits20=[]
for cm in (20,40,60):
 out=ROOT/'runs'/f'time_corner_multiscale{cm}_20260916';index=read(out/'collection_index.json');moving={};reasons=Counter();acquisition_overspeed_stops=0;retry_successes=Counter();failures={}
 for run in index['runs']:
  raw=Path(run['raw_path']);assert sha(raw/'control.jsonl')==run['control_sha256']
  rows=[json.loads(s) for s in (raw/'control.jsonl').read_text().splitlines()]
  for r in rows:
   if r.get('reason')=='RECOVERY_TEACHER_TRACKING' and r.get('target_speed_mps',0)>0 and r.get('publication'):
    retry_successes[len(r.get('snapshot_retry_reasons',[]))]+=1
  if run['status']!='COMPLETE_LAP':failures[run['run_id']]=dict(status=run['status'],fault=run['fault'],error=read(raw/'result.json').get('error'),accepted=run['accepted'])
  moving[run['run_id']]=[r['monotonic_ns'] for r in rows if r.get('publication') and r['speed_mps'] is not None and r['speed_mps']>.1 and r['reason']=='RECOVERY_TEACHER_TRACKING']
  acquisition_overspeed_stops+=sum((r.get('large_recovery') or {}).get('state',{}).get('reason')=='PREPARATION_SPEED_LIMIT' for r in rows)
  reason=run['end_reason'];reasons[str(reason)]+=1
  if cm==20:
   audit=read(out/(run['run_id']+'_collection_summary.json'));audit['anchor_states']=read(out/(run['run_id']+'_anchor_states.json')) if run['accepted'] else [];audits20.append(audit)
 parallel=[]
 for left,ticks in moving.items():
  if not left.endswith('-d1'):continue
  right=left[:-1]+'2';other=moving[right]
  overlap=max(0.,(min(max(ticks),max(other))-max(min(ticks),min(other)))/1e9) if ticks and other else 0.
  parallel.append(dict(left=left,right=right,moving_overlap_wall_s=overlap))
 phase=dict(amplitude_cm=cm,index_sha256=sha(out/'collection_index.json'),totals=index['totals'],attempts=len(index['runs']),complete_laps=sum(r['status']=='COMPLETE_LAP' for r in index['runs']),accepted_events=sum(r['accepted_events'] for r in index['runs']),raw_bytes=index['raw_bytes'],maximum_speed_kmh=max(r['maximum_measured_speed_mps'] for r in index['runs'])*3.6,accepted_anchors_above_old_speed_band=sum(r['above_legacy_accepted_anchor_count'] for r in index['runs']),preparation_speed_abort_rows=acquisition_overspeed_stops,end_reasons=dict(reasons),parallel=parallel,missing_exact_entry=index['missing_mandatory_corners'])
 phase['peak_abs_accepted_lateral_m']=max((r.get('peak_abs_accepted_lateral_m',0.) for r in index['runs']),default=0.)
 phase['successful_control_retry_histogram']={str(k):v for k,v in sorted(retry_successes.items())}
 phase['failed_runs']=failures
 assert acquisition_overspeed_stops==0
 phases.append(phase)
old=ROOT/'runs/time_corner_recovery_20260916';old_anchors=0;old_events=0
for p in old.glob('*_collection_summary.json'):
 a=read(p);a['anchor_states']=read(old/(a['run_id']+'_anchor_states.json')) if a['accepted'] else [];audits20.append(a)
 old_anchors+=a['accepted'];old_events+=sum(e.get('accepted_camera_anchors',0)>0 for e in a['events'])
assert old_anchors==1475 and old_events==17
cat=read(Path('configs/collection/corner_recovery_20260916.json'))
combined=corner_coverage([LargeRecoverySite(**r['site']) for r in cat['corners']],audits20)
out=ROOT/'runs/time_corner_multiscale20_20260916'
(out/'combined_20cm_coverage.json').write_text(json.dumps(combined,indent=2))
report=dict(schema='corner_multiscale_collection_20260916',phases=phases,new_accepted_anchors=sum(s['anchors'] for p in phases for s in p['totals'].values()),new_accepted_events=sum(p['accepted_events'] for p in phases),raw_bytes=sum(p['raw_bytes'] for p in phases),old_20cm_anchors=old_anchors,old_20cm_events=old_events,combined_20cm_exact_entry_missing=combined['missing'],fixed_target_kmh=5,speed_policy='record_actual_v1',actual_speed_is_recorded=True,preparation_is_not_teacher=True,training_started=False,awsim_modified=False)
(out/'multiscale_summary.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
