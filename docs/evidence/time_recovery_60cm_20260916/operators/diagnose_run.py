"""Summarize preparation failures and distinguish shutdown-only diagnostics."""
from pathlib import Path
import argparse,hashlib,json,math
import numpy as np

OUT=Path('/home/thistle/e2e_autonomous/runs/time_recovery_60cm_20260916')
RAW=Path('/home/thistle/e2e_autonomous/raw/time_recovery_60cm_20260916')
ap=argparse.ArgumentParser();ap.add_argument('--run',required=True);args=ap.parse_args()
raw=RAW/args.run;control=raw/'control.jsonl'
rows=[json.loads(line) for line in control.read_bytes().splitlines()]
result=json.loads((raw/'result.json').read_bytes())
summary=json.loads((OUT/(args.run+'_collection_summary.json')).read_bytes())
failures=[]
for event in summary['events']:
    if event['completed']:continue
    selected=[r for r in rows if (r.get('large_recovery') or {}).get('state',{}).get('event_id')==event['event_id'] and r.get('publication')]
    prep=[r for r in selected if r['large_recovery']['state']['stage']=='preparing']
    aborted=next((r for r in selected if r['large_recovery']['state']['stage']=='aborted'),None)
    if not prep:continue
    site=next(s for s in summary['planned'] if s['site_id']==event['site_id'])
    near=[r for r in prep if r['projection']['s_m']>=site['release_s_m']-2.]
    def slim(r):
        lr=r['large_recovery'];return dict(sim_ns=r['publication']['sim_ns'],s_m=r['projection']['s_m'],lateral_m=lr['lateral_error_m'],heading_rad=lr['heading_error_rad'],speed_mps=r['speed_mps'],target_since_ns=lr['state']['target_since_ns'])
    gaps=np.diff([r['publication']['sim_ns'] for r in prep])/1e6
    failures.append(dict(site=site,max_abs_preparation_lateral_m=max(abs(r['large_recovery']['lateral_error_m']) for r in prep),
        max_preparation_control_sim_gap_ms=float(gaps.max()),preparation_gaps_over_150ms=int((gaps>150.0001).sum()),
        near_release=[slim(r) for r in near[::5]],first_abort=slim(aborted) if aborted else None,
        reason=aborted['large_recovery']['state']['reason'] if aborted else None))
faults=[dict(sim_ns=r.get('sim_ns'),monotonic_ns=r.get('monotonic_ns'),reason=r.get('reason'),fault=r.get('fault')) for r in rows if r.get('fault')]
last_teacher_future=max((e['end_publication_ns']+3*10**9 for e in summary['events'] if e.get('accepted_camera_anchors',0)),default=None)
proof=dict(run_id=args.run,control_sha256=hashlib.sha256(control.read_bytes()).hexdigest(),
 final_result_status=result['status'],final_result_fault=result['last_control'].get('fault'),
 final_stop_confirmed=result['last_control'].get('stop_confirmed'),last_accepted_teacher_future_ns=last_teacher_future,
 preparation_failures=failures,recorded_fault_rows=faults,
 scope='Frozen run result and accepted future windows; a mutable heartbeat after teardown is not the driving result')
with (OUT/(args.run+'_diagnosis.json')).open('x') as f:json.dump(proof,f,indent=2)
print(json.dumps(proof))
