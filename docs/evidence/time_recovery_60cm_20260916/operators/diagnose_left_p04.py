"""Preserve the failed left P04 end-of-recovery measurements without relabeling."""
from pathlib import Path
import hashlib,json
import numpy as np

OUT=Path('/home/thistle/e2e_autonomous/runs/time_recovery_60cm_20260916')
RAW=Path('/home/thistle/e2e_autonomous/raw/time_recovery_60cm_20260916')
name='codex-time-recovery-60cm-d60-g03-left-r03'
raw=RAW/name;path=raw/'control.jsonl'
rows=[json.loads(line) for line in path.read_bytes().splitlines()]
summary=json.loads((OUT/(name+'_collection_summary.json')).read_bytes())
event=next(e for e in summary['events'] if e['site_id']=='P04')
assert not event['completed'] and event['accepted_camera_anchors']==0 and event['release_ns'] is not None
release=event['release_ns']
selected=[r for r in rows if r.get('publication') and (r.get('large_recovery') or {}).get('state',{}).get('event_id')==event['event_id'] and r['phase']=='recovery']
def slim(r):
    lr=r['large_recovery'];return dict(seconds_after_handover=(r['publication']['sim_ns']-release)/1e9,
        s_m=r['projection']['s_m'],lateral_m=lr['lateral_error_m'],heading_rad=lr['heading_error_rad'],
        speed_mps=r['speed_mps'],stable_since_ns=lr['state']['stable_since_ns'])
gaps=np.diff([r['publication']['sim_ns'] for r in selected])/1e6
proof=dict(run_id=name,control_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),event=event,
 last_recovery_observation=slim(selected[-1]),last_2_seconds=[slim(r) for r in selected if r['publication']['sim_ns']-release>=8*10**9][::4],
 max_recovery_control_sim_gap_ms=float(gaps.max()),recovery_gaps_over_150ms=int((gaps>150.0001).sum()),
 required_continuous_stability_s=1.,lateral_tolerance_m=.1,heading_tolerance_degrees=2.,
 status='EXCLUDED_RECOVERY_NOT_CONFIRMED',teacher_anchors_from_failed_event=0,
 scope='Observed 10-second recovery ended outside the 10 cm lateral gate; no threshold or deadline changes')
with (OUT/(name+'_diagnosis.json')).open('x') as f:json.dump(proof,f,indent=2)
print(json.dumps({k:v for k,v in proof.items() if k not in ('event','last_2_seconds')}))
