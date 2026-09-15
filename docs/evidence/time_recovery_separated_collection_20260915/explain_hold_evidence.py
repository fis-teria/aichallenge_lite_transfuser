"""Explain strict hold qualifications from immutable final raw publications."""
import hashlib
import json
from pathlib import Path

from aic_transfuser_lite.data.time_published_hold_v1 import longest_published_hold_s

OUT=Path('/home/thistle/e2e_autonomous/runs/time_recovery_separated_20260915')
RAW=Path('/home/thistle/e2e_autonomous/raw/time_recovery_separated_20260915')
def read(p):return json.loads(p.read_bytes())
index=read(OUT/'collection_index.json');details=[]
for entry in index['holds']:
    root=RAW/entry['run_id'];data=(root/'control.jsonl').read_bytes()
    assert hashlib.sha256(data).hexdigest()==read(root/'transfer_manifest.json')['control.jsonl']['sha256']
    rows=[json.loads(s) for s in data.splitlines()]
    rows=[r for r in rows if r.get('publication') and r['pulse']['applied'] and r['phase'] in ('hold','recovery')
          and r['random_pulse']['state']['event_id']==entry['event_id']]
    samples=[];gaps=[];duplicates=0
    for r in rows:
        p=r['publication'];g=r['guide_control']
        good=r['phase']=='hold' and g['guide_weight']>=1.-1e-9 and entry['sign']*(r['issued_angle_rad']-g['guide_angle_rad'])>=.09
        sample=(p['sim_ns'],p['monotonic_ns'],p['sequence'],good)
        if samples:
            a=samples[-1];ds=sample[0]-a[0];dw=sample[1]-a[1];dq=sample[2]-a[2]
            if a[3]:
                duplicates+=ds==0 and good
                if ds>150_000_000 or dw>150_000_000 or dq!=1:
                    gaps.append(dict(sim_interval_ms=ds/1e6,wall_interval_ms=dw/1e6,sequences=[a[2],sample[2]]))
        samples.append(sample)
    duration=longest_published_hold_s(samples)
    assert duration==entry['issued_full_plateau_s']
    qualified=duration>=.95;assert qualified==entry['one_second_hold_qualified']
    shortage=max(0.,.95-duration)
    classification=('QUALIFIED' if qualified else 'CONTINUITY_EVIDENCE_GAP' if gaps
                    else 'WITHIN_ONE_MICROSECOND_OF_FIXED_THRESHOLD' if shortage<=1e-6 else 'SHORTER_DOCUMENTED_PLATEAU')
    details.append(dict(run_id=entry['run_id'],site_id=entry['site_id'],sign=entry['sign'],
        strict_continuous_hold_s=duration,qualified=qualified,classification=classification,
        shortage_from_095_s=shortage,continuous_full_hold_gaps=gaps,duplicate_sim_stamps_during_full_hold=duplicates,
        recovery_confirmed=entry['recovery_confirmed'],accepted=entry['accepted']))
value=dict(threshold_s=.95,threshold_changed=False,scope='PUBLICATION_EVIDENCE_NOT_ACTUATOR_HOLD_MEASUREMENT',events=details)
with (OUT/'hold_evidence_detail.json').open('x') as f:json.dump(value,f,indent=2,allow_nan=False)
print(json.dumps(value),flush=True)
