"""Actual published holds, measured deviation, confirmed PP recovery; no model claim."""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from aic_transfuser_lite.data.time_nominal_steering_guide_v1 import NominalSteeringGuide
from aic_transfuser_lite.data.time_random_steering_pulse_v1 import random_pulse_events
from aic_transfuser_lite.data.time_published_hold_v1 import longest_published_hold_s

OUT = Path('/home/thistle/e2e_autonomous/runs/time_recovery_separated_20260915')
RAW = Path('/home/thistle/e2e_autonomous/raw/time_recovery_separated_20260915')
def read(p):return json.loads(p.read_bytes())
def write(p,v):
    if p.exists():
        assert read(p)==v, str(p)
        return
    with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False)
ap=argparse.ArgumentParser();ap.add_argument('--pair',type=int,required=True);args=ap.parse_args()
plan=read(OUT/'selected_site_plan.json');limits=plan['pilot_acceptance'];reports=[]
if (OUT/'effective_data_plan.json').exists():plan=read(OUT/'effective_data_plan.json')
for item in [r for r in plan['runs'] if r['pair']==args.pair]:
    name=item['run_id'];raw=RAW/name;manifest=read(raw/'transfer_manifest.json')
    assert hashlib.sha256((raw/'control.jsonl').read_bytes()).hexdigest()==manifest['control.jsonl']['sha256']
    rows=[json.loads(s) for s in (raw/'control.jsonl').read_text().splitlines()]
    ref=read(raw/'reference.json');guide=NominalSteeringGuide(ref['steering_pulse']['steering_guide'])
    summary=read(OUT/(name+'_collection_summary.json'))
    events=random_pulse_events(rows);result=read(raw/'result.json')
    for event in events:
        eid=event['event_id'];sign=event['sign'];pubs=[r for r in rows if r.get('publication') and r['pulse']['applied']
            and r['random_pulse']['state']['event_id']==eid and r['phase'] in ('hold','recovery')]
        holds=[r for r in pubs if r['phase']=='hold'];recovery=[r for r in pubs if r['phase']=='recovery']
        hold_samples=[];wave=[]
        for i,r in enumerate(pubs):
            t=r['publication']['sim_ns'];s=r['projection']['s_m'];g=guide.at(s);data=r['guide_control']
            delta=sign*(r['issued_angle_rad']-g)
            good=r['phase']=='hold' and data['guide_weight']>=1.-1e-9 and delta>=.09
            hold_samples.append((t,r['publication']['monotonic_ns'],r['publication']['sequence'],good))
            wave.append(dict(t_s=(t-event['start_publication_ns'])/1e9,progress_m=s,phase=r['phase'],
                guide_input_rad=g,pp_input_rad=r['nominal_angle_rad'],issued_input_rad=r['issued_angle_rad'],
                measured_wheel_rad=r['measured_steering_rad'],guide_weight=data['guide_weight'],
                lateral_m=r['pulse']['lateral_error_m'],heading_deg=math.degrees(r['pulse']['heading_error_rad'])))
        peak=max(pubs,key=lambda r:sign*r['pulse']['lateral_error_m'])
        stop=[r for r in pubs if 119.<=r['projection']['s_m']<=120.]
        signed_peak=sign*peak['pulse']['lateral_error_m']
        stop_peak=max([sign*r['pulse']['lateral_error_m'] for r in stop],default=0.)
        accepted=next(e for e in summary['events'] if e['event_id']==eid).get('accepted',{}).get('count',0)
        entry=dict(run_id=name,event_id=eid,site_id=event['site_id'],sign=sign,
            start_progress_m=holds[0]['projection']['s_m'],pp_return_progress_m=recovery[0]['projection']['s_m'] if recovery else None,
            start_to_pp_s=(event['zero_publication_ns']-event['start_publication_ns'])/1e9 if event['zero_publication_ns'] else None,
            issued_full_plateau_s=longest_published_hold_s(hold_samples),release_reason=pubs[-1]['pulse']['state']['reason'],
            peak_signed_lateral_m=signed_peak,peak_progress_m=peak['projection']['s_m'],
            stop_region_peak_signed_lateral_m=stop_peak,max_abs_heading_deg=max(abs(w['heading_deg']) for w in wave),
            recovery_confirmed=event['recovery_confirmed'],accepted=accepted,
            one_second_hold_qualified=longest_published_hold_s(hold_samples)>=limits['min_issued_plateau_s'],
            result_status=result['status'],normal_stop=result['last_control']['stop_confirmed'])
        entry['pilot_pass']=(entry['one_second_hold_qualified'] and entry['recovery_confirmed']
            and signed_peak>=limits['minimum_signed_lateral_m'] and stop_peak>=limits['stop_region_minimum_signed_lateral_m']
            and accepted>=limits['minimum_anchors_per_event'] and result['status']=='COMPLETE_LAP'
            and result['last_control']['stop_confirmed'] and not result['last_control']['fault'])
        write(OUT/(name+f'_event{eid}_waveform.json'),wave);reports.append(entry)
output=dict(pair=args.pair,events=reports,plan_sha256=hashlib.sha256((OUT/'selected_site_plan.json').read_bytes()).hexdigest(),
    hold_audit_policy='consecutive_publications_allow_equal_sim_v1',
    hold_audit_module_sha256=hashlib.sha256(Path('src/aic_transfuser_lite/data/time_published_hold_v1.py').read_bytes()).hexdigest(),
    expansion_allowed=len(reports)==2 and all(r['site_id']=='S00' and r['pilot_pass'] for r in reports) if args.pair==1 else None)
write(OUT/('pilot_gate.json' if args.pair==1 else f'pair{args.pair:02d}_holds.json'),output)
print(json.dumps(output),flush=True)
