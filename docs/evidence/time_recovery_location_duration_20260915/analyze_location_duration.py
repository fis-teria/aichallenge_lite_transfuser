"""Audit actual publication duration and stop-site placement in native WSL."""
from pathlib import Path
import hashlib
import json
import math

import numpy as np
from aic_transfuser_lite.data.time_random_steering_pulse_v1 import random_pulse_events

root=Path('/home/thistle/e2e_autonomous/runs/time_recovery_sites_20260915')
rawroot=Path('/home/thistle/e2e_autonomous/raw/time_recovery_sites_20260915')
out=root/'location_duration_review';out.mkdir(exist_ok=False)
reports=[]
for side in ('left','right'):
    name='codex-time-recovery-sites-g01-'+side;raw=rawroot/name
    manifest=json.loads((raw/'transfer_manifest.json').read_text())
    for rel in ('control.jsonl','reference.json','result.json'):
        data=(raw/rel).read_bytes();assert hashlib.sha256(data).hexdigest()==manifest[rel]['sha256']
    rows=[json.loads(s) for s in (raw/'control.jsonl').read_text().splitlines()]
    published=[r for r in rows if r.get('publication')]
    for event in random_pulse_events(rows):
        start=event['start_publication_ns'];end=event['end_publication_ns'];zero=event['zero_publication_ns']
        sampled=[r for r in published if start<=r['publication']['sim_ns']<=end]
        pulse=[r for r in sampled if r.get('pulse',{}).get('applied') and r['phase']=='hold']
        def held_time(threshold):
            return sum((b['publication']['sim_ns']-a['publication']['sim_ns'])/1e9
                for a,b in zip(sampled,sampled[1:]) if a.get('pulse',{}).get('applied')
                and a['phase']=='hold' and abs(a['pulse']['effective_rad'])>threshold)
        stop=[r for r in sampled if r.get('projection') and 119.<=r['projection']['s_m']<=120.]
        usable=[r for r in sampled if r.get('pulse',{}).get('lateral_error_m') is not None]
        zero_row=min(sampled,key=lambda r:abs(r['publication']['sim_ns']-zero))
        first=sampled[0];release=[r for r in sampled if r.get('pulse',{}).get('state',{}).get('release_ns') is not None]
        report=dict(run_id=name,site_id=event['site_id'],sign=event['sign'],
            actual_start_s_m=event['start_s_m'],actual_zero_s_m=zero_row['projection']['s_m'],
            publication_start_to_zero_s=(zero-start)/1e9,
            effective_nonzero_held_s=held_time(1e-6),effective_over_009_rad_held_s=held_time(.09),
            effective_peak_rad=max(abs(r['pulse']['effective_rad']) for r in pulse),
            release_reason=release[0]['pulse']['state']['reason'] if release else zero_row['pulse']['state']['reason'],
            maximum_absolute_lateral_error_m=max(abs(r['pulse']['lateral_error_m']) for r in usable),
            maximum_absolute_heading_error_deg=max(abs(math.degrees(r['pulse']['heading_error_rad'])) for r in usable),
            stop_region_samples=len(stop),stop_region_with_effective_pulse=sum(abs(r['pulse']['effective_rad'])>1e-6 for r in stop),
            source_control_sha256=manifest['control.jsonl']['sha256'])
        reports.append(report)
result=dict(scope='PUBLISHED_STEERING_AND_MEASURED_DEVIATION_NOT_ACTUATOR_HOLD_PROOF',events=reports,
            stop_region_progress_m=[119.,120.],s00_planned_start_m=116.,
            s00_is_only_in_group_1=True,maximum_duration_s=2.,planned_plateau_s=1.5)
(out/'duration_and_location.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result),flush=True)
