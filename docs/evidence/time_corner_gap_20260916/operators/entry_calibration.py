"""Use closed run 3 to calibrate admission geometry, preserving requested goals."""
from dataclasses import asdict, replace
from pathlib import Path
import itertools
import json
import math
import sys
sys.path.insert(0,'tools')
from generate_time_large_recovery_reference import measured_normal_trace
from aic_transfuser_lite.data.time_large_recovery_v1 import LargeRecoverySite, LargeRecoveryConfig
from aic_transfuser_lite.data.time_large_recovery_reference_v1 import preparation_course
from aic_transfuser_lite.data.time_recovery_collection_v1 import load_pose_course
from aic_transfuser_lite.data.recovery_reference_v3 import load_occupancy_map_v3

root=Path('/home/thistle/e2e_autonomous');out=root/'runs/time_corner_gap_20260916'
inputs=root/'runs/time_recovery_collection_20260913/inputs'
normal_path=root/'raw/time_recovery_sites_20260915/codex-time-recovery-sites-normal-n03'
read=lambda p:json.loads(p.read_bytes())
screen=read(out/'candidate_screen.json')
base=load_pose_course(inputs/'base.csv')
normal=measured_normal_trace(normal_path,read(root/'runs/time_recovery_separated_20260915/selected_site_plan.json')['source_hashes'][normal_path.name])
occ=load_occupancy_map_v3(inputs/'occupancy_grid_map.yaml')
c03=replace(LargeRecoverySite(**screen['selected']['40:C03']['site']),
    entry_window_lead_m=.5,preparation_heading_bias_rad=math.radians(.5))
c06=replace(LargeRecoverySite(**read(out/'c06_20_override.json')['20:C06']),approach_distance_m=4.)
overrides={'40:C03':asdict(c03),'20:C06':asdict(c06)}
evidence={}
for key,site in [('40:C03',c03),('20:C06',c06)]:
    _,evidence[key]=preparation_course(base,normal,LargeRecoveryConfig((site,),map_screen_policy='oriented_body_v1'),occ)
original=LargeRecoverySite(**next(c['site'] for c in screen['catalogs']['60']['corners'] if c['site']['site_id']=='C03'))
trials=[]
for bias,heading,origin,ret in itertools.product((-.06,-.08,-.1),(0.,.3,.5,1.),('nominal_path','measured_normal'),(4.,6.,10.)):
    site=replace(original,approach_distance_m=8.,settle_distance_m=4.,entry_window_lead_m=.5,
        preparation_offset_bias_m=bias,preparation_heading_bias_rad=math.radians(heading),preparation_origin=origin,return_length_m=ret)
    try:
        _,proof=preparation_course(base,normal,LargeRecoveryConfig((site,),map_screen_policy='oriented_body_v1'),occ)
        trials.append(dict(site=asdict(site),map_pass=True,evidence=proof))
    except ValueError as exc:
        if not str(exc).startswith(('LARGE_SITE_MAP_REJECTED','LARGE_PREPARATION_PREVIEW_TOO_SHORT')):raise
        trials.append(dict(site=asdict(site),map_pass=False,reason=str(exc)))
valid=[r for r in trials if r['map_pass']]
if valid:
    choice=min(valid,key=lambda r:(abs(r['site']['preparation_heading_bias_rad']-math.radians(.5)),abs(r['site']['preparation_offset_bias_m']+.06),abs(r['site']['return_length_m']-6.)))
    overrides['60:C03']=choice['site']
rows=[json.loads(line) for line in (root/'raw/time_corner_gap_20260916/codex-time-recovery-cornergap-p03-d1/control.jsonl').read_text().splitlines()]
windows=[]
for start in (121.19,121.69,123.69,125.69,127.69,198.28,200.28,202.28,204.28):
    selected=[r for r in rows if r.get('projection') and start<=r['projection']['s_m']<=start+1.
              and (r.get('large_recovery') or {}).get('applied')]
    samples=[]
    for r in selected:
        lr=r['large_recovery'];since=lr['state']['stable_since_ns']
        samples.append(dict(s_m=r['projection']['s_m'],lateral_m=lr['lateral_error_m'],heading_rad=lr['heading_error_rad'],
            speed_mps=r['speed_mps'],minimum_ray_margin_m=r['guard']['minimum_ray_margin_m'],
            stable_s=None if since is None else (r['sim_ns']-since)/1e9))
    windows.append(dict(start_s_m=start,samples=samples))
with (out/'entry_calibration_overrides.json').open('x') as f:json.dump(overrides,f,indent=2)
with (out/'entry_calibration_screen.json').open('x') as f:json.dump(dict(selected=overrides,evidence=evidence,c03_60_trials=trials),f,indent=2)
with (out/'entry_window_pair03.json').open('x') as f:json.dump(dict(source_run='codex-time-recovery-cornergap-p03-d1',windows=windows,scope='Closed normal-pass observations, not proof of future dynamic recovery'),f,indent=2)
print(json.dumps(dict(selected=overrides,c03_60_feasible_variants=len(valid))),flush=True)
