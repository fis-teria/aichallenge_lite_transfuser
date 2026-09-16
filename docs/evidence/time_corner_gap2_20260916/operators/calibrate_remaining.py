"""Bounded static search of unchanged C06/C01/C03 goals, not dynamic proof."""
from dataclasses import asdict, replace
from pathlib import Path
import itertools
import json
import math
import sys
sys.path.insert(0, 'tools')
from generate_time_large_recovery_reference import measured_normal_trace
from aic_transfuser_lite.data.time_large_recovery_v1 import LargeRecoveryConfig, LargeRecoverySite
from aic_transfuser_lite.data.time_large_recovery_reference_v1 import preparation_course
from aic_transfuser_lite.data.time_recovery_collection_v1 import load_pose_course
from aic_transfuser_lite.data.recovery_reference_v3 import load_occupancy_map_v3

root=Path('/home/thistle/e2e_autonomous')
out=root/'runs/time_corner_gap2_20260916'
read=lambda p:json.loads(p.read_bytes())
screen=read(out/'candidate_screen.json')
inputs=root/'runs/time_recovery_collection_20260913/inputs'
normal_path=root/'raw/time_recovery_sites_20260915/codex-time-recovery-sites-normal-n03'
normal=measured_normal_trace(normal_path,read(root/'runs/time_recovery_separated_20260915/selected_site_plan.json')['source_hashes'][normal_path.name])
base=load_pose_course(inputs/'base.csv')
occ=load_occupancy_map_v3(inputs/'occupancy_grid_map.yaml')
trials=[]
for key in ('20:C06','60:C01','60:C03'):
    cm,corner=key.split(':')
    original=LargeRecoverySite(**next(c['site'] for c in screen['catalogs'][cm]['corners'] if c['site']['corner_id']==corner))
    parameters=(itertools.product((4.,6.),(2.,4.),(.07,.085,.1),(1.,1.5,2.),('nominal_path','measured_normal'))
                if key=='20:C06' else itertools.product((4.,6.,8.),(2.,4.),(-.08,-.1),(-.3,.3),('nominal_path',)))
    for approach,settle,bias,heading,origin in parameters:
        site=replace(original,approach_distance_m=approach,settle_distance_m=settle,return_length_m=6.,
            preparation_origin=origin,preparation_offset_bias_m=bias,preparation_heading_bias_rad=math.radians(heading),entry_window_lead_m=.5)
        try:
            _,proof=preparation_course(base,normal,LargeRecoveryConfig((site,),map_screen_policy='oriented_body_v1'),occ)
            row=dict(key=key,site=asdict(site),map_pass=True,evidence=proof)
        except ValueError as exc:
            if not str(exc).startswith(('LARGE_SITE_MAP_REJECTED','LARGE_PREPARATION_PREVIEW_TOO_SHORT')):raise
            row=dict(key=key,site=asdict(site),map_pass=False,reason=str(exc))
        trials.append(row)
    print(json.dumps(dict(key=key,passing=[r['site'] for r in trials if r['key']==key and r['map_pass']])),flush=True)
with (out/'remaining_static_screen.json').open('x') as f:json.dump(dict(trials=trials,scope='STATIC_PATH_SCREEN_ONLY'),f,indent=2)
