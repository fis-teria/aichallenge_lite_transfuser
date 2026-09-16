"""Read-only planning diagnostics; no generated reference may be deployed."""
from pathlib import Path
from dataclasses import replace, asdict
from unittest.mock import patch
import hashlib
import json
import math
import sys

import numpy as np

sys.path.insert(0, 'tools')
from generate_time_large_recovery_reference import measured_normal_trace
from aic_transfuser_lite.data import time_large_recovery_reference_v1 as reference
from aic_transfuser_lite.data.time_large_recovery_v1 import LargeRecoverySite, LargeRecoveryConfig
from aic_transfuser_lite.data.time_recovery_collection_v1 import load_pose_course
from aic_transfuser_lite.data.recovery_reference_v3 import load_occupancy_map_v3

ROOT = Path('/home/thistle/e2e_autonomous')
OUT = ROOT/'runs/time_corner_gap_20260916'
OUT.mkdir(exist_ok=False)
INPUTS = ROOT/'runs/time_recovery_collection_20260913/inputs'
RUN = ROOT/'raw/time_recovery_sites_20260915/codex-time-recovery-sites-normal-n03'
PROOF = ROOT/'runs/time_recovery_separated_20260915/selected_site_plan.json'
read = lambda p: json.loads(p.read_bytes())
normal = measured_normal_trace(RUN, read(PROOF)['source_hashes'][RUN.name])
base = load_pose_course(INPUTS/'base.csv')
occ = load_occupancy_map_v3(INPUTS/'occupancy_grid_map.yaml')
catalog = [LargeRecoverySite(**r['site']) for r in read(Path('configs/collection/corner_recovery_20260916.json'))['corners']]


def rectangle_probe(xy: np.ndarray, yaw: np.ndarray, uncertainty_deg: float = 0.) -> dict:
    """Static diagnostic only: standard inflated body in base_link coordinates."""
    assert xy.shape == (len(yaw), 2) and np.isfinite(xy).all() and np.isfinite(yaw).all()
    height, width = occ.free.shape
    cell = occ.resolution_m_per_px
    radius = math.hypot(1.985, .85)
    extra = cell/math.sqrt(2.) + .025
    angles = np.linspace(-math.radians(uncertainty_deg), math.radians(uncertainty_deg), 25) if uncertainty_deg else [0.]
    if uncertainty_deg:
        extra += radius*math.radians(uncertainty_deg)/24.
    bad = []
    for i, ((x, y), heading) in enumerate(zip(xy, yaw)):
        cx=(x-occ.origin_x_m)/cell; cy=(height-1)-(y-occ.origin_y_m)/cell
        extent=math.ceil((radius+extra)/cell)+1
        x0,x1=math.floor(cx)-extent,math.ceil(cx)+extent
        y0,y1=math.floor(cy)-extent,math.ceil(cy)+extent
        if x0<0 or y0<0 or x1>=width or y1>=height:
            bad.append(i);continue
        rr,cc=np.where(~occ.free[y0:y1+1,x0:x1+1])
        dx=(cc+x0)*cell+occ.origin_x_m-x
        dy=(height-1-(rr+y0))*cell+occ.origin_y_m-y
        for delta in angles:
            c,s=math.cos(heading+delta),math.sin(heading+delta)
            forward=dx*c+dy*s; left=-dx*s+dy*c
            if np.any((forward>=-.509-extra)&(forward<=1.985+extra)&(abs(left)<=.85+extra)):
                bad.append(i);break
    return dict(samples=len(xy),bad=len(bad),first_bad_index=bad[0] if bad else None,
                last_bad_index=bad[-1] if bad else None,heading_uncertainty_deg=uncertainty_deg)


screens=[]
for cm in (20,40,60):
    for name in ('C01','C02','C03','C06'):
        original=next(s for s in catalog if s.site_id==name)
        site=replace(original,target_offset_m=math.copysign(cm/100.,original.target_offset_m))
        segments=[]
        # Capture geometry only. No reference bundle is written or marked admissible.
        def capture(occupancy, xy):
            segments.append(xy.copy())
            return True
        with patch.object(reference,'_map_free',capture):
            reference.preparation_course(base,normal,LargeRecoveryConfig((site,),speed_policy='record_actual_v1'),occ)
        diagnostics=[]
        for xy in segments:
            delta=np.gradient(xy,axis=0)
            yaw=np.arctan2(delta[:,1],delta[:,0])
            diagnostics.append(dict(circle_pass=reference._map_free(occ,xy),
                rectangle=rectangle_probe(xy,yaw),rectangle_heading_envelope=rectangle_probe(xy,yaw,12.)))
        sel=(normal[:,0]>=site.start_s_m-1)&(normal[:,0]<=site.release_s_m+site.return_length_m+1)
        observed=normal[sel]
        screens.append(dict(cm=cm,site=asdict(site),segments=diagnostics,
                            observed_normal_rectangle=rectangle_probe(observed[:,1:3],observed[:,3])))
        print(json.dumps(dict(cm=cm,site=name,segments=diagnostics,normal=screens[-1]['observed_normal_rectangle'])),flush=True)

gaps=[]
for cm in (20,40,60):
    out=ROOT/'runs'/f'time_corner_multiscale{cm}_20260916'
    target=read(out/'catalog.json')
    goals={r['site']['site_id']:r['site'] for r in target['corners']}
    roots=[out] + ([ROOT/'runs/time_corner_recovery_20260916'] if cm==20 else [])
    for old in roots:
        for summary in sorted(old.glob('*_collection_summary.json')):
            audit=read(summary)
            if not audit['accepted']:continue
            states=read(old/(audit['run_id']+'_anchor_states.json'))
            for event in audit['events']:
                if event.get('accepted_camera_anchors',0)==0 or event['site_id'] not in goals:continue
                goal=goals[event['site_id']]
                selected=[a for a in states if a['event_id']==event['event_id'] and goal['release_s_m']-.5<=a['base_s_m']<=goal['release_s_m']+3. and abs(a['lateral_m']-goal['target_offset_m'])<=.05]
                exact=[a for a in selected if abs(a['heading_rad']-goal['target_heading_rad'])<=goal['heading_tolerance_rad']]
                gaps.append(dict(cm=cm,run=audit['run_id'],split=audit['split'],site=event['site_id'],
                    accepted=event['accepted_camera_anchors'],lateral_entry_count=len(selected),exact_count=len(exact),
                    entry=[{k:a[k] for k in ('base_s_m','lateral_m','heading_rad','speed_mps')} for a in selected]))

report=dict(diagnostic_only=True,not_deployable=True,map_screens=screens,entry_states=gaps,
            body_base_link_m=dict(rear=-.509,front=1.985,half_width=.85),
            source_hashes=read(PROOF)['source_hashes'][RUN.name],
            map_sha256=hashlib.sha256((INPUTS/'occupancy_grid_map.pgm').read_bytes()).hexdigest())
with (OUT/'initial_diagnosis.json').open('x') as f:json.dump(report,f,indent=2)
print(json.dumps({'output':str(OUT/'initial_diagnosis.json'),'entry_events':len(gaps)}),flush=True)
