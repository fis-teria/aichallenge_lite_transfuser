"""Same recorded scan/curvature/travel; isolate the lateral reserve only."""
import hashlib
import json
from pathlib import Path
import numpy as np
from aic_transfuser_lite.control.curvature_support_v2 import (
    curvature_support_envelope, stopping_envelope_parameters, STANDARD_CLEARANCE,
    NEAR_LIMIT_CLEARANCE, ONE_METRE_STOPPING_TRAVEL,
)
from aic_transfuser_lite.control.vehicle_motion_v1 import stopping_motion, AWSIM_15KMH_POLICY

root=Path('/home/thistle/e2e_autonomous/runs/time_curvature_launch_20260917')
raw=root/'raw/codex-time-curve15-lap02'
data=(raw/'control.jsonl').read_bytes()
manifest={r['path']:r for r in json.loads((raw/'transfer_manifest.json').read_bytes())}
assert hashlib.sha256(data).hexdigest()==manifest['control.jsonl']['sha256']
rows=[json.loads(line) for line in data.splitlines()]
event=next(r for r in rows if r.get('event')=='SCAN_GUARD_REJECTED')
scan=event['scan'];sensor=np.asarray(event['scan_in_current_rear']);obs=event['motion_observation']
motion=stopping_motion(event['speed_mps'],event['measured_steer_rad'],event['issued_steer_rad'],event['previous_steer_rad'],
    policy=AWSIM_15KMH_POLICY,heading_rate_radps=obs['heading_rate_radps'],reported_lateral_mps=obs['reported_lateral_mps'])
travel,lateral,_=stopping_envelope_parameters(event['speed_mps'],motion,stopping_distance_policy=ONE_METRE_STOPPING_TRAVEL)
ranges=np.asarray(scan['ranges'],float)
angles=scan['angle_min']+np.arange(len(ranges))*scan['angle_increment']+sensor[2]
results=[]
for profile,half in [(STANDARD_CLEARANCE,.85),(NEAR_LIMIT_CLEARANCE,.70)]:
    # Explicit travel=1.0 is held fixed; only this existing helper's width differs.
    n,h,meta=curvature_support_envelope(*motion['curvature_interval_per_m'],travel,scan['angle_increment'],sensor[:2],
        lateral_padding_m=lateral,clearance_profile=profile,vehicle_model_policy=AWSIM_15KMH_POLICY)
    remaining=h-n@sensor[:2];directions=n@np.column_stack([np.cos(angles),np.sin(angles)]).T
    parallel=abs(directions)<1e-12
    intersections=remaining[:,None]/np.where(parallel,1.,directions)
    near=np.where(directions < -1e-12,intersections,-np.inf).max(axis=0)
    far=np.where(directions > 1e-12,intersections,np.inf).min(axis=0)
    intersects=~np.any(parallel & (remaining[:,None]<0),axis=0)&(far>=np.maximum(near,0.))
    required=np.where(intersects,far,0.); observed=required>0
    margin=np.minimum(ranges,scan['range_max'])-required
    results.append(dict(body_half_width_m=half,lateral_fixed_reserve_m=half-.65,
        stopping_travel_m=travel,lateral_uncertainty_m=lateral,
        numerical_and_uncertainty_padding_m=meta['maximum_discretization_padding_m'],
        minimum_ray_margin_m=float(margin[observed].min()),blocked_rays=int(np.sum(margin[observed]<=0))))
assert results[0]['blocked_rays']==2 and results[1]['blocked_rays']==0
fixture=dict(source_run=str(raw),source_control_sha256=manifest['control.jsonl']['sha256'],
    scan=scan,scan_in_current_rear=event['scan_in_current_rear'],speed_mps=event['speed_mps'],
    measured_steer_rad=event['measured_steer_rad'],issued_steer_rad=event['issued_steer_rad'],
    previous_steer_rad=event['previous_steer_rad'],heading_rate_radps=obs['heading_rate_radps'],
    reported_lateral_mps=obs['reported_lateral_mps'],expected=results)
(root/'evaluation/clearance_regression_fixture.json').write_text(json.dumps(fixture,indent=2)+'\n')
(root/'evaluation/clearance_analysis.json').write_text(json.dumps(dict(
    scope='OFFLINE_FIXED_TRAVEL_WIDTH_ONLY_COMPARISON_NOT_NEW_DRIVING_EVIDENCE',results=results),indent=2)+'\n')
print(json.dumps(results))
