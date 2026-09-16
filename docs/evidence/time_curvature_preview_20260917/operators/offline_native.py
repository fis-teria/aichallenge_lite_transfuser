"""Counterfactual speed requests on recorded states; not a new vehicle rollout."""
from collections import Counter
import hashlib
import json
from pathlib import Path

import numpy as np

from aic_transfuser_lite.control.curvature_speed_v1 import ADAPTIVE_SPEED_POLICY
from aic_transfuser_lite.control.time_reference_v1 import TimePlan, TimedBodyPose
from aic_transfuser_lite.control.time_trial_v1 import time_trial_control
from aic_transfuser_lite.control.vehicle_motion_v1 import AWSIM_15KMH_POLICY

base=Path('/home/thistle/e2e_autonomous/runs')
raw=base/'time_launch_15kmh_stop1m_20260917/raw/codex-time-launch15-stop1m-lap01'
out=base/'time_curvature_preview_20260917/offline_preview';out.mkdir(exist_ok=False)
control=[json.loads(line) for line in (raw/'control.jsonl').read_bytes().splitlines()]
plans={r['plan_id']:r for line in (raw/'inference.jsonl').read_bytes().splitlines()
       if (r:=json.loads(line)).get('event')=='PLAN'}
armed=next(r['sim_ns'] for r in control if r['event']=='ARMED')
rows=[]; rejected=Counter()
for row in control:
    if row.get('event')!='COMMAND_SENT' or row['sim_ns']<armed:continue
    detail=row.get('details',{})
    if not all(k in detail for k in ('current_pose','observation_pose')) or row.get('plan_id') not in plans:continue
    observed=TimedBodyPose(**detail['observation_pose']); current=TimedBodyPose(**detail['current_pose'])
    try:
        result=time_trial_control(TimePlan(row['plan_id'],observed,np.asarray(plans[row['plan_id']]['raw_xy_m'])),
            current,speed_mps=row['speed_mps'],rear_axle_offset_m=(.0010000169277191162,0.),
            speed_policy=ADAPTIVE_SPEED_POLICY,vehicle_model_policy=AWSIM_15KMH_POLICY,
            lookahead_policy='stopping_preview_extended_v1')
    except ValueError as exc:
        rejected[str(exc)]+=1
        continue
    preview=result['longitudinal_preview']
    rows.append(dict(sim_after_arm_s=(row['sim_ns']-armed)/1e9,original_reason=row['reason'],
        actual_speed_kmh=row['speed_mps']*3.6,target_speed_kmh=result['target_speed_mps']*3.6,
        acceleration_mps2=result['acceleration_mps2'],limiting_reason=preview['limiting_reason'],
        selected_lookahead_m=result['selected_lookahead_distance_m'],
        minimum_preview_m=result['minimum_preview_distance_m']))
record=dict(scope='COUNTERFACTUAL_SPEED_ON_FIXED_RECORDED_STATES_NOT_CLOSED_LOOP',samples=len(rows),
    original_control_sha256=hashlib.sha256((raw/'control.jsonl').read_bytes()).hexdigest(),
    target_kmh_percentiles=np.percentile([r['target_speed_kmh'] for r in rows],[0,10,50,90,100]).tolist(),
    limits=dict(Counter(r['limiting_reason'] for r in rows)),rejections=dict(rejected),
    preemptive_braking_while_old_control_admitted=sum(r['original_reason']=='TIME_PATH_TRACKING' and r['acceleration_mps2']<0 for r in rows),
    all_measured_speed_pp_limits_preserved=all(r['selected_lookahead_m']>=r['minimum_preview_m'] for r in rows))
assert record['all_measured_speed_pp_limits_preserved'] and record['preemptive_braking_while_old_control_admitted']>0
(out/'summary.json').write_text(json.dumps(record,indent=2)+'\n')
(out/'requests.json').write_text(json.dumps(rows)+'\n')
print(json.dumps(record))
