"""Replay recorded launch geometry in native WSL; no counterfactual sensor claim."""
from __future__ import annotations
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from aic_transfuser_lite.control.time_reference_v1 import TimePlan, TimedBodyPose, prepare_time_reference
from aic_transfuser_lite.control.time_trial_v1 import time_trial_control, validate_trial_config
from aic_transfuser_lite.control.vehicle_motion_v1 import effective_response_length


def inspect_launch(run: Path) -> tuple[dict, np.ndarray]:
    """Use unchanged [30,2] observed-body XY, m/rad/s, for first 5.2 sim seconds."""
    rows = [json.loads(line) for line in (run/'control.jsonl').read_text().splitlines()]
    plans = {p['plan_id']:p for p in map(json.loads,(run/'inference.jsonl').read_text().splitlines())
             if p.get('event')=='PLAN'}
    start = next(r['sim_ns'] for r in rows if r.get('event')=='ARMED')
    cfg = json.loads((run/'trial_config.json').read_text())
    validate_trial_config(cfg)
    commands = [r for r in rows if r.get('event')=='COMMAND_SENT'
                and start <= r['sim_ns'] < start+5_200_000_000
                and r.get('plan_id') and r.get('reason')!='REQUESTED_BRAKE']
    records = []
    first_xy = None
    for c in commands:
        detail=c['details'];saved=plans[c['plan_id']]
        observed=TimedBodyPose(**detail['observation_pose']);current=TimedBodyPose(**detail['current_pose'])
        xy=np.asarray(saved['raw_xy_m'],dtype=float)
        assert xy.shape==(30,2) and np.isfinite(xy).all()
        plan=TimePlan(saved['plan_id'],observed,xy)
        try:
            time_trial_control(plan,current,speed_mps=c['speed_mps'],
                rear_axle_offset_m=(cfg['geometry']['rear_axle_forward_in_base_link_m'],0.),
                speed_policy=cfg['speed_policy'],lookahead_policy=cfg['lookahead_policy'],
                vehicle_model_policy=cfg['vehicle_model_policy'])
            replay_reason='TIME_PATH_TRACKING'
        except ValueError as exc:
            replay_reason=str(exc)
        if c['reason'] in {'TIME_PATH_TRACKING','STEERING_FEASIBLE_LOOKAHEAD_MISSING'}:
            assert replay_reason==c['reason']
        reference=prepare_time_reference(plan,current,
            rear_axle_offset_m=(cfg['geometry']['rear_axle_forward_in_base_link_m'],0.))
        minimum=max(1., .4+max(0.,c['speed_mps'])*.5+c['speed_mps']**2/2)
        length=effective_response_length(max(0.,c['speed_mps']),cfg['vehicle_model_policy'])
        angles=[]
        for point in np.asarray(reference.xy_current_m[1:],dtype=np.float32):
            x,y=map(float,point);squared=x*x+y*y
            if x>1e-6 and minimum <= math.sqrt(squared) <= minimum+.5:
                angles.append(math.atan(length*2*y/max(squared,1e-6)))
        if first_xy is None:
            first_xy=xy
        records.append({'sim_ns':c['sim_ns'],'reason':c['reason'],'replayed_reason':replay_reason,
            'speed_mps':c['speed_mps'],'plan_id':c['plan_id'],
            'observation_pose':detail['observation_pose'],'current_pose':detail['current_pose'],
            'preview_band_m':[minimum,minimum+.5], 'candidate_angles_rad':angles,
            'minimum_abs_angle_rad':min(map(abs,angles)) if angles else None,
            'feasible_point_count':sum(abs(a)<=.3 for a in angles),
            'endpoint_xy_m':xy[-1].tolist()})
    assert records and first_xy is not None
    minima=[r['minimum_abs_angle_rad'] for r in records if r['minimum_abs_angle_rad'] is not None]
    return {'run_id':run.name,'commands':len(records),'reasons':dict(Counter(r['reason'] for r in records)),
        'point_angle_limit_rad':.3,'feasible_commands':sum(r['feasible_point_count']>0 for r in records),
        'minimum_abs_angle_rad_range':[min(minima),max(minima)] if minima else None,
        'first':records[0], 'records':records,
        'source_sha256':{f:hashlib.sha256((run/f).read_bytes()).hexdigest()
                         for f in ('control.jsonl','inference.jsonl','trial_config.json')}}, first_xy


def main() -> None:
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    result={};paths={}
    for name,rid in [('baseline','codex-time-recovery-model-base01'),('candidate','codex-time-recovery-model-candidate01')]:
        result[name],paths[name]=inspect_launch(args.root/name/rid)
    assert result['candidate']['feasible_commands']==0
    assert result['candidate']['reasons']=={'STEERING_FEASIBLE_LOOKAHEAD_MISSING':103}
    result['scope']='recorded_plan_and_control_replay_different_runs_not_identical_sensor_counterfactual'
    result['cause_boundary']='No admissible PP lookahead point; dataset sufficiency and root training cause are not established'
    (args.output/'launch_rejection.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(figsize=(8,4.5))
    for name,color in [('baseline','#008fbd'),('candidate','#d42a96')]:
        xy=paths[name];ax.plot(xy[:,0],xy[:,1],'.-',label=name,color=color)
    ax.scatter([0],[0],marker='>',c='black',label='observation origin')
    ax.set(xlabel='Forward x (m)',ylabel='Lateral y (m, left positive)',
           title='Recorded first launch predictions (different runs)')
    ax.axhline(0,c='grey',linewidth=.7);ax.grid(alpha=.25);ax.legend();ax.set_aspect('equal',adjustable='datalim')
    fig.tight_layout();fig.savefig(args.output/'launch_paths.png',dpi=160);plt.close(fig)
    print(json.dumps({k:{field:value for field,value in v.items() if field!='records'}
                      for k,v in result.items() if isinstance(v,dict)},indent=2))


if __name__=='__main__':
    main()
