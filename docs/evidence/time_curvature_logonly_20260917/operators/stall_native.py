"""Trace the actual non-proximity stop and extract real video frames."""
import hashlib
import json
from pathlib import Path
import subprocess
import numpy as np
from aic_transfuser_lite.control.time_reference_v1 import TimePlan, TimedBodyPose, prepare_time_reference
from aic_transfuser_lite.control.vehicle_motion_v1 import effective_response_length, AWSIM_15KMH_POLICY
from aic_transfuser_lite.control.polyline_lookahead_v1 import select_polyline_lookahead

root=Path('/home/thistle/e2e_autonomous/runs/time_curvature_logonly_20260917')
raw=root/'raw/codex-time-curve15-logonly-lap03';out=root/'evaluation'
rows=[json.loads(line) for line in (raw/'control.jsonl').read_bytes().splitlines()]
plans={r['plan_id']:r for line in (raw/'inference.jsonl').read_bytes().splitlines()
       if (r:=json.loads(line)).get('event')=='PLAN'}
start=next(r['sim_ns'] for r in rows if r.get('event')=='ARMED')
stop=next(r for r in rows if r.get('event')=='STOP_REQUESTED')
active=[r for r in rows if r.get('event')=='COMMAND_SENT' and start<=r['sim_ns']<=stop['sim_ns']]
missing=[r for r in active if r['reason']=='STEERING_FEASIBLE_LOOKAHEAD_MISSING']
assert missing
first=missing[0]
diagnoses=[]
for row in [first,*[r for r in active if first['sim_ns']-3_000_000_000<=r['sim_ns']<=first['sim_ns']+3_000_000_000][::5]]:
    d=row.get('details',{});source=plans.get(row.get('plan_id'))
    if source is None or not all(k in d for k in ('current_pose','observation_pose')):continue
    pose=TimedBodyPose(**d['current_pose']);observed=TimedBodyPose(**d['observation_pose'])
    ref=prepare_time_reference(TimePlan(row['plan_id'],observed,np.asarray(source['raw_xy_m'])),pose,
        rear_axle_offset_m=(.0010000169277191162,0.))
    points=ref.xy_current_m;v=max(0.,row['speed_mps']);minimum=max(1.,.4+.5*v+v*v/2)
    forward=points[points[:,0]>1e-6]
    radial=np.linalg.norm(forward,axis=1)
    selection=None;failure=None
    try:
        selection=select_polyline_lookahead(points,ref.remaining_sec,minimum_m=minimum,
            maximum_m=minimum+1.,response_length_m=effective_response_length(v,AWSIM_15KMH_POLICY))
    except ValueError as exc:failure=str(exc)
    diagnoses.append(dict(t_s=(row['sim_ns']-start)/1e9,reason=row['reason'],speed_kmh=v*3.6,
        target_kmh=row['target_speed_mps']*3.6,acceleration_mps2=row['acceleration_mps2'],
        minimum_preview_m=minimum,max_forward_radius_m=float(radial.max()) if len(radial) else None,
        endpoint_radius_m=float(np.linalg.norm(points[-1])),
        raw_endpoint_m=source['raw_xy_m'][-1],failure=failure,
        selected=selection,pose=d['current_pose']))
record=dict(stop_request=stop['reason'],first_lookahead_missing_after_arm_s=(first['sim_ns']-start)/1e9,
    first_lookahead_missing_speed_kmh=first['speed_mps']*3.6,sequence=diagnoses,
    proximity_stop_events=sum(r.get('event')=='SCAN_GUARD_REJECTED' for r in rows),
    proximity_observation_events=sum(r.get('event')=='SCAN_GUARD_OBSERVED' for r in rows),
    physical_contact_confirmed=False)
video=json.loads((raw/'video_recording.json').read_bytes());frames=[]
for name,command in [('before',active[max(0,active.index(first)-40)]),('first_missing',first),('stalled',missing[-1])]:
    seek=max(0.,(command['monotonic_ns']-video['start_monotonic_ns'])/1e9)
    output=out/('awsim_stall_'+name+'.png')
    assert not output.exists()
    subprocess.run(['ffmpeg','-nostdin','-v','error','-ss',str(seek),'-i',str(raw/'awsim.mp4'),
        '-frames:v','1',str(output)],check=True,timeout=30)
    assert output.is_file()
    frames.append(dict(file=output.name,video_time_s=seek,sim_after_arm_s=(command['sim_ns']-start)/1e9,
        sha256=hashlib.sha256(output.read_bytes()).hexdigest()))
record['frames']=frames
(out/'stall_diagnosis.json').write_text(json.dumps(record,indent=2)+'\n')
print(json.dumps(record))
