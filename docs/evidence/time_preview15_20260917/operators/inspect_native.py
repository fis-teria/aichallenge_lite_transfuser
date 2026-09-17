"""Recorded-run diagnostics and faithful screenshot conversion; native WSL only."""
from collections import Counter
from pathlib import Path
import hashlib
import json
import struct

import numpy as np
from PIL import Image
from aic_transfuser_lite.evaluation.time_clearance_v1 import scan_margin

root=Path('/home/thistle/e2e_autonomous/runs/time_preview15_20260917')
raw=root/'raw/codex-time-timepreview15-lap01'
out=root/'evaluation'
manifest={r['path']:r for r in json.loads((raw/'transfer_manifest.json').read_text())}

def read_rows(name):
    data=(raw/name).read_bytes()
    assert hashlib.sha256(data).hexdigest()==manifest[name]['sha256']
    return [json.loads(line) for line in data.splitlines()]

control=read_rows('control.jsonl'); inference=read_rows('inference.jsonl')
armed=next(r['sim_ns'] for r in control if r['event']=='ARMED')
tracking=[r for r in control if r['event']=='COMMAND_SENT' and r['reason']=='TIME_PATH_TRACKING']
moving=[r['speed_mps']*3.6 for r in tracking if r['speed_mps']>.1]
host=json.loads((raw/'host_result.json').read_text())
result=dict(scope='Observed curvature-adaptive model-driven trial with explicitly capped diagnostic stopping horizon',
    host_status=host['status'], cruise_ceiling_kmh=15.0, moving_speed_median_kmh=float(np.median(moving)) if moving else None,
    moving_speed_max_kmh=max(moving) if moving else None,
    tracking_commands=len(tracking), tracking_extended_lookahead=sum(bool(r['details']['lookahead_selection']['extended_search']) for r in tracking),
    active_command_reasons=dict(Counter(r['reason'] for r in control if r['event']=='COMMAND_SENT' and r['sim_ns']>=armed)),
    stop_confirmed=host['last_control']['stop_confirmed'], raw_path_subscribers=host.get('rviz_path_subscribers'),
    inference_checkpoint_sha256=sorted({r['checkpoint_sha256'] for r in inference if r['event']=='PLAN'}))
guards=[]; observations=[]
for row in control:
    if row['event'] not in ('SCAN_GUARD_REJECTED','SCAN_GUARD_OBSERVED'):continue
    motion=row['motion_observation']
    proof=scan_margin(row['scan'], row['scan_in_current_rear'], speed_mps=row['speed_mps'],
        measured_rad=row['measured_steer_rad'],issued_rad=row['issued_steer_rad'],previous_rad=row['previous_steer_rad'],
        yaw_rate_radps=motion['heading_rate_radps'],lateral_mps=motion['reported_lateral_mps'],
        vehicle_model_policy='awsim_understeer_15kmh_trial_v1',
        stopping_distance_policy='awsim_cap_1m_diagnostic_v1')
    assert proof['reason']==row['reason']
    (guards if row['event']=='SCAN_GUARD_REJECTED' else observations).append(dict(sim_after_arm_s=(row['sim_ns']-armed)/1e9,reason=row['reason'],replay=proof))
result['scan_rejections']=guards
result['scan_log_only_observations']=observations
result['scan_would_stop_commands']=sum(bool(r['details']['obstacle_guard'].get('would_stop_reason')) for r in tracking)
result['scan_occupancy_policy']='log_only_awsim_v1'
guard_details=[r['details']['obstacle_guard'] for r in tracking]
assert all(g['stopping_distance_policy']=='awsim_cap_1m_diagnostic_v1' and g['actual_speed_stopping_envelope'] is False for g in guard_details)
result['maximum_recorded_stopping_travel_m']=max(g['stopping_travel_m'] for g in guard_details)
result['stopping_distance_policy']='awsim_cap_1m_diagnostic_v1'
(out/'diagnostics.json').write_text(json.dumps(result,indent=2,allow_nan=False))
available=sorted(raw.glob('rviz_drive_*.xwd'))
targets=available[-1:]+([raw/'rviz_after_freeze.xwd'] if (raw/'rviz_after_freeze.xwd').exists() else [])
images=[]
for path in targets:
    data=path.read_bytes();assert hashlib.sha256(data).hexdigest()==manifest[path.name]['sha256']
    h=struct.unpack('>25I',data[:100])
    assert h[1:4]==(7,2,24) and h[6:8]==(0,0) and h[11] in (24,32)
    assert h[13] in (4,5) and h[14:17]==(0xFF0000,0xFF00,0xFF)
    if h[13]==5:
        for i in range(h[19]):
            pixel,red,green,blue,flags,_=struct.unpack('>IHHHBB',data[h[0]+i*12:h[0]+(i+1)*12])
            for mask,shift,color,flag in [(0xFF0000,16,red,1),(0xFF00,8,green,2),(0xFF,0,blue,4)]:
                if flags&flag:assert color==((pixel&mask)>>shift)*257
    pixels=data[h[0]+h[19]*12:]
    assert len(pixels)==h[12]*h[5] and h[12]>=h[4]*(h[11]//8)
    mode='BGR' if h[11]==24 else 'BGRX'
    dst=out/(path.stem+'.png');assert not dst.exists()
    Image.frombytes('RGB',(h[4],h[5]),pixels,'raw',mode,h[12],1).save(dst)
    images.append(dict(source=path.name,source_sha256=manifest[path.name]['sha256'],output=dst.name,
        output_sha256=hashlib.sha256(dst.read_bytes()).hexdigest(),size=[h[4],h[5]],bits_per_pixel=h[11],generated=False,resized=False))
(out/'image_conversion.json').write_text(json.dumps(images,indent=2))
print(json.dumps(result))
print(json.dumps(images))
