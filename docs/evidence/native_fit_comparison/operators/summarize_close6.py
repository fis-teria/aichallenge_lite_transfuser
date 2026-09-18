from pathlib import Path
import json,sys,re,math
from collections import Counter
import numpy as np
sys.path.insert(0,str(Path.cwd()/'tools'))
from filter_teacher_pose_prefix import verified_bag,sha
from rosbags.highlevel import AnyReader
root=Path('/home/thistle/e2e_autonomous/runs/mppi_v45_pc10_20260918')
run='lidar-v45-pc10-front-cone-close6-a02';collected=root/'collected'/run;raw=collected/'raw'/run
bag,hashes=verified_bag(collected,run)
manifest=json.loads((collected/'export_manifest.json').read_text())
for name in ('samples.jsonl','compose.log','result.json','d1-result-details.json'):
    assert sha(raw/name)==manifest['files'][f'raw/{run}/{name}']['sha256']
rows=[json.loads(s) for s in (raw/'samples.jsonl').read_text().splitlines()]
rows=[r for r in rows if r.get('time',-1)>=0 and r.get('ego') and r.get('ego_gt',{}).get('source')=='gnss']
placement=json.loads((collected/'provenance/scenarios'/f'{run}.json').read_text())['locations'][0]
def body_object(row):
    p=row['ego'];c,s=math.cos(p['yaw']),math.sin(p['yaw'])
    return ((np.array(placement['map_pose'][:2])-np.array([p['x'],p['y']]))@np.array([[c,-s],[s,c]])).tolist()
commands=Counter();zero_commands=0
with AnyReader([bag]) as reader:
    for c,_,blob in reader.messages(connections=[c for c in reader.connections if c.topic=='/mppi/direct/trajectory_command']):
        m=reader.deserialize(blob,c.msgtype);commands[(m.mode,m.reason,bool(m.emergency_stop))]+=1
collision=[];retime=[]
with (raw/'compose.log').open(errors='replace') as f:
    for line in f:
        if '[MPPI_SELECTION_CANDIDATES]' in line and 'reason:collision,stage:execution_sweep' in line:collision.append(line)
        if '[MPPI_RETIME]' in line and 'first_speed=0.000000' in line:retime.append(line)
report=dict(run_id=run,status='HOLD_FAILED_PASS_NOT_TRAINING_TEACHER',
    scenario_verdict=json.loads((raw/'result.json').read_text())['scenario_verdict'],
    official_penalties=json.loads((raw/'d1-result-details.json').read_text())['penalty_by_kind'],
    progress_start_m=rows[0]['ego_gt']['progress_m'],progress_end_m=rows[-1]['ego_gt']['progress_m'],
    requested_teacher_route_spawn_difference_m=6.,initial_object_in_recorded_ego_frame_m=body_object(rows[0]),
    final_object_in_recorded_ego_frame_m=body_object(rows[-1]),
    max_progress_m=max(r['ego_gt']['progress_m'] for r in rows),
    recorded_driving_duration_sim_s=rows[-1]['ego']['stamp']-rows[0]['ego']['stamp'],
    last_speed_mps=rows[-1]['ego']['speed_mps'],commands=[dict(mode=k[0],reason=k[1],emergency=k[2],messages=v) for k,v in commands.most_common()],
    execution_sweep_collision_candidate_messages=len(collision),zero_first_speed_retime_messages=len(retime),
    limitations='Counts are messages, not independent failures. Planner collision rejection is not an observed physical collision. One 6 m central-obstacle start condition; no general teacher incapability claim.',
    source_bag_sha256=hashes,operator_sha256=sha(Path(__file__)))
(root/'front_close6_summary.json').write_text(json.dumps(report,indent=2))
print(json.dumps({k:v for k,v in report.items() if k not in ('source_bag_sha256','commands')},indent=2))
