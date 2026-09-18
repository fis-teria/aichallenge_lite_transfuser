from pathlib import Path
import json,sys,math,hashlib
import numpy as np
from rosbags.highlevel import AnyReader
repo=Path.cwd();sys.path[:0]=[str(repo),str(repo/'tools')]
from tools.filter_teacher_pose_prefix import verified_bag
from tools.curate_native_teacher_data import read_rows,stamp_ns,yaw_of
root=Path('/home/thistle/e2e_autonomous/runs/mppi_v45_pc10_20260918')
selection=root/'expand_alignment_tolerant_v2'
manifest=json.loads((selection/'selection_manifest.json').read_text())
output=Path('/home/thistle/e2e_autonomous/runs/avoidance_expand_replay_20260919')
geometry=[];summaries=[]
for entry in manifest['runs']:
    run=entry['run_id'];bag,hashes=verified_bag(root/'collected'/run,run)
    poses={}
    with AnyReader([bag]) as reader:
        for c,_,blob in reader.messages(connections=[c for c in reader.connections if c.topic=='/localization/kinematic_state']):
            m=reader.deserialize(blob,c.msgtype);p=m.pose.pose
            poses[stamp_ns(m.header.stamp)]=[p.position.x,p.position.y,yaw_of(p.orientation)]
    ts=np.array(sorted(poses),np.int64);ps=np.array([poses[t] for t in ts]);ps[:,2]=np.unwrap(ps[:,2])
    placement=json.loads((root/'collected'/run/'provenance/scenarios'/(run+'.json')).read_text())['locations']
    assert len(placement)==1 and placement[0]['object_type']=='cone'
    center=np.array(placement[0]['map_pose'][:2])
    rows=read_rows(selection/run/'selected_anchors.jsonl')
    with np.load(selection/run/'selected_teachers.npz',allow_pickle=False) as labels:
        subset=[]
        for i,row in enumerate(rows):
            t=row['observation_ns'];assert ts[0]<=t<=ts[-1]
            p=np.array([np.interp(t,ts,ps[:,j]) for j in range(3)])
            c,s=np.cos(p[2]),np.sin(p[2]);body=(center-p[:2])@np.array([[c,-s],[s,c]])
            record=dict(anchor_id=row['anchor_id'],object_body_xy_m=body.tolist(),
                front_0_6m_lateral_1m=bool(0<=body[0]<=6 and abs(body[1])<=1),
                endpoint_y_m=float(labels['xy_m'][i,-1,1]),use=row['selection_use'])
            geometry.append(record);subset.append(record)
    summaries.append(dict(run_id=run,selected=len(rows),
        front_cone=sum(r['use']=='static_cone_xy_speed' and r['object_body_xy_m'][0]>0 for r in subset),
        front_0_6m_lateral_1m=sum(r['front_0_6m_lateral_1m'] for r in subset),
        endpoint_abs_y_gt_020=sum(abs(r['endpoint_y_m'])>.2 for r in subset),
        original_audit=json.loads((root/'expand_audit_v1'/run/'audit.json').read_text())))
assert len(geometry)==140
(output/'geometry.json').write_text(json.dumps(geometry,indent=2))
summary=dict(runs=summaries,selected=140,
    front_cone=sum(r['front_cone'] for r in summaries),
    front_0_6m_lateral_1m=sum(r['front_0_6m_lateral_1m'] for r in summaries),
    pose_source='recorded EKF, allowed map residual; not physical truth',
    counts_are_windows_not_independent_avoidance_events=True)
(output/'coverage_summary.json').write_text(json.dumps(summary,indent=2))
print(json.dumps({k:v for k,v in summary.items() if k!='runs'}))
