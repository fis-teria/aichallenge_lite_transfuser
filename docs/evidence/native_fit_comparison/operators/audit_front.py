from pathlib import Path
import sys,json,subprocess,math,shutil,argparse
import numpy as np
import torch
repo=Path.cwd();sys.path[:0]=[str(repo),str(repo/'tools')]
from tools.audit_native_corner_collection import audit
from tools.filter_teacher_pose_prefix import filter_audit,verified_bag,sha
from tools.curate_native_teacher_data import curate
from tools.train_time_native_replay import replay_run
from rosbags.highlevel import AnyReader
from tools.curate_native_teacher_data import stamp_ns,yaw_of
ROOT=Path('/home/thistle/e2e_autonomous/runs/mppi_v45_pc10_20260918')
parser=argparse.ArgumentParser();parser.add_argument('--replay-only',action='store_true');args=parser.parse_args()
output=ROOT/'front_validation_v1'
if not args.replay_only:
    output.mkdir(exist_ok=False)
    for collected in sorted((ROOT/'collected').glob('lidar-v45-pc10-front-*')):
        run=collected.name
        audit(collected,ROOT/'front_audit_v1'/run)
        shutil.copytree(ROOT/'front_audit_v1'/run/'pose_prefix',ROOT/'front_pose_prefix_v1'/run)
    subprocess.run([sys.executable,str(Path(__file__).with_name('check_front_clearance.py'))],check=True)
    selection=curate(ROOT,ROOT/'front_curated_v1',run_pattern='lidar-v45-pc10-front-*',
        prefix_directory='front_pose_prefix_v1',clearance_directory='front_prefix_clearance_v1')
else:
    assert output.is_dir() and not list(output.iterdir()),'replay restart expects empty unfinished output'
    selection=json.loads((ROOT/'front_curated_v1/selection_manifest.json').read_text())
for name,digest in selection['output_sha256'].items():assert sha(ROOT/'front_curated_v1'/name)==digest
summaries=[]
for summary in selection['runs']:
    run=summary['run_id'];selected=ROOT/'front_curated_v1'/run;collected=ROOT/'collected'/run
    rowlist=[json.loads(s) for s in (selected/'selected_anchors.jsonl').read_text().splitlines()] if summary['selected'] else []
    coverage=json.loads((ROOT/'front_audit_v1'/run/'corner_coverage.json').read_text())
    result=dict(run_id=run,selected=summary['selected'],uses=summary.get('uses',{}),coverage=coverage,
        split=summary.get('split'),split_group=summary.get('split_group'),
        replay_verified=0,front_0to6m_abs_y_le1m=0,front_0to6m=0,front_0to12m=0,side_or_behind=0,
        labels='observed XY and velocity only; stop and mode invalid',independent_drives=1)
    if rowlist:
        with np.load(selected/'selected_teachers.npz',allow_pickle=False) as z:labels={k:z[k].copy() for k in z.files}
        placement=json.loads((collected/'provenance/scenarios'/f'{run}.json').read_text())['locations'][0]
        assert placement['split']=='train'
        bag,_=verified_bag(collected,run);samples=replay_run(bag,rowlist,labels,
            expected_split='train',expected_group=placement['split_group'])
        torch.save(samples,output/(run+'.pt'));result['replay_verified']=len(samples)
        result['shard_sha256']=sha(output/(run+'.pt'))
        placement=json.loads((collected/'provenance/scenarios'/f'{run}.json').read_text())['locations'][0]
        poses={}
        with AnyReader([bag]) as reader:
            for c,_,blob in reader.messages(connections=[c for c in reader.connections if c.topic=='/localization/kinematic_state']):
                m=reader.deserialize(blob,c.msgtype);p=m.pose.pose
                poses[stamp_ns(m.header.stamp)]=[p.position.x,p.position.y,yaw_of(p.orientation)]
        stamps=np.array(sorted(poses),np.int64);values=np.array([poses[t] for t in stamps]);values[:,2]=np.unwrap(values[:,2]);details=[]
        for r in rowlist:
            t=r['observation_ns'];p=np.array([np.interp(t,stamps,values[:,j]) for j in range(3)])
            c,s=math.cos(p[2]),math.sin(p[2]);x,y=(np.array(placement['map_pose'][:2])-p[:2])@np.array([[c,-s],[s,c]])
            result['front_0to6m_abs_y_le1m']+=int(0<x<=6 and abs(y)<=1)
            result['front_0to6m']+=int(0<x<=6);result['front_0to12m']+=int(0<x<=12);result['side_or_behind']+=int(x<=0)
            details.append(dict(anchor_id=r['anchor_id'],observation_ns=t,object_body_xy_m=[float(x),float(y)],use=r['selection_use']))
        (output/(run+'-geometry.json')).write_text(json.dumps(details,indent=2))
    summaries.append(result);print('VERIFIED_COLLECTION',json.dumps(result),flush=True)
(output/'summary.json').write_text(json.dumps(dict(runs=summaries,selection_manifest_sha256=sha(ROOT/'front_curated_v1/selection_manifest.json'),
    source_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),operator_sha256=sha(Path(__file__)),
    note='No new training or heldout avoidance claim. Same placement groups stay train-only; raw preserved.'),indent=2))
