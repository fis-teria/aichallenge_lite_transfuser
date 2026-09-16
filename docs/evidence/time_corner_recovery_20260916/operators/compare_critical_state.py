"""Compare accepted new teachers to the same frozen failure-state queries."""
from pathlib import Path
from dataclasses import asdict
import argparse
import hashlib
import json
import sys

import numpy as np

REPO=Path.cwd()
sys.path.insert(0,str(REPO/'tools'))
import analyze_time_corner_learning as prior
from rosbags.typesys import Stores,get_typestore


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--suffix',required=True);args=ap.parse_args()
    root=REPO.parent;out=root/'runs/time_corner_recovery_20260916'
    coverage=prior.read(root/'runs/time_corner4_learning_diagnosis_20260916/coverage.json')
    config=prior.read(REPO/'configs/control/time_path_vehicle_model_5kmh_20260913.json')
    reference,base,proof=prior.nominal_reference(root,config)
    assert proof==coverage['reference']
    store=get_typestore(Stores.ROS2_HUMBLE);states=[];sources={}
    for summary in sorted(out.glob('*_collection_summary.json')):
        report=prior.read(summary)
        if not report['accepted']:continue
        name=report['run_id'];raw=root/'raw/time_corner_recovery_20260916'/name
        # Recheck the raw bag identity used for the already accepted teachers.
        expected=prior.read(raw/'transfer_manifest.json')['bag/bag_0.db3']['sha256']
        assert prior._sha(raw/'bag/bag_0.db3')==expected
        anchors=[json.loads(s) for s in (out/'materialized'/name/'anchors.jsonl').read_text().splitlines()]
        speeds={r['anchor_id']:r['speed_mps'] for r in prior.read(out/(name+'_anchor_states.json'))}
        ids=sorted({i for a in anchors for i in a['observation_pose_row_ids']});messages={}
        with prior.connection(raw/'bag/bag_0.db3') as con:
            types=dict(con.execute('select id,type from topics'))
            for start in range(0,len(ids),400):
                chunk=ids[start:start+400]
                for seq,receipt,topic,data in con.execute('select id,timestamp,topic_id,data from messages where id in ('+','.join('?' for _ in chunk)+')',chunk):
                    assert types[topic]=='nav_msgs/msg/Odometry'
                    messages[seq]=(store.deserialize_cdr(data,types[topic]),int(receipt))
        assert len(messages)==len(ids) and len(anchors)==report['accepted']
        for a in anchors:
            pairs=[(prior.message_pose(messages[i][0],a['epoch']),messages[i][1]) for i in a['observation_pose_row_ids']]
            pose=prior.replay_observation_pose(pairs,observation_ns=a['observation_ns'],freeze_receipt_ns=a['freeze_ns'])
            states.append(dict(run_id=name,split=report['split'],anchor_id=a['anchor_id'],site_id=a['recovery_site_id'],
                **prior.locate(pose,speeds[a['anchor_id']],reference,base)))
        sources[name]=dict(bag_sha256=expected,accepted=report['accepted'],summary_sha256=prior._sha(summary))
    counts=[]
    for query in coverage['runtime_queries']:
        row=dict(time_s=query['time_s'],base_s_m=query['base_s_m'])
        for split in ('train','validation'):
            selected=[r for r in states if r['split']==split]
            row[split]={key:len(prior.neighborhood(selected,query,tol)) for key,tol in prior.NEIGHBORHOODS.items()}
        counts.append(row)
    result=dict(sources=sources,reference=proof,queries=coverage['runtime_queries'],matched_new_anchors=counts,
        comparison='same_r30_observed_frame_and_original_fixed_tolerances',model_training_performed=False)
    prior.write(out/('critical_state_'+args.suffix+'.json'),result)
    prior.write(out/('critical_anchor_states_'+args.suffix+'.json'),states)
    print(json.dumps(counts),flush=True)


if __name__=='__main__':main()
