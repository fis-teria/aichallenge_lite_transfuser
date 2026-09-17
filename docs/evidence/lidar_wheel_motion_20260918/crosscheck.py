import importlib.util
import json
import sys
from pathlib import Path
import numpy as np

repo=Path('/home/thistle/e2e_autonomous/e2e_lite_transfuser')
sys.path.insert(0,str(repo/'src'))
spec=importlib.util.spec_from_file_location('motion',repo/'tools/diagnose_lidar_wheel_motion.py')
m=importlib.util.module_from_spec(spec);sys.modules[spec.name]=m;spec.loader.exec_module(m)
from aic_transfuser_lite.runtime.lidar_map_localization import BoundaryMap, match_scan

out=Path('/home/thistle/e2e_autonomous/runs/lidar_wheel_motion_20260918/compare03')
r=json.loads((out/'pairs.json').read_text());scans=np.load(out/'selected_scans.npz')
cloud={int(t):p for t,p in zip(scans['stamps'],scans['clouds'])}
windows=[]
for end in [43587393038,45589043940,46590171633,49800000000]:
    i=min(range(len(r)),key=lambda i:abs(r[i]['t1_ns']-end))
    sub=r[max(0,i-4):i+1]
    lidar=np.zeros(3);wheel=np.zeros(3)
    for row in sub:
        lidar=m.compose(lidar,row['lidar']);wheel=m.compose(wheel,row['wheel'])
    seed=m.compose(m.compose(m.inverse(m.MOUNT),lidar),m.MOUNT)
    t0,t1=sub[0]['t0_ns'],sub[-1]['t1_ns']
    direct=m.register(cloud[t0],cloud[t1],seed)
    reverse=m.register(cloud[t1],cloud[t0],m.inverse(seed))
    direct_base=m.base_motion(direct.pose)
    windows.append(dict(t0_s=t0/1e9,t1_s=t1/1e9,steps=len(sub),wheel=wheel.tolist(),
                        lidar_chain=lidar.tolist(),lidar_direct=direct_base.tolist(),
                        direct_minus_wheel=(direct_base-wheel).tolist(),
                        chain_minus_wheel=(lidar-wheel).tolist(),
                        closure=m.compose(direct.pose,reverse.pose).tolist(),
                        support=direct.fraction,weak_ratio=direct.weak_ratio,
                        note='Direct fit seeded from LiDAR-only incremental chain, no wheel prior'))
rows=[json.loads(x) for x in (repo.parent/'runs/lidar_map_runtime_20260918/replay05/records.jsonl').read_text().splitlines()]
idx=next(i for i,x in enumerate(rows) if x['status'].startswith('REJECTED'))
before,first=rows[idx-1:idx+1];pair=next(x for x in r if x['t1_ns']==first['stamp_ns'])
wheel_prior=m.compose(before['match']['pose'],pair['wheel'])
lidar_prior=m.compose(before['match']['pose'],pair['lidar'])
points=cloud[first['stamp_ns']];points=points[np.isfinite(points).all(axis=1)][::3]
course=BoundaryMap.from_lanelet(repo.parent/'runs/lidar_map_alignment_20260918/lanelet2_map.osm')
experiments={}
for name,prior in [('wheel',wheel_prior),('lidar',lidar_prior)]:
    normal=match_scan(course,points,prior);retry=match_scan(course,points,prior,initializing=True)
    experiments[name]=dict(normal_correction_m=normal.correction_m,normal_reason=normal.reason,
                           retry_correction_m=retry.correction_m,retry_reason=retry.reason,
                           estimated_pose=normal.pose.tolist())
chain_l=np.zeros(3);chain_w=np.zeros(3)
for row in r:
    if row['t1_ns']>first['stamp_ns']:
        break
    chain_l=m.compose(chain_l,row['lidar']);chain_w=m.compose(chain_w,row['wheel'])
result=dict(windows=windows,map_prediction_swap=experiments,
            cumulative_to_first_reject=dict(start_s=r[0]['t0_ns']/1e9,end_s=first['stamp_ns']/1e9,
              wheel=chain_w.tolist(),lidar=chain_l.tolist(),difference=(chain_l-chain_w).tolist(),
              note='Diagnostic accumulation includes one weak pair. Not a global localization truth.'))
(out/'crosscheck.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
