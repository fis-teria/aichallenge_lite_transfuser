from pathlib import Path
import sys,json,math,hashlib
root=Path('/home/graneple/e2e_autonomous/mppi_close_adjust_20260918')
old=root.with_name('mppi_v45_collection_fix_20260918')
sys.path.insert(0,'/home/graneple/git/autononous_ai/aichallenge-racingkart/scenario_tool')
from scenario_tool import yamlio
from scenario_tool.geometry import ReferenceLine
teacher=ReferenceLine.from_mpc_config(root/'source/integrations/mppi_v45/assets/multi_purpose_mpc_ros/env/final_ver3/in_corce_line.csv',.6,2,True)
receipt=[]
for key,source,distance in [('a6','cone-a01',6.),('a8','cone-a01',8.),('an','cone-a01',None),('b8','cone-b01',8.)]:
    name='lidar-v45-pc10-front-collect10-'+key+'-a01'
    original=old/'scenarios'/('lidar-v45-pc10-front-'+source+'.yaml')
    p=yamlio.load_file(original);p['name']=name;p['description']='10 km/h MPPI V45 teacher, original entry timing; train-only controlled approach'
    placement=p['objects'][0]['pose']['map_xy'];station,lateral=teacher.project(*placement)
    if distance is not None:
        pose=teacher.pose_at(station-distance,0.)
        p['ego']['pose']={'map_xy':[pose.x,pose.y],'yaw':math.degrees(pose.yaw)}
    target=root/'scenarios'/(name+'.yaml');assert not target.exists()
    yamlio.dump_file(target,p)
    meta=json.loads(original.with_suffix('.json').read_text())
    meta.update(cases=[name],run_id=name,target_kmh=10.,early_entry_search=False,
                requested_teacher_reference_start_gap_m=distance,teacher_object_station_m=station,
                source_scenario_sha256=hashlib.sha256(original.read_bytes()).hexdigest(),
                scope='Train-only teacher collection; existing placement split retained; quality audit required')
    (root/'scenarios'/(name+'.json')).write_text(json.dumps(meta,indent=2)+'\n')
    receipt.append(dict(run_id=name,teacher_object_station_m=station,teacher_lateral_m=lateral,ego=p['ego']['pose'],split_group=meta['locations'][0]['split_group']))
with (root/'collect10-scenarios.json').open('x') as f:json.dump(receipt,f,indent=2)
print(json.dumps(receipt))
