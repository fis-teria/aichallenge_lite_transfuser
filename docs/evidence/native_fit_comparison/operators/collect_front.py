from pathlib import Path
import sys,json,copy,subprocess,math,argparse
ROOT=Path('/home/graneple/e2e_autonomous/mppi_v45_collection_fix_20260918')
REPO=Path('/home/graneple/git/autononous_ai/aichallenge-racingkart')
sys.path.insert(0,str(REPO/'scenario_tool'))
from scenario_tool.context import Context
from scenario_tool import calibration,yamlio
ctx=Context(REPO); transform=calibration.load(ctx,require=True); line=ctx.map_paths().reference_line()
p=argparse.ArgumentParser();p.add_argument('--case',choices=['cone-a01','cone-a02','box-a01','cone-b01','empty5-a01','cone-close6-a02'],default='cone-a01');p.add_argument('--metadata-only',action='store_true');a=p.parse_args()
run=('lidar-v45-pc10-counterfactual-' if a.case.startswith('empty') else 'lidar-v45-pc10-front-')+a.case
base=yamlio.load_file(ROOT/'scenarios/lidar-v45-pc10-box-r2a.yaml')
x,y=transform.to_map(359.3672,-15.9015);yaw=transform.scenario_yaw_to_map(-126.5927)
if a.case in ['box-a01','cone-b01','empty5-a01','cone-close6-a02']:
    original=base['objects'][0]['pose'];x,y=original['map_xy'];yaw=original['yaw_deg']
s,d=line.project(x,y)
base.update(name=run,description='Single cone at the exact failed E2E placement; approach, pass and recovery collection.',seed=975045201)
kind='box' if a.case.startswith('box') else 'cone'
base['objects']=[dict(id=kind+'_1',type=kind,pose=dict(map_xy=[x,y],yaw_deg=yaw))]
if a.case.startswith('empty'):base['objects']=[]
if a.case=='cone-close6-a02':
    from scenario_tool.geometry import ReferenceLine
    teacher=ReferenceLine.from_mpc_config(ROOT/'source/integrations/mppi_v45/assets/multi_purpose_mpc_ros/env/final_ver3/in_corce_line.csv',.6,2,True)
    ego=teacher.pose_at(82.355-6.,0.)
    base['ego']['pose']=dict(map_xy=[ego.x,ego.y],yaw=math.degrees(ego.yaw))
    base['description']='Single cone, ego begins 6 route metres before the obstacle, cap 5 km/h. Collect close approach response.'
base['expect']['finish']['ego_reference_s_greater_than']=s+25
path=ROOT/'scenarios'/f'{run}.yaml'
if not a.metadata_only:
    assert not path.exists();yamlio.dump_file(path,base)
group='failed_e2e_cone_location' if a.case.startswith('cone-a') else 'straight_b_center'
(path.with_suffix('.json')).write_text(json.dumps(dict(cases=[run],split='train',scenario_group=group,monitor_length_m=line.length,locations=[dict(object_id=kind+'_1',object_type=kind,site=group,split_group=group,split='train',placement_id=group,map_pose=[x,y,math.radians(yaw)],yaw_unit='rad',monitor_s_m=s,monitor_lateral_m=d)],scope='native single object, no simulator modification; train-only diagnostic collection'),indent=2))
if a.case.startswith('empty'):
    path.with_suffix('.json').write_text(json.dumps(dict(cases=[run],locations=[],purpose='No-object paired 5 km/h teacher control; comparison only, not an obstacle event'),indent=2))
if a.metadata_only:sys.exit(0)
command=['python3',str(ROOT/'source/tools/collect_mppi_v45.py'),'--awsim-repo',str(REPO),'--runtime',str(ROOT/'runtime'),'--scenario',str(path),'--run-id',run,'--speed-cap-kmh','10' if a.case=='cone-a02' else '5','--wall-timeout-s','480','--run-budget-gib','0.75','--free-reserve-gib','2','--rviz','--execute']
print(json.dumps(dict(command=command,object_map_xy=[x,y],monitor_s=s)),flush=True)
subprocess.run(command,check=True)

