from pathlib import Path
from dataclasses import replace
import json,math,subprocess,sys
import numpy as np
sys.path.insert(0,'tools')
from generate_time_large_recovery_reference import measured_normal_trace
from aic_transfuser_lite.data.time_recovery_collection_v1 import load_pose_course
from aic_transfuser_lite.data.recovery_reference_v3 import load_occupancy_map_v3
from aic_transfuser_lite.data.time_recovery_map_body_v1 import body_path_is_free

ROOT=Path('/home/thistle/e2e_autonomous');OUT=ROOT/'runs/time_corner_gap_20260916'
INPUTS=ROOT/'runs/time_recovery_collection_20260913/inputs'
NORMAL=ROOT/'raw/time_recovery_sites_20260915/codex-time-recovery-sites-normal-n03'
PROOF=ROOT/'runs/time_recovery_separated_20260915/selected_site_plan.json'
read=lambda p:json.loads(p.read_bytes())
screen=read(OUT/'candidate_screen.json');schedule=read(OUT/'initial_schedule.json')

# Pack the seventh train singleton into six finite runs without dropping a goal.
updates={'train_lap02_packed':['60:C01','60:C02','60:C09'],
         'train_lap06_packed':['40:C03','60:C05','60:C08']}
for name,keys in updates.items():
    plan=read(OUT/'plans/train_lap01.json')
    plan.update(name=name,goal_keys=keys,candidates=[screen['selected'][k]['site'] for k in keys],
                required_site_ids=[screen['selected'][k]['site']['site_id'] for k in keys],event_cap=len(keys))
    with (OUT/'plans'/(name+'.json')).open('x') as f:json.dump(plan,f,indent=2)
    log=OUT/(name+'_generation.log')
    command=[sys.executable,'tools/generate_time_large_recovery_reference.py','--inputs',str(INPUTS),
        '--normal-run',str(NORMAL),'--normal-proof',str(PROOF),'--plan',str(OUT/'plans'/(name+'.json')),
        '--side','left','--output',str(OUT/'references'/name)]
    with log.open('x') as f:p=subprocess.run(command,stdout=f,stderr=subprocess.STDOUT,timeout=120)
    if p.returncode:raise RuntimeError(log.read_text()[-3000:])
effective=[]
for pair in range(1,7):
    left=f'train_lap{pair:02}'+('_packed' if pair in (2,6) else '')
    right=f'validation_lap{pair:02}'
    effective.append(dict(pair=pair,left=left,right=right,
        train_goals=read(OUT/'plans'/(left+'.json'))['goal_keys'],
        validation_goals=read(OUT/'plans'/(right+'.json'))['goal_keys']))
old_train=[k for p in schedule['plans'] if p['split']=='train' for k in p['goal_keys']]
assert sorted(old_train)==sorted(k for p in effective for k in p['train_goals'])
with (OUT/'effective_schedule.json').open('x') as f:json.dump(dict(maximum_attempts=12,pairs=effective),f,indent=2)
print(json.dumps(effective),flush=True)

# Determine whether the exact requested C06 body states themselves occupy map cells.
trace=measured_normal_trace(NORMAL,read(PROOF)['source_hashes'][NORMAL.name])
base=load_pose_course(INPUTS/'base.csv');occ=load_occupancy_map_v3(INPUTS/'occupancy_grid_map.yaml')
bs=np.array([p.s_m for p in base]);bxy=np.array([[p.x_m,p.y_m] for p in base])
observations=[]
for cm in (20,40,60):
    target=next(r['site'] for r in screen['catalogs'][str(cm)]['corners'] if r['site']['site_id']=='C06')
    for progress_delta in (-.5,0.,.5,1.):
        s=target['release_s_m']+progress_delta
        idx=np.searchsorted(bs,s)-1
        direction=bxy[idx+1]-bxy[idx];direction/=np.linalg.norm(direction)
        left=np.array([-direction[1],direction[0]])
        centre=np.array([np.interp(s,bs,bxy[:,k]) for k in (0,1)])
        nominal_offset=np.interp(s,trace[:,0],trace[:,5])
        nominal_yaw=np.interp(s,trace[:,0],np.unwrap(trace[:,3]))
        rows=[]
        for lateral_delta in (-.05,0.,.05):
            lateral=target['target_offset_m']+lateral_delta
            for heading_delta in (-1.,0.,1.):
                heading=target['target_heading_rad']+math.radians(heading_delta)
                xy=centre+left*(nominal_offset+lateral)
                pose=np.r_[xy,nominal_yaw+heading]
                rows.append(dict(lateral_m=lateral,heading_rad=heading,static_body_free=body_path_is_free(occ,np.stack((pose,pose)))))
        observations.append(dict(cm=cm,base_s_m=s,goal_rows=rows,free_count=sum(r['static_body_free'] for r in rows)))
with (OUT/'c06_goal_geometry.json').open('x') as f:json.dump(dict(diagnostic_only=True,rows=observations),f,indent=2)
print(json.dumps([dict(cm=r['cm'],s=r['base_s_m'],free=r['free_count'],tested=9) for r in observations]),flush=True)
