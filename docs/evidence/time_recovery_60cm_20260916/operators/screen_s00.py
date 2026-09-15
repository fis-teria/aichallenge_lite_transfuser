"""Screen longer settling using existing allowed parameters, retaining guards."""
import ops60 as m
m.remote('OUT='+repr(m.OUT)+'\n'+r'''
from pathlib import Path
from dataclasses import asdict
import json,subprocess,sys,importlib.util
from aic_transfuser_lite.data.time_large_recovery_v1 import LargeRecoveryConfig
from aic_transfuser_lite.data.time_large_recovery_reference_v1 import preparation_course
from aic_transfuser_lite.data.recovery_reference_v3 import load_occupancy_map_v3
from aic_transfuser_lite.data.time_recovery_collection_v1 import load_pose_course
spec=importlib.util.spec_from_file_location('gen','tools/generate_time_large_recovery_reference.py');gen=importlib.util.module_from_spec(spec);spec.loader.exec_module(gen)
out=Path(OUT);inputs=out.parent/'time_recovery_collection_20260913/inputs'
normal_run=Path('/home/thistle/e2e_autonomous/raw/time_recovery_sites_20260915/codex-time-recovery-sites-normal-n03')
proof=out.parent/'time_recovery_separated_20260915/selected_site_plan.json'
normal=gen.measured_normal_trace(normal_run,json.loads(proof.read_text())['source_hashes'][normal_run.name])
base=load_pose_course(inputs/'base.csv');occupancy=load_occupancy_map_v3(inputs/'occupancy_grid_map.yaml')
original=json.loads((out/'d60_g03_left_initial_plan.json').read_text());reports=[]
for release in (120.,122.,124.,126.):
    plan=json.loads(json.dumps(original));site=next(s for s in plan['candidates'] if s['site_id']=='S00');site.update(release_s_m=release,settle_distance_m=4.)
    cfg=LargeRecoveryConfig(tuple(sorted(plan['candidates'],key=lambda s:s['release_s_m'])),plan['seed'],3)
    entry=normal[(normal[:,0]>=release-14)&(normal[:,0]<=release-11)]
    row=dict(release_s_m=release,start_s_m=release-12,entry_normal_speed_min=float(entry[:,4].min()),entry_normal_speed_max=float(entry[:,4].max()))
    try:
        _,screen=preparation_course(base,normal,cfg,occupancy);row.update(map_pass=True,map_screen=screen)
    except ValueError as e:row.update(map_pass=False,error=str(e))
    reports.append(row)
valid=[r for r in reports if r['map_pass'] and 1.15<=r['entry_normal_speed_min']<=r['entry_normal_speed_max']<=1.4]
report=dict(candidates=reports,selected_tag=None,scope='Static screening; no relaxed goal, deadline, speed, or monitor')
if valid:
    selected=min(valid,key=lambda r:r['release_s_m']);release=selected['release_s_m']
    plan=json.loads(json.dumps(original));next(s for s in plan['candidates'] if s['site_id']=='S00').update(release_s_m=release,settle_distance_m=4.)
    tag='d60_g03_left_s00_'+str(int(release))+'_s4';p=out/(tag+'_plan.json');p.write_text(json.dumps(plan,indent=2)+'\n')
    cmd=[sys.executable,'tools/generate_time_large_recovery_reference.py','--inputs',str(inputs),'--normal-run',str(normal_run),'--normal-proof',str(proof),'--plan',str(p),'--side','left','--output',str(out/tag)]
    r=subprocess.run(cmd,capture_output=True,text=True,timeout=180);(out/(tag+'_generate.log')).write_text(r.stdout+r.stderr);assert r.returncode==0,r.stderr
    report.update(selected=selected,selected_tag=tag,command=cmd)
(out/'s00_candidate_screen.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report))
''',native=True,lock=True,timeout=180)
