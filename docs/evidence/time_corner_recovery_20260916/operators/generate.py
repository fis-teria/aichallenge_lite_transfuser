"""Generate each planned static reference in the locked native WSL worktree."""
import ops_corner as m
m.remote(r'''
from pathlib import Path
import subprocess,sys,json,hashlib
out=Path('/home/thistle/e2e_autonomous/runs/time_corner_recovery_20260916')
gate=json.loads((out/'test_gate.json').read_text());assert gate['full_exit']==0
assert subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()==gate['commit']
plans=json.loads((out/'plans/plan.json').read_text())['plans']; results=[]
for plan in plans:
 side='left' if plan['split']=='train' else 'right'
 output=out/'references'/plan['name']
 command=[sys.executable,'tools/generate_time_large_recovery_reference.py',
 '--inputs','/home/thistle/e2e_autonomous/runs/time_recovery_collection_20260913/inputs',
 '--normal-run','/home/thistle/e2e_autonomous/raw/time_recovery_sites_20260915/codex-time-recovery-sites-normal-n03',
 '--normal-proof','/home/thistle/e2e_autonomous/runs/time_recovery_separated_20260915/selected_site_plan.json',
 '--plan',str(out/'plans'/(plan['name']+'.json')),'--side',side,'--output',str(output)]
 p=subprocess.run(command,capture_output=True,text=True)
 (out/(plan['name']+'_generation.log')).write_text(p.stdout+p.stderr)
 row=dict(plan=plan['name'],returncode=p.returncode,sites=plan['required_site_ids'],error=p.stderr[-1500:] if p.returncode else None)
 results.append(row);print(json.dumps(row),flush=True)
(out/'map_generation.json').write_text(json.dumps(results,indent=2))
# Plot the explicitly catalogued entry points on the verified reference.
import numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from aic_transfuser_lite.data.time_recovery_collection_v1 import load_pose_course
base=load_pose_course(Path('/home/thistle/e2e_autonomous/runs/time_recovery_collection_20260913/inputs/base.csv'))
bs=np.array([p.s_m for p in base]);xy=np.array([[p.x_m,p.y_m] for p in base]);origin=xy.min(axis=0)
catalog=json.loads(Path('configs/collection/corner_recovery_20260916.json').read_text())
fig,ax=plt.subplots(figsize=(10,10));ax.plot(*(xy-origin).T,color='gray',linewidth=2)
for row in catalog['corners']:
 site=row['site'];s=site['release_s_m'];pos=np.array([np.interp(s,bs,xy[:,i]) for i in range(2)])-origin
 ax.scatter(*pos,c='red' if row['turn_sign']<0 else 'blue');ax.annotate(site['corner_id']+f' {s:.1f}m',pos,xytext=(5,5),textcoords='offset points',fontsize=9)
ax.set_aspect('equal');ax.set_xlabel('map x offset [m]');ax.set_ylabel('map y offset [m]');ax.grid();ax.set_title('11 mandatory corner entry targets (compound bends split)')
fig.tight_layout();fig.savefig(out/'corner_catalog.png',dpi=140)
''',native=True,lock=True,timeout=900)
