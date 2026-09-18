from pathlib import Path
import json,hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path('/home/thistle/e2e_autonomous/runs/mppi_v45_pc10_20260918')
fig,axes=plt.subplots(1,2,figsize=(10,4),layout='constrained')
for suffix,label,color in [('close-entry-control-a01','Near start / OFF','#3675b6'),('close-entry-a02','Near start / ON','#d85642'),('entry-normal-a01','Normal approach / ON','#34885b')]:
    name='lidar-v45-pc10-front-'+suffix; base=root/'collected'/name
    manifest=json.loads((base/'export_manifest.json').read_text())
    def checked(rel):
        data=(base/rel).read_bytes()
        assert hashlib.sha256(data).hexdigest()==manifest['files'][rel]['sha256']
        return data.decode()
    rows=[json.loads(s) for s in checked('raw/'+name+'/samples.jsonl').splitlines()]
    rows=[r for r in rows if r.get('time',-1)>=0 and r.get('ego') and r.get('ego_gt',{}).get('source')=='gnss']
    placement=json.loads(checked('provenance/scenarios/'+name+'.json'))['locations'][0]['map_pose'][:2]
    if suffix!='entry-normal-a01':
        t=np.array([r['ego']['stamp']-rows[0]['ego']['stamp'] for r in rows]);p=np.array([r['ego_gt']['progress_m']-rows[0]['ego_gt']['progress_m'] for r in rows])
        axes[0].plot(t[t<=60],p[t<=60],label=label,color=color)
    xy=np.array([[r['ego']['x'],r['ego']['y']] for r in rows])-placement
    axes[1].plot(xy[:,0],xy[:,1],label=label,color=color)
axes[0].set(xlabel='Driving time (simulation s)',ylabel='GNSS reference progress from start (m)',xlim=(0,60),ylim=(0,3.1),title='Near start: neither setting passed')
axes[0].legend(fontsize=8);axes[0].grid(alpha=.25)
axes[1].scatter([0],[0],s=55,marker='^',color='#ee9b21',label='Native cone placement')
axes[1].set(xlabel='Map X relative to cone (m)',ylabel='Map Y relative to cone (m)',xlim=(-10,10),ylim=(-10,10),title='Recorded GNSS paths (not predicted paths)')
axes[1].set_aspect('equal');axes[1].legend(fontsize=7);axes[1].grid(alpha=.25)
fig.suptitle('MPPI V45 / target 5 km/h / one run per condition',fontsize=12)
fig.savefig(root/'close_entry_comparison.png',dpi=160)
