"""Render exact paired PP counts without coincident curves or count offsets."""
from pathlib import Path
import hashlib
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root=Path('/home/thistle/e2e_autonomous/runs/time_launch_regression_20260916')
source=root/'baseline/summary.json'
summary=json.loads(source.read_bytes())['near_arm_pp']['all']
ages=['0.0','0.1','0.2','0.3','0.4','0.5']
names=['teacher','old','epoch1','epoch2','epoch3']
counts=np.array([[summary[a][n]['accepted'] for a in ages] for n in names])
assert counts.shape==(5,6)
assert all(summary[a][n]['attempted']==14 for a in ages for n in names)
fig,ax=plt.subplots(figsize=(8.8,4.8),layout='constrained')
ax.imshow(counts,vmin=0,vmax=14,cmap='RdYlGn',aspect='auto')
ax.set(xticks=range(6),xticklabels=ages,yticks=range(5),yticklabels=names,
    xlabel='Observation-to-control age with recorded vehicle motion [s]',
    title='PP-accepted frames / 14 at launch\n14 correlated frames from 2 existing baseline runs; offline only')
for i in range(5):
    for j in range(6):
        ax.text(j,i,f'{counts[i,j]} / 14',ha='center',va='center',color='white' if counts[i,j]>=12 else 'black',fontsize=13)
out=root/'supplement/paired_acceptance_matrix.png'
fig.savefig(out,dpi=160);plt.close(fig)
(root/'supplement/plot_provenance.json').write_text(json.dumps(dict(source=str(source),
    source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    output_sha256=hashlib.sha256(out.read_bytes()).hexdigest(),exact_unoffset_counts=True),indent=2)+'\n')
print('EXACT_COUNT_PLOT_COMPLETE')
