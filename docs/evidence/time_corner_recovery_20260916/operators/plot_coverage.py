"""Plot final measured coverage separately from the mandatory collection plan."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from aic_transfuser_lite.data.time_recovery_collection_v1 import load_pose_course

out = Path('/home/thistle/e2e_autonomous/runs/time_corner_recovery_20260916')
coverage = json.loads((out / 'coverage_final.json').read_bytes())
index = json.loads((out / 'collection_index.json').read_bytes())
base = load_pose_course(Path('/home/thistle/e2e_autonomous/runs/time_recovery_collection_20260913/inputs/base.csv'))
progress = np.asarray([p.s_m for p in base])
xy = np.asarray([[p.x_m, p.y_m] for p in base])
origin = xy.min(axis=0)
acquired = {e['corner_id'] for r in index['runs'] for e in r['teacher_anchors_per_event'] if e['accepted']}
fig, ax = plt.subplots(figsize=(10, 10))
ax.plot(*(xy - origin).T, color='#687487', linewidth=2, zorder=1)
for corner, row in coverage['corners'].items():
    site = row['target']
    s = site['release_s_m']
    point = np.asarray([np.interp(s, progress, xy[:, i]) for i in range(2)]) - origin
    complete = bool(row['train'] and row['validation'])
    color = '#208854' if complete else '#c88710' if corner in acquired else '#ae3440'
    ax.scatter(*point, color=color, s=60, zorder=2)
    offset = {'C05': (-98, 12), 'C06': (7, -16), 'C10': (7, -16)}.get(corner, (7, 6))
    ax.annotate(corner + f'  {s:.1f} m', point, xytext=offset, textcoords='offset points', fontsize=9)
extra = {s['corner_id']: s for r in index['runs'] for s in r['sites'] if s['corner_id'] not in coverage['corners']}
for corner, site in extra.items():
    s = site['release_s_m']
    point = np.asarray([np.interp(s, progress, xy[:, i]) for i in range(2)]) - origin
    ax.scatter(*point, color='#6852a3', s=60, marker='s', zorder=3)
    ax.annotate(corner + f'  {s:.1f} m (additional)', point, xytext=(7, -18), textcoords='offset points', fontsize=9)
handles = [Line2D([], [], color=color, marker=marker, linestyle='', label=label) for color, marker, label in [
    ('#208854', 'o', 'Entry state qualified in train and validation'),
    ('#c88710', 'o', 'Recovery teachers acquired; entry coverage incomplete'),
    ('#ae3440', 'o', 'No accepted recovery teachers'),
    ('#6852a3', 's', 'Additional failure-location target')]]
ax.legend(handles=handles, loc='best', fontsize=9)
ax.set_aspect('equal')
ax.set_xlabel('Map x offset [m]')
ax.set_ylabel('Map y offset [m]')
ax.grid(alpha=.25)
ax.set_title('Corner recovery collection\nEntry-state coverage of accepted teacher runs')
fig.tight_layout()
fig.savefig(out / 'coverage_final.png', dpi=140)
print(out / 'coverage_final.png')
