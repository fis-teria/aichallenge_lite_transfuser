"""Offline static-map inventory and picture, no ROS/controller/model inputs."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
import numpy as np
from scipy.ndimage import label, find_objects
from aic_transfuser_lite.control.static_course_map_v4 import load_map, OCCUPIED, FREE, UNKNOWN


def main() -> None:
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-yaml',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--source-commit',required=True)
    parser.add_argument('--source-path',required=True)
    args=parser.parse_args()
    course=load_map(args.map_yaml)
    args.output.mkdir(parents=True,exist_ok=False)
    occupied=course.grid==OCCUPIED
    labels,n=label(occupied)
    counts=np.bincount(labels.ravel())
    components=[]
    for index, slices in enumerate(find_objects(labels),1):
        if slices is None: continue
        rows,cols=slices
        components.append(dict(id=index,occupied_cells=int(counts[index]),
            area_m2=float(counts[index]*course.resolution_m**2),
            pixel_bbox_xy_half_open=[cols.start,rows.start,cols.stop,rows.stop],
            semantic_type='UNKNOWN', meaning='4-connected occupied cells; not an object count'))
    summary=dict(schema='V4_STATIC_MAP_INVENTORY_V1',source_commit=args.source_commit,
        source_path=args.source_path,provenance=course.provenance,
        shape_hw=list(course.grid.shape),resolution_m=course.resolution_m,
        cell_counts={name:int(np.count_nonzero(course.grid==code)) for name,code in
                     [('occupied',OCCUPIED),('free',FREE),('unknown',UNKNOWN)]},
        occupied_components=n,components=components,runtime_permission=False,
        occupied_is_static_obstacle_candidate=True,free_is_map_only_not_live_clearance=True)
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    np.savez_compressed(args.output/'static_grid.npz',occupancy=course.grid,
                       origin_xy_yaw=course.origin_xy_yaw,resolution_m=course.resolution_m)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    display=np.where(course.grid==OCCUPIED,0,np.where(course.grid==FREE,2,1))
    fig,ax=plt.subplots(figsize=(9,8))
    h,w=course.grid.shape
    ax.imshow(display,cmap=ListedColormap(['#202020','#b2b2b2','#ffffff']),vmin=0,vmax=2,
              extent=[0,w*course.resolution_m,0,h*course.resolution_m],origin='upper',interpolation='nearest')
    ax.set(xlabel='Map-local x offset [m]',ylabel='Map-local y offset [m]',
           title='Static map only: black occupied / white free / gray unknown\nNo AWSIM-frame binding; no motion permission')
    fig.tight_layout();fig.savefig(args.output/'static_map.png',dpi=140);plt.close(fig)
    print(json.dumps({k:summary[k] for k in ('shape_hw','cell_counts','occupied_components','runtime_permission')}))


if __name__=='__main__':main()
