"""Saved bounded map-observation logs only. No new inference/ROS/sensor reads."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
import numpy as np
from aic_transfuser_lite.control.static_course_map_v4 import load_map


def main() -> None:
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--input',type=Path,required=True)
    ap.add_argument('--map-yaml',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args();log=args.input/'supervisor.jsonl'
    if log.stat().st_size>64*1024**2: raise ValueError('BOUNDED_LOG_REQUIRED')
    rows=[];commands=[]
    with log.open() as f:
        for line in f:
            r=json.loads(line)
            if r.get('event')=='STATIC_MAP_COMPARISON':rows.append(r)
            elif r.get('event')=='COMMAND_SENT':commands.append(r['receipt'])
    if not rows or len(rows)>10:raise ValueError('EXPECTED_ONE_TO_TEN_MAP_COMPARISONS')
    summary=json.loads((args.input/'supervisor_summary.json').read_text())
    course=load_map(args.map_yaml)
    if any(r['comparison']['map_image_sha256']!=course.provenance['image_sha256'] for r in rows):
        raise ValueError('MAP_HASH_MISMATCH')
    args.output.mkdir(parents=True,exist_ok=False)
    comparisons=[]
    for r in rows:
        c=r['comparison'];n=c['valid_hit_count']
        comparisons.append(dict(pose_ns=r['pose_ns'],scan_ns=r['scan_ns'],map_xy_m=c['map_xy_m'],
            map_yaw_rad=c['map_yaw_rad'],centre_class=c['centre_class'],hits=n,
            hit_counts=c['exact_hit_counts'],near_occupied_020m=c['hits_near_occupied_020m'],
            near_fraction=None if not n else c['hits_near_occupied_020m']/n))
    result=dict(schema='STATIC_MAP_AWSIM_COMPARISON_V1',comparisons=comparisons,
        supervisor_log_sha256=hashlib.sha256(log.read_bytes()).hexdigest(),map=course.provenance,
        supervisor_summary=summary,positive_acceleration_sends=sum(c['acceleration_mps2']>0 for c in commands),
        control_publish_count=len(commands),runtime_permission=False,
        static_only_not_collision_free_proof=True,geometry_match='NOT_CERTIFIED')
    (args.output/'summary.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    c=rows[-1]['comparison'];hits=np.asarray(c['hit_map_xy_m']);xy=np.array(c['map_xy_m'])
    origin=np.array(course.origin_xy_yaw[:2]);h,w=course.grid.shape
    if course.origin_xy_yaw[2]!=0:raise ValueError('PLOT_REQUIRES_ZERO_MAP_YAW')
    fig,axes=plt.subplots(1,2,figsize=(13,6))
    pixels=np.where(course.grid==100,0,np.where(course.grid==0,2,1))
    for ax in axes:
        ax.imshow(pixels,cmap=ListedColormap(['#303030','#aaaaaa','#ffffff']),vmin=0,vmax=2,
            extent=[0,w*course.resolution_m,0,h*course.resolution_m],origin='upper',interpolation='nearest')
        if len(hits):ax.scatter(hits[:,0]-origin[0],hits[:,1]-origin[1],s=4,c='#ed6b20',label='Actual LiDAR hits')
        ax.scatter([xy[0]-origin[0]],[xy[1]-origin[1]],c='#009cd9',s=40,label='GNSS/IMU base')
        ax.set(xlabel='Map-local x [m]',ylabel='Map-local y [m]');ax.set_aspect('equal')
    axes[1].set_xlim(xy[0]-origin[0]-6,xy[0]-origin[0]+6)
    axes[1].set_ylim(xy[1]-origin[1]-6,xy[1]-origin[1]+6)
    axes[1].legend(loc='upper right')
    fig.suptitle('Unmodified AWSIM stationary scan vs fixed official raster\nDiagnostic overlay, not runtime clearance')
    fig.tight_layout();fig.savefig(args.output/'map_scan_overlay.png',dpi=130);plt.close(fig)
    print(json.dumps(dict(comparisons=comparisons,positive_acceleration_sends=result['positive_acceleration_sends'])))


if __name__=='__main__':main()
