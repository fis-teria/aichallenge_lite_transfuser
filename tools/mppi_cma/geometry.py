"""ROS-map raceline offsets (m), rigid calibration, and conservative vehicle/OT overlap."""
from __future__ import annotations

import csv
import math
from pathlib import Path
import numpy as np


def closed_path_offsets(xy_m: np.ndarray, anchors_m: np.ndarray) -> np.ndarray:
    """Return displaced XY [N,2] in m using periodic cubic B-spline offsets [K]."""
    xy=np.asarray(xy_m,dtype=float); anchors=np.asarray(anchors_m,dtype=float)
    if xy.ndim!=2 or xy.shape[1]!=2 or len(xy)<4:
        raise ValueError('xy_m must have shape [N>=4,2]')
    if anchors.ndim!=1 or len(anchors)<4 or not np.isfinite(anchors).all() or not np.isfinite(xy).all():
        raise ValueError('Finite [K>=4] anchors and XY are required')
    segments=np.linalg.norm(np.roll(xy,-1,axis=0)-xy,axis=1)
    if segments.sum()<=1e-6 or segments[:-1].min()<=1e-8:
        raise ValueError('Degenerate closed path')
    s=np.r_[0,np.cumsum(segments[:-1])]
    phase=s/segments.sum()*len(anchors); idx=np.floor(phase).astype(int);t=phase-idx
    weights=[(1-t)**3/6,(3*t**3-6*t*t+4)/6,(-3*t**3+3*t*t+3*t+1)/6,t**3/6]
    offset=sum(w*anchors[(idx+shift)%len(anchors)] for w,shift in zip(weights,[-1,0,1,2]))
    directions=(np.roll(xy,-1,axis=0)-xy)/np.maximum(segments[:,None],1e-8)
    tangent=directions+np.roll(directions,1,axis=0)
    # A duplicated closing point has a zero-length final segment; use its neighbours.
    tangent/=np.maximum(np.linalg.norm(tangent,axis=1)[:,None],1e-8)
    normal=np.column_stack([-tangent[:,1],tangent[:,0]])
    result=xy+offset[:,None]*normal
    assert result.shape==xy.shape and np.isfinite(result).all()
    return result


def generate_reference(source: Path, destination: Path, anchors_m: list[float]) -> None:
    """Keep the source heading convention; only change its geometric tangent delta."""
    with source.open(newline='') as stream:
        reader=csv.DictReader(stream);fields=reader.fieldnames;rows=list(reader)
    xy=np.array([[float(r['x_m']),float(r['y_m'])] for r in rows])
    shifted=closed_path_offsets(xy,np.asarray(anchors_m))
    old_direction=np.roll(xy,-1,axis=0)-np.roll(xy,1,axis=0)
    new_direction=np.roll(shifted,-1,axis=0)-np.roll(shifted,1,axis=0)
    delta=np.arctan2(new_direction[:,1],new_direction[:,0])-np.arctan2(old_direction[:,1],old_direction[:,0])
    s=np.r_[0,np.cumsum(np.linalg.norm(np.diff(shifted,axis=0),axis=1))]
    for i,row in enumerate(rows):
        row['x_m']=repr(float(shifted[i,0]));row['y_m']=repr(float(shifted[i,1]))
        row['s_m']=repr(float(s[i]))
        row['psi_rad']=repr(math.remainder(float(row['psi_rad'])+float(delta[i]),2*math.pi))
    destination.parent.mkdir(parents=True,exist_ok=True)
    with destination.open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerows(rows)


def fit_similarity(source_xy_m: np.ndarray, target_xy_m: np.ndarray) -> dict:
    """Fit 2D rotation/uniform scale/translation from [N>=4,2] metre correspondences."""
    source=np.asarray(source_xy_m,dtype=float);target=np.asarray(target_xy_m,dtype=float)
    if source.shape!=target.shape or source.ndim!=2 or source.shape[1]!=2 or len(source)<4:
        raise ValueError('Require matching [N>=4,2] arrays')
    if not np.isfinite(source).all() or not np.isfinite(target).all():
        raise ValueError('Calibration coordinates must be finite')
    if np.linalg.matrix_rank(source-source.mean(axis=0))<2:
        raise ValueError('Calibration points must span an area')
    design=[]
    for x,y in source:design.extend([[x,-y,1,0],[y,x,0,1]])
    a,b,tx,ty=np.linalg.lstsq(design,target.reshape(-1),rcond=None)[0]
    matrix=np.array([[a,-b],[b,a]]);translation=np.array([tx,ty])
    residual=np.linalg.norm(source@matrix.T+translation-target,axis=1)
    return {'matrix':matrix.tolist(),'translation':translation.tolist(),
            'scale':float(math.hypot(a,b)),'max_residual_m':float(residual.max()),'point_count':len(source)}


def convex_overlap(first: np.ndarray, second: np.ndarray, margin_m: float = 0.0) -> bool:
    """SAT overlap with a conservative gap margin, polygons [N>=3,2] in metres."""
    if margin_m<0 or not math.isfinite(margin_m):raise ValueError('Invalid margin_m')
    polygons=[np.asarray(x,dtype=float) for x in (first,second)]
    if any(x.ndim!=2 or x.shape[1]!=2 or len(x)<3 or not np.isfinite(x).all() for x in polygons):
        raise ValueError('Require finite [N>=3,2] convex polygons')
    for polygon in polygons:
        for edge in np.roll(polygon,-1,axis=0)-polygon:
            length=float(np.linalg.norm(edge))
            if length<1e-9:continue
            axis=np.array([-edge[1],edge[0]])/length
            p=polygons[0]@axis;q=polygons[1]@axis
            if p.max()+margin_m<q.min() or q.max()+margin_m<p.min():return False
    return True


def vehicle_polygon(x_m: float,y_m: float,yaw_rad: float) -> np.ndarray:
    """Conservative rectangle enclosing the frozen MeshCollider relative to base_link."""
    if not all(math.isfinite(x) for x in [x_m,y_m,yaw_rad]):raise ValueError('Nonfinite pose')
    hull=np.array([[-.379,-.769],[1.616,-.769],[1.616,.769],[-.379,.769]])
    c,s=math.cos(yaw_rad),math.sin(yaw_rad)
    return hull@np.array([[c,s],[-s,c]])+np.array([x_m,y_m])
