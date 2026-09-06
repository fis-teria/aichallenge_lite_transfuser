"""Polyline-only preparation; no smoothing, extrapolation or teacher support."""
import numpy as np
from .spatial_tracking_contracts_v4 import SpatialPathCandidate, PreparedPath


def wrap(a):
    return (np.asarray(a)+np.pi)%(2*np.pi)-np.pi


def transform(xy: np.ndarray, pose: np.ndarray, inverse: bool=False) -> np.ndarray:
    c,s=np.cos(pose[2]),np.sin(pose[2]); r=np.array([[c,-s],[s,c]])
    return (xy-pose[:2])@r if inverse else xy@r.T+pose[:2]


def sample_path(path: PreparedPath, s: np.ndarray) -> tuple:
    q=np.clip(np.asarray(s),0,path.actual_s[-1])
    i=np.clip(np.searchsorted(path.actual_s,q,side='right')-1,0,len(path.world_xy)-2)
    f=(q-path.actual_s[i])/(path.actual_s[i+1]-path.actual_s[i])
    xy=path.world_xy[i]+f[...,None]*(path.world_xy[i+1]-path.world_xy[i])
    return xy,path.heading[i],i


def footprint(state: np.ndarray, cfg: dict) -> np.ndarray:
    front=cfg['wheelbase_m']+cfg['front_overhang_m']; rear=cfg['rear_overhang_m']; w=cfg['body_width_m']/2
    return transform(np.array([[-rear,-w],[front,-w],[front,w],[-rear,w]]),state[:3])


def region_clearance(state: np.ndarray, cfg: dict, scene: dict) -> float:
    """SAT signed separation for rectangles; positive outside, negative overlap.
    Region min margin uses all footprint corners. UNKNOWN uses +inf only internally.
    """
    body=footprint(state,cfg); margins=[]
    if scene.get('region') is not None:
        xmin,ymin,xmax,ymax=scene['region']
        margins.extend([body[:,0].min()-xmin,xmax-body[:,0].max(),body[:,1].min()-ymin,ymax-body[:,1].max()])
    for xmin,ymin,xmax,ymax in scene.get('obstacles',[]):
        box=np.array([[xmin,ymin],[xmax,ymin],[xmax,ymax],[xmin,ymax]])
        axes=np.array([[1,0],[0,1],[np.cos(state[2]),np.sin(state[2])],[-np.sin(state[2]),np.cos(state[2])]])
        bp=body@axes.T; op=box@axes.T
        separation=np.maximum(op.min(axis=0)-bp.max(axis=0),bp.min(axis=0)-op.max(axis=0))
        margins.append(float(separation.max()))
    return float(min(margins)) if margins else float('inf')


def prepare(candidate: SpatialPathCandidate, cfg: dict, scene: dict) -> PreparedPath:
    raw=candidate.raw_xy
    diag=dict(raw_hash=candidate.raw_hash,frame=candidate.frame,origin_separate=True,
              nominal_is_actual_s=False,clearance_verified=None if scene.get('saved') else bool(scene.get('region')),
              environment='SAVED_PATH_OPEN_SPACE_ASSUMED' if scene.get('saved') else 'SYNTHETIC',
              geometry_support_m=cfg['geometry_support_m'],removed_duplicates=[],interpolation='ONLY_ON_SOURCE_POLYLINE',
              interpolation_off_polyline_error_m=0.0,corner_cut_smoothing_applied=False)
    def rejected(reason):
        return PreparedPath(np.empty((0,2)),np.empty(0),np.empty(0,dtype=int),np.empty(0),np.empty(0),diag,reason)
    if raw.ndim!=2 or raw.shape[1]!=2 or len(raw)<2 or raw.dtype!=np.float32: return rejected('SHAPE_DTYPE')
    if candidate.units!='m' or candidate.frame!='base_link@t_obs': return rejected('FRAME_OR_UNIT')
    if candidate.nominal_s.shape!=(len(raw),) or not np.isfinite(candidate.nominal_s).all(): return rejected('NOMINAL_SHAPE_FINITE')
    if not np.isfinite(raw).all(): return rejected('NONFINITE_NO_BRIDGE')
    points=[np.zeros(2)]; indices=[-1]
    for j,point in enumerate(raw.astype(np.float64)):
        distance=np.linalg.norm(point-points[-1])
        if distance<cfg['minimum_segment_m']:
            diag['removed_duplicates'].append(j); continue
        if distance>cfg['maximum_segment_m']: return rejected('DISCONTINUOUS_OR_ORIGIN_DISCONNECTED')
        points.append(point); indices.append(j)
    if len(points)<3: return rejected('ORIGIN_CONCENTRATED')
    xy=np.array(points); ds=np.linalg.norm(np.diff(xy,axis=0),axis=1); actual=np.r_[0,np.cumsum(ds)]
    heading=np.arctan2(np.diff(xy,axis=0)[:,1],np.diff(xy,axis=0)[:,0])
    turn=wrap(np.diff(heading)); support=(ds[:-1]+ds[1:])/2
    k=turn/support
    diag.update(raw_actual_length_m=float(actual[-1]),min_segment_m=float(ds.min()),
                turn_support_m=support.tolist(),max_abs_local_curvature=float(np.max(np.abs(k))))
    if np.any(np.abs(turn)>cfg['cusp_angle_rad']): return rejected('CUSP_OR_REVERSAL')
    def cross(a,b): return a[0]*b[1]-a[1]*b[0]
    for i in range(len(xy)-1):
        for j in range(i+2,len(xy)-1):
            a,b,c,d=xy[i],xy[i+1],xy[j],xy[j+1]
            if cross(b-a,c-a)*cross(b-a,d-a)<0 and cross(d-c,a-c)*cross(d-c,b-c)<0:
                return rejected('SELF_INTERSECTION')
    # No local tiny denominator is hidden by a wide-window smoother.
    if np.max(np.abs(k))>np.tan(cfg['steering_limit_rad'])/cfg['wheelbase_m']:
        return rejected('STEERING_GEOMETRY_LIMIT')
    path=PreparedPath(xy,actual,np.array(indices),heading,np.r_[k[0],k,k[-1]][:len(xy)],diag,None)
    queries=np.arange(0,actual[-1]+1e-10,cfg['footprint_sample_m'])
    checks=[]
    for s in queries:
        p,y,_=sample_path(path,np.array(s)); checks.append(region_clearance(np.r_[p,y,0,0],cfg,scene))
    bad=next((j for j,c in enumerate(checks) if c<=0),None)
    usable=actual[-1]
    if bad is not None:
        usable=max(0,float(queries[bad])-cfg['footprint_sample_m'])
        diag['trim_reason']='KNOWN_FOOTPRINT_BOUNDARY'
    else: diag['trim_reason']=None
    diag.update(reference_footprint_clearance_samples=checks,usable_prefix_s_m=usable,
                trimmed_length_m=float(actual[-1]-usable))
    return path


def project_progress(path: PreparedPath, state: np.ndarray, previous_s: float, cfg: dict) -> dict:
    """Reachability-bounded segment search with heading penalty, never moves plant."""
    lo=max(0,previous_s-.05); hi=min(path.actual_s[-1],previous_s+cfg['maximum_speed_mps']*cfg['controller_dt_s']+.15)
    choices=[]
    for i in range(len(path.world_xy)-1):
        if path.actual_s[i]>hi or path.actual_s[i+1]<lo: continue
        a,b=path.world_xy[i:i+2]; d=b-a
        t=float(np.clip(np.dot(state[:2]-a,d)/np.dot(d,d),0,1))
        s=path.actual_s[i]+t*(path.actual_s[i+1]-path.actual_s[i])
        if s<lo or s>hi: continue
        point=a+t*d; err=state[:2]-point; yaw=float(wrap(state[2]-path.heading[i]))
        choices.append((np.dot(err,err)+.01*yaw*yaw,s,float(np.linalg.norm(err)),yaw,i))
    if not choices: return dict(s=previous_s,cross_track=None,yaw_error=None,segment=None,reason='LOCAL_CORRESPONDENCE_MISSING')
    _,s,e,yaw,i=min(choices)
    tangent=(path.world_xy[-1]-path.world_xy[-2]); tangent/=np.linalg.norm(tangent)
    beyond=float(np.dot(state[:2]-path.world_xy[-1],tangent))
    return dict(s=float(s),cross_track=e,yaw_error=yaw,segment=i,endpoint_unclamped_overshoot_m=max(0,beyond),reason=None)
