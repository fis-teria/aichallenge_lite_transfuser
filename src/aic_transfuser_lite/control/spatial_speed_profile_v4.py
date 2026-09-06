"""Explicit speed caps and rest-to-rest minimum-jerk time law on fixed polyline."""
import numpy as np
from .spatial_tracking_contracts_v4 import TimedReference
from .spatial_path_adapter_v4 import sample_path


def stopping_distance(v: float, a: float, cfg: dict, delay_s: float=0.0) -> float:
    """Numerical jerk-limited bounded braking, including supplied synthetic delay."""
    x=0.; h=cfg['plant_substep_s']; elapsed=0.
    for _ in range(1000):
        if v<=1e-8: break
        desired=a if elapsed<delay_s else -cfg['braking_max_mps2']
        a=float(np.clip(desired,a-cfg['jerk_limit_mps3']*h,a+cfg['jerk_limit_mps3']*h))
        nv=max(0,v+a*h); x+=(v+nv)*h/2; v=nv; elapsed+=h
    return x


def plan(path, cfg: dict, permission: str='RUN') -> TimedReference:
    endpoint=max(0,path.diagnostics['usable_prefix_s_m']-cfg['endpoint_margin_m'])
    s=path.actual_s; k=np.abs(path.curvature)
    curve=np.sqrt(cfg['lateral_acceleration_limit_mps2']/np.maximum(k,1e-12))
    delta=np.arctan(cfg['wheelbase_m']*path.curvature)
    derivative=np.abs(np.gradient(delta,s))
    rate=cfg['steering_rate_limit_rad_s']/np.maximum(derivative,1e-12)
    stop=np.sqrt(2*cfg['braking_max_mps2']*np.maximum(0,endpoint-s))
    base=np.minimum.reduce([np.full(len(s),cfg['nominal_speed_mps']),np.full(len(s),cfg['maximum_speed_mps']),curve,rate])
    caps=dict(mission=np.full(len(s),cfg['nominal_speed_mps']),vehicle=np.full(len(s),cfg['maximum_speed_mps']),
              curvature_lateral_acceleration=curve,steering_rate=rate,endpoint_constant_deceleration_estimate=stop,
              environment_permission=np.full(len(s),cfg['maximum_speed_mps'] if permission=='RUN' else 0),
              actual_s=s,final_spatial_cap=np.minimum(base,stop),method='GLOBAL_CONSERVATIVE_CAP_MINIMUM_JERK_REST_TO_REST')
    if permission!='RUN' or endpoint<=0:
        return TimedReference(np.array([0.,1.]),np.repeat(path.world_xy[:1],2,axis=0),np.repeat(path.heading[0],2),
                              np.zeros(2),np.zeros(2),np.zeros(2),np.zeros(2,dtype=int),caps,0.,0.)
    vmax=float(base.min())
    # Analytic maxima of quintic progress: v=1.875 L/T, |a|<5.774 L/T², |jerk|<=60 L/T³.
    duration=max(endpoint*1.875/vmax,np.sqrt(endpoint*5.774/min(cfg['acceleration_max_mps2'],cfg['braking_max_mps2'])),
                 np.cbrt(endpoint*60/cfg['jerk_limit_mps3']))
    time=np.r_[np.arange(0,duration,cfg['plant_substep_s']),duration]
    q=time/duration
    progress=endpoint*(10*q**3-15*q**4+6*q**5)
    speed=endpoint/duration*(30*q**2-60*q**3+30*q**4)
    acceleration=endpoint/duration**2*(60*q-180*q**2+120*q**3)
    xy,yaw,indices=sample_path(path,progress)
    return TimedReference(time,xy,np.unwrap(yaw),speed,acceleration,progress,indices,caps,endpoint,duration)


def horizon(reference: TimedReference, path, t: float, cfg: dict) -> dict:
    time=t+np.arange(1,cfg['prediction_steps']+1)*cfg['controller_dt_s']
    s=np.interp(time,reference.time,reference.s)
    xy,yaw,index=sample_path(path,s)
    return dict(time=time,xy=xy,yaw=yaw,speed=np.interp(time,reference.time,reference.speed),
                acceleration=np.interp(time,reference.time,reference.acceleration),s=s,index=index,
                endpoint_hold=time>=reference.duration,spatial_extension_m=0.)
