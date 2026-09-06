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
              actual_s=s,final_spatial_cap=np.minimum(base,stop) if permission=='RUN' else np.zeros(len(s)),method='GLOBAL_CONSERVATIVE_CAP_MINIMUM_JERK_REST_TO_REST')
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


def rolling_horizon(path, cfg: dict, *, progress_s: float, current_v: float,
                    previous_a: float, delay_s: float, permission: str,
                    safety_cap_mps: float) -> dict:
    """Fresh horizon carries measured v / previous operation; no time-zero reset.

    No actuator call. A rejected stop margin requires independent sim stop/pause.
    The reference cap is a desired cap; current state can temporarily exceed it
    during a bounded stop and is never overwritten by the desired reference.
    """
    values = np.array([progress_s,current_v,previous_a,delay_s,safety_cap_mps])
    if not np.isfinite(values).all() or min(progress_s,current_v,delay_s,safety_cap_mps) < 0:
        raise ValueError('INVALID_ROLLING_STATE')
    usable = float(path.diagnostics['usable_prefix_s_m'])
    endpoint = max(0.,usable-cfg['endpoint_margin_m'])
    remaining = max(0.,endpoint-progress_s)
    needed = stopping_distance(current_v,previous_a,cfg,delay_s)
    k = np.abs(path.curvature)
    delta = np.arctan(cfg['wheelbase_m']*path.curvature)
    caps = dict(mission=cfg['nominal_speed_mps'],vehicle=cfg['maximum_speed_mps'],
                curvature=float(np.sqrt(cfg['lateral_acceleration_limit_mps2']/np.maximum(k,1e-12)).min()),
                steering_rate=float((cfg['steering_rate_limit_rad_s']/np.maximum(np.abs(np.gradient(delta,path.actual_s)),1e-12)).min()),
                permission=cfg['maximum_speed_mps'] if permission=='RUN' else 0.,safety=safety_cap_mps,
                residual_distance=float(np.sqrt(2*cfg['braking_max_mps2']*remaining)))
    final = min(caps.values()); caps['final'] = final
    reason = 'STOPPING_DISTANCE_INSUFFICIENT' if needed > remaining else None
    s=progress_s; v=current_v; a=previous_a; dt=cfg['controller_dt_s']
    rows=[]
    for _ in range(cfg['prediction_steps']):
        remain = max(0.,endpoint-s)
        target = final
        if stopping_distance(v,a,cfg,delay_s)+v*dt >= remain:
            target = 0.
        desired = np.clip((target-v)/dt,-cfg['braking_max_mps2'],cfg['acceleration_max_mps2'])
        a = float(np.clip(desired,a-cfg['jerk_limit_mps3']*dt,a+cfg['jerk_limit_mps3']*dt))
        nv = max(0., v+a*dt)
        s += (v+nv)*dt/2; v=nv
        rows.append((min(s,endpoint),v,a))
    rows=np.array(rows); xy,yaw,index=sample_path(path,rows[:,0])
    return dict(xy=xy,yaw=yaw,speed=rows[:,1],acceleration=rows[:,2],s=rows[:,0],index=index,
                caps=caps,initial_v_mps=current_v,previous_a_mps2=previous_a,delay_s=delay_s,
                usable_path_end_s=usable,stop_target_s=endpoint,remaining_m=remaining,
                stopping_distance_m=needed,reason=reason,
                usable_end_overshoot_m=max(0.,progress_s-usable),
                stop_target_overshoot_m=max(0.,progress_s-endpoint),spatial_extension_m=0.)
