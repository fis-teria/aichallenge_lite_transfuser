"""Frozen reference / synthetic plant only. No ROS, inference or MPC solve."""
from __future__ import annotations
import numpy as np
from aic_transfuser_lite.control.waypoint_controller import ControllerConfig, control_from_waypoints
from aic_transfuser_lite.control.spatial_path_adapter_v4 import project_progress, sample_path
from aic_transfuser_lite.control.spatial_speed_profile_v4 import stopping_distance
from aic_transfuser_lite.control.spatial_tracking_contracts_v4 import PreparedPath
from aic_transfuser_lite.evaluation.spatial_tracking_sim_v4 import plant_step


def evaluate(path: PreparedPath, initial: np.ndarray, cfg: dict, *, force_stop_s: float | None = None) -> dict:
    """State float[5] = rear x/y metres, yaw rad, v m/s, steer rad.

    Fixed 0.5 m arc lookahead, existing geometric PP, P speed control kp=1/s.
    Endpoint braking uses existing jerk-limited stopping distance plus one
    control period and 0.03 m reserve. No reference extension or refitting.
    """
    z=np.asarray(initial,dtype=float).copy()
    if (path.reason or z.shape!=(5,) or not np.isfinite(z).all() or
            len(path.world_xy)<2 or not np.isfinite(path.world_xy).all() or
            np.any(np.diff(path.actual_s)<=0)):
        raise ValueError('INVALID_FROZEN_REFERENCE_OR_STATE')
    if force_stop_s is not None and (not np.isfinite(force_stop_s) or force_stop_s<0):
        raise ValueError('INVALID_STOP_TIME')
    dt=cfg['controller_dt_s']; previous_a=0.; progress=0.; stopped_s=0.; moving=False
    endpoint=path.actual_s[-1]-cfg['endpoint_margin_m']
    if endpoint<=0: raise ValueError('NO_STOPPING_SUPPORT')
    goal=sample_path(path,np.array(endpoint))[0]
    pp=ControllerConfig(wheelbase_m=cfg['wheelbase_m'],min_lookahead_m=.5,
        max_steer_rad=cfg['steering_limit_rad'],min_accel_mps2=-cfg['braking_max_mps2'],
        max_accel_mps2=cfg['acceleration_max_mps2'],speed_kp=1.)
    rows=[]; stop_reason=None; violations=set(); max_error=0.; max_speed=0.; negative_brakes=0
    max_steering=max_rate=max_accel=max_decel=max_jerk=max_lateral=0.
    for index in range(int(60/dt)):
        t=index*dt; projection=project_progress(path,z,progress,cfg)
        if projection['reason']: stop_reason=stop_reason or projection['reason']
        else:
            progress=projection['s']; max_error=max(max_error,projection['cross_track'])
        remaining=endpoint-progress
        if force_stop_s is not None and t>=force_stop_s: stop_reason=stop_reason or 'EXPLICIT_OFFLINE_STOP'
        if remaining<=stopping_distance(z[3],previous_a,cfg,dt)+z[3]*dt+.03:
            stop_reason=stop_reason or 'ENDPOINT_BRAKE'
        if t>=55: stop_reason=stop_reason or 'FINITE_TIME_BRAKE'
        target=sample_path(path,np.array(min(endpoint,progress+.5)))[0]
        delta_xy=target-z[:2]; c,s=np.cos(z[2]),np.sin(z[2])
        local=np.array([[c*delta_xy[0]+s*delta_xy[1],-s*delta_xy[0]+c*delta_xy[1]]])
        if local[0,0]<=0: stop_reason=stop_reason or 'TARGET_NOT_FORWARD'
        command=control_from_waypoints(local,0. if stop_reason else min(.2,cfg['maximum_speed_mps']),z[3],pp)
        desired_a=(-cfg['braking_max_mps2'] if z[3]>.001 else 0.) if stop_reason else command.acceleration_mps2
        a=float(np.clip(desired_a,max(-cfg['braking_max_mps2'],previous_a-cfg['jerk_limit_mps3']*dt),
                        min(cfg['acceleration_max_mps2'],previous_a+cfg['jerk_limit_mps3']*dt)))
        desired_delta=z[4] if stop_reason else command.steering_rad
        rate=float(np.clip((desired_delta-z[4])/dt,-cfg['steering_rate_limit_rad_s'],cfg['steering_rate_limit_rad_s']))
        before=z.copy(); z,act=plant_step(z,np.array([a,rate]),previous_a,cfg)
        sub=act['substep_states']; jerk=abs(a-previous_a)/dt
        lateral=float(np.max(abs(sub[:,3]**2*np.tan(sub[:,4])/cfg['wheelbase_m'])))
        max_speed=max(max_speed,float(sub[:,3].max())); max_steering=max(max_steering,float(abs(sub[:,4]).max()))
        max_rate=max(max_rate,abs(rate)); max_accel=max(max_accel,a); max_decel=max(max_decel,-a)
        max_jerk=max(max_jerk,jerk); max_lateral=max(max_lateral,lateral)
        if act['speed_saturation_substeps']: violations.add('PLANT_SPEED_SATURATION')
        if act['steering_saturation_substeps']: violations.add('PLANT_STEERING_SATURATION')
        if lateral>cfg['lateral_acceleration_limit_mps2']+1e-9: violations.add('LATERAL_ACCELERATION')
        if jerk>cfg['jerk_limit_mps3']+1e-9: violations.add('JERK')
        for state in sub:
            p=project_progress(path,state,progress,cfg)
            if p['reason']: violations.add(p['reason'])
            else: max_error=max(max_error,p['cross_track'])
        moving |= z[3]>.1
        negative_brakes+=int(stop_reason is not None and a<0)
        stopped_s=stopped_s+dt if stop_reason and z[3]<=.03 else 0.
        rows.append(dict(t_s=t,state_before=before.tolist(),state_after=z.tolist(),progress_s_m=progress,
            lookahead_xy_m=target.tolist(),request_acceleration_mps2=a,request_steering_rate_rad_s=rate,
            applied=act['applied'].tolist(),stop_reason=stop_reason,unilateral_stop_substeps=act['unilateral_stop_substeps']))
        previous_a=a
        if stopped_s>=1.: break
    final=project_progress(path,z,progress,cfg)
    goal_error=float(np.linalg.norm(z[:2]-goal))
    return dict(schema='V4_PP_FROZEN_VIRTUAL_TRIAL_V1',lookahead_m=.5,maximum_virtual_time_s=60.,
        virtual_seconds=len(rows)*dt,initial_state=np.asarray(initial).tolist(),final_state=z.tolist(),
        max_cross_track_m=max_error,goal_distance_m=goal_error,stop_target_s_m=float(endpoint),
        final_progress_s_m=final['s'],max_speed_mps=max_speed,max_steering_rad=max_steering,
        max_steering_rate_rad_s=max_rate,max_acceleration_mps2=max_accel,max_deceleration_mps2=max_decel,
        max_jerk_mps3=max_jerk,max_lateral_acceleration_mps2=max_lateral,
        moved=moving,stopped_after_motion=moving and stopped_s>=1.,negative_brake_cycles=negative_brakes,
        stop_reason=stop_reason,violations=sorted(violations),
        tracking_stop_pass=moving and stopped_s>=1. and goal_error<=.1 and max_error<=.1 and not violations,
        live_safety_verified=False,collision_free='UNKNOWN',new_model_observations=0,rows=rows)
