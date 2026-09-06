"""Frozen-world virtual control closed loop; predictor states never become plant."""
import time
import numpy as np
from aic_transfuser_lite.control.spatial_tracking_contracts_v4 import SpatialPathCandidate
from aic_transfuser_lite.control.spatial_path_adapter_v4 import prepare, sample_path, region_clearance, project_progress
from aic_transfuser_lite.control.spatial_speed_profile_v4 import plan, horizon, stopping_distance
from aic_transfuser_lite.control.spatial_mpc_v4 import SpatialMPC, bounded_braking


def plant_step(z: np.ndarray, request: np.ndarray, previous_a: float, cfg: dict) -> tuple:
    """Independent substepped RK4 kinematic bicycle; unilateral stop and saturation explicit."""
    dt=cfg['controller_dt_s']; h=cfg['plant_substep_s']
    if not np.isfinite(z).all() or not np.isfinite(request).all(): raise ValueError('NONFINITE_PLANT')
    applied=request.copy()
    applied[0]=np.clip(applied[0],max(-cfg['braking_max_mps2'],previous_a-cfg['jerk_limit_mps3']*dt),
                       min(cfg['acceleration_max_mps2'],previous_a+cfg['jerk_limit_mps3']*dt))
    applied[1]=np.clip(applied[1],-cfg['steering_rate_limit_rad_s'],cfg['steering_rate_limit_rad_s'])
    state=z.copy(); sub=[state.copy()]; stop_clips=steer_clips=speed_clips=0
    def derivative(q):
        x,y,yaw,v,delta=q; a,rate=applied
        return np.array([v*np.cos(yaw),v*np.sin(yaw),v*np.tan(delta)/cfg['wheelbase_m'],a,rate])
    steps=int(round(dt/h))
    assert abs(steps*h-dt)<1e-10
    for _ in range(steps):
        step=h
        # Integrate only until v hits zero, then hold position for rest of substep.
        if applied[0]<0 and state[3]+step*applied[0]<0:
            step=max(0,state[3]/-applied[0]); stop_clips+=1
        k1=derivative(state); k2=derivative(state+step*k1/2); k3=derivative(state+step*k2/2); k4=derivative(state+step*k3)
        state=state+step*(k1+2*k2+2*k3+k4)/6
        if step<h: state[3]=0.; state[4]+=(h-step)*applied[1]
        if abs(state[4])>cfg['steering_limit_rad']: steer_clips+=1
        if state[3]>cfg['maximum_speed_mps']: speed_clips+=1
        state[4]=np.clip(state[4],-cfg['steering_limit_rad'],cfg['steering_limit_rad'])
        state[3]=np.clip(state[3],0,cfg['maximum_speed_mps'])
        sub.append(state.copy())
    return state,dict(requested=request,applied=applied,current_delta_rad=z[4],dt_s=dt,
        acceleration_unit='m/s^2',steering_rate_unit='rad/s',unilateral_stop_substeps=stop_clips,
        steering_saturation_substeps=steer_clips,speed_saturation_substeps=speed_clips,substep_states=np.array(sub))


def synthetic_scenes() -> list[dict]:
    s=np.linspace(.1,2.5,25)
    straight=np.c_[s,s*0].astype('float32')
    arc=lambda sign: np.c_[4*np.sin(s/4),sign*4*(1-np.cos(s/4))].astype('float32')
    base=dict(saved=False,permission='RUN',region=[-2.,-3.,6.,3.],obstacles=[])
    specs=[('straight',straight,[0,0,0,0,0]),('left',arc(1),[0,0,0,0,0]),('right',arc(-1),[0,0,0,0,0]),
           ('offset_left',straight,[0,.12,0,0,0]),('offset_right',straight,[0,-.12,0,0,0]),
           ('yaw_offset',straight,[0,0,.08,0,0]),('short_switch',straight,[0,0,0,0,0]),
           ('obstacle',straight,[0,0,0,0,0])]
    rows=[]
    for name,xy,z in specs:
        scene=dict(base,id=name,initial_state=z,raw_xy=xy,nominal_s=s.copy())
        if name=='short_switch': scene.update(switch_time_s=2.5,switch_remaining_m=.08)
        if name=='obstacle': scene['obstacles']=[[2.1,-.5,2.3,.5]]
        rows.append(scene)
    return rows


def execute_scene(scene: dict, cfg: dict, consume_cycle, emit) -> dict:
    raw=scene['raw_xy']; candidate=SpatialPathCandidate(raw.copy(),scene['nominal_s'].copy(),scene['id'],t_obs=scene.get('t_obs'))
    path=prepare(candidate,cfg,scene)
    base=dict(scene_id=scene['id'],saved=scene.get('saved',False),source_raw_hash=candidate.raw_hash,
              geometry=path.diagnostics,raw_xy=raw,nominal_s=candidate.nominal_s,prepared_xy=path.world_xy,
              actual_s=path.actual_s,source_indices=path.source_indices,static_reference='STATIC_REFERENCE_EXPERIMENT',
              t_obs_original=candidate.t_obs,virtual_start_s=0.,initial_state=scene['initial_state'])
    emit('path',base)
    if path.reason:
        return dict(scene_id=scene['id'],execution='REJECTED_PATH',reason=path.reason,tracking_success=False,cycles=0,solver_calls=0)
    reference=plan(path,cfg,scene['permission']); emit('speed_profile',reference.__dict__)
    z=np.array(scene['initial_state'],dtype=float); u=np.zeros(2); mpc=SpatialMPC(cfg); progress=0.
    errors=[]; posts=[]; timings=[]; fallback=0; collision=False; maximum_violation=0.; rows=0; switched=False; insuff=False
    goal=cfg['goals']; endpoint=reference.endpoint_s; budget_stopped=False
    for cycle in range(cfg['budgets']['scene_cycles']):
        if not consume_cycle(): budget_stopped=True; break
        t=cycle*cfg['controller_dt_s']
        projection=project_progress(path,z,progress,cfg); progress=projection['s']
        if scene.get('switch_time_s') is not None and not switched and t>=scene['switch_time_s']:
            # Truncate same fixed world path, never paste raw path at current ego.
            path.diagnostics['usable_prefix_s_m']=min(path.actual_s[-1],progress+scene['switch_remaining_m']+cfg['endpoint_margin_m'])
            path.diagnostics['trim_reason']='SYNTHETIC_SHORT_PATH_SWITCH'
            reference=plan(path,cfg,scene['permission']); endpoint=reference.endpoint_s; switched=True
            emit('switched_speed_profile',reference.__dict__)
        remain=endpoint-progress
        needed=stopping_distance(z[3],u[0],cfg,scene.get('artificial_delay_s',0.))
        shortage=needed>max(0,remain)+cfg['solver']['constraint_tolerance']; insuff|=shortage
        ref=horizon(reference,path,t,cfg)
        invalid=scene['permission']!='RUN' or scene.get('stale',False) or shortage
        if invalid:
            request=bounded_braking(z,u[0],cfg); result=None
            mode='HOLD' if scene['permission']!='RUN' else 'STALE' if scene.get('stale') else 'STOP_DISTANCE_INSUFFICIENT'
        else:
            result=mpc.solve(z,ref,u,scene,fault=scene.get('solver_fault'))
            request=result.first_control; mode='MPC' if result.accepted else 'FALLBACK_BRAKING'
            timings.append(result.diagnostics['wall_seconds'])
            maximum_violation=max(maximum_violation,result.diagnostics.get('maximum_violation',0.) if result.accepted else 0.)
        if mode!='MPC': fallback+=1
        # Explicit synthetic actuator delay: hold previous input, never count as measured CPU latency.
        plant_request=u if scene.get('artificial_delay_s',0)>0 and t<scene['artificial_delay_s'] else request
        nxt,applied=plant_step(z,plant_request,u[0],cfg)
        footprint_margins=[region_clearance(q,cfg,scene) for q in applied['substep_states']]
        collision|=min(footprint_margins)<=0
        pnext=project_progress(path,nxt,progress,cfg); progress=pnext['s']
        error=pnext['cross_track']
        if error is not None: errors.append(error)
        if t>=goal['convergence_after_s'] and error is not None: posts.append(error)
        target,yaw,_=sample_path(path,np.array(endpoint))
        tangent=np.array([np.cos(yaw),np.sin(yaw)])
        overshoot=max(0,float(np.dot(nxt[:2]-target,tangent)))
        plant_values=dict(speed_violation=max(0,float(nxt[3]-cfg['maximum_speed_mps'])),
            steering_violation=max(0,float(abs(nxt[4])-cfg['steering_limit_rad'])),
            lateral_acceleration_violation=max(0,float(nxt[3]**2*abs(np.tan(nxt[4]))/cfg['wheelbase_m']-cfg['lateral_acceleration_limit_mps2'])))
        maximum_violation=max(maximum_violation,*plant_values.values())
        emit('cycle',dict(cycle=cycle,scene_id=scene['id'],source_raw_hash=candidate.raw_hash,virtual_time_s=t,
            state=z,next_state=nxt,reference=ref,raw_length_m=path.actual_s[-1],used_prefix_s_m=endpoint,
            remaining_s_m=remain,trim_reason=path.diagnostics['trim_reason'],requested_control=request,plant=applied,
            solver=None if result is None else result.diagnostics,mpc_states=None if result is None else result.states,
            mpc_controls=None if result is None else result.controls,mode=mode,required_stopping_distance_m=needed,
            stop_distance_insufficient=shortage,projection=pnext,endpoint_distance_m=float(np.linalg.norm(nxt[:2]-target)),
            endpoint_overshoot_m=overshoot,plant_footprint_clearance=footprint_margins,plant_constraint_violations=plant_values,
            speed_error_mps=float(nxt[3]-ref['speed'][0]),artificial_delay_s=scene.get('artificial_delay_s',0.)))
        z=nxt; u=applied['applied']; rows+=1
        if t>reference.duration+.5 and z[3]<.01: break
    target,_,_=sample_path(path,np.array(endpoint))
    rms=lambda x: float(np.sqrt(np.mean(np.square(x)))) if x else None
    endpoint_error=float(np.linalg.norm(z[:2]-target))
    progress_ok=progress>=goal['progress_fraction']*endpoint and (endpoint<=0 or progress>.1)
    success=bool(rows and progress_ok and posts and rms(posts)<=goal['cross_track_rms_m'] and max(posts)<=goal['cross_track_max_m']
        and endpoint_error<=goal['endpoint_error_m'] and z[3]<=goal['final_speed_mps'] and not collision and not insuff
        and maximum_violation<=cfg['solver']['constraint_tolerance'] and fallback==0)
    return dict(scene_id=scene['id'],execution=('PARTIAL_BUDGET' if rows else 'NOT_EXECUTED_BUDGET') if budget_stopped else 'COMPLETE',
        tracking_success=success,cycles=rows,solver_calls=mpc.calls,fallback_cycles=fallback,mpc_accepted_cycles=rows-fallback,
        raw_length_m=float(path.actual_s[-1]),endpoint_s_m=endpoint,progress_s_m=progress,progress_ok=progress_ok,
        cte_rms_all_m=rms(errors),cte_max_all_m=max(errors) if errors else None,cte_rms_post_m=rms(posts),
        cte_max_post_m=max(posts) if posts else None,endpoint_error_m=endpoint_error,final_speed_mps=float(z[3]),
        stop_distance_insufficient_seen=insuff,collision=collision,maximum_applied_violation=maximum_violation,
        solver_wall_max_s=max(timings) if timings else None,solver_wall_p95_s=float(np.percentile(timings,95)) if timings else None,
        ten_hz_wall_cycles=sum(v<=cfg['controller_dt_s'] for v in timings),final_state=z)
