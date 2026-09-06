"""Finite-horizon SLSQP direct shooting. No live/controller imports."""
import time
import numpy as np
from scipy.optimize import minimize
from .spatial_tracking_contracts_v4 import MpcResult
from .spatial_path_adapter_v4 import wrap, region_clearance


def prediction(z: np.ndarray, u: np.ndarray, cfg: dict) -> np.ndarray:
    """Euler predictor: state [x,y,yaw,v,delta], u [a,delta_rate], SI."""
    states=[np.asarray(z,dtype=float).copy()]; dt=cfg['controller_dt_s']; wheelbase=cfg['wheelbase_m']
    for a,rate in u:
        x,y,yaw,v,delta=states[-1]
        states.append(np.array([x+dt*v*np.cos(yaw),y+dt*v*np.sin(yaw),
                               yaw+dt*v*np.tan(delta)/wheelbase,v+dt*a,delta+dt*rate]))
    return np.array(states)


def constraint_margins(states: np.ndarray, u: np.ndarray, previous_a: float, cfg: dict, scene: dict) -> dict:
    v=states[1:,3]; delta=states[1:,4]; a=u[:,0]; rate=u[:,1]
    jerk=np.diff(np.r_[previous_a,a])/cfg['controller_dt_s']
    margins=dict(speed_lower=v,speed_upper=cfg['maximum_speed_mps']-v,
        steering=cfg['steering_limit_rad']-np.abs(delta),steering_rate=cfg['steering_rate_limit_rad_s']-np.abs(rate),
        acceleration=cfg['acceleration_max_mps2']-a,braking=cfg['braking_max_mps2']+a,
        jerk=cfg['jerk_limit_mps3']-np.abs(jerk),
        lateral_acceleration=cfg['lateral_acceleration_limit_mps2']-np.abs(v*v*np.tan(delta)/cfg['wheelbase_m']))
    if scene.get('region') is not None or scene.get('obstacles'):
        margins['footprint']=np.array([region_clearance(z,cfg,scene) for z in states])
    return margins


def bounded_braking(z: np.ndarray, previous_a: float, cfg: dict) -> np.ndarray:
    dt=cfg['controller_dt_s']; step=cfg['jerk_limit_mps3']*dt
    target=-cfg['braking_max_mps2'] if z[3]>.001 else 0.
    a=float(np.clip(target,previous_a-step,previous_a+step))
    # No guarantee of an immediate stop; plant's unilateral v>=0 is explicit.
    return np.array([np.clip(a,-cfg['braking_max_mps2'],cfg['acceleration_max_mps2']),0.])


class SpatialMPC:
    def __init__(self, cfg: dict):
        self.cfg=cfg; self.warm=None; self.calls=0

    def solve(self, z: np.ndarray, reference: dict, previous_u: np.ndarray, scene: dict,
              *, fault: str | None=None) -> MpcResult:
        cfg=self.cfg; n=cfg['prediction_steps']; w=cfg['solver']['weights']; start=time.monotonic()
        guess=np.zeros((n,2)) if self.warm is None else np.vstack([self.warm[1:],self.warm[-1]])
        cache_x=None; cache_states=None
        def states(flat):
            nonlocal cache_x,cache_states
            if cache_x is None or not np.array_equal(cache_x,flat):
                cache_x=flat.copy(); cache_states=prediction(z,flat.reshape(n,2),cfg)
            return cache_states
        def objective(flat):
            p=states(flat)[1:]; u=flat.reshape(n,2); change=np.diff(np.vstack([previous_u,u]),axis=0)
            e=p[:,:2]-reference['xy']
            return float(w['position']*np.sum(e*e)+w['yaw']*np.sum(wrap(p[:,2]-reference['yaw'])**2)
                +w['speed']*np.sum((p[:,3]-reference['speed'])**2)
                +w['acceleration']*np.sum(u[:,0]**2)+w['steering_rate']*np.sum(u[:,1]**2)
                +w['acceleration_change']*np.sum(change[:,0]**2)+w['steering_rate_change']*np.sum(change[:,1]**2)
                +w['terminal_position']*np.sum(e[-1]**2))
        def constraints(flat):
            return np.concatenate(list(constraint_margins(states(flat),flat.reshape(n,2),previous_u[0],cfg,scene).values()))
        self.calls+=1
        try:
            result=minimize(objective,guess.ravel(),method='SLSQP',
                bounds=[(-cfg['braking_max_mps2'],cfg['acceleration_max_mps2']),
                        (-cfg['steering_rate_limit_rad_s'],cfg['steering_rate_limit_rad_s'])]*n,
                constraints=[dict(type='ineq',fun=constraints)],
                options=dict(maxiter=cfg['solver']['maxiter'],ftol=cfg['solver']['ftol'],disp=False))
            controls=result.x.reshape(n,2); predicted=prediction(z,controls,cfg)
            if fault=='nonfinite': controls[0,0]=np.nan
            elapsed=time.monotonic()-start
            margins=constraint_margins(predicted,controls,previous_u[0],cfg,scene)
            residual={k:float(max(0,-np.min(v))) for k,v in margins.items()}
            finite=bool(np.isfinite(controls).all() and np.isfinite(predicted).all() and np.isfinite(result.fun))
            violation=max(residual.values())
            reason=('NONFINITE' if not finite else 'INJECTED_SOLVER_FAILURE' if fault=='failure' else
                'DEADLINE_EXCEEDED' if elapsed>cfg['solver']['wall_time_limit_s'] or fault=='timeout' else
                'CONSTRAINT_VIOLATION' if violation>cfg['solver']['constraint_tolerance'] else
                'SOLVER_UNSUCCESSFUL' if not result.success else None)
            diagnostics=dict(success=bool(result.success),status=int(result.status),message=str(result.message),iterations=int(result.nit),
                cost=float(result.fun),wall_seconds=elapsed,deadline_s=cfg['solver']['wall_time_limit_s'],
                numerical_10hz_deadline_met=elapsed<=cfg['controller_dt_s'],residual_by_constraint=residual,
                maximum_violation=violation,finite=finite,rejection_reason=reason,fault_injection=fault,actual_solver_called=True)
        except Exception as exc:
            controls=np.empty((0,2)); predicted=np.empty((0,5)); reason='SOLVER_EXCEPTION'
            diagnostics=dict(success=False,message=repr(exc),wall_seconds=time.monotonic()-start,rejection_reason=reason,actual_solver_called=True)
        accepted=reason is None
        self.warm=controls.copy() if accepted else None
        first=controls[0].copy() if accepted else bounded_braking(z,previous_u[0],cfg)
        return MpcResult(predicted,controls,first,accepted,diagnostics)
