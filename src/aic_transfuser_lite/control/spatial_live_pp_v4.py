"""Sim-only PP proposal on a freshly transformed V4 reference; no publisher."""
from __future__ import annotations
import numpy as np
from .waypoint_controller import ControllerConfig, control_from_waypoints
from .spatial_path_adapter_v4 import project_progress, sample_path
from .spatial_speed_profile_v4 import stopping_distance
from .spatial_tracking_contracts_v4 import PreparedPath
from aic_transfuser_lite.evaluation.spatial_tracking_sim_v4 import plant_step


def propose(path: PreparedPath, state: np.ndarray, previous_a: float, cfg: dict,
            *, permission: str) -> dict:
    """World rear-axle state [x m,y m,yaw rad,v m/s,steer rad]; u=[a,steer rate].

    Local progress is recomputed for each NEW path, never inherited between
    predictions. The unmodified 20-point output is not this fitted reference.
    Constant-control synthetic rollout is diagnostic/clearance input only.
    """
    z = np.asarray(state, dtype=float)
    if z.shape != (5,) or not np.isfinite(np.r_[z, previous_a]).all() or path.reason:
        raise ValueError('PP_INVALID_STATE_OR_PATH')
    if permission not in ('RUN', 'HOLD'):
        raise ValueError('PP_INVALID_PERMISSION')
    projection = project_progress(path, z, 0., cfg)
    if projection['reason']:
        raise ValueError('PP_' + projection['reason'])
    endpoint = float(path.actual_s[-1])-cfg['endpoint_margin_m']
    progress = projection['s']
    dt = cfg.get('command_schedule_s', cfg['controller_dt_s'])
    stop = permission != 'RUN' or endpoint-progress <= (
        stopping_distance(max(0., z[3]), previous_a, cfg, dt)+max(0., z[3])*dt+.03)
    target = sample_path(path, np.array(min(endpoint, progress+.5)))[0]
    d = target-z[:2]; c, s = np.cos(z[2]), np.sin(z[2])
    local = np.array([[c*d[0]+s*d[1], -s*d[0]+c*d[1]]])
    stop = stop or local[0, 0] <= 0
    target_speed = 0. if stop else min(.2, cfg['maximum_speed_mps'])
    controller = ControllerConfig(wheelbase_m=cfg['wheelbase_m'], min_lookahead_m=.5,
        max_steer_rad=cfg['steering_limit_rad'], min_accel_mps2=-cfg['braking_max_mps2'],
        max_accel_mps2=cfg['acceleration_max_mps2'], speed_kp=1.)
    cmd = control_from_waypoints(local, target_speed, z[3], controller)
    wanted = (-cfg['braking_max_mps2'] if z[3]>.001 else 0.) if stop else cmd.acceleration_mps2
    a = float(np.clip(wanted, max(-cfg['braking_max_mps2'], previous_a-cfg['jerk_limit_mps3']*dt),
                      min(cfg['acceleration_max_mps2'], previous_a+cfg['jerk_limit_mps3']*dt)))
    rate = 0. if stop else float(np.clip((cmd.steering_rad-z[4])/dt,
        -cfg['steering_rate_limit_rad_s'], cfg['steering_rate_limit_rad_s']))
    u = np.array([a, rate]); states = [z.copy()]; prev = previous_a
    for _ in range(cfg['prediction_steps']):
        next_z, details = plant_step(states[-1], u, prev, cfg)
        states.append(next_z); prev = a
    return dict(first_control=u, target_speed_reference_mps=target_speed, states=np.array(states),
                stop=bool(stop), progress_s_m=float(progress), cross_track_m=projection['cross_track'],
                lookahead_world_xy_m=target, controller='PURE_PURSUIT', mpc_used=False)
