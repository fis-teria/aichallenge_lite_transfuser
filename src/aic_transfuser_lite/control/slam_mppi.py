"""Reference-space MPPI over SLAM occupancy, independent of ROS.

All geometry is metres/radians. Samples are lateral reference offsets; unknown
space is forbidden. This static-occupancy planner does not classify road surface
or predict moving objects. The low-speed AWSIM profile retains a separate guard.
"""
from __future__ import annotations

import math
from typing import Any
import numpy as np

from .slam_slowdown import SlamSlowdown
from .vehicle_motion_v1 import effective_response_length
from .time_path_recovery import (RECOVERY_POLICY, RECOVERY_SPEED_MPS, RetainedPathRecovery,
                                 recoverable_geometry, validate_recovery_policy)

POLICY = 'slam_reference_mppi_v1'
FAST_POLICY = 'slam_reference_mppi_15kmh_trial_v1'
SPEED_MPS = 5. / 3.6


def validate_slam_mppi_policy(config: dict[str, Any]) -> str:
    validate_recovery_policy(config)
    policy = config.get('slam_mppi_policy', 'off')
    if policy not in ('off', POLICY, FAST_POLICY):
        raise ValueError('SLAM_MPPI_POLICY')
    fast = policy == FAST_POLICY
    if policy != 'off' and (config.get('slam_slowdown_policy', 'off') != 'off'
            or config.get('obstacle_policy') != 'steering_support_v2'
            or config.get('scan_occupancy_policy') != 'stop_v1'
            or config.get('stopping_distance_policy') != 'measured_speed_v1'
            or config.get('vehicle_model_policy') != ('awsim_understeer_20kmh_trial_v1' if fast else 'awsim_understeer_v1')
            or config.get('speed_policy') != ('curvature_time_preview_20kmh_v1' if fast else 'fixed_5kmh')):
        raise ValueError('SLAM_MPPI_REQUIRES_LOW_SPEED_AND_SCAN_STOP')
    if fast and (config.get('speed_parameters') != {'max_speed_kmh': 20., 'corner_max_speed_kmh': 15.}
                 or config.get('mppi_max_speed_kmh') != 15.):
        raise ValueError('SLAM_MPPI_20_15_15_TRIAL_CONTRACT')
    return policy


def path_speed_limit(path: np.ndarray, policy: str) -> float:
    """Target ceiling in m/s, including 1 m endpoint reserve for the 15 km/h trial.

    Braking uses .4 + .5*v + v*v/2 metres. Lateral acceleration is limited to
    2.5 m/s^2 and physical tire demand to .3 rad with the empirical response.
    """
    if policy == POLICY:
        return SPEED_MPS
    if policy != FAST_POLICY:
        raise ValueError('SLAM_MPPI_POLICY')
    path = np.asarray(path, dtype=float)
    if (path.ndim != 2 or path.shape[1:] != (2,) or not 3 <= len(path) <= 170
            or not np.isfinite(path).all()):
        raise ValueError('MPPI_PATH_SPEED_SHAPE_OR_FINITE')
    delta = np.diff(path, axis=0)
    ds = np.linalg.norm(delta, axis=1)
    yaw = np.unwrap(np.arctan2(delta[:, 1], delta[:, 0]))
    curvature = float(np.max(np.abs(np.diff(yaw)/np.maximum(.01, .5*(ds[:-1]+ds[1:])))))
    k = max(curvature, 1e-9)
    tire_cap = math.sqrt(max(0., (math.tan(.3)/k-1.087)/.045))
    horizon_cap = max(0., math.sqrt(.25+2*max(0., float(ds.sum())-1.-.4))-.5)
    return min(15./3.6, math.sqrt(2.5/k), tire_cap, horizon_cap)


def transform(points: np.ndarray, pose: np.ndarray) -> np.ndarray:
    """[N,2] body m -> world m; pose [x m,y m,yaw rad]."""
    c, s = np.cos(pose[2]), np.sin(pose[2])
    return points @ np.array([[c, s], [-s, c]])+pose[:2]


def inverse_transform(points: np.ndarray, pose: np.ndarray) -> np.ndarray:
    c, s = np.cos(pose[2]), np.sin(pose[2])
    return (points-pose[:2]) @ np.array([[c, -s], [s, c]])


class ReferenceMppi:
    """256 samples x 3 exponential-weighted updates; recheck the mean path.

    Smooth lateral offsets preserve the initial state but may end beside the
    reference. Rejoining is a cost, not a terminal constraint. No extrapolation.
    Three overlapping radius-0.95 m circles cover the 2.4 x 1.4 m body with margin.
    Additional clearance covers cell discretization and between-sample motion.
    """
    def __init__(self, seed: int = 19, *, policy: str = POLICY) -> None:
        if policy not in (POLICY, FAST_POLICY):
            raise ValueError('SLAM_MPPI_POLICY')
        self.policy = policy
        from scipy.ndimage import distance_transform_edt
        self._distance_transform = distance_transform_edt
        self.rng = np.random.default_rng(seed)
        self.side = 0
        self.diagnostics: list[dict[str, int]] = []
        self.mean = np.zeros(4)
        self.rollout_mean = np.zeros(4)
        self.rollout_active = False

    def solve(self, reference_world: np.ndarray, base_pose: np.ndarray,
              values: np.ndarray, origin_xy_m: np.ndarray, resolution_m: float) -> dict[str, Any]:
        ref, base = np.asarray(reference_world, dtype=float), np.asarray(base_pose, dtype=float)
        self.diagnostics = []
        grid, origin = np.asarray(values), np.asarray(origin_xy_m, dtype=float)
        if (ref.ndim != 2 or ref.shape[1:] != (2,) or not 2 <= len(ref) <= 200
                or not np.isfinite(ref).all() or base.shape != (3,) or not np.isfinite(base).all()
                or origin.shape != (2,) or not np.isfinite(origin).all()
                or grid.ndim != 2 or min(grid.shape) < 2 or max(grid.shape) > 250
                or not np.isin(grid, (-1, 0, 100)).all() or resolution_m != .2):
            raise ValueError('MPPI_INPUT_SHAPE_OR_UNITS')
        local = inverse_transform(ref, base)
        delta = np.diff(local, axis=0)
        squared = np.sum(delta**2, axis=1)
        fraction = np.clip(-np.sum(local[:-1]*delta, axis=1)/np.maximum(squared, 1e-12), 0., 1.)
        projected = local[:-1]+fraction[:, None]*delta
        nearest = int(np.argmin(np.linalg.norm(projected, axis=1)))
        if np.linalg.norm(projected[nearest]) > 2.5:
            raise ValueError('MPPI_REFERENCE_TOO_FAR')
        route = np.vstack((projected[nearest], local[nearest+1:]))
        route = route[np.r_[True, np.linalg.norm(np.diff(route, axis=0), axis=1) > .01]]
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))]
        length = min(16., float(arc[-1]))
        if length < 1.5:
            raise ValueError('MPPI_REFERENCE_TOO_SHORT')
        stations = np.linspace(0., length, math.ceil(length/.10)+1)
        guide = np.column_stack([np.interp(stations, arc, route[:, j]) for j in range(2)])
        tangent = np.gradient(guide, axis=0)
        headings = np.unwrap(np.arctan2(tangent[:, 1], tangent[:, 0]))
        if abs(headings[0]) > .7:
            raise ValueError('MPPI_REFERENCE_HEADING')
        normals = np.column_stack((-np.sin(headings), np.cos(headings)))
        u = stations/length
        # Hermite boundary conditions preserve the current lateral displacement
        # and heading during a manoeuvre; reconnecting immediately to the nominal
        # line would create an impossible sideways jump on the next update.
        d0 = float(-guide[0]@normals[0])
        start_correction = -guide[0]-d0*normals[0]
        if np.linalg.norm(start_correction) > .3:
            raise ValueError('MPPI_REFERENCE_START_AHEAD')
        slope0 = math.tan(-headings[0])
        boundary = d0*(1-3*u**2+2*u**3)+length*slope0*(u-2*u**2+u**3)
        # Keep the original bypass family; terminal displacement AND heading
        # are free so a short horizon can finish while still turning.
        basis = np.column_stack((np.sin(np.pi*u)**2,
                                 np.sin(np.pi*u)**2*(2*u-1), 3*u**2-2*u**3,
                                 length*(u**3-u**2)))
        field = self._distance_transform(np.pad(grid == 0, 1, constant_values=False))[1:-1, 1:-1]*resolution_m
        # Two cell-center errors, plus max half-step displacement of any circle.
        required = .95+math.sqrt(2)*resolution_m+.15

        def evaluate(coefficients: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
            offset = boundary+coefficients @ basis.T
            paths = guide[None]+offset[:, :, None]*normals
            # The prediction's first point may be centimetres ahead of ego.
            # Join from the actual pose with zero endpoint derivative; retain
            # the original reference endpoint and recheck all geometry.
            paths += (1-3*u**2+2*u**3)[None, :, None]*start_correction
            return evaluate_paths(paths)

        def evaluate_paths(paths: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
            offset = np.sum((paths-guide[None])*normals[None], axis=2)
            segments = np.diff(paths, axis=1)
            distances = np.linalg.norm(segments, axis=2)
            yaw = np.unwrap(np.arctan2(segments[:, :, 1], segments[:, :, 0]), axis=1)
            curvature = np.diff(yaw, axis=1)/np.maximum(.01, .5*(distances[:, :-1]+distances[:, 1:]))
            valid = ((np.abs(offset).max(axis=1) <= 2.5) & (np.abs(yaw[:, 0]) <= .05)
                     & (np.abs(curvature).max(axis=1) <= math.tan(.3)/1.2)
                     & (distances.min(axis=1) > .01) & (distances.max(axis=1) <= .2)
                     & (np.abs(np.diff(yaw, axis=1)).max(axis=1) < .1))
            valid &= np.all(np.cos(yaw-headings[:-1]) > 0., axis=1)
            if self.side:
                valid &= np.mean(offset, axis=1)*self.side >= -.05
            geometric_valid = valid.copy()
            occupied_valid = np.ones(len(paths), dtype=bool)
            heading = np.concatenate((yaw, yaw[:, -1:]), axis=1)+base[2]
            world = transform(paths.reshape(-1, 2), base).reshape(paths.shape)
            clearance_cost = np.zeros(len(paths))
            for shift in (-.2, .6, 1.4):
                centers = world+shift*np.stack((np.cos(heading), np.sin(heading)), axis=2)
                cells = np.floor((centers-origin)/resolution_m).astype(int)
                ix, iy = cells[:, :, 0], cells[:, :, 1]
                inside = (ix >= 0) & (ix < grid.shape[1]) & (iy >= 0) & (iy < grid.shape[0])
                clearance = field[np.clip(iy, 0, grid.shape[0]-1), np.clip(ix, 0, grid.shape[1]-1)]
                occupied_valid &= np.all(inside & (clearance > required), axis=1)
                clearance_cost += np.mean(np.maximum(0., required+.5-clearance)**2, axis=1)
            valid &= occupied_valid
            self.diagnostics.append(dict(samples=len(paths), geometry_pass=int(geometric_valid.sum()),
                occupancy_pass=int(occupied_valid.sum()), feasible=int(valid.sum())))
            cost = np.mean(offset**2, axis=1)+4*np.mean(curvature**2, axis=1)+.5*offset[:, -1]**2
            cost += 80*clearance_cost
            cost[~valid] = np.inf
            return cost, world

        mean = self.mean.copy()
        best = None
        for iteration in range(0 if self.rollout_active else 3):
            noise = self.rng.normal(size=(256, 4))*[1.8/(1+iteration*.3), 1., 1.2, .25]
            samples = mean+noise
            samples[128:, 2:] = 0.  # Preserve explicit rejoining candidates.
            samples[0] = mean
            samples[1:9] = [[-2.4, 0., 0., 0.], [2.4, 0., 0., 0.], [-1.8, 0., 0., 0.], [1.8, 0., 0., 0.],
                            [0., 0., -1.2, 0.], [0., 0., 1.2, 0.], [0., 0., -.5, 0.], [0., 0., .5, 0.]]
            for j, k in enumerate(np.linspace(-.24, .24, 25), start=9):
                # Seed continuations from the CURRENT offset/heading, rather
                # than restarting a turn from the original zero-heading pose.
                samples[j] = [0., 0., d0+slope0*length+.5*k*length**2, slope0+k*length]
            costs, paths = evaluate(samples)
            finite = np.isfinite(costs)
            if not finite.any():
                continue
            i = int(np.argmin(costs))
            if best is None or costs[i] < best[0]:
                best = (float(costs[i]), paths[i], samples[i].copy())
            weights = np.exp(-(costs[finite]-costs[finite].min())/.35)
            weights /= weights.sum()
            mean += np.sum(weights[:, None]*(samples[finite]-mean), axis=0)
        if best is None:
            # A noisy model polyline can inject curvature into every lateral
            # perturbation. Generate physical curvature rollouts from ego pose
            # instead, using the model only as a spatial tracking cost.
            mean_k = self.rollout_mean.copy()
            limit = .999*math.tan(.3)/1.2
            ds = float(stations[1]-stations[0])
            knot_s = np.linspace(0., length, 4)
            mid_s = .5*(stations[1:]+stations[:-1])
            weights_k = np.column_stack([np.interp(mid_s, knot_s, np.eye(4)[j]) for j in range(4)])

            def rollout(controls: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
                curvature = controls @ weights_k.T
                angle = np.cumsum(curvature*ds, axis=1)
                midpoint = angle-.5*curvature*ds
                steps = ds*np.stack((np.cos(midpoint), np.sin(midpoint)), axis=2)
                paths = np.concatenate((np.zeros((len(controls), 1, 2)), np.cumsum(steps, axis=1)), axis=1)
                return evaluate_paths(paths)

            fallback = None
            for iteration in range(3):
                controls = np.clip(mean_k+self.rng.normal(0., .16/(1+.3*iteration), (256, 4)), -limit, limit)
                controls[0] = mean_k
                for j, k in enumerate(np.linspace(-limit, limit, 21), start=1):
                    controls[j] = k
                # Include transitions from turning to straight / countersteer.
                for j, k in enumerate(np.linspace(-limit, limit, 9), start=22):
                    controls[j] = [k, k, 0., 0.]
                    controls[j+9] = [k, k, -k, -k]
                costs, paths = rollout(controls)
                finite = np.isfinite(costs)
                if not finite.any():
                    continue
                i = int(np.argmin(costs))
                if fallback is None or costs[i] < fallback[0]:
                    fallback = (float(costs[i]), paths[i], controls[i].copy())
                w = np.exp(-(costs[finite]-costs[finite].min())/.35); w /= w.sum()
                mean_k = np.sum(w[:, None]*controls[finite], axis=0)
            if fallback is None:
                raise ValueError('MPPI_NO_FEASIBLE_PATH')
            costs, paths = rollout(mean_k[None])
            if np.isfinite(costs[0]) and costs[0] < fallback[0]:
                fallback = (float(costs[0]), paths[0], mean_k.copy())
            self.rollout_mean = fallback[2].copy()
            self.rollout_active = True
            selected_local = inverse_transform(fallback[1], base)
            selected_offsets = np.sum((selected_local-guide)*normals, axis=1)
            largest = selected_offsets[np.argmax(np.abs(selected_offsets))]
            if abs(largest) > .3:
                self.side = 1 if largest > 0 else -1
            return dict(path_world_xy_m=fallback[1].tolist(), target_speed_mps=path_speed_limit(fallback[1], self.policy),
                cost=fallback[0], weighted_path_rechecked=True,
                samples=sum(d['samples'] for d in self.diagnostics if d['samples'] > 1),
                reference_horizon_m=length, terminal_offset_m=float(selected_offsets[-1]),
                candidate_diagnostics=self.diagnostics, trajectory_family='curvature_rollout')
        costs, paths = evaluate(mean[None])
        if np.isfinite(costs[0]) and costs[0] < best[0]:
            best = (float(costs[0]), paths[0], mean.copy())
        self.mean = best[2].copy()
        selected_local = inverse_transform(best[1], base)
        selected_offsets = np.sum((selected_local-guide)*normals, axis=1)
        largest = selected_offsets[np.argmax(np.abs(selected_offsets))]
        if abs(largest) > .3:
            self.side = 1 if largest > 0 else -1
        return dict(path_world_xy_m=best[1].tolist(), target_speed_mps=path_speed_limit(best[1], self.policy),
                    cost=best[0], weighted_path_rechecked=True, samples=768,
                    reference_horizon_m=length, terminal_offset_m=float(selected_offsets[-1]),
                    trajectory_family='reference_offset', candidate_diagnostics=self.diagnostics)


class AvoidancePlanner:
    """Hold side until fresh clearance, never reuse a path without rechecking."""
    def __init__(self, *, policy: str = POLICY, recovery_enabled: bool = False) -> None:
        self.policy = policy
        self.solver = ReferenceMppi(policy=policy)
        self.recovery = RetainedPathRecovery() if recovery_enabled else None
        self.recovery_solver = ReferenceMppi(policy=policy) if recovery_enabled else None
        self.recovery_decision: dict[str, Any] | None = None
        self.active = False
        self.clear_since_ns: int | None = None
        self.last_stamp_ns: int | None = None
        self.reference_world: np.ndarray | None = None
        self.reference_updated_ns: int | None = None
        self.last_reference_world: np.ndarray | None = None
        self.blocking_world: np.ndarray | None = None

    def prepare_reference(self, fresh: np.ndarray | None, pose: np.ndarray, stamp_ns: int,
                          speed_mps: float, *, source_stamp_ns: int | None = None) -> np.ndarray | None:
        """Select detection reference; update() still checks the current grid."""
        if self.recovery is None:
            return fresh
        self.recovery_decision = self.recovery.select(fresh, pose, stamp_ns, speed_mps,
                                                      avoidance_active=self.active, source_stamp_ns=source_stamp_ns)
        if self.recovery_decision['reason'] == 'RECOVERY_ENTER':
            self.recovery_solver = ReferenceMppi(policy=self.policy)
        return self.recovery_decision['reference_world']

    def _reference(self, fresh: np.ndarray, stamp: int, base_pose: np.ndarray | None = None) -> np.ndarray:
        """Retain observed spatial intent for at most 10 s without extension.

        Fresh source packets are still mandatory. Never extend past a model
        prediction: accept an overlapping continuation or a fresh ego-aligned
        prediction of at least 3 m, then check occupancy again in solve().
        Stored points are SLAM-world metres, not stale body-relative waypoints.
        """
        if (fresh.ndim != 2 or fresh.shape[1:] != (2,) or not 2 <= len(fresh) <= 200
                or not np.isfinite(fresh).all()):
            raise ValueError('MPPI_REFERENCE_SHAPE_OR_FINITE')
        if not self.active or self.reference_world is None:
            if np.linalg.norm(np.diff(fresh, axis=0), axis=1).sum() >= 3.:
                self.reference_world = fresh.copy()
                self.reference_updated_ns = stamp
            return fresh
        held = self.reference_world
        delta = np.diff(fresh, axis=0)
        lengths = np.linalg.norm(delta, axis=1)
        fraction = np.clip(np.sum((held[-1]-fresh[:-1])*delta, axis=1)
                           /np.maximum(lengths**2, 1e-12), 0., 1.)
        projection = fresh[:-1]+fraction[:, None]*delta
        i = int(np.argmin(np.linalg.norm(projection-held[-1], axis=1)))
        previous = held[-1]-held[-2]
        aligned = float(previous@delta[i])/(max(1e-12, np.linalg.norm(previous)*lengths[i])) > math.cos(.35)
        remaining = (1-fraction[i])*lengths[i]+lengths[i+1:].sum()
        # During a turn a fresh ego-aligned prediction need not cross the old
        # terminal point. Requiring that crossing strands the car at a finite
        # retained endpoint even though new valid predictions keep arriving.
        fresh_from_ego = False
        if base_pose is not None and lengths.sum() >= 3.:
            local = inverse_transform(fresh, base_pose)
            heading_indices = np.flatnonzero(np.linalg.norm(local-local[0], axis=1) >= .5)
            if len(heading_indices):
                start_delta = local[int(heading_indices[0])]-local[0]
                fresh_from_ego = (np.linalg.norm(local[0]) <= .5
                                  and abs(math.atan2(start_delta[1], start_delta[0])) <= .7)
        if fresh_from_ego or (np.linalg.norm(projection[i]-held[-1]) <= .5 and aligned and remaining > .15):
            # Never splice two predictions: even a centimetre lateral mismatch
            # creates a curvature spike at the join. Adopt the whole fresh
            # polyline after continuation/ego alignment checks, preserving its geometry.
            self.reference_world = fresh.copy()
            self.reference_updated_ns = stamp
        elif (np.linalg.norm(fresh[-1]-held[-1]) <= .15 and aligned
              and lengths.sum() >= 3.):
            self.reference_updated_ns = stamp
        if self.reference_updated_ns is None or stamp-self.reference_updated_ns > 10_000_000_000:
            self.reference_world = None
            raise ValueError('MPPI_RETAINED_REFERENCE_EXPIRED')
        return self.reference_world

    def update(self, observation: dict[str, Any], reference_world: np.ndarray | None,
               grid: Any) -> dict[str, Any]:
        stamp = observation['stamp_ns']
        result = dict(policy=self.policy, mode='STOP', reason='MPPI_NO_REFERENCE', stamp_ns=stamp,
                      frame='time_slam_map', target_speed_mps=0.)
        if type(stamp) is not int or (self.last_stamp_ns is not None and stamp <= self.last_stamp_ns):
            raise ValueError('MPPI_CLOCK_RESET_OR_REPLAY')
        if self.last_stamp_ns is not None and stamp-self.last_stamp_ns > 350_000_000:
            self.clear_since_ns = None
        self.last_stamp_ns = stamp
        decision = self.recovery_decision
        if self.recovery is not None:
            if decision is None or decision['stamp_ns'] != stamp:
                return dict(result, reason='RECOVERY_PREPARATION_REQUIRED')
            result['recovery'] = {k: v for k, v in decision.items() if k != 'reference_world'}
        if self.recovery is not None and decision is not None and decision['mode'] != 'NOMINAL':
            meta = {k: v for k, v in decision.items() if k != 'reference_world'}
            if decision['mode'] == 'STOP':
                return dict(result, reason=decision['reason'], recovery=meta)
            if (observation.get('input_valid') is not True or observation.get('path_valid') is not True
                    or observation.get('path_blocked') is not False):
                return dict(result, reason='RECOVERY_OBSERVATION_OR_OCCUPANCY', recovery=meta)
            try:
                plan = self.recovery_solver.solve(decision['reference_world'], np.asarray(observation['base_pose_xyyaw']),
                    grid.values, grid.origin*grid.resolution_m, grid.resolution_m)
            except ValueError as exc:
                return dict(result, reason=str(exc), recovery=meta)
            plan['target_speed_mps'] = min(RECOVERY_SPEED_MPS, plan['target_speed_mps'])
            return dict(result, **plan, mode='RECOVER', reason='RETAINED_PATH_RECOVERY', recovery=meta)
        if reference_world is None or observation.get('path_valid') is not True:
            self.clear_since_ns = None
            return result
        fresh = np.asarray(reference_world, dtype=float)
        try:
            reference_world = self._reference(fresh, stamp, np.asarray(observation['base_pose_xyyaw']))
        except ValueError as exc:
            return dict(result, reason=str(exc))
        if observation.get('path_blocked'):
            if not self.active:
                points = [p for surface in observation.get('surfaces', []) if surface.get('path_overlap')
                          for p in surface.get('points_xy_m', [])]
                self.blocking_world = np.asarray(points[:500], dtype=float).reshape(-1, 2) if points else None
            self.active = True; self.clear_since_ns = None
        elif self.active:
            if self.clear_since_ns is None:
                self.clear_since_ns = stamp
            # A shrinking fresh prediction alone cannot clear a bypass.
            local = inverse_transform(np.asarray(reference_world), np.asarray(observation['base_pose_xyyaw']))
            delta = np.diff(local, axis=0)
            fraction = np.clip(-np.sum(local[:-1]*delta, axis=1)/np.maximum(1e-12, np.sum(delta**2, axis=1)), 0., 1.)
            distance = np.linalg.norm(local[:-1]+fraction[:, None]*delta, axis=1)
            i = int(np.argmin(distance))
            aligned = distance[i] <= .25 and abs(math.atan2(delta[i, 1], delta[i, 0])) <= .15
            fresh_local = inverse_transform(fresh, np.asarray(observation['base_pose_xyyaw']))
            fresh_length = np.linalg.norm(np.diff(fresh_local, axis=0), axis=1).sum()
            fresh_delta = np.diff(fresh_local, axis=0)
            fresh_t = np.clip(-np.sum(fresh_local[:-1]*fresh_delta, axis=1)
                             /np.maximum(1e-12, np.sum(fresh_delta**2, axis=1)), 0., 1.)
            fresh_distance = np.linalg.norm(fresh_local[:-1]+fresh_t[:, None]*fresh_delta, axis=1)
            fi = int(np.argmin(fresh_distance))
            fresh_aligned = fresh_distance[fi] <= .25 and abs(math.atan2(fresh_delta[fi, 1], fresh_delta[fi, 0])) <= .15
            passed = (self.blocking_world is None or np.all(inverse_transform(self.blocking_world,
                np.asarray(observation['base_pose_xyyaw']))[:, 0] < -.8))
            if stamp-self.clear_since_ns >= 500_000_000 and aligned and fresh_aligned and fresh_length >= 3. and passed:
                self.active = False; self.solver.side = 0; self.solver.mean.fill(0.); self.solver.rollout_mean.fill(0.)
                self.solver.rollout_active = False
                self.blocking_world = None
        if not self.active:
            return dict(result, mode='NOMINAL', reason='CLEAR')
        try:
            self.last_reference_world = np.asarray(reference_world).copy()
            plan = self.solver.solve(reference_world, np.asarray(observation['base_pose_xyyaw']),
                grid.values, grid.origin*grid.resolution_m, grid.resolution_m)
        except ValueError as exc:
            return dict(result, reason=str(exc), candidate_diagnostics=self.solver.diagnostics)
        result.update(plan, mode='AVOID', reason='MPPI_FEASIBLE')
        result.update(reference_retained=not np.array_equal(reference_world, fresh),
                      reference_updated_ns=self.reference_updated_ns)
        return result


def mppi_nominal_control(plan: Any, current: Any, *, mppi_policy: str = POLICY,
                         recovery_enabled: bool = False, **kwargs: Any) -> dict[str, Any]:
    """Permit fresh validated MPPI to replace an unavailable nominal lookahead.

    This produces no authority by itself: avoidance_command must reject NOMINAL
    for this result. Sensor, identity, age, geometry and speed errors still stop.
    Only the explicit static-obstacle profiles may use this arbitration.
    """
    from .time_trial_v1 import time_trial_control
    expected = 'curvature_time_preview_20kmh_v1' if mppi_policy == FAST_POLICY else 'fixed_5kmh'
    if mppi_policy not in (POLICY, FAST_POLICY) or kwargs.get('speed_policy') != expected:
        raise ValueError('MPPI_NOMINAL_SPEED_POLICY')
    try:
        return time_trial_control(plan, current, **kwargs)
    except ValueError as exc:
        recovery_only = recovery_enabled and recoverable_geometry(plan.xy_m, str(exc))
        if not recovery_only and str(exc) not in ('STEERING_FEASIBLE_LOOKAHEAD_MISSING', 'TIME_PATH_MOTION_UNRESOLVED',
                            'NO_FORWARD_REFERENCE', 'REFERENCE_STOPPING_DISTANCE', 'REFERENCE_TIME_PREVIEW_DISTANCE'):
            raise
        return dict(nominal_tracking_unavailable=str(exc), recovery_only=recovery_only,
                    steer_rad=0., target_speed_mps=SPEED_MPS,
                    acceleration_mps2=float(np.clip(2*(SPEED_MPS-kwargs['speed_mps']), -1., .5)))


def avoidance_command(packet: dict[str, Any] | None, *, run_id: str, source_valid: bool,
                      now_sim_ns: int, now_wall_ns: int, receipt_ns: int | None,
                      speed_mps: float, target_mps: float, acceleration_mps2: float, dt_s: float,
                      scan_wheel_pose: np.ndarray | None, current_wheel_pose: np.ndarray,
                      vehicle_model_policy: str, nominal_tracking_unavailable: str | None = None,
                      mppi_policy: str = POLICY, recovery_enabled: bool = False,
                      recovery_only: bool = False) -> dict[str, Any]:
    """Admit atomic SLAM+MPPI packet; PP physical tire rad, no actuator authority.

    Poses are [x m,y m,yaw rad]. scan_wheel_pose MUST be interpolated at the
    packet scan stamp. The caller retains its current sensor/plan safety checks.
    """
    admission = SlamSlowdown().update(packet, run_id=run_id, source_valid=source_valid,
        now_sim_ns=now_sim_ns, now_wall_ns=now_wall_ns, receipt_ns=receipt_ns,
        speed_mps=speed_mps, target_mps=target_mps, acceleration_mps2=acceleration_mps2, dt_s=dt_s)
    if mppi_policy not in (POLICY, FAST_POLICY):
        raise ValueError('SLAM_MPPI_POLICY')
    result = dict(policy=mppi_policy, mode='STOP', reason=admission['reason'], steer_rad=None,
                  target_speed_mps=0., acceleration_mps2=-1.)
    if not admission['valid']:
        return result
    assert packet is not None
    try:
        plan = packet['mppi']
        if (plan['policy'] != mppi_policy or plan['stamp_ns'] != packet['stamp_ns']
                or plan['frame'] != 'time_slam_map' or plan['mode'] not in ('STOP', 'NOMINAL', 'AVOID', 'RECOVER')):
            raise ValueError('MPPI_PLAN_IDENTITY')
        if plan['mode'] == 'STOP':
            return dict(result, reason=plan['reason'])
        recovering = plan['mode'] == 'RECOVER'
        if recovery_only and not recovering:
            raise ValueError('RECOVERY_PLAN_REQUIRED')
        if recovering:
            meta = plan['recovery']
            if (not recovery_enabled or meta['policy'] != RECOVERY_POLICY
                    or type(meta['attempts']) is not int or not 1 <= meta['attempts'] <= 2
                    or type(meta['started_ns']) is not int or not 0 <= now_sim_ns-meta['started_ns'] < 6_000_000_000
                    or type(meta['reference_source_stamp_ns']) is not int
                    or not 0 <= now_sim_ns-meta['reference_source_stamp_ns'] <= 10_000_000_000
                    or not math.isfinite(meta['travelled_m']) or not 0 <= meta['travelled_m'] < 2.5
                    or packet['path_blocked']):
                raise ValueError('RECOVERY_PACKET_CONTRACT')
        if plan['mode'] == 'NOMINAL':
            if nominal_tracking_unavailable is not None:
                raise ValueError('MPPI_NOMINAL_UNAVAILABLE:'+nominal_tracking_unavailable)
            if packet['path_blocked']:
                raise ValueError('MPPI_BLOCKED_NOMINAL')
            return dict(result, mode='NOMINAL', reason='CLEAR', target_speed_mps=target_mps,
                        acceleration_mps2=acceleration_mps2)
        planned_speed = plan.get('target_speed_mps')
        ceiling = RECOVERY_SPEED_MPS if recovering else 15/3.6 if mppi_policy == FAST_POLICY else SPEED_MPS
        if (type(planned_speed) not in (int, float) or not math.isfinite(planned_speed)
                or not 0 < planned_speed <= ceiling):
            raise ValueError('MPPI_TARGET_SPEED')
        if speed_mps > planned_speed+1/3.6:
            return dict(result, reason='MPPI_DECELERATE_BEFORE_AVOIDANCE')
        if not recovering and target_mps <= .05:
            return dict(result, reason='MPPI_MODEL_STOP')
        base, scan, current = (np.asarray(p, dtype=float) for p in
                              (packet['base_pose_xyyaw'], scan_wheel_pose, current_wheel_pose))
        path = np.asarray(plan['path_world_xy_m'], dtype=float)
        if (any(p.shape != (3,) or not np.isfinite(p).all() for p in (base, scan, current))
                or path.ndim != 2 or path.shape[1:] != (2,) or not 3 <= len(path) <= 170
                or not np.isfinite(path).all() or plan['weighted_path_rechecked'] is not True
                or (not recovering and mppi_policy == POLICY and planned_speed != SPEED_MPS)
                or np.linalg.norm(path[0]-base[:2]) > .01):
            raise ValueError('MPPI_PATH_CONTRACT')
        if planned_speed > path_speed_limit(path, mppi_policy)+1e-9:
            raise ValueError('MPPI_PATH_SPEED_LIMIT')
        local = inverse_transform(transform(inverse_transform(path, base), scan), current)
        remaining = np.linalg.norm(np.diff(local, axis=0), axis=1)
        nearest = int(np.argmin(np.linalg.norm(local, axis=1)))
        available = float(remaining[nearest:].sum())
        if available < .4+.5*max(0., speed_mps)+max(0., speed_mps)**2/2:
            raise ValueError('MPPI_REFERENCE_STOPPING_DISTANCE')
        distances = np.linalg.norm(local, axis=1)
        preview = max(1., .4+1.5*max(0., speed_mps)) if mppi_policy == FAST_POLICY and not recovering else 1.
        candidates = np.flatnonzero((local[:, 0] > 0) & (distances >= preview) & (distances <= preview+1.5))
        if not len(candidates):
            raise ValueError('MPPI_TRACKING_LOOKAHEAD')
        points = local[candidates]
        tires = np.arctan(effective_response_length(max(0., speed_mps), vehicle_model_policy)
                          *2*points[:, 1]/np.sum(points**2, axis=1))
        feasible = np.flatnonzero(np.abs(tires) <= .3)
        if not len(feasible):
            raise ValueError('MPPI_TRACKING_STEERING')
        tire = float(tires[feasible[0]])
        target = planned_speed if recovering else min(target_mps, planned_speed)
        if recovering:
            return dict(result, mode='RECOVER', reason='RETAINED_PATH_TRACKING', steer_rad=tire,
                        target_speed_mps=target, acceleration_mps2=float(np.clip(2*(target-max(0., speed_mps)), -1., .5)),
                        recovery=plan['recovery'])
        return dict(result, mode='AVOID', reason='MPPI_TRACKING', steer_rad=tire,
                    target_speed_mps=target, acceleration_mps2=min(acceleration_mps2,
                        max(-1., min(.5, 2*(target-max(0., speed_mps))))))
    except (KeyError, TypeError, ValueError) as exc:
        return dict(result, reason='MPPI_REJECTED:'+str(exc))
