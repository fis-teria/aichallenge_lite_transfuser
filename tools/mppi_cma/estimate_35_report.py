"""Create an offline 35 km/h estimate beside, never over, the selected references."""
from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import hashlib
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import numpy as np
from PIL import Image
from scipy.interpolate import CubicSpline, splprep, splev
from scipy.optimize import minimize
from scipy.signal import find_peaks
import yaml

if __package__:
    from .speed_envelope import KartLimits, estimate_closed_profile
else:
    from speed_envelope import KartLimits, estimate_closed_profile


def polygon_separation(xy: np.ndarray, yaw: np.ndarray, polygon: np.ndarray) -> np.ndarray:
    """Signed separating-axis gap [N], metres, for the frozen conservative hull."""
    hull = np.array([[-.379, -.769], [1.616, -.769], [1.616, .769], [-.379, .769]])
    direction = np.column_stack((np.cos(yaw), np.sin(yaw)))
    lateral = np.column_stack((-np.sin(yaw), np.cos(yaw)))
    body = xy[:, None, :] + hull[None, :, :1] * direction[:, None, :] + hull[None, :, 1:] * lateral[:, None, :]
    edges = np.roll(polygon, -1, axis=0) - polygon
    axes = np.column_stack((-edges[:, 1], edges[:, 0]))
    axes /= np.linalg.norm(axes, axis=1)[:, None]
    axes = np.concatenate((direction[:, None, :], lateral[:, None, :], np.broadcast_to(axes, (len(xy), *axes.shape))), axis=1)
    body_proj = np.einsum('nvi,nai->nav', body, axes)
    polygon_proj = np.einsum('vi,nai->nav', polygon, axes)
    return np.max(np.maximum(body_proj.min(axis=2) - polygon_proj.max(axis=2),
                             polygon_proj.min(axis=2) - body_proj.max(axis=2)), axis=1)


def approximate_line(xy_m: np.ndarray, ot_polygon_m: np.ndarray, *, points: int = 180,
                     spacing_m: float = .25) -> dict:
    """Fit a nearby steering-feasible approximation, not a minimum-lap-time line.

    The 0.1 m RMS periodic fit regularizes CSV vertex/seam artifacts. Bounded
    normal offsets minimize further displacement, with a conservative coarse
    curvature bound and OT gap. Dense steering/OT checks must pass afterwards.
    Source coordinates are not changed. XYZ/CSV heading conventions are not reused.
    """
    xy = np.asarray(xy_m, dtype=float).copy()
    polygon = np.asarray(ot_polygon_m, dtype=float)
    if xy.ndim != 2 or xy.shape[1] != 2 or len(xy) < 8 or not np.isfinite(xy).all():
        raise ValueError('xy_m must be finite [N>=8,2] in metres')
    if polygon.ndim != 2 or polygon.shape[1] != 2 or len(polygon) < 3 or not np.isfinite(polygon).all():
        raise ValueError('OT polygon must be finite [M>=3,2]')
    if points < 16 or not math.isfinite(spacing_m) or spacing_m <= 0:
        raise ValueError('At least 16 fit points and a positive spacing_m are required')
    closing_gap = float(np.linalg.norm(xy[-1] - xy[0]))
    removed_closing_point = closing_gap < .1
    if removed_closing_point:
        xy = xy[:-1]
    origin = xy.mean(axis=0)
    local = xy - origin
    closed = np.vstack((local, local[0]))
    distances = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    if distances.min() < 1e-6:
        raise ValueError('Duplicate consecutive coordinates')
    station = np.r_[0., np.cumsum(distances)]
    parameter = station / station[-1]
    fitted, _ = splprep(closed.T, u=parameter, s=len(local) * .1 ** 2, per=True)
    grid = np.linspace(0., 1., points, endpoint=False)
    base = np.asarray(splev(grid, fitted)).T
    tangent = np.asarray(splev(grid, fitted, der=1)).T
    tangent /= np.linalg.norm(tangent, axis=1)[:, None]
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))

    def displaced(offset: np.ndarray) -> np.ndarray:
        return base + normal * offset[:, None]

    def curvature(p: np.ndarray) -> np.ndarray:
        previous = p - np.roll(p, 1, axis=0)
        following = np.roll(p, -1, axis=0) - p
        return 2 * (previous[:, 0] * following[:, 1] - previous[:, 1] * following[:, 0]) / (
            np.linalg.norm(previous, axis=1) * np.linalg.norm(following, axis=1)
            * np.linalg.norm(previous + following, axis=1))

    def objective(offset: np.ndarray) -> float:
        return float(np.dot(offset, offset) + 2 * np.sum((np.roll(offset, -1) - offset) ** 2))

    def constraints(offset: np.ndarray) -> np.ndarray:
        p = displaced(offset)
        tangent = np.roll(p, -1, axis=0) - np.roll(p, 1, axis=0)
        yaw = np.arctan2(tangent[:, 1], tangent[:, 0])
        return np.r_[.25 ** 2 - curvature(p) ** 2,
                     polygon_separation(p, yaw, polygon - origin) - .16]

    result = minimize(objective, np.zeros(points), method='SLSQP', bounds=[(-.8, .8)] * points,
                      constraints=[{'type': 'ineq', 'fun': constraints}],
                      options={'maxiter': 250, 'ftol': 1e-6})
    if not result.success or constraints(result.x).min() < -1e-6:
        raise RuntimeError(f'Nearby-line fit did not converge: {result.message}')
    p = displaced(result.x)
    coarse_s = np.r_[0., np.cumsum(np.linalg.norm(np.roll(p, -1, axis=0) - p, axis=1))]
    spline = CubicSpline(coarse_s, np.vstack((p, p[0])), bc_type='periodic')
    dense_s = np.linspace(0., coarse_s[-1], int(math.ceil(coarse_s[-1] / spacing_m)), endpoint=False)
    first = spline(dense_s, 1)
    second = spline(dense_s, 2)
    kappa = (first[:, 0] * second[:, 1] - first[:, 1] * second[:, 0]) / np.linalg.norm(first, axis=1) ** 3
    dense_xy = spline(dense_s) + origin
    yaw = np.arctan2(first[:, 1], first[:, 0])
    gap = polygon_separation(dense_xy - origin, yaw, polygon - origin)
    if np.any(gap < .1) or np.any(np.abs(np.arctan(1.087 * kappa)) > math.pi / 10):
        raise RuntimeError('Dense steering/OT check rejected the fitted line')
    ds = np.linalg.norm(np.roll(dense_xy, -1, axis=0) - dense_xy, axis=1)
    # Distance to the original segments, rather than waypoint-to-waypoint error.
    starts = local
    vectors = np.roll(local, -1, axis=0) - local
    length2 = np.sum(vectors ** 2, axis=1)
    residual = dense_xy[:, None, :] - origin - starts[None, :, :]
    fraction = np.clip(np.sum(residual * vectors[None, :, :], axis=2) / length2[None, :], 0., 1.)
    distance = np.linalg.norm(residual - fraction[:, :, None] * vectors[None, :, :], axis=2).min(axis=1)
    return {'xy_m': dense_xy, 'yaw_rad': yaw, 'curvature_1pm': kappa, 'segment_lengths_m': ds,
            'station_m': np.r_[0., np.cumsum(ds[:-1])],
            'audit': {'kind': 'nearby steering-feasible approximation; not a minimum-time line optimization',
                      'source_closing_gap_m': closing_gap, 'removed_near_duplicate_closing_point': removed_closing_point,
                      'spline_rms_budget_m': .1, 'fit_points': points, 'dense_spacing_target_m': spacing_m,
                      'optimizer_success': bool(result.success), 'optimizer_iterations': int(result.nit),
                      'max_normal_offset_from_initial_fit_m': float(np.max(np.abs(result.x))),
                      'max_distance_to_source_polyline_m': float(distance.max()),
                      'rms_distance_to_source_polyline_m': float(np.sqrt(np.mean(distance ** 2))),
                      'source_length_m': float(station[-1]), 'approximate_length_m': float(ds.sum()),
                      'maximum_required_tire_angle_deg': float(np.degrees(np.max(np.abs(np.arctan(1.087 * kappa))))),
                      'minimum_radius_m': float(1 / np.max(np.abs(kappa))),
                      'minimum_ot_sat_gap_m': float(gap.min()), 'ot_overlap_samples_margin_0p1m': int(np.sum(gap < .1))}}


def check_walls(line: dict, image_path: Path, metadata_path: Path) -> dict:
    """Sample the enlarged full footprint on the frozen occupancy raster."""
    occupancy = np.asarray(Image.open(image_path))
    meta = yaml.safe_load(metadata_path.read_text())
    if meta['origin'][2] != 0 or meta.get('negate', 0):
        raise ValueError('This audit requires an unrotated, non-negated wall raster')
    x, y = np.meshgrid(np.linspace(-.479, 1.716, 45), np.linspace(-.869, .869, 36))
    local = np.column_stack((x.ravel(), y.ravel()))
    failures = 0
    for point, yaw in zip(line['xy_m'], line['yaw_rad']):
        c, s = math.cos(yaw), math.sin(yaw)
        body = local @ np.array([[c, s], [-s, c]]) + point
        cell = np.floor((body - np.array(meta['origin'][:2])) / meta['resolution']).astype(int)
        x = cell[:, 0]
        y = occupancy.shape[0] - 1 - cell[:, 1]
        valid = (x >= 0) & (x < occupancy.shape[1]) & (y >= 0) & (y < occupancy.shape[0])
        failures += int(not valid.all() or np.any(occupancy[y[valid], x[valid]] < 250))
    return {'footprint_extra_margin_m': .1, 'footprint_grid_max_spacing_m': .05,
            'path_samples': len(line['xy_m']), 'nonfree_footprint_samples': failures,
            'passed': failures == 0, 'limit': 'static raster check, not AWSIM collision validation'}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--selected-dir', type=Path, required=True)
    parser.add_argument('--inputs-dir', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    output = args.output_dir.resolve()
    if output.exists():
        raise ValueError('Use a new output directory to preserve previous estimates')
    source = args.selected_dir.resolve()
    inputs = args.inputs_dir.resolve()
    physics = yaml.safe_load((inputs / 'vehicle.yaml').read_text())['physics']
    if any(abs(physics[key] - value) > 1e-7 for key, value in {
        'maxSteerAngle': 30., 'gripSteerFactor': .6, 'steerTimeConstant': .02,
        'maxSteerRate': 60., 'maxAcceleration': 1.37, 'rollingResistance': .37,
        'drag': .03, 'maxSpeed': 36., 'driveFadeExponent': 40.,
    }.items()):
        raise ValueError('Frozen vehicle tuning differs from this estimator profile')
    output.mkdir(parents=True)
    limits = KartLimits()
    polygon = np.asarray(json.loads((source / 'calibration.json').read_text())['ot_lane_polygon_map_m'])
    observed = json.loads((inputs / 'telemetry_provenance.json').read_text())
    report = {'scope': 'Offline 35 km/h estimates. Both line geometries use normal non-handicap vehicle physics.',
              'speed_cap_kmh': 35., 'vehicle_limits': asdict(limits), 'conditions': {},
              'observed_telemetry_context': observed,
              'limitations': ['Lateral limits 6/10/14/18/22 m/s^2 are sensitivity assumptions, not measured tire limits.',
                              'The 0.1 m RMS fit plus constrained offsets changes geometry; original CSVs are preserved.',
                              'Known-curve steering anticipation is assumed; yaw dynamics, slip, tire load transfer, and intermittent acceleration hold are not simulated.',
                              'No AWSIM evaluation or global minimum-lap-time optimization was performed.'],
              'method_sources': ['https://www.mathworks.com/help/robotics/ug/mobile-robot-kinematics-equations.html',
                                 'https://github.com/TUMFTM/trajectory_planning_helpers/blob/master/trajectory_planning_helpers/calc_vel_profile.py'],
              'input_sha256': {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for folder in (source, inputs)
                               for p in folder.iterdir() if p.is_file()}}
    fig, axes = plt.subplots(2, 2, figsize=(14, 11), constrained_layout=True)
    corner_rows = []
    for column, condition in enumerate(('normal', 'leader')):
        raw = np.genfromtxt(source / f'{condition}_reference.csv', delimiter=',', names=True)
        original_xy = np.column_stack((raw['x_m'], raw['y_m']))
        line = approximate_line(original_xy, polygon)
        wall_check = check_walls(line, inputs / 'wall.pgm', inputs / 'wall.yaml')
        if not wall_check['passed']:
            raise RuntimeError(f'{condition}: wall check rejected approximation')
        s = line['station_m']
        k = line['curvature_1pm']
        ds = line['segment_lengths_m']
        curves = {}
        for lateral in (6., 10., 14., 18., 22.):
            result = estimate_closed_profile(k, ds, limits, lateral)
            speed = result['speed_mps']
            data = np.column_stack((s, line['xy_m'], line['yaw_rad'], k, result['tire_angle_rad'],
                                    speed, speed * 3.6, result['longitudinal_acceleration_mps2'],
                                    result['lateral_acceleration_mps2']))
            np.savetxt(output / f'{condition}_ay{int(lateral)}_estimate.csv', data, delimiter=',', comments='',
                       header='s_m,x_m,y_m,yaw_rad,curvature_1pm,tire_angle_rad,speed_mps,speed_kmh,ax_mps2,ay_mps2')
            curves[str(int(lateral))] = {'assumed_lateral_limit_mps2': lateral, 'lap_time_s': result['lap_time_s'],
                                        'speed_min_kmh': float(speed.min() * 3.6),
                                        'speed_max_kmh': float(speed.max() * 3.6),
                                        'distance_fraction_at_cap': float(ds[speed >= limits.speed_cap_mps - .01].sum() / ds.sum()),
                                        'constraint_residuals': result['constraint_residuals'], 'passes': result['passes']}
            axes[1, column].plot(s, speed * 3.6, label=f'ay limit {int(lateral)} m/s²', linewidth=1.5)
            if lateral == 14:
                illustrative = result
        stronger_brakes = estimate_closed_profile(k, ds, replace(limits, brake_command_cap_mps2=8.), 14.)
        illustrative_xy = line['xy_m'] - np.array([89600., 43100.])
        segments = np.stack((illustrative_xy, np.roll(illustrative_xy, -1, axis=0)), axis=1)
        collection = LineCollection(segments, cmap='turbo', norm=plt.Normalize(15, 35), linewidth=3)
        collection.set_array(illustrative['speed_mps'] * 3.6)
        axis = axes[0, column]
        axis.add_collection(collection)
        axis.plot(original_xy[:, 0] - 89600., original_xy[:, 1] - 43100., color='.55', linestyle='--', linewidth=1., label='Adopted CSV')
        axis.fill(polygon[:, 0] - 89600., polygon[:, 1] - 43100., color='#dc3545', alpha=.25, label='OT lane')
        peaks, _ = find_peaks(np.tile(np.abs(k), 3), prominence=.035,
                             distance=max(1, int(12 / np.mean(ds))))
        peaks = peaks[(peaks >= len(k)) & (peaks < 2 * len(k))] - len(k)
        for number, index in enumerate(peaks, 1):
            axis.annotate(f'C{number}', illustrative_xy[index], xytext=(5, 7), textcoords='offset points', fontsize=8,
                          bbox={'facecolor': 'white', 'edgecolor': 'none', 'alpha': .8, 'pad': 1})
            corner_rows.append({'line': condition, 'corner': f'C{number}', 'station_m': float(s[index]),
                                'radius_m': float(1 / abs(k[index])),
                                'required_tire_deg': float(abs(math.degrees(math.atan(limits.wheelbase_m * k[index])))),
                                'speed_ay14_kmh': float(illustrative['speed_mps'][index] * 3.6)})
        axis.set_aspect('equal')
        axis.set_xlabel('Map x - 89600 (m)')
        axis.set_ylabel('Map y - 43100 (m)')
        axis.set_title(f'{condition.title()} line shape · NO handicap in this estimate\nIllustration: assumed ay limit 14 m/s²; 35 km/h cap')
        axis.legend(fontsize=8)
        fig.colorbar(collection, ax=axis, label='Estimated speed (km/h)')
        axis = axes[1, column]
        axis.set_xlabel('Distance around lap (m)')
        axis.set_ylabel('Estimated speed (km/h)')
        axis.set_ylim(14, 36)
        axis.grid(alpha=.2)
        axis.legend(fontsize=8, ncol=2)
        report['conditions'][condition] = {'geometry': line['audit'], 'wall_check': wall_check,
                                            'scenarios': curves,
                                            'ay14_with_8mps2_brake_command_lap_s': stronger_brakes['lap_time_s']}
    fig.suptitle('Calculation only · nearby line approximation · lateral grip is assumed, not calibrated', fontsize=13)
    fig.savefig(output / 'speed_estimate.png', dpi=160)
    fig.savefig(output / 'speed_estimate.svg')
    plt.close(fig)
    report['corners'] = corner_rows
    (output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    (output / 'README.md').write_text(
        '# Offline 35 km/h estimate\n\n'
        'These CSVs are analysis artifacts, not drop-in controller references. '
        'The geometry is a nearby steering-feasible approximation of each selected CSV. '
        'Both shapes are evaluated with **normal, non-handicap** physics at 35 km/h; '
        'the leader-shape estimate does not predict native handicap performance.\n\n'
        'See report.json for assumed grip limits, physical parameters, source hashes, '
        'static footprint checks, and omitted dynamics. No new AWSIM trial was run.\n')
    print(json.dumps({key: value['scenarios'] for key, value in report['conditions'].items()}, indent=2))


if __name__ == '__main__':
    main()
