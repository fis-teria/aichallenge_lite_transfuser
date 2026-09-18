"""Select observed XY/speed teachers from verified pre-drift native-object data.

Run in native WSL under with_wsl_training_lock.sh. Writes a separate immutable
selection bundle; original strict avoidance masks and split assignments survive.
"""
from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict
import json
import math
from pathlib import Path
import subprocess
import sys
from typing import Any

import numpy as np
from PIL import Image
from rosbags.highlevel import AnyReader
from scipy.ndimage import binary_erosion
from scipy.spatial import cKDTree
import yaml

from aic_transfuser_lite.data.native_teacher_curation import (
    NativeCurationConfig, classify_anchor, convex_point_distance, spaced_indices,
    teacher_command_evidence, window_indices,
)
from aic_transfuser_lite.runtime.lidar_map_localization import transform

# Direct `python tools/...py` places tools/ rather than the repository on sys.path.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.filter_teacher_pose_prefix import sha, verified_bag, write_json


def read_rows(path: Path) -> list[dict[str, Any]]:
    return [json.loads(s) for s in path.read_text().splitlines() if s.strip()]


def stamp_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def yaw_of(q: Any) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def load_wall_map(root: Path) -> tuple[cKDTree, np.ndarray, dict[str, str]]:
    directory = root / 'pc10_physical_wall_map'
    proof = json.loads((directory / 'provenance.json').read_text())
    pgm = directory / 'occupancy_grid_map.pgm'
    if sha(pgm) != proof['output_pgm_sha256'] or proof['awsim_level_sha256'] != '9ab2e1e8865c02885594e0bbdde372302530f047b5a2e89c455be6a18f3e090b':
        raise ValueError('physical map provenance mismatch')
    meta = yaml.safe_load((directory / 'occupancy_grid_map.yaml').read_text())
    pixels = np.flipud(np.asarray(Image.open(pgm)))
    occupied = pixels < 100
    yy, xx = np.nonzero(occupied & ~binary_erosion(occupied))
    points = np.column_stack([xx + .5, yy + .5]) * float(meta['resolution']) + np.asarray(meta['origin'][:2])
    hull = np.asarray(proof['body_xy_vertices'], dtype=float)
    convex_point_distance(np.zeros((1, 2)), hull)
    return cKDTree(points), hull, {p.name: sha(p) for p in directory.iterdir() if p.name in {
        'provenance.json', 'occupancy_grid_map.pgm', 'occupancy_grid_map.yaml'}}


def unique_series(rows: dict[int, Any]) -> tuple[np.ndarray, np.ndarray]:
    times = np.asarray(sorted(rows), dtype=np.int64)
    return times, np.asarray([rows[int(t)] for t in times])


def extract_evidence(bag: Path, low: int, high: int, placements: list[dict[str, Any]],
                     wall: cKDTree, hull: np.ndarray, config: NativeCurationConfig) -> dict[str, np.ndarray]:
    """Recorded pose/scan consistency; no pose correction or label regeneration."""
    poses: dict[int, Any] = {}; scans: dict[int, Any] = {}; commands: dict[int, Any] = {}
    perception: dict[int, Any] = {}; static: dict[str, Any] = {}
    wanted = {'/localization/kinematic_state', '/sensing/lidar/scan',
              '/mppi/direct/trajectory_command', '/collection/lidar_v2x/objects'}
    with AnyReader([bag]) as reader:
        for conn, _, blob in reader.messages(connections=[c for c in reader.connections if c.topic == '/tf_static']):
            for t in reader.deserialize(blob, conn.msgtype).transforms:
                v, q = t.transform.translation, t.transform.rotation
                row = (t.header.frame_id, [v.x, v.y, v.z], [q.x, q.y, q.z, q.w])
                if t.child_frame_id in static and static[t.child_frame_id] != row:
                    raise ValueError('conflicting sensor mount')
                static[t.child_frame_id] = row
        if static['lidar'][0] != 'lidar_base_link' or static['lidar_base_link'][0] != 'base_link':
            raise ValueError('unsupported scan mount chain')
        for key in ('lidar', 'lidar_base_link'):
            np.testing.assert_allclose(static[key][2], [0., 0., 0., 1.], atol=1e-8)
        mount = np.asarray(static['lidar'][1]) + np.asarray(static['lidar_base_link'][1])
        np.testing.assert_allclose(mount[:2], [1.65, 0.], atol=1e-7)
        for conn, _, blob in reader.messages(connections=[c for c in reader.connections if c.topic in wanted],
                                             start=max(0, low - 500_000_000), stop=high + 500_000_000):
            m = reader.deserialize(blob, conn.msgtype)
            if conn.topic.endswith('/objects'):
                d = json.loads(m.data)
                perception[round(d['stamp_s'] * 1e9)] = bool(d.get('teacher_ready') is True
                    and d.get('stop_requested') is False and d.get('source') == 'lidar')
                continue
            t = stamp_ns(m.header.stamp)
            if conn.topic.endswith('/kinematic_state'):
                if m.header.frame_id != 'map' or m.child_frame_id != 'base_link':
                    raise ValueError('unexpected odometry frame')
                p = m.pose.pose
                poses[t] = [p.position.x, p.position.y, yaw_of(p.orientation)]
            elif conn.topic.endswith('/trajectory_command'):
                commands.setdefault(t, []).append((m.mode, m.emergency_stop, m.reason))
            else:
                if m.header.frame_id != 'lidar' or len(m.ranges) != 750:
                    raise ValueError('unexpected native LiDAR contract')
                scans[t] = (np.asarray(m.ranges, float), float(m.angle_min), float(m.angle_increment),
                            float(m.range_min), float(m.range_max))
    pose_t, pose = unique_series(poses)
    if pose.shape != (len(pose_t), 3) or len(pose_t) < 2 or not np.isfinite(pose).all():
        raise ValueError('invalid recorded pose support')
    pose[:, 2] = np.unwrap(pose[:, 2])
    objects = {kind: np.asarray([x['map_pose'][:2] for x in placements if x['object_type'] == kind], float)
               for kind in ('box', 'cone')}
    if any(v.shape != (6, 2) for v in objects.values()):
        raise ValueError('expected six boxes and six static cones')
    box_distance, cone_distance, cone_gap = [], [], []
    for p in pose:
        body = transform(hull, p)
        box_distance.append(float(np.linalg.norm(objects['box'] - p[:2], axis=1).min()))
        cone_distance.append(float(np.linalg.norm(objects['cone'] - p[:2], axis=1).min()))
        cone_gap.append(max(0., float(convex_point_distance(objects['cone'], body).min()) - .175))
    scan_t, registration = [], []
    for t, (ranges, amin, inc, rmin, rmax) in sorted(scans.items()):
        metrics = [0., 1e6, 0., 0., 0.]
        bounds = window_indices(pose_t, t, t, config.max_evidence_gap_ns)
        valid = np.flatnonzero(np.isfinite(ranges) & (ranges > max(1., rmin)) & (ranges < min(12., rmax)))[::3]
        if bounds is not None and len(valid):
            p = np.asarray([np.interp(t, pose_t, pose[:, i]) for i in range(3)])
            angles = amin + valid * inc
            xy = np.column_stack([ranges[valid] * np.cos(angles), ranges[valid] * np.sin(angles)]) + mount[:2]
            world = transform(xy, p)
            distances = wall.query(world)[0]
            metrics = [1., float(np.median(distances)), float(np.mean(distances <= config.map_inlier_distance_m)),
                       float(len(valid)), float(np.ptp(angles))]
        scan_t.append(t); registration.append(metrics)
    command_t, command = unique_series({t: teacher_command_evidence(group) for t, group in commands.items()})
    perception_t, ready = unique_series(perception)
    if not len(command_t) or not len(perception_t) or not len(scan_t):
        raise ValueError('missing curation evidence stream')
    return dict(pose_t=pose_t, box_distance_m=np.asarray(box_distance), cone_distance_m=np.asarray(cone_distance),
                cone_gap_m=np.asarray(cone_gap), scan_t=np.asarray(scan_t, np.int64), registration=np.asarray(registration),
                command_t=command_t, command=command, perception_t=perception_t, ready=ready)


def summarize_window(series: dict[str, np.ndarray], low: int, high: int,
                     config: NativeCurationConfig) -> dict[str, Any]:
    bounds = {key: window_indices(series[key + '_t'], low, high, config.max_evidence_gap_ns)
              for key in ('pose', 'scan', 'command', 'perception')}
    if any(value is None for value in bounds.values()):
        return {'coverage_ok': False}
    windows = {key: slice(*value) for key, value in bounds.items() if value is not None}
    r = series['registration'][windows['scan']]
    command = series['command'][windows['command']]
    return dict(coverage_ok=True, teacher_ok=bool(command[:, 0].all()),
                all_free_run=bool(command[:, 1].all()), perception_ok=bool(series['ready'][windows['perception']].all()),
                pose_geometry_ok=bool(r[:, 0].all()),
                box_distance_m=float(series['box_distance_m'][windows['pose']].min()),
                cone_distance_m=float(series['cone_distance_m'][windows['pose']].min()),
                cone_gap_m=float(series['cone_gap_m'][windows['pose']].min()),
                map_median_max_m=float(r[:, 1].max()), map_inlier_fraction_min=float(r[:, 2].min()),
                wall_points_min=int(r[:, 3].min()), scan_span_min_rad=float(r[:, 4].min()))


def curate(root: Path, output: Path, config: NativeCurationConfig = NativeCurationConfig()) -> dict[str, Any]:
    root, output = root.resolve(), output.resolve()
    if output.exists() or output.is_relative_to(root / 'collected') or output.is_relative_to(root / 'pose_prefix_v2'):
        raise ValueError('new output directory outside original data required')
    tree, hull, map_hashes = load_wall_map(root)
    output.mkdir(parents=True)
    summaries: list[dict[str, Any]] = []
    for prefix in sorted((root / 'pose_prefix_v2').glob('lidar-v45-pc10-corners-native-*')):
        run = prefix.name; collected = root / 'collected' / run; target = output / run; target.mkdir()
        quality = json.loads((prefix / 'pose_prefix.json').read_text())
        for file, digest in quality['output_sha256'].items():
            if sha(prefix / file) != digest:
                raise ValueError('prefix source checksum mismatch')
        rows = read_rows(prefix / 'prefix_candidates.jsonl')
        if (quality['future_horizon_ns'] != config.horizon_ns
                or quality['interpolation_endpoint_guard_ns'] > config.endpoint_guard_ns
                or any(r['epoch'] != quality['epoch'] for r in rows)):
            raise ValueError('prefix time horizon, support or epoch contract mismatch')
        clearance = root / 'native_prefix_clearance_v2' / run
        prior = json.loads((clearance / 'summary.json').read_text())
        identity = dict(pose_prefix_sha256=sha(prefix / 'pose_prefix.json'), clearance_summary_sha256=sha(clearance / 'summary.json'))
        if not rows:
            summary = dict(run_id=run,source_camera_anchors=quality['camera_anchors'],prefix_candidates=0,
                           eligible=0,selected=0,hold=0,exclude=0,prefix_reason=quality['quality']['reason'],identity=identity)
            write_json(target / 'summary.json', summary); summaries.append(summary); continue
        checks = read_rows(clearance / 'candidate_checks.jsonl')
        if sha(clearance / 'candidate_checks.jsonl') != prior['output_sha256']['candidate_checks.jsonl'] or len(checks) != len(rows):
            raise ValueError('clearance source checksum/count mismatch')
        manifest = json.loads((collected / 'export_manifest.json').read_text())
        metadata_path = f'provenance/scenarios/{run}.json'
        if sha(collected / metadata_path) != manifest['files'][metadata_path]['sha256']:
            raise ValueError('scenario metadata checksum mismatch')
        placements = json.loads((collected / metadata_path).read_text())['locations']
        groups = sorted({p['split_group'] for p in placements})
        if len(groups) != 1 or any(p['split'] != 'unassigned' for p in placements):
            raise ValueError('respect existing scenario split before curation')
        if any(x['count'] for x in prior['official_penalties'].values()):
            raise ValueError('nonzero official penalties require time-local contact review')
        bag, bag_hashes = verified_bag(collected, run)
        identity.update(source_bag=str(bag),source_bag_sha256=bag_hashes,scenario_sha256=sha(collected / metadata_path))
        with np.load(prefix / 'prefix_candidates.npz', allow_pickle=False) as file:
            arrays = {key: file[key].copy() for key in file.files}
        if (not np.array_equal(arrays['observation_ns'], [r['observation_ns'] for r in rows])
                or arrays['forward_avoidance_eligible'].any()):
            raise ValueError('source candidate identity/strict mask differs from expected cohort')
        begin = max(0, int(arrays['observation_ns'][0]) - config.history_ns - config.endpoint_guard_ns)
        end = int(arrays['observation_ns'][-1]) + config.horizon_ns + config.endpoint_guard_ns
        if end >= quality['quality']['valid_until_ns']:
            raise ValueError('candidate future crosses pose cutoff')
        series = extract_evidence(bag, begin, end, placements, tree, hull, config)
        np.savez_compressed(target / 'window_evidence.npz', **series)
        decisions = []
        for i, (row, check) in enumerate(zip(rows, checks)):
            for key in ('run_id','epoch','observation_ns','source_label_index'):
                if row[key] != check[key]:
                    raise ValueError('clearance and candidate identity mismatch')
            t = int(row['observation_ns']); lo = t - config.history_ns - config.endpoint_guard_ns
            hi = t + config.horizon_ns + config.endpoint_guard_ns
            evidence = summarize_window(series, lo, hi, config) if lo >= 0 else {'coverage_ok':False}
            decision = classify_anchor(xy_m=arrays['xy_m'][i],xy_mask=arrays['xy_mask'][i],
                velocity_mps=arrays['velocity_mps'][i],velocity_mask=arrays['velocity_mask'][i],
                findings=check['findings'],evidence=evidence,config=config)
            decisions.append(dict(anchor_id=row['anchor_id'],run_id=run,epoch=row['epoch'],
                source_prefix_index=i,source_label_index=row['source_label_index'],observation_ns=t,
                split_group=groups[0],split='unassigned',window_ns=[lo,hi],evidence=evidence,**decision))
        eligible = np.asarray([r['selected'] for r in decisions],dtype=bool)
        indices = spaced_indices(arrays['observation_ns'],eligible,config.minimum_anchor_spacing_ns)
        chosen = set(indices.tolist())
        with (target / 'decisions.jsonl').open('x') as stream:
            for i,row in enumerate(decisions):
                row['selected'] = i in chosen
                if eligible[i] and i not in chosen:
                    row['disposition'] = 'eligible_thinned';row['reasons'] = ['TEMPORAL_THINNING_ONLY']
                stream.write(json.dumps(row,allow_nan=False,separators=(',',':'))+'\n')
        selected_rows = [dict(rows[i],label_index=j,source_prefix_index=int(i),
            selection_use=decisions[i]['use'],split_group=groups[0],split='unassigned',
            allowed_targets=['xy_m','velocity_mps'],stop_label_valid=False,mode_label_valid=False,
            curation_evidence=decisions[i]['evidence']) for j,i in enumerate(indices)]
        with (target / 'selected_anchors.jsonl').open('x') as stream:
            for row in selected_rows:stream.write(json.dumps(row,allow_nan=False,separators=(',',':'))+'\n')
        selected = {key:value[indices] for key,value in arrays.items()}
        selected.update(source_prefix_index=indices,stop_mask=np.zeros(len(indices),bool),mode_mask=np.zeros(len(indices),bool))
        np.savez_compressed(target / 'selected_teachers.npz', **selected)
        count = Counter(r['disposition'] for r in decisions)
        reasons = Counter(reason for r in decisions for reason in r['reasons'])
        summary = dict(run_id=run,source_camera_anchors=quality['camera_anchors'],prefix_candidates=len(rows),
            eligible=int(eligible.sum()),selected=len(indices),hold=count['hold'],exclude=count['exclude'],
            eligible_thinned=count['eligible_thinned'],uses=dict(Counter(r['selection_use'] for r in selected_rows)),
            reasons=dict(reasons),split_group=groups[0],split='unassigned',identity=identity,
            output_sha256={p.name:sha(p) for p in target.iterdir() if p.is_file()})
        write_json(target / 'summary.json', summary); summaries.append(summary)
        print(json.dumps({k:summary[k] for k in ('run_id','prefix_candidates','eligible','selected','hold','exclude','uses')},separators=(',',':')),flush=True)
    if not summaries:
        raise ValueError('no source prefixes found')
    report = dict(format='native_observed_xy_speed_selection_v1',config=asdict(config),runs=summaries,
        totals={key:sum(r.get(key,0) for r in summaries) for key in
            ('source_camera_anchors','prefix_candidates','eligible','selected','hold','exclude','eligible_thinned')},
        source_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        map_sha256=map_hashes,training_performed=False,split_assigned=False,
        allowed_targets=['xy_m','velocity_mps'],stop_mode_supervision=False,strict_avoidance_certification=False,
        teacher_command_quality=dict(require_nonempty_reason=True,
            reject_reason_tokens=['infeasible_braking_fallback'],
            duplicate_capture_stamps='all_publications_must_pass',
            inspect_full_history_and_future=True),
        limitations=['Observed-motion quality selection, not a physical contact oracle or certified avoidance-success set.',
            'Additional 0.30 m projected margin is an audit screen, not a statistical pose uncertainty bound.',
            'Initial box proximity is only an exclusion screen; no box position is invented.',
            'Scan/map agreement uses visible wall points only, with recorded EKF pose; it does not repair teacher poses.',
            'All runs share a scenario group; do not frame-split or distribute this group across train/validation/test.',
            'Only moving futures are selected. Stop intent requires separately verified data.'])
    report['output_sha256'] = {str(p.relative_to(output)):sha(p) for p in output.rglob('*') if p.is_file()}
    write_json(output / 'selection_manifest.json',report)
    print(json.dumps(report['totals']),flush=True)
    return report


def main() -> None:
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    curate(args.root,args.output)


if __name__=='__main__':
    main()
