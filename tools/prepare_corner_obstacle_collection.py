"""Prepare one native box or cone per corner, all in the same PC10 run.

No driving occurs here. Map XY and station are metres; internal yaw is radians.
Each complete run remains unassigned until a grouped dataset split is chosen.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import json
import math
from pathlib import Path
import sys
from typing import Any

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.prepare_avoidance_scenarios import sha256, write_json


def collection_document(plan: dict[str, Any], name: str, members: list[dict[str, Any]],
                        length_m: float) -> dict[str, Any]:
    if not math.isfinite(length_m) or length_m <= 60:
        raise ValueError("monitor reference length must exceed 60 m")
    if not 1 <= len(members) <= 32 or len({p['site'] for p in members}) != len(members):
        raise ValueError('one object per distinct corner, maximum 32 objects')
    objects = []
    for i, p in enumerate(members):
        pose = p['map_pose']
        if len(pose) != 3 or not all(math.isfinite(v) for v in pose):
            raise ValueError('map_pose must be finite [x_m, y_m, yaw_rad]')
        kind = 'box' if i % 2 == 0 else 'cone'
        objects.append(dict(id=p['site']+'_'+kind, type=kind,
            pose=dict(map_xy=pose[:2], yaw_deg=math.degrees(math.atan2(math.sin(pose[2]), math.cos(pose[2]))))))
    document = dict(schema_version=1, name=name, seed=plan['seed'],
        simulator=dict(start_mode='count', start_count_seconds=5, laps='unlimited',
            timeout=plan['sim_timeout_s'], collisions='on', render='window', camera='gpu',
            lidar='gpu', npcs=0, handicap='off', ranking='off', wall_recovery='on'),
        runtime=dict(sample_rate_hz=20, ready_timeout_sec=180, playstart_stall_sec=30, rosbag=True),
        ego=dict(submission=dict(type='docker_image', image=plan['image']), pose=dict(start_grid=1)),
        objects=objects,
        expect=dict(timeout_sec=plan['sim_timeout_s'], no_collision=True, no_off_track=True))
    # The grid begins around s=24 m; a lap-counter finish would miss corner 01.
    # Scenario Tool compares this field against UNWRAPPED progress.
    document['expect']['finish'] = {'ego_reference_s_greater_than': round(length_m + 60., 3)}
    document['description'] = 'One native box/cone per corner. Official contacts and per-corner coverage must be audited; mesh clearance is unknown.'
    return document


def prepare(repo: Path, reference: Path, output: Path, prefix: str) -> dict[str, Any]:
    import re
    if not re.fullmatch(r'lidar-v45-pc10-[a-z0-9-]+', prefix):
        raise ValueError('safe collection run prefix required')
    if output.exists():
        raise FileExistsError(f'preserve existing scenarios: {output}')
    sys.path.insert(0, str(repo/'scenario_tool'))
    from scenario_tool import calibration, compiler, scenario, yamlio
    from scenario_tool.context import Context
    from scenario_tool.geometry import ReferenceLine, footprint, polygon_distance
    from scenario_tool.occupancy import OccupancyGrid
    from scenario_tool.regression import detect_road_sections, road_positions

    context = Context(repo)
    maps = context.map_paths()
    monitor = maps.reference_line()
    line = ReferenceLine.from_mpc_config(reference, .6, 2, True)
    occupancy = OccupancyGrid(maps.occupancy_yaml)
    if not occupancy.available:
        raise ValueError(occupancy.reason)
    transform = calibration.load(context, require=True)
    if calibration.staleness(context, transform, maps.reference_csv):
        raise ValueError('stale AWSIM calibration')
    sections = [s for s in detect_road_sections(line) if s.kind == 'corner']
    selected = []
    for section in sections:
        # Just inside the turn, so the teacher can initiate avoidance on approach.
        s = (section.s_start + min(2., (section.s_end-section.s_start) % line.length / 2.)) % line.length
        corridor = road_positions(line, occupancy, replace(section, target_s=s))
        nominal = max(corridor.safe_min, min(corridor.safe_max, 0.))
        candidates = []
        for delta in (0.4, -0.4, 0., 0.8, -0.8):
            d = max(corridor.safe_min, min(corridor.safe_max, nominal+delta))
            pose = line.pose_at(s, d)
            body = footprint(pose)
            if occupancy.polygon_overlap_depth(body) > 0:
                continue
            if polygon_distance(body, footprint(line.pose_at(s))) > .1:
                continue
            gaps = []
            for k in range(-35, 36):
                trial = footprint(line.pose_at(s, k/10.))
                gap = polygon_distance(body, trial)
                if gap >= .3 and occupancy.polygon_overlap_depth(trial) == 0:
                    gaps.append({'lateral_m': k/10., 'clearance_m': gap})
            if gaps:
                candidates.append((max(g['clearance_m'] for g in gaps), d, pose, gaps))
        if not candidates:
            raise ValueError(f'{section.id}: no footprint-safe placement with a passing cross-section')
        # Choose a modest offset with the widest feasible passing side.
        gap, d, pose, passing = max(candidates, key=lambda c: (min(c[0], 1.), -abs(c[1])))
        ms, md = monitor.project(pose.x, pose.y)
        selected.append(dict(site=section.id, section=asdict(section), context='entry',
            placement_id=section.id+'_entry', split='unassigned', split_group='all_corners_20260918',
            group='all_corners_20260918', teacher_s_m=s, monitor_s_m=ms, monitor_d_m=md,
            actual_lateral_m=d, map_pose=[pose.x, pose.y, pose.yaw],
            cross_section_passing_candidates=passing, status='CART_FOOTPRINT_PLACEMENT_PROXY_ONLY'))
    plan = dict(seed=20260918, image='aichallenge-2025-dev:latest', sim_timeout_s=780,
                finish_after_actor_m=25., min_clearance_m=.3)
    name = prefix
    document = collection_document(plan, name, selected, monitor.length)
    warnings = scenario.validate_document(document, context)
    resolved = scenario.resolve(document, context, monitor, transform, warnings=warnings)
    native = compiler.build_awsim_scenario(resolved, True)
    if len(native['vehicles']) != 1 or sum(len(v) for v in native.get('objects', {}).values()) != len(selected):
        raise ValueError('compiled native object identity/count mismatch')
    supported_warning = '物体はAWSIM標準形状です。物体別の離隔・通過判定は未対応で、接触は公式wall/obstacleカウンタで監視します。'
    unexpected = [w for w in resolved.warnings if w != supported_warning]
    if unexpected:
        raise ValueError(f'{name}: placement warnings: {unexpected}')
    locations = [dict(p, object_id=o['id'], object_type=o['type']) for p, o in zip(selected, document['objects'], strict=True)]
    metadata = dict(schema_version=1, cases=[name], locations=locations, split='unassigned',
            split_policy='Entire suite remains grouped; no frame split and no automatic training registration.',
            teacher_revision='lidar-motion-intent-r2', monitor_length_m=monitor.length,
            required_post_actor_m=25., required_min_clearance_m=.3,
            all_object_audit_required=True, teacher_reference_sha256=sha256(reference),
            geometry_sha256=sha256(Path(sys.modules['scenario_tool.geometry'].__file__)),
            clearance_quality='UNKNOWN_PREFAB_MESH', warnings=resolved.warnings,
            static_geometry_only=True, awsim_modified=False)
    output.mkdir(parents=True)
    yamlio.dump_file(output/(name+'.yaml'), document)
    scenario.validate_document(yamlio.load_file(output/(name+'.yaml')), context)
    write_json(output/(name+'.json'), metadata)
    write_json(output/(name+'-native.json'), native)
    summary = dict(corners=len(selected), runs=[dict(run_id=name, corners=[p['site'] for p in locations])],
        native_types=[o['type'] for o in document['objects']],
        reference_length_m=line.length, monitor_length_m=monitor.length,
        finish_progress_m=round(monitor.length+60, 3), speed_cap_kmh=5,
        teacher_reference_sha256=sha256(reference), training_split_assigned=False,
        limitations=['Static cross-section feasibility does not prove dynamic avoidance.',
                     'Native prefab mesh dimensions are unknown; no 0.30 m clearance claim.'])
    write_json(output/'summary.json', summary)
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--awsim-repo', type=Path, required=True)
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--prefix', required=True)
    args = parser.parse_args()
    print(json.dumps(prepare(args.awsim_repo.resolve(), args.reference.resolve(), args.output.resolve(),
                             args.prefix), indent=2))


if __name__ == '__main__':
    main()
