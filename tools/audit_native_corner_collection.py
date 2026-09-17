"""Audit per-corner native-object coverage in WSL; retain strict clearance gate.

Crossing a configured station proves route coverage, not mesh clearance or an
avoidance manoeuvre. The original raw recording is never rewritten.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import sys
from typing import Any

if __package__ in {None, ''}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def corner_coverage(samples: list[dict[str, Any]], locations: list[dict[str, Any]],
                    length_m: float, post_m: float = 25.) -> list[dict[str, Any]]:
    """Use GNSS progress [m] and ego simulation stamps [s], never wall time."""
    if not math.isfinite(length_m) or length_m <= 0 or not math.isfinite(post_m) or post_m < 0:
        raise ValueError('finite reference length and post distance required [m]')
    if len({p['object_id'] for p in locations}) != len(locations):
        raise ValueError('distinct object IDs required')
    valid = []
    for row in samples:
        ego, truth = row.get('ego') or {}, row.get('ego_gt') or {}
        if row.get('time', -1) < 0 or truth.get('source') != 'gnss':
            continue
        values = [truth.get(k) for k in ('progress_m', 'x', 'y')] + [ego.get('stamp')]
        if any(v is None or not math.isfinite(v) for v in values):
            continue
        valid.append((values[0], values[3], values[1], values[2]))
    output = []
    for placement in locations:
        station = placement['monitor_s_m']
        if not math.isfinite(station) or not 0 <= station < length_m:
            raise ValueError('object station outside circular reference [m]')
        row = dict(corner=placement['site'], object_id=placement['object_id'],
            object_type=placement['object_type'], configured_station_m=station,
            passed=False, post_distance_observed=False, crossing_sim_s=None,
            closest_gnss_to_configured_center_m=None)
        if not valid:
            output.append(row)
            continue
        target = station + max(0, math.ceil((valid[0][0]-station)/length_m))*length_m
        row['first_encounter_progress_m'] = target
        previous = None
        approached = False
        distances = []
        for progress, stamp, x, y in valid:
            if target-15 <= progress <= target+post_m:
                distances.append(math.hypot(x-placement['map_pose'][0], y-placement['map_pose'][1]))
            if target-15 <= progress <= target-2:
                approached = True
            if previous and not row['passed'] and approached:
                ps, pt = previous
                if ps <= target <= progress and 0 < stamp-pt <= .5 and 0 < progress-ps <= 2.:
                    row['passed'] = True
                    row['crossing_sim_s'] = pt+(stamp-pt)*(target-ps)/(progress-ps)
            if row['passed'] and progress >= target+post_m:
                row['post_distance_observed'] = True
            previous = (progress, stamp)
        row['closest_gnss_to_configured_center_m'] = min(distances) if distances else None
        output.append(row)
    return output


def sha(path: Path) -> str:
    h = hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda: f.read(4*1024*1024), b''):
            h.update(chunk)
    return h.hexdigest()


def audit(collected: Path, output: Path) -> dict[str, Any]:
    from tools.audit_lidar_v2x_obstacles import audit as audit_time_teachers
    manifest = json.loads((collected/'export_manifest.json').read_text())
    run_id = manifest['run_id']
    for name, item in manifest['files'].items():
        path = collected/name
        if not path.resolve().is_relative_to(collected.resolve()) or path.is_symlink():
            raise ValueError('manifest path escapes collection')
        assert path.stat().st_size == item['bytes'] and sha(path) == item['sha256'], name
    run = collected/'raw'/run_id
    metadata = json.loads((collected/'provenance/scenarios'/(run_id+'.json')).read_text())
    samples = [json.loads(line) for line in (run/'samples.jsonl').read_text().splitlines()]
    coverage = corner_coverage(samples, metadata['locations'], metadata['monitor_length_m'])
    matches = re.findall(r'Spawned (\d+) object\(s\) from ', (run/'awsim-player.log').read_text(errors='replace'))
    spawned = int(matches[-1]) if matches else None
    report = audit_time_teachers(collected, output)
    anchors = [json.loads(s) for s in (output/'anchors.jsonl').read_text().splitlines()]
    # These are diagnostic full-future windows, not promoted training examples.
    for item in coverage:
        stamp = item['crossing_sim_s']
        nearby = [r for r in anchors if stamp is not None and
                  abs(r['observation_ns']/1e9-stamp) <= 10.]
        item['nearby_camera_anchors_20s'] = len(nearby)
        item['nearby_avoid_mode_anchors_20s'] = sum(r['mode'] in {'AVOID','OVERTAKE'} for r in nearby)
    result = dict(run_id=run_id, configured_objects=len(coverage), spawned_objects=spawned,
        spawn_count_matches=spawned == len(coverage), covered_corners=sum(r['passed'] for r in coverage),
        corners_with_25m_post_observation=sum(r['post_distance_observed'] for r in coverage), corners=coverage,
        official_penalties=report['official_penalties'], finish_reached=report['finish_reached'],
        camera_anchors=report['camera_anchors'], full_future_anchors=report['full_future_anchors'],
        strict_forward_eligible_anchors=report['forward_avoidance_eligible_anchors'],
        clearance_quality='UNKNOWN_PREFAB_MESH', training_split_assigned=False,
        limitations=['Station crossing and centre distance are not footprint clearance.',
                     'Official contact counters do not guarantee detection of every physical contact.',
                     'All strict training masks remain subject to the existing 0.30 m clearance requirement.'])
    (output/'corner_coverage.json').write_text(json.dumps(result, indent=2)+'\n')
    return result


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--collected', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    print(json.dumps(audit(args.collected, args.output), indent=2))


if __name__ == '__main__':
    main()
