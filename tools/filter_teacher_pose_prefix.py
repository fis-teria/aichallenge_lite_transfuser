"""Keep the verified pre-drift prefix of an existing obstacle-teacher audit.

Run under the native WSL worktree lock. Original raw recordings and audit files
remain unchanged; the output is an additional exclusion, never an admission override.
"""
from __future__ import annotations

import argparse
from bisect import bisect_left
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
from rosbags.highlevel import AnyReader

from aic_transfuser_lite.data.teacher_pose_prefix import (
    HeadingPrefixConfig, heading_prefix, mask_teacher_arrays, prefix_anchor_mask, wrap_rad,
)


def sha(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(4*1024*1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path: Path, value: Any) -> None:
    with path.open('x', encoding='utf-8') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')


def verified_bag(collected: Path, run_id: str) -> tuple[Path, dict[str, str]]:
    receipt = json.loads((collected/'transfer_verified.json').read_text())
    manifest = json.loads((collected/'export_manifest.json').read_text())
    if receipt.get('all_sha256_match') is not True or any(x.get('run_id') != run_id for x in (receipt, manifest)):
        raise ValueError('verified source/run identity required')
    bag = collected/'raw'/run_id/'d1/rosbag2_autoware'
    checked = {}
    for name, item in manifest['files'].items():
        if '/rosbag2_autoware/' not in name:
            continue
        path = collected/name
        if not path.resolve().is_relative_to(collected.resolve()) or path.is_symlink():
            raise ValueError('source bag path escapes collection')
        actual = sha(path)
        if actual != item['sha256'] or path.stat().st_size != item['bytes']:
            raise ValueError(f'source bag checksum mismatch: {name}')
        checked[name] = actual
    relative = {str(p.relative_to(collected)).replace('\\','/') for p in bag.iterdir() if p.is_file()}
    if not relative or not relative.issubset(checked):
        raise ValueError('source bag contains unverified files')
    return bag, checked


def paired_heading_samples(bag: Path, max_gap_ns: int) -> tuple[list[tuple[int, float]], dict[str, Any]]:
    """Compare timestamped EKF and corrected IMU yaw; no wall/map fitting."""
    topics = {'/sensing/imu/imu_data': 'imu', '/localization/kinematic_state': 'odom'}
    streams: dict[str, list[tuple[int, float]]] = {'imu': [], 'odom': []}
    frames: dict[str, set[str]] = {'imu': set(), 'odom': set()}
    duplicates = {'imu': 0, 'odom': 0}
    with AnyReader([bag]) as reader:
        for connection, _, raw in reader.messages(connections=[c for c in reader.connections if c.topic in topics]):
            m = reader.deserialize(raw, connection.msgtype)
            role = topics[connection.topic]
            frames[role].add(m.header.frame_id)
            if role == 'odom' and m.child_frame_id != 'base_link':
                raise ValueError('odometry child frame is not base_link')
            q = m.orientation if role == 'imu' else m.pose.pose.orientation
            quat = np.asarray([q.x, q.y, q.z, q.w])
            valid = bool(np.isfinite(quat).all() and abs(np.dot(quat,quat)-1.) <= .01)
            if role == 'imu' and m.orientation_covariance[0] < 0:
                valid = False
            angle = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)) if valid else math.nan
            stamp = int(m.header.stamp.sec)*1_000_000_000+int(m.header.stamp.nanosec)
            previous = streams[role][-1] if streams[role] else None
            if previous and stamp < previous[0]:
                raise ValueError('capture clock reversal needs a separate epoch')
            if previous and stamp == previous[0]:
                duplicates[role] += 1
                if not math.isfinite(angle) or not math.isfinite(previous[1]) or abs(float(wrap_rad(angle-previous[1]))) > 1e-8:
                    raise ValueError('conflicting heading at duplicate capture stamp')
                continue
            streams[role].append((stamp,angle))
    if frames != {'imu': {'base_link'}, 'odom': {'map'}} or any(len(v) < 2 for v in streams.values()):
        raise ValueError('required map/base_link EKF and corrected IMU heading streams unavailable')
    ot = [t for t,_ in streams['odom']]
    oa = [a for _,a in streams['odom']]
    pairs = []
    for stamp, angle in streams['imu']:
        i = bisect_left(ot,stamp)
        estimate = math.nan
        if i < len(ot) and ot[i] == stamp:
            estimate = oa[i]
        elif (0 < i < len(ot) and stamp-ot[i-1] <= max_gap_ns and ot[i]-stamp <= max_gap_ns
              and ot[i]-ot[i-1] <= max_gap_ns):
            estimate = oa[i-1]+float(wrap_rad(oa[i]-oa[i-1]))*(stamp-ot[i-1])/(ot[i]-ot[i-1])
        pairs.append((stamp,float(wrap_rad(estimate-angle))))
    return pairs, dict(topics=topics,counts={k:len(v) for k,v in streams.items()},
                      duplicate_identical_captures=duplicates,frames={k:sorted(v) for k,v in frames.items()})


def filter_audit(collected: Path, audit: Path, output: Path,
                 config: HeadingPrefixConfig = HeadingPrefixConfig()) -> dict[str, Any]:
    if output.exists():
        raise FileExistsError('preserve previous results; choose a new output directory')
    report = json.loads((audit/'audit.json').read_text())
    run_id = report['run_id']
    source_files = {name:sha(audit/name) for name in ('audit.json','anchors.jsonl','observed_teachers.npz')}
    rows = [json.loads(line) for line in (audit/'anchors.jsonl').read_text().splitlines() if line.strip()]
    if not rows or len({r['epoch'] for r in rows}) != 1:
        raise ValueError('one nonempty audited epoch is required')
    if any(r['run_id'] != run_id or r['label_index'] != i for i,r in enumerate(rows)):
        raise ValueError('audited anchor order/identity mismatch')
    epoch_bounds = {tuple(r['epoch_bounds_ns']) for r in rows}
    if len(epoch_bounds) != 1:
        raise ValueError('inconsistent epoch bounds')
    bounds = next(iter(epoch_bounds))
    bag, source_bags = verified_bag(collected,run_id)
    pairs, sensor_report = paired_heading_samples(bag,config.max_gap_ns)
    quality = heading_prefix(pairs,drive_start_ns=report['driving_start_ns'],observed_end_ns=bounds[1],config=config)
    trace = quality.pop('trace')
    with np.load(audit/'observed_teachers.npz',allow_pickle=False) as bundle:
        arrays = {name:bundle[name].copy() for name in bundle.files}
    if not np.array_equal(arrays['observation_ns'],np.asarray([r['observation_ns'] for r in rows],dtype=np.int64)):
        raise ValueError('label and anchor timestamps differ')
    contract = report['label_contract']
    tolerance_ms = float(contract['config']['teacher_tolerance_ms'])
    if contract['dt_s'] != .1 or contract['shape'] != [len(rows),30,2] or not math.isfinite(tolerance_ms) or tolerance_ms < 0:
        raise ValueError('expected 30 x 0.1 s teacher contract')
    tolerance_ns = round(tolerance_ms*1e6)
    keep = prefix_anchor_mask(arrays['observation_ns'],valid_from_ns=quality['valid_from_ns'],
                              valid_until_ns=quality['valid_until_ns'],endpoint_tolerance_ns=tolerance_ns)
    filtered = mask_teacher_arrays(arrays,keep)
    filtered_rows = []
    candidates = []
    sensor_reasons = {'STARTUP','HISTORY_GAP','INCOMPLETE_INPUT_OR_FUTURE','REVERSE_IN_HISTORY_OR_FUTURE',
                      'TEACHER_UNAVAILABLE_OR_EMERGENCY','PERCEPTION_UNAVAILABLE'}
    for i, source_row in enumerate(rows):
        row = dict(source_row)
        row['rejection_reasons'] = list(source_row['rejection_reasons'])
        row['teacher_reasons'] = list(source_row['teacher_reasons'])
        row['teacher_pose_prefix_eligible'] = bool(keep[i])
        if not keep[i]:
            row['rejection_reasons'].append('OUTSIDE_VERIFIED_TEACHER_POSE_PREFIX')
            row['teacher_reasons'].append('TEACHER_POSE_QUALITY_END')
            row.update(usable_full=False,usable_partial=False,forward_avoidance_eligible=False)
        if (keep[i] and row['usable_full'] and filtered['xy_mask'][i].all()
                and not sensor_reasons.intersection(row['rejection_reasons'])):
            candidates.append(i)
        filtered_rows.append(row)
    if any(sha(audit/name) != digest for name,digest in source_files.items()):
        raise ValueError('source audit changed during filtering')
    output.mkdir(parents=True,exist_ok=False)
    np.savez_compressed(output/'observed_teachers.npz',**filtered)
    (output/'anchors.jsonl').write_text(''.join(json.dumps(r,separators=(',',':'))+'\n' for r in filtered_rows))
    indices = np.asarray(candidates,dtype=np.int64)
    np.savez_compressed(output/'prefix_candidates.npz',source_label_index=indices,
                        **{key:value[indices] for key,value in filtered.items()})
    candidate_rows = [dict(filtered_rows[i],source_label_index=i,label_index=j) for j,i in enumerate(candidates)]
    (output/'prefix_candidates.jsonl').write_text(''.join(json.dumps(r,separators=(',',':'))+'\n' for r in candidate_rows))
    (output/'heading_consistency.jsonl').write_text(''.join(json.dumps(r,separators=(',',':'))+'\n' for r in trace))
    summary = dict(run_id=run_id,quality=quality,sensors=sensor_report,epoch=rows[0]['epoch'],
        camera_anchors=len(rows),source_full_future_anchors=int(arrays['xy_mask'].all(axis=1).sum()),
        prefix_full_future_anchors=int(filtered['xy_mask'].all(axis=1).sum()),
        prefix_candidate_anchors=len(candidates),excluded_by_prefix=int((~keep).sum()),
        strict_forward_eligible_anchors=int(filtered['forward_avoidance_eligible'].sum()),
        first_candidate_ns=rows[candidates[0]]['observation_ns'] if candidates else None,
        last_candidate_ns=rows[candidates[-1]]['observation_ns'] if candidates else None,
        future_horizon_ns=3_000_000_000,interpolation_endpoint_guard_ns=tolerance_ns,
        source_collected=str(collected.resolve()),source_audit=str(audit.resolve()),
        source_bag_sha256=source_bags,source_audit_sha256=source_files,
        training_split_assigned=False,automatic_training_admission=False,
        limitations=['Relative heading consistency does not certify absolute initial pose or translation.',
                     'Existing clearance/contact/input exclusions are preserved; candidates are not admitted training data.',
                     'Same-run prefixes retain the original run/scenario split identity.',
                     'Original recordings and audits remain unchanged; no inferred stop-intent label.'])
    summary['output_sha256'] = {p.name:sha(p) for p in output.iterdir() if p.is_file()}
    write_json(output/'pose_prefix.json',summary)
    print(json.dumps({k:summary[k] for k in ('run_id','prefix_candidate_anchors','strict_forward_eligible_anchors')}
                     | {'quality':quality},separators=(',',':')),flush=True)
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--collected',type=Path,required=True)
    parser.add_argument('--audit',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--max-heading-error-deg',type=float,default=5.)
    args = parser.parse_args()
    filter_audit(args.collected,args.audit,args.output,
                 replace(HeadingPrefixConfig(),max_error_rad=math.radians(args.max_heading_error_deg)))


if __name__ == '__main__':
    main()
