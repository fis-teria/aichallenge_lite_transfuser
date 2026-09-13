"""Read a closed raw pilot in WSL; measure recovery without fitting a model."""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sqlite3

import numpy as np
from rosbags.typesys import Stores, get_typestore, get_types_from_idl

from aic_transfuser_lite.data.time_recovery_collection_v1 import (
    PhaseWindow, phase_at_s, project_course, recovery_teacher_mask,
)


def sha(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4*1024**2), b''):
            digest.update(block)
    return digest.hexdigest()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--run', type=Path, required=True)
    ap.add_argument('--types', type=Path, required=True, help='IDL snapshot from the recording image')
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    run = args.run.resolve()
    result = json.loads((run/'result.json').read_text())
    if not result.get('nodes', {}).get('closed_bag'):
        raise ValueError('BAG_NOT_FINALIZED')
    manifest = json.loads((run/'transfer_manifest.json').read_text())
    for name, item in manifest.items():
        path = (run/name).resolve()
        if not path.is_relative_to(run) or path.stat().st_size != item['bytes'] or sha(path) != item['sha256']:
            raise ValueError('TRANSFER_IDENTITY:'+name)
    reference = json.loads((run/'reference.json').read_text())
    store = get_typestore(Stores.ROS2_HUMBLE); definitions = {}
    for path in args.types.rglob('*.idl'):
        definitions.update(get_types_from_idl(path.read_text()))
    store.register(definitions)
    dbs = list((run/'bag').glob('*.db3'))
    if len(dbs) != 1:
        raise ValueError('EXPECTED_ONE_SQLITE_BAG')
    conn = sqlite3.connect('file:'+str(dbs[0])+'?mode=ro&immutable=1', uri=True)
    if conn.execute('PRAGMA quick_check').fetchone()[0] != 'ok':
        raise ValueError('SQLITE_INTEGRITY')
    topics = {n:(i,t) for i,n,t in conn.execute('SELECT id,name,type FROM topics')}
    counts = dict(conn.execute('SELECT topics.name, COUNT(*) FROM messages JOIN topics ON topics.id=messages.topic_id GROUP BY topics.id'))

    def messages(topic):
        index, kind = topics[topic]
        for receipt, blob in conn.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp,id', (index,)):
            yield int(receipt), store.deserialize_cdr(bytes(blob),kind)

    def stamp(t):
        return int(t.sec)*10**9+int(t.nanosec)

    sensor_meta = {}
    camera_ns = []
    for role,topic in [('camera','/sensing/camera/image_raw'),('lidar','/sensing/lidar/scan')]:
        times=[]; shapes=Counter(); invalid=0
        for _,m in messages(topic):
            times.append(stamp(m.header.stamp))
            if role=='camera':
                shapes[(m.height,m.width,m.encoding)] += 1
                invalid += len(m.data) != m.step*m.height
            else:
                ranges=np.asarray(m.ranges)
                shapes[(len(ranges),m.header.frame_id)] += 1
                invalid += bool(np.isnan(ranges).any() or np.isneginf(ranges).any())
        delta=np.diff(times)
        sensor_meta[role]=dict(count=len(times),shapes=[dict(shape=list(k),count=v) for k,v in shapes.items()],
            backward_headers=int((delta<0).sum()),maximum_capture_gap_s=float(delta.max()/1e9) if len(delta) else None,
            invalid_messages=int(invalid))
        if role=='camera': camera_ns=times
    control=[json.loads(line) for line in (run/'control.jsonl').read_text().splitlines()]
    track=[r for r in control if r['reason']=='RECOVERY_TEACHER_TRACKING']
    final=result.get('last_control',{}); armed=final.get('armed_ns'); end=final.get('sim_ns')
    # Duplicate frozen-clock ticks overwrite at the same original sim timestamp.
    phases={r['sim_ns']:r['phase'] for r in control if r['sim_ns'] is not None}
    windows=[]
    ordered=sorted(phases)
    for a,b in zip(ordered,ordered[1:]):
        phase=phases[a] if b-a<=150_000_000 else 'invalid'
        if windows and windows[-1].phase==phase and windows[-1].end_ns==a:
            windows[-1]=PhaseWindow(windows[-1].start_ns,b,phase)
        else:
            windows.append(PhaseWindow(a,b,phase))
    pose_rows=[]; rejections=Counter(); excluded_pose_phases=Counter()
    for _,m in messages('/localization/kinematic_state'):
        t=stamp(m.header.stamp)
        if armed is None or end is None or not armed <= t <= end: continue
        control_phase=next((w.phase for w in windows if w.start_ns<=t<w.end_ns),'uncovered')
        if control_phase not in ('baseline','approach','hold','recovery'):
            excluded_pose_phases[control_phase]+=1
            continue
        p=m.pose.pose; q=p.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        try:
            projection=project_course(np.asarray(reference['baseline_xy_m']),[p.position.x,p.position.y],yaw)
            pose_rows.append(dict(stamp_ns=t,**projection))
        except ValueError as exc:
            rejections[str(exc)]+=1
    by_phase={phase:[] for phase in ('baseline','approach','hold','recovery','after_recovery')}
    recover_end=reference['intervals'][-1]['end_s_m']
    for row in pose_rows:
        phase=phase_at_s(row['s_m'],reference['intervals'])
        by_phase[phase].append(row['offset_m'])
        if recover_end <= row['s_m'] < recover_end+5.:
            by_phase['after_recovery'].append(row['offset_m'])
    metrics={phase:dict(pose_count=len(values),median_signed_offset_m=float(np.median(values)),
                        max_absolute_offset_m=float(np.max(np.abs(values)))) if values else dict(pose_count=0)
             for phase,values in by_phase.items()}
    speeds=[]; speed_stamps=[]
    for _,m in messages('/vehicle/status/velocity_status'):
        t=stamp(m.header.stamp)
        if armed is not None and end is not None and armed<=t<=end:
            speed_stamps.append(t);speeds.append(float(m.longitudinal_velocity))
    moving=np.asarray([v for v in speeds if v>.1])
    candidates=Counter()
    for t in camera_ns:
        mask=recovery_teacher_mask(t,windows)
        if mask.all():
            phase=next((w.phase for w in windows if w.start_ns<=t<w.end_ns),'invalid')
            candidates[phase]+=1
    hold=metrics['hold'].get('median_signed_offset_m'); after=metrics['after_recovery'].get('median_signed_offset_m')
    # This report deliberately does not claim that phase mask alone validates
    # sensor synchronization or all 30 future measured poses for training.
    report=dict(run_id=run.name,status=result['status'],verified_files=len(manifest),sqlite_quick_check='ok',
        source_sha=result['source_sha'],reference_sha256=result['reference_sha256'],counts=counts,sensors=sensor_meta,
        stop_confirmed=final.get('stop_confirmed',False),fault=final.get('fault'),lap_confirmed=result.get('lap_confirmed',False),
        lap_records=result.get('judge_laps',[]),phase_metrics=metrics,pose_projection_rejections=dict(rejections),
        pose_metrics_scope='ACTIVE_TRACKING_ONLY_EXCLUDES_BRAKING_INVALID_UNCOVERED',
        excluded_pose_control_phases=dict(excluded_pose_phases),
        fully_traversed_recovery_intervals=sum(a.phase=='recovery' and b.phase=='baseline'
            and a.end_ns==b.start_ns for a,b in zip(windows,windows[1:])),
        moving_speed_median_kmh=float(np.median(moving)*3.6) if len(moving) else None,
        max_measured_speed_kmh=float(max(speeds)*3.6) if speeds else None,
        tracking_target_kmh=sorted({round(r['target_speed_mps']*3.6,6) for r in track}),
        phase_only_three_second_camera_candidates=dict(candidates),
        absolute_offset_reduction_m=abs(hold)-abs(after) if hold is not None and after is not None else None,
        recovery_interval_reached=bool(by_phase['recovery']),training_materialized=False,
        full_body_collision_free_verified=False,
        note='Phase candidates still require time-corpus sensor/history/observed-future validation; no training performed.')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False))
    print(json.dumps(report,indent=2,allow_nan=False))


if __name__=='__main__':
    main()
