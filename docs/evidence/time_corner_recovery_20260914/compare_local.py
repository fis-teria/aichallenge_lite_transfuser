"""Compare closed corner pilots to the predeclared same-host nominal passages.

Native WSL, after transfer audit. Map/base_link positions in m, headings in rad.
The output is collection evidence, not a trained-model or collision evaluation.
"""
from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sqlite3

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from rosbags.typesys import Stores, get_typestore

from aic_transfuser_lite.control.time_reference_v1 import TimedBodyPose
from aic_transfuser_lite.evaluation.time_corner_comparison_v1 import RecordedLine, angle_delta


def read(path: Path) -> dict:
    return json.loads(path.read_text())


def rows(run: Path) -> list[dict]:
    assert read(run/'result.json')['nodes']['closed_bag']
    return [r for line in (run/'control.jsonl').read_text().splitlines()
            if (r := json.loads(line)).get('reason') == 'RECOVERY_TEACHER_TRACKING'
            and r.get('projection') and r.get('current_pose')
            and 70. <= r['projection']['s_m'] <= 112.]


def nominal(run: Path) -> tuple[list[dict], RecordedLine]:
    trace = rows(run)
    assert trace and all(r['phase'] == 'baseline' for r in trace)
    start = min(r['current_pose']['stamp_ns'] for r in trace)-100_000_000
    end = max(r['current_pose']['stamp_ns'] for r in trace)+100_000_000
    store = get_typestore(Stores.ROS2_HUMBLE)
    dbs = list((run/'bag').glob('*.db3'))
    assert len(dbs) == 1
    poses = []
    with sqlite3.connect(dbs[0].as_uri()+'?mode=ro&immutable=1', uri=True) as conn:
        topic_id, kind = conn.execute('SELECT id,type FROM topics WHERE name=?',
                                     ('/localization/kinematic_state',)).fetchone()
        assert kind == 'nav_msgs/msg/Odometry'
        for (blob,) in conn.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY id', (topic_id,)):
            msg = store.deserialize_cdr(blob, kind)
            t = int(msg.header.stamp.sec)*10**9+int(msg.header.stamp.nanosec)
            if not start <= t <= end:
                continue
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            poses.append(TimedBodyPose(t, 'sim', '0', msg.header.frame_id, msg.child_frame_id, p.x, p.y, yaw))
    return trace, RecordedLine(poses)


def stats(values: list[float]) -> dict:
    a = np.asarray(values, dtype=float)
    return dict(n=len(a), median=float(np.median(a)), maximum_abs=float(np.max(np.abs(a))),
                minimum=float(a.min()), maximum=float(a.max())) if len(a) else dict(n=0)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--runs', nargs='+', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    assert not args.output.exists()
    primary = 'codex-time-recovery-left020-r23'
    names = (primary, 'codex-time-recovery-right040-r22')
    references = {n: nominal(args.root/'raw/time_recovery_batches_20260914'/n) for n in names}
    report = dict(scope='MEASURED_COLLECTION_LOCAL_RECOVERY_NOT_MODEL_EVALUATION', primary=primary,
                  prior_nominal={}, runs=[], model_trained=False, split_assigned=False)
    for name, (_, line) in references.items():
        report['prior_nominal'][name] = line.audit
    fig, axes = plt.subplots(2, len(args.runs), figsize=(6*len(args.runs), 8), squeeze=False)
    for column, name in enumerate(args.runs):
        run = args.root/'raw/time_corner_recovery_20260914'/name
        trace = rows(run)
        ref = read(run/'reference.json')
        result = read(run/'result.json')
        start, end = ref['intervals'][0]['start_s_m'], ref['intervals'][-1]['end_s_m']
        trace = [r for r in trace if start-2 <= r['projection']['s_m'] < end+5]
        assert trace
        matched = []
        rejected = Counter()
        for row in trace:
            p = TimedBodyPose(**row['current_pose'])
            phase = 'after' if row['projection']['s_m'] >= end else row['phase']
            record = dict(stamp_ns=p.stamp_ns, s_m=row['projection']['s_m'], phase=phase,
                          csv_left_m=row['projection']['offset_m'], references={})
            for nominal_name, (base_trace, line) in references.items():
                try:
                    projected = line.project(np.array([p.x_m, p.y_m]), yaw_hint_rad=p.yaw_rad)
                except ValueError as exc:
                    rejected[nominal_name+':'+str(exc)] += 1
                    continue
                # Narrow, unique first passage. Interpolation only within observed s bounds.
                ordered = sorted(base_trace, key=lambda r:r['projection']['s_m'])
                s = [r['projection']['s_m'] for r in ordered]
                assert s[0] <= record['s_m'] <= s[-1]
                nominal_offset = float(np.interp(record['s_m'], s, [r['projection']['offset_m'] for r in ordered]))
                record['references'][nominal_name] = dict(
                    direct_left_m=projected.left_m, direct_distance_m=projected.distance_m,
                    same_base_s_offset_difference_m=record['csv_left_m']-nominal_offset,
                    heading_difference_rad=angle_delta(p.yaw_rad, projected.body_yaw_rad))
            matched.append(record)
        phases = {}
        for phase in ('approach', 'hold', 'recovery', 'after'):
            selected = [r for r in matched if r['phase'] == phase]
            phases[phase] = dict(csv_left_m=stats([r['csv_left_m'] for r in selected]), references={})
            for nominal_name in names:
                phases[phase]['references'][nominal_name] = {
                    key:stats([r['references'][nominal_name][key] for r in selected if nominal_name in r['references']])
                    for key in ('direct_left_m','direct_distance_m','same_base_s_offset_difference_m','heading_difference_rad')}
        gates = {}
        sign = np.sign(ref['signed_offset_m'])
        for nominal_name in names:
            h = phases['hold']['references'][nominal_name]['same_base_s_offset_difference_m']
            a = phases['after']['references'][nominal_name]['same_base_s_offset_difference_m']
            gates[nominal_name] = dict(hold_at_least_10cm=h.get('n',0)>0 and sign*h['median'] >= .1,
                after_within_10cm=a.get('n',0)>0 and a['maximum_abs'] <= .1,
                reduction_at_least_5cm=h.get('n',0)>0 and a.get('n',0)>0 and abs(h['median'])-a['maximum_abs'] >= .05)
        entry = dict(run_id=name, phase_metrics=phases, local_geometry_gates=gates,
                     all_local_geometry_gates_pass=not rejected and all(all(v.values()) for v in gates.values()),
                     projection_rejections=dict(rejected), status=result['status'],
                     normal_stop=result['last_control']['stop_confirmed'] and not result['last_control']['fault'],
                     reference_sha256=ref['reference_sha256'],
                     control_sha256=hashlib.sha256((run/'control.jsonl').read_bytes()).hexdigest())
        report['runs'].append(entry)
        with args.output.with_name(name+'_local_matches.jsonl').open('x') as stream:
            for row in matched:
                stream.write(json.dumps(row, allow_nan=False)+'\n')
        axis = axes[0,column]
        for key, label in [('same_base_s_offset_difference_m','At same base progress'),('direct_left_m','Direct nominal-line projection')]:
            plotted = [r for r in matched if primary in r['references']]
            axis.plot([r['s_m'] for r in plotted], [r['references'][primary][key] for r in plotted], label=label)
        for interval, color in zip(ref['intervals'], ('#f7d694','#dedede','#b8dbbf')):
            axis.axvspan(interval['start_s_m'],interval['end_s_m'],color=color,alpha=.4)
        axis.axhline(0,color='gray',linewidth=.7)
        axis.set(xlabel='Original course progress [m]',ylabel='Left deviation from measured nominal [m]',title=name)
        axis.grid(alpha=.3); axis.legend(fontsize=8)
        axis = axes[1,column]
        origin = np.array([89649.,43143.])
        line = references[primary][1]
        axis.plot(*(line.xy-origin).T,label='Measured nominal r23',color='gray')
        actual = np.array([[r['current_pose']['x_m'],r['current_pose']['y_m']] for r in trace])
        axis.plot(*(actual-origin).T,label='New measured pilot',color='#c65c25' if sign>0 else '#2471a3')
        axis.set_aspect('equal'); axis.grid(alpha=.3); axis.legend(fontsize=8)
        axis.set(xlabel='Map x - 89649 [m]',ylabel='Map y - 43143 [m]')
    fig.tight_layout(); fig.savefig(args.output.with_suffix('.png'),dpi=150); plt.close(fig)
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(report,indent=2,allow_nan=False))


if __name__ == '__main__':
    main()
