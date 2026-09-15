"""Audit one transferred pair and materialize only observed successful recoveries."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import sqlite3

import numpy as np
import yaml

from aic_transfuser_lite.data.time_random_steering_pulse_v1 import random_pulse_events
from aic_transfuser_lite.data.time_recovery_training_v1 import materialize_recovery_run
from aic_transfuser_lite.data.time_training_cache_v1 import _prepare_run
from aic_transfuser_lite.data.time_dataset_v1 import TimeDatasetConfig
from aic_transfuser_lite.runtime.recovery_disturbance_markers import DisturbanceLocations, MARKER_TOPIC
from rosbags.typesys import Stores, get_typestore

OUT = Path('/home/thistle/e2e_autonomous/runs/time_recovery_sites_20260915')
RAW = Path('/home/thistle/e2e_autonomous/raw/time_recovery_sites_20260915')
TYPES = Path('/home/thistle/e2e_autonomous/runs/time_recovery_collection_20260913/types')


def read(path):
    return json.loads(path.read_bytes())


def write(path, data):
    with path.open('x') as stream:
        json.dump(data, stream, indent=2, allow_nan=False)


def stage(name, kind, command):
    target = OUT/(name+'_'+kind+'.json')
    if target.exists():
        raise FileExistsError(target)
    with target.with_suffix('.log').open('x') as stream:
        result = subprocess.run([sys.executable, *command, '--output', str(target)],
            stdout=stream, stderr=subprocess.STDOUT, timeout=900)
    print(json.dumps(dict(stage=kind, run_id=name, exit_code=result.returncode)), flush=True)
    if result.returncode:
        raise RuntimeError(target.with_suffix('.log').read_text()[-5000:])
    return target


def audit_markers(raw, control, events):
    trace = DisturbanceLocations()
    for sample in control:
        trace.add(sample)
    expected = trace.report()
    stored = read(raw/'disturbance_markers.json') if trace.events else expected
    assert stored == expected, 'MARKER_FIRST_PUBLISHED_POSE_MISMATCH'
    assert [e['site_id'] for e in expected['events']] == [e['site_id'] for e in events]
    heartbeat = read(raw/'path_heartbeat.json')
    assert heartbeat['disturbance_sites'] == [e['label'] for e in expected['events']]
    assert heartbeat['disturbance_marker_count'] == 3*len(events)
    # RViz is closed before the paths process during orderly shutdown.
    # Its absence from the final heartbeat is expected, not a display failure.
    displays=yaml.safe_load((raw/'autoware.rviz').read_text())['Visualization Manager']['Displays']
    configured=[d for d in displays if d.get('Class')=='rviz_default_plugins/MarkerArray'
                and d.get('Topic',{}).get('Value')==MARKER_TOPIC and d.get('Enabled') is True]
    assert len(configured)==1
    db, = (raw/'bag').glob('*.db3')
    conn=sqlite3.connect('file:'+str(db)+'?mode=ro&immutable=1',uri=True)
    topic_id, kind = conn.execute('SELECT id,type FROM topics WHERE name=?',(MARKER_TOPIC,)).fetchone()
    blob, = conn.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp DESC,id DESC LIMIT 1',(topic_id,)).fetchone()
    message=get_typestore(Stores.ROS2_HUMBLE).deserialize_cdr(bytes(blob),kind);conn.close()
    assert len(message.markers)==3*len(events)
    texts={m.text:(m.pose.position.x,m.pose.position.y) for m in message.markers if m.type==9}
    assert texts=={e['label']:(e['x_m'],e['y_m']) for e in expected['events']}
    assert all(m.header.frame_id=='map' and m.lifetime.sec==m.lifetime.nanosec==0 for m in message.markers)
    return dict(first_published_location_matches_control_trace=True,ordinary_rviz_display_configured=True,
                subscribers_at_shutdown=heartbeat['marker_subscribers'],
                recorded_marker_array_matches=True,locations=expected['events'])


ap = argparse.ArgumentParser(); ap.add_argument('--pair', type=int, required=True); args = ap.parse_args()
receipt = read(OUT/f'sites_pair{args.pair:02d}_20260915_verified.json')
assert receipt['all_files_and_directory_structure_identical'] and receipt['all_sqlite_quick_checks_passed']
names = [r['run_id'] for r in receipt['runs']]
if args.pair:
    plan_path = OUT/'selected_site_plan.json'; plan = read(plan_path)
    assigned = [r for r in plan['runs'] if r['pair'] == args.pair]
    assert names == [r['run_id'] for r in assigned]
    sys.path.insert(0, str(Path('docs/evidence/time_recovery_expansion_20260914').resolve()))
    from summarize_expansion import alternate_state, goal, bins
    alternate_path = OUT/(plan['normal_runs'][1]+'_full_guide.json')
    alternate_guide = np.asarray(read(alternate_path)['guide'])
else:
    assigned = [dict(run_id=name, split='reference_only') for name in names]
reports = []
for row in assigned:
    name = row['run_id']; raw = RAW/name; result = read(raw/'result.json')
    bag_path = stage(name, 'bag_audit', ['tools/audit_time_recovery_collection.py', '--run', str(raw), '--types', str(TYPES)])
    bag = read(bag_path)
    assert bag['sqlite_quick_check'] == 'ok'
    assert all(v['invalid_messages'] == 0 and v['backward_headers'] == 0 for v in bag['sensors'].values())
    summary = dict(run_id=name, split=row['split'], status=result['status'], bag_verified=True,
                   normal_stop_confirmed=result.get('last_control',{}).get('stop_confirmed'),
                   fault=result.get('last_control',{}).get('fault'), accepted=0)
    if not args.pair:
        reports.append(summary); continue
    control = [json.loads(line) for line in (raw/'control.jsonl').read_text().splitlines()]
    events = random_pulse_events(control)
    summary['marker_audit'] = audit_markers(raw, control, events)
    final = result['last_control']['random_pulse']['state']
    emitted_sites = {e['site_id'] for e in events}
    summary.update(events=events, planned_sites=row['sites'], skipped_sites=[s['site_id'] for s in row['sites'] if s['site_id'] not in emitted_sites],
                   final_pulse_state=final, success_events=sum(bool(e['recovery_confirmed']) for e in events))
    if any(e['recovery_confirmed'] for e in events):
        probe_path = stage(name, 'causal_probe', ['docs/evidence/time_recovery_collection_20260913/causal_replay_probe.py',
            '--run', str(raw), '--types', str(TYPES), '--all-candidates'])
        states_path = stage(name, 'state_audit', ['docs/evidence/time_steering_pulse_20260914/analyze_pulse.py',
            '--run', str(raw), '--probe', str(probe_path)])
        states = read(states_path)
        guide = np.asarray(read(raw/'reference.json')['steering_pulse']['nominal_guide'])
        for event in events:
            accepted = [r for r in states['records'] if r['accepted'] and r.get('recovery_event_id') == event['event_id']]
            alternate = [alternate_state(r, guide, alternate_guide) for r in accepted]
            outward = [a['anchor_id'] for a,b in zip(accepted, alternate) if goal(a) and goal(b)]
            event.update(accepted=bins(accepted), alternate=bins(alternate), outward_anchor_ids=outward,
                         stop_region_anchors=sum(119. <= a.get('base_s_m', -1.) <= 120. for a in accepted))
        summary['accepted'] = sum(e['accepted']['count'] for e in events)
        if summary['accepted']:
            destination = OUT/'materialized'/name
            material = materialize_recovery_run(raw, destination, split=row['split'], types=TYPES, previous_probe=probe_path)
            cache = OUT/'prepared'/row['split']/name
            prepared = _prepare_run(destination, cache, name, TimeDatasetConfig())
            assert prepared['input_valid'] == prepared['anchors'] == material['accepted'] == summary['accepted']
            assert not prepared['input_reason_refinements']
            labels = np.load(destination/'teachers.npz')
            assert labels['xy_m'].shape == (summary['accepted'], 30, 2) and np.isfinite(labels['xy_m']).all() and labels['xy_mask'].all()
            anchors = [json.loads(s) for s in (destination/'anchors.jsonl').read_text().splitlines()]
            counts = Counter(a['recovery_event_id'] for a in anchors)
            assert all(counts[e['event_id']] == e['accepted']['count'] for e in events)
            summary['prepared'] = prepared
    summary['all_sites_have_at_least_60_anchors'] = (len(events) == len(row['sites'])
        and all(e.get('accepted',{}).get('count',0) >= 60 and e['recovery_confirmed'] for e in events))
    write(OUT/(name+'_collection_summary.json'), summary)
    reports.append(summary)
    print(json.dumps(dict(run_id=name, emitted=len(events), success=summary['success_events'], skipped=summary['skipped_sites'],
                          accepted=summary['accepted'], sixty_per_site=summary['all_sites_have_at_least_60_anchors'])), flush=True)
final = dict(scope='ACTUAL_OBSERVATIONS_AND_RECOVERY_TEACHERS_NOT_MODEL_PERFORMANCE', pair=args.pair,
             runs=reports, total_accepted=sum(r['accepted'] for r in reports),
             events=sum(len(r.get('events',[])) for r in reports),
             successful_events=sum(r.get('success_events',0) for r in reports),
             plan_sha256=hashlib.sha256(plan_path.read_bytes()).hexdigest() if args.pair else None,
             training_started=False, sealed_test_read=False)
write(OUT/f'pair{args.pair:02d}_audit.json', final)
print(json.dumps({k:v for k,v in final.items() if k!='runs'}), flush=True)
