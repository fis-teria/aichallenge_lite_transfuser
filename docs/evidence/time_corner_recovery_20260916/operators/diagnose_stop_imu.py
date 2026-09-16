"""Locate non-finite IMU records; never re-admit an excluded collection run."""
from pathlib import Path
import argparse
import json
import math
import sqlite3
from rosbags.typesys import Stores, get_typestore

OUT = Path('/home/thistle/e2e_autonomous/runs/time_corner_recovery_20260916')
RAW = Path('/home/thistle/e2e_autonomous/raw/time_corner_recovery_20260916')
store = get_typestore(Stores.ROS2_HUMBLE)
parser = argparse.ArgumentParser()
parser.add_argument('--output', type=Path, default=OUT / 'stopped_imu_diagnosis.json')
args = parser.parse_args()
reports = []
for summary in sorted(OUT.glob('*_collection_summary.json')):
    report = json.loads(summary.read_bytes())
    if report['fault'] != 'IMU_RATE_CONTRACT':
        continue
    assert report['accepted'] == 0
    raw = RAW / report['run_id']
    result = json.loads((raw / 'result.json').read_bytes())
    controls = [json.loads(line) for line in (raw / 'control.jsonl').read_text().splitlines()]
    first_control = next(r for r in controls if r.get('reason') == 'IMU_RATE_CONTRACT')
    invalid = []
    with sqlite3.connect((raw / 'bag/bag_0.db3').as_uri() + '?mode=ro', uri=True) as connection:
        rows = connection.execute('select m.id,m.data from messages m join topics t on m.topic_id=t.id where t.type=? order by m.id', ('sensor_msgs/msg/Imu',))
        for sequence, data in rows:
            message = store.deserialize_cdr(data, 'sensor_msgs/msg/Imu')
            values = [message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z]
            if message.header.frame_id != 'imu_link' or not all(math.isfinite(v) for v in values):
                invalid.append(dict(row_id=sequence, capture_ns=message.header.stamp.sec * 10**9 + message.header.stamp.nanosec,
                    frame=message.header.frame_id, nonfinite_components=[axis for axis, value in zip('xyz', values) if not math.isfinite(value)]))
    assert invalid
    stop_since = result['last_control']['stop_since_ns']
    event_end = max(e['end_publication_ns'] for e in report['events'] if e['completed'])
    reports.append(dict(run_id=report['run_id'], accepted=0, policy='EXCLUDED_UNCHANGED',
        lap_confirmed=result['lap_confirmed'], stop_confirmed=result['last_control']['stop_confirmed'],
        first_fault_control_sim_ns=first_control['sim_ns'], first_fault_speed_mps=first_control['speed_mps'],
        first_invalid_imu=invalid[0], last_invalid_imu=invalid[-1], invalid_imu_messages=len(invalid),
        first_invalid_after_stop_since_s=(invalid[0]['capture_ns'] - stop_since) / 1e9,
        separation_from_last_recovery_plus_3s_s=(invalid[0]['capture_ns'] - event_end) / 1e9 - 3.))
with args.output.open('x') as stream:
    json.dump(reports, stream, indent=2, allow_nan=False)
print(json.dumps(reports), flush=True)
