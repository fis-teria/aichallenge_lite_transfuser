"""Causal replay of scan + wheel reports + clock; no GNSS/IMU/pose/TF reads."""
from __future__ import annotations

import argparse
from collections import Counter, deque
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from rosbags.highlevel import AnyReader

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from aic_transfuser_lite.control.time_trial_v1 import interpolate_body_pose
from aic_transfuser_lite.control.vehicle_motion_v1 import AWSIM_20KMH_POLICY
from aic_transfuser_lite.data.time_sqlite_reader_v1 import _store
from aic_transfuser_lite.runtime.time_control_odometry import TimeControlOdometry, ClockAlignedControlInputs
from aic_transfuser_lite.runtime.lidar_map_localization import BoundaryMap, MapLocalizer, compose, inverse, scan_points


def ns(stamp) -> int:
    return int(stamp.sec)*10**9+int(stamp.nanosec)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run',type=Path,required=True,help='Existing run with bag/ and types/')
    ap.add_argument('--map',type=Path,required=True)
    ap.add_argument('--initial-pose',type=float,nargs=3,required=True,help='Explicit approximate map start: x m, y m, yaw rad')
    ap.add_argument('--output',type=Path,required=True)
    args = ap.parse_args(); args.output.mkdir(exist_ok=False,parents=True)
    course = BoundaryMap.from_lanelet(args.map); tracker = MapLocalizer(course)
    tracker.initialize(np.array(args.initial_pose))
    odom = TimeControlOdometry(.0010000169277191162,AWSIM_20KMH_POLICY)
    inputs = ClockAlignedControlInputs(); poses = deque(maxlen=512); scans = deque(maxlen=4)
    clock = None; last_scan = -1; counts = Counter(); records = []; fixed_map_odom = None
    topics = {'/clock','/vehicle/status/velocity_status','/vehicle/status/steering_status','/sensing/lidar/scan'}
    digest = hashlib.sha256(); fault = None
    with AnyReader([args.run/'bag'],default_typestore=_store(args.run)) as reader:
        for connection, receipt, raw in reader.messages(connections=[c for c in reader.connections if c.topic in topics]):
            digest.update(connection.topic.encode());digest.update(receipt.to_bytes(8,'little'));digest.update(raw)
            msg = reader.deserialize(raw,connection.msgtype); counts[connection.topic] += 1
            try:
                if connection.topic == '/clock':
                    current = ns(msg.clock)
                    if clock is not None and current < clock: raise ValueError('CLOCK_RESET')
                    clock = current
                elif connection.topic == '/vehicle/status/velocity_status':
                    inputs.add('velocity',ns(msg.header.stamp),float(msg.longitudinal_velocity),receipt)
                elif connection.topic == '/vehicle/status/steering_status':
                    inputs.add('steering',ns(msg.stamp),float(msg.steering_tire_angle),receipt)
                else:
                    if msg.header.frame_id != 'lidar': raise ValueError('SCAN_FRAME')
                    scans.append((msg,receipt))
                if clock is None: continue
                for role, stamp, value in inputs.ready(clock,receipt):
                    if role == 'velocity': odom.add_speed(stamp,value)
                    else: odom.add_steering(stamp,value)
                for pose, trace in odom.drain(clock,'0'): poses.append(pose)
                while scans and poses:
                    scan, arrived = scans[0]; stamp = ns(scan.header.stamp)
                    if stamp > clock or stamp > poses[-1].stamp_ns:
                        break
                    scans.popleft()
                    if stamp-last_scan < 190_000_000: continue
                    if clock-stamp > 350_000_000 or receipt-arrived > 350_000_000:
                        counts['stale_scans'] += 1; continue
                    try:
                        body = interpolate_body_pose(poses,stamp)
                    except ValueError:
                        counts['unbracketed_scans'] += 1; continue
                    wheel = np.array([body.x_m,body.y_m,body.yaw_rad])
                    if fixed_map_odom is None:
                        fixed_map_odom = compose(np.array(args.initial_pose),inverse(wheel))
                    cloud = scan_points(scan.ranges,scan.angle_min,scan.angle_increment,scan.range_min,scan.range_max)
                    result = tracker.update(stamp,wheel,cloud);last_scan = stamp
                    counts[tracker.status] += 1
                    row = dict(stamp_ns=stamp,status=tracker.status,valid=tracker.valid(clock),
                               wheel_pose=wheel.tolist(),uncorrected_map_pose=compose(fixed_map_odom,wheel).tolist())
                    if result is not None:
                        row['match'] = {k: (v.tolist() if isinstance(v,np.ndarray) else
                                            None if isinstance(v,float) and not np.isfinite(v) else v)
                                        for k,v in asdict(result).items()}
                    records.append(row)
            except ValueError as exc:
                fault = str(exc); break
    with (args.output/'records.jsonl').open('x') as stream:
        for row in records: stream.write(json.dumps(row,allow_nan=False)+'\n')
    valid = [r for r in records if r['valid']]
    complete = len(valid)/max(1,len(records)) >= .95 and tracker.valid(clock or 0) and fault is None
    summary = dict(status='TRACKED_REPLAY' if complete else 'PARTIAL_OR_FAILED',fault=fault,
        input_topics=sorted(topics),gnss_imu_pose_tf_inputs=False,selected_message_sha256=digest.hexdigest(),
        map_sha256=hashlib.sha256(args.map.read_bytes()).hexdigest(),initial_pose=args.initial_pose,
        initialization_source='Explicit fixed approximate start supplied on command line; not read from bag pose',
        counts=dict(counts),scans=len(records),valid_scans=len(valid),
        valid_fraction=len(valid)/max(1,len(records)),
        valid_mean_boundary_distance_m=float(np.mean([r['match']['mean_distance_m'] for r in valid])) if valid else None,
        scope='Offline localization only; lane boundaries are not physical-wall truth; no motion authority')
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    if valid:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,ax = plt.subplots(figsize=(10,9))
        for segment in course.segments: ax.plot(segment[:,0],segment[:,1],c='black',linewidth=.8)
        before=np.array([r['uncorrected_map_pose'][:2] for r in records])-course.origin
        after=np.array([r['match']['pose'][:2] for r in valid])-course.origin
        ax.plot(before[:,0],before[:,1],label='Wheel odometry with fixed initial map pose',alpha=.7)
        ax.plot(after[:,0],after[:,1],label='Wheel + LiDAR map registration')
        ax.set_aspect('equal');ax.set_xlabel('Map-local x [m]');ax.set_ylabel('Map-local y [m]')
        ax.set_title('No GNSS / IMU / recorded pose inputs; offline replay');ax.legend()
        fig.tight_layout();fig.savefig(args.output/'trajectory.png',dpi=140)
    print(json.dumps(summary,indent=2))


if __name__ == '__main__': main()
