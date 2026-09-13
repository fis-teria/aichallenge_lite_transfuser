"""Own exactly the recovery PP, generator, collector and rosbag child processes."""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

TOPICS = ['/clock', '/sensing/camera/image_raw', '/sensing/camera/camera_info',
    '/sensing/lidar/scan', '/sensing/gnss/nav_sat_fix', '/sensing/imu/imu_raw',
    '/vehicle/status/velocity_status', '/vehicle/status/steering_status',
    '/localization/kinematic_state', '/localization/pose', '/control/command/control_cmd',
    '/awsim/state', '/awsim/status', '/tf', '/tf_static', '/recovery_teacher/nominal_control_cmd',
    '/recovery_teacher/raw_control_cmd', '/recovery_teacher/trajectory', '/recovery_teacher/phase',
    '/recovery_teacher/baseline_path', '/recovery_teacher/reference_path', '/recovery_teacher/observed_path']


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--reference-root', type=Path, required=True)
    ap.add_argument('--side', choices=['left', 'right'], required=True)
    ap.add_argument('--run-id', required=True)
    args = ap.parse_args()
    commands = {
        'generator': ['ros2', 'run', 'simple_trajectory_generator', 'simple_trajectory_generator_node', '--ros-args',
            '-r', '__node:=recovery_teacher_trajectory', '-r', 'trajectory:=/recovery_teacher/trajectory',
            '-p', 'use_sim_time:=true', '-p', 'z:=0.0', '-p', 'csv_path:='+str(args.reference_root/(args.side+'.csv'))],
        'pure_pursuit': ['ros2', 'launch', '/capture/inputs/pure_pursuit.launch.xml',
            'node_name:=recovery_teacher_pure_pursuit', 'use_sim_time:=true',
            'input_kinematics:=/localization/kinematic_state', 'input_trajectory:=/recovery_teacher/trajectory',
            'output_control_cmd:=/recovery_teacher/nominal_control_cmd',
            'output_raw_control_cmd:=/recovery_teacher/raw_control_cmd',
            'use_external_target_vel:=true', 'external_target_vel:=1.3888888888888888',
            'use_overtake_reference_override:=false'],
        # Preserve independent wall-clock bag receipt and original sim capture.
        # --use-sim-time collapses multiple receipts onto the same /clock value,
        # which cannot replay the existing causal availability/epoch contract.
        'bag': ['ros2', 'bag', 'record', '--storage', 'sqlite3', '-o', str(args.output/'bag'), *TOPICS],
        'collector': ['python3', str(Path(__file__).with_name('time_recovery_collector_node.py')),
            '--output', str(args.output), '--reference', str(args.reference_root/(args.side+'.json')), '--run-id', args.run_id],
    }
    (args.output/'node_commands.json').write_text(json.dumps(commands, indent=2))
    # The monitor uses tiny NumPy matrices, not a training workload. Avoid
    # creating 20 BLAS workers alongside AWSIM, RViz and ROS callbacks.
    collector_env = dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1')
    (args.output/'collector_environment.json').write_text(json.dumps(
        {key: collector_env[key] for key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS')}, indent=2))
    streams = {}; children = {}; result = {'closed_bag': False, 'error': None}

    def interrupted(signum, frame):
        raise RuntimeError('NODE_WRAPPER_INTERRUPTED')
    signal.signal(signal.SIGTERM, interrupted)
    try:
        for name, cmd in commands.items():
            streams[name] = (args.output/(name+'.log')).open('x')
            children[name] = subprocess.Popen(cmd, stdout=streams[name], stderr=subprocess.STDOUT,
                start_new_session=True, env=collector_env if name == 'collector' else None)
        started = time.monotonic()
        while time.monotonic()-started < 1970:
            if (args.output/'finish_nodes.json').exists():
                break
            for name, child in children.items():
                if child.poll() is not None:
                    raise RuntimeError(name+'_EARLY_EXIT_'+str(child.returncode))
            p = args.output/'nodes_heartbeat.pending'
            p.write_text(json.dumps({'monotonic_ns': time.monotonic_ns(), 'pids': {k: p.pid for k,p in children.items()}}))
            p.replace(args.output/'nodes_heartbeat.json')
            time.sleep(.1)
        else:
            raise RuntimeError('NODE_WRAPPER_WALL_LIMIT')
    except Exception as exc:
        result['error'] = str(exc)
    finally:
        # Host freezes/stops its simulator before requesting child shutdown.
        # SQLite must receive SIGINT and finish before any offline reader opens it.
        for name in ('bag', 'collector', 'pure_pursuit', 'generator'):
            child = children.get(name)
            if child is None:
                continue
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
                try:
                    child.wait(timeout=25 if name == 'bag' else 5)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGTERM)
                    try:
                        child.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid, signal.SIGKILL); child.wait(timeout=3)
            result[name+'_exit'] = child.returncode
        result['closed_bag'] = (args.output/'bag/metadata.yaml').is_file() and result.get('bag_exit') in (0, -2)
        for stream in streams.values():
            stream.close()
        (args.output/'nodes_result.json').write_text(json.dumps(result, indent=2))
    if result['error'] or not result['closed_bag']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
