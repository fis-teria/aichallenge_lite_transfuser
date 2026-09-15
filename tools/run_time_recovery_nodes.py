"""Own exactly the recovery PP, generator, collector and rosbag child processes."""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from aic_transfuser_lite.data.time_recovery_collection_v1 import (
    COLLECTION_SPEED_POLICIES, collection_speed_gain,
)
from aic_transfuser_lite.runtime.recovery_disturbance_markers import MARKER_TOPIC
from aic_transfuser_lite.runtime.recovery_parallel_v1 import validate_domain
from aic_transfuser_lite.data.time_large_recovery_reference_v1 import validate_large_reference

TOPICS = ['/clock', '/sensing/camera/image_raw', '/sensing/camera/camera_info',
    '/sensing/lidar/scan', '/sensing/gnss/nav_sat_fix', '/sensing/imu/imu_raw',
    '/vehicle/status/velocity_status', '/vehicle/status/steering_status',
    '/localization/kinematic_state', '/localization/pose', '/control/command/control_cmd',
    '/awsim/state', '/awsim/status', '/tf', '/tf_static', '/recovery_teacher/nominal_control_cmd',
    '/recovery_teacher/raw_control_cmd', '/recovery_teacher/trajectory', '/recovery_teacher/phase',
    '/recovery_teacher/baseline_path', '/recovery_teacher/reference_path', '/recovery_teacher/observed_path', MARKER_TOPIC]
PREPARATION_TOPICS = ['/recovery_teacher/preparation_trajectory', '/recovery_teacher/preparation_control_cmd',
    '/recovery_teacher/preparation_raw_control_cmd', '/recovery_teacher/preparation_path']


def preparation_commands(reference_root: Path, side: str, commands: dict[str, list[str]]) -> dict[str, list[str]]:
    """Start a second official PP on a static, hash-verified preparation path."""
    reference = json.loads((reference_root/(side+'.json')).read_text())
    if reference.get('large_recovery') is None:
        return {}
    validate_large_reference(reference, reference_root)
    replacements = {
        '__node:=recovery_teacher_trajectory': '__node:=recovery_preparation_trajectory',
        'trajectory:=/recovery_teacher/trajectory': 'trajectory:=/recovery_teacher/preparation_trajectory',
        'csv_path:='+str(reference_root/(side+'.csv')):
            'csv_path:='+str(reference_root/reference['large_recovery']['preparation_csv']),
        'node_name:=recovery_teacher_pure_pursuit': 'node_name:=recovery_preparation_pure_pursuit',
        'input_trajectory:=/recovery_teacher/trajectory': 'input_trajectory:=/recovery_teacher/preparation_trajectory',
        'output_control_cmd:=/recovery_teacher/nominal_control_cmd': 'output_control_cmd:=/recovery_teacher/preparation_control_cmd',
        'output_raw_control_cmd:=/recovery_teacher/raw_control_cmd': 'output_raw_control_cmd:=/recovery_teacher/preparation_raw_control_cmd',
    }
    result = {'preparation_'+name: [replacements.get(part, part) for part in commands[name]]
              for name in ('generator', 'pure_pursuit')}
    # Keep the second PP's diagnostics/envelopes separate as well as its command.
    for name in ('recovery_control_cmd', 'debug', 'controller_tracking_status', 'controller_command_envelope',
                 'controller_execution_envelope', 'free_run_execution_ack', 'free_run_source_key', 'lookahead_point'):
        result['preparation_pure_pursuit'].append('output_'+name+':=/recovery_teacher/preparation/'+name)
    return result


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--reference-root', type=Path, required=True)
    ap.add_argument('--side', choices=['left', 'right'], required=True)
    ap.add_argument('--run-id', required=True)
    ap.add_argument('--ros-domain-id', type=int, default=1)
    ap.add_argument('--speed-policy', choices=COLLECTION_SPEED_POLICIES, default='legacy_gain1_v1')
    args = ap.parse_args()
    validate_domain(args.ros_domain_id, dict(os.environ))
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
            'speed_proportional_gain:='+str(collection_speed_gain(args.speed_policy)),
            'use_overtake_reference_override:=false'],
        # Preserve independent wall-clock bag receipt and original sim capture.
        # --use-sim-time collapses multiple receipts onto the same /clock value,
        # which cannot replay the existing causal availability/epoch contract.
        'bag': ['ros2', 'bag', 'record', '--storage', 'sqlite3', '-o', str(args.output/'bag'), *TOPICS],
        'collector': ['python3', str(Path(__file__).with_name('time_recovery_collector_node.py')),
            '--output', str(args.output), '--reference', str(args.reference_root/(args.side+'.json')), '--run-id', args.run_id,
            '--ros-domain-id', str(args.ros_domain_id)],
        'paths': ['python3', str(Path(__file__).with_name('time_recovery_paths_node.py')),
            '--output', str(args.output), '--reference', str(args.reference_root/(args.side+'.json'))],
    }
    preparation = preparation_commands(args.reference_root, args.side, commands)
    if preparation:
        commands.update(preparation)
        commands['bag'].extend(PREPARATION_TOPICS)
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
        for name in ('bag', 'collector', 'preparation_pure_pursuit', 'pure_pursuit', 'preparation_generator', 'generator', 'paths'):
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
