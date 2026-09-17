"""Own Cartographer and the diagnostic obstacle node; no vehicle commands."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import signal
import subprocess
import time


def main() -> None:
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(); args.output.mkdir(exist_ok=True, parents=True)
    config = Path(get_package_share_directory('aic_e2e_runtime'))/'config'
    binary = Path(get_package_prefix('cartographer_ros'))/'lib/cartographer_ros/cartographer_node'
    record = dict(cartographer_binary=str(binary),
                  binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
                  config_sha256=hashlib.sha256((config/'time_slam_2d.lua').read_bytes()).hexdigest())
    (args.output/'slam_runtime_identity.json').write_text(json.dumps(record, indent=2))
    commands = [
        [str(binary), '-configuration_directory', str(config), '-configuration_basename', 'time_slam_2d.lua',
         '--ros-args', '-r', '__node:=cartographer', '-r', '__ns:=/time_slam', '-p', 'use_sim_time:=true',
         '-r', 'scan:=/time_path/slam/input_scan', '-r', 'odom:=/time_path/slam/input_odom',
         '-r', '/tf:=/time_path/slam/input_tf', '-r', '/tf_static:=/time_path/slam/input_tf_static'],
        ['ros2', 'run', 'aic_e2e_runtime', 'slam_obstacle_node', '--output', str(args.output)],
    ]
    processes = []; logs = []
    def stop(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    try:
        for command, name in zip(commands, ('cartographer', 'slam_obstacle_node')):
            log = (args.output/(name+'.log')).open('x'); logs.append(log)
            processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT))
        while True:
            for process, command in zip(processes, commands):
                if process.poll() is not None:
                    raise RuntimeError('SLAM_CHILD_EXIT:'+str(process.returncode)+':'+command[0])
            time.sleep(.1)
    except KeyboardInterrupt:
        pass
    finally:
        for process in processes:
            if process.poll() is None: process.send_signal(signal.SIGINT)
        for process in processes:
            try: process.wait(timeout=1)
            except subprocess.TimeoutExpired: process.kill(); process.wait()
        for log in logs: log.close()


if __name__ == '__main__':
    main()
