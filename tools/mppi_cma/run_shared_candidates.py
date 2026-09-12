"""Four separately mounted references sharing one ghost AWSIM at the D1 start."""
from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path
import subprocess
import time

if __package__:
    from .run_episode import ROOT, prepare_episode
else:
    from run_episode import ROOT, prepare_episode


REFERENCE_TARGET = ('/aichallenge/workspace/install/multi_purpose_mpc_ros/share/'
                    'multi_purpose_mpc_ros/env/final_ver3/in_corce_line_straight_smooth.csv')


def controller_commands(output: Path, references: list[Path], image: str,
                        simulator: str, target_mps: float) -> list[list[str]]:
    """Bind one reference per ROS domain; all four join only the owned simulator."""
    if len(references) != 4:
        raise ValueError('Exactly four references are required')
    commands = []
    for number, reference in enumerate(references, 1):
        commands.append([
            'docker', 'run', '--rm', '--name', f'{simulator}-d{number}',
            '--network', 'container:' + simulator, '--shm-size', '1g', '--cap-add', 'NET_ADMIN',
            '--gpus', 'all', '-e', 'NVIDIA_DRIVER_CAPABILITIES=all',
            '-e', f'ROS_DOMAIN_ID={number}', '-e', f'VEHICLE_ID=d{number}',
            '-e', 'SIM_MODE=h2h-race', '-e', 'ROS_HOME=/eval/ros-home', '-e', 'ROS_LOG_DIR=/eval/ros',
            '-e', 'RMW_IMPLEMENTATION=rmw_cyclonedds_cpp',
            '-e', 'CYCLONEDDS_URI=file:///opt/autoware/cyclonedds.xml',
            '-v', str(output / 'cyclonedds.xml') + ':/opt/autoware/cyclonedds.xml:ro',
            '-v', str(output / f'd{number}') + ':/eval', '--workdir', '/eval',
            '-v', str(reference) + ':' + REFERENCE_TARGET + ':ro',
            '-v', str(output / 'mppi.yaml') + ':/aichallenge/workspace/install/'
            'reference_space_mppi_planner/share/reference_space_mppi_planner/config/reference_space_mppi.param.yaml:ro',
            '-v', str(output / 'mppi-ghost.launch.xml') + ':/aichallenge/workspace/install/'
            'aichallenge_submit_launch/share/aichallenge_submit_launch/launch/control/mppi.launch.xml:ro',
            image, 'ros2', 'launch', 'aichallenge_system_launch', 'aichallenge_system.launch.xml',
            'simulation:=true', 'use_sim_time:=true', 'run_rviz:=false', f'domain_id:={number}',
            'control_method_override:=mppi', f'reference_execution_speed_cap_mps:={target_mps}',
            'rosbag:=false', 'capture:=false'])
    return commands


def run_shared_candidates(name: str, references: list[Path], target_mps: float = 10.) -> Path:
    if len(references) != 4:
        raise ValueError('Exactly four candidates are required')
    references = [path.resolve(strict=True) for path in references]
    output, config, sim_command = prepare_episode(
        name, reference=references[0], target_mps=target_mps, vehicle_count=4,
        ghost=True, ignore_other_vehicles=True, same_start=True)
    first_points = []
    for path in references:
        with path.open() as stream:
            first = next(csv.DictReader(stream))
        first_points.append([float(first['x_m']), float(first['y_m'])])
    config.update(external_controllers=True, reference_first_xy_m_by_vehicle=first_points,
                  vehicle_references=[str(path) for path in references],
                  reference_sha256_by_vehicle=[hashlib.sha256(path.read_bytes()).hexdigest() for path in references])
    (output / 'config.json').write_text(json.dumps(config, indent=2))
    simulator = 'cma-mppi-' + name
    image = json.loads((ROOT / 'environment.json').read_text())['controller_image_id']
    commands = controller_commands(output, references, image, simulator, target_mps)
    (output / 'controller_commands.json').write_text(json.dumps(commands, indent=2))
    logs, processes = [], []
    names = [simulator] + [f'{simulator}-d{i}' for i in range(1, 5)]
    begin = time.monotonic()

    def launch(command: list[str], path: Path) -> subprocess.Popen:
        log = path.open('w'); logs.append(log)
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        processes.append(process)
        return process

    try:
        print('START ' + name, flush=True)
        sim = launch(sim_command, output / 'observer.log')
        for _ in range(100):
            result = subprocess.run(['docker', 'inspect', simulator, '--format', '{{.State.Running}}'],
                                    capture_output=True, text=True, timeout=10)
            if result.returncode == 0 and result.stdout.strip() == 'true':
                break
            if sim.poll() is not None:
                raise RuntimeError('Shared simulator container exited during startup')
            time.sleep(.1)
        else:
            raise RuntimeError('Shared simulator namespace startup timeout')
        for number, command in enumerate(commands, 1):
            (output / f'd{number}').mkdir(exist_ok=True)
            launch(command, output / f'autoware-d{number}.log')
        while sim.poll() is None:
            if any(process.poll() is not None for process in processes[1:]):
                raise RuntimeError('A candidate controller container exited early')
            if time.monotonic() - begin > config['wall_timeout_s'] + 60:
                raise RuntimeError('Shared candidate host timeout')
            time.sleep(.5)
        if sim.returncode:
            raise RuntimeError(f'Shared runtime failed: {output}/observer.log')
    finally:
        for container in reversed(names):
            active = subprocess.run(['docker', 'inspect', container, '--format', '{{.State.Running}}'],
                                    capture_output=True, text=True, timeout=10)
            if active.returncode == 0 and active.stdout.strip() == 'true':
                subprocess.run(['docker', 'stop', '-t', '6', container], check=True,
                               capture_output=True, timeout=15)
        for process in processes:
            process.wait(timeout=15)
        for log in logs:
            log.close()
        (output / 'container_cleanup.json').write_text(json.dumps({
            'owned_containers': names, 'docker_clients_exited': all(p.poll() is not None for p in processes)
        }, indent=2))
    print('FINISH ' + name, flush=True)
    return output
