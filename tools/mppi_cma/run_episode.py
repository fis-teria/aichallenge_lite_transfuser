"""Launch one finite episode with private ROS networking and immutable production inputs."""
from __future__ import annotations
import argparse
import csv
import json
import math
from pathlib import Path
import subprocess
import tarfile

ROOT = Path('/home/si26-pc008/cma_mppi_20260912')


def prepare_episode(name: str, calibration: bool = False, handicap: bool = False,
                reference: Path | None = None, target_mps: float = 10.0,
                vehicle_count: int = 1, ghost: bool = False,
                ignore_other_vehicles: bool = False, same_start: bool = False) -> tuple[Path, dict, list[str]]:
    """Create immutable inputs and a command without starting simulation."""
    if not name or any(c not in 'abcdefghijklmnopqrstuvwxyz0123456789-_' for c in name):
        raise ValueError('Invalid episode name')
    if vehicle_count not in (1, 4) or (calibration and (vehicle_count != 1 or ghost)):
        raise ValueError('Driving supports one or four vehicles; calibration is separate')
    if not math.isfinite(target_mps) or not 0 < target_mps <= 10:
        raise ValueError('Target speed must be finite and in (0, 10] m/s')
    if (ghost or ignore_other_vehicles) and vehicle_count != 4:
        raise ValueError('Ghost comparison is supported only by the four-vehicle runtime')
    if same_start and (vehicle_count != 4 or not ghost or not ignore_other_vehicles):
        raise ValueError('Coincident start requires four ghosts with other-vehicle input disabled')
    output = ROOT / 'episodes' / name
    output.mkdir(parents=True, exist_ok=False)
    config = {'mode': 'calibration' if calibration else 'drive', 'target_mps': target_mps,
              'handicap': handicap, 'wall_timeout_s': 70 if calibration else 360, 'sim_timeout_s': 260,
              'vehicle_count': vehicle_count, 'vehicle_collisions': not ghost,
              'ignore_other_vehicles': ignore_other_vehicles, 'same_start': same_start}
    reference = reference or ROOT / 'snapshot/base_reference.csv'
    with reference.open() as stream:
        first = next(csv.DictReader(stream))
    config['reference_first_xy_m']=[float(first['x_m']),float(first['y_m'])]
    if calibration:
        config['calibration_points'] = [[375.2536,17.4885],[345.154,-11.163],
                                        [321.3849,-17.3376],[341.6715,13.4708]]
        scenario = ['schemaVersion: 2','name: cma calibration','vehicles:']
        for i,(x,z) in enumerate(config['calibration_points'],1):
            scenario += [f'  "{i}":',f'    at: [{x}, {z}]','    yaw: -121.84']
        (output/'scenario.yaml').write_text('\n'.join(scenario)+'\n')
    if same_start:
        pose = json.loads((ROOT/'d1_start_pose.json').read_text())
        config['start_pose_source'] = pose
        scenario = ['schemaVersion: 2', 'name: four ghosts at native D1 start', 'vehicles:']
        for number in range(1, 5):
            scenario += [f'  "{number}":', f"    at: {pose['unity_xz_m']}",
                         f"    yaw: {pose['unity_yaw_deg']}"]
        (output/'scenario.yaml').write_text('\n'.join(scenario)+'\n')
    (output/'config.json').write_text(json.dumps(config,indent=2))
    source=(ROOT/'snapshot/mppi.yaml').read_text()
    for key in ['brain.cruise_speed_mps','brain.overtake_speed_mps','brain.minimum_proximity_speed_mps']:
        needle=key+': 10.0'
        if source.count(needle)!=1:
            raise RuntimeError('Frozen speed configuration contract changed: '+key)
        source=source.replace(needle,key+': '+str(target_mps))
    (output/'mppi.yaml').write_text(source)
    cyclone = (ROOT/'snapshot/cyclonedds.xml').read_text()
    cyclone = cyclone.replace('<General>', '<Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>99</MaxAutoParticipantIndex></Discovery>\n    <General>')
    (output/'cyclonedds.xml').write_text(cyclone)
    extra_mounts = []
    if ignore_other_vehicles:
        with tarfile.open(ROOT/'snapshot/submission.tar.gz') as archive:
            member = 'aichallenge_submit/aichallenge_submit_launch/launch/control/mppi.launch.xml'
            launch = archive.extractfile(member).read().decode()
        if launch.count('/v2x/vehicle_positions') != 2:
            raise RuntimeError('Expected planner and recovery V2X inputs in frozen launch')
        (output/'mppi-ghost.launch.xml').write_text(
            launch.replace('/v2x/vehicle_positions', '/cma/ghost/vehicle_positions'))
        extra_mounts = ['-v', str(output/'mppi-ghost.launch.xml') +
                        ':/aichallenge/workspace/install/aichallenge_submit_launch/share/'
                        'aichallenge_submit_launch/launch/control/mppi.launch.xml:ro']
    metadata=json.loads((ROOT/'environment.json').read_text())
    container='cma-mppi-'+name
    args=['docker','run','--name',container,'--rm','--network','none','--cap-add','NET_ADMIN',
          '--gpus','all','--shm-size','1g','-e','NVIDIA_DRIVER_CAPABILITIES=all',
          '-e','RMW_IMPLEMENTATION=rmw_cyclonedds_cpp',
          '-e','CYCLONEDDS_URI=file:///opt/autoware/cyclonedds.xml',
          '-v',str(output/'cyclonedds.xml')+':/opt/autoware/cyclonedds.xml:ro',
          '-v',str(ROOT/'snapshot/AWSIM')+':/aichallenge/simulator/AWSIM:ro',
          '-v',str(ROOT/'tools')+':/tools:ro','-v',str(output)+':/eval',
          '-v',str(reference)+':/aichallenge/workspace/install/multi_purpose_mpc_ros/share/multi_purpose_mpc_ros/env/final_ver3/in_corce_line_straight_smooth.csv:ro',
          '-v',str(output/'mppi.yaml')+':/aichallenge/workspace/install/reference_space_mppi_planner/share/reference_space_mppi_planner/config/reference_space_mppi.param.yaml:ro',
          *extra_mounts, metadata['controller_image_id'],'python3',
          '/tools/shared_course_runtime.py' if vehicle_count == 4 else '/tools/runtime.py']
    (output/'docker_command.json').write_text(json.dumps(args,indent=2))
    return output, config, args


def run_episode(name: str, calibration: bool = False, handicap: bool = False,
                reference: Path | None = None, target_mps: float = 10.0,
                vehicle_count: int = 1, ghost: bool = False,
                ignore_other_vehicles: bool = False, same_start: bool = False) -> Path:
    output, config, args = prepare_episode(name, calibration, handicap, reference, target_mps,
                                           vehicle_count, ghost, ignore_other_vehicles, same_start)
    container = 'cma-mppi-' + name
    print('START '+name,flush=True)
    try:
        with (output/'observer.log').open('w') as log:
            result=subprocess.run(args,stdout=log,stderr=subprocess.STDOUT,timeout=config['wall_timeout_s']+60)
        if result.returncode:
            raise RuntimeError(f'Episode {name} failed, exit={result.returncode}; see {output}/observer.log')
    finally:
        active=subprocess.run(['docker','inspect',container,'--format','{{.State.Running}}'],capture_output=True,text=True)
        if active.returncode==0 and active.stdout.strip()=='true':
            subprocess.run(['docker','stop','-t','10',container],check=True,capture_output=True,timeout=20)
    print('FINISH '+name,flush=True)
    return output


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('name');parser.add_argument('--calibration',action='store_true')
    parser.add_argument('--handicap',action='store_true');parser.add_argument('--target-mps',type=float,default=10.0)
    args=parser.parse_args()
    run_episode(args.name,args.calibration,args.handicap,target_mps=args.target_mps)
