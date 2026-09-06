"""Linux make-dev runner: existing AWSIM binary + bounded V4/MPC containers.

No original racing-kart checkout edits, no broad source/weight tree hashing, no
new images, no datasets, no hardware network/device access and no git push.
The normal racing-kart Makefile is NOT silently claimed to have been executed.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import time

import yaml

IMAGE = 'sha256:8c650c13157ffabbc3a72bab08865ccff8338f9025b43a8f4405b4c6b96d1ba7'
EXPECTED_CHECKPOINT = '0316692543a901d9d6b96718c5651d6739367aa851f83fb3a133c126ccc8919f'
CHECKPOINT_TARGET = '/home/thistle/e2e_autonomous/runs/spatial_diagnostic_v4_20260906_f33b197/checkpoints/final.pt'


def digest(path: Path) -> str:
    h = hashlib.sha256()
    with path.open('rb') as f:
        for part in iter(lambda:f.read(1024*1024), b''): h.update(part)
    return h.hexdigest()


def compose_definition(source: Path, sim: Path, xvfb: Path, output: Path, checkpoint: Path,
                       project: str, commit: str) -> dict:
    common = dict(image=IMAGE, pull_policy='never', privileged=False, network_mode='none',
                  cap_drop=['ALL'], security_opt=['no-new-privileges:true'], ipc='private',
                  pids_limit=512, mem_limit='6g', cpus=4, shm_size='256m',
                  user=f'{os.getuid()}:{os.getgid()}', entrypoint=['/bin/bash'], restart='no',
                  gpus='all', stop_grace_period='12s', working_dir='/evidence')
    env = dict(ROS_LOCALHOST_ONLY='0', RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',
               CYCLONEDDS_URI='file:///v4/integrations/awsim_dev_v4/cyclonedds.xml',
               V4_SOURCE_COMMIT=commit, V4_PROJECT=project)
    def volume(path: Path, target: str, ro: bool = True) -> dict:
        return dict(type='bind', source=str(path.resolve()), target=target, read_only=ro)
    shared = [volume(source, '/v4'), volume(output, '/evidence', False)]
    simulator = dict(common, command=['/v4/integrations/awsim_dev_v4/simulator.sh'],
        environment=dict(env, ROS_DOMAIN_ID='0', DISPLAY=':99', __NV_PRIME_RENDER_OFFLOAD='1',
            AWSIM_START_MODE='off', AWSIM_VEHICLES='1', AWSIM_LAPS='600', AWSIM_TIMEOUT='60000000',
            AWSIM_EXTRA_ARGS='--manual-mode true --npcs 0 --boosts 0 --collisions on --wall-recovery off --start-random off --camera gpu --lidar gpu --sound off --target-fps 20 -screen-width 640 -screen-height 360 -logFile /evidence/awsim_unity.log'),
        volumes=shared+[volume(sim/'aichallenge/simulator/AWSIM', '/aichallenge/simulator/AWSIM'),
                       volume(sim/'aichallenge/run_simulator.bash', '/aichallenge/run_simulator.bash'),
                       volume(xvfb, '/xvfb'), volume(xvfb/'usr/bin/xkbcomp', '/usr/bin/xkbcomp'),
                       volume(output/'x11', '/tmp/.X11-unix', False)])
    runtime = dict(common, network_mode='service:simulator', depends_on=['simulator'],
        command=['/v4/integrations/awsim_dev_v4/runtime.sh'], environment=dict(env, ROS_DOMAIN_ID='1'),
        volumes=shared+[volume(checkpoint, CHECKPOINT_TARGET)])
    return dict(name=project, services=dict(simulator=simulator, autoware=runtime))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--sim-repo', type=Path, required=True)
    ap.add_argument('--checkpoint', type=Path, required=True)
    ap.add_argument('--xvfb-root', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--commit', required=True)
    ap.add_argument('--phase', choices=('stationary', 'run'), default='stationary')
    ap.add_argument('--wall-seconds', type=int, default=60)
    args = ap.parse_args()
    if os.name != 'posix' or not re.fullmatch('[0-9a-f]{40}', args.commit):
        raise ValueError('LINUX_AND_FIXED_SOURCE_COMMIT_REQUIRED')
    if not 10 <= args.wall_seconds <= 120: raise ValueError('WALL_BUDGET')
    source = Path(__file__).resolve().parents[1]
    sim = args.sim_repo.resolve()
    output = args.output.resolve()
    if output.exists(): raise FileExistsError(output)
    if digest(args.checkpoint) != EXPECTED_CHECKPOINT: raise ValueError('CHECKPOINT_IDENTITY')
    simulator_binary = sim/'aichallenge/simulator/AWSIM/AWSIM.x86_64'
    vehicle_yaml = sim/'aichallenge/simulator/AWSIM/AWSIM_Data/StreamingAssets/Vehicle/vehicle.yaml'
    expected_assets = {
        simulator_binary: '0bfe51325720c950b4ad75ce7c3b65595fd7e939908520ec33600919919327a8',
        vehicle_yaml: '5b66e58691091c82d5511535ca458d4c89e85f3b59fa0d0c4a2c7eee47887244',
        sim/'aichallenge/simulator/AWSIM/AWSIM_Data/Managed/Assembly-CSharp.dll':
            '859e5560dbffd7d0833cd1fe5eb1f36b87fa8d22f6e45eb0e1203d04a1ea6d13',
    }
    before = {str(p):digest(p) for p in expected_assets}
    if any(before[str(p)] != sha for p, sha in expected_assets.items()):
        raise ValueError('SELECTED_SIMULATOR_CHANGED')
    output.mkdir(parents=True)
    (output/'x11').mkdir(); (output/'x11').chmod(0o1777)
    project = 'codex-v4-dev-'+output.name.lower().replace('_', '-')
    if not re.fullmatch('[a-z0-9-]+', project): raise ValueError('INVALID_PROJECT')
    cfg = yaml.safe_load((source/'configs/control/spatial_sim_e2e_v4.yaml').read_text())
    cfg.update(enabled=args.phase == 'run', rear_x_in_base_m=.0010000169,
               lidar_x_in_base_m=1.6499999762, lidar_y_in_base_m=0.)
    cfg['dev'] = dict(wall_limit_s=args.wall_seconds, forward_limit=60, footprint_profile_verified=False,
                      stage=args.phase, geometry_source='UNCHANGED_AWSIM_LEVEL1_GOKART1',
                      pose_timing='PENDING_FULL_BINDING', collision_monitor='MISSING',
                      input_policy='SIM_ONLY_POLICY_CHANGED')
    (output/'resolved_config.yaml').write_text(yaml.safe_dump(cfg, sort_keys=False))
    spec = compose_definition(source, sim, args.xvfb_root, output, args.checkpoint, project, args.commit)
    spec_path = output/'compose.json'
    spec_path.write_text(json.dumps(spec, indent=2))
    command = ['docker', 'compose', '-p', project, '-f', str(spec_path)]
    stdout = (output/'host.log').open('x', buffering=1)
    def run(argv: list[str], *, timeout: float = 30, check: bool = True) -> subprocess.CompletedProcess:
        proc = subprocess.run(argv, capture_output=True, text=True, timeout=timeout)
        stdout.write(json.dumps(dict(command=argv, returncode=proc.returncode, stdout=proc.stdout, stderr=proc.stderr))+'\n')
        stdout.flush()
        if check and proc.returncode: raise RuntimeError(proc.stderr[:1500])
        return proc
    if run(command+['ps', '-aq']).stdout.strip(): raise ValueError('PROJECT_ALREADY_EXISTS')
    resolved = run(command+['config', '--format', 'json'])
    (output/'compose_resolved.json').write_text(resolved.stdout)
    runtime_id = sim_id = None
    started = time.monotonic()
    pause_verified = False
    error = None
    exitcode = None
    try:
        run(command+['up', '-d', '--no-build', '--pull', 'never', 'simulator', 'autoware'])
        sim_id = run(command+['ps', '-q', 'simulator']).stdout.strip()
        runtime_id = run(command+['ps', '-q', 'autoware']).stdout.strip()
        if not re.fullmatch('[0-9a-f]{64}', sim_id) or not re.fullmatch('[0-9a-f]{64}', runtime_id):
            raise ValueError('OWNED_CONTAINER_IDENTIFICATION')
        inspected = run(['docker', 'inspect', sim_id, runtime_id])
        (output/'instance_inspect.json').write_text(inspected.stdout)
        while time.monotonic()-started < args.wall_seconds+60:
            state = json.loads(run(['docker', 'inspect', runtime_id, '--format', '{{json .State}}']).stdout)
            if not state['Running']:
                exitcode = state['ExitCode']
                break
            heartbeat = output/'heartbeat.json'
            if heartbeat.exists():
                heartbeat_data = json.loads(heartbeat.read_text())
                age = (time.monotonic_ns()-heartbeat_data['monotonic_ns'])*1e-9
                if age > .75 and heartbeat_data['powered']:
                    run(['docker', 'pause', sim_id])
                    paused = json.loads(run(['docker', 'inspect', sim_id, '--format', '{{json .State}}']).stdout)
                    pause_verified = paused['Paused'] is True
                    error = 'SUPERVISOR_HEARTBEAT_STALE_SIM_PROCESS_PAUSED'
                    break
            time.sleep(.20)
        else: error = 'HOST_FINITE_TIMEOUT'
    except BaseException as exc:
        error = type(exc).__name__+': '+str(exc)
    finally:
        # Exact task-owned services only; no down/remove-orphans and no broad pkill.
        run(command+['logs', '--no-color', '--tail', '300', 'autoware'], check=False)
        if pause_verified and sim_id: run(['docker', 'unpause', sim_id], check=False)
        run(command+['stop', '-t', '12', 'autoware', 'simulator'], timeout=35, check=False)
        after = {str(p):digest(p) for p in expected_assets}
        summary = dict(error=error, runtime_exitcode=exitcode, source_commit=args.commit, project=project,
            simulator_assets_before=before, simulator_assets_after=after, simulator_assets_unchanged=before == after,
            simulator_container_id=sim_id, runtime_container_id=runtime_id,
            simulator_wall_seconds=time.monotonic()-started, prior_task_simulator_wall_seconds=1005.283827104,
            host_pause_verified=pause_verified, pause_is_not_natural_braking_stop=True,
            normal_racingkart_makefile_executed=False, entrypoint='LITE_TRANSFUSER_MAKE_DEV',
            original_simulator_script='aichallenge/run_simulator.bash dev', awsimmutation=False)
        (output/'host_summary.json').write_text(json.dumps(summary, indent=2))
        stdout.close()
    print(json.dumps(summary))
    return 1 if error or exitcode != 0 else 0


if __name__ == '__main__': raise SystemExit(main())
