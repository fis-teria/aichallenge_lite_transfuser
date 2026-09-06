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
from spatial_dev_host_v4 import HostWatch, AttemptBudget, atomic_json, cleanup_owned

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
               V4_SOURCE_COMMIT=commit, V4_PROJECT=project,
               ROS_LOG_DIR='/evidence/ros_logs')
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
    ap.add_argument('--budget', type=Path, required=True)
    ap.add_argument('--binding', type=Path, required=True)
    ap.add_argument('--forward-limit',type=int,default=0,help='0 selects phase default; otherwise finite 1..240')
    args = ap.parse_args()
    if os.name != 'posix' or not re.fullmatch('[0-9a-f]{40}', args.commit):
        raise ValueError('LINUX_AND_FIXED_SOURCE_COMMIT_REQUIRED')
    if not 10 <= args.wall_seconds <= 120: raise ValueError('WALL_BUDGET')
    if not 0 <= args.forward_limit <= 240: raise ValueError('FORWARD_BUDGET')
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
        sim/'aichallenge/simulator/AWSIM/AWSIM_Data/level1':
            '9ab2e1e8865c02885594e0bbdde372302530f047b5a2e89c455be6a18f3e090b',
        sim/'aichallenge/simulator/AWSIM/AWSIM_Data/sharedassets1.assets':
            '932d21250cfe685c9599acf33cca2d00d17e4141676fec236b813a069d9df34c',
        sim/'aichallenge/simulator/AWSIM/AWSIM_Data/globalgamemanagers.assets':
            '9b19aa2e22e004049272dd9c9ee2a58bddef4c8f055c44540023a3de98d9cfbd',
        sim/'aichallenge/simulator/AWSIM/AWSIM_Data/resources.assets':
            '1bbe49232479849779814fb4d1259d179d6466dc7a95f21f99382e168c5ad84f',
    }
    before = {str(p):digest(p) for p in expected_assets}
    if any(before[str(p)] != sha for p, sha in expected_assets.items()):
        raise ValueError('SELECTED_SIMULATOR_CHANGED')
    binding = json.loads(args.binding.read_text())
    if binding.get('assembly_sha256') != expected_assets[sim/'aichallenge/simulator/AWSIM/AWSIM_Data/Managed/Assembly-CSharp.dll']:
        raise ValueError('BINDING_ASSET_MISMATCH')
    expected_assets[sim/'aichallenge/run_simulator.bash']=digest(sim/'aichallenge/run_simulator.bash')
    before[str(sim/'aichallenge/run_simulator.bash')]=expected_assets[sim/'aichallenge/run_simulator.bash']
    output.mkdir(parents=True)
    (output/'x11').mkdir(); (output/'x11').chmod(0o1777)
    project = 'codex-v4-dev-'+output.name.lower().replace('_', '-')
    if not re.fullmatch('[a-z0-9-]+', project): raise ValueError('INVALID_PROJECT')
    cfg = yaml.safe_load((source/'configs/control/spatial_sim_e2e_v4.yaml').read_text())
    cfg.update(enabled=args.phase == 'run', rear_x_in_base_m=.0010000169,
               lidar_x_in_base_m=1.6499999762, lidar_y_in_base_m=0.,command_schedule_s=.05)
    for key in ('body_width_m','rear_overhang_m','front_overhang_m'):
        cfg[key]=max(cfg[key],binding['body'][key])
    forward_limit=args.forward_limit or (240 if args.phase=='run' else 40)
    cfg['dev'] = dict(wall_limit_s=args.wall_seconds, forward_limit=forward_limit,
                      stage=args.phase, geometry_source='UNCHANGED_AWSIM_LEVEL1_GOKART1',
                      input_policy='SIM_ONLY_POLICY_CHANGED', history_policy='SIM_GRID_MISSING_V2',
                      evidence_binding=binding, binding_sha256=digest(args.binding), snapshots_limit=2)
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
    budget = AttemptBudget(args.budget)
    budget.reserve(output.name,dict(wall_s=args.wall_seconds+110.,forward=forward_limit,mpc=forward_limit,snapshots=2,
                                   powered=int(args.phase=='run'),powered_s=60. if args.phase=='run' else 0.,log_bytes=208*1024**2))
    watch = HostWatch(project)
    started = time.monotonic()
    pause_verified = False
    error = None
    exitcode = None
    finalization_freeze_ns = None
    try:
        run(command+['up', '-d', '--no-build', '--pull', 'never', 'simulator', 'autoware'])
        sim_id = run(command+['ps', '-q', 'simulator']).stdout.strip()
        runtime_id = run(command+['ps', '-q', 'autoware']).stdout.strip()
        if not re.fullmatch('[0-9a-f]{64}', sim_id) or not re.fullmatch('[0-9a-f]{64}', runtime_id):
            raise ValueError('OWNED_CONTAINER_IDENTIFICATION')
        inspected = run(['docker', 'inspect', sim_id, runtime_id])
        atomic_json(output/'instance_inspect.json',json.loads(inspected.stdout))
        while time.monotonic()-started < args.wall_seconds+60:
            if sum(p.stat().st_size for p in output.rglob('*') if p.is_file()) >= 207*1024**2:
                raise RuntimeError('ATTEMPT_LOG_RESERVATION_LIMIT')
            state = json.loads(run(['docker', 'inspect', runtime_id, '--format', '{{json .State}}']).stdout)
            if not state['Running']:
                exitcode = state['ExitCode']
                break
            if finalization_freeze_ns is not None:
                # Heartbeats may end ONLY after host verified the owned simulator
                # frozen. This is process cleanup, never proof of vehicle braking.
                if time.monotonic_ns()-finalization_freeze_ns > 5_000_000_000:
                    raise RuntimeError('RUNTIME_FINALIZE_TIMEOUT_WHILE_SIM_FROZEN')
                time.sleep(.10)
                continue
            heartbeat = output/'heartbeat.json'
            heartbeat_data = json.loads(heartbeat.read_text()) if heartbeat.exists() else None
            fault = watch.check(heartbeat_data,time.monotonic_ns())
            if fault:
                run(['docker','pause',sim_id])
                paused = json.loads(run(['docker','inspect',sim_id,'--format','{{json .State}}']).stdout)
                pause_verified = paused['Paused'] is True
                error = fault
                break
            if watch.freeze_requested:
                run(['docker','pause',sim_id])
                paused = json.loads(run(['docker','inspect',sim_id,'--format','{{json .State}}']).stdout)
                if paused.get('Paused') is not True:
                    raise RuntimeError('FINALIZATION_FREEZE_NOT_VERIFIED')
                pause_verified = True
                finalization_freeze_ns = time.monotonic_ns()
                atomic_json(output/'finalization_freeze.json',dict(monotonic_ns=finalization_freeze_ns,
                    token=project,sim_id=sim_id,runtime_id=runtime_id,paused=True,
                    reason='SUPERVISOR_FINISHED_STOP_POLICY_NOT_BRAKING_PROOF'))
                continue
            if watch.armed:
                atomic_json(output/'host_armed.json',dict(token=project,armed=True,monotonic_ns=time.monotonic_ns(),
                    sim_id=sim_id,runtime_id=runtime_id,paused_kill_verified=binding['host_paused_kill']['verified']))
            time.sleep(.20)
        else: error = 'HOST_FINITE_TIMEOUT'
    except BaseException as exc:
        error = type(exc).__name__+': '+str(exc)
    finally:
        # Shutdown operations still execute if writing host.log has failed.
        def cleanup_run(argv,timeout=15):
            proc = subprocess.run(argv,capture_output=True,text=True,timeout=timeout)
            try:
                stdout.write(json.dumps(dict(cleanup_command=argv,returncode=proc.returncode,
                                            stdout=proc.stdout,stderr=proc.stderr))+'\n'); stdout.flush()
            except Exception:
                pass  # resource cleanup outcome retained separately below
            if proc.returncode: raise RuntimeError(proc.stderr)
            return proc
        cleanup = cleanup_owned(cleanup_run,sim_id,runtime_id,
            paused_kill_verified=binding['host_paused_kill']['verified'] is True)
        after = {}
        for p in expected_assets:
            try: after[str(p)] = digest(p)
            except Exception as exc: cleanup['errors'].append(dict(stage='asset_hash',error=str(exc)))
        summary = dict(error=error, runtime_exitcode=exitcode, source_commit=args.commit, project=project,
            simulator_assets_before=before, simulator_assets_after=after, simulator_assets_unchanged=before == after,
            simulator_container_id=sim_id, runtime_container_id=runtime_id,
            simulator_wall_seconds=time.monotonic()-started, cleanup=cleanup,host_armed=watch.armed,
            finalization_freeze_ns=finalization_freeze_ns,
            host_pause_verified=pause_verified, pause_is_not_natural_braking_stop=True,
            normal_racingkart_makefile_executed=False, entrypoint='LITE_TRANSFUSER_MAKE_DEV',
            original_simulator_script='aichallenge/run_simulator.bash dev', awsimmutation=False)
        (output/'host_summary.json').write_text(json.dumps(summary, indent=2))
        consumption = dict(wall_s=summary['simulator_wall_seconds'])
        # Charge an extra 1 MiB for summary/budget file finalization, explicitly
        # conservative rather than understating bytes written after this count.
        consumption['log_bytes']=sum(p.stat().st_size for p in output.rglob('*') if p.is_file())+1024**2
        worker_summary, supervisor_summary = output/'worker_summary.json',output/'supervisor_summary.json'
        if worker_summary.exists():
            w=json.loads(worker_summary.read_text())
            consumption.update(forward=w['forward_calls'],mpc=w['mpc_calls'],snapshots=w['sensor_tensor_snapshots'])
        if supervisor_summary.exists():
            s=json.loads(supervisor_summary.read_text())
            consumption.update(powered=s['powered_episode_count'],powered_s=s['powered_sim_seconds'])
        budget.finish(consumption,exact=len(consumption)==7)
        (output/'budget_after.json').write_text(args.budget.read_text())
        budget.close()
        stdout.close()
    print(json.dumps(summary))
    return 1 if error or exitcode != 0 or cleanup['errors'] else 0


if __name__ == '__main__': raise SystemExit(main())
