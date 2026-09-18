"""Run one bounded AWSIM teacher recording with the deployed V45/LiDAR overlay.

Default action compiles a preview. --execute starts the named simulation once.
The scenario tool remains responsible for Start, monitoring and scoped cleanup.
No source-control action, model training or deletion is performed here.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone
from typing import Any


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def desktop_environment() -> None:
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            # GNOME on Xorg has no Xwayland process. Its session still carries
            # the authoritative DISPLAY/XAUTHORITY required by Docker mounts.
            if entry.stat().st_uid != os.getuid() or (entry/'comm').read_text().strip() not in {
                'Xwayland', 'gnome-shell', 'gnome-session-b', 'Xorg',
            }:
                continue
            values = dict(item.split('=',1) for item in (entry/'environ').read_text().split('\0') if '=' in item)
            for key in ('DISPLAY','XAUTHORITY','XDG_RUNTIME_DIR'):
                if values.get(key):
                    os.environ[key] = values[key]
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
    os.environ.update(HOST_UID=str(os.getuid()),HOST_GID=str(os.getgid()))
    os.environ.pop('CONTROL_METHOD',None)


def finalize_interrupted(context: Any, scenario: Path, run_id: str) -> dict:
    """Persist an explicitly interrupted verdict after scoped cleanup.

    The external runner raises KeyboardInterrupt before writing result.json.
    Re-evaluate the closed raw artifacts without claiming a normal exit.
    """
    from scenario_tool import report
    from scenario_tool.cli import _load_and_resolve
    from scenario_tool.evaluate import evaluate
    from scenario_tool.occupancy import load as load_occupancy
    run = context.output_root/run_id
    if (run/'result.json').exists():
        return json.loads((run/'result.json').read_text())
    assert not subprocess.check_output(['docker','ps','-aq','--filter',
        'label=com.docker.compose.project=codex-'+run_id],text=True).strip()
    manifest = json.loads((run/'run-manifest.json').read_text())
    assert sha(scenario) == manifest['scenario']['sha256']
    resolved, maps, line, _ = _load_and_resolve(context,scenario,False)
    assert sha(maps.reference_csv) == manifest['environment']['map_reference_csv_sha256']
    status = json.loads((run/'monitor-status.json').read_text())
    report.normalize_events(run)
    evaluation = evaluate(resolved,run,status,line,load_occupancy(maps.occupancy_yaml))
    now = datetime.now(timezone.utc)
    started = manifest['timing']['runner_started_at']
    result = report.build_result(run_id,resolved,evaluation,dict(ok=False,phase='failed',
        reason='Collection supervisor interrupted the run; see the preserved stop reason',
        exit_code=130,monitor_status=None),run,started,now.isoformat(),
        (now-datetime.fromisoformat(started)).total_seconds())
    errors = report.write_result(context,run,result)
    assert not errors, errors
    report.write_report_md(run,result,resolved,resolved.warnings)
    report.write_junit(run,result)
    return result


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--awsim-repo', type=Path, required=True)
    ap.add_argument('--runtime', type=Path, required=True)
    ap.add_argument('--scenario', type=Path, required=True)
    ap.add_argument('--run-id', required=True)
    ap.add_argument('--speed-cap-kmh', type=float, default=5.)
    ap.add_argument('--early-entry-search', action='store_true',
                    help='Explore earlier lateral entry for collection AVOID at <=10 km/h')
    ap.add_argument('--wall-timeout-s', type=int, default=480)
    ap.add_argument('--run-budget-gib', type=float, default=1.)
    ap.add_argument('--free-reserve-gib', type=float, default=2.)
    ap.add_argument('--execute', action='store_true')
    ap.add_argument('--rviz', action='store_true', help='Show the teacher path and LiDAR in standard RViz')
    args = ap.parse_args()
    if not re.fullmatch(r'lidar-v45-pc10-[a-z0-9-]+',args.run_id):
        raise ValueError('A new lidar-v45-pc10-* run ID is required')
    if not 0 < args.speed_cap_kmh <= 10 or not 120 <= args.wall_timeout_s <= 2400:
        raise ValueError('Pilot cap must be (0,10] km/h, wall timeout 120..2400s')
    if not .25 <= args.run_budget_gib <= 4 or not 2 <= args.free_reserve_gib <= 16:
        raise ValueError('Explicit per-run storage and at least 2GiB reserve required')
    repo = args.awsim_repo.resolve()
    source = Path(__file__).resolve().parents[1]
    runtime = args.runtime.resolve()
    vendor_root = source/'integrations/mppi_v45'
    vendor_manifest = vendor_root/'source_manifest.json'
    for row in json.loads(vendor_manifest.read_text())['files']:
        assert sha(vendor_root/row['path']) == row['sha256'], row['path']
    assert json.loads((runtime/'build-result.json').read_text())['exit_code']==0
    identity = json.loads((runtime/'runtime-identity.json').read_text())
    assert identity['source_manifest_sha256'] == sha(vendor_manifest), 'Runtime/source revision mismatch'
    for item in identity['files'].values():
        assert sha(runtime/'install'/item['path']) == item['sha256']
    sys.path.insert(0,str(repo/'scenario_tool'))
    from scenario_tool import compiler, yamlio
    from scenario_tool.cli import _load_and_resolve
    from scenario_tool.context import Context
    from scenario_tool.runner import run_once
    from scenario_tool.submission import Submission
    context = Context(repo)
    desktop_environment()
    original_compile = compiler.compile_run
    scenario = yamlio.load_file(args.scenario)
    assert scenario['runtime']['rosbag'] is True
    assert scenario['simulator']['collisions']=='on'
    assert scenario['expect']['timeout_sec'] < args.wall_timeout_s
    assert scenario['ego']['submission']['type']=='docker_image'
    assert all(actor['profile']=='static_physical' for actor in scenario.get('actors',[]))
    assert scenario['ego'].get('id','ego')=='ego'
    image = scenario['ego']['submission']['image']

    def compile_case(*positional, **kwargs):
        compiled = original_compile(*positional, **kwargs)
        # Create the recorder destination as the host user before root-owned ROS
        # processes start. Otherwise the teacher can create d1 first as root.
        (compiled.run_dir/'d1').mkdir(parents=True, exist_ok=True)
        document = yamlio.load_file(compiled.compose_file)
        services = document['services']
        run = compiled.container_run_dir
        assets = source/'integrations/mppi_v45/assets/multi_purpose_mpc_ros/env/final_ver3'
        recovery_config = source/'integrations/mppi_v45/assets/multi_purpose_mpc_ros/config/config.yaml'
        volumes = [f'{repo}/aichallenge:/aichallenge:ro',f'{source}:/source:ro',f'{runtime}:/runtime:ro',
            f'{assets}:/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/env/final_ver3:ro',
            f'{assets}:/aichallenge/workspace/install/multi_purpose_mpc_ros/share/multi_purpose_mpc_ros/env/final_ver3:ro',
            f'{recovery_config}:/aichallenge/workspace/install/multi_purpose_mpc_ros/share/multi_purpose_mpc_ros/config/config.yaml:ro']
        ego = services['scn-car1']
        ego['volumes'] = [*ego.get('volumes',[]),*volumes]
        ego['environment'].update(VEHICLE_ID='d1',TEACHER_SPEED_CAP_MPS=str(args.speed_cap_kmh/3.6),
                                  TEACHER_RVIZ='true' if args.rviz else 'false',
                                  TEACHER_EARLY_ENTRY_SEARCH='true' if args.early_entry_search else 'false')
        if args.rviz:
            ego['volumes'].append(f'{source}/integrations/mppi_v45/teacher_collection.rviz:'
                '/aichallenge/workspace/install/aichallenge_system_launch/share/aichallenge_system_launch/config/autoware.rviz:ro')
        ego['command'] = ['bash','/source/integrations/mppi_v45/run_teacher.bash']
        recorder = services['scn-rosbag1']
        recorder['volumes'] = [*recorder.get('volumes',[]),*volumes]
        recorder['user'] = '${HOST_UID}:${HOST_GID}'
        recorder['environment'].update(HOME='/tmp',ROS_HOME='/tmp/ros-recorder',
            E2E_ROSBAG_MAX_CACHE_SIZE='67108864',E2E_ROSBAG_COMPRESSION_FORMAT='zstd',E2E_ROSBAG_COMPRESSION_MODE='file')
        raw_script = (context.tool_dir/'agent/record_e2e_rosbag.bash').read_text()
        hook = 'source /aichallenge/workspace/install/setup.bash'
        assert raw_script.count(hook)==1
        raw_script = raw_script.replace(hook,hook+'\nsource /runtime/install/local_setup.bash')
        raw_script = raw_script.replace('  --storage "${STORAGE_ID}"',
            '  --storage "${STORAGE_ID}"\n  --max-bag-size 268435456')
        (compiled.run_dir/'record_teacher.bash').write_text(raw_script)
        shutil.copyfile(context.tool_dir/'agent/validate_e2e_rosbag.py',compiled.run_dir/'validate_e2e_rosbag.py')
        recorder['command'] = ['bash',f'{run}/record_teacher.bash',f'{run}/d1',f'{run}/e2e-rosbag-topics.txt',
            scenario['simulator']['camera'],scenario['simulator']['lidar']]
        simulator = services['scn-simulator']
        simulator['volumes'] = [*simulator.get('volumes',[]),f'{repo}/aichallenge:/aichallenge:ro']
        for service in services.values():
            service['ulimits'] = {'core':0}
            service['logging'] = {'driver':'json-file','options':{'max-size':'10m','max-file':'2'}}
        extra_topics = ['/collection/lidar_v2x/vehicle_positions','/collection/lidar_v2x/objects',
            '/collection/lidar_v2x/status','/collection/lidar_v2x/markers','/control/mpc/stop_request',
            '/debug/mppi/status','/debug/mppi/stuck_recovery','/mppi/direct/trajectory_command',
            '/sensing/gnss/pose_with_covariance','/sensing/imu/imu_data']
        topics_file = compiled.run_dir/'e2e-rosbag-topics.txt'
        topics = topics_file.read_text().splitlines()
        topics_file.write_text('\n'.join(dict.fromkeys([*topics,*extra_topics]))+'\n')
        yamlio.dump_file(compiled.compose_file,document)
        compiled.services = services
        (compiled.run_dir/'teacher-contract.json').write_text(json.dumps(dict(
            teacher='MPPI_SIM_V45',runtime_identity=identity,cap_mps=args.speed_cap_kmh/3.6,
            perception='LIDAR_V2X_SURFACE_EXISTING_MARGIN',native_v2x_role='EVALUATION_ONLY',
            student_control=False,student_inference=False,online_training=False,awsim_modified=False,
            source_manifest_sha256=sha(vendor_manifest),
            rviz_requested=args.rviz,
            scenario_sha256=sha(args.scenario),wall_budget_s=args.wall_timeout_s,
            storage_budget_bytes=int(args.run_budget_gib*2**30)),indent=2)+'\n')
        return compiled

    compiler.compile_run = compile_case
    if not args.execute:
        output = runtime.parent/'previews'/args.run_id
        assert not output.exists()
        resolved,maps,_,transform = _load_and_resolve(context,args.scenario,False)
        submission = Submission(kind='docker_image',image=image,base_service='autoware',mounts_workspace=False)
        compiled = compile_case(context=context,resolved=resolved,run_dir=output,
            container_run_dir=context.container_output_root+'/'+args.run_id,submissions={'ego':submission},
            reference_csv=maps.reference_csv,transform_available=transform is not None,gpu_enabled=True)
        subprocess.run(['docker','compose','-p','codex-'+args.run_id,'-f',str(context.compose_file),
            '-f',str(compiled.compose_file),'config','--quiet'],cwd=repo,check=True)
        print(json.dumps({'preview':str(output),'compose_valid':True}),flush=True)
        return
    output = context.output_root/args.run_id
    assert not output.exists(), 'Preserve previous recordings'
    assert subprocess.run(['pgrep','-x','AWSIM.x86_64'],capture_output=True).returncode==1
    assert shutil.disk_usage(repo).free > (args.free_reserve_gib+args.run_budget_gib)*2**30
    done = threading.Event()
    started = time.monotonic()
    def budget() -> None:
        while not done.wait(2):
            free = shutil.disk_usage(repo).free
            used = sum(p.stat().st_size for p in output.rglob('*') if p.is_file()) if output.exists() else 0
            if (time.monotonic()-started > args.wall_timeout_s or free < args.free_reserve_gib*2**30
                    or used > args.run_budget_gib*2**30):
                (runtime.parent/(args.run_id+'-budget-stop.json')).write_text(json.dumps({'free_bytes':free,'used_bytes':used})+'\n')
                os.kill(os.getpid(),signal.SIGINT)
                return
    threading.Thread(target=budget,daemon=True).start()
    try:
        try:
            result = run_once(context,args.scenario,dict(run_id=args.run_id,project='codex-'+args.run_id,
                gpu=True,rviz=False,allow_stale_calibration=False))
        except KeyboardInterrupt:
            done.set()
            result = finalize_interrupted(context,args.scenario,args.run_id)
        (runtime.parent/(args.run_id+'-result.json')).write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps({'run_id':args.run_id,'exit_code':result.get('exit_code'),
            'verdict':result.get('scenario_verdict')}),flush=True)
        raise SystemExit(result['exit_code'])
    finally:
        done.set()


if __name__ == '__main__':
    main()
