"""Finite, owned .10 AWSIM recovery collection with raw recording and stop proof."""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import signal
import subprocess
import time

from aic_transfuser_lite.runtime.awsim_trial_session import JudgeLog
from integrate_normal_rviz_v4 import DISPLAY, follow_ego_view

ROOT = Path('/home/graneple/e2e_autonomous/time_recovery_collection_20260913')
REPO = Path('/home/graneple/git/autononous_ai/aichallenge-racingkart')


def sha(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4*1024**2), b''):
            digest.update(block)
    return digest.hexdigest()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', required=True)
    ap.add_argument('--side', choices=['left', 'right'], required=True)
    ap.add_argument('--preflight-only', action='store_true')
    args = ap.parse_args()
    if not re.fullmatch(r'codex-time-recovery-[a-z0-9-]+', args.run_id):
        raise ValueError('OWNED_RUN_ID_REQUIRED')
    source = Path(__file__).resolve().parents[1]
    if source != ROOT/'source':
        raise ValueError('DEDICATED_SOURCE_REQUIRED')
    output = ROOT/args.run_id; output.mkdir(exist_ok=False)
    env = dict(os.environ); streams = []; children = []; owned = False; compose = None
    result = dict(status='FAILED', run_id=args.run_id, side=args.side, official_start_requested=False,
                  started_unix_s=time.time(), scope='MEASURED_RECOVERY_AWSIM', fixed_target_mps=5/3.6,
                  source_sha=(ROOT/'deployed_commit.txt').read_text().strip(),
                  sim_limit_s=1800, wall_limit_s=1860, outer_limit_s=1980,
                  run_byte_limit=3*1024**3, task_bag_byte_limit=10*1024**3, required_free_bytes=10*1024**3)
    started = time.monotonic(); judge = JudgeLog(args.run_id); read_offset=0; pending=b''
    lap_seen_ns = None; fault_wall = None; last_size_check = 0.; rviz_window = None; control = {}

    def run(cmd, timeout=10, check=True):
        return subprocess.run(cmd, cwd=REPO, env=env, capture_output=True, text=True, timeout=timeout, check=check)

    def launch(cmd, name):
        stream = (output/(name+'.log')).open('x'); streams.append(stream)
        p = subprocess.Popen(cmd, cwd=REPO, env=env, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        children.append(p); return p

    def request_stop(reason):
        if not (output/'stop_request.json').exists():
            p = output/'stop_request.pending'; p.write_text(json.dumps({'run_id':args.run_id, 'reason':reason}))
            p.replace(output/'stop_request.json')

    def interrupted(signum, frame):
        raise RuntimeError('OUTER_SUPERVISOR_INTERRUPTED')
    signal.signal(signal.SIGTERM, interrupted)
    try:
        (output/'containers_before.jsonl').write_text(run(['docker','ps','-a','--format','{{json .}}']).stdout)
        (output/'compose_before.json').write_text(run(['docker','compose','ls','--all','--format','json']).stdout)
        if run(['docker','ps','-q']).stdout.strip():
            raise RuntimeError('ACTIVE_CONTAINER_PRESENT')
        if shutil.disk_usage(ROOT).free < 10*1024**3:
            raise RuntimeError('INSUFFICIENT_DISK')
        result['prior_task_bag_bytes'] = sum(p.stat().st_size for p in ROOT.glob('codex-time-recovery-*/bag/*') if p.is_file())
        if result['prior_task_bag_bytes']+result['run_byte_limit'] > result['task_bag_byte_limit']:
            raise RuntimeError('TASK_BAG_BUDGET_EXHAUSTED')
        if any(Path(p).exists() for p in ('/dev/vcu','/dev/gnss','/dev/ttyUSB0')):
            raise RuntimeError('PHYSICAL_DEVICE_PRESENT')
        result['gpu'] = run(['nvidia-smi','--query-gpu=name,driver_version','--format=csv,noheader']).stdout
        config = json.loads((source/'configs/control/time_path_vehicle_model_5kmh_20260913.json').read_text())
        simulator = REPO/'aichallenge/simulator/AWSIM'
        hashes = {config['geometry']['scene_file']:config['geometry']['scene_sha256'], **config['steering_asset_sha256']}
        if {p:sha(simulator/p) for p in hashes} != hashes:
            raise RuntimeError('SIMULATOR_ASSET_IDENTITY')
        result['simulator_assets'] = hashes
        ref = ROOT/'references'/(args.side+'.json'); reference = json.loads(ref.read_text())
        if sha(ref.with_suffix('.csv')) != reference['reference_sha256']:
            raise RuntimeError('REFERENCE_SHA_MISMATCH')
        result['reference_sha256'] = reference['reference_sha256']
        (output/'reference.json').write_bytes(ref.read_bytes())
        if not (ROOT/'cpp_install/setup.bash').is_file():
            raise RuntimeError('ISOLATED_CPP_INSTALL_MISSING')
        dds = (REPO/'vehicle/cyclonedds.xml').read_text()
        if 'name="lo"' not in dds or '<SocketReceiveBufferSize min="10MB"/>' not in dds:
            raise RuntimeError('DDS_PROFILE_CHANGED')
        dds = dds.replace('<SocketReceiveBufferSize min="10MB"/>','<SocketReceiveBufferSize min="128kB"/>')
        dds = dds.replace('<AllowMulticast>default</AllowMulticast>','<AllowMulticast>false</AllowMulticast>')
        dds = dds.replace('</Domain>', '<Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain>')
        runtime_dds = output/'cyclonedds.xml'; runtime_dds.write_text(dds)
        rviz = REPO/'aichallenge/workspace/src/aichallenge_system/aichallenge_system_launch/config/autoware.rviz'
        original = rviz.read_text(); (output/'autoware.rviz.before').write_text(original)
        anchor = 'Visualization Manager:\n  Class: ""\n  Displays:\n'
        if original.count(anchor) != 1:
            raise RuntimeError('RVIZ_LAYOUT_CHANGED')
        blocks = ''
        for name,color in [('baseline','220; 220; 220'),('reference','255; 130; 30'),('observed','0; 220; 255')]:
            blocks += DISPLAY.replace('V4-20 raw prediction','Recovery teacher '+name).replace('/visualization/v4_20/raw_path','/recovery_teacher/'+name+'_path').replace('255; 60; 180',color)
        rviz_copy = output/'autoware.rviz'; rviz_copy.write_text(follow_ego_view(original.replace(anchor,anchor+blocks,1)))
        mounts = [str(runtime_dds)+':/opt/autoware/cyclonedds.xml:ro', str(rviz_copy)+':/aichallenge/workspace/src/aichallenge_system/aichallenge_system_launch/config/autoware.rviz:ro']
        override = output/'compose.json'
        override.write_text(json.dumps({'services': {s:{'volumes':mounts} for s in ('simulator','autoware','autoware-command')}}))
        auth = Path('/run/user/1000/gdm/Xauthority')
        if not auth.is_file():
            raise RuntimeError('DISPLAY_AUTH_MISSING')
        env.update(DISPLAY=':1', XAUTHORITY=str(auth), COMPOSE_PROJECT_NAME=args.run_id,
            COMPOSE_FILE=':'.join(map(str,(REPO/'docker-compose.yml',REPO/'docker-compose.gpu.yml',override))),
            CONTROL_METHOD='v4_20_external',V4_SHADOW_ENABLED='false',AWSIM_VEHICLES='1')
        compose = ['docker','compose','-p',args.run_id]
        inside = '/capture/'+args.run_id
        shell = ('source /aichallenge/workspace/install/setup.bash && source /capture/cpp_install/setup.bash && '
            'export PYTHONPATH=/capture/source/src:${PYTHONPATH:-} && exec '+shlex.join(['python3','/capture/source/tools/run_time_recovery_nodes.py',
            '--output',inside,'--reference-root','/capture/references','--side',args.side,'--run-id',args.run_id]))
        probe_cmd = ['docker','run','--rm','--name',args.run_id+'-nodes','--network','host','-e','ROS_DOMAIN_ID=1',
            '-e','CYCLONEDDS_URI=file:///opt/autoware/cyclonedds.xml','-v',str(runtime_dds)+':/opt/autoware/cyclonedds.xml:ro',
            '-v',str(ROOT)+':/capture','-v',str(REPO/'aichallenge')+':/aichallenge:ro','--entrypoint','bash',
            'codex-cartographer-v4-build:20260910','-lc',shell]
        # Allow the simulator to keep producing observed future after lap 1.
        # Collection stops at first verified lap +4 s, before another full lap.
        make_args = ['CONTROL_METHOD=v4_20_external','CAPTURE=false','ROSBAG=false','AWSIM_LAPS=2','AWSIM_VEHICLES=1',
            'RUN_ID='+args.run_id,'OUTPUT_HOST_ROOT='+str(output),'AWSIM_TIMEOUT=1920',
            'AWSIM_EXTRA_ARGS=-logFile /output/'+args.run_id+'/awsim_unity.log']
        result['commands'] = [probe_cmd,['make','dev','DEV_AUTO_START=false',*make_args]]
        owned = True; probe = launch(probe_cmd,'nodes'); make = launch(result['commands'][1],'make')
        while time.monotonic()-started < 1940:
            if probe.poll() is not None:
                raise RuntimeError('NODES_EARLY_EXIT')
            if make.poll() not in (None,0):
                raise RuntimeError('MAKE_FAILED')
            cp = output/'control_heartbeat.json'; unity = output/args.run_id/'awsim_unity.log'
            if cp.exists():
                control = json.loads(cp.read_text()); result['last_control'] = control
                if time.monotonic_ns()-control['monotonic_ns'] > 1_000_000_000:
                    raise RuntimeError('COLLECTOR_WATCHDOG')
                if control['fault']:
                    if fault_wall is None:
                        fault_wall = time.monotonic(); request_stop('COLLECTOR_FAULT')
                    if control['fault'] in ('COMPETING_CONTROLLER','COMMAND_AUTHORITY') or time.monotonic()-fault_wall > 10:
                        raise RuntimeError('COLLECTOR_'+control['fault'])
                if control['stop_confirmed']:
                    result['status'] = 'COMPLETE_LAP' if judge.completed and not control['fault'] and control['stop_reason']=='JUDGE_LAP_PLUS_FUTURE' else 'STOPPED_FAILED'
                    break
            if unity.exists():
                if unity.stat().st_size < read_offset:
                    raise RuntimeError('JUDGE_LOG_RESET')
                offset = read_offset-len(pending)
                with unity.open('rb') as stream:
                    stream.seek(read_offset); chunk=stream.read(256*1024); read_offset=stream.tell()
                lines=(pending+chunk).split(b'\n'); pending=lines.pop()
                for line in lines:
                    judge.feed(line.decode(errors='replace'),byte_offset=offset); offset += len(line)+1
                if judge.laps and not judge.completed:
                    request_stop('JUDGE_EVIDENCE_INCOMPLETE')
                if judge.completed and control.get('sim_ns') is not None:
                    if lap_seen_ns is None:
                        lap_seen_ns = control['sim_ns']
                    if control['sim_ns']-lap_seen_ns >= 4_000_000_000:
                        request_stop('JUDGE_LAP_PLUS_FUTURE')
            if time.monotonic()-last_size_check > 1:
                last_size_check = time.monotonic()
                size = sum(p.stat().st_size for p in (output/'bag').glob('*') if p.is_file())
                result['bag_bytes'] = size
                if (size >= 3*1024**3 or result['prior_task_bag_bytes']+size >= result['task_bag_byte_limit']
                        or shutil.disk_usage(ROOT).free < 10*1024**3):
                    request_stop('DISK_BOUND')
                p = output/'progress.pending'
                p.write_text(json.dumps(dict(result, judge_sections=judge.section_events, judge_laps=judge.laps),allow_nan=False))
                p.replace(output/'progress.json')
            if control.get('ready_ticks',0) >= 100 and make.poll() == 0 and not result['official_start_requested']:
                path_heartbeat = output/'path_heartbeat.json'
                display = json.loads(path_heartbeat.read_text()) if path_heartbeat.exists() else {}
                if (not display or time.monotonic_ns()-display['monotonic_ns'] > 2_000_000_000
                        or any(display.get('point_counts',{}).get(name,0) < minimum
                               for name,minimum in (('baseline',20),('reference',20),('observed',1)))):
                    if time.monotonic()-started > 120:
                        raise RuntimeError('RVIZ_PATH_PUBLICATION_NOT_READY')
                    time.sleep(.1); continue
                if not any(n.startswith('rviz') for n in control['rviz_subscribers']):
                    if time.monotonic()-started > 120:
                        raise RuntimeError('RVIZ_NOT_SUBSCRIBED')
                    time.sleep(.1); continue
                if not unity.exists() or not (output/'nodes_heartbeat.json').exists():
                    raise RuntimeError('RECORDER_OR_JUDGE_NOT_READY')
                inspections=[]
                for service in ('simulator','autoware'):
                    cid=run(compose+['ps','-q',service]).stdout.strip()
                    info=json.loads(run(['docker','inspect',cid]).stdout)[0]
                    if info['Config']['Labels'].get('com.docker.compose.project') != args.run_id:
                        raise RuntimeError('COMPOSE_OWNER_MISMATCH')
                    if any(m['Destination'] in ('/dev/vcu','/dev/gnss','/dev/ttyUSB0') for m in info['Mounts']):
                        raise RuntimeError('PHYSICAL_DEVICE_MOUNT')
                    inspections.append(info)
                (output/'inspect.json').write_text(json.dumps(inspections,indent=2))
                windows=[line for line in run(['xwininfo','-root','-tree']).stdout.splitlines() if 'autoware.rviz' in line]
                if len(windows)!=1:
                    raise RuntimeError('NORMAL_RVIZ_WINDOW_MISSING')
                rviz_window=windows[0].strip().split()[0]
                run(['xwd','-silent','-id',rviz_window,'-out',str(output/'normal_rviz.xwd')])
                if args.preflight_only:
                    result['status']='PREFLIGHT_PASSED'; break
                # Record loaded parameter values before granting drive authority.
                params=run(['docker','exec',args.run_id+'-nodes','bash','-lc',
                    'source /aichallenge/workspace/install/setup.bash && source /capture/cpp_install/setup.bash && ros2 param dump /recovery_teacher_pure_pursuit'],timeout=15)
                (output/'pure_pursuit_loaded.yaml').write_text(params.stdout)
                official=launch(['make','awsim-request-start',*make_args],'official_start')
                if official.wait(timeout=20):
                    raise RuntimeError('OFFICIAL_START_FAILED')
                result['official_start_requested']=True
                authorization = output/'drive_authorized.pending'
                authorization.write_text(json.dumps({'run_id':args.run_id,'scope':'MEASURED_RECOVERY_AWSIM',
                    'expires_monotonic_s':time.monotonic()+20,'source':'USER_REQUEST_20260913'}))
                authorization.replace(output/'drive_authorized.json')
            if time.monotonic()-started > 120 and not result['official_start_requested']:
                raise RuntimeError('STOPPED_PREFLIGHT_TIMEOUT:'+str(control.get('reason')))
            time.sleep(.1)
        else:
            raise RuntimeError('OUTER_WALL_LIMIT')
    except Exception as exc:
        result['error']=type(exc).__name__+': '+str(exc)
    finally:
        cleanup=[]
        if owned and compose is not None:
            # Never leave a simulation actuating after its final publisher exits.
            for service in ('simulator','autoware'):
                try:
                    cid=run(compose+['ps','-q',service],timeout=5).stdout.strip()
                    if cid:
                        if service=='simulator':
                            run(['docker','pause',cid],timeout=5,check=False)
                            if rviz_window:
                                run(['xwd','-silent','-id',rviz_window,'-out',str(output/'rviz_final.xwd')],check=False)
                            run(['docker','kill',cid],timeout=5,check=False)
                        else:
                            run(['docker','stop','-t','2',cid],timeout=5,check=False)
                except Exception as exc:
                    cleanup.append(str(exc))
            (output/'finish_nodes.json').write_text(json.dumps({'run_id':args.run_id}))
            try:
                probe.wait(timeout=45)
            except Exception as exc:
                cleanup.append(str(exc)); run(['docker','stop','-t','30',args.run_id+'-nodes'],timeout=35,check=False)
            run(compose+['down','--timeout','2'],timeout=12,check=False)
        for p in children:
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGTERM)
                try:
                    p.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid,signal.SIGKILL); p.wait(timeout=3)
        for stream in streams:
            stream.close()
        result.update(cleanup_errors=cleanup,judge_sections=judge.section_events,judge_laps=judge.laps,lap_confirmed=judge.completed,
                      ended_unix_s=time.time(),free_bytes=shutil.disk_usage(ROOT).free)
        nodes=output/'nodes_result.json'
        if nodes.exists():
            result['nodes']=json.loads(nodes.read_text())
        if result['status']=='COMPLETE_LAP' and not result.get('nodes',{}).get('closed_bag'):
            result['status']='FAILED_BAG_NOT_CLOSED'
        (output/'result.json').write_text(json.dumps(result,indent=2,allow_nan=False))
        # Only finalized files are hashed. This record is for subsequent transfer.
        manifest={str(p.relative_to(output)):dict(bytes=p.stat().st_size,sha256=sha(p))
                  for p in output.rglob('*') if p.is_file() and p.name!='transfer_manifest.json'}
        (output/'transfer_manifest.json').write_text(json.dumps(manifest,indent=2))
        print(json.dumps(result,indent=2,allow_nan=False))
    if result['status'] not in ('COMPLETE_LAP','PREFLIGHT_PASSED'):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
