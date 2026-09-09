"""One bounded PP 20 km/h gain correction trial; exact-project shutdown."""
import json, os, signal, subprocess, sys, threading, time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from spatial_dev_host_v4 import AttemptBudget, atomic_json

root=Path(__file__).resolve().parent
out=root/'evidence';out.mkdir()
repo=Path('/home/graneple/git/autononous_ai/aichallenge-racingkart')
project='codex-v4-pp-shadow-14'
config_target='/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/config/config.yaml'
override=root/'overlay.json'
override.write_text(json.dumps({'services':{'autoware':{'volumes':[
    str(root)+':/v4',
    '/home/graneple/e2e_autonomous/spatial_sim_e2e_20260906/fixed_final.pt:/home/thistle/e2e_autonomous/runs/spatial_diagnostic_v4_20260906_f33b197/checkpoints/final.pt:ro']}}}))
env=dict(os.environ,DISPLAY=':0',XAUTHORITY='/run/user/1000/.mutter-Xwaylandauth.H56RU3',
         COMPOSE_PROJECT_NAME=project,COMPOSE_FILE=str(repo/'docker-compose.yml')+':'+str(repo/'docker-compose.gpu.yml')+':'+str(override),CONTROL_METHOD='pure_pursuit',V4_SHADOW_ENABLED='true',
         V4_SHADOW_SETUP='/v4/install/setup.bash',
         V4_SHADOW_LAUNCH='/v4/install/share/aic_e2e_runtime/launch/v4_shadow.launch.py',
         V4_SHADOW_CONFIG='/v4/live.json')
compose=['docker','compose','-p',project]
def run(cmd,timeout=8):
    return subprocess.run(cmd,cwd=repo,env=env,timeout=timeout,capture_output=True,text=True,check=True)
before=run(['docker','ps','-a','--format','{{.Names}} {{.Status}}']).stdout
(out/'inventory_before.log').write_text(before)
if run(['docker','ps','-q']).stdout.strip() or project in before: raise RuntimeError('NOT_QUIESCENT')
# User explicitly authorized one additional bounded run. Preserve all earlier usage.
prior=AttemptBudget(Path('/home/graneple/e2e_autonomous/pp_speed20_lap_13/budget.json'))
assert prior.value['active'] is None
reservation=dict(wall_s=120.,forward=40,mpc=0,snapshots=0,powered=1,powered_s=90.,log_bytes=16*1024**2)
new_budget=root/'budget.json'
assert not new_budget.exists()
renewal=dict(id=project,source='User requested V4 shadow AWSIM test; wall120s, post-Start90sim s, forward40; PP alone controls, MPC OFF',unix_s=time.time(),
             prior_used={k:v+prior.value['authorization']['prior_used'].get(k,0) for k,v in prior.value['used'].items()},prior_budget=str(prior.path),new_budget=str(new_budget),limits=reservation,
             old_history_preserved=True)
prior.value.setdefault('renewals',[]).append(renewal)
atomic_json(prior.path,prior.value)
atomic_json(new_budget,dict(version='V4_FINITE_RENEWAL_V1',used={k:0 for k in reservation},
                           active=None,attempts=[],authorization=renewal))
prior.close()
budget=AttemptBudget(new_budget)
budget.limits=reservation
budget.reserve(project,reservation)
live=json.loads((root/'live.template.json').read_text())
live['envelope'].update(session_id=project,authorized_until_unix_s=time.time()+300)
(root/'live.json').write_text(json.dumps(live,indent=2))
start=time.monotonic();done=threading.Event();lock=threading.Lock()
result={'source_commit':(root/'source_commit.txt').read_text().strip(),'status':'STARTING','cleanup':[],
        'start_requested':False,'v4_control_publish':False,'mpc_accounting':'CONSERVATIVE_RESERVED_NOT_MEASURED'}
def stop():
    with lock:
        for service in ('simulator','autoware'):
            try:
                cid=run(compose+['ps','-q',service]).stdout.strip()
                if cid:
                    if service=='simulator':
                        run(['docker','pause',cid],4);run(['docker','kill','--signal','KILL',cid],4)
                    else: run(['docker','stop','-t','3',cid],6)
            except Exception as exc: result['cleanup'].append(str(exc))
def watch():
    while not done.wait(.05):
        reason=None
        if time.monotonic()-start>105: reason='HOST_WALL_LIMIT'
        p=out/'guard.json'
        if p.exists():
            try:
                s=json.loads(p.read_text())
                if s.get('finish_seen'): reason='SIMULATOR_FINISH'
                if s['fault']: reason=s['fault']
                elif time.monotonic()-s['monotonic']>1: reason='OBSERVER_STALE'
                if not result.get('start_requested'):
                    stdout=out/'make.stdout.log'
                    if stdout.exists() and 'publish exactly one /admin/awsim/start=true pulse' in stdout.read_text():
                        result.update(start_requested=True,start_sim_s=s.get('sim_s',0),start_monotonic=time.monotonic())
                if result.get('start_requested'):
                    if s.get('sim_s',0)-result['start_sim_s']>=88: reason='DRIVE_TIME_LIMIT'
                    if time.monotonic()-result['start_monotonic']>=95: reason='START_WALL_LIMIT'
                    if (out/'v4.done').exists(): reason='V4_ENDED'
                # Preserve the existing failure-stop rule used manually in run 09.
                log=out/'pp_reference_lap_12/d1/autoware.log'
                if log.exists():
                    tail=log.read_text(errors='replace')[-32000:]
                    import re
                    if 'MPC_REJECTED:' in tail: reason='UNEXPECTED_MPC_PROCESS'
                    if 'Collision detected!' in tail: reason='COLLISION_REPORTED'
            except Exception: reason='OBSERVER_READ_ERROR'
        if reason:
            result['stop_reason']=reason;stop();done.set();return
threading.Thread(target=watch,daemon=True).start()
processes=[];streams=[]

def interrupted(signum, frame):
    raise RuntimeError('OUTER_SIGNAL_'+str(signum))

signal.signal(signal.SIGTERM,interrupted)
signal.signal(signal.SIGINT,interrupted)
def start_logged(cmd,label):
    stdout=(out/(label+'.stdout.log')).open('w');stderr=(out/(label+'.stderr.log')).open('w')
    streams.extend([stdout,stderr])
    p=subprocess.Popen(cmd,cwd=repo,env=env,stdout=stdout,stderr=stderr,start_new_session=True)
    processes.append(p);return p
try:
    # Read-only observer is already alive before normal off-mode can move.
    observer_name=project+'-observer'
    observe_cmd=['docker','run','--rm','--name',observer_name,'--network','host',
        '--entrypoint','bash','-v',str(root)+':/v4','-v',str(repo/'aichallenge')+':/aichallenge',
        '-v',str(repo/'vehicle/cyclonedds.xml')+':/opt/autoware/cyclonedds.xml:ro',
        'aichallenge-2025-dev','-lc',
        'source /aichallenge/workspace/install/setup.bash && export ROS_DOMAIN_ID=1 && python3 /v4/guard.py']
    observer=start_logged(observe_cmd,'observer')
    until=time.monotonic()+10
    while time.monotonic()<until:
        if observer.poll() is not None: raise RuntimeError('OBSERVER_START_FAILED')
        if (out/'guard.json').exists(): break
        time.sleep(.05)
    else: raise RuntimeError('OBSERVER_START_DEADLINE')
    result.update(start_requested=False,start_sim_s=0.,start_monotonic=time.monotonic(),
                  start_basis='START_PULSE_LOG_OBSERVATION')
    (out/'armed.json').write_text(json.dumps(dict(project=project,start_sim_s=0.)))
    cmd=['make','dev','CONTROL_METHOD=pure_pursuit','CAPTURE=false','ROSBAG=false','AWSIM_LAPS=1',
         'RUN_ID=v4_pp_shadow_14','OUTPUT_HOST_ROOT='+str(out)]
    result['command']=cmd
    make_proc=start_logged(cmd,'make')
    while make_proc.poll() is None and not done.is_set():
        time.sleep(.05)
    if done.is_set():
        result['make_exit']=make_proc.poll()
        result['status']='MONITOR_STOP_DURING_START'
        raise RuntimeError('BOUNDED_MONITOR_STOP_DURING_MAKE')
    result['make_exit']=make_proc.returncode
    if make_proc.returncode: raise RuntimeError('MAKE_FAILED')
    result['normal_start_helper_completed']=True
    cid=run(compose+['ps','-q','autoware']).stdout.strip()
    prefix=['docker','exec',cid,'bash','-lc']
    inspected=[]
    for service in ('autoware','simulator'):
        owned_id=run(compose+['ps','-q',service]).stdout.strip()
        if done.is_set() or not owned_id: break
        info=json.loads(run(['docker','inspect',owned_id]).stdout)[0]
        if done.is_set(): break
        if info['Config']['Labels'].get('com.docker.compose.project')!=project:
            raise RuntimeError('INSTANCE_IDENTITY')
        mounts=[dict(source=m['Source'],destination=m['Destination']) for m in info['Mounts']]
        if any(m['source'] in ('/dev/vcu','/dev/gnss') for m in mounts):
            raise RuntimeError('PHYSICAL_DEVICE_MOUNT')
        inspected.append(dict(service=service,id=owned_id,network=info['HostConfig']['NetworkMode'],
            privileged=info['HostConfig']['Privileged'],devices=info['HostConfig'].get('Devices'),mounts=mounts))
    (out/'instance_inspect.json').write_text(json.dumps(inspected,indent=2))
    if not done.is_set(): run(prefix+['test ! -e /dev/vcu && test ! -e /dev/gnss && grep -q \'name="lo"\' /opt/autoware/cyclonedds.xml'])

    while not done.is_set():
        if observer.poll() is not None: raise RuntimeError('OBSERVER_EXIT')
        path=out/'shadow.jsonl'
        if path.exists():
            for line in path.read_text().splitlines():
                try: record=json.loads(line)
                except ValueError: continue
                if record.get('event')=='SESSION_END':
                    result['v4_end']=record
                    (out/'v4.done').write_text(str(record))
        time.sleep(.05)
    result['status']='ATTEMPT_FINISHED'
except Exception as exc:
    result['status']='FAILED';result['error']=str(exc)
finally:
    stop();done.set()
    for p in processes:
        if p.poll() is None:
            os.killpg(p.pid,signal.SIGTERM)
            try: p.wait(timeout=2)
            except subprocess.TimeoutExpired: os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=2)
    try:
        if run(['docker','ps','-q','--filter','name='+project+'-observer']).stdout.strip():
            run(['docker','stop','-t','1',project+'-observer'],3)
    except Exception as exc: result['observer_cleanup_note']=str(exc)
    for f in streams: f.close()
    try:
        run(compose+['down','--timeout','2'],10)
        result['remaining_owned']=run(['docker','ps','-a','--filter','label=com.docker.compose.project='+project,'--format','{{.ID}} {{.Status}}']).stdout
        (out/'projects_after.log').write_text(run(['docker','compose','ls','--all']).stdout)
    except Exception as exc: result['cleanup'].append(str(exc))
    result['wall_s']=time.monotonic()-start
    result['log_bytes']=sum(p.stat().st_size for p in out.rglob('*') if p.is_file())
    budget.finish(dict(reservation,wall_s=result['wall_s'],log_bytes=result['log_bytes']),exact=False)
    budget.close();(out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
