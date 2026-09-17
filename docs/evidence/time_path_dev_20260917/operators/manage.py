"""One-task orchestration; existing runtime tools own all simulation behavior."""
from pathlib import Path
import hashlib
import json
import shlex
import subprocess
import sys

REPO = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
HOST = 'graneple@192.168.3.10'
ROOT = '/home/graneple/e2e_autonomous/time_path_dev_20260917'
WSL = '/home/thistle/e2e_autonomous/runs/time_path_dev_20260917'
CHECKPOINT = '/home/thistle/e2e_autonomous/runs/time_launch_protection_v2_20260916/launch_balanced/epoch_03.pt'
SHA = json.loads((HERE/'runtime_state.json').read_text())['checkpoint_sha256']
CONFIG = 'configs/control/time_path_dev.json'
RUN_ID = 'codex-time-dev-lap01'


def remote(code: str, host: str = HOST, lock: bool = False, timeout: int = 60) -> str:
    command = 'python3 -'
    if lock:
        command = ('cd /home/thistle/e2e_autonomous/e2e_lite_transfuser && '
                   'tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -')
    transport = (['wsl', '-d', 'Ubuntu-22.04-Recovered', '-u', 'thistle', '--exec', 'bash', '-c', command] if host == 'codex-wsl' else ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10', host, command])
    p = subprocess.run(transport,
                       input=code, text=True, encoding='utf-8', capture_output=True, timeout=timeout)
    if p.stdout:
        print(p.stdout, end='', flush=True)
    if p.stderr:
        print(p.stderr, file=sys.stderr, end='', flush=True)
    p.check_returncode()
    return p.stdout


def run(args: list[str], timeout: int = 120) -> None:
    subprocess.run(args, cwd=REPO, check=True, timeout=timeout)


if __name__ == '__main__':
    stage = sys.argv[1]
    head = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=REPO, text=True).strip()
    if stage == 'package':
        assert not subprocess.check_output(['git','status','--porcelain'],cwd=REPO).strip()
        gate = json.loads(remote(f"import json;from pathlib import Path;print((Path({WSL!r})/'deployment_gate.json').read_text())",host='codex-wsl'))
        assert gate['exit'] == 0 and gate['source_commit'] == head
        source = 'source_' + head[:7]
        archive = HERE / (source + '.tar')
        assert not archive.exists()
        paths = ['src','tools','configs','schemas','ros2_ws','tests/fixtures','Makefile','docs/time_path_make_dev.md']
        run(['git','archive','--format=tar','--prefix='+source+'/','--output='+str(archive),head,*paths])
        tree = subprocess.check_output(['git','ls-tree','-r',head,'--',*paths],cwd=REPO,text=True)
        entries=[]
        for row in tree.splitlines():
            meta,path=row.split('\t',1);mode,kind,blob=meta.split()
            assert kind=='blob' and mode in ('100644','100755'),path
            entries.append(dict(path=path,git_blob=blob))
        manifest=dict(source_commit=head,source_dir=source,archive_file=archive.name,
                      archive_sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),
                      config=CONFIG,checkpoint_sha256=SHA,entries=entries,
                      candidate_id='launch_balanced_epoch3',user_requested_single_exploratory_trial=True,automatic_promotion=False)
        (HERE/'source_manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
        print(json.dumps(dict(status='PACKAGED',source_commit=head,files=len(entries),archive_bytes=archive.stat().st_size)))
    elif stage == 'prepare':
        manifest=json.loads((HERE/'source_manifest.json').read_text(encoding='utf-8'))
        changed=subprocess.check_output(['git','diff','--name-only',manifest['source_commit'],head,'--','src','tools','configs','schemas','ros2_ws','tests/fixtures','Makefile','docs/time_path_make_dev.md'],cwd=REPO).strip()
        assert not changed,changed
        archive=HERE/manifest['archive_file']
        assert hashlib.sha256(archive.read_bytes()).hexdigest()==manifest['archive_sha256']
        remote(f"""
from pathlib import Path
import subprocess,shutil
root=Path({ROOT!r})
assert not subprocess.check_output(['docker','ps','-q']).strip()
assert not root.exists()
assert shutil.disk_usage(root.parent).free>5_000_000_000
root.mkdir()
print('CREATED '+str(root))
""")
        run(['scp',str(archive),str(HERE/'source_manifest.json'),str(HERE/'prepare_remote.py'),HOST+':'+ROOT+'/'])
        run(['scp', str(Path(r'\\wsl.localhost\Ubuntu-22.04-Recovered')/CHECKPOINT.lstrip('/')),HOST+':'+ROOT+'/command_off_best.pt'],timeout=180)
        remote(f"import subprocess;subprocess.run(['python3',{ROOT!r}+'/prepare_remote.py'],check=True,timeout=330)",timeout=350)
    elif stage == 'start':
        remote(f"""
from pathlib import Path
import json,subprocess,time
root=Path({ROOT!r});manifest=json.loads((root/'source_manifest.json').read_text())
assert json.loads((root/'smoke/summary.json').read_text())['status']=='PASS'
assert not subprocess.check_output(['docker','ps','-q']).strip()
assert not (root/{RUN_ID!r}).exists()
command=['make','-C',str(root),'dev','MAX_SPEED_KMH=20','CORNER_MAX_SPEED_KMH=10','TIME_RECORD_VIDEO=1','TIME_RUN_ID='+{RUN_ID!r},'DISPLAY=:0']
wrapper='import subprocess,json,time;from pathlib import Path;start=time.time();p=subprocess.run('+repr(command)+');Path('+repr(str(root/'runner_exit.json'))+').write_text(json.dumps(dict(exit=p.returncode,start=start,end=time.time())));raise SystemExit(p.returncode)'
with (root/'runner.log').open('x') as log:
 p=subprocess.Popen(['python3','-c',wrapper],stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
record=dict(pid=p.pid,started_unix=time.time(),command=command,run_id={RUN_ID!r})
with (root/'runner_start.json').open('x') as f:json.dump(record,f,indent=2)
print(json.dumps(record))
""")
    elif stage == 'status':
        remote(f"""
from pathlib import Path
import json,subprocess
root=Path({ROOT!r});run=root/{RUN_ID!r}
for name in ['runner_start.json','runner_exit.json']:
 p=root/name
 if p.exists():print(name,p.read_text())
for name in ['control_heartbeat.json','inference_heartbeat.json','lap_progress.json','host_result.json']:
 p=run/name
 if p.exists():
  data=json.loads(p.read_text())
  if name=='host_result.json':data={{k:data.get(k) for k in ['status','error','wall_s','judge_lap_confirmed','judge_section_events','judge_laps','cleanup_errors','rviz_path_subscribers']}}
  print(name,json.dumps(data))
print('RUNNING_CONTAINERS',subprocess.check_output(['docker','ps','--format','{{{{.Names}}}}'],text=True).strip())
""")
    else:
        raise ValueError(stage)
