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
ROOT = '/home/graneple/e2e_autonomous/time_corner_model_lap_20260916'
WSL = '/home/thistle/e2e_autonomous/runs/time_corner_model_lap_20260916'
CHECKPOINT = '/home/thistle/e2e_autonomous/runs/time_corner_retraining_20260916/training_serial/best.pt'
SHA = json.loads((HERE/'runtime_state.json').read_text())['checkpoint_sha256']
CONFIG = 'configs/control/time_path_corner_lap_20260916.json'
RUN_ID = 'codex-time-corner-lap01'


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
    if stage == 'test':
        remote(f"""
from pathlib import Path
import json,hashlib,subprocess,time,sys
from aic_transfuser_lite.control.time_trial_v1 import validate_trial_config
root=Path({WSL!r});root.mkdir(exist_ok=False)
head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()
assert head=={head!r}
config=json.loads(Path({CONFIG!r}).read_text())
assert validate_trial_config(config)=='fixed_5kmh'
old=json.loads(Path('configs/control/time_path_lap_candidate_20260914.json').read_text())
assert [k for k in old.keys()|config.keys() if old.get(k)!=config.get(k)]==['checkpoint_sha256']
assert hashlib.sha256(Path({CHECKPOINT!r}).read_bytes()).hexdigest()=={SHA!r}
begin=time.monotonic()
with (root/'pytest.log').open('x') as log:
 p=subprocess.run([sys.executable,'-m','pytest','-q'],stdout=log,stderr=subprocess.STDOUT,timeout=300)
receipt=dict(source_commit=head,exit=p.returncode,seconds=time.monotonic()-begin,config_sha256=hashlib.sha256(Path({CONFIG!r}).read_bytes()).hexdigest(),checkpoint_sha256={SHA!r})
(root/'test_gate.json').write_text(json.dumps(receipt,indent=2))
print(json.dumps(receipt))
print((root/'pytest.log').read_text().splitlines()[-4:])
assert p.returncode==0
""", host='codex-wsl', lock=True, timeout=360)
    elif stage == 'package':
        assert not subprocess.check_output(['git','status','--porcelain'],cwd=REPO).strip()
        gate = json.loads(remote(f"import json;from pathlib import Path;print((Path({WSL!r})/'deployment_gate.json').read_text())",host='codex-wsl'))
        assert gate['exit'] == 0 and gate['source_commit'] == head
        source = 'source_' + head[:7]
        archive = HERE / (source + '.tar')
        assert not archive.exists()
        paths = ['src','tools','configs','schemas','ros2_ws','tests/fixtures']
        run(['git','archive','--format=tar','--prefix='+source+'/','--output='+str(archive),head,*paths])
        tree = subprocess.check_output(['git','ls-tree','-r',head,'--',*paths],cwd=REPO,text=True)
        entries=[]
        for row in tree.splitlines():
            meta,path=row.split('\t',1);mode,kind,blob=meta.split()
            assert kind=='blob' and mode in ('100644','100755'),path
            entries.append(dict(path=path,git_blob=blob))
        manifest=dict(source_commit=head,source_dir=source,archive_file=archive.name,
                      archive_sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),
                      config=CONFIG,checkpoint_sha256=SHA,entries=entries)
        (HERE/'source_manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
        template=(REPO/'tmp/time_normal_lap_20260914/prepare_remote.py').read_text(encoding='utf-8')
        template=template.replace('/home/graneple/e2e_autonomous/time_normal_lap_20260914',ROOT)
        (HERE/'prepare_remote.py').write_text(template,encoding='utf-8',newline='\n')
        print(json.dumps(dict(status='PACKAGED',source_commit=head,files=len(entries),archive_bytes=archive.stat().st_size)))
    elif stage == 'prepare':
        manifest=json.loads((HERE/'source_manifest.json').read_text(encoding='utf-8'))
        changed=subprocess.check_output(['git','diff','--name-only',manifest['source_commit'],head,'--','src','tools','configs','schemas','ros2_ws','tests/fixtures'],cwd=REPO).strip()
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
command=['timeout','--signal=TERM','--kill-after=10s','710s','python3',str(root/manifest['source_dir']/'tools/run_time_path_awsim_trial.py'),'--deployment',str(root),'--run-id',{RUN_ID!r},'--display',':0','--config',{CONFIG!r}]
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
