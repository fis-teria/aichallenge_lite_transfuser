"""Owned 2 normal + 8 recovery laps; native WSL transfer and audit orchestration."""
from pathlib import Path
import argparse
import hashlib
import json
import subprocess
import sys
import time

REPO = next(p for p in Path(__file__).resolve().parents if (p/'AGENTS.md').is_file() and (p/'pyproject.toml').is_file())
HERE = Path(__file__).resolve().parent
HOST = 'graneple@192.168.3.10'
ROOT = '/home/graneple/e2e_autonomous/time_recovery_sites_20260915'
OUT = '/home/thistle/e2e_autonomous/runs/time_recovery_sites_20260915'
RAW = '/home/thistle/e2e_autonomous/raw/time_recovery_sites_20260915'
UNC = Path(r'\\wsl.localhost\Ubuntu-22.04-Recovered\home\thistle\e2e_autonomous\runs\time_recovery_sites_20260915')


def remote(code: str, *, native: bool = False, lock: bool = False, timeout: int = 120) -> str:
    if native:
        command = ('cd /home/thistle/e2e_autonomous/e2e_lite_transfuser && '
                   + ('tools/with_wsl_training_lock.sh ' if lock else '')
                   + 'env PYTHONPATH=src .venv/bin/python -')
        argv = ['wsl.exe', '-d', 'Ubuntu-22.04-Recovered', '-u', 'thistle', '--exec', 'bash', '-lc', command]
    else:
        argv = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10', HOST, 'python3 -']
    result = subprocess.run(argv, input=code.encode(), capture_output=True, timeout=timeout)
    for stream, output in ((sys.stdout, result.stdout), (sys.stderr, result.stderr)):
        if output:
            stream.write(output.decode('utf-8', errors='replace')); stream.flush()
    result.check_returncode()
    return result.stdout.decode()


def copy_to_host(files: list[Path], destination: str = ROOT) -> None:
    subprocess.run(['scp', *map(str, files), HOST+':'+destination+'/'], check=True, timeout=180)


def copy_to_wsl(names: list[str]) -> None:
    for name in names:
        destination = UNC/name
        if destination.exists():
            raise FileExistsError(destination)
        subprocess.run(['scp', HOST+':'+ROOT+'/'+name, str(destination)], check=True, timeout=1200)


def setup() -> None:
    head = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=REPO, text=True).strip()
    assert not subprocess.check_output(['git', 'status', '--porcelain'], cwd=REPO, text=True).strip()
    gate_name = f'time_site_recovery_gate_{head[:7]}.json'
    parent_unc = UNC.parent
    gate = json.loads((parent_unc/gate_name).read_bytes())
    assert gate['commit'] == head and gate['full_exit'] == 0
    log = parent_unc/Path(gate['log']).name
    assert hashlib.sha256(log.read_bytes()).hexdigest() == gate['log_sha256']
    archive = HERE/'source.tar.gz'
    assert not archive.exists()
    subprocess.run(['git', 'archive', '--format=tar.gz', '--prefix=source/', '--output='+str(archive),
                    head, 'src', 'tools', 'configs'], cwd=REPO, check=True)
    prep = dict(commit=head, archive_sha256=hashlib.sha256(archive.read_bytes()).hexdigest(), test_gate=gate,
                scope='STOP_SITE_PLUS_TEN_SEEDED_RANDOM_SITES', maximum_attempts=10, maximum_pulse_events=22)
    (HERE/'bootstrap.json').write_text(json.dumps(prep, indent=2)+'\n')
    remote(f"from pathlib import Path\np=Path({ROOT!r});assert p.resolve()==p and p.parent==Path('/home/graneple/e2e_autonomous');p.mkdir(exist_ok=False)\n")
    copy_to_host([archive, HERE/'bootstrap.json', REPO/'docs/evidence/time_recovery_batches_20260914/move_pair.py'])
    remote(rf'''
from pathlib import Path
import hashlib,json,shutil,subprocess,tarfile
root=Path({ROOT!r});base=root.parent/'time_recovery_random_confirmed_20260915'
prep=json.loads((root/'bootstrap.json').read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
assert not subprocess.check_output(['docker','ps','-q'],text=True).strip()
assert shutil.disk_usage(root).free>=12*2**30
deployment=json.loads((base/'deployment.json').read_text());files={{}}
for rel,digest in deployment['files'].items():
 if rel.startswith('source/'):continue
 src=base/rel;assert src.resolve().is_relative_to(base) and sha(src)==digest
 dst=root/rel;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst);files[rel]=digest
assert sha(root/'source.tar.gz')==prep['archive_sha256']
with tarfile.open(root/'source.tar.gz') as archive:
 for member in archive.getmembers():
  dst=root/member.name
  assert dst.resolve().is_relative_to(root/'source') and (member.isdir() or member.isfile())
  if member.isdir():dst.mkdir(parents=True,exist_ok=True)
  else:
   dst.parent.mkdir(parents=True,exist_ok=True)
   with dst.open('xb') as f:f.write(archive.extractfile(member).read())
   files[member.name]=sha(dst)
(root/'references').mkdir();(root/'planned_references').mkdir()
normal_base=root.parent/'time_recovery_speed_20260914'/'references'
for side in ('left','right'):
 ref=json.loads((normal_base/(side+'.json')).read_text())
 assert not ref.get('steering_pulse') and not ref['intervals'] and ref['signed_offset_m']==0
 assert ref['reference_xy_m']==ref['baseline_xy_m'] and sha(normal_base/(side+'.csv'))==ref['reference_sha256']
 for ext in ('.json','.csv'):shutil.copy2(normal_base/(side+ext),root/'references'/(side+ext))
 shutil.copy2(normal_base/(side+'.json'),root/'planned_references'/('normal_'+side+'.json'))
shutil.copy2(base/'speed_gate.json',root/'speed_gate.json')
(root/'deployed_commit.txt').write_text(prep['commit']+'\n')
(root/'test_gate.json').write_text(json.dumps(prep['test_gate'],indent=2))
(root/'deployment.json').write_text(json.dumps(dict(source_commit=prep['commit'],files=files),indent=2))
runs=[dict(run_id='codex-time-recovery-sites-normal-n01',side='left',kind='normal',pair=0,reference='normal_left.json'),
      dict(run_id='codex-time-recovery-sites-normal-n02',side='right',kind='normal',pair=0,reference='normal_right.json')]
(root/'campaign_20260914.json').write_text(json.dumps(dict(maximum_attempts=10,maximum_pulse_events=22,batch_size=2,attempts=[],planned_runs=runs,sealed=False),indent=2))
repo='/home/graneple/git/autononous_ai/aichallenge-racingkart'
(root/'host_before.json').write_text(json.dumps(dict(head=subprocess.check_output(['git','-C',repo,'rev-parse','HEAD'],text=True).strip(),
 git_status_sha256=hashlib.sha256(subprocess.check_output(['git','-C',repo,'status','--porcelain'])).hexdigest(),
 containers=[json.loads(r) for r in subprocess.check_output(['docker','ps','-a','--format','{{{{json .}}}}'],text=True).splitlines()],
 compose=json.loads(subprocess.check_output(['docker','compose','ls','--all','--format','json'],text=True)),free_bytes=shutil.disk_usage(root).free),indent=2))
print(json.dumps(dict(status='SETUP_OK',source_commit=prep['commit'],runtime_files=len(files))))
''')
    remote(f"from pathlib import Path\np=Path({OUT!r});p.mkdir(exist_ok=False);Path({RAW!r}).mkdir(exist_ok=False)\n", native=True, lock=True)
    copy_to_wsl(['bootstrap.json','deployment.json','host_before.json'])


def start(run_id: str) -> None:
    remote(rf'''
from pathlib import Path
import hashlib,json,os,shutil,subprocess,time
root=Path({ROOT!r});run_id={run_id!r}
ledger=json.loads((root/'campaign_20260914.json').read_text())
assert not ledger['sealed'] and len(ledger['attempts'])<ledger['maximum_attempts']==10
row=ledger['planned_runs'][len(ledger['attempts'])];assert row['run_id']==run_id
assert not (root/run_id).exists() and not subprocess.check_output(['docker','ps','-q'],text=True).strip()
assert len([r for r in ledger['attempts'] if r['state']!='WSL_MOVED'])<2 and shutil.disk_usage(root).free>=12*2**30
for previous in ledger['attempts']:
 p=root/previous['run_id'];r=json.loads((p/('MOVED_TO_WSL.json' if previous['state']=='WSL_MOVED' else 'result.json')).read_text())
 if previous['state']=='WSL_MOVED':r=r['result']
 assert r['status']=='COMPLETE_LAP' and not r['last_control']['fault'] and r['nodes']['closed_bag'] and not r['cleanup_errors']
deployment=json.loads((root/'deployment.json').read_text());gate=json.loads((root/'test_gate.json').read_text())
assert gate['commit']==deployment['source_commit']==(root/'deployed_commit.txt').read_text().strip() and gate['full_exit']==0
for rel,digest in deployment['files'].items():
 p=root/rel;assert p.resolve().is_relative_to(root) and hashlib.sha256(p.read_bytes()).hexdigest()==digest,rel
assert json.loads((root/'speed_gate.json').read_text())['all_within_005_mps']
ref=root/'planned_references'/row['reference'];ref_value=json.loads(ref.read_text())
assert hashlib.sha256((root/'references'/(row['side']+'.csv')).read_bytes()).hexdigest()==ref_value['reference_sha256']
if row['kind']=='recovery':assert hashlib.sha256(ref.read_bytes()).hexdigest()==row['reference_sha256']
shutil.copy2(ref,root/'references'/(row['side']+'.json'))
command=['timeout','--signal=TERM','--kill-after=20s','1980s','python3',str(root/'source/tools/run_time_recovery_awsim.py'),
 '--campaign-root',str(root),'--run-id',run_id,'--side',row['side'],'--speed-policy','aligned_gain4_v1','--separate-cpus']
attempt=dict(row,state='STARTING',started_unix_s=time.time(),reference_sha256=hashlib.sha256(ref.read_bytes()).hexdigest(),command=command)
ledger['attempts'].append(attempt);(root/'campaign_20260914.json').write_text(json.dumps(ledger,indent=2))
display=json.loads((root/'display.json').read_text())
env=dict(os.environ,PYTHONPATH=str(root/'source/src'),**display)
subprocess.run(['xdpyinfo'],env=env,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE,check=True,timeout=10)
with (root/(run_id+'_supervisor.log')).open('x') as stream:
 p=subprocess.Popen(command,env=env,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
attempt.update(state='STARTED',pid=p.pid);(root/'campaign_20260914.json').write_text(json.dumps(ledger,indent=2))
print(json.dumps(attempt))
''')


def status() -> None:
    remote(rf'''
from pathlib import Path
import json,shutil,subprocess
root=Path({ROOT!r});ledger=json.loads((root/'campaign_20260914.json').read_text());report=[]
for a in ledger['attempts']:
 p=root/a['run_id'];v=dict(run_id=a['run_id'],state=a['state'])
 if (p/'result.json').exists():
  r=json.loads((p/'result.json').read_text());c=r.get('last_control',{{}})
  v.update(status=r['status'],fault=c.get('fault'),stop_confirmed=c.get('stop_confirmed'),closed_bag=r.get('nodes',{{}}).get('closed_bag'),judge_laps=r.get('judge_laps'),result_error=r.get('error'),cleanup_errors=r.get('cleanup_errors'),transfer_ready=(p/'transfer_manifest.json').exists(),pulse=c.get('random_pulse',{{}}).get('state'))
 elif (p/'control_heartbeat.json').exists():
  c=json.loads((p/'control_heartbeat.json').read_text());v.update(phase=c.get('phase'),fault=c.get('fault'),progress_m=(c.get('projection') or {{}}).get('s_m'),speed_mps=c.get('speed_mps'),pulse=c.get('random_pulse',{{}}).get('state'))
 else:
  log=root/(a['run_id']+'_supervisor.log');v['startup_log']=log.read_text()[-2000:] if log.exists() else None
 report.append(v)
print(json.dumps(dict(runs=report,free_bytes=shutil.disk_usage(root).free,running_containers=subprocess.check_output(['docker','ps','--format','{{{{.Names}}}}'],text=True).splitlines())))
''')


def ship(pair: int) -> None:
    prefix=f'sites_pair{pair:02d}_20260915'
    receipt=json.loads(remote(rf'''
from pathlib import Path
import json,importlib.util
root=Path({ROOT!r});s=importlib.util.spec_from_file_location('move',str(root/'move_pair.py'));m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
ledger=json.loads((root/'campaign_20260914.json').read_text());names=[r['run_id'] for r in ledger['attempts'] if r['state']!='WSL_MOVED']
assert names and all(r['pair']=={pair} for r in ledger['attempts'] if r['run_id'] in names)
m.REMOTE=root;m.pack(root,{prefix!r},names)
''',timeout=600))
    copy_to_wsl([prefix+'.tar.gz',prefix+'_snapshot.json',prefix+'_shipping.json'])
    names=receipt['run_ids'];snapshot=receipt['snapshot_sha256']
    remote(rf'''
from pathlib import Path
import importlib.util
s=importlib.util.spec_from_file_location('move','docs/evidence/time_recovery_batches_20260914/move_pair.py');m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
m.ANALYSIS=Path({OUT!r});m.RAW=Path({RAW!r});m.verify(m.ANALYSIS,{prefix!r},{names!r},{snapshot!r})
''',native=True,lock=True,timeout=600)
    copy_to_host([UNC/(prefix+'_verified.json')])
    remote(rf'''
from pathlib import Path
import json,subprocess,shutil
root=Path({ROOT!r});raw=Path({RAW!r});prefix={prefix!r};names={names!r};snapshot={snapshot!r}
assert root.resolve()==root and root.parent==Path('/home/graneple/e2e_autonomous')
assert not subprocess.check_output(['docker','ps','-q'],text=True).strip()
v=json.loads((root/(prefix+'_verified.json')).read_bytes())
assert v['snapshot_sha256']==snapshot and v['all_files_and_directory_structure_identical'] and v['all_sqlite_quick_checks_passed']
ledger=json.loads((root/'campaign_20260914.json').read_bytes())
assert sorted(r['run_id'] for r in ledger['attempts'] if r['state']!='WSL_MOVED')==sorted(names)
for name in names:
 marker=root/(prefix+'_markers')/name;marker.mkdir(parents=True,exist_ok=False)
 result=json.loads((root/name/'result.json').read_bytes())
 (marker/'MOVED_TO_WSL.json').write_text(json.dumps(dict(snapshot_sha256=snapshot,raw_path=str(raw/name),result=result),indent=2)+'\n')
with (root/(prefix+'_cleanup.json')).open('x') as f:json.dump(dict(state='PREPARED',removed_runs=[],before_free_bytes=shutil.disk_usage(root).free),f,indent=2)
code="from pathlib import Path;import importlib.util;s=importlib.util.spec_from_file_location('m','/collection/move_pair.py');m=importlib.util.module_from_spec(s);s.loader.exec_module(m);m.RAW=Path("+repr(str(raw))+");m.cleanup(Path('/collection'),"+repr(prefix)+","+repr(names)+","+repr(snapshot)+")"
subprocess.run(['docker','run','--rm','--network','none','--entrypoint','python3','-v',str(root)+':/collection','codex-cartographer-v4-build:20260910','-c',code],check=True,timeout=600)
for row in ledger['attempts']:
 if row['run_id'] in names:row.update(state='WSL_MOVED',raw_path=str(raw/row['run_id']))
ledger['sealed']=len(ledger['attempts'])==ledger['maximum_attempts'] and all(r['state']=='WSL_MOVED' for r in ledger['attempts'])
(root/'campaign_20260914.json').write_text(json.dumps(ledger,indent=2)+'\n')
print('PAIR_MOVED',names)
''',timeout=700)
    copy_to_wsl([prefix+'_cleanup.json'])


def collect(pair: int) -> None:
    info = json.loads(remote(rf'''
from pathlib import Path
import json
root=Path({ROOT!r});ledger=json.loads((root/'campaign_20260914.json').read_text())
print(json.dumps(dict(planned=[r for r in ledger['planned_runs'] if r['pair']=={pair}],attempted=[r['run_id'] for r in ledger['attempts']])))
'''))
    assert len(info['planned']) == 2
    for row in info['planned']:
        name=row['run_id']
        if name not in info['attempted']:
            start(name)
        deadline=time.monotonic()+2030
        while time.monotonic()<deadline:
            status_value=json.loads(remote(rf'''
from pathlib import Path
import json
p=Path({ROOT!r})/{name!r}
if (p/'result.json').exists() and (p/'transfer_manifest.json').exists():
 r=json.loads((p/'result.json').read_text());c=r.get('last_control',{{}})
 print(json.dumps(dict(run_id=p.name,complete=True,status=r['status'],fault=c.get('fault'),stopped=c.get('stop_confirmed'),closed_bag=r.get('nodes',{{}}).get('closed_bag'),cleanup_errors=r.get('cleanup_errors'))))
else:
 c=json.loads((p/'control_heartbeat.json').read_text()) if (p/'control_heartbeat.json').exists() else {{}}
 st=c.get('random_pulse',{{}}).get('state',{{}})
 print(json.dumps(dict(run_id=p.name,complete=False,progress_m=(c.get('projection') or {{}}).get('s_m'),fault=c.get('fault'),phase=c.get('phase'),events=st.get('event_id'),recovered=st.get('completed_events'),skipped=st.get('skipped_sites'))))
'''))
            if status_value['complete']:
                if (status_value['status']!='COMPLETE_LAP' or status_value['fault'] or not status_value['stopped']
                        or not status_value['closed_bag'] or status_value['cleanup_errors']):
                    raise RuntimeError('RUN_FAILED_PRESERVE_FOR_DIAGNOSIS:'+name)
                break
            time.sleep(30)
        else:
            raise TimeoutError(name)
    ship(pair)
    remote("import sys\nsys.argv=['audit_native.py','--pair',"+repr(str(pair))+"]\n"+(HERE/'audit_native.py').read_text(),
           native=True,lock=True,timeout=2400)
    if pair==0:
        remote((HERE/'select_native.py').read_text(),native=True,lock=True,timeout=240)
        copy_to_host([UNC/'selected_site_plan.json'])
        copy_to_host(sorted((UNC/'planned_references').glob('*.json')),ROOT+'/planned_references')
        remote(rf'''
from pathlib import Path
import hashlib,json
root=Path({ROOT!r});ledger=json.loads((root/'campaign_20260914.json').read_text());plan=json.loads((root/'selected_site_plan.json').read_text())
assert len(ledger['attempts'])==2 and all(r['state']=='WSL_MOVED' for r in ledger['attempts'])
assert plan['maximum_pulse_events']==22 and len(plan['runs'])==8
assert plan['normal_runs']==[r['run_id'] for r in ledger['attempts']]
assert len([s for s in [plan['stop_site'],*plan['additional_sites']] if s['site_id']=='S00'])==1
for row in plan['runs']:
 p=root/'planned_references'/row['reference'];assert hashlib.sha256(p.read_bytes()).hexdigest()==row['reference_sha256']
ledger['planned_runs']+=plan['runs'];ledger['selection_sha256']=hashlib.sha256((root/'selected_site_plan.json').read_bytes()).hexdigest()
(root/'campaign_20260914.json').write_text(json.dumps(ledger,indent=2)+'\n')
print('SELECTED_PLAN_DEPLOYED',[(r['run_id'],len(r['sites'])) for r in plan['runs']])
''')
    print('PAIR_COLLECTION_TRANSFER_AND_AUDIT_DONE',pair,flush=True)


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('mode',choices=('setup','start','status','ship','collect'));ap.add_argument('--run-id');ap.add_argument('--pair',type=int)
    args=ap.parse_args()
    if args.mode=='setup':setup()
    elif args.mode=='start':start(args.run_id)
    elif args.mode=='ship':ship(args.pair)
    elif args.mode=='collect':collect(args.pair)
    else:status()
