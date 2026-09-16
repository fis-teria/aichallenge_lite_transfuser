"""Bounded all-corner collection: existing AWSIM, two private instances."""
from pathlib import Path
import argparse
from functools import partial
import hashlib
import json
import subprocess
import sys

REPO=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(REPO/'tmp/time_parallel_multiscale_20260916'))
import ops as parallel
remote=parallel.remote
transport=parallel.previous.previous.transport
ROOT='/home/graneple/e2e_autonomous/time_recovery_corner60_20260916'
OUT='/home/thistle/e2e_autonomous/runs/time_corner_multiscale60_20260916'
RAW='/home/thistle/e2e_autonomous/raw/time_corner_multiscale60_20260916'
UNC=Path(r'\\wsl.localhost\Ubuntu-22.04-Recovered')/OUT.lstrip('/')
PREFIX='ROOT='+repr(ROOT)+'\n'
# Reuse only the proven namespaced network/status/transfer functions. Each
# invocation has its own process and explicit new campaign root.
parallel.ROOT=ROOT;parallel.PREFIX=PREFIX


def setup():
    head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO,text=True).strip()
    assert not subprocess.check_output(['git','status','--porcelain'],cwd=REPO,text=True).strip()
    gate=json.loads((UNC/'test_gate.json').read_text())
    assert gate['commit']==head and gate['full_exit']==0
    assert hashlib.sha256((UNC/Path(gate['log']).name).read_bytes()).hexdigest()==gate['log_sha256']
    archive=HERE/'source.tar.gz'
    subprocess.run(['git','archive','--format=tar.gz','--prefix=source/','--output='+str(archive),head,'src','tools','configs'],check=True,cwd=REPO)
    boot=dict(commit=head,archive_sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),test_gate=gate)
    (HERE/'bootstrap.json').write_text(json.dumps(boot,indent=2))
    remote(PREFIX+"from pathlib import Path;import subprocess,shutil\nr=Path(ROOT);assert r.resolve()==r and r.parent==Path('/home/graneple/e2e_autonomous');assert not subprocess.check_output(['docker','ps','-q'],text=True).strip();assert shutil.disk_usage(r.parent).free>=12*2**30;r.mkdir(exist_ok=False)")
    transport.copy_to_host([archive,HERE/'bootstrap.json',REPO/'docs/evidence/time_recovery_batches_20260914/move_pair.py'],destination=ROOT)
    remote(PREFIX+r'''
from pathlib import Path
import hashlib,json,subprocess,shutil,tarfile,sys,os
root=Path(ROOT);base=root.parent/'time_recovery_parallel_20260916'
repo=Path('/home/graneple/git/autononous_ai/aichallenge-racingkart')
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
boot=json.loads((root/'bootstrap.json').read_text());files={}
assert sha(root/'source.tar.gz')==boot['archive_sha256']
with tarfile.open(root/'source.tar.gz') as tf:
 for m in tf.getmembers():
  p=root/m.name
  assert p.resolve().is_relative_to(root/'source') and (m.isfile() or m.isdir())
  if m.isdir():p.mkdir(parents=True,exist_ok=True)
  else:
   p.parent.mkdir(parents=True,exist_ok=True)
   with p.open('xb') as f:f.write(tf.extractfile(m).read())
   if m.mode&0o111:p.chmod(0o755)
   files[m.name]=sha(p)
dep=json.loads((base/'deployment.json').read_text())
for rel,digest in dep['files'].items():
 if not rel.startswith(('inputs/','cpp_install/')):continue
 assert sha(base/rel)==digest
 p=root/rel;p.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(base/rel,p);files[rel]=digest
for name in ('display.json','speed_gate.json'):shutil.copy2(base/name,root/name)
display=json.loads((root/'display.json').read_text())
subprocess.run(['xdpyinfo'],env=dict(os.environ,**display),check=True,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
sys.path[:0]=[str(root/'source/src'),str(root/'source/tools')]
from run_time_recovery_awsim import collection_dds_profile
(root/'probe_cyclonedds.xml').write_text(collection_dds_profile((repo/'vehicle/cyclonedds.xml').read_text(),large_mode=True))
(root/'references').mkdir();(root/'planned_references').mkdir()
(root/'deployed_commit.txt').write_text(boot['commit']+'\n')
(root/'test_gate.json').write_text(json.dumps(boot['test_gate'],indent=2))
(root/'deployment.json').write_text(json.dumps(dict(source_commit=boot['commit'],files=files),indent=2))
(root/'campaign_20260914.json').write_text(json.dumps(dict(maximum_attempts=12,maximum_planned_runs=12,batch_size=2,attempts=[],planned_runs=[],sealed=False),indent=2))
sim=repo/'aichallenge/simulator/AWSIM'
fingerprint={p.relative_to(sim).as_posix():dict(bytes=p.stat().st_size,sha256=sha(p)) for p in sorted(sim.rglob('*')) if p.is_file()}
(root/'host_before.json').write_text(json.dumps(dict(head=subprocess.check_output(['git','-C',str(repo),'rev-parse','HEAD'],text=True).strip(),git_status_sha256=hashlib.sha256(subprocess.check_output(['git','-C',str(repo),'status','--porcelain'])).hexdigest(),awsim_files=fingerprint),indent=2))
print(json.dumps(dict(status='CORNER_SETUP_READY',source=boot['commit'],awsim_files=len(fingerprint),free_gib=shutil.disk_usage(root).free/2**30)))
''',timeout=180)


def prepare_pair(pair, left, right):
    # References were generated/map-screened in locked native WSL.
    for slot,folder in [('left',left),('right',right)]:
        src=UNC/'references'/folder
        transport.copy_to_host([src/(slot+s) for s in ('.json','.csv','_preparation.csv')],destination=ROOT+'/planned_references')
    remote(PREFIX+'PAIR='+repr(pair)+'\n'+r'''
from pathlib import Path
import json,subprocess,shutil,hashlib,sys
root=Path(ROOT);assert not subprocess.check_output(['docker','ps','-q'],text=True).strip()
ledger=json.loads((root/'campaign_20260914.json').read_text())
assert not ledger['sealed'] and len(ledger['attempts'])+2<=ledger['maximum_attempts']
assert all(r['state']=='WSL_MOVED' for r in ledger['attempts'])
sys.path.insert(0,str(root/'source/src'))
from aic_transfuser_lite.data.time_large_recovery_reference_v1 import validate_large_reference
from aic_transfuser_lite.runtime.recovery_parallel_v1 import validate_parallel_plan
instances=[]
for side,domain,nodes,sim,split in [('left',1,'0-3','8-9,12-15','train'),('right',2,'4-7','10-11,16-19','validation')]:
 ref=json.loads((root/'planned_references'/(side+'.json')).read_text())
 validate_large_reference(ref,root/'planned_references')
 for suffix in ('.json','.csv','_preparation.csv'):shutil.copy2(root/'planned_references'/(side+suffix),root/'references'/(side+suffix))
 name=f'codex-time-recovery-corner60-p{PAIR:02}-d{domain}'
 assert not (root/name).exists() and not any(r['run_id']==name for r in ledger['attempts'])
 row=dict(run_id=name,side=side,split=split,ros_domain_id=domain,network_container=name+'-net',node_cpus=nodes,simulation_cpus=sim,event_cap=len(ref['large_recovery']['config']['sites']),pair=PAIR)
 instances.append(row);ledger['planned_runs'].append(row)
plan=dict(schema='awsim_recovery_parallel_v1',scope=root.name,instances=instances)
for row in instances:validate_parallel_plan(plan,scope=root.name,run_id=row['run_id'])
(root/'parallel_plan.json').write_text(json.dumps(plan,indent=2))
(root/f'parallel_plan_pair{PAIR:02}.json').write_text(json.dumps(plan,indent=2))
(root/'campaign_20260914.json').write_text(json.dumps(ledger,indent=2))
print(json.dumps(plan))
''')


def start(side):
    remote(PREFIX+'SIDE='+repr(side)+'\n'+r'''
from pathlib import Path
import json,subprocess,hashlib,sys,os,time,shutil
root=Path(ROOT);plan=json.loads((root/'parallel_plan.json').read_text());row=next(r for r in plan['instances'] if r['side']==SIDE)
ledger=json.loads((root/'campaign_20260914.json').read_text());name=row['run_id']
assert len(ledger['attempts'])<ledger['maximum_attempts']<=12 and not ledger['sealed'] and not (root/name).exists()
assert json.loads((root/'domain0_isolation.json').read_text())['status']=='PRIVATE_DOMAIN0_ISOLATION_PASS'
assert shutil.disk_usage(root).free>=12*2**30
for other in plan['instances']:
 p=root/other['run_id']
 if p.exists():assert (p/'drive_authorized.json').exists() and not (p/'result.json').exists()
for rel,digest in json.loads((root/'deployment.json').read_text())['files'].items():assert hashlib.sha256((root/rel).read_bytes()).hexdigest()==digest,rel
assert json.loads((root/'speed_gate.json').read_text())['all_within_005_mps']
sys.path.insert(0,str(root/'source/src'))
from aic_transfuser_lite.runtime.recovery_parallel_v1 import validate_parallel_containers
ids=subprocess.check_output(['docker','ps','-q'],text=True).splitlines()
validate_parallel_containers(plan,json.loads(subprocess.check_output(['docker','inspect',*ids],text=True)))
command=['timeout','--signal=TERM','--kill-after=20s','1980s','python3',str(root/'source/tools/run_time_recovery_awsim.py'),'--campaign-root',str(root),'--run-id',name,'--side',SIDE,'--speed-policy','aligned_gain4_v1','--separate-cpus','--ros-domain-id',str(row['ros_domain_id']),'--parallel-plan',str(root/'parallel_plan.json')]
env=dict(os.environ,PYTHONPATH=str(root/'source/src'),**json.loads((root/'display.json').read_text()))
with (root/(name+'_supervisor.log')).open('x') as f:p=subprocess.Popen(command,env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
ledger['attempts'].append(dict(row,state='STARTED',pid=p.pid,command=command,started_unix_s=time.time()))
(root/'campaign_20260914.json').write_text(json.dumps(ledger,indent=2));print(json.dumps(ledger['attempts'][-1]))
''')


def ship(pair):
    transport.ROOT=ROOT;transport.OUT=OUT;transport.RAW=RAW;transport.UNC=UNC
    transport.copy_to_host=partial(transport.copy_to_host,destination=ROOT)
    transport.ship(pair,prefix=f'corner60_pair{pair:02}_20260916')


def revise():
    head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO,text=True).strip()
    assert not subprocess.check_output(['git','status','--porcelain'],cwd=REPO,text=True).strip()
    gate=json.loads((UNC/('test_gate_'+head[:7]+'.json')).read_text())
    assert gate['commit']==head and gate['full_exit']==0
    assert hashlib.sha256((UNC/Path(gate['log']).name).read_bytes()).hexdigest()==gate['log_sha256']
    archive=HERE/('source_'+head[:7]+'.tar.gz')
    subprocess.run(['git','archive','--format=tar.gz','--prefix=source/','--output='+str(archive),head,'src','tools','configs'],check=True,cwd=REPO)
    meta=HERE/('revision_'+head[:7]+'.json')
    meta.write_text(json.dumps(dict(commit=head,test_gate=gate,archive=archive.name,sha256=hashlib.sha256(archive.read_bytes()).hexdigest()),indent=2))
    transport.copy_to_host([archive,meta],destination=ROOT)
    remote(PREFIX+'META='+repr(meta.name)+'\n'+r'''
from pathlib import Path
import subprocess,json,hashlib,tarfile,shutil
root=Path(ROOT);assert not subprocess.check_output(['docker','ps','-q'],text=True).strip()
ledger=json.loads((root/'campaign_20260914.json').read_text());assert all(r['state']=='WSL_MOVED' for r in ledger['attempts'])
meta=json.loads((root/META).read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
assert sha(root/meta['archive'])==meta['sha256']
old=(root/'deployed_commit.txt').read_text().strip();source=root/'source';prior=root/('source_'+old[:7])
assert source.resolve().parent==root and prior.resolve().parent==root and not prior.exists()
source.rename(prior)
dep=json.loads((root/'deployment.json').read_text());files={k:v for k,v in dep['files'].items() if not k.startswith('source/')}
with tarfile.open(root/meta['archive']) as tf:
 for m in tf.getmembers():
  p=root/m.name;assert p.resolve().is_relative_to(root/'source') and (m.isfile() or m.isdir())
  if m.isdir():p.mkdir(parents=True,exist_ok=True)
  else:
   p.parent.mkdir(parents=True,exist_ok=True)
   with p.open('xb') as f:f.write(tf.extractfile(m).read())
   if m.mode&0o111:p.chmod(0o755)
   files[m.name]=sha(p)
shutil.copy2(root/'deployment.json',root/('deployment_'+old[:7]+'.json'))
(root/'deployment.json').write_text(json.dumps(dict(source_commit=meta['commit'],files=files),indent=2))
(root/'test_gate.json').write_text(json.dumps(meta['test_gate'],indent=2))
(root/'deployed_commit.txt').write_text(meta['commit']+'\n')
print('CORNER_REVISION_READY',meta['commit'])
''',timeout=120)


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('mode',choices=['setup','prepare','networks','isolation','start','status','remove','ship','revise']);ap.add_argument('--pair',type=int);ap.add_argument('--left');ap.add_argument('--right');ap.add_argument('--side')
    a=ap.parse_args()
    if a.mode=='setup':setup()
    elif a.mode=='prepare':prepare_pair(a.pair,a.left,a.right)
    elif a.mode=='start':start(a.side)
    elif a.mode=='ship':ship(a.pair)
    elif a.mode=='revise':revise()
    else:{'networks':parallel.networks,'isolation':parallel.isolation,'status':parallel.status,'remove':parallel.remove_networks}[a.mode]()
