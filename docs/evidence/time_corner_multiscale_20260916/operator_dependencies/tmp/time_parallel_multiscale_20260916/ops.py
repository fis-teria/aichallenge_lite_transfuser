"""Finite two-instance pilot. AWSIM source and assets remain unchanged."""
from pathlib import Path
import argparse
import hashlib
import json
import subprocess
import sys

REPO = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(REPO/'tmp/time_recovery_60cm_20260916'))
import ops60 as previous

ROOT = '/home/graneple/e2e_autonomous/time_recovery_parallel_20260916'
HOST = previous.HOST
remote = previous.remote
PREFIX = 'ROOT='+repr(ROOT)+'\n'


def setup():
    head = subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO,text=True).strip()
    assert not subprocess.check_output(['git','status','--porcelain'],cwd=REPO,text=True).strip()
    log = Path(r'\\wsl.localhost\Ubuntu-22.04-Recovered\home\thistle\e2e_autonomous\runs\time_parallel_retrain_checks_20260916\full_14c745a.log')
    assert '2695 passed, 4 skipped' in log.read_text()
    # Additional commits may contain only generated experiment configuration.
    assert not subprocess.check_output(['git','diff','14c745a',head,'--','src','tools','tests'],cwd=REPO)
    archive=HERE/'source.tar.gz'
    subprocess.run(['git','archive','--format=tar.gz','--prefix=source/','--output='+str(archive),head,'src','tools','configs'],cwd=REPO,check=True)
    boot=dict(commit=head,archive_sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),
        test_gate=dict(commit=head,tested_commit='14c745aa3ca4dad60b41558a07a6eab8bd6d661c',full_exit=0,
            log=str(log),log_sha256=hashlib.sha256(log.read_bytes()).hexdigest(),passed=2695,skipped=4))
    (HERE/'bootstrap.json').write_text(json.dumps(boot,indent=2))
    remote(PREFIX+r'''
from pathlib import Path
import subprocess,shutil
root=Path(ROOT)
assert root.resolve()==root and root.parent==Path('/home/graneple/e2e_autonomous')
assert not subprocess.check_output(['docker','ps','-q'],text=True).strip()
assert shutil.disk_usage(root.parent).free>=12*2**30
root.mkdir(exist_ok=False)
''')
    previous.previous.transport.copy_to_host([archive,HERE/'bootstrap.json',REPO/'docs/evidence/time_recovery_batches_20260914/move_pair.py'],destination=ROOT)
    remote(PREFIX+r'''
from pathlib import Path
import hashlib,json,shutil,subprocess,tarfile,os,sys
root=Path(ROOT);base=root.parent/'time_recovery_60cm_20260916'
repo=Path('/home/graneple/git/autononous_ai/aichallenge-racingkart')
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
boot=json.loads((root/'bootstrap.json').read_text()); files={}
assert json.loads((base/'campaign_final.json').read_text())['sealed']
dep=json.loads((base/'deployment.json').read_text())
for rel,digest in dep['files'].items():
    if not rel.startswith(('inputs/','cpp_install/')):continue
    assert sha(base/rel)==digest,rel
    dst=root/rel;dst.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(base/rel,dst);files[rel]=digest
assert sha(root/'source.tar.gz')==boot['archive_sha256']
with tarfile.open(root/'source.tar.gz') as tf:
    for member in tf.getmembers():
        dst=root/member.name
        assert dst.resolve().is_relative_to(root/'source') and (member.isfile() or member.isdir())
        if member.isdir():dst.mkdir(parents=True,exist_ok=True)
        else:
            dst.parent.mkdir(parents=True,exist_ok=True)
            with dst.open('xb') as f:f.write(tf.extractfile(member).read())
            if member.mode&0o111:dst.chmod(0o755)
            files[member.name]=sha(dst)
(root/'references').mkdir()
refs={'left':'d60_g03_left_p05_108_s4','right':'d60_g03_right_p02_247_s4'}
ref_hashes={'left':'8e4f5f07a8316ce689d9ec8215112652fec73b19db27369e2a8bfba7af39b0c0','right':'30a22b14b87bb4890b58ecb9f82c5957303111d94da99dc7d31943c7515c3819'}
for side,folder in refs.items():
    src=base/'planned_references'/folder
    assert sha(src/(side+'.json'))==ref_hashes[side]
    ref=json.loads((src/(side+'.json')).read_text())
    assert ref['large_recovery']['map_screen_pass']
    for suffix in ('.csv','.json','_preparation.csv'):
        dst=root/'references'/(side+suffix);shutil.copy2(src/(side+suffix),dst)
        files[dst.relative_to(root).as_posix()]=sha(dst)
for name in ('display.json','speed_gate.json'):shutil.copy2(base/name,root/name)
display=json.loads((root/'display.json').read_text())
subprocess.run(['xdpyinfo'],env=dict(os.environ,**display),stdout=subprocess.DEVNULL,stderr=subprocess.PIPE,check=True)
sys.path[:0]=[str(root/'source/src'),str(root/'source/tools')]
from run_time_recovery_awsim import collection_dds_profile
from aic_transfuser_lite.runtime.recovery_parallel_v1 import validate_parallel_plan
(root/'probe_cyclonedds.xml').write_text(collection_dds_profile((repo/'vehicle/cyclonedds.xml').read_text(),large_mode=True))
instances=[]
for side,domain,nodes,sim in [('left',1,'0-3','8-9,12-15'),('right',2,'4-7','10-11,16-19')]:
    name='codex-time-recovery-parallel-d'+str(domain)+'-'+side+'-r01'
    instances.append(dict(run_id=name,side=side,ros_domain_id=domain,network_container=name+'-net',node_cpus=nodes,simulation_cpus=sim,event_cap=3))
plan=dict(schema='awsim_recovery_parallel_v1',scope=root.name,instances=instances)
for row in instances:validate_parallel_plan(plan,scope=root.name,run_id=row['run_id'])
(root/'parallel_plan.json').write_text(json.dumps(plan,indent=2))
(root/'campaign_20260914.json').write_text(json.dumps(dict(maximum_attempts=2,maximum_planned_runs=2,initial_event_cap=3,maximum_event_cap=3,event_increment=0,batch_size=2,attempts=[],planned_runs=[dict(r,pair=1) for r in instances],sealed=False),indent=2))
(root/'deployed_commit.txt').write_text(boot['commit']+'\n')
(root/'deployment.json').write_text(json.dumps(dict(source_commit=boot['commit'],files=files),indent=2))
(root/'test_gate.json').write_text(json.dumps(boot['test_gate'],indent=2))
sim=repo/'aichallenge/simulator/AWSIM'
fingerprint={p.relative_to(sim).as_posix():dict(bytes=p.stat().st_size,sha256=sha(p)) for p in sorted(sim.rglob('*')) if p.is_file()}
before=dict(head=subprocess.check_output(['git','-C',str(repo),'rev-parse','HEAD'],text=True).strip(),git_status_sha256=hashlib.sha256(subprocess.check_output(['git','-C',str(repo),'status','--porcelain'])).hexdigest(),awsim_files=fingerprint,free_bytes=shutil.disk_usage(root).free)
(root/'host_before.json').write_text(json.dumps(before,indent=2))
print(json.dumps(dict(status='PARALLEL_SETUP_READY',files=len(files),awsim_files=len(fingerprint),free_gib=shutil.disk_usage(root).free/2**30)))
''',timeout=180)


def networks():
    remote(PREFIX+r'''
from pathlib import Path
import json,subprocess,sys
root=Path(ROOT);p=json.loads((root/'parallel_plan.json').read_text())
assert not subprocess.check_output(['docker','ps','-q'],text=True).strip()
repo=Path('/home/graneple/git/autononous_ai/aichallenge-racingkart')
for row in p['instances']:
    subprocess.run(['docker','run','-d','--name',row['network_container'],'--network','none',
        '--label','aic.recovery.scope='+root.name,'--label','aic.recovery.run_id='+row['run_id'],
        '-e','ROS_DOMAIN_ID=0','-e','CYCLONEDDS_URI=file:///opt/autoware/cyclonedds.xml',
        '-v',str(root/'probe_cyclonedds.xml')+':/opt/autoware/cyclonedds.xml:ro',
        '-v',str(repo/'aichallenge')+':/aichallenge:ro','--entrypoint','sleep',
        'codex-cartographer-v4-build:20260910','infinity'],check=True,capture_output=True)
sys.path.insert(0,str(root/'source/src'))
from aic_transfuser_lite.runtime.recovery_parallel_v1 import validate_parallel_containers
ids=subprocess.check_output(['docker','ps','-q'],text=True).splitlines()
infos=json.loads(subprocess.check_output(['docker','inspect',*ids],text=True))
validate_parallel_containers(p,infos)
(root/'network_owners.json').write_text(json.dumps(infos,indent=2))
print('PRIVATE_NETWORKS_READY')
''')


def isolation():
    remote(PREFIX+r'''
from pathlib import Path
import json,subprocess,concurrent.futures
root=Path(ROOT);plan=json.loads((root/'parallel_plan.json').read_text())
source="""import json,time,rclpy\nfrom std_msgs.msg import String\nrclpy.init()\nn=rclpy.create_node('private_domain0_probe_'+LABEL)\nseen=[]\np=n.create_publisher(String,'/codex_private_domain0_isolation_probe',10)\ns=n.create_subscription(String,'/codex_private_domain0_isolation_probe',lambda m:seen.append(m.data),10)\nt=time.monotonic()\nwhile time.monotonic()-t<16:\n p.publish(String(data=LABEL)); rclpy.spin_once(n,timeout_sec=.15)\nprint(json.dumps(dict(label=LABEL,seen_labels=sorted(set(seen)),messages=len(seen))))\nn.destroy_node();rclpy.shutdown()\nassert len(seen)>10 and set(seen)=={LABEL}\n"""
def probe(row):
    result=subprocess.run(['docker','exec','-i',row['network_container'],'bash','-lc','source /aichallenge/workspace/install/setup.bash && python3 -'],input=('LABEL='+repr(row['side'])+'\n'+source).encode(),capture_output=True,timeout=40)
    (root/('domain0_probe_'+row['side']+'.log')).write_bytes(result.stdout+result.stderr)
    result.check_returncode()
    parsed=json.loads(result.stdout.decode().splitlines()[-1]);parsed['returncode']=result.returncode
    parsed['network_namespace']=subprocess.check_output(['docker','exec',row['network_container'],'readlink','/proc/self/ns/net'],text=True).strip()
    return parsed
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
    results=list(pool.map(probe,plan['instances']))
assert len({r['network_namespace'] for r in results})==2
proof=dict(status='PRIVATE_DOMAIN0_ISOLATION_PASS',probes=results,no_admin_start_or_reset_published=True)
(root/'domain0_isolation.json').write_text(json.dumps(proof,indent=2))
print(json.dumps(proof))
''',timeout=60)


def start(side):
    remote(PREFIX+'SIDE='+repr(side)+'\n'+r'''
from pathlib import Path
import json,subprocess,shutil,hashlib,os,time,sys
root=Path(ROOT);plan=json.loads((root/'parallel_plan.json').read_text())
row=next(r for r in plan['instances'] if r['side']==SIDE);name=row['run_id']
assert json.loads((root/'domain0_isolation.json').read_text())['status']=='PRIVATE_DOMAIN0_ISOLATION_PASS'
ledger=json.loads((root/'campaign_20260914.json').read_text())
assert not ledger['sealed'] and len(ledger['attempts'])<ledger['maximum_attempts']<=4 and not (root/name).exists()
for other in plan['instances']:
    if other['run_id']==name:continue
    first=root/other['run_id']
    if first.exists():assert (first/'drive_authorized.json').exists() and not (first/'result.json').exists()
assert shutil.disk_usage(root).free>=12*2**30
dep=json.loads((root/'deployment.json').read_text())
for rel,digest in dep['files'].items():assert hashlib.sha256((root/rel).read_bytes()).hexdigest()==digest,rel
assert json.loads((root/'speed_gate.json').read_text())['all_within_005_mps']
sys.path.insert(0,str(root/'source/src'))
from aic_transfuser_lite.runtime.recovery_parallel_v1 import validate_parallel_containers
ids=subprocess.check_output(['docker','ps','-q'],text=True).splitlines()
validate_parallel_containers(plan,json.loads(subprocess.check_output(['docker','inspect',*ids],text=True)))
command=['timeout','--signal=TERM','--kill-after=20s','1980s','python3',str(root/'source/tools/run_time_recovery_awsim.py'),
    '--campaign-root',str(root),'--run-id',name,'--side',SIDE,'--speed-policy','aligned_gain4_v1','--separate-cpus',
    '--ros-domain-id',str(row['ros_domain_id']),'--parallel-plan',str(root/'parallel_plan.json')]
env=dict(os.environ,PYTHONPATH=str(root/'source/src'),**json.loads((root/'display.json').read_text()))
with (root/(name+'_supervisor.log')).open('x') as f:
    proc=subprocess.Popen(command,env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
ledger['attempts'].append(dict(row,pair=1+int(name.endswith('r02')),state='STARTED',pid=proc.pid,command=command,started_unix_s=time.time()))
(root/'campaign_20260914.json').write_text(json.dumps(ledger,indent=2))
print(json.dumps(ledger['attempts'][-1]))
''')


def status():
    remote(PREFIX+r'''
from pathlib import Path
import json,subprocess,shutil
root=Path(ROOT);plan=json.loads((root/'parallel_plan.json').read_text());rows=[]
for row in plan['instances']:
    d=root/row['run_id'];c={};r={}
    if (d/'control_heartbeat.json').exists():c=json.loads((d/'control_heartbeat.json').read_text())
    if (d/'result.json').exists():r=json.loads((d/'result.json').read_text());c=r.get('last_control',c)
    rows.append(dict(side=row['side'],started=d.exists(),status=r.get('status'),error=r.get('error'),fault=c.get('fault'),reason=c.get('reason'),phase=c.get('phase'),progress_m=(c.get('projection') or {}).get('s_m'),speed=c.get('speed_mps'),ready_ticks=c.get('ready_ticks'),authorized=(d/'drive_authorized.json').exists(),stop_confirmed=c.get('stop_confirmed'),state=c.get('large_recovery',{}).get('state'),closed_bag=r.get('nodes',{}).get('closed_bag'),cleanup_errors=r.get('cleanup_errors')))
print(json.dumps(dict(runs=rows,containers=subprocess.check_output(['docker','ps','--format','{{.Names}}'],text=True).splitlines(),free_gib=shutil.disk_usage(root).free/2**30)))
''')


def remove_networks():
    remote(PREFIX+r'''
from pathlib import Path
import subprocess,json,sys
root=Path(ROOT);plan=json.loads((root/'parallel_plan.json').read_text())
for row in plan['instances']:
    result=json.loads((root/row['run_id']/'result.json').read_text())
    assert result['nodes']['closed_bag'] and not result['cleanup_errors']
ids=subprocess.check_output(['docker','ps','-q'],text=True).splitlines()
infos=json.loads(subprocess.check_output(['docker','inspect',*ids],text=True))
assert {r['Name'].lstrip('/') for r in infos}=={r['network_container'] for r in plan['instances']}
for r in infos:
    assert r['Config']['Labels']['aic.recovery.scope']==root.name
    subprocess.run(['docker','rm','-f',r['Id']],check=True,capture_output=True)
print('COMPLETED_INSTANCE_NETWORKS_REMOVED')
''')


def ship(pair):
    from functools import partial
    t=previous.previous.transport
    t.ROOT=ROOT
    t.OUT='/home/thistle/e2e_autonomous/runs/time_recovery_parallel_20260916'
    t.RAW='/home/thistle/e2e_autonomous/raw/time_recovery_parallel_20260916'
    t.UNC=Path(r'\\wsl.localhost\Ubuntu-22.04-Recovered')/t.OUT.lstrip('/')
    t.UNC.mkdir(exist_ok=True)
    t.copy_to_host=partial(t.copy_to_host,destination=ROOT)
    t.ship(pair,prefix='parallel_pair%02d_20260916'%pair)


def revise():
    head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO,text=True).strip()
    assert not subprocess.check_output(['git','status','--porcelain'],cwd=REPO,text=True).strip()
    log=Path(r'\\wsl.localhost\Ubuntu-22.04-Recovered\home\thistle\e2e_autonomous\runs\time_parallel_retrain_checks_20260916')/('full_'+head[:7]+'.log')
    assert '2696 passed, 4 skipped' in log.read_text()
    archive=HERE/('source_'+head[:7]+'.tar.gz')
    subprocess.run(['git','archive','--format=tar.gz','--prefix=source/','--output='+str(archive),head,'src','tools','configs'],cwd=REPO,check=True)
    metadata=dict(commit=head,archive=archive.name,sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),test_gate=dict(commit=head,full_exit=0,log=str(log),log_sha256=hashlib.sha256(log.read_bytes()).hexdigest(),passed=2696,skipped=4))
    meta=HERE/('revision_'+head[:7]+'.json');meta.write_text(json.dumps(metadata,indent=2))
    previous.previous.transport.copy_to_host([archive,meta],destination=ROOT)
    remote(PREFIX+'META='+repr(meta.name)+'\n'+r'''
from pathlib import Path
import hashlib,json,shutil,subprocess,tarfile
root=Path(ROOT);meta=json.loads((root/META).read_text())
assert not subprocess.check_output(['docker','ps','-q'],text=True).strip()
ledger=json.loads((root/'campaign_20260914.json').read_text())
assert len(ledger['attempts'])==2 and all(a['state']=='WSL_MOVED' for a in ledger['attempts'])
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
dep=json.loads((root/'deployment.json').read_text())
for rel,digest in dep['files'].items():assert sha(root/rel)==digest,rel
old=dep['source_commit'];records=root/'revision_records'/old;records.mkdir(parents=True,exist_ok=False)
for name in ('parallel_plan.json','network_owners.json','domain0_isolation.json','deployment.json','deployed_commit.txt','test_gate.json','campaign_20260914.json','parallel_capacity_profile.json'):
    shutil.copy2(root/name,records/name)
assert sha(root/meta['archive'])==meta['sha256']
staged=root/('staged_'+meta['commit'][:7]);staged.mkdir(exist_ok=False)
with tarfile.open(root/meta['archive']) as tf:
    for member in tf.getmembers():
        dst=staged/member.name
        assert dst.resolve().is_relative_to(staged/'source') and (member.isfile() or member.isdir())
        if member.isdir():dst.mkdir(parents=True,exist_ok=True)
        else:
            dst.parent.mkdir(parents=True,exist_ok=True)
            with dst.open('xb') as f:f.write(tf.extractfile(member).read())
            if member.mode&0o111:dst.chmod(0o755)
(root/'source').rename(root/('source_'+old[:7]));(staged/'source').rename(root/'source')
dep['files']={k:v for k,v in dep['files'].items() if not k.startswith('source/')}
dep['files'].update({p.relative_to(root).as_posix():sha(p) for p in (root/'source').rglob('*') if p.is_file()})
dep['source_commit']=meta['commit']
(root/'deployment.json').write_text(json.dumps(dep,indent=2));(root/'deployed_commit.txt').write_text(meta['commit']+'\n')
(root/'test_gate.json').write_text(json.dumps(meta['test_gate'],indent=2))
plan=json.loads((root/'parallel_plan.json').read_text())
for row in plan['instances']:
    row['run_id']=row['run_id'].replace('-r01','-r02');row['network_container']=row['run_id']+'-net'
    ledger['planned_runs'].append(dict(row,pair=2))
ledger.update(maximum_attempts=4,maximum_planned_runs=4,sealed=False,budget_amendment='Two finite replacement laps after correcting collector node-name assumption. Preserve original failed preflight and completed left lap; no change to AWSIM.')
(root/'parallel_plan.json').write_text(json.dumps(plan,indent=2))
(root/'campaign_20260914.json').write_text(json.dumps(ledger,indent=2))
print(json.dumps(dict(status='PARALLEL_COLLECTOR_REVISED',source_commit=meta['commit'])))
''',timeout=180)


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('command',choices=['setup','networks','isolation','start','status','remove_networks','ship','revise']);ap.add_argument('--side',choices=['left','right']);ap.add_argument('--pair',type=int);a=ap.parse_args()
    if a.command=='start':start(a.side)
    elif a.command=='ship':ship(a.pair)
    else:globals()[a.command]()
