"""Preserve the sealed 40 cm source; deploy byte-identical tested runtime."""
import hashlib,json,subprocess,tarfile
import ops60 as m

head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=m.REPO,text=True).strip()
assert not subprocess.check_output(['git','status','--porcelain'],cwd=m.REPO,text=True).strip()
assert not subprocess.check_output(['git','diff',m.RUNTIME_COMMIT,head,'--','src','tools','configs','tests'],cwd=m.REPO)
archive=m.HERE/'source.tar.gz'
assert not archive.exists()
subprocess.run(['git','archive','--format=tar.gz','--prefix=source/','--output='+str(archive),m.RUNTIME_COMMIT,'src','tools','configs'],cwd=m.REPO,check=True)
refs=m.HERE/'references_initial.tar.gz'
with tarfile.open(refs,'x:gz') as tf:
    for p in sorted((m.UNC/'d60_g03_left_initial').iterdir()):
        assert p.is_file()
        tf.add(p,arcname='planned_references/d60_g03_left_initial/'+p.name,recursive=False)
boot=dict(commit=m.RUNTIME_COMMIT,operator_head=head,runtime_source_unchanged=True,
 source_archive_sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),
 reference_archive_sha256=hashlib.sha256(refs.read_bytes()).hexdigest())
(m.HERE/'bootstrap.json').write_text(json.dumps(boot,indent=2))
m.remote('ROOT='+repr(m.ROOT)+'\n'+r'''
from pathlib import Path
import json,shutil,subprocess
root=Path(ROOT);old=root.parent/'time_recovery_40cm_20260915'
assert root.resolve()==root and root.parent==Path('/home/graneple/e2e_autonomous')
assert json.loads((old/'campaign_final.json').read_text())['sealed']
assert not subprocess.check_output(['docker','ps','-q'],text=True).strip()
assert shutil.disk_usage(root.parent).free>=12*2**30
root.mkdir(exist_ok=False)
''')
m.copy_to_host([archive,refs,m.HERE/'bootstrap.json',m.REPO/'docs/evidence/time_recovery_batches_20260914/move_pair.py'])
m.remote('ROOT='+repr(m.ROOT)+'\n'+r'''
from pathlib import Path
import hashlib,json,shutil,subprocess,tarfile,os
root=Path(ROOT);old=root.parent/'time_recovery_40cm_20260915'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
boot=json.loads((root/'bootstrap.json').read_text());files={};common={}
dep=json.loads((old/'deployment.json').read_text())
assert dep['source_commit']==boot['commit']
for rel,digest in dep['files'].items():
    assert sha(old/rel)==digest,rel
    if not rel.startswith(('inputs/','cpp_install/')):continue
    dest=root/rel;dest.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(old/rel,dest);files[rel]=common[rel]=digest
for name,digest,subroot in [('source.tar.gz',boot['source_archive_sha256'],'source'),('references_initial.tar.gz',boot['reference_archive_sha256'],'planned_references')]:
    assert sha(root/name)==digest
    with tarfile.open(root/name) as tf:
        for member in tf.getmembers():
            dest=root/member.name
            assert dest.resolve().is_relative_to(root/subroot) and (member.isdir() or member.isfile())
            if member.isdir():dest.mkdir(parents=True,exist_ok=True)
            else:
                dest.parent.mkdir(parents=True,exist_ok=True)
                with dest.open('xb') as f:f.write(tf.extractfile(member).read())
                if member.mode&0o111:dest.chmod(0o755)
                if subroot=='source':
                    files[member.name]=sha(dest)
                    assert files[member.name]==dep['files'][member.name]
assert files==dep['files']
(root/'references').mkdir()
for name in ('speed_gate.json','display.json','test_gate.json','latency_live_gate.json'):
    shutil.copy2(old/name,root/name)
(root/'prior_synthetic').mkdir()
shutil.copy2(old/'synthetic_r01/synthetic_smoke_result.json',root/'prior_synthetic/synthetic_smoke_result.json')
test=json.loads((root/'test_gate.json').read_text());smoke=json.loads((root/'prior_synthetic/synthetic_smoke_result.json').read_text())
assert test['commit']==boot['commit']==smoke['source_commit'] and test['full_exit']==0
assert smoke['status']=='SYNTHETIC_ROS_WIRING_PASS'
proof=dict(source_commit=boot['commit'],operator_head=boot['operator_head'],runtime_source_unchanged=True,
 proof_hashes={n:sha(root/n) for n in ('test_gate.json','latency_live_gate.json','prior_synthetic/synthetic_smoke_result.json')},
 scope='Reuse source-identical runtime tests, ROS wiring and delay mitigation only. 60 cm physical feasibility is unverified until this campaign. Three events per lap, no event-count increase.')
(root/'reused_runtime_proof.json').write_text(json.dumps(proof,indent=2))
(root/'deployment.json').write_text(json.dumps(dict(source_commit=boot['commit'],files=files),indent=2))
(root/'common_assets.json').write_text(json.dumps(common,indent=2))
(root/'deployed_commit.txt').write_text(boot['commit']+'\n')
runs=[dict(run_id='codex-time-recovery-60cm-d60-g03-left-r%02d'%i,side='left',pair=i,event_cap=3,reference='d60_g03_left_initial') for i in (1,2)]
(root/'campaign_20260914.json').write_text(json.dumps(dict(maximum_attempts=8,maximum_planned_runs=4,initial_event_cap=3,maximum_event_cap=3,event_increment=0,batch_size=2,shipping_after_each_run=True,attempts=[],planned_runs=runs,sealed=False,scope='60 cm, exactly 3 planned deviations per lap; target 2 complete laps per side with run-wise split; at most 8 attempts including calibration'),indent=2))
display=json.loads((root/'display.json').read_text())
subprocess.run(['xdpyinfo'],env=dict(os.environ,**display),stdout=subprocess.DEVNULL,stderr=subprocess.PIPE,check=True,timeout=10)
repo='/home/graneple/git/autononous_ai/aichallenge-racingkart'
(root/'host_before.json').write_text(json.dumps(dict(head=subprocess.check_output(['git','-C',repo,'rev-parse','HEAD'],text=True).strip(),git_status_sha256=hashlib.sha256(subprocess.check_output(['git','-C',repo,'status','--porcelain'])).hexdigest(),free_bytes=shutil.disk_usage(root).free),indent=2))
print(json.dumps(dict(status='SETUP_60CM_READY',source_commit=boot['commit'],runtime_files=len(files),free_gib=shutil.disk_usage(root).free/2**30)))
''',timeout=180)
for name in ('bootstrap.json','reused_runtime_proof.json','deployment.json','common_assets.json','test_gate.json','host_before.json'):
    assert not (m.UNC/name).exists()
    subprocess.run(['scp',m.HOST+':'+m.ROOT+'/'+name,str(m.UNC/name)],check=True,timeout=60)
