"""Bounded new large-recovery campaign; never mutates earlier collection roots."""
from pathlib import Path
import argparse
import hashlib
import json
import subprocess
import sys
import tarfile

REPO = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(REPO/'tmp/time_recovery_separated_20260915'))
import manage as transport

ROOT = '/home/graneple/e2e_autonomous/time_recovery_large_live_20260915'
OUT = '/home/thistle/e2e_autonomous/runs/time_recovery_large_live_20260915'
RAW = '/home/thistle/e2e_autonomous/raw/time_recovery_large_live_20260915'
UNC = Path(r'\\wsl.localhost\Ubuntu-22.04-Recovered')/OUT.lstrip('/')


def remote(code: str, **kw) -> str:
    return transport.remote(code, **kw)


def setup() -> None:
    head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO,text=True).strip()
    assert not subprocess.check_output(['git','status','--porcelain'],cwd=REPO,text=True).strip()
    gate=json.loads((UNC/'test_gate.json').read_bytes())
    assert gate['commit']==head and gate['full_exit']==0
    assert hashlib.sha256((UNC/Path(gate['log']).name).read_bytes()).hexdigest()==gate['log_sha256']
    archive=HERE/'source.tar.gz'
    assert not archive.exists()
    subprocess.run(['git','archive','--format=tar.gz','--prefix=source/','--output='+str(archive),head,'src','tools','configs'],cwd=REPO,check=True)
    refs=HERE/'references.tar.gz'
    with tarfile.open(refs,'x:gz') as tf:
        for side in ('left','right'):
            for p in sorted((UNC/('g03_'+side)).iterdir()):
                assert p.is_file()
                tf.add(p,arcname='planned_references/g03_'+side+'/'+p.name,recursive=False)
    boot=dict(commit=head,source_archive_sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),reference_archive_sha256=hashlib.sha256(refs.read_bytes()).hexdigest(),test_gate=gate)
    (HERE/'bootstrap.json').write_text(json.dumps(boot,indent=2))
    remote(f"from pathlib import Path\np=Path({ROOT!r});assert p.resolve()==p and p.parent==Path('/home/graneple/e2e_autonomous');p.mkdir(exist_ok=False)\n")
    transport.copy_to_host([archive,refs,HERE/'bootstrap.json',REPO/'docs/evidence/time_recovery_batches_20260914/move_pair.py'],destination=ROOT)
    remote(rf'''
from pathlib import Path
import hashlib,json,shutil,subprocess,tarfile,os
root=Path({ROOT!r});base=root.parent/'time_recovery_separated_20260915'
assert not subprocess.check_output(['docker','ps','-q'],text=True).strip()
assert shutil.disk_usage(root).free>=12*2**30
boot=json.loads((root/'bootstrap.json').read_text());files={{}}
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
deployment=json.loads((base/'deployment.json').read_text())
for rel,digest in deployment['files'].items():
    if not rel.startswith(('inputs/','cpp_install/')):continue
    src=base/rel;assert src.resolve().is_relative_to(base) and sha(src)==digest
    dst=root/rel;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst);files[rel]=digest
for name,digest,subroot in [('source.tar.gz',boot['source_archive_sha256'],'source'),('references.tar.gz',boot['reference_archive_sha256'],'planned_references')]:
    assert sha(root/name)==digest
    with tarfile.open(root/name) as tf:
        for member in tf.getmembers():
            dst=root/member.name
            assert dst.resolve().is_relative_to(root/subroot) and (member.isfile() or member.isdir())
            if member.isdir():dst.mkdir(parents=True,exist_ok=True)
            else:
                dst.parent.mkdir(parents=True,exist_ok=True)
                with dst.open('xb') as f:f.write(tf.extractfile(member).read())
                if member.mode&0o111:dst.chmod(0o755)
                if subroot=='source':files[member.name]=sha(dst)
(root/'references').mkdir()
for name in ('speed_gate.json','display.json'):shutil.copy2(base/name,root/name)
display=json.loads((root/'display.json').read_text())
subprocess.run(['xdpyinfo'],env=dict(os.environ,**display),stdout=subprocess.DEVNULL,stderr=subprocess.PIPE,check=True)
(root/'deployed_commit.txt').write_text(boot['commit']+'\n')
(root/'test_gate.json').write_text(json.dumps(boot['test_gate'],indent=2))
(root/'deployment.json').write_text(json.dumps(dict(source_commit=boot['commit'],files=files),indent=2))
runs=[dict(run_id='codex-time-recovery-large-live-g03-'+side+'-r01',side=side,pair=1,event_cap=3,reference='g03_'+side) for side in ('left','right')]
(root/'campaign_20260914.json').write_text(json.dumps(dict(maximum_attempts=6,maximum_planned_runs=4,initial_event_cap=3,event_increment=2,batch_size=2,attempts=[],planned_runs=runs,sealed=False),indent=2))
repo='/home/graneple/git/autononous_ai/aichallenge-racingkart'
(root/'host_before.json').write_text(json.dumps(dict(head=subprocess.check_output(['git','-C',repo,'rev-parse','HEAD'],text=True).strip(),git_status_sha256=hashlib.sha256(subprocess.check_output(['git','-C',repo,'status','--porcelain'])).hexdigest(),free_bytes=shutil.disk_usage(root).free),indent=2))
print(json.dumps(dict(status='LARGE_LIVE_SETUP_OK',source_commit=boot['commit'],runtime_files=len(files),free_gib=shutil.disk_usage(root).free/2**30)))
''')


def start(run_id: str) -> None:
    remote(rf'''
from pathlib import Path
import hashlib,json,os,shutil,subprocess,time
root=Path({ROOT!r});name={run_id!r}
ledger=json.loads((root/'campaign_20260914.json').read_text())
assert not ledger['sealed'] and ledger['maximum_attempts'] in (6,8) and len(ledger['attempts'])<ledger['maximum_attempts']
row=next(r for r in ledger['planned_runs'] if r['run_id']==name)
assert not any(r['run_id']==name for r in ledger['attempts']) and not (root/name).exists()
assert not subprocess.check_output(['docker','ps','-q'],text=True).strip()
assert shutil.disk_usage(root).free>=12*2**30
assert len([r for r in ledger['attempts'] if r['state']!='WSL_MOVED'])<2
if row['event_cap']>3:
    gate=json.loads((root/'expansion_gate.json').read_text())
    assert gate['expansion_allowed'] and gate['next_event_cap']==row['event_cap']
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
dep=json.loads((root/'deployment.json').read_text());gate=json.loads((root/'test_gate.json').read_text())
assert gate['commit']==dep['source_commit']==(root/'deployed_commit.txt').read_text().strip() and gate['full_exit']==0
for rel,digest in dep['files'].items():assert sha(root/rel)==digest,rel
assert json.loads((root/'speed_gate.json').read_text())['all_within_005_mps']
src=root/'planned_references'/row['reference'];side=row['side']
ref=json.loads((src/(side+'.json')).read_text());cfg=ref['large_recovery']['config']
assert cfg['event_cap']==row['event_cap'] and len(cfg['sites'])==row['event_cap']
assert sha(src/(side+'.csv'))==ref['reference_sha256']
assert sha(src/(side+'_preparation.csv'))==ref['large_recovery']['preparation_sha256']
for ext in ('.csv','.json','_preparation.csv'):shutil.copy2(src/(side+ext),root/'references'/(side+ext))
command=['timeout','--signal=TERM','--kill-after=20s','1980s','python3',str(root/'source/tools/run_time_recovery_awsim.py'),'--campaign-root',str(root),'--run-id',name,'--side',side,'--speed-policy','aligned_gain4_v1','--separate-cpus']
display=json.loads((root/'display.json').read_text());env=dict(os.environ,PYTHONPATH=str(root/'source/src'),**display)
subprocess.run(['xdpyinfo'],env=env,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE,check=True,timeout=10)
attempt=dict(row,state='STARTING',started_unix_s=time.time(),reference_sha256=sha(src/(side+'.json')),command=command)
ledger['attempts'].append(attempt);(root/'campaign_20260914.json').write_text(json.dumps(ledger,indent=2))
with (root/(name+'_supervisor.log')).open('x') as f:
    proc=subprocess.Popen(command,env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
attempt.update(state='STARTED',pid=proc.pid);(root/'campaign_20260914.json').write_text(json.dumps(ledger,indent=2))
print(json.dumps(attempt))
''')


def deploy_settling_revision(variant: str = 'g03_s4', replacement: str = 'codex-time-recovery-large-live-g03-left-r02') -> None:
    assert variant in ('g03_s4','g03_cmd','g03_final')
    assert replacement in ('codex-time-recovery-large-live-g03-left-r02','codex-time-recovery-large-live-g03-left-r03','codex-time-recovery-large-live-g03-left-r04')
    head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO,text=True).strip()
    assert not subprocess.check_output(['git','status','--porcelain'],cwd=REPO,text=True).strip()
    gate=json.loads((UNC/('test_gate_'+head[:7]+'.json')).read_bytes())
    assert gate['commit']==head and gate['full_exit']==0
    assert hashlib.sha256((UNC/Path(gate['log']).name).read_bytes()).hexdigest()==gate['log_sha256']
    archive=HERE/('source_'+head[:7]+'.tar.gz')
    assert not archive.exists()
    subprocess.run(['git','archive','--format=tar.gz','--prefix=source/','--output='+str(archive),head,'src','tools','configs'],cwd=REPO,check=True)
    refs=HERE/('references_'+head[:7]+'.tar.gz')
    with tarfile.open(refs,'x:gz') as tf:
        for ref_variant in ([variant,'g05_final'] if variant=='g03_final' else [variant]):
            for side in ('left','right'):
                for p in sorted((UNC/(ref_variant+'_'+side)).iterdir()):
                    tf.add(p,arcname='planned_references/'+ref_variant+'_'+side+'/'+p.name,recursive=False)
    meta=dict(commit=head,test_gate=gate,source_archive=archive.name,source_archive_sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),reference_archive=refs.name,reference_archive_sha256=hashlib.sha256(refs.read_bytes()).hexdigest())
    metadata=HERE/('revision_'+head[:7]+'.json');metadata.write_text(json.dumps(meta,indent=2))
    transport.copy_to_host([archive,refs,metadata],destination=ROOT)
    remote(rf'''
from pathlib import Path
import hashlib,json,shutil,subprocess,tarfile
root=Path({ROOT!r});meta=json.loads((root/{metadata.name!r}).read_text())
assert not subprocess.check_output(['docker','ps','-q'],text=True).strip()
ledger=json.loads((root/'campaign_20260914.json').read_text())
assert all(a['state']=='WSL_MOVED' for a in ledger['attempts'])
old=(root/'deployed_commit.txt').read_text().strip();assert old!=meta['commit']
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
dep=json.loads((root/'deployment.json').read_text())
for rel,digest in dep['files'].items():assert sha(root/rel)==digest,rel
old_records=root/'revision_records'/old;old_records.mkdir(parents=True,exist_ok=False)
for name in ('deployment.json','test_gate.json','deployed_commit.txt'):shutil.copy2(root/name,old_records/name)
staged=root/('staged_'+meta['commit'][:7]);staged.mkdir(exist_ok=False)
for archive_name,digest,base_dir in [(meta['source_archive'],meta['source_archive_sha256'],staged),(meta['reference_archive'],meta['reference_archive_sha256'],root)]:
    assert sha(root/archive_name)==digest
    with tarfile.open(root/archive_name) as tf:
        for member in tf.getmembers():
            dst=base_dir/member.name
            expected=base_dir/('source' if base_dir==staged else 'planned_references')
            assert dst.resolve().is_relative_to(expected) and (member.isdir() or member.isfile())
            if member.isdir():dst.mkdir(parents=True,exist_ok=True)
            else:
                dst.parent.mkdir(parents=True,exist_ok=True)
                with dst.open('xb') as f:f.write(tf.extractfile(member).read())
                if member.mode&0o111:dst.chmod(0o755)
source=root/'source';saved=root/('source_'+old[:7]);assert not saved.exists()
assert source.resolve().parent==root and (staged/'source').resolve().parent==staged
source.rename(saved);(staged/'source').rename(source)
files={{rel:digest for rel,digest in dep['files'].items() if not rel.startswith('source/')}}
files.update({{p.relative_to(root).as_posix():sha(p) for p in source.rglob('*') if p.is_file()}})
(root/'deployment.json').write_text(json.dumps(dict(source_commit=meta['commit'],files=files),indent=2))
(root/'deployed_commit.txt').write_text(meta['commit']+'\n')
(root/'test_gate.json').write_text(json.dumps(meta['test_gate'],indent=2))
for row in ledger['planned_runs']:
    if row['side']=='right' and row['event_cap']==3 and not any(a['run_id']==row['run_id'] for a in ledger['attempts']):row['reference']={variant!r}+'_right'
assert not any(row['run_id']=={replacement!r} for row in ledger['planned_runs'])
ledger['planned_runs'].append(dict(run_id={replacement!r},side='left',pair=1,event_cap=3,reference={variant!r}+'_left',replacement_for='codex-time-recovery-large-live-g03-left-r01'))
ledger['revision_reason']=('P00 0.2697 m overshoot: keep target tolerance and allow 4 m settling' if {variant!r}=='g03_s4' else 'S00 yaw swing: offset original command path while retaining measured goal and all guards')
if {variant!r}=='g03_final':
    assert ledger['maximum_attempts']==6 and len(ledger['attempts'])==4
    ledger['maximum_attempts']=8
    ledger['budget_amendment']='Preserve four calibration attempts; allow revised three-event left/right, then conditional five-event left/right. No seven-event driving in this task.'
    ledger['revision_reason']='Keep successful P00/S00; move P02 beyond observed speed transient to preparation 235 m, release 245 m'
(root/'campaign_20260914.json').write_text(json.dumps(ledger,indent=2))
print(json.dumps(dict(status='REVISION_DEPLOYED',old_source_preserved=str(saved),source_commit=meta['commit'],files=len(files))))
''')


def status() -> None:
    remote(rf'''
from pathlib import Path
import json,shutil,subprocess
root=Path({ROOT!r});ledger=json.loads((root/'campaign_20260914.json').read_text());reports=[]
for a in ledger['attempts']:
    p=root/a['run_id'];r={{}};c={{}}
    if (p/'result.json').exists():r=json.loads((p/'result.json').read_text());c=r.get('last_control',{{}})
    elif (p/'MOVED_TO_WSL.json').exists():r=json.loads((p/'MOVED_TO_WSL.json').read_text())['result'];c=r.get('last_control',{{}})
    elif (p/'control_heartbeat.json').exists():c=json.loads((p/'control_heartbeat.json').read_text())
    state=c.get('large_recovery',{{}}).get('state',{{}})
    reports.append(dict(run_id=a['run_id'],ledger_state=a['state'],status=r.get('status'),fault=c.get('fault'),error=r.get('error'),phase=c.get('phase'),progress_m=(c.get('projection') or {{}}).get('s_m'),speed_mps=c.get('speed_mps'),large_state=state,stop_confirmed=c.get('stop_confirmed'),closed_bag=r.get('nodes',{{}}).get('closed_bag'),transfer_ready=(p/'transfer_manifest.json').exists(),cleanup_errors=r.get('cleanup_errors')))
print(json.dumps(dict(runs=reports,free_gib=shutil.disk_usage(root).free/2**30,containers=subprocess.check_output(['docker','ps','--format','{{{{.Names}}}}'],text=True).splitlines())))
''')


def ship(pair: int, prefix: str) -> None:
    # Reuse the verified transport with all campaign-specific roots replaced.
    # No old start/collect/finalizer is called.
    assert ROOT.endswith('/time_recovery_large_live_20260915')
    transport.ROOT=ROOT;transport.OUT=OUT;transport.RAW=RAW;transport.UNC=UNC
    from functools import partial
    transport.copy_to_host=partial(transport.copy_to_host,destination=ROOT)
    transport.ship(pair,prefix=prefix)


def finish_verified_transfer(prefix: str) -> None:
    receipt=json.loads((UNC/(prefix+'_verified.json')).read_bytes())
    snapshot=receipt['snapshot_sha256'];names=[r['run_id'] for r in receipt['runs']]
    transport.copy_to_host([UNC/(prefix+'_verified.json')],destination=ROOT)
    remote(rf'''
from pathlib import Path
import hashlib,json,subprocess,shutil
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
(root/'campaign_20260914.json').write_text(json.dumps(ledger,indent=2)+'\n')
# Correct only the newly misplaced receipt from the first transfer attempt.
misplaced=root.parent/'time_recovery_separated_20260915'/(prefix+'_verified.json')
if misplaced.exists():
    assert misplaced.resolve().parent==root.parent/'time_recovery_separated_20260915'
    assert misplaced.read_bytes()==(root/(prefix+'_verified.json')).read_bytes()
    misplaced.unlink()
print(json.dumps(dict(status='VERIFIED_TRANSFER_COMPLETE',runs=names,free_gib=shutil.disk_usage(root).free/2**30)))
''',timeout=700)


def promote_to_five() -> None:
    remote(rf'''
from pathlib import Path
import hashlib,json,subprocess
out=Path({OUT!r});raw=Path({RAW!r})
names=['codex-time-recovery-large-live-g03-left-r04','codex-time-recovery-large-live-g03-right-r03']
receipts=[json.loads((out/(prefix+'_verified.json')).read_text()) for prefix in ('large_g03_final','large_g03_right_r03')]
assert all(r['all_files_and_directory_structure_identical'] and r['all_sqlite_quick_checks_passed'] for r in receipts)
assert set(names).issubset({{r['run_id'] for receipt in receipts for r in receipt['runs']}})
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
summaries={{}}
for name in names:
    p=out/(name+'_collection_summary.json');r=json.loads(p.read_text())
    assert r['increase_eligible'] and r['event_cap']==3 and len(r['events'])==3
    assert r['result_status']=='COMPLETE_LAP' and r['fault'] is None and r['closed_bag'] and r['stop_confirmed']
    assert all(e['completed'] and e['stable_at_end'] and e['accepted_camera_anchors']>=60 and e['target_band_camera_anchors']>0 for e in r['events'])
    summaries[name]=dict(sha256=sha(p),accepted=r['accepted'],target_band_anchors=r['target_band_anchors'])
gate=dict(expansion_allowed=True,previous_event_cap=3,next_event_cap=5,summary_evidence=summaries,
    source_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
    five_reference_sha256={{side:sha(out/('g05_final_'+side)/(side+'.json')) for side in ('left','right')}},
    new_teacher_samples=sum(r['accepted'] for r in summaries.values()),maximum_event_cap_this_task=5)
with (out/'expansion_gate.json').open('x') as f:json.dump(gate,f,indent=2)
print(json.dumps(gate))
''',native=True,lock=True)
    transport.copy_to_host([UNC/'expansion_gate.json'],destination=ROOT)
    remote(rf'''
from pathlib import Path
import hashlib,json,subprocess
root=Path({ROOT!r});gate=json.loads((root/'expansion_gate.json').read_text());p=root/'campaign_20260914.json';v=json.loads(p.read_text())
assert gate['expansion_allowed'] and gate['next_event_cap']==5
assert v['maximum_attempts']==8 and len(v['attempts'])==7 and all(r['state']=='WSL_MOVED' for r in v['attempts'])
assert not subprocess.check_output(['docker','ps','-q'],text=True).strip()
assert (root/'deployed_commit.txt').read_text().strip()==gate['source_commit']
for side in ('left',):
    assert hashlib.sha256((root/'planned_references'/('g05_final_'+side)/(side+'.json')).read_bytes()).hexdigest()==gate['five_reference_sha256'][side]
    name='codex-time-recovery-large-live-g05-'+side+'-r01';assert not any(r['run_id']==name for r in v['planned_runs'])
    v['planned_runs'].append(dict(run_id=name,side=side,pair=2,event_cap=5,reference='g05_final_'+side))
v['expansion_gate_sha256']=hashlib.sha256((root/'expansion_gate.json').read_bytes()).hexdigest()
tmp=p.with_suffix('.pending');tmp.write_text(json.dumps(v,indent=2));tmp.replace(p)
print(json.dumps(dict(status='FIVE_EVENTS_AUTHORIZED_BY_VERIFIED_COLLECTION_RESULT',remaining_runs=1)))
''')


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('mode',choices=['setup','start','status','ship','revise','promote']);ap.add_argument('--run-id');ap.add_argument('--pair',type=int);ap.add_argument('--prefix');ap.add_argument('--variant',default='g03_s4');ap.add_argument('--replacement',default='codex-time-recovery-large-live-g03-left-r02');args=ap.parse_args()
    if args.mode=='setup':setup()
    elif args.mode=='start':start(args.run_id)
    elif args.mode=='status':status()
    elif args.mode=='revise':deploy_settling_revision(args.variant,args.replacement)
    elif args.mode=='promote':promote_to_five()
    else:ship(args.pair,args.prefix)
