"""Seal the finite campaign after native teacher audit and verified transfer."""
import hashlib
import json
import ops_corner as m

index_path = m.UNC / 'collection_index.json'
index = json.loads(index_path.read_bytes())
digest = hashlib.sha256(index_path.read_bytes()).hexdigest()
assert index['all_raw_file_hashes_and_structure_verified'] and index['all_sqlite_quick_checks_passed']
proof = m.remote(m.PREFIX + 'INDEX_SHA=' + repr(digest) + '\nRUN_IDS=' + repr([r['run_id'] for r in index['runs']]) + '\n' + r'''
from pathlib import Path
import hashlib,json,subprocess,shutil,time
root=Path(ROOT);repo=Path('/home/graneple/git/autononous_ai/aichallenge-racingkart')
assert root.resolve()==root and root.parent==Path('/home/graneple/e2e_autonomous')
assert not subprocess.check_output(['docker','ps','-q'],text=True).strip()
processes=subprocess.check_output(['ps','-eo','pid,args'],text=True).splitlines()
assert not [p for p in processes if ROOT in p and 'run_time_recovery_awsim.py' in p]
ledger=json.loads((root/'campaign_20260914.json').read_bytes())
assert len(ledger['attempts'])<=ledger['maximum_attempts']==12
assert not ledger['sealed'] or len(ledger['attempts'])==ledger['maximum_attempts']
assert {r['run_id'] for r in ledger['attempts']}==set(RUN_IDS)
assert all(r['state']=='WSL_MOVED' for r in ledger['attempts'])
assert not list(root.glob('codex-time-recovery-corner60-*/bag/*.db3'))
before=json.loads((root/'host_before.json').read_bytes())
head=subprocess.check_output(['git','-C',str(repo),'rev-parse','HEAD'],text=True).strip()
status=hashlib.sha256(subprocess.check_output(['git','-C',str(repo),'status','--porcelain'])).hexdigest()
assert head==before['head'] and status==before['git_status_sha256']
def sha(path):
 h=hashlib.sha256()
 with path.open('rb') as stream:
  for block in iter(lambda:stream.read(8*1024*1024),b''):h.update(block)
 return h.hexdigest()
sim=repo/'aichallenge/simulator/AWSIM'
files={p.relative_to(sim).as_posix():dict(bytes=p.stat().st_size,sha256=sha(p)) for p in sorted(sim.rglob('*')) if p.is_file()}
assert files==before['awsim_files']
report=dict(status='CORNER_CAMPAIGN_SEALED',original_repo_unchanged=True,
 awsim_files_unchanged=True,awsim_files=len(files),no_running_containers=True,
 no_owned_supervisors=True,new_raw_removed_only_after_verified_transfer=True,
 collection_index_sha256=INDEX_SHA,attempts=len(RUN_IDS),free_gib=shutil.disk_usage(root).free/2**30,
 source_commit=(root/'deployed_commit.txt').read_text().strip(),finished_unix_s=time.time())
with (root/'host_final_checks.json').open('x') as f:json.dump(report,f,indent=2)
ledger['sealed']=True;ledger['collection_index_sha256']=INDEX_SHA
with (root/'campaign_final.json').open('x') as f:json.dump(ledger,f,indent=2)
(root/'campaign_20260914.json').write_text(json.dumps(ledger,indent=2))
print(json.dumps(report),flush=True)
''', timeout=180)
(m.HERE / 'host_final_checks.json').write_text(proof)
