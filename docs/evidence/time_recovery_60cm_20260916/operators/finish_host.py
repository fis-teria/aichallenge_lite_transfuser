"""Seal only closed, hash-verified and moved recordings and verify the original host."""
import json
import subprocess
import ops60 as m

m.copy_to_host([m.UNC/'collection_index.json'])
m.remote('ROOT='+repr(m.ROOT)+'\n'+r'''
from pathlib import Path
import hashlib,json,shutil,subprocess,time
root=Path(ROOT);sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
containers=subprocess.check_output(['docker','ps','--format','{{.Names}}'],text=True).splitlines()
assert not containers
p=root/'campaign_20260914.json';v=json.loads(p.read_text());index=json.loads((root/'collection_index.json').read_text())
assert set(r['run_id'] for r in v['attempts'])==set(r['run_id'] for r in index['runs'])
assert 1<=len(v['attempts'])<=v['maximum_attempts']==8 and all(a['state']=='WSL_MOVED' for a in v['attempts'])
assert index['all_raw_file_hashes_and_structure_verified'] and index['all_sqlite_quick_checks_passed']
for a in v['attempts']:
 r=root/a['run_id'];assert (r/'MOVED_TO_WSL.json').exists()
 assert not (r/'bag').exists() and not (r/'control.jsonl').exists()
dep=json.loads((root/'deployment.json').read_text())
for rel,digest in dep['files'].items():assert sha(root/rel)==digest,rel
common=json.loads((root/'common_assets.json').read_text());old=root.parent/'time_recovery_40cm_20260915'
for rel,digest in common.items():assert sha(old/rel)==sha(root/rel)==digest,rel
repo='/home/graneple/git/autononous_ai/aichallenge-racingkart';before=json.loads((root/'host_before.json').read_text())
head=subprocess.check_output(['git','-C',repo,'rev-parse','HEAD'],text=True).strip()
status_sha=hashlib.sha256(subprocess.check_output(['git','-C',repo,'status','--porcelain'])).hexdigest()
assert head==before['head'] and status_sha==before['git_status_sha256']
check=dict(original_repo_head=head,original_git_status_sha256=status_sha,original_repo_unchanged=True,
 source_files_verified=len(dep['files']),previous_common_files_verified=len(common),
 no_running_containers=True,new_raw_removed_only_after_verified_transfer=True,
 moved_runs=len(v['attempts']),free_bytes=shutil.disk_usage(root).free,
 collection_index_sha256=sha(root/'collection_index.json'),runtime_source_commit=dep['source_commit'],
 finalized_unix_s=time.time())
with (root/'host_final_checks.json').open('x') as f:json.dump(check,f,indent=2)
v.update(sealed=True,collection_index_sha256=check['collection_index_sha256'],finalized_unix_s=check['finalized_unix_s'])
with p.with_suffix('.pending').open('x') as f:json.dump(v,f,indent=2)
p.with_suffix('.pending').replace(p)
with (root/'campaign_final.json').open('x') as f:json.dump(v,f,indent=2)
print(json.dumps(check))
''')
for name in ('host_final_checks.json','campaign_final.json'):
    target=m.UNC/name;assert not target.exists()
    subprocess.run(['scp',m.HOST+':'+m.ROOT+'/'+name,str(target)],check=True,timeout=60)
