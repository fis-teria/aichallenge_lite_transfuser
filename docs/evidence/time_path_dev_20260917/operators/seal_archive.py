"""Resume packaging after environment verification; preserve latest log aliases."""
from pathlib import Path
import hashlib
import json
import subprocess
import tarfile

root=Path('/home/graneple/e2e_autonomous/time_path_dev_20260917')
run=root/'codex-time-dev-lap01'
assert json.loads((root/'post_environment.json').read_text())['status']=='PASS'
assert json.loads((root/'awsim_preserved.json').read_text())['all_files_equal']
assert not subprocess.check_output(['docker','ps','-q']).strip()
host=json.loads((run/'host_result.json').read_text())
assert not host['cleanup_errors']
def sha(p):
    digest=hashlib.sha256()
    with p.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
    return digest.hexdigest()
aliases=[]
for p in sorted(run.rglob('*')):
    if not p.is_symlink():continue
    resolved=p.resolve(strict=True)
    assert resolved.is_file() and resolved.is_relative_to(run) and not resolved.is_symlink()
    aliases.append(dict(path=p.relative_to(run).as_posix(),target=str(p.readlink()),
        included_target=resolved.relative_to(run).as_posix(),sha256=sha(resolved)))
with (run/'archive_aliases.json').open('x') as f:json.dump(aliases,f,indent=2)
entries=[]
for p in sorted(run.rglob('*')):
    if p.is_symlink():continue
    if p.is_file():entries.append(dict(path=p.relative_to(run).as_posix(),bytes=p.stat().st_size,sha256=sha(p)))
with (run/'transfer_manifest.json').open('x') as f:json.dump(entries,f,indent=2)
archive=root/'raw_trial.tar.gz';assert not archive.exists()
with tarfile.open(archive,'w:gz',compresslevel=1) as tar:
    tar.add(run,arcname=run.name,recursive=False)
    for p in sorted(run.rglob('*')):
        if not p.is_symlink():tar.add(p,arcname=run.name+'/'+p.relative_to(run).as_posix(),recursive=False)
receipt=dict(status='SEALED',archive=archive.name,archive_bytes=archive.stat().st_size,archive_sha256=sha(archive),
    raw_files=len(entries),raw_bytes=sum(r['bytes'] for r in entries),host_status=host['status'],
    runner_exit=json.loads((root/'runner_exit.json').read_text())['exit'],preserved_aliases=len(aliases))
with (root/'shipping.json').open('x') as f:json.dump(receipt,f,indent=2)
print(json.dumps(receipt))
