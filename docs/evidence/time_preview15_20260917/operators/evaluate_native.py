"""Verify the sealed transfer and evaluate it under the native WSL worktree lock."""
from pathlib import Path
import hashlib
import json
import subprocess
import sys
import tarfile

root=Path('/home/thistle/e2e_autonomous/runs/time_preview15_20260917')
run_id='codex-time-timepreview15-lap01'


def sha(path: Path) -> str:
    digest=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


shipping=json.loads((root/'shipping.json').read_text())
archive=root/shipping['archive']
assert archive.stat().st_size==shipping['archive_bytes']
assert sha(archive)==shipping['archive_sha256']
raw=root/'raw';raw.mkdir(exist_ok=False)
run=raw/run_id
with tarfile.open(archive) as tar:
    for member in tar.getmembers():
        path=(raw/member.name).resolve()
        assert (path==run or run in path.parents) and (member.isdir() or member.isfile()),member.name
    tar.extractall(raw)
entries=json.loads((run/'transfer_manifest.json').read_text())
assert len(entries)==shipping['raw_files']
assert {r['path'] for r in entries}=={p.relative_to(run).as_posix() for p in run.rglob('*') if p.is_file() and p.name!='transfer_manifest.json'}
for entry in entries:
    path=run/entry['path']
    assert path.stat().st_size==entry['bytes'] and sha(path)==entry['sha256'],path
receipt=dict(status='PASS',files=len(entries),bytes=sum(r['bytes'] for r in entries),archive_sha256=sha(archive))
assert receipt['bytes']==shipping['raw_bytes']
with (root/'transfer_verification.json').open('x') as stream:
    json.dump(receipt,stream,indent=2)
print(json.dumps(receipt),flush=True)
command=[sys.executable,'tools/evaluate_time_awsim_trial.py','--run',str(run),'--output',str(root/'evaluation')]
with (root/'evaluation.log').open('x') as log:
    p=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,timeout=240)
(root/'evaluation_exit.json').write_text(json.dumps(dict(exit=p.returncode,command=command),indent=2))
if p.returncode:
    print((root/'evaluation.log').read_text())
    raise SystemExit(p.returncode)
print((root/'evaluation/summary.json').read_text())
