"""Seal the completed owned trial, preserving previous host work and containers."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess
import tarfile

root = Path('/home/graneple/e2e_autonomous/time_multiscale_model_lap_20260916')
run = root/'codex-time-multiscale-lap01'
repo = Path('/home/graneple/git/autononous_ai/aichallenge-racingkart')


def sha(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def command(args: list[str]) -> str:
    return subprocess.check_output(args, cwd=repo, text=True, timeout=20).strip()


host = json.loads((run/'host_result.json').read_text())
runner = json.loads((root/'runner_exit.json').read_text())
assert runner['exit'] in (0, 1), runner
assert not host['cleanup_errors'], host['cleanup_errors']
assert not command(['docker', 'ps', '-q'])
pre = json.loads((root/'pre_environment.json').read_text())
rviz = repo/'aichallenge/workspace/src/aichallenge_system/aichallenge_system_launch/config/autoware.rviz'
rviz_before = run/'autoware.rviz.before'
assert sha(rviz_before) == pre['rviz_sha256']
restored = False
if sha(rviz) != pre['rviz_sha256']:
    assert sha(rviz) == host['rviz_sha256'], 'RViz changed independently; do not overwrite'
    rviz.write_bytes(rviz_before.read_bytes())
    restored = True
post = dict(repo_head=command(['git','rev-parse','HEAD']), repo_status=command(['git','status','--porcelain']),
            repo_diff_sha256=hashlib.sha256(subprocess.check_output(['git','diff','--binary','HEAD'],cwd=repo)).hexdigest(),
            rviz_sha256=sha(rviz), restored_owned_rviz_change=restored,
            free_bytes=shutil.disk_usage(root).free)
post['preserved'] = {k: pre[k] == post[k] for k in ['repo_head','repo_status','repo_diff_sha256','rviz_sha256']}
assert all(post['preserved'].values()), post
before_ids = {json.loads(line)['ID'] for line in pre['containers'].splitlines()}
after_ids = set(command(['docker','ps','-a','--format','{{.ID}}']).splitlines())
assert before_ids == after_ids, (before_ids-after_ids, after_ids-before_ids)
before_compose = {r['Name']:r['ConfigFiles'] for r in json.loads(pre['compose'])}
after_compose = {r['Name']:r['ConfigFiles'] for r in json.loads(command(['docker','compose','ls','--all','--format','json']))}
assert before_compose == after_compose
post.update(status='PASS', historical_containers_preserved=len(after_ids), historical_compose_preserved=len(after_compose), running_containers=0)
with (root/'post_environment.json').open('x') as stream:
    json.dump(post,stream,indent=2)
sim=repo/'aichallenge/simulator/AWSIM'
before=json.loads((root/'awsim_before.json').read_text())
after={p.relative_to(sim).as_posix():dict(bytes=p.stat().st_size,sha256=sha(p)) for p in sorted(sim.rglob('*')) if p.is_file()}
assert before['files']==after, 'AWSIM changed during evaluation'
with (root/'awsim_preserved.json').open('x') as stream:
    json.dump(dict(status='PASS',files=len(after),bytes=sum(v['bytes'] for v in after.values()),all_files_equal=True),stream,indent=2)
entries=[]
for path in sorted(run.rglob('*')):
    assert not path.is_symlink(), path
    if path.is_file():
        entries.append(dict(path=path.relative_to(run).as_posix(),bytes=path.stat().st_size,sha256=sha(path)))
with (run/'transfer_manifest.json').open('x') as stream:
    json.dump(entries,stream,indent=2)
archive=root/'raw_trial.tar.gz'
assert not archive.exists()
with tarfile.open(archive,'w:gz') as tar:
    tar.add(run,arcname=run.name)
receipt=dict(status='SEALED',archive=archive.name,archive_bytes=archive.stat().st_size,archive_sha256=sha(archive),
             raw_files=len(entries),raw_bytes=sum(r['bytes'] for r in entries),host_status=host['status'],runner_exit=runner['exit'])
with (root/'shipping.json').open('x') as stream:
    json.dump(receipt,stream,indent=2)
print(json.dumps(dict(post=post,shipping=receipt)))
