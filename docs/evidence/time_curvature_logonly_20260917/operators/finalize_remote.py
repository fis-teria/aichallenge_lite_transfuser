"""Seal the completed owned trial, preserving previous host work and containers."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess
import tarfile

root = Path('/home/graneple/e2e_autonomous/time_curvature_logonly_20260917')
run = root/'codex-time-curve15-logonly-lap03'
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
post_path = root/'post_environment.json'
if post_path.exists():
    previous_post = json.loads(post_path.read_text())
    assert previous_post['status'] == 'PASS'
    for key in ['repo_head','repo_status','repo_diff_sha256','rviz_sha256','preserved',
                'historical_containers_preserved','historical_compose_preserved','running_containers']:
        assert post[key] == previous_post[key], key
else:
    with post_path.open('x') as stream:
        json.dump(post,stream,indent=2)
sim=repo/'aichallenge/simulator/AWSIM'
baseline_path=root/'awsim_before.json'
baseline_bytes=baseline_path.read_bytes()
before=json.loads(baseline_bytes)
after={p.relative_to(sim).as_posix():dict(bytes=p.stat().st_size,sha256=sha(p)) for p in sorted(sim.rglob('*')) if p.is_file()}
assert before['files']==after, 'AWSIM files changed during this trial'
with (root/'awsim_reference_snapshot.json').open('xb') as stream:stream.write(baseline_bytes)
with (root/'awsim_preserved.json').open('x') as stream:
    json.dump(dict(status='PASS',files=len(after),bytes=sum(v['bytes'] for v in after.values()),all_files_equal=True,
        scope='FRESH_PRE_POST_TRIAL_HASH_EQUALITY',baseline_path=str(baseline_path),baseline_sha256=sha(baseline_path),
        fresh_full_pre_run_snapshot_available=True,pre_run_critical_asset_hashes=host['steering_asset_sha256']),stream,indent=2)
print(json.dumps(dict(status='ENVIRONMENT_PRESERVED',post=post)),flush=True)
