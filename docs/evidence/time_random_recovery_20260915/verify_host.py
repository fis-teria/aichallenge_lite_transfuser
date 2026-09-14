"""Read-only host restoration and deployed-byte audit after the bounded pilot."""
from pathlib import Path
import json
import runpy

ops = runpy.run_path('tools/collect_time_recovery_phases.py')
code = r'''
from pathlib import Path
import hashlib, json, shutil, subprocess, time
root = Path('/home/graneple/e2e_autonomous/time_recovery_random_confirmed_20260915')
repo = '/home/graneple/git/autononous_ai/aichallenge-racingkart'
read = lambda p: json.loads(p.read_bytes())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
before = read(root/'host_before.json')
deployment = read(root/'deployment.json')
preparation = read(root/'preparation.json')
plan = read(root/'collection_plan.json')
ledger = read(root/'campaign_20260914.json')
containers = [json.loads(s) for s in subprocess.check_output(
    ['docker','ps','-a','--format','{{json .}}'], text=True).splitlines()]
compose = json.loads(subprocess.check_output(['docker','compose','ls','--all','--format','json'], text=True))
prior = {r['ID']:r for r in before['containers']}
current = {r['ID']:r for r in containers}
fields = ('ID', 'Names', 'Image', 'Command', 'State', 'Mounts', 'Networks', 'Labels')
def comparable(row, key):
    value = row.get(key)
    # Docker ps serializes the mount collection in varying order on each call.
    # Preserve every token and its multiplicity; all other fields stay exact.
    return sorted(value.split(',')) if key == 'Mounts' and isinstance(value, str) else value
changed = [ident for ident, r in prior.items() if ident not in current or
    any(comparable(r, k) != comparable(current[ident], k) for k in fields)]
prior_compose = {r['Name']:r for r in before['compose']}
current_compose = {r['Name']:r for r in compose}
compose_changed = [name for name, row in prior_compose.items() if current_compose.get(name) != row]
source_mismatch = [rel for rel, digest in deployment['files'].items()
    if not (root/rel).is_file() or sha(root/rel) != digest]
ref_mismatch = [name for name, digest in preparation['references'].items()
    if sha(root/'references'/name) != digest]
git_head = subprocess.check_output(['git','-C',repo,'rev-parse','HEAD'], text=True).strip()
git_status = hashlib.sha256(subprocess.check_output(['git','-C',repo,'status','--porcelain'])).hexdigest()
active = subprocess.check_output(['docker','ps','-q'], text=True).splitlines()
markers = []
for row in plan['runs']:
    run = root/row['run_id']
    note = read(run/'MOVED_TO_WSL.json')
    markers.append(dict(run_id=run.name, raw_path=note['raw_path'],
        snapshot_sha256=note['snapshot_sha256'], only_marker=sorted(p.name for p in run.iterdir()) == ['MOVED_TO_WSL.json'],
        result_status=note['result']['status'], closed_bag=note['result']['nodes']['closed_bag']))
checks = dict(source_manifest_equal=not source_mismatch and len(deployment['files']) == 555,
    reference_hashes_equal=not ref_mismatch,
    source_commit_equal=deployment['source_commit'] == preparation['commit'] == '404ea7cf51097c74cb0ba0a26753fa5b8d219666',
    user_git_head_unchanged=git_head == before['head'], user_git_status_unchanged=git_status == before['git_status_sha256'],
    historical_containers_preserved=not changed, historical_compose_preserved=not compose_changed,
    no_active_containers=not active, no_extra_containers=set(current) == set(prior),
    no_extra_compose=set(current_compose) == set(prior_compose),
    plan_sealed=ledger['sealed'] and len(ledger['attempts']) == 2 and all(r['state'] == 'WSL_MOVED' for r in ledger['attempts']),
    only_exact_verified_source_markers=all(r['only_marker'] and r['closed_bag'] and r['result_status'] == 'COMPLETE_LAP' for r in markers),
    diagnostic_source_retained=(root.parent/'time_recovery_random_20260915/codex-time-recovery-random-r62/bag').is_dir())
result = dict(status='PASS' if all(checks.values()) else 'FAILED', checks=checks, checked_unix_s=time.time(),
    mount_comparison='SORTED_DOCKER_PS_TOKENS_WITH_MULTIPLICITY;_OTHER_FIELDS_EXACT',
    previous_order_sensitive_check_sha256=sha(root/'host_final.json'),
    git_head=git_head, git_status_sha256=git_status, historical_container_count=len(prior),
    current_container_count=len(current), historical_compose_count=len(prior_compose), current_compose_count=len(current_compose),
    changed_historical_containers=changed, changed_historical_compose=compose_changed,
    source_file_count=len(deployment['files']), source_mismatches=source_mismatch, reference_mismatches=ref_mismatch,
    free_bytes=shutil.disk_usage(root).free, raw_markers=markers)
with (root/'host_final_v2.json').open('x') as f:
    json.dump(result, f, indent=2)
print(json.dumps(result, indent=2))
'''
result = json.loads(ops['remote_python']('graneple@192.168.3.10', code))
destination = Path('tmp/time_random_recovery_confirmed_20260915_host_final_v2.json')
with destination.open('x', encoding='utf-8') as stream:
    json.dump(result, stream, indent=2)
print(json.dumps(result, indent=2), flush=True)
assert result['status'] == 'PASS'
