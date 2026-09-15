"""Close only the two owned pilot namespaces and prove AWSIM immutability."""
import json
import ops as m

text=m.remote(m.PREFIX+r'''
from pathlib import Path
import json,time,hashlib,subprocess,shutil
root=Path(ROOT);plan=json.loads((root/'parallel_plan.json').read_text())
deadline=time.monotonic()+180
while not all((root/r['run_id']/'result.json').exists() and (root/r['run_id']/'transfer_manifest.json').exists() for r in plan['instances']):
    assert time.monotonic()<deadline
    time.sleep(3)
rows=[]
for spec in plan['instances']:
    path=root/spec['run_id'];r=json.loads((path/'result.json').read_text());c=r['last_control']
    rows.append(dict(run_id=spec['run_id'],ros_domain_id=spec['ros_domain_id'],status=r['status'],error=r.get('error'),
        lap_confirmed=r['lap_confirmed'],fault=c.get('fault'),stop_confirmed=c.get('stop_confirmed'),
        stop_reason=c.get('stop_reason'),closed_bag=r['nodes']['closed_bag'],cleanup_errors=r['cleanup_errors'],
        completed_events=c['large_recovery']['state']['completed_events'],skipped_sites=c['large_recovery']['state']['skipped_sites'],
        started_unix_s=r['started_unix_s'],ended_unix_s=r['ended_unix_s'],raw_bytes=sum(f.stat().st_size for f in path.rglob('*') if f.is_file())))
repo=Path('/home/graneple/git/autononous_ai/aichallenge-racingkart');sim=repo/'aichallenge/simulator/AWSIM'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
before=json.loads((root/'host_before.json').read_text())
current={p.relative_to(sim).as_posix():dict(bytes=p.stat().st_size,sha256=sha(p)) for p in sorted(sim.rglob('*')) if p.is_file()}
assert current==before['awsim_files']
proof=dict(awsim_all_files_unchanged=True,awsim_file_count=len(current),awsim_bytes=sum(r['bytes'] for r in current.values()),
    git_head_unchanged=before['head']==subprocess.check_output(['git','-C',str(repo),'rev-parse','HEAD'],text=True).strip(),
    git_status_unchanged=before['git_status_sha256']==hashlib.sha256(subprocess.check_output(['git','-C',str(repo),'status','--porcelain'])).hexdigest(),
    runs=rows,free_gib=shutil.disk_usage(root).free/2**30)
proof['all_two_laps_complete']=all(r['status']=='COMPLETE_LAP' and r['lap_confirmed'] and r['fault'] is None and r['stop_confirmed'] and r['closed_bag'] and not r['cleanup_errors'] and r['completed_events']==3 for r in rows)
(root/'parallel_result.json').write_text(json.dumps(proof,indent=2));print(json.dumps(proof))
''',timeout=200)
(m.HERE/'parallel_result.json').write_text(text)
m.remove_networks()
