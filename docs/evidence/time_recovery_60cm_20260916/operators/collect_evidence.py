"""Copy only compact reports and operators; raw sensors and arrays stay in WSL."""
from pathlib import Path
import hashlib,json,shutil,subprocess
import ops60 as m

dest=m.REPO/'docs/evidence/time_recovery_60cm_20260916'
dest.mkdir(exist_ok=False,parents=True)
def copy(src,rel):
    p=dest/rel
    assert p.resolve().is_relative_to(dest.resolve()) and not p.exists()
    p.parent.mkdir(exist_ok=True,parents=True)
    shutil.copyfile(src,p)
    assert hashlib.sha256(src.read_bytes()).digest()==hashlib.sha256(p.read_bytes()).digest()

names=('collection_index.json','initial_map_screen.json','reused_runtime_proof.json','test_gate.json',
       'common_assets.json','host_before.json','host_final_checks.json','campaign_final.json',
       'accepted_recovery_anchors.png','accepted_recovery_anchors_provenance.json',
       'third_site_heading_screen.json','parallel_capacity_profile.json')
for name in names:copy(m.UNC/name,name)
patterns=('*_collection_summary.json','*_anchor_states.json','*_bag_audit.json','*_clock.json','*_diagnosis.json',
          '*_verified.json','*_cleanup.json','*_plan.json','*_generation_result.json','*_generate.log','*candidate_screen.json')
for pattern in patterns:
    for p in sorted(m.UNC.glob(pattern)):copy(p,p.name)
for name in ('latency_live_gate.json','prior_synthetic/synthetic_smoke_result.json'):
    src=m.UNC/name
    if not src.exists():
        src.parent.mkdir(exist_ok=True,parents=True)
        subprocess.run(['scp',m.HOST+':'+m.ROOT+'/'+name,str(src)],check=True,timeout=60)
    copy(src,name)
gate=json.loads((m.UNC/'test_gate.json').read_text())
source_log=Path(r'\\wsl.localhost\Ubuntu-22.04-Recovered')/gate['log'].lstrip('/')
assert hashlib.sha256(source_log.read_bytes()).hexdigest()==gate['log_sha256']
copy(source_log,'full_runtime_tests.log')
for p in sorted(m.HERE.glob('*.py')):
    copy(p,'operators/'+p.name)
for rel in ('tmp/time_recovery_large_live_20260915/manage_live.py',
            'tmp/time_recovery_separated_20260915/manage.py',
            'tmp/time_ten_site_recovery_plan_20260915/sync_native_transport.py'):
    copy(m.REPO/rel,'operator_dependencies/'+rel)
for p in sorted(m.UNC.glob('rviz_*.png')):copy(p,p.name)
for p in sorted(m.UNC.glob('rviz_*_provenance.json')):copy(p,p.name)
with (dest/'.gitattributes').open('x',newline='\n') as f:f.write('* -text\n')
files={p.relative_to(dest).as_posix():dict(bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in sorted(dest.rglob('*')) if p.is_file()}
assert not any(p.endswith(('.db3','.npz','.npy','.tar.gz','.xwd','.pt','.pth')) for p in files)
with (dest/'manifest.json').open('x',newline='\n') as f:json.dump(dict(schema='compact_60cm_recovery_evidence_v1',files=files,total_bytes=sum(v['bytes'] for v in files.values()),raw_and_training_arrays_included=False),f,indent=2)
print(json.dumps(dict(evidence=str(dest),files=len(files),bytes=sum(v['bytes'] for v in files.values()))))
