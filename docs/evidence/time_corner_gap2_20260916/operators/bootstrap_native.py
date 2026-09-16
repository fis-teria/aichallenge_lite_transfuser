"""Create a separate finite campaign and test the unchanged runtime in native WSL."""
from pathlib import Path
import ops_corner as m

m.remote(r'''
from pathlib import Path
import hashlib,json,subprocess,sys,shutil
out=Path('/home/thistle/e2e_autonomous/runs/time_corner_gap2_20260916')
prior=out.parent/'time_corner_gap_20260916'
expected='36b47e1d58167ed0b03580bcb3248fda5246b728'
assert subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()==expected
assert not subprocess.check_output(['git','status','--porcelain'],text=True).strip()
out.mkdir(exist_ok=False)
for name in ('plans','references'): (out/name).mkdir()
screen=json.loads((prior/'candidate_screen.json').read_bytes())
screen['missing_before']=json.loads((prior/'coverage_final.json').read_bytes())['missing_after']
screen['prior_campaign']='time_corner_gap_20260916'
screen['prior_coverage_sha256']=hashlib.sha256((prior/'coverage_final.json').read_bytes()).hexdigest()
(out/'candidate_screen.json').write_text(json.dumps(screen,indent=2))
shutil.copy2(prior/'clock_audit.py',out/'clock_audit.py')
log=out/'full_36b47e1.log'
with log.open('x') as f:p=subprocess.run([sys.executable,'-m','pytest','-q'],stdout=f,stderr=subprocess.STDOUT,timeout=1800)
print(log.read_text()[-3500:],flush=True)
assert p.returncode==0
with (out/'test_gate.json').open('x') as f:json.dump(dict(commit=expected,full_exit=0,log=str(log),log_sha256=hashlib.sha256(log.read_bytes()).hexdigest()),f,indent=2)
print('GAP2_NATIVE_READY',expected,flush=True)
''',native=True,lock=True,timeout=2100)
for name in ('audit_corner.py',):
    import shutil
    shutil.copy2(m.HERE/name,m.UNC/name)
