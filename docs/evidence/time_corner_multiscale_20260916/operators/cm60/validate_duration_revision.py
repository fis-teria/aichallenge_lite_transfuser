import ops_corner as m
m.remote(r'''
from pathlib import Path
import subprocess,sys,json,hashlib
expected=subprocess.check_output(['git','rev-parse','a488ab6'],text=True).strip()
assert subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()==expected
assert not subprocess.check_output(['git','status','--porcelain'],text=True).strip()
out=Path('/home/thistle/e2e_autonomous/runs/time_corner_multiscale60_20260916')
for name,extra in [('focused_a488ab6.log',['tests/test_time_large_recovery_v1.py']),('full_a488ab6.log',[])]:
 log=out/name
 with log.open('x') as f:p=subprocess.run([sys.executable,'-m','pytest','-q',*extra],stdout=f,stderr=subprocess.STDOUT,timeout=1800)
 print(log.read_text()[-1500:],flush=True);assert p.returncode==0
with (out/'test_gate_a488ab6.json').open('x') as f:json.dump(dict(commit=expected,full_exit=0,log=str(log),log_sha256=hashlib.sha256(log.read_bytes()).hexdigest()),f,indent=2)
print('DURATION_REVISION_FULL_TEST_PASS',expected,flush=True)
''',native=True,lock=True,timeout=2100)
