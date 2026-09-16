"""Native regression gate for the preparation-only interpolation."""
import ops_corner as m

m.remote(r'''
from pathlib import Path
import subprocess,sys,json,hashlib
expected='1b845eddad371c98aeb54d1176fd6493cc9c6ed4'
assert subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()==expected
assert not subprocess.check_output(['git','status','--porcelain'],text=True).strip()
out=Path('/home/thistle/e2e_autonomous/runs/time_corner_gap2_20260916')
for name,extra in [('focused_1b845ed.log',['tests/test_time_large_recovery_reference_v1.py','tests/test_time_large_recovery_v1.py','tests/test_time_corner_recovery_v1.py']),('full_1b845ed.log',[])]:
 log=out/name
 with log.open('x') as f:p=subprocess.run([sys.executable,'-m','pytest','-q',*extra],stdout=f,stderr=subprocess.STDOUT,timeout=1800)
 print(log.read_text()[-3500:],flush=True)
 if p.returncode:raise RuntimeError('TEST_FAILED:'+name)
with (out/'test_gate_1b845ed.json').open('x') as f:json.dump(dict(commit=expected,full_exit=0,log=str(log),log_sha256=hashlib.sha256(log.read_bytes()).hexdigest()),f,indent=2)
print('DELAY_SOURCE_FULL_TEST_PASS',expected,flush=True)
''',native=True,lock=True,timeout=2100)
