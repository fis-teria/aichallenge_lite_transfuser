from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'time_corner_multiscale60_20260916'))
import ops_corner as transport

transport.remote(r'''
from pathlib import Path
import subprocess,sys,json,hashlib
expected='d591c8f94d384df63748b3132b53c0c74900bf49'
assert subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()==expected
assert not subprocess.check_output(['git','status','--porcelain'],text=True).strip()
out=Path('/home/thistle/e2e_autonomous/runs/time_corner_gap_20260916')
for name,extra in [('focused_d591c8f.log',['tests/test_time_recovery_map_body_v1.py','tests/test_time_large_recovery_reference_v1.py','tests/test_time_corner_recovery_v1.py','tests/test_time_large_recovery_v1.py']),('full_d591c8f.log',[])]:
 log=out/name
 with log.open('x') as f:p=subprocess.run([sys.executable,'-m','pytest','-q',*extra],stdout=f,stderr=subprocess.STDOUT,timeout=1800)
 print(log.read_text()[-3500:],flush=True)
 if p.returncode:raise RuntimeError('TEST_FAILED:'+name)
with (out/'test_gate_d591c8f.json').open('x') as f:json.dump(dict(commit=expected,full_exit=0,log=str(log),log_sha256=hashlib.sha256(log.read_bytes()).hexdigest()),f,indent=2)
print('GAP_SOURCE_FULL_TEST_PASS',expected,flush=True)
''',native=True,lock=True,timeout=2100)
