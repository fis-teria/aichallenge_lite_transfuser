import ops_corner as m
m.remote(r'''
from pathlib import Path
import subprocess,sys,json,hashlib,shutil
expected='21ccfef00a415c85dd033d447cd8ed25a5ad974f'
assert subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()==expected
assert not subprocess.check_output(['git','status','--porcelain'],text=True).strip()
base=Path('/home/thistle/e2e_autonomous/runs')
out=base/'time_corner_multiscale20_20260916'
log=out/'full_21ccfef.log'
with log.open('x') as f:p=subprocess.run([sys.executable,'-m','pytest','-q'],stdout=f,stderr=subprocess.STDOUT,timeout=1800)
print(log.read_text()[-1800:],flush=True)
assert p.returncode==0
digest=hashlib.sha256(log.read_bytes()).hexdigest()
for cm in (20,40,60):
 folder=base/f'time_corner_multiscale{cm}_20260916';target=folder/log.name
 if cm!=20:shutil.copy2(log,target)
 with (folder/'test_gate_21ccfef.json').open('x') as f:json.dump(dict(commit=expected,full_exit=0,log=str(target),log_sha256=digest),f,indent=2)
print('FULL_TEST_GATE_PASS',expected,flush=True)
''',native=True,lock=True,timeout=2000)
