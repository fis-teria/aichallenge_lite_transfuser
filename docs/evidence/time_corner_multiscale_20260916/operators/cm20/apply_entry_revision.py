"""Apply the tested source and immutable future plans before any new run."""
from pathlib import Path
import subprocess,sys
import ops_corner as m
assert (m.UNC/'test_gate_21ccfef.json').exists()
subprocess.run([sys.executable,'-u','tmp/time_corner_multiscale40_20260916/diagnose_c07.py'],check=True)
m.remote((m.HERE/'prepare_e2.py').read_text(),native=True,lock=True,timeout=900)
for cm in (40,60):
 subprocess.run([sys.executable,'-u',f'tmp/time_corner_multiscale{cm}_20260916/ops_corner.py','revise'],check=True)
print('ENTRY_REVISION_DEPLOYED_BOTH_CAMPAIGNS',flush=True)
