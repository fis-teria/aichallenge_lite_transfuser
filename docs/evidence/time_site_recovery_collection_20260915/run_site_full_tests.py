from pathlib import Path
import hashlib
import json
import subprocess
import sys
import time

head = subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip()
root = Path('/home/thistle/e2e_autonomous/runs')
log = root / f'time_site_recovery_full_{head[:7]}.log'
gate = root / f'time_site_recovery_gate_{head[:7]}.json'
with log.open('x') as stream:
    start = time.monotonic()
    result = subprocess.run([sys.executable, '-m', 'pytest', '-q'], stdout=stream, stderr=subprocess.STDOUT)
value = dict(commit=head, full_exit=result.returncode, log=str(log),
             log_sha256=hashlib.sha256(log.read_bytes()).hexdigest(), seconds=time.monotonic()-start)
with gate.open('x') as stream:
    json.dump(value, stream, indent=2)
print(json.dumps(value), flush=True)
print(log.read_text()[-6000:], flush=True)
if result.returncode:
    raise SystemExit(result.returncode)
