"""Finish exactly the three remaining planned pairs after pair 1 validates."""
import json
from pathlib import Path
import subprocess
import sys
import time

from manage import HERE, UNC, remote

deadline=time.monotonic()+2400
while time.monotonic()<deadline:
    log=(HERE/'pair01.log').read_text()
    if 'PAIR_COLLECTION_TRANSFER_AND_AUDIT_DONE 1' in log:
        audit=json.loads((UNC/'pair01_audit.json').read_text())
        assert len(audit['runs'])==2 and all(r['bag_verified'] and not r['fault'] and r['normal_stop_confirmed']
            and r['marker_audit']['recorded_marker_array_matches'] for r in audit['runs'])
        break
    if 'Traceback (most recent call last)' in log:
        raise RuntimeError('PAIR01_FAILED_NO_FURTHER_RUNS')
    time.sleep(10)
else:
    raise TimeoutError('PAIR01_VALIDATION_WAIT')
for pair in (2,3,4):
    print('STARTING_FINITE_PAIR',pair,flush=True)
    with (HERE/f'pair{pair:02d}.log').open('x') as log:
        result=subprocess.run([sys.executable,'-u',str(HERE/'manage.py'),'collect','--pair',str(pair)],
            stdout=log,stderr=subprocess.STDOUT,timeout=7000)
    if result.returncode:
        raise RuntimeError('PAIR_FAILED_PRESERVE:'+str(pair))
    print('PAIR_VALIDATED',pair,flush=True)
print('ALL_PLANNED_PAIRS_VALIDATED',flush=True)
remote((HERE/'summarize_native.py').read_text(),native=True,lock=True,timeout=300)
print('COLLECTION_INDEX_AND_FIGURES_SAVED_IN_NATIVE_WSL',flush=True)
