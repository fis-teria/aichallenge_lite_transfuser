"""One bounded execution of the remaining four planned pairs, no retry budget reset."""
import json
import subprocess
import sys
import manage as m

gate=json.loads((m.UNC/'pilot_gate.json').read_bytes())
assert gate['expansion_allowed'] and all(r['pilot_pass'] for r in gate['events'])
for pair in (2,3,4,5):
    m.collect(pair)
    subprocess.run([sys.executable,str(m.HERE/'assess_pair.py'),str(pair)],check=True)
m.remote((m.HERE/'summarize_native.py').read_text(),native=True,lock=True,timeout=240)
print('BOUNDED_SEPARATED_COLLECTION_COMPLETE',flush=True)
