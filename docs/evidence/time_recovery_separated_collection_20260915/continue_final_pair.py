"""Exactly the remaining left replacement and original right; no retries."""
import json
import subprocess
import sys
import manage as m

amendment=json.loads((m.UNC/'replacement_amendment.json').read_bytes())
assert amendment['maximum_attempts']==11 and not amendment['additional_retry_on_failure']
m.collect(5)
subprocess.run([sys.executable,str(m.HERE/'assess_pair.py'),'5'],check=True)
m.remote((m.HERE/'summarize_native.py').read_text(),native=True,lock=True,timeout=180)
print('BOUNDED_SEPARATED_COLLECTION_COMPLETE',flush=True)
