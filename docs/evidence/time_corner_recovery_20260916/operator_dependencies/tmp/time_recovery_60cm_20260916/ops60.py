"""Operators for the isolated 60 cm campaign, exactly three sites per lap."""
from pathlib import Path
import sys

REPO = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(REPO/'tmp/time_recovery_large_live_20260915'))
import manage_live as previous

remote = previous.remote
HOST = 'graneple@192.168.3.10'
ROOT = '/home/graneple/e2e_autonomous/time_recovery_60cm_20260916'
OUT = '/home/thistle/e2e_autonomous/runs/time_recovery_60cm_20260916'
RAW = '/home/thistle/e2e_autonomous/raw/time_recovery_60cm_20260916'
UNC = Path(r'\\wsl.localhost\Ubuntu-22.04-Recovered')/OUT.lstrip('/')
RUNTIME_COMMIT = 'adfc444818a26bae021d463cca5312b5d37a9f9a'

def copy_to_host(files):
    previous.transport.copy_to_host(files, destination=ROOT)

def start(name):
    assert name.startswith('codex-time-recovery-60cm-d60-g03-')
    remote('ROOT='+repr(ROOT)+'\nNAME='+repr(name)+'\n'+r'''
from pathlib import Path
import hashlib,json
root=Path(ROOT);sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
ledger=json.loads((root/'campaign_20260914.json').read_text())
planned=next(r for r in ledger['planned_runs'] if r['run_id']==NAME)
assert planned['event_cap']==3 and not planned.get('deferred',False)
provenance=json.loads((root/'reused_runtime_proof.json').read_text())
assert provenance['runtime_source_unchanged'] and provenance['source_commit']==(root/'deployed_commit.txt').read_text().strip()
for rel,digest in provenance['proof_hashes'].items():assert sha(root/rel)==digest
ref=json.loads((root/'planned_references'/planned['reference']/(planned['side']+'.json')).read_text())
assert ref['large_recovery']['map_screen_pass']
assert all(abs(s['target_offset_m'])==.6 for s in ref['large_recovery']['config']['sites'])
''')
    previous.ROOT=ROOT; previous.OUT=OUT; previous.RAW=RAW; previous.UNC=UNC
    previous.start(name)

def ship(pair,prefix):
    from functools import partial
    t=previous.transport
    t.ROOT=ROOT; t.OUT=OUT; t.RAW=RAW; t.UNC=UNC
    t.copy_to_host=partial(t.copy_to_host,destination=ROOT)
    t.ship(pair,prefix=prefix)
