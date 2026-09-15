"""Wait for this run to close, then move only this campaign's resident pair."""
import argparse
import ops60 as m

ap=argparse.ArgumentParser();ap.add_argument('--run',required=True);ap.add_argument('--pair',type=int,required=True);ap.add_argument('--prefix',required=True);a=ap.parse_args()
assert a.run.startswith('codex-time-recovery-60cm-')
m.remote('ROOT='+repr(m.ROOT)+'\nNAME='+repr(a.run)+'\n'+r'''
from pathlib import Path
import json,time,subprocess
root=Path(ROOT);p=root/NAME;deadline=time.monotonic()+900
while time.monotonic()<deadline:
 if (p/'result.json').exists() and (p/'transfer_manifest.json').exists() and not subprocess.check_output(['docker','ps','-q'],text=True).strip():
  r=json.loads((p/'result.json').read_text());assert r['nodes']['closed_bag']
  print(json.dumps(dict(run=NAME,status=r['status'],fault=r['last_control'].get('fault'))));break
 time.sleep(5)
else:raise TimeoutError('Live run did not close before bounded shipping wait')
''',timeout=920)
m.ship(a.pair,a.prefix)
