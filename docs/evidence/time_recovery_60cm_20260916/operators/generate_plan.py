"""Generate an immutable plan already written to the native output folder."""
import argparse
import ops60 as m
ap=argparse.ArgumentParser();ap.add_argument('--tag',required=True);ap.add_argument('--side',choices=['left','right'],required=True);a=ap.parse_args()
assert a.tag.startswith('d60_g03_'+a.side+'_')
m.remote('OUT='+repr(m.OUT)+'\nTAG='+repr(a.tag)+'\nSIDE='+repr(a.side)+'\n'+r'''
from pathlib import Path
import json,subprocess,sys
out=Path(OUT);plan=out/(TAG+'_plan.json')
cmd=[sys.executable,'tools/generate_time_large_recovery_reference.py','--inputs',str(out.parent/'time_recovery_collection_20260913/inputs'),'--normal-run','/home/thistle/e2e_autonomous/raw/time_recovery_sites_20260915/codex-time-recovery-sites-normal-n03','--normal-proof',str(out.parent/'time_recovery_separated_20260915/selected_site_plan.json'),'--plan',str(plan),'--side',SIDE,'--output',str(out/TAG)]
r=subprocess.run(cmd,capture_output=True,text=True,timeout=180)
with (out/(TAG+'_generate.log')).open('x') as f:f.write(r.stdout+r.stderr)
print(r.stdout+r.stderr)
with (out/(TAG+'_generation_result.json')).open('x') as f:json.dump(dict(command=cmd,exit=r.returncode),f,indent=2)
assert r.returncode==0
''',native=True,lock=True,timeout=210)
