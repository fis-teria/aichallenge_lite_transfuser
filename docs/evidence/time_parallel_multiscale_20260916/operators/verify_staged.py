"""Verify native WSL files before removing the exact original pair."""
from pathlib import Path
import json
import ops as m

out='/home/thistle/e2e_autonomous/runs/time_recovery_parallel_20260916'
raw='/home/thistle/e2e_autonomous/raw/time_recovery_parallel_20260916'
unc=Path(r'\\wsl.localhost\Ubuntu-22.04-Recovered')/out.lstrip('/')
prefix='parallel_pair02_20260916'
receipt=json.loads((unc/(prefix+'_shipping.json')).read_text());names=receipt['run_ids'];snapshot=receipt['snapshot_sha256']
m.remote('OUT='+repr(out)+'\nRAW='+repr(raw)+'\nPREFIX='+repr(prefix)+'\nNAMES='+repr(names)+'\nSNAPSHOT='+repr(snapshot)+'\n'+r'''
from pathlib import Path
import importlib.util
spec=importlib.util.spec_from_file_location('move','docs/evidence/time_recovery_batches_20260914/move_pair.py')
move=importlib.util.module_from_spec(spec);spec.loader.exec_module(move)
move.ANALYSIS=Path(OUT);move.RAW=Path(RAW);move.verify(move.ANALYSIS,PREFIX,NAMES,SNAPSHOT)
''',native=True,lock=True,timeout=600)
m.previous.previous.transport.copy_to_host([unc/(prefix+'_verified.json')],destination=m.ROOT)
m.remote(m.PREFIX+'RAW='+repr(raw)+'\nPREFIX='+repr(prefix)+'\nNAMES='+repr(names)+'\nSNAPSHOT='+repr(snapshot)+'\n'+r'''
from pathlib import Path
import json,subprocess
root=Path(ROOT);raw=Path(RAW)
assert root.resolve()==root and root.parent==Path('/home/graneple/e2e_autonomous')
assert not subprocess.check_output(['docker','ps','-q'],text=True).strip()
v=json.loads((root/(PREFIX+'_verified.json')).read_text())
assert v['snapshot_sha256']==SNAPSHOT and v['all_files_and_directory_structure_identical'] and v['all_sqlite_quick_checks_passed']
ledger=json.loads((root/'campaign_20260914.json').read_text())
assert sorted(r['run_id'] for r in ledger['attempts'] if r['state']!='WSL_MOVED')==sorted(NAMES)
for name in NAMES:
 marker=root/(PREFIX+'_markers')/name;marker.mkdir(parents=True,exist_ok=False)
 result=json.loads((root/name/'result.json').read_text())
 (marker/'MOVED_TO_WSL.json').write_text(json.dumps(dict(snapshot_sha256=SNAPSHOT,raw_path=str(raw/name),result=result),indent=2)+'\n')
with (root/(PREFIX+'_cleanup.json')).open('x') as f:json.dump(dict(state='PREPARED',removed_runs=[]),f)
code="from pathlib import Path;import importlib.util;s=importlib.util.spec_from_file_location('m','/collection/move_pair.py');m=importlib.util.module_from_spec(s);s.loader.exec_module(m);m.RAW=Path("+repr(str(raw))+");m.cleanup(Path('/collection'),"+repr(PREFIX)+","+repr(NAMES)+","+repr(SNAPSHOT)+")"
subprocess.run(['docker','run','--rm','--network','none','--entrypoint','python3','-v',str(root)+':/collection','codex-cartographer-v4-build:20260910','-c',code],check=True,timeout=600)
for row in ledger['attempts']:
 if row['run_id'] in NAMES:row.update(state='WSL_MOVED',raw_path=str(raw/row['run_id']))
ledger['sealed']=True
(root/'campaign_20260914.json').write_text(json.dumps(ledger,indent=2)+'\n')
print('PAIR02_VERIFIED_AND_MOVED')
''',timeout=700)
m.previous.previous.transport.ROOT=m.ROOT
m.previous.previous.transport.UNC=unc
m.previous.previous.transport.copy_to_wsl([prefix+'_cleanup.json'])
