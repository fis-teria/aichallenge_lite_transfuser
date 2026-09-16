"""Resume the verified-copy phase after the WSL sync lock rejected its launch."""
from functools import partial
import json
import ops_corner as m

prefix = 'corner40_pair01_20260916'
receipt = json.loads((m.UNC / (prefix + '_shipping.json')).read_text())
names = receipt['run_ids']
snapshot = receipt['snapshot_sha256']
assert names == ['codex-time-recovery-corner40-p01-d1', 'codex-time-recovery-corner40-p01-d2']
m.remote('OUT='+repr(m.OUT)+'\nRAW='+repr(m.RAW)+'\nPREFIX='+repr(prefix)+'\nNAMES='+repr(names)+'\nSHA='+repr(snapshot)+'\n'+r'''
from pathlib import Path
import importlib.util
s=importlib.util.spec_from_file_location('move','docs/evidence/time_recovery_batches_20260914/move_pair.py')
move=importlib.util.module_from_spec(s);s.loader.exec_module(move)
move.ANALYSIS=Path(OUT);move.RAW=Path(RAW)
move.verify(move.ANALYSIS,PREFIX,NAMES,SHA)
''',native=True,lock=True,timeout=600)
m.transport.copy_to_host([m.UNC/(prefix+'_verified.json')],destination=m.ROOT)
m.remote(m.PREFIX+'RAW='+repr(m.RAW)+'\nPREFIX='+repr(prefix)+'\nNAMES='+repr(names)+'\nSHA='+repr(snapshot)+'\n'+r'''
from pathlib import Path
import json,subprocess,shutil
root=Path(ROOT);raw=Path(RAW);prefix=PREFIX;names=NAMES;snapshot=SHA
assert root.resolve()==root and root.parent==Path('/home/graneple/e2e_autonomous')
assert not subprocess.check_output(['docker','ps','-q'],text=True).strip()
v=json.loads((root/(prefix+'_verified.json')).read_bytes())
assert v['snapshot_sha256']==snapshot and v['all_files_and_directory_structure_identical'] and v['all_sqlite_quick_checks_passed']
ledger=json.loads((root/'campaign_20260914.json').read_bytes())
assert sorted(r['run_id'] for r in ledger['attempts'] if r['state']!='WSL_MOVED')==sorted(names)
for name in names:
 marker=root/(prefix+'_markers')/name;marker.mkdir(parents=True,exist_ok=False)
 result=json.loads((root/name/'result.json').read_bytes())
 (marker/'MOVED_TO_WSL.json').write_text(json.dumps(dict(snapshot_sha256=snapshot,raw_path=str(raw/name),result=result),indent=2)+'\n')
with (root/(prefix+'_cleanup.json')).open('x') as f:json.dump(dict(state='PREPARED',removed_runs=[],before_free_bytes=shutil.disk_usage(root).free),f,indent=2)
code="from pathlib import Path;import importlib.util;s=importlib.util.spec_from_file_location('m','/collection/move_pair.py');m=importlib.util.module_from_spec(s);s.loader.exec_module(m);m.RAW=Path("+repr(str(raw))+");m.cleanup(Path('/collection'),"+repr(prefix)+","+repr(names)+","+repr(snapshot)+")"
subprocess.run(['docker','run','--rm','--network','none','--entrypoint','python3','-v',str(root)+':/collection','codex-cartographer-v4-build:20260910','-c',code],check=True,timeout=600)
for row in ledger['attempts']:
 if row['run_id'] in names:row.update(state='WSL_MOVED',raw_path=str(raw/row['run_id']))
(root/'campaign_20260914.json').write_text(json.dumps(ledger,indent=2)+'\n')
print('PAIR_MOVED',names)
''',timeout=700)
m.transport.ROOT=m.ROOT;m.transport.OUT=m.OUT;m.transport.UNC=m.UNC
m.transport.copy_to_wsl([prefix+'_cleanup.json'])
