"""Stage the finished pair while WSL training owns its worktree lock."""
from functools import partial
import json
from pathlib import Path
import ops as m

t=m.previous.previous.transport
t.ROOT=m.ROOT
t.OUT='/home/thistle/e2e_autonomous/runs/time_recovery_parallel_20260916'
t.RAW='/home/thistle/e2e_autonomous/raw/time_recovery_parallel_20260916'
t.UNC=Path(r'\\wsl.localhost\Ubuntu-22.04-Recovered')/t.OUT.lstrip('/')
prefix='parallel_pair02_20260916'
receipt=json.loads(m.remote(m.PREFIX+'PREFIX='+repr(prefix)+'\n'+r'''
from pathlib import Path
import json,importlib.util
root=Path(ROOT);spec=importlib.util.spec_from_file_location('move',str(root/'move_pair.py'))
move=importlib.util.module_from_spec(spec);spec.loader.exec_module(move)
ledger=json.loads((root/'campaign_20260914.json').read_text())
names=[r['run_id'] for r in ledger['attempts'] if r['state']!='WSL_MOVED']
assert len(names)==2 and all(r['pair']==2 for r in ledger['attempts'] if r['run_id'] in names)
move.REMOTE=root;move.pack(root,PREFIX,names)
''',timeout=600))
t.copy_to_wsl([prefix+'.tar.gz',prefix+'_snapshot.json',prefix+'_shipping.json'])
(m.HERE/'pair02_staged.json').write_text(json.dumps(receipt,indent=2))
print('PAIR02_STAGED_ORIGINALS_PRESERVED_UNTIL_NATIVE_VERIFICATION')
