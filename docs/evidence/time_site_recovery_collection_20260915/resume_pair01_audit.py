from manage import remote, HERE

remote(r'''
from pathlib import Path
import json
root=Path('/home/thistle/e2e_autonomous/runs/time_recovery_sites_20260915')
assert not (root/'pair01_audit.json').exists()
backup=root/'failed_shutdown_subscriber_assumption';backup.mkdir(exist_ok=False)
for ext in ('.json','.log'):
 p=root/('codex-time-recovery-sites-g01-left_bag_audit'+ext)
 assert p.resolve().parent==root and p.is_file()
 p.rename(backup/p.name)
print('PRESERVED_FIRST_BAG_AUDIT_BEFORE_FIX',flush=True)
''',native=True,lock=True)
remote("import sys\nsys.argv=['audit_native.py','--pair','1']\n"+(HERE/'audit_native.py').read_text(),
    native=True,lock=True,timeout=2400)
print('PAIR_COLLECTION_TRANSFER_AND_AUDIT_DONE 1',flush=True)
