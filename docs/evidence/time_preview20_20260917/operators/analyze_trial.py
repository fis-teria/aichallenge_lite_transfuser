"""Analyze the sealed trial and export hash-verified recordings."""
from pathlib import Path
import hashlib
import json
import shutil
from manage import HERE, WSL, RUN_ID, remote

native=Path(r'\\wsl.localhost\Ubuntu-22.04-Recovered')/WSL.lstrip('/')
summary=json.loads((native/'evaluation/summary.json').read_bytes())
assert summary['control_replay']['status']=='PASS'
for name in ('inspect_native.py','route_native.py','compare_native.py','verify_video_native.py','diagnose_stop_native.py'):
    print('ANALYZE '+name,flush=True)
    remote((HERE/name).read_text(),host='codex-wsl',lock=True,timeout=300)
verified=json.loads((native/'evaluation/video_verification.json').read_bytes())
assert verified['status']=='PASS'
destination=HERE/'videos';destination.mkdir(exist_ok=False)
for item in verified['videos']:
    target=destination/item['file']
    shutil.copy2(native/'raw'/RUN_ID/item['file'],target)
    assert target.stat().st_size==item['bytes']
    assert hashlib.sha256(target.read_bytes()).hexdigest()==item['sha256']
print(json.dumps(dict(status='ANALYZED',trial_status=summary['status'],videos=str(destination))),flush=True)
