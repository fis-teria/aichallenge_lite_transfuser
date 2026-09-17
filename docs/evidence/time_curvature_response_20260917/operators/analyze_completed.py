"""Analyze this completed trial in locked native WSL, then export verified video."""
from pathlib import Path
import hashlib
import json
import shutil

from manage import HERE, WSL, remote


native = Path(r'\\wsl.localhost\Ubuntu-22.04-Recovered') / WSL.lstrip('/')
summary = json.loads((native / 'evaluation/summary.json').read_bytes())
assert summary['judge_lap_confirmed'] and summary['status'] == 'LAP_COMPLETED'
for name in ('inspect_native.py', 'route_native.py', 'compare_native.py',
             'verify_video_native.py', 'diagnose_stop_native.py'):
    print('ANALYZE ' + name, flush=True)
    remote((HERE / name).read_text(), host='codex-wsl', lock=True, timeout=300)

verified = json.loads((native / 'evaluation/video_verification.json').read_bytes())
assert verified['status'] == 'PASS'
destination = HERE / 'videos'
destination.mkdir(exist_ok=False)
for item in verified['videos']:
    source = native / 'raw/codex-time-curve15-response-lap04' / item['file']
    target = destination / item['file']
    shutil.copy2(source, target)
    assert target.stat().st_size == item['bytes']
    assert hashlib.sha256(target.read_bytes()).hexdigest() == item['sha256']
print(json.dumps(dict(status='COMPLETE', videos=str(destination))), flush=True)
