"""Preserve old reports and correct only offline publication-duration metrics."""
import json
import manage as m

gate=json.loads((m.UNC.parent/'time_site_recovery_gate_1d20998.json').read_bytes())
assert gate['full_exit']==0
m.remote(rf'''
from pathlib import Path
import hashlib,json
out=Path({m.OUT!r});before={{}}
for name in ('pilot_gate.json','pair02_holds.json'):
 p=out/name;destination=out/('before_duplicate_clock_fix_'+name)
 assert p.resolve().parent==destination.resolve().parent==out and not destination.exists()
 before[name]=hashlib.sha256(p.read_bytes()).hexdigest();p.rename(destination)
v=dict(reason='Identical /clock stamps of publications 5322 and 5323 were incorrectly treated as a continuity break.',
 affected_run='codex-time-recovery-separated-g02-right',event_id=3,observed_wall_interval_ms=7.30159,
 before_sha256=before,old_wrong_duration_s=.749999983,expected_correct_duration_s=1.039999976,
 runtime_changed=False,raw_or_teachers_changed=False,qualification_threshold_changed=False,
 test_gate={gate!r})
with (out/'hold_audit_correction.json').open('x') as f:json.dump(v,f,indent=2)
print('OLD_DURATION_REPORTS_PRESERVED')
''',native=True,lock=True)
for pair in (1,2):
    m.remote("import sys\nsys.argv=['assess_native.py','--pair',"+repr(str(pair))+"]\n"+(m.HERE/'assess_native.py').read_text(),
        native=True,lock=True,timeout=180)
print('CORRECTED_ONLY_OFFLINE_METRICS_REMOTE_PILOT_GATE_REMAINS_AS_USED')
