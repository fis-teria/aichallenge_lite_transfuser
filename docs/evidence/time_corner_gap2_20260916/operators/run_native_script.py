"""Run this campaign's bounded analysis operators under the native worktree lock."""
from pathlib import Path
import hashlib
import shutil
import sys
import ops_corner as m

source = m.HERE / sys.argv[1]
assert source.name in ('adaptive_plan.py', 'adaptive_plan_v2.py', 'adaptive_plan_v3.py', 'finalize_native.py', 'inspect_pair.py', 'entry_calibration.py', 'diagnose_failures.py', 'calibrate_remaining.py', 'screen_blended.py', 'screen_c03_blended.py', 'screen_delayed.py', 'adaptive_plan_v4.py', 'screen_blended_entry.py', 'screen_c03_heading.py', 'screen_earlier_goals.py', 'coverage_diagnosis.py')
destination = m.UNC / source.name
if destination.exists():
    assert hashlib.sha256(destination.read_bytes()).digest() == hashlib.sha256(source.read_bytes()).digest()
else:
    shutil.copy2(source, destination)
command = [m.OUT + '/' + source.name, *sys.argv[2:]]
m.remote('import runpy,sys\nsys.argv=' + repr(command) + '\nrunpy.run_path(sys.argv[0],run_name="__main__")',
         native=True, lock=True, timeout=1200)
