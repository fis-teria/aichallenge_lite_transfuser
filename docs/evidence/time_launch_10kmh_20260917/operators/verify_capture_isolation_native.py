"""Recheck recorder-only fix; physics and model inputs remain at the tested source."""
from pathlib import Path
import hashlib
import json
import subprocess
import sys

root = Path('/home/thistle/e2e_autonomous/runs/time_launch_10kmh_20260917')
path = root/'deployment_gate.json'
before = path.read_bytes(); gate = json.loads(before)
assert gate['source_commit'] == '7bb70809fec844e33bbcc298d92ecba4d374f4e5'
assert not subprocess.check_output(['git', 'status', '--porcelain']).strip()
head = subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip()
changed = subprocess.check_output(['git', 'diff', '--name-only', gate['source_commit'], head], text=True).splitlines()
assert set(changed) == {'tools/time_trial_video.py', 'tools/run_time_path_awsim_trial.py', 'tests/test_time_trial_video.py'}, changed
assert not subprocess.check_output(['git','diff',gate['full_pytest_source_commit'],head,'--','src','ros2_ws']).strip()
command = [sys.executable, '-m', 'pytest', '-q', 'tests/test_time_trial_10kmh.py', 'tests/test_time_trial_video.py']
with (root/'capture_isolation_pytest.log').open('x') as log:
    tested = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, timeout=90)
assert tested.returncode == 0, (root/'capture_isolation_pytest.log').read_text()
(root/'deployment_gate_7bb7080.json').write_bytes(before)
gate.update(source_commit=head, tested_source_commit=head,
    recorder_fix_source_change_files=changed,control_and_model_source_unchanged_since_full_pytest=True,
    capture_isolation_pytest=dict(exit=0,command=command,log_sha256=hashlib.sha256((root/'capture_isolation_pytest.log').read_bytes()).hexdigest()))
path.write_text(json.dumps(gate, indent=2)+'\n')
print(json.dumps(dict(status='PASS',source_commit=head,changed=changed)))
