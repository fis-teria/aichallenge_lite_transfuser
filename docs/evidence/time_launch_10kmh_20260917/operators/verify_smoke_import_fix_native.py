"""Verify the isolated entrypoint import fix after the complete pytest pass."""
from pathlib import Path
import hashlib
import json
import subprocess
import sys

root = Path('/home/thistle/e2e_autonomous/runs/time_launch_10kmh_20260917')
path = root/'deployment_gate.json'
before = path.read_bytes()
gate = json.loads(before)
assert gate['source_commit'] == '03d4bccd2bbd8508995362eee15970977679a1e4'
assert not subprocess.check_output(['git', 'status', '--porcelain']).strip()
head = subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip()
changed = subprocess.check_output(['git', 'diff', '--name-only', gate['source_commit'], head], text=True).splitlines()
assert set(changed) == {'tools/check_time_ros_connection.py', 'tests/test_time_trial_10kmh.py'}, changed
assert hashlib.sha256((root/'pytest.log').read_bytes()).hexdigest() == gate['full_pytest_log_sha256']
command = [sys.executable, '-m', 'pytest', '-q', 'tests/test_time_trial_10kmh.py', 'tests/test_time_trial_video.py']
with (root/'entrypoint_fix_pytest.log').open('x') as log:
    tested = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, timeout=90)
assert tested.returncode == 0, (root/'entrypoint_fix_pytest.log').read_text()
(root/'deployment_gate_03d4bcc.json').write_bytes(before)
gate.update(source_commit=head, tested_source_commit=head, full_pytest_source_commit=gate['source_commit'],
    existing_full_pytest_reused=True, followup_source_change_files=changed,
    entrypoint_fix_pytest=dict(exit=0,command=command,log_sha256=hashlib.sha256((root/'entrypoint_fix_pytest.log').read_bytes()).hexdigest()))
path.write_text(json.dumps(gate, indent=2)+'\n')
print(json.dumps(dict(status='PASS',source_commit=head,changed=changed)))
