"""Verify the report-only acceptance adjustment; driving source stays identical."""
from pathlib import Path
import hashlib
import json
import subprocess
import sys

root=Path('/home/thistle/e2e_autonomous/runs/time_launch_15kmh_stop1m_20260917')
gate=root/'deployment_gate.json';data=gate.read_bytes();old=json.loads(data)
head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()
changed=subprocess.check_output(['git','diff','--name-only',old['source_commit'],head],text=True).splitlines()
assert set(changed)=={'src/aic_transfuser_lite/evaluation/time_speed_ladder_v1.py','tests/test_time_trial_speed_ladder.py','docs/time_speed_ladder_20260917.md'}
assert not subprocess.check_output(['git','diff','--name-only',old['source_commit'],head,'--','src/aic_transfuser_lite/control','src/aic_transfuser_lite/runtime','src/aic_transfuser_lite/models','ros2_ws','configs','tools'])
with (root/'acceptance_pytest.log').open('x') as f:
    run=subprocess.run([sys.executable,'-m','pytest','-q','tests/test_time_trial_speed_ladder.py'],stdout=f,stderr=subprocess.STDOUT,timeout=60)
assert run.returncode==0,(root/'acceptance_pytest.log').read_text()
with (root/('deployment_gate_'+old['source_commit'][:7]+'.json')).open('xb') as f:f.write(data)
new={**old,'source_commit':head,'full_pytest_source_commit':old['source_commit'],
     'existing_full_pytest_reused':True,'followup_source_commit':head,'followup_changed_files':changed,
     'followup_pytest_exit':0,'followup_pytest_sha256':hashlib.sha256((root/'acceptance_pytest.log').read_bytes()).hexdigest(),
     'control_model_ros_runtime_unchanged':True}
gate.write_text(json.dumps(new,indent=2))
print((root/'pytest.log').read_text().splitlines()[-1])
print((root/'acceptance_pytest.log').read_text().splitlines()[-1])
print(json.dumps(dict(status='PASS',source_commit=head,control_unchanged=True)))
