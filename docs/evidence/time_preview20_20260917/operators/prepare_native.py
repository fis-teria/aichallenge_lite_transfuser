"""Reuse unchanged-source full tests after the 15 km/h trial completes."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

from aic_transfuser_lite.control.time_trial_v1 import validate_trial_config

repo=Path('/home/thistle/e2e_autonomous/e2e_lite_transfuser')
head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()
assert not subprocess.check_output(['git','status','--porcelain']).strip()
previous=repo.parent/'runs/time_preview15_20260917'
gate=json.loads((previous/'deployment_gate.json').read_bytes())
assert gate['exit']==0 and gate['source_commit']==gate['tested_source_commit']==head
result=json.loads((previous/'evaluation/summary.json').read_bytes())
assert result['judge_lap_confirmed'] and result['stop_confirmed_before_cleanup']
assert result['control_replay']['status']=='PASS'
assert json.loads((previous/'transfer_verification.json').read_bytes())['status']=='PASS'
config_path=repo/'configs/control/time_path_timepreview_20kmh_20260917.json'
config=json.loads(config_path.read_bytes())
old=json.loads((repo/'configs/control/time_path_timepreview_15kmh_20260917.json').read_bytes())
assert {k for k in config if config[k]!=old[k]}=={'speed_policy','vehicle_model_policy','speed_cap_mps','overspeed_limit_mps'}
assert validate_trial_config(config)=='curvature_time_preview_20kmh_v1'
assert config['checkpoint_sha256']==gate['checkpoint_sha256']
checkpoint=repo.parent/'runs/time_launch_protection_v2_20260916/launch_balanced/epoch_03.pt'
assert hashlib.sha256(checkpoint.read_bytes()).hexdigest()==gate['checkpoint_sha256']
assert hashlib.sha256((previous/'pytest.log').read_bytes()).hexdigest()==gate['full_pytest_log_sha256']
out=repo.parent/'runs/time_preview20_20260917';out.mkdir(exist_ok=False)
shutil.copy2(previous/'pytest.log',out/'pytest.log')
gate.update(existing_full_pytest_reused=True,reused_from=str(previous),
    prerequisite_15kmh_lap_seconds=result['judge_laps'][0]['lap_seconds'],
    prerequisite_15kmh_evaluation_sha256=hashlib.sha256((previous/'evaluation/summary.json').read_bytes()).hexdigest(),
    config_sha256=hashlib.sha256(config_path.read_bytes()).hexdigest(),
    user_authorization='User requested time-preview alignment following the 20 km/h ceiling discussion; bounded 20 km/h trial after the same-source 15 km/h lap, retaining log-only proximity')
(out/'deployment_gate.json').write_text(json.dumps(gate,indent=2)+'\n')
print(json.dumps(dict(status='NATIVE_RUNTIME_READY',source_commit=head,full_tests_reused=True,prerequisite_15kmh_lap=True)))
