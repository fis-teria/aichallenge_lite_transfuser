"""Locked native validation for the new ROS package / make entrypoint."""
from pathlib import Path
import hashlib,json,subprocess,sys
from aic_transfuser_lite.control.time_trial_v1 import validate_trial_config
repo=Path('/home/thistle/e2e_autonomous/e2e_lite_transfuser')
out=repo.parent/'runs/time_path_dev_20260917';out.mkdir(exist_ok=True)
head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()
assert not subprocess.check_output(['git','status','--porcelain']).strip()
config=repo/'configs/control/time_path_dev.json'
cfg=json.loads(config.read_bytes());validate_trial_config(cfg)
checkpoint=repo.parent/'runs/time_launch_protection_v2_20260916/launch_balanced/epoch_03.pt'
assert hashlib.sha256(checkpoint.read_bytes()).hexdigest()==cfg['checkpoint_sha256']
log_path=out/('pytest_'+head[:7]+'.log')
with log_path.open('x') as log:
 p=subprocess.run([sys.executable,'-m','pytest','-q'],stdout=log,stderr=subprocess.STDOUT,timeout=450)
print(log_path.read_text()[-6500:],flush=True)
assert p.returncode==0
baseline=repo.parent/'runs/time_preview20_20260917/raw/codex-time-timepreview20-lap01'
with (out/'baseline_replay.log').open('x') as log:
 p=subprocess.run([sys.executable,'tools/evaluate_time_awsim_trial.py','--run',str(baseline),
  '--output',str(out/'baseline_replay')],stdout=log,stderr=subprocess.STDOUT,timeout=180)
assert p.returncode==0,(out/'baseline_replay.log').read_text()
result=json.loads((out/'baseline_replay/summary.json').read_bytes())
assert result['control_replay']['status']=='PASS' and result['judge_lap_confirmed']
gate=dict(status='PASS',exit=0,source_commit=head,tested_source_commit=head,checkpoint_sha256=cfg['checkpoint_sha256'],
 full_pytest_log=log_path.name,full_pytest_log_sha256=hashlib.sha256(log_path.read_bytes()).hexdigest(),
 baseline_replay=result['control_replay'],config_sha256=hashlib.sha256(config.read_bytes()).hexdigest())
(out/'deployment_gate.json').write_text(json.dumps(gate,indent=2)+'\n')
print(json.dumps(gate),flush=True)
