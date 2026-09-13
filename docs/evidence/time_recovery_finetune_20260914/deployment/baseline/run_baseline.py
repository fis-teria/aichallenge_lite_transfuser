from pathlib import Path
import subprocess,json,time
p=Path('/home/graneple/e2e_autonomous/time_recovery_model_baseline_20260914')
cmd=['timeout', '--signal=TERM', '--kill-after=10s', '710s', 'python3', '/home/graneple/e2e_autonomous/time_recovery_model_baseline_20260914/source_1777708/tools/run_time_path_awsim_trial.py', '--deployment', '/home/graneple/e2e_autonomous/time_recovery_model_baseline_20260914', '--run-id', 'codex-time-recovery-model-base01', '--display', ':1', '--config', 'configs/control/time_path_vehicle_model_5kmh_20260913.json']
t=time.monotonic()
with (p/"baseline_outer.log").open("x") as f:
 r=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL)
(p/"baseline_exit.json").write_text(json.dumps({"run_id":'codex-time-recovery-model-base01',"exit":r.returncode,"seconds":time.monotonic()-t,"command":cmd},indent=2))
