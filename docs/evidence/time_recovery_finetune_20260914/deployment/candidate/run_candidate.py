from pathlib import Path
import subprocess,json,time
p=Path('/home/graneple/e2e_autonomous/time_recovery_model_candidate_20260914')
cmd=['timeout', '--signal=TERM', '--kill-after=10s', '710s', 'python3', '/home/graneple/e2e_autonomous/time_recovery_model_candidate_20260914/source_5451333/tools/run_time_path_awsim_trial.py', '--deployment', '/home/graneple/e2e_autonomous/time_recovery_model_candidate_20260914', '--run-id', 'codex-time-recovery-model-candidate01', '--display', ':1', '--config', 'configs/control/time_path_recovery_5kmh_20260914.json']
t=time.monotonic()
with (p/"candidate_outer.log").open("x") as f:
 r=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL)
(p/"candidate_exit.json").write_text(json.dumps({"run_id":'codex-time-recovery-model-candidate01',"exit":r.returncode,"seconds":time.monotonic()-t,"command":cmd},indent=2))
