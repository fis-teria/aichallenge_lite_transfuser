from pathlib import Path
import subprocess,json,time
root=Path('/home/graneple/e2e_autonomous/time_segment_following_20260914')
m=json.loads((root/'source_manifest.json').read_text())
assert json.loads((root/'smoke/summary.json').read_text())['status']=='PASS'
cmd=['timeout','--signal=TERM','--kill-after=10s','710s','python3',str(root/m['source_dir']/'tools/run_time_path_awsim_trial.py'),'--deployment',str(root),'--run-id','codex-time-segment01','--display',':1','--config',m['config']]
began=time.monotonic()
with (root/'trial_outer.log').open('x') as f:
 p=subprocess.run(cmd,stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT)
(root/'trial_exit.json').write_text(json.dumps({'run_id':'codex-time-segment01','exit':p.returncode,'seconds':time.monotonic()-began,'command':cmd},indent=2))
