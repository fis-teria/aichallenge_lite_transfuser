"""File transport only: system Python receiver never accesses the training worktree."""
import subprocess,json,sys
from pathlib import Path
run=sys.argv[1]
assert run.startswith(('lidar-v45-pc10-front-','lidar-v45-pc10-counterfactual-')) and all(c.isalnum() or c=='-' for c in run)
remote='/home/graneple/e2e_autonomous/mppi_close_adjust_20260918'
root='/home/thistle/e2e_autonomous/runs/mppi_v45_pc10_20260918'
result=subprocess.run(['ssh','graneple@192.168.3.10',f'python3 {remote}/seal_run.py {run}'],capture_output=True,text=True,check=True)
print(result.stdout,flush=True)
with (Path(__file__).parent/(run+'-transport.stderr')).open('wb') as err:
    source=subprocess.Popen(['ssh','graneple@192.168.3.10',f'python3 {remote}/seal_run.py {run} --export'],stdout=subprocess.PIPE,stderr=err)
    target=subprocess.Popen(['ssh','codex-wsl',f'python3 {root}/receive_run.py {run}'],stdin=source.stdout,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    source.stdout.close();output,errors=target.communicate(timeout=300);code=source.wait(timeout=30)
assert code==target.returncode==0,(code,target.returncode,errors.decode())
report=json.loads(output);assert report['all_sha256_match']
(Path(__file__).parent/(run+'-transfer.json')).write_text(json.dumps(report,indent=2))
print(json.dumps(report),flush=True)
