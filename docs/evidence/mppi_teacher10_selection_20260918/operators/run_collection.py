from pathlib import Path
import json,os,signal,subprocess,sys,time
root=Path('/home/graneple/e2e_autonomous/mppi_close_adjust_20260918')
name=sys.argv[1]
assert name in {'lidar-v45-pc10-front-collect10-'+s+'-a01' for s in ['a6','a8','an','b8']}
assert not subprocess.check_output(['docker','ps','-q'],text=True).strip()
command=['python3',str(root/'source/tools/collect_mppi_v45.py'),'--awsim-repo',
 '/home/graneple/git/autononous_ai/aichallenge-racingkart','--runtime',str(root/'runtime'),
 '--scenario',str(root/'scenarios'/(name+'.yaml')),'--run-id',name,'--speed-cap-kmh','10',
 '--wall-timeout-s','480','--run-budget-gib','0.75','--free-reserve-gib','2','--rviz','--execute']
started=time.time()
with (root/(name+'-launch.log')).open('x') as stream:
    proc=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
    (root/(name+'-pid.json')).write_text(json.dumps(dict(pid=proc.pid,command=command)))
    try:code=proc.wait(timeout=540)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid,signal.SIGINT)
        try:code=proc.wait(timeout=45)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid,signal.SIGTERM);code=proc.wait(timeout=20)
receipt=dict(command=command,exit_code=code,elapsed_wall_s=time.time()-started)
with (root/(name+'-launcher-result.json')).open('x') as f:json.dump(receipt,f,indent=2)
print(json.dumps(receipt),flush=True)
raise SystemExit(code)
