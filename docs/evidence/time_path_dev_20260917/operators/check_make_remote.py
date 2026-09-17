from pathlib import Path
import json,subprocess
root=Path('/home/graneple/e2e_autonomous/time_path_dev_20260917')
assert not subprocess.check_output(['docker','ps','-q']).strip()
command=['make','--no-print-directory','-C',str(root),'-n','dev',
         'MAX_SPEED_KMH=12','CORNER_MAX_SPEED_KMH=8','TIME_RECORD_VIDEO=0']
p=subprocess.run(command,check=True,capture_output=True,text=True,timeout=10)
assert '--max-speed-kmh "12"' in p.stdout and '--corner-max-speed-kmh "8"' in p.stdout,p.stdout
assert '--record-video' not in p.stdout
with (root/'make_parameter_smoke.json').open('x') as stream:
 json.dump(dict(status='PASS',command=command,stdout=p.stdout),stream,indent=2)
print(p.stdout)
