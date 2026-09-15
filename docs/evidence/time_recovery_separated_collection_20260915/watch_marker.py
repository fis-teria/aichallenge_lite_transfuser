from manage import remote, ROOT
import sys

name=sys.argv[1]
assert name.startswith('codex-time-recovery-separated-')
remote('RUN_ID='+repr(name)+'\n'+r'''
from pathlib import Path
import hashlib,json,os,subprocess,time,shutil
root=Path('/home/graneple/e2e_autonomous/time_recovery_separated_20260915');run=root/RUN_ID
proofs=root/'visual_proofs';proofs.mkdir(exist_ok=True)
env=dict(os.environ,**json.loads((root/'display.json').read_text()))
seen=set();deadline=time.monotonic()+480
while time.monotonic()<deadline:
 if (run/'result.json').exists():break
 p=run/'path_heartbeat.json'
 if p.exists():
  h=json.loads(p.read_text());sites=h.get('disturbance_sites',[])
  if sites and len(sites) not in seen:
   marker=json.loads((run/'disturbance_markers.json').read_text())
   assert 'rviz2' in h['marker_subscribers']
   windows=[s for s in subprocess.check_output(['xwininfo','-root','-tree'],env=env,text=True).splitlines() if 'autoware.rviz' in s]
   assert len(windows)==1;window=windows[0].strip().split()[0]
   prefix=RUN_ID+'_event'+str(len(sites));xwd=proofs/(prefix+'.xwd')
   assert not xwd.exists()
   subprocess.run(['xwd','-silent','-id',window,'-out',str(xwd)],env=env,check=True,timeout=5)
   png=None
   if shutil.which('convert'):
    png=proofs/(prefix+'.png');subprocess.run(['convert',str(xwd),str(png)],check=True,timeout=10)
   v=dict(run_id=RUN_ID,marker_locations=marker,heartbeat=h,window=windows[0].strip(),
     captured_unix_s=time.time(),xwd_sha256=hashlib.sha256(xwd.read_bytes()).hexdigest(),png=str(png) if png else None)
   (proofs/(prefix+'.json')).write_text(json.dumps(v,indent=2))
   print(json.dumps(v),flush=True);seen.add(len(sites))
 time.sleep(.5)
print(json.dumps(dict(run_id=RUN_ID,captured_marker_counts=sorted(seen))),flush=True)
''',timeout=500)
