import argparse,subprocess,sys
import ops_corner as m
ap=argparse.ArgumentParser();ap.add_argument('--pair',type=int,required=True);a=ap.parse_args()
subprocess.run([sys.executable,'-u',str(m.HERE/'audit_pair.py'),'--pair',str(a.pair)],check=True)
m.remote('OUT='+repr(m.OUT)+'\n'+r"""
from pathlib import Path
import subprocess,sys
out=Path(OUT)
for name,extra in [('finalize_native.py',[]),('compare_critical_state.py',['--suffix','final'])]:
 log=out/(name.replace('.py','')+'_execution.log')
 with log.open('x') as f:p=subprocess.run([sys.executable,str(out/name),*extra],stdout=f,stderr=subprocess.STDOUT,timeout=900)
 if p.returncode:raise RuntimeError(log.read_text()[-4000:])
 print(log.read_text()[-2500:])
""",native=True,lock=True,timeout=2000)
