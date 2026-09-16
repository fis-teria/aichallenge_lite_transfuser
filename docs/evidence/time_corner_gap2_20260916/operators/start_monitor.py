import argparse
import ops_corner as m
ap=argparse.ArgumentParser();ap.add_argument('--pair',type=int,required=True);a=ap.parse_args()
m.transport.copy_to_host([m.HERE/'resource_monitor.py'],destination=m.ROOT)
m.remote(m.PREFIX+'PAIR='+repr(a.pair)+'\n'+r'''
from pathlib import Path
import subprocess,json
root=Path(ROOT)
with (root/f'pair{PAIR:02}_monitor_supervisor.log').open('x') as f:
 p=subprocess.Popen(['python3',str(root/'resource_monitor.py'),'--root',str(root),'--pair',str(PAIR)],stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
print(json.dumps(dict(monitor_pid=p.pid,pair=PAIR)))
''')
