from pathlib import Path
import os,signal,json,time
pid=453257;run='lidar-v45-pc10-front-cone-close6-a02'
cmd=Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')
assert run.encode() in cmd and any(c.endswith(b'/collect_mppi_v45.py') for c in cmd)
root=Path('/home/graneple/e2e_autonomous/mppi_v45_collection_fix_20260918')
(root/(run+'-operator-stop.json')).write_text(json.dumps(dict(reason='Teacher makes no passing progress: raw execution-sweep candidates reject collision, selected retime commands zero speed; preserve as unsuccessful demonstration',signal='SIGINT',pid=pid,wall_time=time.time()),indent=2))
os.kill(pid,signal.SIGINT)
