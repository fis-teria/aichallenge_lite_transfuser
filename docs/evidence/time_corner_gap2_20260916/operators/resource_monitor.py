"""Read-only, low-priority host sampling for one bounded collection pair."""
from pathlib import Path
import argparse,json,os,time
ap=argparse.ArgumentParser();ap.add_argument('--root',type=Path,required=True);ap.add_argument('--pair',type=int,required=True);a=ap.parse_args()
root=a.root.resolve();assert root.parent==Path('/home/graneple/e2e_autonomous')
os.nice(10);os.sched_setaffinity(0,set(range(12,20)))
names=[f'codex-time-recovery-cornergap2-p{a.pair:02}-d{d}' for d in (1,2)]
output=root/f'pair{a.pair:02}_resource_monitor.jsonl';started=time.monotonic()
def read(p):return Path(p).read_text().strip()
with output.open('x') as stream:
 while time.monotonic()-started<1100:
  memory={s.split(':')[0]:int(s.split()[1]) for s in read('/proc/meminfo').splitlines() if s.split(':')[0] in ('MemAvailable','SwapFree','SwapTotal')}
  vm={p[0]:int(p[1]) for s in read('/proc/vmstat').splitlines() if (p:=s.split())[0] in ('pswpin','pswpout')}
  freq={str(i):int(read(f'/sys/devices/system/cpu/cpu{i}/cpufreq/scaling_cur_freq')) for i in range(20)}
  thermal={read(p/'type'):int(read(p/'temp')) for p in Path('/sys/class/thermal').glob('thermal_zone*')}
  throttle={str(i):int(read(f'/sys/devices/system/cpu/cpu{i}/thermal_throttle/package_throttle_count')) for i in (0,4,8,12)}
  runs=[]
  for name in names:
   p=root/name;heartbeat=p/'control_heartbeat.json';result=p/'result.json'
   if result.exists():
    r=json.loads(result.read_text());c=r['last_control'];status=r['status']
   elif heartbeat.exists():c=json.loads(heartbeat.read_text());status=None
   else:c={};status=None
   runs.append(dict(name=name,status=status,fault=c.get('fault'),sim_ns=c.get('sim_ns'),s_m=(c.get('projection') or {}).get('s_m'),phase=c.get('phase'),max_processing_ms=c.get('max_processing_ms')))
  row=dict(monotonic_ns=time.monotonic_ns(),unix_s=time.time(),elapsed_s=time.monotonic()-started,memory_kb=memory,swap_pages=vm,freq_khz=freq,temperature_millic=thermal,package_throttle_count=throttle,loadavg=read('/proc/loadavg'),pressure={k:read('/proc/pressure/'+k) for k in ('cpu','memory','io')},runs=runs)
  stream.write(json.dumps(row)+'\n');stream.flush()
  if all(r['status'] is not None for r in runs):break
  time.sleep(1)
print(json.dumps(dict(path=str(output),samples_elapsed_s=time.monotonic()-started,results=runs)),flush=True)
