"""Record capacity throughout a bounded overlap of the two collection laps."""
import json
import ops as m

reply=m.remote(m.PREFIX+r'''
from pathlib import Path
import json,subprocess,time,os
root=Path(ROOT);plan=json.loads((root/'parallel_plan.json').read_text());samples=[]
def cpu():
 a=list(map(int,Path('/proc/stat').read_text().splitlines()[0].split()[1:9]));return sum(a),a[3]+a[4]
last=cpu()
for i in range(110):
 gpu=subprocess.check_output(['nvidia-smi','--query-gpu=utilization.gpu,utilization.memory,memory.used,memory.total,power.draw','--format=csv,noheader,nounits'],text=True).strip()
 now=cpu();busy=None if i==0 else 100*(1-(now[1]-last[1])/(now[0]-last[0]));last=now
 mem={p[0].rstrip(':'):int(p[1]) for line in Path('/proc/meminfo').read_text().splitlines() if len(p:=line.split())>=2 and p[1].isdigit()}
 states={}
 for row in plan['instances']:
  p=root/row['run_id']/'control_heartbeat.json';h=json.loads(p.read_text()) if p.exists() else {}
  states[row['side']]={k:h.get(k) for k in ['sim_ns','monotonic_ns','phase','fault','speed_mps','reason']}
  states[row['side']]['progress_m']=(h.get('projection') or {}).get('s_m')
 samples.append(dict(wall_monotonic_s=time.monotonic(),gpu=list(map(float,gpu.split(','))),cpu_busy_percent=busy,available_ram_gib=mem['MemAvailable']/2**20,states=states))
 if i<109:time.sleep(2)
report=dict(host='graneple@192.168.3.10',logical_cpus=os.cpu_count(),samples=samples,scope='Two unchanged AWSIM instances; profile includes startup and running periods; use running overlap for throughput')
(root/'parallel_capacity_profile.json').write_text(json.dumps(report,indent=2))
moving=[s for s in samples if all(v['phase'] not in (None,'invalid','braking') and v['fault'] is None for v in s['states'].values())]
summary=dict(samples=len(samples),moving_overlap_samples=len(moving),gpu_max=max(s['gpu'][0] for s in samples),vram_max_mib=max(s['gpu'][2] for s in samples),available_ram_min_gib=min(s['available_ram_gib'] for s in samples))
if len(moving)>1:
 summary['moving_gpu_mean']=sum(s['gpu'][0] for s in moving)/len(moving)
 summary['moving_rtf']={side:(moving[-1]['states'][side]['sim_ns']-moving[0]['states'][side]['sim_ns'])/1e9/(moving[-1]['wall_monotonic_s']-moving[0]['wall_monotonic_s']) for side in ('left','right')}
(root/'parallel_capacity_summary.json').write_text(json.dumps(summary,indent=2))
print(json.dumps(summary))
''',timeout=260)
(m.HERE/'parallel_capacity_summary.json').write_text(reply)
