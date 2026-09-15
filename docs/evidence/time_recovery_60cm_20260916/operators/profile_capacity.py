"""Read-only resource sample during the current single-car AWSIM lap."""
import json
import ops60 as m
text=m.remote('ROOT='+repr(m.ROOT)+'\n'+r'''
from pathlib import Path
import json,subprocess,time,os
root=Path(ROOT);run=root/'codex-time-recovery-60cm-d60-g03-right-r03'
samples=[]
def cpu():
    a=list(map(int,Path('/proc/stat').read_text().splitlines()[0].split()[1:9]));return sum(a),a[3]+a[4]
last=cpu()
for i in range(12):
    gpu=subprocess.check_output(['nvidia-smi','--query-gpu=utilization.gpu,utilization.memory,memory.used,memory.total,power.draw','--format=csv,noheader,nounits'],text=True).strip()
    now=cpu();busy=None if i==0 else 100*(1-(now[1]-last[1])/(now[0]-last[0]));last=now
    mem={p[0].rstrip(':'):int(p[1]) for line in Path('/proc/meminfo').read_text().splitlines() if len(p:=line.split())>=2 and p[1].isdigit()}
    h=json.loads((run/'control_heartbeat.json').read_text())
    samples.append(dict(wall_monotonic_s=time.monotonic(),gpu=list(map(float,gpu.split(','))),cpu_busy_percent=busy,
      available_ram_gib=mem['MemAvailable']/2**20,sim_ns=h.get('sim_ns'),phase=h.get('phase'),fault=h.get('fault')))
    if i<11:time.sleep(2)
print(json.dumps(dict(host='graneple@192.168.3.10',run_id=run.name,logical_cpus=os.cpu_count(),
 gpu_columns=['gpu_util_percent','memory_util_percent','memory_used_mib','memory_total_mib','power_w'],samples=samples,
 scope='One car, one AWSIM instance; measurement does not establish multi-instance throughput')))
''',timeout=45)
report=json.loads(text)
with (m.UNC/'parallel_capacity_profile.json').open('x') as f:json.dump(report,f,indent=2)
samples=report['samples'];gpu=[r['gpu'][0] for r in samples];ram=[r['available_ram_gib'] for r in samples];cpu=[r['cpu_busy_percent'] for r in samples if r['cpu_busy_percent'] is not None]
rtf=(samples[-1]['sim_ns']-samples[0]['sim_ns'])/1e9/(samples[-1]['wall_monotonic_s']-samples[0]['wall_monotonic_s'])
print(json.dumps(dict(gpu_percent_min=min(gpu),gpu_percent_max=max(gpu),gpu_percent_mean=sum(gpu)/len(gpu),
 gpu_memory_mib_max=max(r['gpu'][2] for r in samples),cpu_percent_mean=sum(cpu)/len(cpu),cpu_percent_max=max(cpu),
 available_ram_gib_min=min(ram),observed_sim_time_per_wall_time=rtf)))
