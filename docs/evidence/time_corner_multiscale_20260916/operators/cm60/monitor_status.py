import argparse
import ops_corner as m
ap=argparse.ArgumentParser();ap.add_argument('--pair',type=int,required=True);a=ap.parse_args()
m.remote(m.PREFIX+'PAIR='+repr(a.pair)+'\n'+r'''
from pathlib import Path
import json
root=Path(ROOT);p=root/f'pair{PAIR:02}_resource_monitor.jsonl'
with p.open('rb') as f:
 f.seek(max(0,p.stat().st_size-12000));lines=f.read().splitlines()
row=json.loads(lines[-2] if len(lines)>1 else lines[-1]);freq=row['freq_khz']
print(json.dumps(dict(elapsed_s=row['elapsed_s'],available_mem_gib=row['memory_kb']['MemAvailable']/1024**2,swap_pages=row['swap_pages'],p_core_freq_mhz=[round(freq[str(i)]/1000) for i in (0,2,4,6,8,10)],package_c=row['temperature_millic'].get('x86_pkg_temp',0)/1000,throttle=row['package_throttle_count'],cpu_pressure=row['pressure']['cpu'],runs=row['runs'])))
for domain in (1,2):
 p=root/f'codex-time-recovery-corner60-p{PAIR:02}-d{domain}'/'control.jsonl'
 if not p.exists():continue
 with p.open('rb') as f:
  f.seek(max(0,p.stat().st_size-80000));lines=f.read().splitlines()
 record=json.loads(lines[-2] if len(lines)>1 else lines[-1])
 print(json.dumps(dict(domain=domain,**{k:record.get(k) for k in ('reason','processing_ms','decision_wall_ms','thread_cpu_ms','sim_ns')})))
''',timeout=25)
