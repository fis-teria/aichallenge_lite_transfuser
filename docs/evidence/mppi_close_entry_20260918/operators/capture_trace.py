from pathlib import Path
import json,re,subprocess,sys,time,collections
root=Path('/home/graneple/e2e_autonomous/mppi_close_adjust_20260918')
name=sys.argv[1]
assert re.fullmatch(r'lidar-v45-pc10-front-[a-z0-9-]+',name)
container='codex-'+name+'-scn-car1-1'
p=subprocess.run(['docker','logs','--tail','2000',container],capture_output=True,text=True,check=True)
text=p.stdout+p.stderr
summary={}
for tag in ('MPPI_EXECUTION','MPPI_SELECTION_CANDIDATES','MPPI_RETIME','MPPI_EXECUTION_INPUT'):
    rows=[s.split('['+tag+']',1)[1] for s in text.splitlines() if '['+tag+']' in s]
    if rows:
        s=re.sub(r'\x1b\[[0-9;]*m','',rows[-1])
        if tag=='MPPI_EXECUTION_INPUT':
            summary[tag]={k:re.search(r' '+k+r'=([^ ]+)',s).group(1) for k in ['stamp_sec','ego','nominal','minimum','maximum','dynamic']}
        else:summary[tag]=s
out=root/(name+'-trace-'+str(time.time_ns())+'.json')
out.write_text(json.dumps(summary,indent=2))
print(json.dumps(dict(file=str(out),**{k:v for k,v in summary.items() if k!='MPPI_SELECTION_CANDIDATES'})))
