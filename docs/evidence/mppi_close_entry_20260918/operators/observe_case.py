from pathlib import Path
import json,sys,os,signal,time
root=Path('/home/graneple/e2e_autonomous/mppi_close_adjust_20260918')
name=sys.argv[1]
assert name in {'lidar-v45-pc10-front-close-entry-a02','lidar-v45-pc10-front-close-entry-control-a01','lidar-v45-pc10-front-entry-normal-a01'}
run=Path('/home/graneple/git/autononous_ai/aichallenge-racingkart/output/scenario_tool')/name
report=dict(run=name)
if (run/'samples.jsonl').exists():
    rows=[]
    for line in (run/'samples.jsonl').read_text().splitlines():
        try:r=json.loads(line)
        except json.JSONDecodeError:continue
        if r.get('time',-1)>=0 and r.get('ego') and r.get('ego_gt',{}).get('source')=='gnss':rows.append(r)
    if rows:
        last=rows[-1];stamp=last['ego']['stamp']
        recent=[r['ego_gt']['progress_m'] for r in rows if r['ego']['stamp']>=stamp-15]
        report.update(sim_s=stamp-rows[0]['ego']['stamp'],start_m=rows[0]['ego_gt']['progress_m'],
                      progress_m=last['ego_gt']['progress_m'],speed_mps=last['ego']['speed_mps'],recent_range_m=max(recent)-min(recent))
        if '--stop-stalled' in sys.argv and report['sim_s']>=60 and report['progress_m']-report['start_m']<3.0:
            pid=json.loads((root/(name+'-pid.json')).read_text())['pid']
            cmd=Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')
            assert name.encode() in cmd and any(c.endswith(b'/collect_mppi_v45.py') for c in cmd)
            stop=root/(name+'-operator-stop.json')
            if not stop.exists():
                with stop.open('x') as f:json.dump(dict(reason='Less than 3 m progress after 60 simulation seconds of driving; bounded unsuccessful near-start trial',pid=pid,measurement=report,wall_time=time.time()),f,indent=2)
                os.kill(pid,signal.SIGINT)
                report['stop_sent']=True
if (run/'result.json').exists():
    r=json.loads((run/'result.json').read_text());report['verdict']=r.get('scenario_verdict');report['judgement']=r.get('judgement')
if (run/'d1-result-details.json').exists():report['penalties']=json.loads((run/'d1-result-details.json').read_text()).get('penalty_by_kind')
print(json.dumps(report))
