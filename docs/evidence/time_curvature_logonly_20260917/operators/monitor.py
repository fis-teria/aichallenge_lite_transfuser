"""Read-only progress snapshot of this owned trial."""
from manage import remote, ROOT, RUN_ID

remote('''
from pathlib import Path
import json,math,subprocess,time
root=Path('''+repr(ROOT)+'''); run=root/'''+repr(RUN_ID)+'''
def read(name,base=run):
    path=base/name
    return json.loads(path.read_text()) if path.exists() else {}
control=read('control_heartbeat.json'); inference=read('inference_heartbeat.json')
lap=read('lap_progress.json'); host=read('host_result.json')
poses={}; reasons={}; last_preview={}
log=run/'control.jsonl'
if log.exists():
    lines=log.read_bytes().split(b'\n')[:-1]
    rows=[json.loads(line) for line in lines if line]
    armed=control.get('armed_ns')
    for row in rows:
        if armed is None or row.get('event')!='COMMAND_SENT' or row['sim_ns']<armed: continue
        reason=row['reason']; reasons[reason]=reasons.get(reason,0)+1
        if reason=='TIME_PATH_TRACKING':last_preview=row.get('details',{}).get('longitudinal_preview',{})
        pose=row.get('details',{}).get('current_pose')
        if pose:poses[pose['stamp_ns']]=(pose['x_m'],pose['y_m'])
points=[p for _,p in sorted(poses.items())]
travel=sum(math.dist(a,b) for a,b in zip(points,points[1:]))
print(json.dumps(dict(wall_elapsed_s=round(time.time()-read('runner_start.json',root)['started_unix'],1),
    active_sim_s=(control['sim_ns']-control['armed_ns'])/1e9 if control.get('armed_ns') else None,
    recorded_travel_m=round(travel,3),speed_kmh=control.get('speed_mps',0)*3.6 if control.get('speed_mps') is not None else None,
    requested_speed_kmh=last_preview.get('target_speed_mps',0)*3.6, speed_limit_source=last_preview.get('limiting_reason'),
    scan_would_stop_commands=control.get('scan_would_stop_commands'),
    judge_sections=[s['next'] for s in lap.get('sections',[])],lap_confirmed=lap.get('lap_confirmed'),
    control_reason=control.get('reason'),command_reasons=reasons,fault=control.get('fault'),
    requested_stop=control.get('requested_stop_reason'),stop_confirmed=control.get('stop_confirmed'),
    raw_path_subscribers=inference.get('path_subscribers'),host_status=host.get('status'),host_error=host.get('error'),
    runner_exit=read('runner_exit.json',root),containers=subprocess.check_output(['docker','ps','--format','{{.Names}}'],text=True).splitlines())))
'''.replace("split(b'\n')", "split(b'\\n')"))
