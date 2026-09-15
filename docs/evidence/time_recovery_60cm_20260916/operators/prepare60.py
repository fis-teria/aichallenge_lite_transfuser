"""Generate initial 60 cm candidates under the native WSL worktree lock."""
import ops60 as m

m.remote('OUT='+repr(m.OUT)+'\nRAW='+repr(m.RAW)+'\n'+r'''
from pathlib import Path
import json,subprocess,sys,shutil
out=Path(OUT);out.mkdir(exist_ok=False);Path(RAW).mkdir(exist_ok=False)
old=out.parent/'time_recovery_40cm_20260915'
reports=[]
for side,previous in [('left','d40_g03_left_p243'),('right','d40_g03_right_safe_s4')]:
    plan=json.loads((old/(previous+'_plan.json')).read_text())
    for s in plan['candidates']:s['target_offset_m']=.6 if side=='left' else -.6
    tag='d60_g03_'+side+'_initial';p=out/(tag+'_plan.json')
    p.write_text(json.dumps(plan,indent=2)+'\n')
    cmd=[sys.executable,'tools/generate_time_large_recovery_reference.py',
      '--inputs','/home/thistle/e2e_autonomous/runs/time_recovery_collection_20260913/inputs',
      '--normal-run','/home/thistle/e2e_autonomous/raw/time_recovery_sites_20260915/codex-time-recovery-sites-normal-n03',
      '--normal-proof','/home/thistle/e2e_autonomous/runs/time_recovery_separated_20260915/selected_site_plan.json',
      '--plan',str(p),'--side',side,'--output',str(out/tag)]
    r=subprocess.run(cmd,capture_output=True,text=True,timeout=180)
    (out/(tag+'_generate.log')).write_text(r.stdout+r.stderr)
    report=dict(tag=tag,exit=r.returncode,command=cmd)
    if r.returncode==0:
        ref=json.loads((out/tag/(side+'.json')).read_text())
        report['map_screen']=ref['large_recovery']['map_screen']
    else:report['error']=r.stderr[-2000:]
    reports.append(report)
(out/'initial_map_screen.json').write_text(json.dumps(reports,indent=2))
print(json.dumps(dict(reports=reports,wsl_free_gib=shutil.disk_usage(out).free/2**30)))
''',native=True,lock=True,timeout=420)
