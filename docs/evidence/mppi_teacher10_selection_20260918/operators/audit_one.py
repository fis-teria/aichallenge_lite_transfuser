from pathlib import Path
import sys,shutil,json,subprocess
repo=Path.cwd();sys.path[:0]=[str(repo),str(repo/'tools')]
from tools.audit_native_corner_collection import audit
root=Path('/home/thistle/e2e_autonomous/runs/mppi_v45_pc10_20260918')
name=sys.argv[1]
allowed=['lidar-v45-pc10-front-'+s for s in ['close10-control-a01','normal10-control-a01','collect10-a6-a01','collect10-a8-a01','collect10-an-a01','collect10-b8-a01']]
assert name in allowed
identity=json.loads((root/'collected'/name/'raw'/name/'d1/teacher-runtime-identity.json').read_text())
assert not identity['early_entry_search'] and abs(identity['speed_cap_mps']-10/3.6)<1e-12
result=audit(root/'collected'/name,root/'collect10_audit_v1'/name)
shutil.copytree(root/'collect10_audit_v1'/name/'pose_prefix',root/'collect10_pose_prefix_v1'/name)
subprocess.run([sys.executable,str(Path(__file__).with_name('check_clearance.py')),name],check=True)
print('AUDIT_DONE',name,json.dumps(result['teacher_pose_prefix']),flush=True)
