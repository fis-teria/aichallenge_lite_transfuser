import manage as m
m.remote(rf'''
from pathlib import Path
import hashlib,json
out=Path({m.OUT!r});base=out.with_name('time_recovery_sites_20260915')
plan=json.loads((out/'selected_site_plan.json').read_text())
candidates=json.loads((base/'candidate_site_audit.json').read_text())['candidates']
correct=next(s for s in candidates if s['start_s_m']==117.)
v=dict(plan_sha256=hashlib.sha256((out/'selected_site_plan.json').read_bytes()).hexdigest(),
 field='stop_site.map_xy_m',old_candidate_progress_m=116.,runtime_start_progress_m=117.,
 planned_map_annotation_xy_m=plan['stop_site']['map_xy_m'],correct_nominal_candidate_xy_m=correct['map_xy_m'],
 reason='Copied descriptive XY from previous 116m candidate; start_s_m and actual ROS markers already use 117m.',
 runtime_affected=False,recorded_marker_locations_affected=False,teacher_labels_affected=False)
with (out/'plan_map_annotation_erratum.json').open('x') as f:json.dump(v,f,indent=2)
print(json.dumps(v))
''',native=True,lock=True)
