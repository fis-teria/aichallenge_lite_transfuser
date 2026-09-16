"""Verified recorded speed attainment, plus reasons for any control ceiling."""
from collections import Counter
from pathlib import Path
import hashlib
import json
import numpy as np
from aic_transfuser_lite.evaluation.time_speed_ladder_v1 import assess_speed_step

root=Path('/home/thistle/e2e_autonomous/runs/time_launch_15kmh_stop1m_20260917')
raw=root/'raw/codex-time-launch15-stop1m-lap01'
manifest={r['path']:r for r in json.loads((raw/'transfer_manifest.json').read_bytes())}
data=(raw/'control.jsonl').read_bytes()
assert hashlib.sha256(data).hexdigest()==manifest['control.jsonl']['sha256']
summary=json.loads((root/'evaluation/summary.json').read_bytes())
rows=[json.loads(line) for line in data.splitlines()]
start=summary['armed_sim_ns'];end=start+round(summary['active_duration_sim_s']*1e9)
active=[r for r in rows if r['event']=='COMMAND_SENT' and start<=r['sim_ns']<end]
speeds=np.array([r['speed_mps'] for r in active if r.get('speed_mps') is not None])
result=assess_speed_step(15.,summary['status']=='LAP_COMPLETED',speeds)
reasons=Counter(r['reason'] for r in active)
detail_rows=[r for r in active if r.get('details',{}).get('reference_xy_rear_m')]
def stats(values):
    a=np.array(values,float)
    return dict(min=float(a.min()),median=float(np.median(a)),p95=float(np.quantile(a,.95)),max=float(a.max())) if len(a) else None
result.update(active_command_reasons=dict(reasons),control_sha256=manifest['control.jsonl']['sha256'],
    reference_endpoint_norm_m=stats([np.linalg.norm(r['details']['reference_xy_rear_m'][-1]) for r in detail_rows]),
    minimum_preview_m=stats([r['details']['minimum_preview_distance_m'] for r in detail_rows]),
    selected_lookahead_m=stats([r['details']['selected_lookahead_distance_m'] for r in detail_rows]))
with (root/'evaluation/speed_step.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result))
