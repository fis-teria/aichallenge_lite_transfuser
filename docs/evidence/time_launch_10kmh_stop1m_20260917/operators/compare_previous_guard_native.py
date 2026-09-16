"""Paired scan replay at the preceding 10 km/h trial's first rejection."""
from pathlib import Path
import hashlib
import json
from aic_transfuser_lite.control.turning_scan_guard import check_turning_scan
from aic_transfuser_lite.control.curvature_support_v2 import FIVE_KMH_STOPPING_SPEED, ONE_METRE_STOPPING_TRAVEL
from aic_transfuser_lite.control.vehicle_motion_v1 import AWSIM_10KMH_POLICY

base=Path('/home/thistle/e2e_autonomous/runs')
raw=base/'time_launch_10kmh_stop5_20260917/raw/codex-time-launch10-stop5-lap01'
out=base/'time_launch_10kmh_stop1m_20260917'
data=(raw/'control.jsonl').read_bytes()
manifest={r['path']:r for r in json.loads((raw/'transfer_manifest.json').read_bytes())}
assert hashlib.sha256(data).hexdigest()==manifest['control.jsonl']['sha256']
rows=[json.loads(line) for line in data.splitlines()]
guard=next(r for r in rows if r['event']=='SCAN_GUARD_REJECTED')
scan=guard['scan'];obs=guard['motion_observation']
args=[scan[k] for k in ['ranges','angle_min','angle_increment','range_min','range_max']]
kwargs=dict(speed_mps=guard['speed_mps'],measured_steer_rad=guard['measured_steer_rad'],
    issued_steer_rad=guard['issued_steer_rad'],previous_steer_rad=guard['previous_steer_rad'],
    scan_in_current_rear=guard['scan_in_current_rear'],envelope_policy='curvature_support_v2',
    vehicle_model_policy=AWSIM_10KMH_POLICY,heading_rate_radps=obs['heading_rate_radps'],
    reported_lateral_mps=obs['reported_lateral_mps'])
try:
    check_turning_scan(*args,**kwargs,stopping_distance_policy=FIVE_KMH_STOPPING_SPEED)
except ValueError as exc:
    assert str(exc)=='STOPPING_SWEEP_OCCUPIED'
else:
    raise AssertionError('previous rejection did not replay')
diagnostic=check_turning_scan(*args,**kwargs,stopping_distance_policy=ONE_METRE_STOPPING_TRAVEL)
record=dict(status='PASS',scope='PAIRED_REPLAY_NOT_NEW_CLOSED_LOOP_RESULT',
    previous_control_sha256=manifest['control.jsonl']['sha256'],previous_reason=guard['reason'],
    diagnostic_reason='PASS',diagnostic_guard=diagnostic)
with (out/'previous_stop_comparison.json').open('x') as f:json.dump(record,f,indent=2)
print(json.dumps(record))
