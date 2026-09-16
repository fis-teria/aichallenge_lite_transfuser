"""Compare observed trial outcomes; each setting has a single runtime trial."""
from pathlib import Path
import hashlib
import json

base=Path('/home/thistle/e2e_autonomous/runs')
cases=[('5 km/h, normal guard','time_launch_model_lap_20260917','codex-time-launch-lap01'),
       ('10 km/h, normal guard','time_launch_10kmh_20260917','codex-time-launch10-lap02'),
       ('10 km/h, 5 km/h guard horizon','time_launch_10kmh_stop5_20260917','codex-time-launch10-stop5-lap01'),
       ('10 km/h, 1 m guard horizon','time_launch_10kmh_stop1m_20260917','codex-time-launch10-stop1m-lap01')]
rows=[]
for name,folder,run_id in cases:
    root=base/folder;raw=root/'raw'/run_id
    manifest={r['path']:r for r in json.loads((raw/'transfer_manifest.json').read_bytes())}
    config_data=(raw/'trial_config.json').read_bytes();host_data=(raw/'host_result.json').read_bytes()
    for path,data in [('trial_config.json',config_data),('host_result.json',host_data)]:
        assert hashlib.sha256(data).hexdigest()==manifest[path]['sha256']
    config=json.loads(config_data);host=json.loads(host_data)
    summary=json.loads((root/'evaluation/summary.json').read_bytes())
    assert config['checkpoint_sha256']=='1fe12ca066791d3eb8907c6921120e2cfbb38925c422d5be54940f4acbdd120a'
    rows.append(dict(condition=name,run_id=run_id,summary_status=summary['status'],
        target_speed_kmh=config['speed_cap_mps']*3.6,stopping_distance_policy=config.get('stopping_distance_policy','measured_speed_v1'),
        travel_from_authorization_m=summary['recorded_pose_travel_m'],judge_lap_confirmed=summary['judge_lap_confirmed'],
        judge_laps=summary['judge_laps'],judge_sections=summary['judge_sections'],host_error=host.get('error'),
        host_sha256=manifest['host_result.json']['sha256'],config_sha256=manifest['trial_config.json']['sha256']))
out=base/'time_launch_10kmh_stop1m_20260917/evaluation/comparison.json'
with out.open('x') as f:json.dump(dict(status='PASS',scope='ONE_OBSERVED_RUN_PER_SETTING_NOT_REPEATABILITY_OR_BRAKING_VALIDATION',runs=rows),f,indent=2)
print(json.dumps(rows))
