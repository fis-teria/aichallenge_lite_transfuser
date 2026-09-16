from pathlib import Path
import json
base=Path(r'\\wsl.localhost\Ubuntu-22.04-Recovered\home\thistle\e2e_autonomous\runs')
rows=[]
for cm in (20,40,60):
 for p in sorted((base/f'time_corner_multiscale{cm}_20260916').glob('*_collection_summary.json')):
  d=json.loads(p.read_text());rows.append(dict(cm=cm,run=d['run_id'],split=d['split'],status=d['result_status'],fault=d['fault'],accepted=d['accepted'],events=[dict(site=e['site_id'],recovered=e['recovery_confirmed'],anchors=e.get('accepted_camera_anchors',0),target=e.get('target_band_camera_anchors',0),speed=e.get('peak_observed_speed_mps'),overspeed_samples=e.get('above_legacy_speed_samples')) for e in d['events']]))
print(json.dumps(rows))
