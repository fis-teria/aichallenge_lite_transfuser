"""Explain the first aborted preparation and which later targets stayed untried."""
from pathlib import Path
import json
import math

OUT = Path('/home/thistle/e2e_autonomous/runs/time_corner_recovery_20260916')
RAW = Path('/home/thistle/e2e_autonomous/raw/time_corner_recovery_20260916')
reports = []
for summary in sorted(OUT.glob('*_collection_summary.json')):
    report = json.loads(summary.read_bytes())
    raw = RAW / report['run_id']
    result = json.loads((raw / 'result.json').read_bytes())
    state = (result['last_control'].get('large_recovery') or {}).get('state') or {}
    if state.get('stage') != 'aborted':
        continue
    rows = [json.loads(line) for line in (raw / 'control.jsonl').read_text().splitlines()]
    first = next(r for r in rows if r.get('large_recovery', {}).get('state', {}).get('stage') == 'aborted')
    data = first['large_recovery']
    cursor = data['state']['site_cursor']
    reports.append(dict(run_id=report['run_id'], site_id=report['planned'][cursor]['site_id'],
        reason=data['state']['reason'], sim_ns=first['sim_ns'], base_s_m=first['projection']['s_m'],
        measured_speed_mps=first['speed_mps'], fixed_target_mps=first['target_speed_mps'],
        preparation_speed_max_mps=1.4, lateral_m=data['lateral_error_m'],
        heading_deg=math.degrees(data['heading_error_rad']),
        later_targets_untried=[s['site_id'] for s in report['planned'][cursor + 1:]],
        accepted_anchors_from_earlier_events=report['accepted'],
        failed_event_accepted_anchors=next(e.get('accepted_camera_anchors', 0) for e in report['events'] if e['site_id'] == report['planned'][cursor]['site_id'])))
assert all(r['failed_event_accepted_anchors'] == 0 for r in reports)
with (OUT / 'aborted_preparation_diagnosis.json').open('x') as stream:
    json.dump(reports, stream, indent=2, allow_nan=False)
print(json.dumps(reports), flush=True)
