"""Measure entry-goal formation and causal camera coverage from closed raw."""
from pathlib import Path
import argparse
import json
import math
from aic_transfuser_lite.data.time_large_recovery_v1 import LargeRecoverySite

parser = argparse.ArgumentParser()
parser.add_argument('--pair', type=int, required=True)
args = parser.parse_args()
root = Path('/home/thistle/e2e_autonomous')
out = root / 'runs/time_corner_gap2_20260916'
raw = root / 'raw/time_corner_gap2_20260916'
read = lambda p: json.loads(p.read_bytes())
records = []
for domain in (1, 2):
    name = f'codex-time-recovery-cornergap2-p{args.pair:02}-d{domain}'
    summary = read(out / (name + '_collection_summary.json'))
    result = read(raw / name / 'result.json')
    controls = [json.loads(line) for line in (raw / name / 'control.jsonl').read_text().splitlines()]
    anchors = read(out / (name + '_anchor_states.json')) if summary['accepted'] else []
    sites = []
    for planned in summary['planned']:
        site = LargeRecoverySite(**planned)
        event = next((e for e in summary['events'] if e['site_id'] == site.site_id), None)
        entry = dict(site=planned, event=event)
        if event is not None:
            rows = [r for r in controls if (r.get('large_recovery') or {}).get('applied')
                    and r['large_recovery']['state']['event_id'] == event['event_id']]
            def pose(r):
                lr = r['large_recovery']
                return dict(sim_ns=r['sim_ns'], s_m=r['projection']['s_m'], lateral_m=lr['lateral_error_m'],
                    heading_deg=math.degrees(lr['heading_error_rad']), stage=lr['state']['stage'])
            preparations = [r for r in rows if r['large_recovery']['command_source'] == 'preparation']
            closest = min(preparations, key=lambda r: max(abs(r['large_recovery']['lateral_error_m']-site.target_offset_m)/.05,
                        abs(r['large_recovery']['heading_error_rad']-site.target_heading_rad)/site.heading_tolerance_rad))
            entry['closest_preparation'] = pose(closest)
            entry['last_preparation'] = pose(preparations[-1])
            entry['requested_pose'] = next((pose(r) for r in rows if r['large_recovery']['state']['stage']=='handover'), None)
            entry['release_pose'] = next((pose(r) for r in rows if r['phase']=='recovery'), None)
            matched = [a for a in anchors if a['event_id']==event['event_id']]
            goal = [a for a in matched if site.release_s_m-.5 <= a['base_s_m'] <= site.release_s_m+3.
                    and site.at_goal(a['lateral_m'],a['heading_rad'])]
            entry['entry_state_anchors'] = len(goal)
            entry['first_causal_anchors'] = matched[:8]
            entry['end_reasons'] = sorted({r['large_recovery']['state']['reason'] for r in rows if r['large_recovery']['state']['reason']})
        sites.append(entry)
    record = dict(run_id=name, status=result['status'], fault=summary['fault'], accepted=summary['accepted'], sites=sites)
    records.append(record)
    print(json.dumps(record), flush=True)
with (out/f'pair{args.pair:02}_entry_diagnosis.json').open('x') as stream:
    json.dump(records, stream, indent=2, allow_nan=False)
