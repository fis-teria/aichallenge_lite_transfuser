"""Separate absence of recovery teachers from the stricter entry-frame target."""
from pathlib import Path
import json
from aic_transfuser_lite.data.time_large_recovery_v1 import LargeRecoverySite,collection_speed_eligible

out=Path('/home/thistle/e2e_autonomous/runs/time_corner_gap2_20260916')
read=lambda p:json.loads(p.read_bytes())
screen=read(out/'candidate_screen.json');coverage=read(out/'coverage_final.json')
conditions=[]
for cm in ('20','40','60'):
    roots=[out,out.parent/'time_corner_gap_20260916',out.parent/('time_corner_multiscale'+cm+'_20260916')]
    if cm=='20':roots.append(out.parent/'time_corner_recovery_20260916')
    audits=[]
    for root in roots:
        for path in sorted(root.glob('*_collection_summary.json')):
            report=read(path)
            if report['accepted']:
                audits.append((report,read(root/(report['run_id']+'_anchor_states.json'))))
    for split,corners in coverage['missing_after'][cm].items():
        for corner in corners:
            site=LargeRecoverySite(**next(c['site'] for c in screen['catalogs'][cm]['corners'] if c['site']['corner_id']==corner))
            events=[]
            for report,states in audits:
                if report['split']!=split:continue
                for e in report['events']:
                    if (e.get('corner_id')!=corner or e['target_offset_m']!=site.target_offset_m
                        or e['target_heading_rad']!=site.target_heading_rad or not e['completed']
                        or not e.get('accepted_camera_anchors',0)):
                        continue
                    anchors=[a for a in states if a['event_id']==e['event_id'] and a['site_id']==site.site_id]
                    entry=[a for a in anchors if site.release_s_m-.5<=a['base_s_m']<=site.release_s_m+3.
                        and site.at_goal(a['lateral_m'],a['heading_rad'])
                        and collection_speed_eligible(a['speed_mps'],e.get('speed_policy','bounded_5kmh_v1'))]
                    events.append(dict(run_id=report['run_id'],event_id=e['event_id'],anchors=len(anchors),entry_anchors=len(entry)))
            conditions.append(dict(amplitude_cm=int(cm),split=split,corner_id=corner,accepted_events=len(events),
                accepted_anchors=sum(e['anchors'] for e in events),total_entry_anchors=sum(e['entry_anchors'] for e in events),
                maximum_entry_anchors_in_one_event=max((e['entry_anchors'] for e in events),default=0),events=events,
                category='ADOPTED_RECOVERY_WITHOUT_STRICT_ENTRY_COVERAGE' if events else 'NO_ADOPTED_RECOVERY_FOR_ORIGINAL_CONDITION'))
report=dict(conditions=conditions,missing_conditions=len(conditions),
    with_adopted_recovery=sum(bool(r['accepted_events']) for r in conditions),
    without_adopted_recovery=sum(not r['accepted_events'] for r in conditions),
    scope='Counts across sealed collections; no data merge or training, and no relaxation of original coverage rules')
with (out/'remaining_coverage_diagnosis.json').open('x') as f:json.dump(report,f,indent=2)
print(json.dumps({k:v for k,v in report.items() if k!='conditions'}),flush=True)
