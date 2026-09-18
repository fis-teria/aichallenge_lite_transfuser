from pathlib import Path
import collections,json
root=Path('/home/thistle/e2e_autonomous/runs/mppi_v45_pc10_20260918/collect10_curated_v1')
report=json.loads((root/'selection_manifest.json').read_text());cfg=report['config']
rows=[]
for r in report['runs']:
    counts=collections.Counter()
    for line in (root/r['run_id']/'decisions.jsonl').read_text().splitlines():
        d=json.loads(line)
        if 'SCAN_MAP_ALIGNMENT_OR_SUPPORT' not in d['reasons']:continue
        counts['held_by_scan_map']+=1;e=d['evidence']
        for key,failed in {
          'median_distance_above_015m':e['map_median_max_m']>cfg['maximum_map_median_m'],
          'inlier_fraction_below_070':e['map_inlier_fraction_min']<cfg['minimum_map_inlier_fraction'],
          'wall_points_below_80':e['wall_points_min']<cfg['minimum_wall_points'],
          'angular_support_below_pi_over_3':e['scan_span_min_rad']<cfg['minimum_scan_span_rad']}.items():
            counts[key]+=int(failed)
    rows.append(dict(run_id=r['run_id'],counts=dict(counts)))
result=dict(runs=rows,totals=dict(sum((collections.Counter(r['counts']) for r in rows),collections.Counter())),
            note='Threshold occurrences overlap; this does not by itself identify odometry as the cause.')
out=root.parent/'collect10_hold_breakdown.json'
with out.open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result['totals']))
