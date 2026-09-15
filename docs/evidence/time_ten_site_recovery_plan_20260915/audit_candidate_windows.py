"""Offline planning screen; nominal observations cannot certify perturbation safety."""
from pathlib import Path
import json,math,sys
import numpy as np
sys.path.insert(0,str(Path.cwd()/'tools'))
from analyze_time_normal_lap_attribution import verify_files
root=Path('/home/thistle/e2e_autonomous/runs/time_ten_site_recovery_plan_20260915');root.mkdir(exist_ok=False)
rawroot=Path('/home/thistle/e2e_autonomous/raw/time_recovery_speed_20260914')
locations=json.loads(Path('docs/evidence/time_objective_driving_comparison_20260915/collection_locations.json').read_text())
stops=[r[-1]['nearest_teacher_progress_m'] for r in locations['runs'].values()]
excluded=[math.floor(min(stops)-10),math.ceil(max(stops)+10)]
window=2.;max_reserved_distance=1.4*(2+10+3+1)
traces={};proof={};xyref=None
for n in (30,31):
    rid=f'codex-time-recovery-speedbase-r{n}';raw=rawroot/rid
    proof[rid]=verify_files(raw,('control.jsonl','reference.json','result.json'))
    assert json.loads((raw/'result.json').read_text())['status']=='COMPLETE_LAP'
    data=[json.loads(x) for x in (raw/'control.jsonl').read_text().splitlines()]
    traces[rid]=[r for r in data if r.get('reason')=='RECOVERY_TEACHER_TRACKING' and r.get('projection') and r.get('publication')]
    ref=json.loads((raw/'reference.json').read_text());value=np.asarray(ref['baseline_xy_m'],float)
    assert value.ndim==2 and value.shape[1]==2 and np.isfinite(value).all()
    if xyref is None:xyref=value
    else:np.testing.assert_array_equal(xyref,value)

def probe(rows,s):
    region=[r for r in rows if s-3<=r['projection']['s_m']<=s+window]
    since=None;previous=None;good=[]
    for r in region:
        t=r['publication']['sim_ns']
        valid=1.15<=r['speed_mps']<=1.4 and abs(r['nominal_angle_rad'])+.1<=.5 and r['guard'].get('minimum_ray_margin_m',-1)>=.5
        if previous is None or not 0<t-previous<=150_000_000:since=None
        since=(t if since is None else since) if valid else None
        if since is not None and t-since>=1_000_000_000 and s<=r['projection']['s_m']<=s+window:
            good.append(r)
        previous=t
    return good

eligible=[]
for s in range(15,326):
    # Reserve both approach and event/future tail outside the provisional stop-zone exclusion.
    if not (s+window+max_reserved_distance<excluded[0] or s-5>excluded[1]):continue
    by_run={name:probe(rows,s) for name,rows in traces.items()}
    if all(by_run.values()):
        r=next(iter(by_run.values()))[0]
        eligible.append(dict(start_s_m=s,map_xy_m=[r['current_pose']['x_m'],r['current_pose']['y_m']],
            nominal_entry_eligible_counts={k:len(v) for k,v in by_run.items()},
            nominal_minimum_ray_margin_m=min(x['guard']['minimum_ray_margin_m'] for values in by_run.values() for x in values),
            nominal_maximum_absolute_steer_rad=max(abs(x['nominal_angle_rad']) for values in by_run.values() for x in values)))
# A maximal early-finish packing on a line; 28 m is a planning spacing, never a runtime release gate.
spaced=[]
for p in eligible:
    if not spaced or p['start_s_m']-spaced[-1]['start_s_m']>=28:spaced.append(p)
result=dict(scope='PLANNING_ONLY_NOMINAL_ENTRY_SCREEN_NOT_RUNTIME_APPROVAL',source_hashes=proof,
    stop_progress_m=stops,provisional_excluded_progress_m=excluded,excluded_zone_requires_later_scene_validation=True,
    candidate_start_window_m=window,approach_reserve_m=5.,event_and_tail_distance_reserve_m=max_reserved_distance,
    minimum_planning_spacing_m=28.,eligible_starts=eligible,maximal_spaced_candidates=spaced,
    missing_checks=['matched normal guide lateral and heading at entry','exact ten-site schedule','perturbed steering and obstacle response','current live scene identity','cross-lap completion and wrap handling'],
    new_awsim_run=False,new_teacher_data=False)
with (root/'candidate_window_audit.json').open('x') as f:json.dump(result,f,indent=2,allow_nan=False)
print(json.dumps(dict(status='PLANNING_ONLY',excluded=excluded,eligible=len(eligible),spaced=[r['start_s_m'] for r in spaced],output=str(root/'candidate_window_audit.json'))))
