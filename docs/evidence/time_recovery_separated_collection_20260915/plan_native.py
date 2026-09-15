"""New finite campaign from verified normals and previously frozen random sites."""
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

from aic_transfuser_lite.data.time_nominal_steering_guide_v1 import POLICY, NominalSteeringGuide
from aic_transfuser_lite.data.time_random_steering_pulse_v1 import SCHEMA, PulseSite, RandomPulseConfig, validate_random_guide
from aic_transfuser_lite.data.time_steering_pulse_v1 import SteeringPulseConfig
sys.path.insert(0, str(Path.cwd()/'tools'))
from analyze_time_normal_lap_attribution import verify_files

BASE = Path('/home/thistle/e2e_autonomous/runs/time_recovery_sites_20260915')
RAW = Path('/home/thistle/e2e_autonomous/raw/time_recovery_sites_20260915')
OUT = BASE.with_name('time_recovery_separated_20260915')

def write(p, value):
    if p.exists():
        assert read(p)==json.loads(json.dumps(value,allow_nan=False)), str(p)
        return
    with p.open('x') as f:
        json.dump(value, f, indent=2, allow_nan=False)

def read(p):
    return json.loads(p.read_bytes())

old = read(BASE/'selected_site_plan.json')
assert hashlib.sha256((BASE/'selected_site_plan.json').read_bytes()).hexdigest() == 'f953ab99306e6d483e8314d50a6168bb0e513b01c2f1a824647efdb3347bd328'
proofs = {}; steering = {}; references = {}
for name in old['normal_runs']:
    raw = RAW/name
    proofs[name] = verify_files(raw, ('control.jsonl', 'reference.json', 'result.json'))
    assert proofs[name] == old['source_hashes'][name]
    result = read(raw/'result.json')
    assert result['status']=='COMPLETE_LAP' and result['last_control']['stop_confirmed'] and not result['last_control']['fault']
    assert result['nodes']['closed_bag'] and not result['cleanup_errors']
    segments = [[]]
    for r in [json.loads(s) for s in (raw/'control.jsonl').read_text().splitlines()]:
        if r.get('reason')!='RECOVERY_TEACHER_TRACKING' or not r.get('publication') or not r.get('projection'):
            continue
        if segments[-1] and r['projection']['s_m'] < segments[-1][-1]['projection']['s_m']-5.:
            segments.append([])
        segments[-1].append(r)
    rows = max(segments, key=len)
    unique = {}
    for r in rows:
        unique.setdefault(r['projection']['s_m'], [r['projection']['s_m'], r['issued_angle_rad']])
    steering[name] = NominalSteeringGuide([unique[s] for s in sorted(unique)])
    references[name] = read(raw/'reference.json')
    full = read(BASE/(name+'_full_guide.json'))
    assert full['control_sha256']==proofs[name]['control.jsonl']
    write(OUT/(name+'_full_guide.json'), full)

normal, other = old['normal_runs']
guide = read(OUT/(normal+'_full_guide.json'))['guide']
sg = steering[normal]
sample = np.linspace(max(sg.values[0,0],steering[other].values[0,0]), min(sg.values[-1,0],steering[other].values[-1,0]), 3500)
differences = np.abs([sg.at(s)-steering[other].at(s) for s in sample])
stop = {**old['stop_site'], 'start_s_m':117., 'target_progress_m':[119.,120.]}
eligible = read(BASE/'candidate_site_audit.json')['candidates']
assert any(s['start_s_m']==117. and min(s['eligible_counts'].values())>=5 for s in eligible)
additional = {s['site_id']:s for s in old['additional_sites']}
groups = [[stop], [additional[k] for k in ('R02','R05','R09')],
          [additional[k] for k in ('R03','R06','R10')], [additional[k] for k in ('R01','R04','R08')], [additional['R07']]]
template = SteeringPulseConfig(10., .1, duration_s=1.5, plateau_s=1., start_window_m=2., goal_min_elapsed_s=1.25)
dest = OUT/'planned_references'; dest.mkdir(exist_ok=True)
runs = []
for pair, group in enumerate(groups,1):
    for side,sign in (('left',1),('right',-1)):
        sites = tuple(PulseSite(s['site_id'],s['start_s_m'],sign) for s in group)
        cfg = RandomPulseConfig(915200+pair*2+(sign<0),max_events=len(sites),sites=sites,
            start_min_m=sites[0].start_s_m,start_max_m=sites[-1].start_s_m,control_policy=POLICY)
        validate_random_guide(cfg,template,guide)
        for site in sites:
            for s in np.linspace(site.start_s_m-3.,site.start_s_m+5.,81):
                assert abs(sg.at(s)-steering[other].at(s))<.035, dict(site=site.site_id,s=s,normal=sg.at(s),other=steering[other].at(s))
                if site.start_s_m<=s<=site.start_s_m+template.start_window_m+1.4*template.duration_s:
                    assert abs(sg.at(s))+.1<=.5, dict(site=site.site_id,s=s,normal=sg.at(s))
        ref = {**references[normal], 'side':side}
        ref['steering_pulse'] = dict(schema=SCHEMA,config=asdict(template),random_config=asdict(cfg),
            nominal_run_id=normal,nominal_control_sha256=proofs[normal]['control.jsonl'],nominal_guide=guide,
            steering_guide=sg.values.tolist(),site_selection_seed=915110)
        name=f'group{pair:02d}_{side}.json'; p=dest/name; write(p,ref)
        runs.append(dict(run_id=f'codex-time-recovery-separated-g{pair:02d}-{side}',side=side,kind='recovery',pair=pair,
            split='validation' if pair==4 else 'train',site_selection_seed=915110,seed=cfg.seed,
            sites=[asdict(s) for s in sites],reference=name,reference_sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
plan = dict(schema='separated_nominal_guide_recovery_collection_v1',control_policy=POLICY,
    previous_plan_sha256=hashlib.sha256((BASE/'selected_site_plan.json').read_bytes()).hexdigest(),
    normal_runs=old['normal_runs'],source_hashes=proofs,seed=915110,stop_site=stop,additional_sites=old['additional_sites'],
    maximum_recovery_runs=10,maximum_pulse_events=22,batch_size=2,runs=runs,
    expansion_requires='S00_BOTH_RECOVERED_AND_VALIDATED_PLATEAU',guard_thresholds_changed=False,
    pilot_acceptance=dict(min_issued_plateau_s=.95,minimum_signed_lateral_m=.10,
        stop_region_minimum_signed_lateral_m=.08,minimum_anchors_per_event=60),
    steering_guide_shape=list(sg.values.shape),normal_steering_difference_p99_rad=float(np.percentile(differences,99)),
    new_model_training=False,sealed_test_read=False)
write(OUT/'selected_site_plan.json',plan)
print(json.dumps(dict(status='NEW_FINITE_PLAN_READY',runs=len(runs),events=22,
    steering_guide_shape=plan['steering_guide_shape'],normal_difference_p99=plan['normal_steering_difference_p99_rad'])))
