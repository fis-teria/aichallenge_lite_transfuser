"""Screen unchanged missing goals, then pack separate whole-run split plans."""
from pathlib import Path
from dataclasses import asdict, replace
import itertools
import json
import math
import subprocess
import sys

import numpy as np

sys.path.insert(0,'tools')
from generate_time_large_recovery_reference import measured_normal_trace
from aic_transfuser_lite.data.time_large_recovery_reference_v1 import preparation_course
from aic_transfuser_lite.data.time_large_recovery_v1 import LargeRecoveryConfig, LargeRecoverySite
from aic_transfuser_lite.data.time_recovery_collection_v1 import load_pose_course
from aic_transfuser_lite.data.recovery_reference_v3 import load_occupancy_map_v3

ROOT=Path('/home/thistle/e2e_autonomous');OUT=ROOT/'runs/time_corner_gap_20260916'
INPUTS=ROOT/'runs/time_recovery_collection_20260913/inputs'
NORMAL=ROOT/'raw/time_recovery_sites_20260915/codex-time-recovery-sites-normal-n03'
PROOF=ROOT/'runs/time_recovery_separated_20260915/selected_site_plan.json'
read=lambda p:json.loads(p.read_bytes())
normal=measured_normal_trace(NORMAL,read(PROOF)['source_hashes'][NORMAL.name])
base=load_pose_course(INPUTS/'base.csv');occ=load_occupancy_map_v3(INPUTS/'occupancy_grid_map.yaml')
controls=[json.loads(s) for s in (NORMAL/'control.jsonl').read_text().splitlines()]
missing={};catalogs={};selected={};attempts=[]


def normal_entry_stability(site: LargeRecoverySite) -> dict:
    since=None;previous=None;longest=0.
    for row in controls:
        if row.get('reason')!='RECOVERY_TEACHER_TRACKING' or not row.get('projection'):continue
        s=row['projection']['s_m'];now=row['sim_ns']
        if not site.start_s_m-5.<=s<=site.start_s_m+1.:continue
        clear=(row.get('guard') or {}).get('minimum_ray_margin_m',-1.)
        ready=clear>=.4 and row['speed_mps']>=1.15
        if previous is None or now-previous>150_000_000:since=None
        since=(since if since is not None else now) if ready else None
        if s>=site.start_s_m and since is not None:longest=max(longest,(now-since)/1e9)
        previous=now
    return dict(normal_predicted_ready=longest>=1.,longest_stable_s=longest,
                scope='same_observed_normal_guide_not_a_new_runtime_admission')


for cm in (20,40,60):
    old=ROOT/'runs'/f'time_corner_multiscale{cm}_20260916'
    coverage=read(ROOT/'runs/time_corner_multiscale20_20260916/combined_20cm_coverage.json') if cm==20 else read(old/'coverage_final.json')
    missing[str(cm)]=coverage['missing'];catalog=read(old/'catalog.json');catalogs[str(cm)]=catalog
    for row in catalog['corners']:
        original=LargeRecoverySite(**row['site'])
        if not any(original.corner_id in names for names in coverage['missing'].values()):continue
        key=f'{cm}:{original.site_id}'
        bias=math.copysign({20:.02,40:.04,60:.06}[cm],-original.target_offset_m)
        heading_bias=math.copysign(math.radians(1.),original.target_heading_rad)
        chosen=None;fallback=None
        variants=[(original.approach_distance_m,original.settle_distance_m,original.return_length_m,'nominal_path')]
        variants+=list(itertools.product((4.,6.,8.),(2.,4.),(4.,6.,10.),('nominal_path','measured_normal')))
        for approach,settle,ret,origin in dict.fromkeys(variants):
            site=replace(original,approach_distance_m=approach,settle_distance_m=settle,return_length_m=ret,
                preparation_origin=origin,preparation_offset_bias_m=bias,preparation_heading_bias_rad=heading_bias)
            cfg=LargeRecoveryConfig((site,),speed_policy='record_actual_v1',entry_heading_tolerance_rad=math.radians(2.),
                recovery_duration_s=15.,map_screen_policy='oriented_body_v1')
            try:
                _,evidence=preparation_course(base,normal,cfg,occ)
                readiness=normal_entry_stability(site)
                candidate=dict(key=key,cm=cm,site=asdict(site),map_pass=True,evidence=evidence,entry_prediction=readiness)
                attempts.append(candidate)
                if readiness['normal_predicted_ready']:
                    chosen=candidate;break
                if fallback is None:fallback=candidate
            except ValueError as exc:
                if not str(exc).startswith(('LARGE_SITE_MAP_REJECTED','LARGE_PREPARATION_PREVIEW_TOO_SHORT')):raise
                attempts.append(dict(key=key,site=asdict(site),map_pass=False,reason=str(exc)))
        if chosen is None and fallback is not None:chosen=fallback
        if chosen:selected[key]=chosen
        print(json.dumps(dict(key=key,selected=chosen,attempts=sum(a['key']==key for a in attempts))),flush=True)

(OUT/'plans').mkdir();(OUT/'references').mkdir()
with (OUT/'candidate_screen.json').open('x') as f:json.dump(dict(missing_before=missing,catalogs=catalogs,selected=selected,attempts=attempts),f,indent=2)

plans=[]
for split in ('train','validation'):
    pending=[r for key,r in selected.items() if r['site']['corner_id'] in missing[str(r['cm'])][split]]
    groups=[]
    for row in sorted(pending,key=lambda r:(LargeRecoverySite(**r['site']).start_s_m,r['cm'])):
        site=LargeRecoverySite(**row['site'])
        group=next((g for g in groups if len(g)<3 and site.start_s_m-LargeRecoverySite(**g[-1]['site']).start_s_m>=40.),None)
        if group is None:group=[];groups.append(group)
        group.append(row)
    for i,group in enumerate(groups,1):
        name=f'{split}_lap{i:02}';side='left' if split=='train' else 'right'
        plan=dict(name=name,split=split,seed=20260916+(0 if split=='train' else 100),event_cap=len(group),
            candidates=[r['site'] for r in group],required_site_ids=[r['site']['site_id'] for r in group],
            speed_policy='record_actual_v1',entry_heading_tolerance_rad=math.radians(2.),recovery_duration_s=15.,
            map_screen_policy='oriented_body_v1',goal_keys=[r['key'] for r in group])
        with (OUT/'plans'/(name+'.json')).open('x') as f:json.dump(plan,f,indent=2)
        log=OUT/(name+'_generation.log')
        cmd=[sys.executable,'tools/generate_time_large_recovery_reference.py','--inputs',str(INPUTS),'--normal-run',str(NORMAL),
             '--normal-proof',str(PROOF),'--plan',str(OUT/'plans'/(name+'.json')),'--side',side,'--output',str(OUT/'references'/name)]
        with log.open('x') as f:p=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,timeout=120)
        if p.returncode:raise RuntimeError(log.read_text()[-3000:])
        plans.append(plan)
        print(json.dumps(dict(generated=name,goals=plan['goal_keys'])),flush=True)
with (OUT/'initial_schedule.json').open('x') as f:json.dump(dict(maximum_attempts=12,plans=plans,
    deferred=[f'{cm}:{name}' for cm,v in missing.items() for name in sorted(set(v['train']+v['validation'])) if f'{cm}:{name}' not in selected]),f,indent=2)
