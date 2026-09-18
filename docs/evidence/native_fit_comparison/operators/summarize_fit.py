from pathlib import Path
import json,numpy as np
root=Path('/home/thistle/e2e_autonomous');out=root/'runs/native_fit_factorial_20260918'
names=['initial','control','focused','geometry','focused_geometry'];results=[]
front=set(json.loads((out/'resolved_plan.json').read_text())['front_indices'])
for name in names:
    r=json.loads((out/(name+'.json')).read_text());r['front_fit'].pop('cases')
    results.append(dict(arm=name,checkpoint=r['checkpoint'],checkpoint_sha256=r['checkpoint_sha256'],
        front=r['front_fit'],native_fit=r['native_fit'],xy=r['xy'],launch_accepted=sum(v['accepted'] for v in r['launch']),wall_s=r['wall_s']))
    if name!='initial':
        schedule=json.loads((out/(name+'_schedule.json')).read_text());draws=[v for batch in schedule for v in batch]
        results[-1]['presentations']=dict(total=len(draws),old=sum(k=='old' for k,i in draws),
            native=sum(k=='native' for k,i in draws),front=sum(k=='native' and i in front for k,i in draws))
summary=dict(results=results,vs_initializer=json.loads((out/'selection_vs_initializer.json').read_text()),
    vs_deployed=json.loads((out/'selection_vs_deployed.json').read_text()),
    metric_notes={'front':'sample-mean ADE on 110 training windows, PP counters conditional on accepted prediction and teacher |steer| > 0.02 rad',
        'xy':'run-macro ADE on fixed 11505 nominal / 8962 recovery development-validation windows',
        'retention':'Both initializer and original deployed checkpoint references; no automatic promotion',
        'budget':'384 optimizer updates of 32 presentations per arm, same seed and starting checkpoint; not a full epoch'},
    test_usage='sealed',closed_loop_comparison=False)
(out/'summary.json').write_text(json.dumps(summary,indent=2))
print(json.dumps(summary,indent=2))
