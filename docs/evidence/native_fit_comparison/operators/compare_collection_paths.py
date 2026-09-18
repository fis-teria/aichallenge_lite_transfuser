from pathlib import Path
import json,numpy as np
ROOT=Path('/home/thistle/e2e_autonomous/runs/mppi_v45_pc10_20260918')
baseline='lidar-v45-pc10-counterfactual-empty5-a01'
def read(run):
    folder=ROOT/'collected'/run;manifest=json.loads((folder/'export_manifest.json').read_text())
    import hashlib
    path=folder/'raw'/run/'samples.jsonl'
    assert hashlib.sha256(path.read_bytes()).hexdigest()==manifest['files'][str(path.relative_to(folder))]['sha256']
    rows=[json.loads(s) for s in path.read_text().splitlines()]
    values={r['ego_gt']['progress_m']:(r['ego_gt']['d'],r['ego']['stamp']) for r in rows
        if r.get('time',-1)>=0 and r.get('ego_gt',{}).get('source')=='gnss' and r.get('ego')}
    s=np.array(sorted(values));return s,np.array([values[v] for v in s])
bs,bv=read(baseline);reports=[]
for folder in sorted((ROOT/'collected').glob('lidar-v45-pc10-front-*')):
    run=folder.name;s,v=read(run);meta=json.loads((folder/'provenance/scenarios'/f'{run}.json').read_text())
    station=meta['locations'][0]['monitor_s_m'];report=dict(run_id=run,station_m=station,
        speed_comparison='matched configured 5 km/h' if run!='lidar-v45-pc10-front-cone-a02' else '10 vs 5 km/h, speed-confounded')
    for phase,lo,hi in [('approach',station-10,station-2),('pass',station-2,station+3),('recovery',station+10,station+23)]:
        mask=(s>=max(lo,bs[0]))&(s<=min(hi,bs[-1]));delta=v[mask,0]-np.interp(s[mask],bs,bv[:,0])
        report[phase]=dict(samples=int(mask.sum()),median_lateral_difference_m=float(np.median(delta)) if len(delta) else None,
            maximum_absolute_lateral_difference_m=float(abs(delta).max()) if len(delta) else None)
    reports.append(report)
out=ROOT/'front_path_comparison.json'
out.write_text(json.dumps(dict(baseline=baseline,runs=reports,scope='GNSS monitor lateral position interpolated by route station; observational paired scenario comparison, not synchronized pose truth or contact certification'),indent=2))
print(out.read_text())
