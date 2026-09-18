from pathlib import Path
import json,hashlib,math
root=Path('/home/thistle/e2e_autonomous/runs/mppi_v45_pc10_20260918')
names=['lidar-v45-pc10-front-'+s for s in ['close-entry-a02','close-entry-control-a01','entry-normal-a01','close10-on-a01','close10-control-a01','normal10-on-a01','normal10-control-a01']]
summaries=[]
for name in names:
    collected=root/'collected'/name
    if not collected.exists():continue
    manifest=json.loads((collected/'export_manifest.json').read_text())
    def read(rel):
        data=(collected/rel).read_bytes()
        expected=manifest['files'][rel]
        assert len(data)==expected['bytes'] and hashlib.sha256(data).hexdigest()==expected['sha256'],rel
        return data.decode()
    raw='raw/'+name+'/'
    samples=[json.loads(s) for s in read(raw+'samples.jsonl').splitlines()]
    rows=[r for r in samples if r.get('time',-1)>=0 and r.get('ego') and r.get('ego_gt',{}).get('source')=='gnss']
    result=json.loads(read(raw+'result.json'))
    details=json.loads(read(raw+'d1-result-details.json'))
    identity=json.loads(read(raw+'d1/teacher-runtime-identity.json'))
    scenario=json.loads(read('provenance/scenarios/'+name+'.json'))
    obj=scenario['locations'][0]['map_pose'][:2]
    first,last=rows[0],rows[-1];stamp=first['ego']['stamp']
    first60=[r for r in rows if r['ego']['stamp']-stamp<=60.]
    def relative(r):
        e=r['ego'];dx,dy=obj[0]-e['x'],obj[1]-e['y'];c,s=math.cos(e['yaw']),math.sin(e['yaw'])
        return [dx*c+dy*s,-dx*s+dy*c]
    summaries.append(dict(run_id=name,early_entry_search=identity.get('early_entry_search',False),
      target_kmh=identity['speed_cap_mps']*3.6, max_speed_kmh=max(r['ego']['speed_mps'] for r in rows)*3.6, verdict=result.get('scenario_verdict'),penalties=details['penalty_by_kind'],
      initial_pose=first['ego'],initial_object_body_m=relative(first),final_object_body_m=relative(last),
      sim_duration_s=last['ego']['stamp']-stamp,progress_start_m=first['ego_gt']['progress_m'],
      progress_end_m=last['ego_gt']['progress_m'],advance_m=last['ego_gt']['progress_m']-first['ego_gt']['progress_m'],
      advance_first60sim_m=first60[-1]['ego_gt']['progress_m']-first['ego_gt']['progress_m'],
      max_progress_m=max(r['ego_gt']['progress_m'] for r in rows),final_speed_mps=last['ego']['speed_mps'],
      stopped_fraction_first60_samples=sum(abs(r['ego']['speed_mps'])<.05 for r in first60)/len(first60),
      source_manifest_sha256=identity['source_manifest_sha256'],runtime_identity=identity,
      train_admission='not admitted; diagnostic trial pending teacher quality audit'))
out=root/'close10_comparison.json'
out.write_text(json.dumps(dict(runs=summaries,limitations='One run per setting; sampled GNSS progress and native placement for evaluation only. Counts and durations do not prove statistical improvement. No candidate is promoted to training by this summary.'),indent=2)+'\n')
print(json.dumps([{k:v for k,v in r.items() if k in ['run_id','early_entry_search','target_kmh','max_speed_kmh','verdict','advance_m','sim_duration_s']} for r in summaries],indent=2))
