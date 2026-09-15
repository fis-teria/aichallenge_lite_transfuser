"""Verify immutable run evidence and index observed 60 cm recovery teachers."""
from pathlib import Path
import hashlib,json,subprocess
import numpy as np

OUT=Path('/home/thistle/e2e_autonomous/runs/time_recovery_60cm_20260916')
RAW=Path('/home/thistle/e2e_autonomous/raw/time_recovery_60cm_20260916')
read=lambda p:json.loads(p.read_bytes())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    verified={};receipts={}
    for p in sorted(OUT.glob('*_verified.json')):
        v=read(p)
        assert v['all_files_and_directory_structure_identical'] and v['all_sqlite_quick_checks_passed']
        receipts[p.name]=dict(sha256=sha(p),archive_sha256=v['archive_sha256'],snapshot_sha256=v['snapshot_sha256'])
        for r in v['runs']:
            assert r['run_id'] not in verified and Path(r['raw_path'])==RAW/r['run_id']
            verified[r['run_id']]=r
    assert 1<=len(verified)<=8 and len(list(OUT.glob('*_collection_summary.json')))==len(verified)
    rows=[];splits={'train':[],'validation':[]};totals={};bands={};max_gap=0.
    for name,v in verified.items():
        assert name.startswith('codex-time-recovery-60cm-d60-g03-')
        raw=RAW/name;r=read(OUT/(name+'_collection_summary.json'))
        result=read(raw/'result.json');ref=read(raw/'reference.json')
        assert r['result_status']==result['status'] and r['fault']==result['last_control']['fault']
        assert r['closed_bag'] and r['stop_confirmed'] and r['event_cap']==3
        assert {abs(s['target_offset_m']) for s in r['planned']}=={.6}
        split=r['split'] if r['accepted'] else 'excluded'
        row=dict(run_id=name,split=split,target_abs_lateral_m=.6,status=r['result_status'],fault=r['fault'],
            event_cap=3,accepted=r['accepted'],target_band_anchors=r['target_band_anchors'],
            completed_events=sum(e['completed'] for e in r['events']),
            accepted_events=sum(e.get('accepted_camera_anchors',0)>0 for e in r['events']),
            all_planned_recovered=r['all_planned_recovered'],three_event_qualified=r['three_event_qualified'],
            end_reason=result['last_control']['large_recovery']['state']['reason'],
            raw_path=str(raw),raw_bytes=v['regular_file_bytes'],
            summary_sha256=sha(OUT/(name+'_collection_summary.json')),control_sha256=sha(raw/'control.jsonl'),
            reference_sha256=sha(raw/'reference.json'),clock_audit_sha256=sha(OUT/(name+'_clock.json')),
            sites=ref['large_recovery']['config']['sites'])
        if r['accepted']:
            assert r['result_status']=='COMPLETE_LAP' and r['fault'] is None
            assert r['accepted']==r['prepared']['anchors']==r['prepared']['input_valid']
            assert not r['prepared']['input_reason_refinements']
            states=read(OUT/(name+'_anchor_states.json'))
            assert len(states)==r['accepted'] and sum(a['target_band'] for a in states)==r['target_band_anchors']
            assert r['accepted']==sum(e['accepted_camera_anchors'] for e in r['events'])
            dest=OUT/'materialized'/name;prepared=OUT/'prepared'/split/name
            assert prepared.is_dir()
            with np.load(dest/'teachers.npz') as labels:
                assert labels['xy_m'].shape==(len(states),30,2) and labels['xy_mask'].all() and np.isfinite(labels['xy_m']).all()
            row.update(teachers_sha256=sha(dest/'teachers.npz'),materialized_path=str(dest),prepared_path=str(prepared),
                peak_abs_accepted_lateral_m=max(abs(a['lateral_m']) for a in states),
                teacher_anchors_per_event=[dict(site_id=e['site_id'],accepted=e.get('accepted_camera_anchors',0),
                    target_band=e.get('target_band_camera_anchors',0)) for e in r['events']])
            splits[split].append(name)
            t=totals.setdefault(split,dict(anchors=0,target_band_anchors=0,events=0,runs=0))
            t['anchors']+=r['accepted'];t['target_band_anchors']+=r['target_band_anchors'];t['events']+=row['accepted_events'];t['runs']+=1
            b=bands.setdefault(split,dict(under_5cm=0,cm5_to_15=0,cm15_to_25=0,cm25_to_35=0,cm35_to_45=0,cm45_to_55=0,cm55_to_65=0,above_65cm=0))
            for a in states:
                lat=abs(a['lateral_m'])
                key='under_5cm' if lat<.05 else 'cm5_to_15' if lat<.15 else 'cm15_to_25' if lat<.25 else 'cm25_to_35' if lat<.35 else 'cm35_to_45' if lat<.45 else 'cm45_to_55' if lat<.55 else 'cm55_to_65' if lat<=.65 else 'above_65cm'
                b[key]+=1
            accepted_sites={e['site_id'] for e in r['events'] if e.get('accepted_camera_anchors',0)}
            clocks=read(OUT/(name+'_clock.json'))
            for e in clocks['events']:
                if e['site_id'] in accepted_sites:max_gap=max(max_gap,e['control_sim_gap']['max_ms'])
        rows.append(row)
    assert not set(splits['train'])&set(splits['validation'])
    sides={s:sorted({'right' if '-right-' in n else 'left' for n in names}) for s,names in splits.items()}
    index=dict(schema='real_60cm_recovery_collection_v1',source_commit='adfc444818a26bae021d463cca5312b5d37a9f9a',
        index_source_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        runtime_host='graneple@192.168.3.10',native_wsl_raw=str(RAW),events_per_lap=3,event_increment=0,
        target_band_tolerance_m=.05,maximum_accepted_control_sim_gap_ms=max_gap,
        lateral_basis='Matched observed normal driving line; not ground-truth road center',
        labels='Measured future trajectory: 30 xy points in meters over 3 seconds',
        inputs='Causal camera, LiDAR and ego history observed at the actual shifted vehicle pose',
        split_policy='Whole runs; no frame-random split; repeated sites are not held-out-site generalization',
        training_started=False,merged_with_existing_dataset=False,
        receipts=receipts,splits=splits,split_sides=sides,totals=totals,absolute_lateral_bands=bands,runs=rows,
        all_raw_file_hashes_and_structure_verified=True,all_sqlite_quick_checks_passed=True,
        all_runs_closed_and_stopped=True)
    with (OUT/'collection_index.json').open('x') as f:json.dump(index,f,indent=2,allow_nan=False)
    print(json.dumps(dict(attempts=len(rows),complete_laps=sum(r['status']=='COMPLETE_LAP' for r in rows),
        totals=totals,split_sides=sides,raw_bytes=sum(r['raw_bytes'] for r in rows),maximum_accepted_control_sim_gap_ms=max_gap)))

if __name__=='__main__':main()
