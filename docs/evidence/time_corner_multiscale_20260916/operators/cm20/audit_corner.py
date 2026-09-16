"""Native WSL audit of real large-recovery recordings, with causal pose bands."""
from pathlib import Path
import argparse
from collections import Counter
import hashlib
import json
import subprocess
import sys

import numpy as np

from aic_transfuser_lite.data.time_large_recovery_v1 import large_recovery_events, large_event_at
from aic_transfuser_lite.data.time_recovery_training_v1 import materialize_recovery_run
from aic_transfuser_lite.data.time_training_cache_v1 import _prepare_run
from aic_transfuser_lite.data.time_dataset_v1 import TimeDatasetConfig, _eligible, _pose_anchor, _endpoints
from aic_transfuser_lite.data.time_corpus_v1 import EventWindows
from aic_transfuser_lite.data.time_sqlite_reader_v1 import read_time_sqlite_run
from aic_transfuser_lite.data.time_recovery_collection_v1 import project_course
from aic_transfuser_lite.data.time_steering_pulse_v1 import nominal_recovery_errors
from aic_transfuser_lite.runtime.recovery_disturbance_markers import DisturbanceLocations

OUT=Path('/home/thistle/e2e_autonomous/runs/time_corner_multiscale20_20260916')
RAW=Path('/home/thistle/e2e_autonomous/raw/time_corner_multiscale20_20260916')
TYPES=Path('/home/thistle/e2e_autonomous/runs/time_recovery_collection_20260913/types')
read=lambda p:json.loads(p.read_bytes())


def write(p,v):
    with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False)


def stage(name,kind,cmd):
    p=OUT/(name+'_'+kind+'.json')
    assert not p.exists()
    with p.with_suffix('.log').open('x') as stream:
        result=subprocess.run([sys.executable,*cmd,'--output',str(p)],stdout=stream,stderr=subprocess.STDOUT,timeout=900)
    print(json.dumps(dict(run=name,stage=kind,exit=result.returncode)),flush=True)
    assert result.returncode==0,p.with_suffix('.log').read_text()[-3000:]
    return p


def audit(name,split):
    raw=RAW/name;result=read(raw/'result.json');ref=read(raw/'reference.json')
    manifest=read(raw/'transfer_manifest.json')
    assert hashlib.sha256((raw/'control.jsonl').read_bytes()).hexdigest()==manifest['control.jsonl']['sha256']
    controls=[json.loads(s) for s in (raw/'control.jsonl').read_text().splitlines()]
    events=large_recovery_events(controls)
    bag_path=stage(name,'bag_audit',['tools/audit_time_recovery_collection.py','--run',str(raw),'--types',str(TYPES)])
    bag=read(bag_path)
    trace=DisturbanceLocations()
    for row in controls:trace.add(row)
    if trace.events:assert read(raw/'disturbance_markers.json')==trace.report()
    report=dict(run_id=name,split=split,result_status=result['status'],fault=result['last_control'].get('fault'),
        stop_confirmed=result['last_control'].get('stop_confirmed'),closed_bag=result['nodes']['closed_bag'],
        planned=ref['large_recovery']['config']['sites'],event_cap=ref['large_recovery']['config']['event_cap'],
        events=events,accepted=0,target_band_anchors=0,marker_locations=trace.report(),training_started=False)
    sensor_ok=all(v['invalid_messages']==v['backward_headers']==0 for v in bag['sensors'].values())
    normal=(result['status']=='COMPLETE_LAP' and report['fault'] is None and report['stop_confirmed'] and sensor_ok)
    if any(e['recovery_confirmed'] and e['completed'] for e in events):
        probe_path=stage(name,'causal_probe',['docs/evidence/time_recovery_collection_20260913/causal_replay_probe.py','--run',str(raw),'--types',str(TYPES),'--all-candidates'])
        probe=read(probe_path)
        view=OUT/(name+'_causal_view_recovery_freeze50000000')
        index=read_time_sqlite_run(view,name);assert len(index.epochs)==1
        windows=EventWindows(index.events);cfg=TimeDatasetConfig()
        bounds=(index.epochs[0].first_sim_stamp_ns,index.epochs[0].last_sim_stamp_ns)
        by_seq={e.sequence:e for e in index.events};guide=np.asarray(ref['large_recovery']['nominal_guide'])
        base=np.asarray(ref['baseline_xy_m']);located=[]
        for row in probe['anchors']:
            if not row['usable_full'] or not row.get('phase_and_xy_full'):continue
            anchor=by_seq[row['camera_row_id']];ev=large_event_at(anchor.capture_ns,events);assert ev is not None
            eligible=_eligible(windows.at(anchor),anchor,cfg,row['freeze_ns'],bounds)
            pose_event,provenance=_pose_anchor(eligible,anchor,cfg);pose=pose_event.payload
            assert [r['sequence'] for r in provenance['sources']]==row['observation_pose_row_ids']
            cp=project_course(base,[pose.x_world_m,pose.y_world_m],pose.yaw_world_rad)
            lat,heading=nominal_recovery_errors(guide,s_m=cp['s_m'],offset_m=cp['offset_m'],yaw_rad=pose.yaw_world_rad)
            ep=_endpoints(eligible,'velocity',anchor.capture_ns,cfg.tolerance_ns)
            assert ep
            weight=0. if len(ep)==1 else (anchor.capture_ns-ep[0].capture_ns)/(ep[1].capture_ns-ep[0].capture_ns)
            speed=ep[0].payload.longitudinal_mps*(1.-weight)+ep[-1].payload.longitudinal_mps*weight
            located.append(dict(anchor_id=row['anchor_id'],event_id=ev['event_id'],site_id=ev['site_id'],
                observation_ns=anchor.capture_ns,base_s_m=cp['s_m'],lateral_m=lat,heading_rad=heading,
                speed_mps=speed,velocity_row_ids=[e.sequence for e in ep],corner_id=ev['corner_id'],
                requested_heading_rad=ev['target_heading_rad'],
                requested_offset_m=ev['target_offset_m'],target_band=abs(lat-ev['target_offset_m'])<=.05))
        write(OUT/(name+'_anchor_states.json'),located)
        for ev in events:
            a=[r for r in located if r['event_id']==ev['event_id']]
            ev.update(causal_camera_candidates=len(a),accepted_camera_anchors=len(a) if normal else 0,target_band_camera_anchors=sum(r['target_band'] for r in a) if normal else 0,
                peak_abs_anchor_lateral_m=max([abs(r['lateral_m']) for r in a],default=None))
        report['causal_candidates']=len(located)
        report['accepted']=len(located) if normal else 0
        report['target_band_anchors']=sum(r['target_band'] for r in located) if normal else 0
        report['input_reasons']=probe['input_reasons'];report['teacher_reasons']=probe['teacher_reasons']
        if normal and located:
            dest=OUT/'materialized'/name
            material=materialize_recovery_run(raw,dest,split=split,types=TYPES,previous_probe=probe_path)
            prepared=_prepare_run(dest,OUT/'prepared'/split/name,name,cfg)
            assert prepared['anchors']==prepared['input_valid']==material['accepted']==len(located)
            assert not prepared['input_reason_refinements']
            labels=np.load(dest/'teachers.npz')
            assert labels['xy_m'].shape==(len(located),30,2) and labels['xy_mask'].all() and np.isfinite(labels['xy_m']).all()
            report['prepared']=prepared
    report['all_planned_recovered']=(normal and len(events)==len(report['planned']) and all(e['completed'] and e['stable_at_end'] and e['target_band_observed'] for e in events))
    report['all_planned_lateral_events_qualified']=(report['all_planned_recovered'] and all(e.get('accepted_camera_anchors',0)>=60 and e.get('target_band_camera_anchors',0)>0 for e in events))
    write(OUT/(name+'_collection_summary.json'),report)
    print(json.dumps({k:v for k,v in report.items() if k not in ('prepared','marker_locations','input_reasons','teacher_reasons')}),flush=True)


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--run',required=True);ap.add_argument('--split',choices=['train','validation'],required=True);a=ap.parse_args()
    audit(a.run,a.split)
