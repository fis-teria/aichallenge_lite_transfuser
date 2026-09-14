"""WSL pilot audit: measured states at camera anchors, including the first 150ms.

Uses the production causal selectors and compares accepted IDs to the separate
replay probe. This does not modify raw data, labels, splits, or model weights.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
from typing import Any

import numpy as np

from aic_transfuser_lite.control.vehicle_motion_v1 import MAX_CURVATURE_PER_M
from aic_transfuser_lite.data.time_corpus_v1 import EventWindows, audit_anchor
from aic_transfuser_lite.data.time_dataset_v1 import TimeDatasetConfig, _eligible, _endpoints, _pose_anchor
from aic_transfuser_lite.data.time_recovery_collection_v1 import collection_phase_windows, project_course, recovery_teacher_mask
from aic_transfuser_lite.data.time_sqlite_reader_v1 import read_time_sqlite_run
from aic_transfuser_lite.data.time_steering_pulse_v1 import nominal_recovery_errors


def bins(rows: list[dict[str, Any]]) -> dict[str, Any]:
    located = [r for r in rows if r.get('left_m') is not None]
    significant = [r for r in located if abs(r['left_m']) >= .05 and abs(r['heading_rad']) >= math.radians(1.)]
    outward = [r for r in significant if r['left_m']*r['heading_rad'] > 0]
    goal = [r for r in outward if abs(r['left_m']) <= .25 and math.radians(2.) <= abs(r['heading_rad']) <= math.radians(4.)]
    return dict(count=len(rows), located=len(located), outward_5cm_1deg=len(outward),
        toward_5cm_1deg=len(significant)-len(outward), goal_5to25cm_2to4deg=len(goal),
        lateral_range_m=[min(r['left_m'] for r in located), max(r['left_m'] for r in located)] if located else None,
        heading_range_deg=[math.degrees(min(r['heading_rad'] for r in located)), math.degrees(max(r['heading_rad'] for r in located))] if located else None)


def smoke() -> None:
    left=dict(left_m=.06, heading_rad=math.radians(3.))
    right=dict(left_m=-.07, heading_rad=-math.radians(2.5))
    returning=dict(left_m=.08, heading_rad=-math.radians(3.))
    small=dict(left_m=.01, heading_rad=math.radians(3.))
    result=bins([left,right,returning,small,dict(left_m=None)])
    assert result['count']==5 and result['located']==4
    assert result['outward_5cm_1deg']==2 and result['toward_5cm_1deg']==1
    assert result['goal_5to25cm_2to4deg']==2 and bins([])['lateral_range_m'] is None
    print('PULSE_STATE_BINS_SMOKE_PASS', flush=True)


def main() -> None:
    ap=argparse.ArgumentParser()
    ap.add_argument('--run',type=Path)
    ap.add_argument('--probe',type=Path)
    ap.add_argument('--output',type=Path)
    ap.add_argument('--smoke-only',action='store_true')
    args=ap.parse_args(); smoke()
    if args.smoke_only: return
    if args.run is None or args.probe is None or args.output is None:
        ap.error('--run, --probe, --output are required')
    run=args.run.resolve(); probe=json.loads(args.probe.read_text())
    assert not args.output.exists() and probe['run_id']==run.name
    assert probe['all_phase_candidates_audited']
    assert hashlib.sha256((run/'transfer_manifest.json').read_bytes()).hexdigest()==probe['raw_manifest_sha256']
    result=json.loads((run/'result.json').read_text())
    assert result['nodes']['closed_bag']
    ref=json.loads((run/'reference.json').read_text()); pulse=ref['steering_pulse']
    guide=np.asarray(pulse['nominal_guide'],dtype=float); base=np.asarray(ref['baseline_xy_m'],dtype=float)
    controls=[json.loads(s) for s in (run/'control.jsonl').read_text().splitlines()]
    phases=collection_phase_windows(controls)
    recovery=[w for w in phases if w.phase=='recovery']; assert recovery
    zero=min(w.start_ns for w in recovery)
    view=args.probe.parent/(run.name+'_causal_view_recovery_freeze50000000')
    for db in (run/'bag').glob('*.db3'):
        assert (view/'bag'/db.name).samefile(db)
    index=read_time_sqlite_run(view,run.name); assert len(index.epochs)==1
    epoch=index.epochs[0]; bounds=(epoch.first_sim_stamp_ns,epoch.last_sim_stamp_ns)
    windows=EventWindows(index.events); cfg=TimeDatasetConfig()
    invalid_velocity={e.sequence for e in index.events if e.role=='velocity' and
        (not np.isfinite(e.payload.yaw_rate_rps) or abs(e.payload.yaw_rate_rps)>max(.2,abs(e.payload.longitudinal_mps))*MAX_CURVATURE_PER_M)}
    cameras={}
    for e in sorted(index.events,key=lambda e:(e.available_ns,e.sequence)):
        if e.role=='camera': cameras.setdefault((e.epoch,e.capture_ns),e)
    records=[]
    for anchor in sorted(cameras.values(),key=lambda e:e.capture_ns):
        phase=next((w for w in recovery if w.start_ns<=anchor.capture_ns<w.end_ns),None)
        if phase is None: continue
        events=windows.at(anchor); freeze=anchor.available_ns+50_000_000
        teacher,row=audit_anchor(events,anchor,config=cfg,bounds=bounds,freeze_ns=freeze,intervention_ns=None)
        bad=sorted({rid for slot in row['history_row_ids']['velocity'] for rid in slot}&invalid_velocity)
        if bad: row.update(input_invalid_reason='RAW_HEADING_RATE_INVALID',input_eligible=False,usable_full=False)
        full_phase=bool(recovery_teacher_mask(anchor.capture_ns,phases).all())
        margin=anchor.capture_ns>=phase.start_ns+150_000_000
        full=bool(row['usable_full'] and teacher is not None and teacher.xy_mask.all() and full_phase)
        row.update(seconds_after_release=(anchor.capture_ns-zero)/1e9, phase_margin_pass=margin,
            full_phase_pass=full_phase, usable_before_margin=full, accepted=bool(full and margin),
            invalid_raw_heading_history_row_ids=bad)
        eligible=_eligible(events,anchor,cfg,freeze,bounds)
        try:
            pose_event,provenance=_pose_anchor(eligible,anchor,cfg); pose=pose_event.payload
            assert [r['sequence'] for r in provenance['sources']]==row['observation_pose_row_ids']
            cp=project_course(base,[pose.x_world_m,pose.y_world_m],pose.yaw_world_rad)
            lateral,heading=nominal_recovery_errors(guide,s_m=cp['s_m'],offset_m=cp['offset_m'],yaw_rad=pose.yaw_world_rad)
            row.update(left_m=lateral,heading_rad=heading,base_s_m=cp['s_m'])
            ep=_endpoints(eligible,'velocity',anchor.capture_ns,cfg.tolerance_ns)
            if ep:
                fraction=0. if len(ep)==1 else (anchor.capture_ns-ep[0].capture_ns)/(ep[-1].capture_ns-ep[0].capture_ns)
                row['speed_mps']=float(ep[0].payload.longitudinal_mps+fraction*(ep[-1].payload.longitudinal_mps-ep[0].payload.longitudinal_mps))
        except ValueError as exc:
            row.update(left_m=None,geometry_reason=str(exc))
        records.append(row)
    accepted=[r for r in records if r['accepted']]
    expected=[r['anchor_id'] for r in probe['anchors'] if r['usable_full'] and r.get('phase_and_xy_full')]
    assert [r['anchor_id'] for r in accepted]==expected
    first=[r for r in records if r['seconds_after_release']<.5]
    control_states=[]
    for row in controls:
        p=row.get('pulse',{}); publication=row.get('publication')
        if publication and p.get('applied') and p.get('lateral_error_m') is not None:
            relative=(publication['sim_ns']-zero)/1e9
            if -3.<=relative<=10.:
                control_states.append(dict(seconds_after_release=relative,phase=row['phase'],
                    left_m=p['lateral_error_m'],heading_rad=p['heading_error_rad'],
                    requested_rad=p['requested_rad'],effective_rad=p['effective_rad'],
                    issued_angle_rad=row['issued_angle_rad'],speed_mps=row['speed_mps']))
    report=dict(run_id=run.name,raw_status=result['status'],fault=result.get('last_control',{}).get('fault'),
        nominal_run_id=pulse['nominal_run_id'],nominal_control_sha256=pulse['nominal_control_sha256'],
        config=pulse['config'],first_zero_publication_ns=zero,
        scope='CAMERA_ANCHOR_MEASURED_STATES_RELATIVE_TO_PROGRESS_INTERPOLATED_NOMINAL; NOT_MODEL_EVALUATION',
        all_recovery_cameras=bins(records),accepted=bins(accepted),
        first_05s_all=bins(first),first_05s_accepted=bins([r for r in first if r['accepted']]),
        first_015s_excluded_by_margin=bins([r for r in first if not r['phase_margin_pass']]),
        first_015s_otherwise_usable=bins([r for r in first if not r['phase_margin_pass'] and r['usable_before_margin']]),
        input_reasons=dict(Counter(r['input_invalid_reason'] or 'OK' for r in records)),
        replay_accepted_ids_match=True,independent_runs=1,split_assigned=False,training_materialized=False,
        records=records,control_states=control_states)
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('records','control_states')},indent=2))


if __name__=='__main__':
    main()
