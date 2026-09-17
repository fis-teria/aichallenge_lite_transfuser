"""Audit verified obstacle recordings in native WSL with the shared time teacher.

Output: raw indices and observed future XY [N,30,2] metres, dt=0.1 s. These
artifacts do not change a training split or train a model. Receipt+50 ms is an
explicit offline availability proxy, not a measured inference latency.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw
from rosbags.highlevel import AnyReader

from aic_transfuser_lite.data.clock_segments import ClockSample, segment_clock_epochs
from aic_transfuser_lite.data.time_corpus_v1 import EventWindows, audit_anchor
from aic_transfuser_lite.data.time_dataset_v1 import TimeDatasetConfig, assemble_time_sample
from aic_transfuser_lite.data.time_history_v1 import read_time_events


def ns(stamp) -> int:
    return int(stamp.sec)*1_000_000_000+int(stamp.nanosec)


def windows(values: list[int], max_gap_ns: int = 250_000_000) -> list[tuple[int, int]]:
    """Group sorted capture times; endpoints are included, units nanoseconds."""
    output: list[tuple[int, int]] = []
    for t in sorted(set(values)):
        if output and t-output[-1][1] <= max_gap_ns:
            output[-1] = (output[-1][0], t)
        else:
            output.append((t, t))
    return output


def overlap(start_ns: int, end_ns: int, intervals: list[tuple[int, int]]) -> bool:
    if start_ns > end_ns:
        raise ValueError('Invalid time window')
    return any(start_ns <= end and end_ns >= start for start, end in intervals)


def coalesce_sim_clock_receipts(samples: list[ClockSample]) -> list[ClockSample]:
    """rosbag --use-sim-time can stamp several /clock updates identically.

    Preserve the last monotonic simulation update per receipt. A real clock
    reversal remains an error; only repeated bag receipt stamps are collapsed.
    Raw rows remain in the original bag.
    """
    result: list[ClockSample] = []
    for sample in samples:
        if result:
            previous = result[-1]
            if sample.bag_stamp_ns < previous.bag_stamp_ns or sample.sim_stamp_ns < previous.sim_stamp_ns:
                raise ValueError('Clock reversal requires separate epoch handling')
            if sample.bag_stamp_ns == previous.bag_stamp_ns:
                result[-1] = sample
                continue
        result.append(sample)
    return result


def audit(collected: Path, output: Path) -> dict:
    receipt = json.loads((collected/'transfer_verified.json').read_text())
    assert receipt['all_sha256_match']
    manifest = json.loads((collected/'export_manifest.json').read_text())
    run_id = manifest['run_id']
    assert receipt['run_id'] == run_id
    run = collected/'raw'/run_id
    bag = run/'d1/rosbag2_autoware'
    # Verify compressed bag and its metadata again immediately before decoding.
    verified_bytes = 0
    for name, row in manifest['files'].items():
        if '/rosbag2_autoware/' not in name:
            continue
        path = collected/name
        assert path.stat().st_size == row['bytes']
        h = hashlib.sha256()
        with path.open('rb') as f:
            for block in iter(lambda:f.read(4*1024*1024), b''):
                h.update(block)
        assert h.hexdigest() == row['sha256'], name
        verified_bytes += row['bytes']
    assert verified_bytes > 0
    result = json.loads((run/'result.json').read_text())
    monitor = json.loads((run/'monitor-status.json').read_text())
    official = json.loads((run/'d1-result-details.json').read_text())
    placements = json.loads((collected/'provenance/scenarios'/(run_id+'.json')).read_text())
    clocks = []
    modes = []
    statuses = []
    objects = []
    native = []
    states = []
    reverse_stamps = []
    stop_requests = []
    wanted = {'/clock','/mppi/direct/trajectory_command','/collection/lidar_v2x/status',
              '/collection/lidar_v2x/objects','/v2x/vehicle_positions','/awsim/state',
              '/vehicle/status/gear_status','/control/mpc/stop_request'}
    with AnyReader([bag]) as reader:
        counts = {c.topic:c.msgcount for c in reader.connections}
        begin, end = reader.start_time, reader.end_time-1
        for c, receipt_ns, raw in reader.messages(connections=[c for c in reader.connections if c.topic in wanted]):
            m = reader.deserialize(raw,c.msgtype)
            if c.topic == '/clock':
                clocks.append(ClockSample(int(receipt_ns), ns(m.clock)))
            elif c.topic == '/mppi/direct/trajectory_command':
                modes.append((ns(m.header.stamp),m.mode,bool(m.emergency_stop)))
            elif c.topic == '/collection/lidar_v2x/status':
                statuses.append((receipt_ns,json.loads(m.data)))
            elif c.topic == '/collection/lidar_v2x/objects':
                objects.append((receipt_ns,json.loads(m.data)))
            elif c.topic == '/v2x/vehicle_positions':
                native.append((receipt_ns,{v.vehicle_id:(float(v.position.x),float(v.position.y)) for v in m.vehicles}))
            elif c.topic == '/vehicle/status/gear_status':
                if int(m.report) in {int(m.REVERSE),int(m.REVERSE_2)}:
                    reverse_stamps.append(ns(m.stamp))
            elif c.topic == '/awsim/state':
                states.append((receipt_ns,m.data))
            else:
                stop_requests.append(receipt_ns)
    clock_receipts = coalesce_sim_clock_receipts(clocks)
    epochs = segment_clock_epochs(clock_receipts,max_forward_jump_ns=5_000_000_000)
    assert len(epochs) == 1, 'Clock reset/jump needs a separate epoch audit'
    epoch = replace(epochs[0],first_bag_stamp_ns=begin,last_bag_stamp_ns=end)
    events = read_time_events(bag,run=run_id,epochs=[epoch],capture_clock='sim')
    config = TimeDatasetConfig()
    event_windows = EventWindows(events)
    bounds = (epoch.first_sim_stamp_ns,epoch.last_sim_stamp_ns)
    starts = [t for t,state in states if state=='Start']
    assert starts, 'No AWSIM driving start'
    drive_start = min(starts)
    # Exclude collection termination from the 3 s future, with an additional 2 s guard.
    intervention = bounds[1]-2_000_000_000
    negatives = [e.capture_ns for e in events if e.role=='velocity' and e.payload.longitudinal_mps < -.05]
    reverse = windows(reverse_stamps+negatives)
    avoid = windows([t for t,mode,_ in modes if mode in {'AVOID','OVERTAKE'}],1_000_000_000)
    mode_times = np.array([t for t,_,_ in modes],dtype=np.int64)
    object_times = np.array([t for t,_ in objects],dtype=np.int64)
    penalties = {k:int(v['count']) for k,v in official['penalty_by_kind'].items()}
    clearance = result['metrics'].get('min_clearance_m',{}).get('value')
    route_clear = result['metrics'].get('max_off_track_depth_m',{}).get('value') == 0
    run_clean = (result['execution_status']['ok'] and result['execution_status'].get('monitor_status') == 0
                 and monitor['finished'] and not any(penalties.values()) and route_clear
                 and clearance is not None and clearance >= .30 and not stop_requests)
    cameras = {}
    for e in sorted(events,key=lambda x:(x.available_ns,x.sequence)):
        if e.role=='camera':
            cameras.setdefault(e.capture_ns,e)
    anchors = sorted(cameras.values(),key=lambda x:x.capture_ns)
    labels = np.full((len(anchors),30,2),np.nan,np.float32)
    masks = np.zeros((len(anchors),30),bool)
    velocity = np.full((len(anchors),30),np.nan,np.float32)
    velocity_masks = np.zeros((len(anchors),30),bool)
    selected = np.zeros(len(anchors),bool)
    rows = []
    rejected = Counter()
    sensor_times = defaultdict(list)
    for e in events:
        sensor_times[e.role].append(e.capture_ns)
    smoke = []
    pictures = []
    for i,anchor in enumerate(anchors):
        t = anchor.capture_ns
        local = event_windows.at(anchor)
        teacher,row = audit_anchor(local,anchor,config=config,bounds=bounds,
            freeze_ns=anchor.available_ns+50_000_000,intervention_ns=intervention)
        if teacher is not None:
            labels[i],masks[i] = teacher.xy_m,teacher.xy_mask
            velocity[i],velocity_masks[i] = teacher.velocity_mps,teacher.velocity_mask
        mi = int(np.searchsorted(mode_times,t,side='right'))-1
        oi = int(np.searchsorted(object_times,anchor.available_ns+50_000_000,side='right'))-1
        mode = modes[mi][1] if mi>=0 and t-modes[mi][0] <= 200_000_000 else 'MISSING'
        perception_ok = (oi>=0 and t-round(objects[oi][1]['stamp_s']*1e9) <= 250_000_000
                         and objects[oi][1].get('teacher_ready') is True)
        reasons = []
        if not run_clean: reasons.append('RUN_NOT_CLEAN')
        if t < drive_start+1_000_000_000: reasons.append('STARTUP')
        if not row['usable_full']: reasons.append('INCOMPLETE_INPUT_OR_FUTURE')
        if any(not all(refs) for refs in row['history_row_ids'].values()): reasons.append('HISTORY_GAP')
        if overlap(t-1_000_000_000,t+3_000_000_000,reverse): reasons.append('REVERSE_IN_HISTORY_OR_FUTURE')
        if mi<0 or mode=='MISSING' or modes[mi][2]: reasons.append('TEACHER_UNAVAILABLE_OR_EMERGENCY')
        if not perception_ok: reasons.append('PERCEPTION_UNAVAILABLE')
        near_avoid = overlap(t,t,[(lo-2_000_000_000,hi+3_000_000_000) for lo,hi in avoid])
        if not near_avoid: reasons.append('OUTSIDE_AVOIDANCE_CONTEXT')
        selected[i] = not reasons
        row.update(label_index=i,run_id=run_id,mode=mode,forward_avoidance_eligible=bool(selected[i]),
                   rejection_reasons=reasons,stop_probability=None)
        rows.append(row);rejected.update(reasons)
        if selected[i] and (not smoke or t-smoke[-1]['observation_ns'] > 2_000_000_000) and len(smoke)<6:
            sample = assemble_time_sample(local,anchor,config=config,epoch_start_ns=bounds[0],epoch_end_ns=bounds[1],
                freeze_ns=row['freeze_ns'],intervention_ns=intervention)
            assert sample.inputs is not None and sample.teacher is not None
            np.testing.assert_allclose(sample.teacher.xy_m,labels[i],atol=1e-5)
            assert sample.inputs.image.shape == (1,4,3,224,384)
            smoke.append(dict(observation_ns=t,image_shape=list(sample.inputs.image.shape),
                              lidar_shape=list(sample.inputs.lidar.shape),xy_shape=list(labels[i].shape)))
            image = Image.fromarray(anchor.payload.image_rgb)
            tile = Image.new('RGB',(384,281));tile.paste(image.resize((384,256)),(0,25))
            ImageDraw.Draw(tile).text((4,5),f'{run_id}: {t/1e9:.2f}s {mode}',fill='white')
            pictures.append(tile)
    sensor_report = {}
    for role in ['camera','lidar','pose','velocity','actual_steering','final_command']:
        stamps=np.asarray(sensor_times[role],dtype=np.int64);dt=np.diff(stamps)
        sensor_report[role]=dict(count=len(stamps),backward=int((dt<0).sum()),duplicate=int((dt==0).sum()),
                                 max_gap_s=float(dt.max()/1e9) if len(dt) else None)
    report = dict(run_id=run_id,cases=placements['cases'],split_groups=sorted({x['split_group'] for x in placements['locations']}),
        official_penalties=penalties,finish_reached=monitor['finished'],min_footprint_clearance_m=clearance,
        no_off_track=route_clear,run_clean_for_forward_audit=bool(run_clean),
        monitor_process_returncode=result['execution_status'].get('monitor_status'),
        monitor_final_status=monitor['exit_status'],topic_counts=counts,sensors=sensor_report,
        teacher_mode_counts=dict(Counter(mode for _,mode,_ in modes)),avoid_intervals_ns=avoid,
        reverse_intervals_ns=reverse,perception_status_counts=dict(Counter(s['reason'] for _,s in statuses)),
        stop_requests=len(stop_requests),camera_anchors=len(anchors),full_future_anchors=int(masks.all(axis=1).sum()),
        forward_avoidance_eligible_anchors=int(selected.sum()),rejections=dict(rejected),tensor_smoke=smoke,
        label_contract=dict(shape=list(labels.shape),units='m',dt_s=.1,frame='base_link_at_observation',
                            source='OBSERVED_FUTURE_POSE',freeze_delay_receipt_ns=50_000_000,config=asdict(config)),
        source_bag=str(bag),source_verified_bytes=verified_bytes,training_split_assigned=False,
        duplicate_sim_clock_receipts_coalesced=len(clocks)-len(clock_receipts),
        collection_scope='MPPI teacher drive; shadow execution is not inferred by this audit',
        limitations=['V2X-derived footprint clearance is approximate, not a contact mesh oracle.',
                     'Forward candidate mask excludes reverse and unknown stop intent; raw reverse/stops are preserved.',
                     'No optimizer run or automatic merge into an existing training split.'])
    output.mkdir(parents=True,exist_ok=False)
    (output/'audit.json').write_text(json.dumps(report,indent=2)+'\n')
    (output/'anchors.jsonl').write_text(''.join(json.dumps(r,separators=(',',':'))+'\n' for r in rows))
    np.savez_compressed(output/'observed_teachers.npz',observation_ns=np.array([a.capture_ns for a in anchors],np.int64),
        xy_m=labels,xy_mask=masks,velocity_mps=velocity,velocity_mask=velocity_masks,
        forward_avoidance_eligible=selected)
    if pictures:
        sheet=Image.new('RGB',(1152,281*((len(pictures)+2)//3)))
        for i,pic in enumerate(pictures):sheet.paste(pic,((i%3)*384,(i//3)*281))
        sheet.save(output/'camera_samples.jpg',quality=92)
    return report


def main() -> None:
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--collected',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();result=audit(a.collected,a.output)
    print(json.dumps({k:result[k] for k in ['run_id','official_penalties','finish_reached','min_footprint_clearance_m',
        'camera_anchors','forward_avoidance_eligible_anchors','teacher_mode_counts','stop_requests']}),flush=True)


if __name__=='__main__':main()
