"""Evaluate frozen estimates against recorded localization, NOT ground truth.

Evaluation-only reference topic; never reruns registration or fits an alignment.
XY metres and yaw radians are compared at identical interpolated sim timestamps.
"""
from pathlib import Path
import argparse
import hashlib
import json
import math

import numpy as np


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def interpolate(times: np.ndarray, poses: np.ndarray, stamp: int) -> np.ndarray | None:
    right=int(np.searchsorted(times,stamp))
    if right<len(times) and int(times[right])==stamp:
        return poses[right].copy()
    if right==0 or right==len(times) or times[right]-times[right-1]>100_000_000:
        return None
    fraction=(stamp-int(times[right-1]))/int(times[right]-times[right-1])
    out=poses[right-1]+fraction*(poses[right]-poses[right-1])
    out[2]=wrap(poses[right-1,2]+fraction*wrap(poses[right,2]-poses[right-1,2]))
    return out


def assess(rows: list[dict], times: np.ndarray, poses: np.ndarray) -> tuple[dict,list[dict]]:
    detail=[];valid_count=sum(bool(r['valid']) for r in rows)
    for row in rows:
        truth=interpolate(times,poses,row['stamp_ns'])
        if truth is None or not row['valid']:
            detail.append(dict(stamp_ns=row['stamp_ns'],valid=bool(row['valid']),reference_available=truth is not None))
            continue
        estimate=np.asarray(row['match']['pose']);delta=estimate[:2]-truth[:2]
        detail.append(dict(stamp_ns=row['stamp_ns'],valid=True,reference_available=True,
                           xy_error_m=float(np.linalg.norm(delta)),yaw_error_deg=abs(math.degrees(wrap(estimate[2]-truth[2]))),
                           estimate=estimate.tolist(),reference=truth.tolist()))
    paired=[d for d in detail if 'xy_error_m' in d]
    def stats(key):
        a=np.array([d[key] for d in paired])
        return dict(mean=float(np.mean(a)),median=float(np.median(a)),p95=float(np.quantile(a,.95)),max=float(np.max(a))) if len(a) else None
    thresholds=[]
    for distance,angle in [(.25,5),(.5,5),(1.,10)]:
        count=sum(d['xy_error_m']<=distance and d['yaw_error_deg']<=angle for d in paired)
        thresholds.append(dict(position_limit_m=distance,yaw_limit_deg=angle,count=count,
                               percent_of_paired_valid=100*count/len(paired) if paired else None,
                               percent_of_all_attempts=100*count/len(rows)))
    return dict(attempts=len(rows),valid_output=valid_count,paired_valid=len(paired),
                missing_reference=sum(not d['reference_available'] for d in detail),
                xy_error_m=stats('xy_error_m'),yaw_error_deg=stats('yaw_error_deg'),thresholds=thresholds),detail


def self_test() -> None:
    t=np.array([0,100_000_000]);p=np.array([[0,0,math.radians(179)],[1,0,math.radians(-179)]])
    mid=interpolate(t,p,50_000_000)
    np.testing.assert_allclose(mid[:2],[.5,0]);assert abs(abs(mid[2])-math.pi)<1e-12
    assert interpolate(t,p,-1) is None and interpolate(t,p,100_000_001) is None
    assert interpolate(np.array([0,200_000_000]),p,100_000_000) is None
    rows=[dict(stamp_ns=0,valid=True,match=dict(pose=p[0].tolist())),dict(stamp_ns=50_000_000,valid=False)]
    summary,_=assess(rows,t,p)
    assert summary['thresholds'][0]['percent_of_paired_valid']==100
    assert summary['thresholds'][0]['percent_of_all_attempts']==50
    print('SELF_TEST_PASS: angular wrap, exact timestamps, no extrapolation, gap limit, invalid-output denominator')


def main() -> None:
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();self_test()
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores,get_typestore
    root=Path('/home/thistle/e2e_autonomous/runs')
    run=root/'time_teacher_si26_20260911/laps03/5kmh_run01'
    by_stamp={};frames=set();digest=hashlib.sha256();ambiguous=set();duplicates=0
    duplicate_max_xy=0.;duplicate_max_yaw=0.
    with AnyReader([run/'bag'],default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as reader:
        topics={c.topic:c.msgcount for c in reader.connections}
        for conn,receipt,raw in reader.messages(connections=[c for c in reader.connections if c.topic=='/localization/pose']):
            msg=reader.deserialize(raw,conn.msgtype);frames.add(msg.header.frame_id)
            assert msg.header.frame_id=='map'
            p,q=msg.pose.position,msg.pose.orientation
            quaternion=np.array([q.x,q.y,q.z,q.w]);assert abs(np.linalg.norm(quaternion)-1)<.01
            stamp=int(msg.header.stamp.sec)*10**9+int(msg.header.stamp.nanosec)
            value=[p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
            assert np.isfinite(value).all()
            if stamp in by_stamp:
                duplicates+=1
                xy=float(np.linalg.norm(np.asarray(by_stamp[stamp][:2])-value[:2]))
                yaw=abs(wrap(by_stamp[stamp][2]-value[2]))
                duplicate_max_xy=max(duplicate_max_xy,xy);duplicate_max_yaw=max(duplicate_max_yaw,yaw)
                # Preserve the first value for numerically equivalent updates.
                # Exclude conflicting stamps rather than silently choose one.
                if xy>.001 or yaw>1e-5:ambiguous.add(stamp)
            else:
                by_stamp[stamp]=value
            digest.update(raw)
    for stamp in ambiguous:del by_stamp[stamp]
    times=np.array(sorted(by_stamp),dtype=np.int64);poses=np.array([by_stamp[int(t)] for t in times])
    reports={};details={};hashes={}
    paths={'bounded':root/'lidar_map_runtime_20260918/replay05',
           'unlimited':root/'lidar_map_aggressive_20260918/unlimited01',
           'simulation_aggressive':root/'lidar_map_aggressive_20260918/aggressive01'}
    for name,path in paths.items():
        blob=(path/'records.jsonl').read_bytes();hashes[name]=hashlib.sha256(blob).hexdigest()
        rows=[json.loads(s) for s in blob.decode().splitlines()]
        reports[name],details[name]=assess(rows,times,poses)
        assert hashlib.sha256((path/'records.jsonl').read_bytes()).hexdigest()==hashes[name]
    output=dict(reference_topic='/localization/pose',reference_kind='recorded localization estimate; NOT simulator ground truth',
                independent_ground_truth_topic_present='/awsim/ground_truth/vehicle/pose' in topics,
                reference_messages=len(times),reference_frames=sorted(frames),reference_messages_sha256=digest.hexdigest(),
                input_estimate_hashes=hashes,alignment_fitted=False,used_for_localization_input=False,
                duplicate_reference_messages=duplicates,ambiguous_reference_stamps_removed=len(ambiguous),
                max_duplicate_position_difference_m=duplicate_max_xy,max_duplicate_yaw_difference_rad=duplicate_max_yaw,
                scope='Reference agreement only; missing estimates count against all-attempt denominator',
                reports=reports,bag_topics=topics)
    args.output.mkdir(exist_ok=False,parents=True)
    (args.output/'summary.json').write_text(json.dumps(output,indent=2)+'\n')
    (args.output/'details.json').write_text(json.dumps(details,indent=2)+'\n')
    print(json.dumps(output,indent=2))


if __name__=='__main__':main()
