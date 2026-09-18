"""Saved-data diagnostic only. Never promotes a training mask or changes raw data."""
from __future__ import annotations
from collections import Counter
from pathlib import Path
import hashlib
import json
import math
import sys
import time
import numpy as np
from rosbags.highlevel import AnyReader

ROOT=Path('/home/thistle/e2e_autonomous/runs/mppi_v45_pc10_20260918')
sys.path.insert(0,str(ROOT/'native_audit_support'))
from scenario_tool.geometry import Pose2D,footprint,polygon_distance
from scenario_tool.occupancy import OccupancyGrid


def sha(path: Path) -> str:
    h=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(4*1024*1024),b''):h.update(block)
    return h.hexdigest()


def dump(path: Path, data: object) -> None:
    with path.open('x') as f:json.dump(data,f,indent=2,allow_nan=False)


def point_distance(points: np.ndarray, polygon: np.ndarray) -> np.ndarray:
    """Finite [N,2] points vs finite convex [M,2] polygon; exact distance, metres."""
    p=np.asarray(points,dtype=float); q=np.asarray(polygon,dtype=float)
    if p.ndim!=2 or p.shape[1]!=2 or q.ndim!=2 or q.shape[1]!=2 or len(q)<3:
        raise ValueError('POINT_POLYGON_SHAPE')
    if not np.isfinite(p).all() or not np.isfinite(q).all():raise ValueError('NONFINITE')
    edges=np.roll(q,-1,axis=0)-q
    lens=np.sum(edges**2,axis=1)
    if (lens<=1e-20).any():raise ValueError('DEGENERATE_EDGE')
    delta=p[:,None,:]-q[None,:,:]
    frac=np.clip(np.sum(delta*edges,axis=2)/lens,0.,1.)
    dist=np.linalg.norm(delta-frac[:,:,None]*edges,axis=2).min(axis=1)
    cross=edges[None,:,0]*delta[:,:,1]-edges[None,:,1]*delta[:,:,0]
    inside=(cross>=-1e-10).all(axis=1)|(cross<=1e-10).all(axis=1)
    return np.where(inside,0.,dist)


def anchor_hits(stamps: np.ndarray, low: int, high: int) -> bool:
    """Any evidence in the closed full history/future window, integer ns."""
    if low>high:raise ValueError('REVERSED_WINDOW')
    i=np.searchsorted(stamps,low,side='left')
    return bool(i<len(stamps) and stamps[i]<=high)


def self_test() -> None:
    square=np.array([[-1.,-1.],[1.,-1.],[1.,1.],[-1.,1.]])
    points=np.array([[0.,0.],[1.,0.],[1.3,0.],[2.,2.]])
    np.testing.assert_allclose(point_distance(points,square),[0.,0.,.3,math.sqrt(2)])
    np.testing.assert_allclose(point_distance(points,square[::-1]),[0.,0.,.3,math.sqrt(2)])
    assert anchor_hits(np.array([10,20]),10,10)
    assert not anchor_hits(np.array([10,20]),11,19)
    assert not anchor_hits(np.array([],dtype=np.int64),0,30)
    for p,q in [(np.array([[np.nan,1.]]),square),(np.zeros((2,3)),square)]:
        try:point_distance(p,q)
        except ValueError:pass
        else:raise AssertionError('invalid geometry accepted')


def main() -> None:
    self_test()
    output=ROOT/'collect10_prefix_clearance_v1';output.mkdir(exist_ok=True)
    geom=ROOT/'pc10_physical_wall_map'
    proof=json.loads((geom/'provenance.json').read_text())
    assert sha(geom/'occupancy_grid_map.pgm')==proof['output_pgm_sha256']
    assert proof['awsim_level_sha256']=='9ab2e1e8865c02885594e0bbdde372302530f047b5a2e89c455be6a18f3e090b'
    hull=np.asarray(proof['body_xy_vertices']);grid=OccupancyGrid(geom/'occupancy_grid_map.yaml')
    assert grid.available
    summaries=[]
    for collected in [ROOT/'collected'/sys.argv[1]]:
        start_wall=time.monotonic();name=collected.name;raw=collected/'raw'/name
        prefix=ROOT/'collect10_pose_prefix_v1'/name; target=output/name;target.mkdir()
        manifest=json.loads((collected/'export_manifest.json').read_text());source_hashes={}
        receipt=json.loads((collected/'transfer_verified.json').read_text())
        assert receipt['all_sha256_match'] and receipt['run_id']==name==manifest['run_id']
        selected={f'raw/{name}/{p}' for p in ('samples.jsonl','d1-result-details.json','awsim-player.log','awsim-scenario.yaml')}
        selected.add(f'provenance/scenarios/{name}.json')
        selected.update(p for p in manifest['files'] if '/rosbag2_autoware/' in p)
        for p in sorted(selected):
            file=collected/p;entry=manifest['files'][p]
            assert not file.is_symlink() and file.resolve().is_relative_to(collected.resolve())
            digest=sha(file)
            assert file.stat().st_size==entry['bytes'] and digest==entry['sha256'],p
            source_hashes[p]=digest
        q=json.loads((prefix/'pose_prefix.json').read_text())
        for file,digest in q['output_sha256'].items():assert sha(prefix/file)==digest
        source_hashes['pose_prefix.json']=sha(prefix/'pose_prefix.json')
        candidates=[json.loads(s) for s in (prefix/'prefix_candidates.jsonl').read_text().splitlines()]
        assert not any(r['forward_avoidance_eligible'] for r in candidates)
        placements=json.loads((collected/f'provenance/scenarios/{name}.json').read_text())['locations']
        penalties=json.loads((raw/'d1-result-details.json').read_text())
        summary=dict(run_id=name,prefix_candidates=len(candidates),official_penalties=penalties['penalty_by_kind'],
            prefix_quality=q['quality']['reason'],strict_eligible=0,training_adoption=False)
        if not candidates:
            dump(target/'input_sha256.json',source_hashes)
            summary['reason']='NO_VALID_POSE_PREFIX';summaries.append(summary);dump(target/'summary.json',summary)
            print(json.dumps(summary),flush=True);continue
        low=min(r['observation_ns'] for r in candidates)-1_050_000_000
        high=max(r['observation_ns'] for r in candidates)+3_050_000_000
        assert high<q['quality']['valid_until_ns']
        observations={}
        for s in (raw/'samples.jsonl').read_text().splitlines():
            row=json.loads(s);ego=row.get('ego')
            if row.get('time',-1)<0 or not ego:continue
            stamp=round(ego['stamp']*1e9)
            if low-150_000_000<=stamp<=high+150_000_000:observations[stamp]=row
        stamps=np.array(sorted(observations),np.int64);rows=[observations[t] for t in stamps]
        assert len(stamps)>1
        nominal_shapes=[]
        for loc in placements:
            x,y,yaw=loc['map_pose']
            nominal_shapes.append(footprint(Pose2D(x,y,yaw),length=.5,width=.5))
        gaps=np.full((len(rows),len(placements)),np.inf)
        centres=np.full_like(gaps,np.inf);wall=np.zeros(len(rows));errors=[]
        cached=ROOT/(name+'-physical-samples.jsonl')
        previous={round(r['sim_s']*1e9):r for r in (json.loads(s) for s in cached.read_text().splitlines())} if cached.exists() else {}
        if cached.exists():source_hashes['derived/'+cached.name]=sha(cached)
        dump(target/'input_sha256.json',source_hashes)
        for i,(t,row) in enumerate(zip(stamps,rows)):
            ego=row['ego'];yaw=ego['yaw'];c,s=math.cos(yaw),math.sin(yaw)
            body=hull@np.array([[c,s],[-s,c]])+np.array([ego['x'],ego['y']])
            # Reference-map overlap only: same recorded pose used by teacher, not physical truth.
            wall[i]=previous[t]['wall_overlap_m'] if t in previous else grid.polygon_overlap_depth(body.tolist())
            err=(row.get('ego_gt') or {}).get('estimate_error_m')
            if err is not None:errors.append(err)
            for j,loc in enumerate(placements):
                centre=np.asarray(loc['map_pose'][:2]);centres[i,j]=np.linalg.norm(centre-body.mean(axis=0))
                if centres[i,j]>6.:continue
                if loc['object_type']=='cone':
                    # Convex cone mesh: radius 0.175 m, height 0.7 m, 24 segments.
                    # A circumscribed circle bounds its XY projection. This is not a 3-D gap.
                    gaps[i,j]=max(0.,float(point_distance(centre[None,:],body)[0])-.175)
                else:gaps[i,j]=polygon_distance(body.tolist(),nominal_shapes[j])
        inside=(stamps>=low)&(stamps<=high)
        summary['sample_interval_sim_s']=[low/1e9,high/1e9]
        summary['pose_samples']=int(inside.sum())
        summary['recorded_pose_wall_overlap_samples']=int(((wall>0)&inside).sum())
        summary['max_recorded_pose_wall_overlap_m']=float(wall[inside].max())
        summary['max_monitor_ekf_gnss_xy_difference_m']=max(errors) if errors else None
        summary['objects']=[]
        for j,loc in enumerate(placements):
            relevant=np.flatnonzero(inside&np.isfinite(gaps[:,j]))
            worst=int(relevant[np.argmin(gaps[relevant,j])]) if len(relevant) else None
            summary['objects'].append(dict(object_id=loc['object_id'],object_type=loc['object_type'],
                pose_source='recorded EKF',object_pose_source='scenario initial placement',
                shape_method='circumscribed circle of convex cone mesh XY projection' if loc['object_type']=='cone' else 'initial upright 0.5 m cube XY projection',
                dynamic_pose_verified=False if loc['object_type']=='box' else None,
                minimum_gap_m=float(gaps[worst,j]) if worst is not None else None,
                minimum_sim_s=stamps[worst]/1e9 if worst is not None else None,
                minimum_recorded_pose=rows[worst]['ego'] if worst is not None else None,
                initial_object_pose=loc['map_pose'],
                samples_below_030=int((inside&(gaps[:,j]<.3)).sum()),
                sampled_nearby=len(relevant)))
        print(json.dumps(dict(run=name,phase='geometry',elapsed_s=time.monotonic()-start_wall)),flush=True)
        # Read saved native scan / detected surfaces. These are not object centres or side/rear coverage.
        scan_stamps=[];scan_min=[];scan_geometry=None;track_count=0;nonempty=0;example=None;topic_counts={}
        tf_static={};frames=set()
        topics={'/sensing/lidar/scan','/collection/lidar_v2x/objects','/tf_static'}
        with AnyReader([raw/'d1/rosbag2_autoware']) as reader:
            topic_counts={c.topic:c.msgcount for c in reader.connections}
            for conn,receipt_ns,blob in reader.messages(connections=[c for c in reader.connections if c.topic=='/tf_static']):
                msg=reader.deserialize(blob,conn.msgtype)
                for tr in msg.transforms:
                    xyz=tr.transform.translation;rot=tr.transform.rotation
                    tf_static[tr.child_frame_id]=dict(parent=tr.header.frame_id,xyz=[xyz.x,xyz.y,xyz.z],q=[rot.x,rot.y,rot.z,rot.w])
            # Existing measured mount must be reproduced, not silently assumed for other vehicles.
            assert tf_static['lidar']['parent']=='lidar_base_link'
            assert tf_static['lidar_base_link']['parent']=='base_link'
            np.testing.assert_allclose(tf_static['lidar']['q'],[0,0,0,1],atol=1e-8)
            np.testing.assert_allclose(tf_static['lidar_base_link']['q'],[0,0,0,1],atol=1e-8)
            mount=np.array(tf_static['lidar']['xyz'])+np.array(tf_static['lidar_base_link']['xyz'])
            np.testing.assert_allclose(mount[:2],[1.65,0],atol=1e-7)
            for conn,receipt_ns,blob in reader.messages(connections=[c for c in reader.connections if c.topic in topics-{'/tf_static'}],
                    start=max(0,low-200_000_000),stop=high+300_000_000):
                msg=reader.deserialize(blob,conn.msgtype)
                if conn.topic.endswith('/objects'):
                    data=json.loads(msg.data);t=round(data['stamp_s']*1e9)
                    if not low<=t<=high:continue
                    track_count+=1
                    if data.get('tracks'):
                        nonempty+=1
                        if example is None:example=data
                    continue
                t=int(msg.header.stamp.sec)*1_000_000_000+int(msg.header.stamp.nanosec)
                if not low<=t<=high:continue
                ranges=np.asarray(msg.ranges,dtype=float)
                angles=msg.angle_min+np.arange(len(ranges))*msg.angle_increment
                scan_geometry=dict(frame_id=msg.header.frame_id,beams=len(ranges),angle_min_rad=msg.angle_min,
                    angle_max_rad=msg.angle_max,scan_time_s=msg.scan_time,range_max_m=msg.range_max,
                    base_to_lidar_xyz_m=mount.tolist())
                frames.add(msg.header.frame_id)
                valid=np.isfinite(ranges)&(ranges>msg.range_min)&(ranges<min(3.,msg.range_max))
                points=np.column_stack([ranges[valid]*np.cos(angles[valid]),ranges[valid]*np.sin(angles[valid])])+mount[:2]
                distance=float(point_distance(points,hull).min()) if len(points) else math.inf
                scan_stamps.append(t);scan_min.append(distance)
        scan_stamps=np.asarray(scan_stamps,np.int64);scan_min=np.asarray(scan_min)
        idx=np.argsort(scan_stamps);scan_stamps=scan_stamps[idx];scan_min=scan_min[idx]
        finite=np.isfinite(scan_min)
        summary['lidar']=dict(scans=len(scan_stamps),geometry=scan_geometry,
            nearest_observed_point_gap_m=float(scan_min[finite].min()) if finite.any() else None,
            scans_with_point_below_030=int((scan_min<.3).sum()),
            detected_surface_messages=track_count,nonempty_surface_messages=nonempty,
            physical_object_pose_or_contact_topics=[k for k in topic_counts if any(s in k.lower() for s in ('collision','contact','ground_truth','object_pose'))],
            native_v2x_messages=topic_counts.get('/v2x/vehicle_positions',0))
        if example:dump(target/'detected_surface_example.json',example)
        cone_cols=[i for i,p in enumerate(placements) if p['object_type']=='cone']
        box_cols=[i for i,p in enumerate(placements) if p['object_type']=='box']
        cone_bad=stamps[(gaps[:,cone_cols]<.3).any(axis=1)]
        box_bad=stamps[(gaps[:,box_cols]<.3).any(axis=1)]
        box_near=stamps[(centres[:,box_cols]<6.).any(axis=1)]
        wall_bad=stamps[wall>0];scan_bad=scan_stamps[scan_min<.3]
        counts=Counter()
        with (target/'candidate_checks.jsonl').open('x') as f:
            for row in candidates:
                t=int(row['observation_ns']);lo=t-1_050_000_000;hi=t+3_050_000_000
                reasons=[]
                for key,evidence in [('RECORDED_POSE_PROJECTED_CONE_GAP_BELOW_030',cone_bad),
                    ('NOMINAL_BOX_GAP_BELOW_030',box_bad),('DYNAMIC_BOX_NEARBY_UNVERIFIED',box_near),
                    ('RECORDED_POSE_WALL_OVERLAP',wall_bad),('LIDAR_OBSERVED_POINT_GAP_BELOW_030',scan_bad)]:
                    if anchor_hits(evidence,lo,hi):reasons.append(key)
                si=np.searchsorted(stamps,lo,side='right')-1;ei=np.searchsorted(stamps,hi)
                if si<0 or ei>=len(stamps) or (np.diff(stamps[max(si,0):ei+1])>150_000_000).any():
                    reasons.append('POSE_WINDOW_COVERAGE_UNKNOWN')
                # Recording has no native full-body object pose or all-contact channel. Do not infer admission.
                reasons.append('NATIVE_PHYSICAL_CLEARANCE_NOT_FULLY_OBSERVED')
                counts.update(reasons)
                f.write(json.dumps(dict(run_id=name,epoch=row.get('epoch'),source_label_index=row['source_label_index'],
                    observation_ns=t,window_ns=[lo,hi],findings=reasons,strict_eligible=False))+'\n')
        summary['candidate_findings']=dict(counts)
        summary['output_sha256']={'candidate_checks.jsonl':sha(target/'candidate_checks.jsonl')}
        dump(target/'summary.json',summary);summaries.append(summary)
        print(json.dumps(dict(run=name,candidates=len(candidates),findings=counts,scan=summary['lidar'],
            elapsed_s=time.monotonic()-start_wall)),flush=True)
    result=dict(runs=summaries,candidates=sum(x['prefix_candidates'] for x in summaries),
        strict_eligible=sum(x['strict_eligible'] for x in summaries),training_performed=False,raw_preserved=True,
        operator_sha256=sha(Path(__file__)),map_provenance_sha256=sha(geom/'provenance.json'),
        geometry_support_sha256={p.name:sha(p) for p in (ROOT/'native_audit_support/scenario_tool').glob('*.py')},
        limitations=['Recorded EKF body pose and initial object placement are estimates, not measured physical separation.',
            'Cone has a static convex mesh: radius 0.175 m, height 0.7 m, 24 segments. Its circumscribed XY circle is a conservative planar envelope, not 3-D collision truth.',
            'Box Rigidbody is dynamic after six fixed updates. Initial box poses do not establish their physical pose during a pass.',
            'Monitor EKF/GNSS distance uses independently received samples, not a synchronized absolute position error.',
            'Full 1 s history and 3 s future plus 50 ms endpoint supports are checked.',
            'Front 180 degree LiDAR at x=1.65 m does not observe the full side/rear vehicle perimeter.',
            'Surface tracks have no native physical object identity, collider centre, or hidden dimensions.',
            'Zero official penalties is not a 0.30 m clearance certificate.'],
        findings_total=dict(sum((Counter(s.get('candidate_findings',{})) for s in summaries),Counter())))
    dump(output/(sys.argv[1]+'-summary.json'),result)
    print(json.dumps({k:v for k,v in result.items() if k not in ('runs','geometry_support_sha256')},indent=2),flush=True)


if __name__=='__main__':main()


