"""Diagnose held scan/map windows without changing poses, labels or gates."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from rosbags.highlevel import AnyReader
from curate_native_teacher_data import load_wall_map, stamp_ns, yaw_of, read_rows
from filter_teacher_pose_prefix import verified_bag, sha
from aic_transfuser_lite.runtime.lidar_map_localization import transform


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    tree, _, map_hashes = load_wall_map(args.root)
    manifest_path = args.root/'collect10_curated_v1/selection_manifest.json'
    manifest = json.loads(manifest_path.read_text())
    reports = []
    for entry in manifest['runs']:
        run = entry['run_id']
        bag, hashes = verified_bag(args.root/'collected'/run, run)
        decisions = read_rows(args.root/'collect10_curated_v1'/run/'decisions.jsonl')
        poses = {}; scans = {}; static = {}
        with AnyReader([bag]) as reader:
            topics = {'/sensing/lidar/scan', '/localization/kinematic_state', '/tf_static'}
            for c, _, raw in reader.messages(connections=[c for c in reader.connections if c.topic in topics]):
                msg = reader.deserialize(raw, c.msgtype)
                if c.topic == '/tf_static':
                    for tr in msg.transforms:
                        t, q = tr.transform.translation, tr.transform.rotation
                        static[tr.child_frame_id] = (tr.header.frame_id, [t.x,t.y,t.z], [q.x,q.y,q.z,q.w])
                elif c.topic.endswith('kinematic_state'):
                    p = msg.pose.pose
                    poses[stamp_ns(msg.header.stamp)] = [p.position.x,p.position.y,yaw_of(p.orientation)]
                else:
                    assert msg.header.frame_id == 'lidar'
                    scans[stamp_ns(msg.header.stamp)] = msg
        for child, parent in [('lidar','lidar_base_link'),('lidar_base_link','base_link')]:
            assert static[child][0] == parent
            np.testing.assert_allclose(static[child][2],[0,0,0,1],atol=1e-8)
        mount = np.array(static['lidar'][1]) + np.array(static['lidar_base_link'][1])
        np.testing.assert_allclose(mount[:2],[1.65,0],atol=1e-7)
        ts = np.array(sorted(poses),dtype=np.int64)
        ps = np.array([poses[t] for t in ts]); ps[:,2] = np.unwrap(ps[:,2])
        details = []
        # 0.5 s diagnostic sampling; no candidate is admitted using this fit.
        last = -10**18
        for t, msg in sorted(scans.items()):
            if t-last < 500_000_000 or not ts[0] <= t <= ts[-1]:continue
            last = t
            ranges = np.asarray(msg.ranges,float)
            idx = np.flatnonzero(np.isfinite(ranges)&(ranges>max(1.,msg.range_min))&(ranges<min(12.,msg.range_max)))[::3]
            if len(idx) < 80:continue
            angles = msg.angle_min+idx*msg.angle_increment
            body = np.column_stack([ranges[idx]*np.cos(angles),ranges[idx]*np.sin(angles)])+mount[:2]
            pose = np.array([np.interp(t,ts,ps[:,j]) for j in range(3)])
            def distances(delta: np.ndarray) -> np.ndarray:
                return tree.query(transform(body,pose+delta))[0]
            original = distances(np.zeros(3))
            fit = least_squares(distances,np.zeros(3),bounds=([-1.,-1.,-.15],[1.,1.,.15]),
                                diff_step=1e-3,loss='soft_l1',f_scale=.1,max_nfev=80)
            corrected = distances(fit.x)
            placement = json.loads((args.root/'collected'/run/'provenance/scenarios'/f'{run}.json').read_text())['locations'][0]
            delta = np.array(placement['map_pose'][:2])-pose[:2]
            c,s = np.cos(pose[2]),np.sin(pose[2])
            ob = delta@np.array([[c,-s],[s,c]])
            details.append(dict(stamp_ns=t,original_median_m=float(np.median(original)),
                original_inlier_fraction=float(np.mean(original<=.25)),
                fitted_median_m=float(np.median(corrected)),fitted_inlier_fraction=float(np.mean(corrected<=.25)),
                fitted_delta_map_x_y_yaw=fit.x.tolist(),object_body_xy_m=ob.tolist(),
                points=len(body),fit_success=bool(fit.success)))
        bad = [r for r in details if r['original_median_m']>.15 or r['original_inlier_fraction']<.7]
        improved = [r for r in bad if r['fitted_median_m']<=.15 and r['fitted_inlier_fraction']>=.7]
        report = dict(run_id=run,scans_sampled=len(details),bad_scans=len(bad),
            bad_scans_matching_after_diagnostic_rigid_fit=len(improved),
            held_windows=sum('SCAN_MAP_ALIGNMENT_OR_SUPPORT' in r['reasons'] for r in decisions),
            source_bag_sha256=hashes,details=details)
        (args.output/(run+'.json')).write_text(json.dumps(report,indent=2))
        reports.append({k:v for k,v in report.items() if k not in ('details','source_bag_sha256')})
        print(json.dumps(reports[-1]),flush=True)
    result=dict(runs=reports,map_sha256=map_hashes,selection_sha256=sha(manifest_path),
        labels_or_selection_changed=False,limitations=[
            'Nearest-wall fit is diagnostic and may be geometrically ambiguous; not localization truth.',
            'Rigid-fit improvement alone cannot separate pose drift, time offset and map/extrinsic bias.',
            'No held windows are admitted by diagnostic alignment.'])
    (args.output/'summary.json').write_text(json.dumps(result,indent=2))


if __name__ == '__main__':main()
