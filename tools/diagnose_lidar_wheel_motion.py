"""Offline scan-to-scan / wheel-motion comparison, without map/GNSS/IMU pose input.

Reads exact scan timestamps from a prior causal localization replay. Motion is
estimated independently from scans with identity initialization; wheel seeding,
reverse registration and shape observability are diagnostics, not ground truth.
All planar poses are [x m, y m, yaw rad]. No controller or runtime changes.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from aic_transfuser_lite.runtime.lidar_map_localization import compose, inverse, pose_array, transform, wrap

MOUNT = np.array([1.65, 0., 0.])


@dataclass
class Registration:
    pose: list[float]
    supported: bool
    count: int
    fraction: float
    median_m: float
    p90_m: float
    eigenvalues: list[float]
    weak_ratio: float
    iterations: int


def cloud_from_scan(scan: object) -> np.ndarray:
    """Native scan -> [N,2] metres; invalid beams stay NaN to preserve adjacency."""
    ranges = np.asarray(scan.ranges, dtype=float)
    if ranges.ndim != 1 or not 3 <= len(ranges) <= 10000:
        raise ValueError('SCAN_SHAPE')
    if not np.isfinite([scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max]).all() or scan.angle_increment <= 0:
        raise ValueError('SCAN_GEOMETRY')
    angle = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
    good = np.isfinite(ranges) & (ranges > max(.3, scan.range_min)) & (ranges < min(15., scan.range_max))
    result = np.full((len(ranges), 2), np.nan)
    result[good] = ranges[good, None] * np.column_stack((np.cos(angle[good]), np.sin(angle[good])))
    return result


class ScanSurface:
    def __init__(self, points: np.ndarray) -> None:
        """Fit local surfaces over up to nine adjacent beams, excluding jumps.

        Two-beam normals amplify centimetre range noise into spurious corners.
        PCA over a <=0.45 m neighbourhood estimates the wall normal instead.
        Reject discontinuities / curved windows with minor/major variance >.08.
        """
        pts = np.asarray(points, dtype=float)
        if pts.ndim != 2 or pts.shape[1] != 2 or not 3 <= len(pts) <= 10000:
            raise ValueError('CLOUD_SHAPE')
        starts, vectors = [], []
        for i, point in enumerate(pts):
            if not np.isfinite(point).all():
                continue
            neighbours=pts[max(0,i-4):min(len(pts),i+5)]
            good=np.isfinite(neighbours).all(axis=1) & (np.linalg.norm(neighbours-point,axis=1)<.45)
            neighbours=neighbours[good]
            if len(neighbours)<5:
                continue
            centre=neighbours.mean(axis=0); centred=neighbours-centre
            values,basis=np.linalg.eigh(centred.T@centred)
            if values[-1]<1e-6 or values[0]/values[-1]>.08:
                continue
            direction=basis[:,-1]; projection=centred@direction
            lo,hi=float(projection.min()),float(projection.max())
            if hi-lo<.05:
                continue
            starts.append(centre+lo*direction);vectors.append((hi-lo)*direction)
        self.a=np.asarray(starts); self.v=np.asarray(vectors)
        if len(self.a) < 30:
            raise ValueError('INSUFFICIENT_SURFACE')
        self.tree = cKDTree(self.a + self.v*.5)

    def nearest(self, xy: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        _, candidates = self.tree.query(xy, k=min(8, len(self.a)))
        a, v = self.a[candidates], self.v[candidates]
        fraction = np.clip(np.sum((xy[:, None]-a)*v, axis=2)/np.sum(v*v, axis=2), 0., 1.)
        q = a + fraction[:, :, None]*v
        distance = np.linalg.norm(xy[:, None]-q, axis=2)
        best = np.argmin(distance, axis=1); rows = np.arange(len(xy))
        direction = v[rows, best]
        normal = np.column_stack((-direction[:, 1], direction[:, 0]))
        normal /= np.linalg.norm(normal, axis=1)[:, None]
        return distance[rows, best], q[rows, best], normal


def register(reference: np.ndarray, current: np.ndarray, seed: np.ndarray | None = None) -> Registration:
    """Current scan -> previous scan transform; identity seed is LiDAR-only.

    [N,2] metres, invalid beams may be NaN. Symmetric point-to-line ICP with a 0.4 m
    correspondence bound and robust residuals. No wheel penalty or map input.
    Eigenvalues use a 5 m yaw lever, so weak_ratio is a heuristic, not a
    calibrated uncertainty. A straight corridor can have tiny residuals while
    its longitudinal translation is unobservable.
    """
    surface = ScanSurface(reference)
    reverse_surface = ScanSurface(current)
    reference_points = np.asarray(reference,dtype=float)
    reference_points = reference_points[np.isfinite(reference_points).all(axis=1)]
    pts = np.asarray(current, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 2 or not 3 <= len(pts) <= 10000:
        raise ValueError('CLOUD_SHAPE')
    pts = pts[np.isfinite(pts).all(axis=1)]
    if len(pts) < 40:
        raise ValueError('INSUFFICIENT_POINTS')
    estimate = np.zeros(3) if seed is None else pose_array(seed)
    eigenvalues = np.zeros(3)
    for iteration in range(40):
        xy = transform(pts, estimate)
        distance, closest, normal = surface.nearest(xy)
        good = distance < .4
        if good.sum() < 40:
            break
        residual = np.sum((xy[good]-closest[good])*normal[good], axis=1)
        angle = estimate[2]; c, s = np.cos(angle), np.sin(angle)
        derivative = pts[good] @ np.array([[-s,c],[-c,-s]])
        jac = np.column_stack((normal[good], np.sum(derivative*normal[good],axis=1)/5.))
        # Include the inverse residual in the SAME optimization. This avoids
        # selecting one noisy scan as the exact surface and systematically
        # biasing the estimate differently in each registration direction.
        reverse_xy=transform(reference_points,inverse(estimate))
        reverse_distance, reverse_closest, reverse_normal=reverse_surface.nearest(reverse_xy)
        reverse_good=reverse_distance<.4
        n=reverse_normal[reverse_good]; z=reverse_xy[reverse_good]
        reverse_residual=np.sum((z-reverse_closest[reverse_good])*n,axis=1)
        reverse_jac=np.column_stack((-c*n[:,0]+s*n[:,1],-s*n[:,0]-c*n[:,1],
                                     (n[:,0]*z[:,1]-n[:,1]*z[:,0])/5.))
        jac=np.r_[jac,reverse_jac];residual=np.r_[residual,reverse_residual]
        weight = np.minimum(1., .04/np.maximum(np.abs(residual), 1e-12))
        hessian = (jac.T*weight) @ jac
        eigenvalues, basis = np.linalg.eigh(hessian)
        observable = eigenvalues > max(1e-6, eigenvalues[-1]*1e-6)
        vectors = basis[:,observable]
        step = -vectors @ ((vectors.T @ (jac.T @ (weight*residual))) / eigenvalues[observable])
        step[2] /= 5.
        step /= max(1., np.linalg.norm(step[:2])/.2, abs(step[2])/.05)
        estimate += step
        if np.linalg.norm(step[:2]) < 1e-5 and abs(step[2]) < 1e-6:
            break
    estimate[2] = wrap(estimate[2])
    distance, closest, normal = surface.nearest(transform(pts,estimate))
    good = distance < .15
    count = int(good.sum()); fraction = count/len(pts)
    values = distance[good]
    return Registration(estimate.tolist(), count >= 100 and fraction >= .7, count, fraction,
                        float(np.median(values)) if count else float('inf'),
                        float(np.quantile(values,.9)) if count else float('inf'),
                        eigenvalues.tolist(), float(max(0.,eigenvalues[0])/max(1e-12,eigenvalues[-1])), iteration+1)


def base_motion(lidar_motion: np.ndarray) -> np.ndarray:
    return compose(compose(MOUNT,lidar_motion),inverse(MOUNT))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--records',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--start',type=float,default=35.)
    parser.add_argument('--end',type=float,default=50.)
    args=parser.parse_args()
    if not 0 <= args.start < args.end:
        raise ValueError('TIME_WINDOW')
    from rosbags.highlevel import AnyReader
    from aic_transfuser_lite.data.time_sqlite_reader_v1 import _store
    rows=[json.loads(s) for s in args.records.read_text().splitlines()]
    rows=[r for r in rows if args.start <= r['stamp_ns']/1e9 <= args.end]
    if len(rows) < 3:
        raise ValueError('INSUFFICIENT_RECORDS')
    targets={r['stamp_ns'] for r in rows}; scans={}; digest=hashlib.sha256()
    with AnyReader([args.run/'bag'],default_typestore=_store(args.run)) as reader:
        for conn,receipt,raw in reader.messages(connections=[c for c in reader.connections if c.topic == '/sensing/lidar/scan']):
            msg=reader.deserialize(raw,conn.msgtype)
            stamp=int(msg.header.stamp.sec)*10**9+int(msg.header.stamp.nanosec)
            if stamp in targets:
                if msg.header.frame_id != 'lidar':
                    raise ValueError('SCAN_FRAME')
                scans[stamp]=cloud_from_scan(msg)
                digest.update(stamp.to_bytes(8,'little'));digest.update(raw)
            if len(scans)==len(targets):
                break
    if set(scans) != targets:
        raise ValueError(f'MISSING_SCANS:{len(targets-set(scans))}')
    args.output.mkdir(parents=True,exist_ok=False)
    result=[]
    for prev,cur in zip(rows,rows[1:]):
        t0,t1=prev['stamp_ns'],cur['stamp_ns']; dt=(t1-t0)/1e9
        if not .15 <= dt <= .3:
            raise ValueError('SCAN_TIME_GAP')
        wheel=compose(inverse(prev['wheel_pose']),cur['wheel_pose'])
        forward=register(scans[t0],scans[t1])
        seeded=register(scans[t0],scans[t1],compose(compose(inverse(MOUNT),wheel),MOUNT))
        reverse=register(scans[t1],scans[t0])
        lidar=base_motion(forward.pose)
        closure=compose(forward.pose,reverse.pose)
        seed_delta=compose(inverse(forward.pose),seeded.pose)
        trusted=(forward.supported and reverse.supported and forward.weak_ratio > .001 and reverse.weak_ratio > .001
                 and np.linalg.norm(closure[:2]) < .03 and abs(closure[2]) < .01
                 and np.linalg.norm(seed_delta[:2]) < .03 and abs(seed_delta[2]) < .01)
        result.append(dict(t0_ns=t0,t1_ns=t1,dt_s=dt,wheel=wheel.tolist(),lidar=lidar.tolist(),
                           difference=(lidar-wheel).tolist(),forward=asdict(forward),reverse=asdict(reverse),
                           wheel_seeded=asdict(seeded),closure=closure.tolist(),seed_delta=seed_delta.tolist(),
                           diagnostic_consistent=bool(trusted),map_status=cur['status'],
                           map_correction_m=cur.get('match',{}).get('correction_m')))
    (args.output/'pairs.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    np.savez_compressed(args.output/'selected_scans.npz',stamps=np.array(sorted(scans)),clouds=np.stack([scans[t] for t in sorted(scans)]))
    summary=dict(scope='Offline diagnostic only; independent scan fit has no map/wheel input; wheel seeding only a cross-check',
                 initializations='identity; wheel-seeded cross-check; identity reverse',
                 input_scan_sha256=digest.hexdigest(),records_sha256=hashlib.sha256(args.records.read_bytes()).hexdigest(),
                 start_s=args.start,end_s=args.end,pairs=len(result),consistent_pairs=sum(r['diagnostic_consistent'] for r in result),
                 warnings=['LiDAR motion is not ground truth','Weak geometry and correlated noise can hide common errors',
                           'Replay wheel poses were reconstructed from speed/steering without GNSS/IMU/pose/TF'])
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))


if __name__ == '__main__':
    main()
