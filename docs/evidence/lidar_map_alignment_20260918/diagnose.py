"""Offline diagnostic only: recorded localization is a reference, not ground truth.

Fit apparent planar scan-to-raster corrections; never write TF or a runtime config.
Map boundary discretization is <= 0.01 m, raster resolution is 0.1 m.
"""
from pathlib import Path
import argparse
import hashlib
import json
import math
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image
import yaml
from scipy.spatial import cKDTree
from scipy.optimize import least_squares
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore


class BoundaryIndex:
    """Find nearby segments with <=2cm samples, then project onto exact segments."""
    def __init__(self, segments: np.ndarray) -> None:
        samples, starts, ends = [], [], []
        for a,b in segments:
            n=max(1,math.ceil(np.linalg.norm(b-a)/.02))
            samples.append(a+np.linspace(0,1,n+1)[:,None]*(b-a))
            starts.append(np.tile(a,(n+1,1)));ends.append(np.tile(b,(n+1,1)))
        self.data=np.concatenate(samples);self.starts=np.concatenate(starts);self.ends=np.concatenate(ends)
        self.tree=cKDTree(self.data)

    def query(self, points: np.ndarray) -> tuple[np.ndarray,np.ndarray]:
        _,indices=self.tree.query(points,k=min(8,len(self.data)))
        a,b=self.starts[indices],self.ends[indices];v=b-a
        denom=np.maximum(np.sum(v*v,axis=2),1e-20)
        fraction=np.clip(np.sum((points[:,None,:]-a)*v,axis=2)/denom,0,1)
        projected=a+fraction[:,:,None]*v
        distance=np.linalg.norm(points[:,None,:]-projected,axis=2)
        best=np.argmin(distance,axis=1)
        return distance[np.arange(len(points)),best],projected[np.arange(len(points)),best]


def rotate(p: np.ndarray, yaw: np.ndarray | float) -> np.ndarray:
    c, s = np.cos(yaw), np.sin(yaw)
    return np.column_stack((c*p[:, 0]-s*p[:, 1], s*p[:, 0]+c*p[:, 1]))


def pose(p: object, q: object) -> np.ndarray:
    return np.array([p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))])


def seconds(t: object) -> float:
    return t.sec + t.nanosec * 1e-9


def boundary(path: Path) -> tuple[BoundaryIndex, dict, np.ndarray]:
    if path.suffix == '.json':
        rows = [m for m in json.loads(path.read_text()) if m['ns'] in ['left_lane_bound','right_lane_bound']]
        assert rows
        origin = np.min(np.concatenate([m['xyz'] for m in rows]),axis=0)[:2]
        samples=[]
        for m in rows:
            assert m['type']==11 and m['frame']=='map'
            assert np.allclose([m['pose']['x'],m['pose']['y']],[0,0])
            assert np.allclose(m['pose']['q'],[0,0,0,1])
            points=np.array(m['xyz'])[:,:2]-origin
            assert len(points)%6==0
            triangles=points.reshape(-1,6,2)
            assert np.allclose(triangles[:,4],triangles[:,2]) and np.allclose(triangles[:,5],triangles[:,1])
            # Actual marker is a 0.1m-wide triangle strip. Compare its centreline,
            # not either arbitrary rendered edge, to measured wall returns.
            for vertices in triangles:
                a=(vertices[0]+vertices[1])/2; b=(vertices[2]+vertices[3])/2
                n=math.ceil(np.linalg.norm(b-a)/.02)
                samples.append([a,b])
        return BoundaryIndex(np.array(samples)),dict(origin=[*origin,0.],map_type='Actual RViz left/right lane bound markers; not physical wall surfaces',
                markers_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),marker_count=len(rows)),None
    if path.suffix == '.osm':
        xml = ET.parse(path).getroot()
        nodes = {}
        for n in xml.findall('node'):
            tags = {t.attrib['k']:t.attrib['v'] for t in n.findall('tag')}
            nodes[n.attrib['id']] = np.array([float(tags['local_x']),float(tags['local_y'])])
        ids = {m.attrib['ref'] for r in xml.findall('relation') for m in r.findall('member')
               if m.attrib.get('role') in ['left','right'] and m.attrib['type']=='way'}
        origin = np.min(list(nodes.values()),axis=0)
        samples = []
        for way in xml.findall('way'):
            if way.attrib['id'] not in ids: continue
            points = [nodes[n.attrib['ref']]-origin for n in way.findall('nd')]
            for a,b in zip(points[:-1],points[1:]):
                n=math.ceil(np.linalg.norm(b-a)/.02)
                samples.append([a,b])
        return BoundaryIndex(np.array(samples)),dict(origin=[*origin,0.],map_type='lanelet left/right boundaries; not physical wall surfaces',
                    image_sha256=None,osm_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),ways=len(ids)),None
    meta = yaml.safe_load(path.read_text())
    assert meta['negate'] == 0 and meta['origin'][2] == 0
    image = path.parent / meta['image']
    pixels = np.asarray(Image.open(image))
    occ = 1-pixels.astype(float)/255
    grid = np.where(occ < meta['free_thresh'], 0, np.where(occ > meta['occupied_thresh'], 100, -1))
    h, w = grid.shape
    segments = []
    for axis in (0, 1):
        a, b = (grid[:-1], grid[1:]) if axis == 0 else (grid[:, :-1], grid[:, 1:])
        rows, cols = np.nonzero(((a == 0) & (b == 100)) | ((a == 100) & (b == 0)))
        if axis == 0:
            start = np.column_stack((cols, h-rows-1)); delta = np.array([1., 0.])
        else:
            start = np.column_stack((cols+1, h-rows-1)); delta = np.array([0., 1.])
        steps = math.ceil(meta['resolution']/.02)
        segments.append(np.stack((start,start+delta),axis=1)*meta['resolution'])
    points = np.concatenate(segments)
    meta['yaml_sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
    meta['image_sha256'] = hashlib.sha256(image.read_bytes()).hexdigest()
    return BoundaryIndex(points), meta, grid


def read_bag(path: Path) -> tuple[list[dict], dict]:
    scans, poses, static = [], {}, {}
    wanted = {'/localization/pose', '/sensing/lidar/scan', '/tf_static'}
    last = -math.inf
    with AnyReader([path], default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as reader:
        for conn, receipt, raw in reader.messages(connections=[c for c in reader.connections if c.topic in wanted]):
            msg = reader.deserialize(raw, conn.msgtype)
            if conn.topic == '/tf_static':
                for tf in msg.transforms:
                    p, q = tf.transform.translation, tf.transform.rotation
                    value = dict(parent=tf.header.frame_id, xyz=[p.x,p.y,p.z], quaternion=[q.x,q.y,q.z,q.w])
                    if tf.child_frame_id in static:
                        assert static[tf.child_frame_id] == value, 'Changing static TF'
                    static[tf.child_frame_id] = value
                continue
            t = seconds(msg.header.stamp)
            if conn.topic == '/localization/pose':
                assert msg.header.frame_id == 'map'
                p = msg.pose.pose if hasattr(msg.pose, 'pose') else msg.pose
                poses[t] = pose(p.position, p.orientation)
            elif t >= 5 and t >= last+3:
                assert msg.header.frame_id == 'lidar'
                ranges = np.asarray(msg.ranges, float)
                angles = msg.angle_min+np.arange(len(ranges))*msg.angle_increment
                good = np.isfinite(ranges) & (ranges > max(.3,msg.range_min)) & (ranges < min(15.,msg.range_max))
                indices = np.flatnonzero(good)[::3]
                points = ranges[indices, None]*np.column_stack((np.cos(angles[indices]),np.sin(angles[indices])))
                scans.append(dict(stamp_s=t, points=points, time_increment_s=float(msg.time_increment)))
                last = t
    mount = static['lidar_base_link']
    assert mount['parent'] == 'base_link' and np.allclose(mount['xyz'], [1.65,0,0])
    assert np.allclose(mount['quaternion'], [0,0,0,1])
    assert static['lidar']['parent'] == 'lidar_base_link'
    assert np.allclose(static['lidar']['xyz'][:2], [0,0]) and np.allclose(static['lidar']['quaternion'], [0,0,0,1])
    times = np.array(sorted(poses)); values = np.array([poses[t] for t in times])
    values[:, 2] = np.unwrap(values[:, 2])
    result = []
    for s in scans:
        t = s['stamp_s']; right = np.searchsorted(times,t)
        if right == 0 or right == len(times) or times[right]-times[right-1] > .1:
            continue
        alpha = (t-times[right-1])/(times[right]-times[right-1])
        s['pose'] = (1-alpha)*values[right-1]+alpha*values[right]
        # AWSIM snapshot scan, not a physical rolling scanner; timing recorded explicitly.
        result.append(s)
    return result, static


def stats(a: np.ndarray) -> dict:
    return dict(n=len(a), mean=float(np.mean(a)), median=float(np.median(a)), p95=float(np.quantile(a,.95)))


def fit(tree: BoundaryIndex, local: np.ndarray, poses: np.ndarray, mode: str) -> tuple[np.ndarray, np.ndarray, object]:
    nominal = rotate(local+[1.65,0], poses[:,2])+poses[:,:2]
    pivot = poses[:,:2].mean(axis=0)
    def predict(d: np.ndarray) -> np.ndarray:
        if mode == 'mount':
            return rotate(rotate(local,d[2])+np.array([1.65,0])+d[:2],poses[:,2])+poses[:,:2]
        return rotate(nominal-pivot,d[2])+pivot+d[:2]
    def residual(d: np.ndarray) -> np.ndarray:
        points = predict(d)
        _, projected = tree.query(points)
        return (points-projected).ravel()
    opt = least_squares(residual,np.zeros(3),bounds=([-1.,-1.,-.15],[1.,1.,.15]),loss='soft_l1',f_scale=.1,
                        max_nfev=80,xtol=1e-9,ftol=1e-9,gtol=1e-9,diff_step=1e-4)
    return opt.x, tree.query(predict(opt.x))[0], opt


def main() -> None:
    ap = argparse.ArgumentParser(); ap.add_argument('--bag',type=Path,required=True)
    ap.add_argument('--map',type=Path,required=True); ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--reference-fit',type=Path)
    args = ap.parse_args(); args.output.mkdir(exist_ok=False)
    # Non-degenerate synthetic rectangle: recover a known mounting translation.
    reference = np.concatenate([np.column_stack((np.linspace(2,8,101),np.full(101,y))) for y in [-2.,3.]] +
                               [np.column_stack((np.full(101,8.),np.linspace(-2,3,101)))])
    synthetic = reference-np.array([1.65+.12,-.08])
    synthetic_tree=BoundaryIndex(np.array([[[2.,-2.],[8.,-2.]],[[2.,3.],[8.,3.]],[[8.,-2.],[8.,3.]]]))
    delta, residual, opt = fit(synthetic_tree,synthetic,np.zeros((len(synthetic),3)),'mount')
    assert np.max(np.abs(delta-np.array([.12,-.08,0.]))) < .005, delta
    tree, meta, grid = boundary(args.map)
    frames, static = read_bag(args.bag)
    origin = np.array(meta['origin'][:2])
    all_points, all_poses, frame_ids, records = [], [], [], []
    for i, frame in enumerate(frames):
        p = frame['pose'].copy(); p[:2] -= origin
        local = frame['points']; pp = np.tile(p,(len(local),1))
        nominal = rotate(local+[1.65,0],p[2])+p[:2]
        before = tree.query(nominal)[0]
        keep = before < .8  # Fixed baseline gate; retain denominator and all-hit error separately.
        if keep.sum() < 40: continue
        d, after, opt = fit(tree,local[keep],pp[keep],'mount')
        records.append(dict(frame=i,stamp_s=frame['stamp_s'],pose_map_local=p.tolist(),
                            all_hits=stats(before),retained=stats(before[keep]),fit_error=stats(after),
                            apparent_mount_delta_xy_m_yaw_rad=d.tolist(),optimizer_success=bool(opt.success),
                            fit_condition_number=float(np.linalg.cond(opt.jac)),time_increment_s=frame['time_increment_s']))
        all_points.append(local[keep]);all_poses.append(pp[keep]);frame_ids.extend([i]*int(keep.sum()))
    local=np.concatenate(all_points);poses=np.concatenate(all_poses);ids=np.array(frame_ids)
    train = ((ids//5)%2 == 0)  # Alternating 15s blocks, not random beams.
    nominal=rotate(local+[1.65,0],poses[:,2])+poses[:,:2]
    result=dict(bag=str(args.bag),map=meta,static_tf=static,frame_count=len(records),
                synthetic_check='PASS',before=stats(tree.query(nominal)[0]),
                yaw_span_rad=float(np.ptp(poses[:,2])),runtime_tf_changed=False,
                reference='/localization/pose: diagnostic recorded localization, not independently measured ground truth',
                fit_scope='Planar nearest displayed lane boundary or raster wall edge, baseline residual <0.8m, range 0.3-15m; map/localization/mount ambiguity unresolved',
                all_valid_sampled_hits=sum(r['all_hits']['n'] for r in records),
                baseline_gate_m=.8,point_sampling_step=3,scan_sampling_s=3,scan_timing_model='AWSIM snapshot header',
                frames=records,models={})
    for mode in ['mount','map']:
        d, error, opt = fit(tree,local[train],poses[train],mode)
        if mode == 'mount':
            transformed=rotate(rotate(local,d[2])+np.array([1.65,0])+d[:2],poses[:,2])+poses[:,:2]
        else:
            pivot=poses[train,:2].mean(axis=0)
            transformed=rotate(nominal-pivot,d[2])+pivot+d[:2]
        result['models'][mode]=dict(delta_xy_m_yaw_rad=d.tolist(),optimizer_success=bool(opt.success),
                                   pivot_map_local_xy_m=poses[train,:2].mean(axis=0).tolist() if mode=='map' else None,
                                   train_error=stats(error),heldout_before=stats(tree.query(nominal[~train])[0]),
                                   heldout_after=stats(tree.query(transformed[~train])[0]))
    if args.reference_fit:
        ref=json.loads(args.reference_fit.read_text())
        assert ref['map']==meta
        model=ref['models']['map'];d=np.array(model['delta_xy_m_yaw_rad']);pivot=np.array(model['pivot_map_local_xy_m'])
        transformed=rotate(nominal-pivot,d[2])+pivot+d[:2]
        result['fixed_reference_transfer']=dict(reference=str(args.reference_fit),delta=model,
                                               before=stats(tree.query(nominal)[0]),after=stats(tree.query(transformed)[0]))
    (args.output/'summary.json').write_text(json.dumps(result,indent=2,allow_nan=False))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,2,figsize=(12,9))
    for ax,idx in zip(axes.ravel(),np.linspace(0,len(records)-1,4,dtype=int)):
        r=records[idx]; frame=frames[r['frame']]; p=np.array(r['pose_map_local']); pts=frame['points']
        model=result['models']['map'];d=np.array(model['delta_xy_m_yaw_rad']);pivot=np.array(model['pivot_map_local_xy_m'])
        baseline=rotate(pts+[1.65,0],p[2])+p[:2]
        fitted=rotate(baseline-pivot,d[2])+pivot+d[:2]
        near=np.linalg.norm(tree.data-p[:2],axis=1)<17
        ax.scatter(tree.data[near,0],tree.data[near,1],s=1,c='black',label='Map boundary')
        ax.scatter(baseline[:,0],baseline[:,1],s=5,c='orange',label='Recorded TF')
        ax.scatter(fitted[:,0],fitted[:,1],s=5,c='deepskyblue',label='One common map-frame correction')
        ax.set_aspect('equal'); ax.set_xlim(p[0]-12,p[0]+12);ax.set_ylim(p[1]-12,p[1]+12)
        ax.set_title(f"t={r['stamp_s']:.1f}s; same correction at all positions")
        ax.set_xlabel('Map-local x [m]');ax.set_ylabel('Map-local y [m]')
    axes[0,0].legend(fontsize=8);fig.suptitle('Apparent scan/map error is not a mounting calibration')
    fig.tight_layout();fig.savefig(args.output/'overlay.png',dpi=140)
    print(json.dumps({k:v for k,v in result.items() if k not in ['frames','static_tf','map']},indent=2))


if __name__ == '__main__': main()
