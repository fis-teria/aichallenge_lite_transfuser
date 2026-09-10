"""Pure planar conversions for SLAM observation poses and RViz-only paths."""
import math


def lidar_to_root(x: float, y: float, yaw: float, forward_m: float) -> tuple[float,float,float]:
    if not all(math.isfinite(v) for v in (x,y,yaw,forward_m)) or forward_m<0:
        raise ValueError('SLAM_GEOMETRY_INVALID')
    return x-forward_m*math.cos(yaw),y-forward_m*math.sin(yaw),yaw


def display_points(record: dict) -> list[tuple[float,float]]:
    """Uncorrected (20,2) metres -> fixed SLAM frame at observation time."""
    if record.get('event')!='PLAN':raise ValueError('NOT_PLAN')
    xy=record['raw_xy_m'];pose=record['observation_pose_xyyaw']
    count = {'LONG_V4_20M_EPOCH12_UNCORRECTED': 46,
             'TEN_V4_10M_EPOCH2_UNCORRECTED': 36}.get(record.get('source'), 20)
    if len(xy)!=count or len(pose)!=3 or any(len(p)!=2 for p in xy):raise ValueError('SHAPE')
    if not all(math.isfinite(v) for v in pose+ [v for p in xy for v in p]):raise ValueError('NONFINITE')
    x,y,a=pose;c,s=math.cos(a),math.sin(a)
    return [(x+c*p[0]-s*p[1],y+s*p[0]+c*p[1]) for p in xy]


def lidar_display_points(record: dict) -> list[tuple[float,float,float]]:
    """Raw root XY -> observed LiDAR frame for standard RViz TF display only.

    Current verified AWSIM mount: forward 1.1649999618530273 m, up
    0.05000000074505806 m, aligned axes. No map/EKF value enters inference.
    """
    display_points(record)  # Same shape/finite/source contract as local display.
    return [(float(x)-1.1649999618530273,float(y),-0.05000000074505806)
            for x,y in record['raw_xy_m']]
