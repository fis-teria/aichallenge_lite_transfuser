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
    if len(xy)!=20 or len(pose)!=3 or any(len(p)!=2 for p in xy):raise ValueError('SHAPE')
    if not all(math.isfinite(v) for v in pose+ [v for p in xy for v in p]):raise ValueError('NONFINITE')
    x,y,a=pose;c,s=math.cos(a),math.sin(a)
    return [(x+c*p[0]-s*p[1],y+s*p[0]+c*p[1]) for p in xy]
