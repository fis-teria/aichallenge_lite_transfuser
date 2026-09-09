"""Synthetic fixtures only; no physical accuracy claim."""
import ast
import math
from pathlib import Path
import pytest
from aic_transfuser_lite.runtime.local_odometry_v4 import LocalOdometry


def make():
    return LocalOdometry(max_gap_s=.25,max_speed_mps=10.,max_yaw_rate_rps=2.)


@pytest.mark.parametrize('vx,vy,w',[(2.,0.,0.),(-2.,0.,0.),(0.,1.,0.),(2.,0.,1.),(2.,0.,-1.),(0.,0.,0.)])
def test_analytic_constant_body_twist(vx,vy,w):
    core=make()
    for i in range(11): pose=core.update(i*100_000_000,vx,vy,w)
    a=1. if w==0 else math.sin(w)/w
    b=0. if w==0 else (1-math.cos(w))/w
    assert (pose.x_m,pose.y_m,pose.yaw_rad)==pytest.approx((a*vx-b*vy,b*vx+a*vy,w))


def test_duplicate_and_conflict_and_reset():
    core=make();first=core.update(0,1.,0.,0.)
    assert core.update(0,1.,0.,0.) is None
    assert core.pose==first
    with pytest.raises(ValueError,match='CONFLICTING'): core.update(0,2.,0.,0.)
    with pytest.raises(ValueError,match='CONFLICTING'): core.update(100,1.,0.,0.)
    core.reset();pose=core.update(0,1.,0.,0.)
    assert pose.epoch==1 and pose.x_m==0


@pytest.mark.parametrize('ns,values,reason',[
    (0,(float('nan'),0.,0.),'NONFINITE'),
    (0,(0.,float('inf'),0.),'NONFINITE'),
    (0,(11.,0.,0.),'PROFILE'),(0,(0.,0.,3.),'PROFILE'),
    (-1,(0.,0.,0.),'INVALID_STAMP')])
def test_invalid_input(ns,values,reason):
    with pytest.raises(ValueError,match=reason): make().update(ns,*values)


def test_gap_and_time_reversal_not_integrated():
    for ns,reason in [(500_000_000,'GAP'),(0,'OUT_OF_ORDER')]:
        core=make();first=core.update(100_000_000,1.,0.,0.)
        with pytest.raises(ValueError,match=reason): core.update(ns,1.,0.,0.)
        assert core.pose==first


def test_trapezoidal_acceleration():
    core=make();core.update(0,0.,0.,0.)
    assert core.update(100_000_000,2.,0.,0.).x_m==pytest.approx(.1)


def test_isolated_node_has_no_ekf_or_actuator_inputs():
    root=Path(__file__).parents[1]/'ros2_ws/src/aic_e2e_runtime'
    source=(root/'aic_e2e_runtime/local_odometry_node_v4.py').read_text()
    tree=ast.parse(source)
    strings={n.value for n in ast.walk(tree) if isinstance(n,ast.Constant) and isinstance(n.value,str)}
    assert '/vehicle/status/velocity_status' in strings and '/v4/local_odometry' in strings
    assert not any(s.startswith(('/localization/','/sensing/imu','/sensing/gnss','/control/','/admin/')) for s in strings)
    attrs={n.attr for n in ast.walk(tree) if isinstance(n,ast.Attribute)}
    assert not attrs.intersection({'create_client','lookup_transform','send_goal_async'})
    assert "parent,child='v4_odom','v4_base_link'" in source
    assert 'local_odometry_node_v4' in (root/'setup.py').read_text()


def test_local_pose_can_be_joined_without_map_ekf():
    from types import SimpleNamespace as O
    import numpy as np
    from aic_transfuser_lite.runtime.shadow_observation_join_v4 import ShadowObservationJoin
    from aic_transfuser_lite.runtime.spatial_sim_adapter_v4 import Sample
    calls=[]
    session=O(observation=lambda *a,**kw:calls.append(a))
    join=ShadowObservationJoin(session,lambda:(),lambda r:None,clock_id='sim',monotonic_id='host',
                              pose_frame='v4_odom',pose_child_frame='v4_base_link',pose_evidence='synthetic')
    ns=1_000_000_000
    join.add('camera',Sample(ns,10,np.zeros((256,384,3),dtype=np.uint8),'camera_optical_link','0'))
    scan=dict(ranges=np.ones(750),angle_min=-1.5666074752807617,angle_increment=.004188789986073971,range_min=0.,range_max=25.)
    join.add('lidar',Sample(ns,10,scan,'lidar','0'))
    join.add('velocity',Sample(ns,10,[1.,0.,0.],'base_link','0'))
    join.add('steering',Sample(ns,10,[0.],'steering_tire_angle','0'))
    message=O(header=O(stamp=O(sec=1,nanosec=0),frame_id='v4_odom'),child_frame_id='v4_base_link',
              pose=O(pose=O(position=O(x=.1,y=.2),orientation=O(x=0.,y=0.,z=0.,w=1.))))
    join.on_input('odometry',message,10,'0');join.tick(20,1.)
    assert len(calls)==1 and calls[0][2].base_in_local==(.1,.2,0.)
