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
