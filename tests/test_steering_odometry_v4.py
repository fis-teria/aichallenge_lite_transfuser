"""Synthetic geometry only; not measured AWSIM calibration."""
import math
import pytest
from aic_transfuser_lite.runtime.steering_odometry_v4 import SteeringOdometry


def make(**kwargs):
    return SteeringOdometry(wheelbase_m=2., reference_left_offset_m=0., **kwargs)


@pytest.mark.parametrize('v,delta',[(2.,0.),(2.,.3),(2.,-.3),(-2.,.3),(0.,.3)])
def test_analytic_motion_and_raw_heading_ignored(v,delta):
    m=make()
    for i in range(11):
        ns=i*100_000_000
        m.add_velocity(ns,v,0.,1256.33 if i%2 else -1256.33)
        assert m.drain(ns)==[]
        m.add_steering(ns,delta)
        pose,trace=m.drain(ns)[0]
    w=v*math.tan(delta)/2
    x=v if w==0 else v*math.sin(w)/w
    y=0 if w==0 else v*(1-math.cos(w))/w
    assert (pose.x_m,pose.y_m,pose.yaw_rad)==pytest.approx((x,y,w))
    assert trace['estimated_yaw_rate_rps']==pytest.approx(w)


def test_interpolation_callback_order_and_nonfinite_diagnostic():
    m=make();m.add_steering(0,0.);m.add_velocity(50_000_000,2.,0.,float('nan'))
    assert m.drain(50_000_000)==[]
    m.add_steering(100_000_000,.2)
    _,trace=m.drain(100_000_000)[0]
    assert trace['steering_rad']==pytest.approx(.1)
    assert trace['steering_stamps_ns']==[0,100_000_000]
    assert trace['raw_heading_rate']=='nan'


def test_reference_offset_and_lateral_velocity_preserved():
    m=SteeringOdometry(wheelbase_m=2.,reference_left_offset_m=.2)
    m.add_steering(0,math.atan(.4));m.add_velocity(0,1.,.2,0.)
    _,trace=m.drain(0)[0]
    assert trace['estimated_yaw_rate_rps']==pytest.approx(.2/(1-.2*.2))
    assert trace['vy_mps']==.2


def test_timeout_latches_and_reset_discards_caches():
    m=make();m.add_velocity(0,1.,0.,0.)
    with pytest.raises(ValueError,match='TIMEOUT'):m.drain(250_000_001)
    assert m.core.pose is None
    m.reset();assert not m.pending and not m.steering
    m.add_steering(0,0.);m.add_velocity(0,1.,0.,0.)
    assert m.drain(0)[0][0].epoch==1


@pytest.mark.parametrize('angle',[float('nan'),float('inf'),math.pi/2,-math.pi/2])
def test_bad_steering(angle):
    with pytest.raises(ValueError,match='INVALID_STEERING'):make().add_steering(0,angle)


def test_duplicate_conflict_reversal_and_capacity():
    m=make(capacity=2);m.add_steering(0,0.);m.add_steering(0,0.)
    with pytest.raises(ValueError,match='CONFLICTING'):m.add_steering(0,.1)
    m.reset();m.add_velocity(2,1.,0.,0.)
    with pytest.raises(ValueError,match='OUT_OF_ORDER'):m.add_velocity(1,1.,0.,0.)
    m.reset();m.add_velocity(0,1.,0.,0.);m.add_velocity(1,1.,0.,0.)
    with pytest.raises(ValueError,match='BUFFER_FULL'):m.add_velocity(2,1.,0.,0.)


def test_bounded_history_long_stream():
    m=make()
    for i in range(200):
        ns=i*35_000_000
        m.add_steering(ns,0.);m.add_velocity(ns,1.,0.,0.);m.drain(ns)
        assert len(m.steering)<12 and not m.pending


def test_missing_bracket_no_extrapolation_and_gap():
    m=make();m.add_steering(10,0.);m.add_velocity(0,1.,0.,0.)
    assert m.drain(20)==[]
    m.reset();m.add_steering(0,0.);m.add_steering(300_000_000,0.)
    m.add_velocity(150_000_000,1.,0.,0.)
    with pytest.raises(ValueError,match='STEERING_GAP'):m.drain(300_000_000)


def test_bad_geometry_and_singular_geometry():
    with pytest.raises(ValueError,match='WHEELBASE'):SteeringOdometry(wheelbase_m=0.,reference_left_offset_m=0.)
    with pytest.raises(ValueError,match='REFERENCE_OFFSET'):SteeringOdometry(wheelbase_m=1.,reference_left_offset_m=float('nan'))
    m=SteeringOdometry(wheelbase_m=1.,reference_left_offset_m=2.)
    m.add_steering(0,math.atan(.5));m.add_velocity(0,1.,0.,0.)
    with pytest.raises(ValueError,match='SINGULAR'):m.drain(0)
