from dataclasses import replace
import math
import numpy as np
import pytest
from aic_transfuser_lite.control.time_reference_v1 import (
    TimedBodyPose,TimePlan,TimeReferenceAdapter,prepare_time_reference,reference_control)


def pose(t=0, x=0.,y=0.,yaw=0.):
    return TimedBodyPose(int(round(t*1e9)),'sim','e','map','rear_axle',x,y,yaw)


def line(speed=.15):
    return TimePlan('line',pose(),np.column_stack((np.arange(1,31)/10*speed,np.zeros(30))))


def test_low_speed_short_oracle_reaches_existing_pp_without_horizon_stop():
    plan=line();r=prepare_time_reference(plan,pose())
    assert r.xy_current_m[-1,0]==pytest.approx(.45)
    cmd=reference_control(r,current_speed_mps=0.)
    assert cmd.steering_rad==0 and cmd.acceleration_mps2>0
    assert r.target_speed_mps==pytest.approx(.15)
    end=prepare_time_reference(plan,pose(2.9,.435),max_age_sec=3.)
    assert end.target_speed_mps==pytest.approx(.15)


def test_fractional_age_speed_does_not_include_tracking_offset():
    a=prepare_time_reference(line(1.),pose(.23,.23))
    b=prepare_time_reference(line(1.),pose(.23,.18))
    assert a.remaining_sec[:2]==pytest.approx([0.,.07])
    assert a.xy_current_m[0]==pytest.approx([0.,0.])
    assert b.xy_current_m[0]==pytest.approx([.05,0.])
    assert a.target_speed_mps==pytest.approx(b.target_speed_mps)
    assert a.target_speed_mps==pytest.approx(1.)


def test_rotating_oracle_motion_compensation_and_pp_turn_sign():
    radius=3.;speed=.5;omega=speed/radius
    times=np.arange(1,31)/10
    points=np.column_stack((radius*np.sin(omega*times),radius*(1-np.cos(omega*times))))
    origin=pose(0,4.,-2.,math.pi/2)
    plan=TimePlan('curve',origin,points)
    age=.2;local=np.array([radius*math.sin(omega*age),radius*(1-math.cos(omega*age))])
    current=pose(age,4.-local[1],-2.+local[0],math.pi/2+omega*age)
    r=prepare_time_reference(plan,current)
    np.testing.assert_allclose(r.xy_current_m[0],0.,atol=1e-12)
    cmd=reference_control(r,current_speed_mps=speed)
    assert cmd.steering_rad==pytest.approx(math.atan(1/radius),abs=1e-6)


def test_stationary_launch_and_reset_expiry():
    stop=prepare_time_reference(line(0.),pose())
    cmd=reference_control(stop,current_speed_mps=.1)
    assert cmd.steering_rad==0 and cmd.acceleration_mps2<0
    launch=reference_control(prepare_time_reference(line(.15),pose()),current_speed_mps=0.)
    assert launch.acceleration_mps2>0
    adapter=TimeReferenceAdapter();adapter.accept(line());assert adapter.poll(pose(.2,.03)) is not None
    assert adapter.poll(replace(pose(.1),epoch='new')) is None
    assert adapter.poll(pose(.3)) is None
    adapter=TimeReferenceAdapter();adapter.accept(line());assert adapter.poll(pose(.6)) is None
    with pytest.raises(ValueError):prepare_time_reference(line(),replace(pose(),body_frame='base_link'))
    with pytest.raises(ValueError):TimePlan('bad',pose(),np.zeros((36,2)))


def test_receding_oracle_completes_synthetic_motion_with_progress():
    # Oracle + existing PP/P longitudinal controller + Euler bicycle, no learned model.
    x=y=yaw=speed=0.;dt=.05
    for k in range(100):
        current=pose(k*dt,x,y,yaw)
        world_x=.15*k*dt+np.arange(1,31)/10*.15
        plan=TimePlan(str(k),current,np.column_stack((world_x-x,np.full(30,-y))))
        cmd=reference_control(prepare_time_reference(plan,current),current_speed_mps=speed)
        x+=speed*math.cos(yaw)*dt;y+=speed*math.sin(yaw)*dt
        yaw+=speed*math.tan(cmd.steering_rad)*dt
        speed=max(0.,speed+cmd.acceleration_mps2*dt)
    assert x>.5 and abs(y)<1e-9 and speed>.1


def test_teacher_to_reference_to_pp_without_velocity_or_commands():
    from aic_transfuser_lite.data.mcap_converter_v2 import TimedPose
    from aic_transfuser_lite.data.time_history_v1 import TimeEvent
    from aic_transfuser_lite.data.time_teacher_v1 import build_time_teacher
    def event(t):
        stamp=int(round(t*1e9))
        return TimeEvent('pose','r','e','sim','receipt',stamp,stamp,stamp,
            TimedPose(stamp,.15*t,0.,0.,'map','rear_axle'))
    teacher=build_time_teacher([event(i/10) for i in range(31)],event(0),
        epoch_start_ns=0,epoch_end_ns=3_000_000_000)
    assert teacher.xy_mask.all() and not teacher.velocity_mask.any()
    plan=TimePlan('oracle_teacher',pose(),teacher.xy_m)
    reference=prepare_time_reference(plan,pose(.23,.0345))
    command=reference_control(reference,current_speed_mps=.10)
    assert reference.target_speed_mps==pytest.approx(.15,abs=1e-6)
    assert command.acceleration_mps2==pytest.approx(.05,abs=1e-6)
    assert command.steering_rad==0
