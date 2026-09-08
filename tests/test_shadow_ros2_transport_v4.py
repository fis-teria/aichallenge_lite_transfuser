"""ROS-free message-info fixtures; no real observation or model inference."""
from types import SimpleNamespace as O
import ast
import inspect
from pathlib import Path
import pytest
from aic_transfuser_lite.runtime.shadow_ros2_transport_v4 import ShadowROS2Transport
from aic_transfuser_lite.runtime.passive_controller_command_v4 import ControllerCommandBinding


ROLES=('image','lidar','velocity','steering','command','odometry','clock')


class Node:
    def __init__(self): self.subs=[]
    def create_subscription(self,t,topic,cb,q): self.subs.append(cb);return cb
    def destroy_subscription(self,sub): self.subs.remove(sub)


def setup():
    events=[];resets=[];inputs=[];node=Node()
    binding=ControllerCommandBinding('/fixture/command','01','final_fallback','FIXTURE',
                                     'TIRE_RAD_TARGET_MPS_ACCEL_MPS2')
    t=ShadowROS2Transport(node,dict.fromkeys(ROLES,object),{r:'/fixture/'+r for r in ROLES},
                          dict.fromkeys(ROLES,1),binding,dict.fromkeys(ROLES,'01'),
                          lambda *a:inputs.append(a),lambda *a:resets.append(a),events.append,
                          clock_id='fixture_sim',monotonic_id='fixture_process',monotonic=lambda:20)
    return t,node,events,resets,inputs


def clock(t,sec): t.receive('clock',O(clock=O(sec=sec,nanosec=0)),O(publisher_gid=[1]))
def command(): return O(stamp=O(sec=1,nanosec=0),lateral=O(steering_tire_angle=.1),longitudinal=O(speed=.2,acceleration=.3))


def test_subscription_command_binding_and_cleanup():
    t,node,events,resets,inputs=setup();assert len(node.subs)==7
    assert all(len(inspect.signature(cb).parameters)==2 for cb in node.subs)
    clock(t,2);node.subs[4](command(),O(publisher_gid=[1]))
    c=t.command_snapshot()[0]
    assert c.source=='final_fallback' and c.stamp.header_ns==1_000_000_000
    assert c.stamp.received_ns==20 and c.stamp.clock_id=='fixture_sim'
    assert inputs[0][0]=='clock' and not events
    t.close();assert not node.subs and not t.command_snapshot()


@pytest.mark.parametrize('gid',[[],[2]])
def test_wrong_publisher_invalidates(gid):
    t,_,events,resets,_=setup();clock(t,2)
    t.receive('command',command(),O(publisher_gid=gid))
    assert not t.command_snapshot() and resets[-1][0]=='INPUT_REJECTED'
    assert 'PUBLISHER_GID_CHANGED' in events[-1]['reason']


def test_clock_required_reset_and_buffer_bound():
    t,_,events,resets,_=setup()
    t.receive('command',command(),O(publisher_gid=[1]))
    assert 'CLOCK_NOT_OBSERVED' in events[-1]['reason']
    clock(t,2)
    for _ in range(65): t.receive('command',command(),O(publisher_gid=[1]))
    assert len(t.command_snapshot())==64 and events[-1]['event']=='COMMAND_BUFFER_EVICT'
    clock(t,0)
    assert not t.command_snapshot() and resets[-1]==('CLOCK_RESET','1')


def test_no_output_or_implicit_start():
    p=Path(__file__).parents[1]/'src/aic_transfuser_lite/runtime/shadow_ros2_transport_v4.py'
    attrs={n.attr for n in ast.walk(ast.parse(p.read_text())) if isinstance(n,ast.Attribute)}
    assert not attrs.intersection({'create_publisher','publish','create_client','send_goal_async','load_fixed'})
