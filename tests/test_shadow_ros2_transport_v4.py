"""Graph-owned single-publisher policy fixtures; no DDS or actual control."""
from types import SimpleNamespace as O
import ast
import inspect
from pathlib import Path
import pytest
from aic_transfuser_lite.runtime.shadow_ros2_transport_v4 import ShadowROS2Transport
from aic_transfuser_lite.runtime.passive_controller_command_v4 import ControllerCommandBinding

ROLES=('image','lidar','velocity','steering','command','odometry','clock')

class Node:
    def __init__(self):
        self.subs=[];self.timers=[]
        self.graph={r:[O(node_name='external',node_namespace='/fixture')] for r in ROLES}
    def get_publishers_info_by_topic(self,topic): return self.graph[topic.split('/')[-1]]
    def create_subscription(self,t,topic,cb,q): self.subs.append(cb);return cb
    def destroy_subscription(self,sub): self.subs.remove(sub)
    def create_timer(self,period,cb): self.timers.append(cb);return cb
    def destroy_timer(self,timer): self.timers.remove(timer)

def setup(node=None):
    events=[];resets=[];inputs=[];node=node or Node();now=[20]
    binding=ControllerCommandBinding('/fixture/command','/fixture/external','final_fallback',
                                    'FIXTURE','TIRE_RAD_TARGET_MPS_ACCEL_MPS2')
    t=ShadowROS2Transport(node,dict.fromkeys(ROLES,object),{r:'/fixture/'+r for r in ROLES},
                         dict.fromkeys(ROLES,1),binding,dict.fromkeys(ROLES,'/fixture/external'),
                         lambda *a:inputs.append(a),lambda *a:resets.append(a),events.append,
                         clock_id='fixture',monotonic_id='fixture_process',monotonic=lambda:now[0])
    return t,node,events,resets,inputs,now

def clock(t,sec): t.receive('clock',O(clock=O(sec=sec,nanosec=0)))
def command(): return O(stamp=O(sec=1,nanosec=0),lateral=O(steering_tire_angle=.1),
                        longitudinal=O(speed=.2,acceleration=.3))

def test_single_argument_receipt_and_cleanup():
    t,node,events,resets,inputs,now=setup()
    assert len(node.subs)==7 and len(node.timers)==1
    assert all(len(inspect.signature(cb).parameters)==1 for cb in node.subs)
    clock(t,2);node.subs[4](command())
    c=t.command_snapshot()[0]
    assert c.source=='final_fallback' and c.stamp.received_ns==20
    assert inputs[0][0]=='clock'
    t.close()
    assert not node.subs and not node.timers and not t.command_snapshot()
    assert resets[-1][0]=='TRANSPORT_CLOSED'

@pytest.mark.parametrize('endpoints',[[],[O(node_name='wrong',node_namespace='/fixture')],
    [O(node_name='external',node_namespace='/fixture')]*2])
def test_bad_startup_rejected(endpoints):
    node=Node();node.graph['command']=endpoints
    with pytest.raises(ValueError,match='PUBLISHER_'): setup(node)
    assert not node.subs and not node.timers

@pytest.mark.parametrize('change',['missing','duplicate','name','error'])
def test_live_change_latches_and_clears(change):
    t,node,events,resets,inputs,now=setup();clock(t,2);t.receive('command',command())
    if change=='missing': node.graph['command']=[]
    elif change=='duplicate': node.graph['command']*=2
    elif change=='name': node.graph['command']=[O(node_name='other',node_namespace='/')]
    else:
        def fail(topic): raise RuntimeError('graph unavailable')
        node.get_publishers_info_by_topic=fail
    node.timers[0]()
    assert not t.command_snapshot() and resets
    node.graph['command']=[O(node_name='external',node_namespace='/fixture')]
    t.receive('command',command())
    assert not t.command_snapshot()  # no automatic recovery
    assert events[-1]['source_verification']=='GRAPH_SINGLE_PUBLISHER_NOT_PER_MESSAGE'

@pytest.mark.parametrize('action',['receive','snapshot','timer'])
def test_monitor_stale_or_paused_timer_invalidates(action):
    t,node,events,resets,inputs,now=setup();clock(t,2);t.receive('command',command())
    now[0]+=500_000_001
    if action=='receive': t.receive('command',command())
    elif action=='snapshot': t.command_snapshot()
    else: node.timers[0]()
    assert t.fault=='GRAPH_MONITOR_STALE' and not t.commands

def test_clock_startup_and_reset_and_finite_history():
    t,node,events,resets,inputs,now=setup()
    t.receive('command',command());assert not t.commands and t.fault is None
    clock(t,2)
    for _ in range(65): t.receive('command',command())
    assert len(t.commands)==64
    clock(t,0);assert not t.commands and resets[-1]==('CLOCK_RESET','1')

def test_no_output_or_gid_ipc():
    p=Path(__file__).parents[1]/'src/aic_transfuser_lite/runtime/shadow_ros2_transport_v4.py'
    code=p.read_text()
    attrs={n.attr for n in ast.walk(ast.parse(code)) if isinstance(n,ast.Attribute)}
    assert not attrs.intersection({'create_publisher','publish','create_client','send_goal_async','load_fixed'})
    assert 'ingest_wire' not in code and 'publisher_gid' not in code
