"""Artificial clocks, helper evidence and observations only. No ROS/assets."""
from dataclasses import replace
from types import SimpleNamespace as Obj
import json
import numpy as np
import pytest
from aic_transfuser_lite.runtime.shadow_start_gate_v4 import StartGate, ForwardPermit, LEASE_NS
from aic_transfuser_lite.runtime.publisherless_shadow_v4 import ShadowSession, Envelope
from aic_transfuser_lite.control.path_control_bridge import ShadowBridge


def receipt(**changes):
    return dict(session_id='fixture', instance_id='synthetic_instance', epoch='0',
                helper_started_ns=20, helper_completed_ns=40, exit_code=0, **changes)


def gate():
    return StartGate('fixture', 'synthetic_instance', 10)


def ready(g):
    g.observe('arm', False, 11)
    g.observe('initialization', True, 12)
    g.observe('arm', True, 30)
    g.complete(receipt(), 40)
    return g.permit(50, '0', 50)


def test_helper_only_retained_true_and_initialization_missing():
    g=gate();g.complete(receipt(),40)
    assert g.permit(50,'0',50) is None
    g.observe('arm',True,51);g.observe('initialization',True,52)
    assert g.permit(53,'0',53) is None  # no current-session false -> true
    g=gate();g.observe('arm',False,11);g.observe('arm',True,30);g.complete(receipt(),40)
    assert g.permit(50,'0',50) is None


@pytest.mark.parametrize('field,value', [('session_id','old'),('instance_id','other'),
    ('epoch','1'),('exit_code',1),('exit_code',True),('helper_started_ns',9),
    ('helper_completed_ns',999)])
def test_bad_helper_receipt(field,value):
    g=gate();r=receipt();r[field]=value;g.complete(r,40)
    assert g.fault=='HELPER_RECEIPT_INVALID' and g.permit(50,'0',50) is None


@pytest.mark.parametrize('event', ['arm_false','init_false','epoch','graph_stale'])
def test_revocation_is_terminal(event):
    g=gate();assert ready(g)
    if event=='arm_false': g.observe('arm',False,51)
    if event=='init_false': g.observe('initialization',False,51)
    result=g.permit(50+LEASE_NS if event=='graph_stale' else 52,
                    '1' if event=='epoch' else '0',50)
    assert result is None and g.fault
    g.observe('arm',True,60);g.observe('initialization',True,60)
    assert g.permit(61,'0',61) is None


def test_latched_true_requires_graph_not_invented_bool_heartbeat():
    g=gate();p=ready(g);assert p['not_before_ns']==50
    assert g.permit(LEASE_NS*5,'0',LEASE_NS*5)['not_before_ns']==50


class Adapter:
    def __init__(self): self.commands=[];self.appended=[];self.builds=0
    def reset(self,reason): self.commands=[]
    def append(self,observation,cutoff): self.appended.append(observation.sample_id);return 'ACCEPTED'
    def add_command(self,c): self.commands=[c]
    def build(self,cutoff): self.builds+=1;return None,{'command_policy':'PASSIVE'}


def session():
    clock=[50];events=[];p=ForwardPermit('fixture',lambda:clock[0]);a=Adapter()
    s=ShadowSession(Envelope('fixture',115.,40,200,200.,100000),a,
        lambda _:np.zeros((20,2),np.float32),ShadowBridge(None),events.append,
        monotonic=lambda:clock[0]*1e-9,unix=lambda:0.,forward_permit=p)
    return s,p,clock,events


def observe(s,i,received_ns=100):
    o=Obj(sample_id=str(i),camera=Obj(epoch='0',clock_id='synthetic',
                                    header_ns=i*100000000,received_ns=received_ns))
    return s.observation(o,(Obj(source='nominal'),),None,finalized_ns=received_ns,now_s=i*.1)


def test_prepare_keeps_history_without_build_forward_or_replay(tmp_path):
    s,p,c,events=session()
    for i in range(5): assert observe(s,i)['event']=='PREPARED'
    assert s.adapter.appended==['0','1','2','3','4'] and s.adapter.builds==s.forward_calls==0
    p.update(ready(gate()))
    assert observe(s,5,49)['reason']=='PRE_START_OBSERVATION'
    assert observe(s,6)['event']=='PLAN' and s.forward_calls==1
    assert s.candidates==7 and s.adapter.builds==1
    assert observe(s,6)['reason']=='DUPLICATE_CANDIDATE'
    (tmp_path/'synthetic_start_trace.json').write_text(json.dumps(events,indent=2))


def test_waiting_and_phase_change_never_reset_limits():
    s,p,c,_=session();s.envelope=replace(s.envelope,candidate_limit=3)
    observe(s,0);observe(s,1);p.update(ready(gate()));observe(s,2)
    assert observe(s,3)['reason']=='CANDIDATE_LIMIT' and s.forward_calls==1
    s,p,c,_=session();c[0]+=116_000_000_000
    assert observe(s,0)['reason']=='SESSION_DEADLINE' and s.forward_calls==0


def test_lease_expires_during_forward_and_idle():
    s,p,c,events=session();p.update(ready(gate()))
    def slow(_): c[0]+=LEASE_NS;return np.zeros((20,2),np.float32)
    s.infer=slow
    assert observe(s,0)['event']=='REJECTED'
    assert s.forward_calls==1 and s.bridge.plan is None
    assert not any(e['event']=='PLAN' for e in events)
    s,p,c,_=session();p.update(ready(gate()));assert s.active()
    c[0]+=LEASE_NS;assert not s.active() and s.terminal


def test_expired_queued_lease_cannot_start_forward():
    s,p,c,_=session();p.update(ready(gate()));c[0]+=LEASE_NS
    assert observe(s,0)['event']=='REJECTED' and s.forward_calls==0


def test_no_control_apis():
    import ast
    from pathlib import Path
    path=Path(__file__).parents[1]/'src/aic_transfuser_lite/runtime/shadow_start_gate_v4.py'
    attrs={n.attr for n in ast.walk(ast.parse(path.read_text())) if isinstance(n,ast.Attribute)}
    assert not attrs.intersection({'create_publisher','publish','create_client','send_goal_async'})


def test_receipt_handoff_never_overwrites_or_accepts_failure(tmp_path):
    from aic_transfuser_lite.runtime.shadow_start_gate_v4 import save_helper_completion
    path=tmp_path/'complete.json'
    save_helper_completion(path,receipt())
    assert json.loads(path.read_text())==receipt()
    with pytest.raises(FileExistsError): save_helper_completion(path,receipt())
    bad=receipt();bad['exit_code']=1
    with pytest.raises(ValueError): save_helper_completion(tmp_path/'bad.json',bad)
    assert not (tmp_path/'bad.json').exists()


def test_ros_observer_with_synthetic_graph_and_messages(monkeypatch,tmp_path):
    import sys
    from aic_transfuser_lite.runtime import shadow_start_gate_v4 as m
    monkeypatch.setitem(sys.modules,'std_msgs.msg',Obj(Bool=object))
    monkeypatch.setitem(sys.modules,'rclpy.qos',Obj(
        QoSProfile=lambda **kw:kw,ReliabilityPolicy=Obj(RELIABLE='reliable'),
        DurabilityPolicy=Obj(TRANSIENT_LOCAL='transient_local')))
    clock=[10];monkeypatch.setattr(m.time,'monotonic_ns',lambda:clock[0])
    config=dict(instance_id='synthetic_instance',receipt_file=str(tmp_path/'receipt.json'),
                expected_node='/autostart_orchestrator',
                topics={'arm':'/overtake/race_armed',
                        'initialization':'/autostart/initialization_ready'})
    class Node:
        def __init__(self): self.callbacks={};self.bad=False;self.removed=[]
        def get_publishers_info_by_topic(self,topic):
            return [] if self.bad else [Obj(node_namespace='/',node_name='autostart_orchestrator')]
        def create_subscription(self,kind,topic,callback,qos):
            assert qos['durability']=='transient_local' and qos['reliability']=='reliable'
            self.callbacks[topic]=callback;return topic
        def create_timer(self,period,callback): assert period==.1;return 'timer'
        def destroy_subscription(self,sub): self.removed.append(sub)
        def destroy_timer(self,timer): self.removed.append(timer)
    n=Node();events=[];o=m.ROSStartObserver(n,config,'fixture',events.append)
    clock[0]=11;n.callbacks[config['topics']['arm']](Obj(data=False))
    clock[0]=12;n.callbacks[config['topics']['initialization']](Obj(data=True))
    assert o.poll('0') is None
    clock[0]=30;n.callbacks[config['topics']['arm']](Obj(data=True))
    assert o.poll('0') is None  # helper has not completed
    clock[0]=40;m.save_helper_completion(tmp_path/'receipt.json',receipt())
    p=o.poll('0');assert p and p['not_before_ns']==40
    clock[0]=50;n.bad=True;o.check_graph()
    assert o.poll('0') is None and o.gate.fault.startswith('START_GRAPH_INVALID')
    o.close();assert len(n.removed)==3
    assert any(e['event']=='HELPER_RECEIPT' for e in events)
    with pytest.raises(ValueError,match='START_RECEIPT_ALREADY_EXISTS'):
        m.ROSStartObserver(n,config,'fixture',events.append)


def test_revoke_worker_permit_prevents_later_reuse():
    s,p,c,_=session();token=ready(gate());p.update(token)
    assert observe(s,0)['event']=='PLAN'
    p.update(None)
    assert observe(s,1)['event']=='REJECTED'
    p.update(token)
    assert observe(s,2)['event']=='REJECTED' and s.forward_calls==1
