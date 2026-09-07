"""Synthetic-only admission/counting/publisher construction verification."""
from dataclasses import replace
from types import SimpleNamespace as Obj
import ast
from pathlib import Path
import sys
import numpy as np
import pytest
from aic_transfuser_lite.runtime.publisherless_shadow_v4 import Envelope, ShadowSession, create_input_only_node
from aic_transfuser_lite.control.path_control_bridge import ShadowBridge


class Adapter:
    def __init__(self): self.commands=[]
    def reset(self, why): self.commands=[]
    def append(self, obs, cutoff): return 'ACCEPTED'
    def add_command(self,c): self.commands.append(c)
    def build(self,cutoff): return None,{'command_policy':'PASSIVE'}


def setup(infer=None):
    events=[]; clock=[0.]
    envelope=Envelope('synthetic',115.,40,200,200.,100000)
    runner=ShadowSession(envelope,Adapter(),infer or (lambda b:np.zeros((20,2),np.float32)),
                         ShadowBridge(None),events.append,monotonic=lambda:clock[0],unix=lambda:0.)
    return runner,events,clock


def obs(i): return Obj(sample_id=str(i),camera=Obj(epoch='0',clock_id='synthetic',header_ns=i*100000000))


def apply(r,i,commands=None):
    return r.observation(obs(i),tuple([Obj(source='nominal')] if commands is None else commands),
                         None,finalized_ns=i,now_s=i*.1)


def test_no_fake_command_no_forward():
    r,e,_=setup();result=apply(r,1,[])
    assert r.forward_calls==0 and 'PASSIVE_COMMAND_MISSING' in result['reason']
    assert not any(x['event']=='FORWARD_STARTED' for x in e)


def test_verified_external_final_command_and_shadow_rejection():
    r,_,_=setup()
    assert 'FINAL_COMMAND_BINDING_UNVERIFIED' in apply(r,1,[Obj(source='final_fallback')])['reason']
    r.adapter.final_fallback_verified=True
    assert apply(r,2,[Obj(source='final_fallback')])['event']=='PLAN'
    assert 'PASSIVE_EXTERNAL_SOURCE_REQUIRED' in apply(r,3,[Obj(source='sim_sent')])['reason']
    assert r.forward_calls==1


def test_exact_forty_including_errors():
    def fail(batch): raise RuntimeError('synthetic forward failure')
    r,e,_=setup(fail)
    for i in range(45): apply(r,i)
    assert r.forward_calls==40
    assert len([x for x in e if x['event']=='FORWARD_STARTED'])==40


def test_duplicate_epoch_missing_profile():
    r,e,_=setup();apply(r,1);apply(r,1)
    assert r.forward_calls==1
    out=r.control_tick(.1,'synthetic','0',None)
    assert not out['value']['valid'] and out['value']['command'] is None
    r.observation(Obj(sample_id='reset',camera=Obj(epoch='1',clock_id='synthetic',header_ns=0)),(),None,finalized_ns=0,now_s=0.)
    assert not r.adapter.commands and r.forward_calls==1


def test_timeout_and_late_model():
    r,e,clock=setup()
    def slow(batch): clock[0]=116.;return np.zeros((20,2),np.float32)
    r.infer=slow
    assert apply(r,1)['reason']=='LATE_FORWARD'
    assert r.bridge.plan is None
    assert not r.active()


@pytest.mark.parametrize('field,value', [('wall_s',116.),('wall_s',float('nan')),('forward_limit',41),
                                       ('candidate_limit',201),('authorized_until_unix_s',1.)])
def test_bounds(field,value):
    with pytest.raises(ValueError): replace(Envelope('s',115.,40,200,200.,100000),**{field:value}).validate(0.)


def test_input_only_node_and_static_no_output_capability():
    roles=('image','lidar','velocity','steering','nominal','odometry','clock')
    class Node:
        def __init__(self,*a,**kwargs): self.options=kwargs;self.roles=[]
        def create_subscription(self,t,topic,cb,qos): self.roles.append(topic)
        def create_publisher(self,*a,**kw): raise AssertionError('No publisher allowed')
    node=create_input_only_node(Node,{**{x:object for x in roles},'qos':{x:1 for x in roles}},
                                {x:'/synthetic/'+x for x in roles},lambda *a:None)
    assert len(node.roles)==7 and not node.options['enable_rosout'] and not node.options['start_parameter_services']
    root=Path(__file__).parents[1]
    for path in (root/'tools/run_v4_publisherless_shadow.py',root/'src/aic_transfuser_lite/runtime/publisherless_shadow_v4.py'):
        tree=ast.parse(path.read_text())
        names={x.attr for x in ast.walk(tree) if isinstance(x,ast.Attribute)}
        assert not names.intersection({'create_publisher','publish','create_client','send_goal_async'})
        assert 'run_spatial_sim_dev_v4' not in path.read_text()


def test_owned_child_outer_timeout():
    sys.path.insert(0,str(Path(__file__).parents[1]/'tools'))
    from run_v4_publisherless_shadow import supervise
    class Process:
        alive=False;exitcode=None
        def start(self): self.alive=True
        def join(self,delay): assert delay>=0
        def is_alive(self): return self.alive
        def terminate(self): pass
        def kill(self): self.alive=False;self.exitcode=-9
    result=supervise(Process(),wall_s=120,monotonic=lambda:0.)
    assert result['timed_out'] and result['child_reaped'] and result['exitcode']==-9
    assert result['retry'] is False
