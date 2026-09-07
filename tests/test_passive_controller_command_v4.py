"""Artificial external-controller requests; no ROS, assets or actual inference."""
from dataclasses import replace
from types import SimpleNamespace as O
import pytest
from aic_transfuser_lite.runtime.passive_controller_command_v4 import ControllerCommandBinding
from aic_transfuser_lite.runtime.spatial_input_v4 import SpatialInputV4, Stamp
from aic_transfuser_lite.runtime.publisherless_shadow_v4 import create_input_only_node


def binding(source='nominal'):
    return ControllerCommandBinding('/fixture/controller', 'fixture-mpc-gid', source,
                                    'SYNTHETIC_ONLY', 'TIRE_RAD_TARGET_MPS_ACCEL_MPS2')


def decode(source='nominal', **kwargs):
    msg=O(stamp=O(sec=1,nanosec=0),lateral=O(steering_tire_angle=.1),
          longitudinal=O(speed=1.,acceleration=.2))
    return binding(source).decode(msg,Stamp(1_000_000_000,10,20),
                                  producer_id=kwargs.get('producer_id','fixture-mpc-gid'),
                                  external_controller=kwargs.get('external_controller',True))


def test_external_command_fields_and_source_not_relabelled():
    c=decode('final_fallback')
    assert (c.steering_rad,c.speed_mps,c.acceleration_mps2)==(.1,1.,.2)
    assert c.source=='final_fallback' and c.stamp.header_ns==1_000_000_000


@pytest.mark.parametrize('speed,header',[(float('nan'),1_000_000_000),(1.,0)])
def test_bad_value_or_retimestamp_rejected(speed,header):
    msg=O(stamp=O(sec=1,nanosec=0),lateral=O(steering_tire_angle=.1),
          longitudinal=O(speed=speed,acceleration=.2))
    with pytest.raises(ValueError):
        binding().decode(msg,Stamp(header,10,20),producer_id='fixture-mpc-gid',external_controller=True)


@pytest.mark.parametrize('change',[{'producer_id':'shadow'},{'external_controller':False}])
def test_no_own_shadow_or_other_publisher(change):
    with pytest.raises(ValueError,match='PRODUCER_MISMATCH'): decode(**change)


@pytest.mark.parametrize('change',[{'topic':'/shadow/control'}, {'source':'sim_sent'},
                                  {'contract_evidence':''}, {'semantics':'UNKNOWN'}])
def test_unknown_binding_rejected(change):
    with pytest.raises(ValueError): replace(binding(),**change).validate()


def test_real_history_selector_keeps_nominal_priority_and_verified_final():
    a=SpatialInputV4(command_binding_known=True)
    anchor=Stamp(1_000_000_000,10,20); current=replace(anchor,header_ns=1_100_000_000)
    a.add_command(decode('final_fallback'))
    assert a._command_for(anchor,current,20)[0] is None
    a.final_fallback_verified=True
    assert a._command_for(anchor,current,20)[0][1]=='final_fallback'
    a.add_command(decode())
    assert a._command_for(anchor,current,20)[0][1]=='nominal'
    assert a._command_for(anchor,current,19)[0] is None  # not available yet
    assert a._command_for(anchor,anchor,20)[0] is None  # not strictly past
    assert a._command_for(replace(anchor,header_ns=1_060_000_000),current,20)[0] is None
    assert a._command_for(anchor,replace(current,epoch='reset'),20)[0] is None


def test_selected_external_topic_is_subscribed_without_output():
    class Node:
        def __init__(self,*a,**kw): self.subscriptions=[]
        def create_subscription(self,t,topic,callback,qos): self.subscriptions.append(topic)
    roles=('image','lidar','velocity','steering','command','odometry','clock')
    topics={r:'/fixture/'+r for r in roles};topics['command']=binding().topic
    node=create_input_only_node(Node,{**dict.fromkeys(roles,object),'qos':dict.fromkeys(roles,1)},
                                topics,lambda *args:None,command_binding=binding())
    assert binding().topic in node.subscriptions and len(node.subscriptions)==7
