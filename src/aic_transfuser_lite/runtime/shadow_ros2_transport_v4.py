"""Dedicated simulator inputs with graph ownership checks, NOT per-message authentication."""
from collections import deque
from types import SimpleNamespace
import time
from .passive_controller_command_v4 import ControllerCommandBinding
from .spatial_bootstrap_v4 import extract_message_stamp
from .spatial_input_v4 import Stamp


def ros2_types() -> dict:
    from sensor_msgs.msg import Image, LaserScan
    from autoware_auto_vehicle_msgs.msg import VelocityReport, SteeringReport
    from autoware_auto_control_msgs.msg import AckermannControlCommand
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    return dict(image=Image,lidar=LaserScan,velocity=VelocityReport,steering=SteeringReport,
                command=AckermannControlCommand,odometry=Odometry,clock=Clock)


class ShadowROS2Transport:
    """Exact single publisher and fully qualified node per topic; faults latch.

    Graph snapshots cannot identify individual queued messages or exclude
    transient publishers between polls. Dedicated isolated simulator only.
    Caller owns node/executor and must expire plans independently of inference.
    """
    def __init__(self,node,types: dict,topics: dict,qos: dict,
                 binding: ControllerCommandBinding,expected_nodes: dict,
                 on_input,on_reset,emit,*,clock_id: str,monotonic_id: str,
                 monotonic=time.monotonic_ns,graph_period_s: float=.1,
                 graph_ttl_ns: int=500_000_000):
        binding.validate()
        roles={'image','lidar','velocity','steering','command','odometry','clock'}
        if (set(topics)!=roles or set(types)!=roles or set(qos)!=roles or set(expected_nodes)!=roles or
                any(not isinstance(v,str) or not v.startswith('/') for v in (*topics.values(),*expected_nodes.values())) or
                topics['command']!=binding.topic or expected_nodes['command']!=binding.producer_id or
                not clock_id or not monotonic_id or not 0<graph_period_s<=.1 or
                type(graph_ttl_ns) is not int or not graph_period_s*1e9<graph_ttl_ns<=500_000_000):
            raise ValueError('EXPLICIT_GRAPH_BINDINGS_REQUIRED')
        self.node,self.binding=node,binding
        self.topics,self.expected_nodes=dict(topics),dict(expected_nodes)
        self.on_input,self.on_reset,self.emit=on_input,on_reset,emit
        self.clock_id,self.monotonic_id,self.monotonic=clock_id,monotonic_id,monotonic
        # Timer cadence and graph age MUST use the same ROS clock. Receipt
        # timestamps and blocking-query budgets remain host monotonic time.
        self.graph_clock=node.get_clock()
        self.commands=deque(maxlen=64);self.epoch=0;self.last_ros_ns=None
        self.closed=False;self.fault=None;self.checked_ns=None;self.graph_ttl_ns=graph_ttl_ns
        self.subscriptions=[];self.timer=None
        if not self.check_graph(): raise ValueError(self.fault)
        def callback_for(role):
            def callback(message): self.receive(role,message)
            return callback
        try:
            for role in topics:
                self.subscriptions.append(node.create_subscription(types[role],topics[role],callback_for(role),qos[role]))
            self.timer=node.create_timer(graph_period_s,self.check_graph,clock=self.graph_clock)
        except Exception:
            self.close();raise

    def _reject(self,reason: str) -> bool:
        if self.fault is None:
            self.fault=reason;self.commands.clear();self.last_ros_ns=None
            self.on_reset(reason,str(self.epoch))
            self.emit(dict(event='TRANSPORT_FAULT',reason=reason,
                           source_verification='GRAPH_SINGLE_PUBLISHER_NOT_PER_MESSAGE'))
        return False

    def check_graph(self) -> bool:
        if self.closed or self.fault: return False
        before=self.monotonic()
        graph_now=self.graph_clock.now().nanoseconds
        if self.last_ros_ns is not None and not self._fresh(): return False
        try:
            for role,topic in self.topics.items():
                endpoints=self.node.get_publishers_info_by_topic(topic)
                if len(endpoints)!=1: return self._reject('PUBLISHER_COUNT:'+role)
                e=endpoints[0]
                name=e.node_namespace.rstrip('/')+'/'+e.node_name
                if name!=self.expected_nodes[role]: return self._reject('PUBLISHER_NODE:'+role)
            after=self.monotonic()
            if not 0<=after-before<=self.graph_ttl_ns: return self._reject('GRAPH_QUERY_TIMEOUT')
            self.checked_ns=graph_now  # ROS time, never a host receipt timestamp.
            return True
        except Exception as exc:
            return self._reject('GRAPH_QUERY_ERROR:'+type(exc).__name__)

    def _fresh(self) -> bool:
        if self.closed or self.fault: return False
        # Before the first /clock, inputs are rejected by receive(). Do not
        # expire a simulation-time lease using startup wall time.
        if self.last_ros_ns is None: return True
        age=self.graph_clock.now().nanoseconds-self.checked_ns
        if age<0:
            self.epoch+=1
            return self._reject('CLOCK_RESET')
        if age>self.graph_ttl_ns:
            return self._reject('GRAPH_MONITOR_STALE')
        return True

    def receive(self,role: str,message: object) -> None:
        if not self._fresh(): return
        received=self.monotonic()
        try:
            if role=='clock':
                ns,_=extract_message_stamp('nominal',SimpleNamespace(stamp=message.clock),
                                           {'stamp_source':'stamp','message_frame':None})
                if self.last_ros_ns is not None and ns<self.last_ros_ns:
                    self.epoch+=1;self._reject('CLOCK_RESET');return
                if self.last_ros_ns is None and not self.check_graph(): return
                self.last_ros_ns=ns
            elif self.last_ros_ns is None:
                self.emit(dict(event='INPUT_REJECTED',role=role,reason='CLOCK_NOT_OBSERVED'));return
            if role=='command':
                ns,_=extract_message_stamp('nominal',message,{'stamp_source':'stamp','message_frame':None})
                stamp=Stamp(ns,received,self.monotonic(),clock_id=self.clock_id,
                            epoch=str(self.epoch),monotonic_id=self.monotonic_id)
                command=self.binding.decode_graph_observed(message,stamp)
                if len(self.commands)==64: self.emit(dict(event='COMMAND_BUFFER_EVICT'))
                self.commands.append(command)
            else: self.on_input(role,message,received,str(self.epoch))
        except Exception as exc:
            self._reject('INPUT_REJECTED:'+type(exc).__name__+':'+str(exc))

    def command_snapshot(self) -> tuple:
        return tuple(self.commands) if self._fresh() else ()

    def close(self) -> None:
        self._reject('TRANSPORT_CLOSED');self.closed=True
        if self.timer is not None: self.node.destroy_timer(self.timer);self.timer=None
        for sub in self.subscriptions: self.node.destroy_subscription(sub)
        self.subscriptions.clear()
