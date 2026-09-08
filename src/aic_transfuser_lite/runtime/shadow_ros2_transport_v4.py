"""Input-only ROS2 transport. No model load, simulator launch or control output.

Caller owns the context/executor and synchronized observation construction.
Raw sensor messages are forwarded, not reinterpreted as aligned observations.
"""
from collections import deque
import time
from types import SimpleNamespace
from typing import Callable

from .passive_controller_command_v4 import ControllerCommandBinding
from .spatial_bootstrap_v4 import extract_message_stamp
from .spatial_input_v4 import Stamp


def ros2_types() -> dict:
    """Lazy imports: inspection/import of this module never initializes ROS."""
    from sensor_msgs.msg import Image, LaserScan
    from autoware_auto_vehicle_msgs.msg import VelocityReport, SteeringReport
    from autoware_auto_control_msgs.msg import AckermannControlCommand
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    return dict(image=Image, lidar=LaserScan, velocity=VelocityReport,
                steering=SteeringReport, command=AckermannControlCommand,
                odometry=Odometry, clock=Clock)


class ShadowROS2Transport:
    """Bounded passive command buffer; caller supplies audited QoS and graph GIDs.

    on_input(role, message, received_monotonic_ns, epoch) must not block for
    inference. on_reset invalidates caller sensor queues, poses and cached plans.
    No clock, frame or ego interpolation is fabricated by this transport.
    """
    def __init__(self, node: object, types: dict, topics: dict, qos: dict,
                 binding: ControllerCommandBinding, expected_gids: dict,
                 on_input: Callable, on_reset: Callable, emit: Callable, *,
                 clock_id: str, monotonic_id: str, monotonic=time.monotonic_ns,
                 serialized_receiver: bool = False):
        binding.validate()
        roles=set(ros_role for ros_role in ('image','lidar','velocity','steering','command','odometry','clock'))
        if (set(topics)!=roles or set(qos)!=roles or set(expected_gids)!=roles or
                any(not isinstance(t,str) or not t.startswith('/') for t in topics.values()) or
                any(not isinstance(g,str) or not g for g in expected_gids.values()) or
                topics['command']!=binding.topic or expected_gids['command']!=binding.producer_id or
                not clock_id or not monotonic_id):
            raise ValueError('EXPLICIT_ROS_BINDINGS_REQUIRED')
        self.node,self.binding,self.expected_gids=node,binding,dict(expected_gids)
        self.on_input,self.on_reset,self.emit=on_input,on_reset,emit
        self.clock_id,self.monotonic_id,self.monotonic=clock_id,monotonic_id,monotonic
        self.commands=deque(maxlen=64)
        self.epoch=0;self.last_ros_ns=None;self.closed=False;self.subscriptions=[]
        self.serialized_receiver=serialized_receiver
        self.types=dict(types)
        def make_callback(role):
            def callback(message, info):
                self.receive(role,message,info)
            return callback
        try:
            for role in (() if serialized_receiver else topics):
                self.subscriptions.append(node.create_subscription(types[role],topics[role],make_callback(role),qos[role]))
        except Exception:
            self.close()
            raise

    def ingest_wire(self, line: bytes, deserialize=None) -> None:
        """One bounded line from owned C++ receiver pipe, NOT untrusted ROS text.

        Reader must bound readline to 2,097,408 bytes and invalidate on EOF/exit.
        The child and parent must share CLOCK_MONOTONIC; do not use across hosts.
        """
        if not self.serialized_receiver or self.closed:
            raise ValueError('SERIALIZED_RECEIVER_NOT_ACTIVE')
        try:
            if len(line)>2_097_408 or not line.endswith(b'\n'): raise ValueError('IPC_LINE_BOUND')
            role,gid,received,payload=line.decode('ascii').strip().split(' ')
            if role not in self.types or gid!=self.expected_gids[role]: raise ValueError('PUBLISHER_GID_CHANGED')
            received=int(received)
            if not 0<=received<=self.monotonic(): raise ValueError('IPC_CLOCK_DOMAIN')
            if deserialize is None:
                from rclpy.serialization import deserialize_message
                deserialize=deserialize_message
            msg=deserialize(bytes.fromhex(payload),self.types[role])
            self.receive(role,msg,SimpleNamespace(publisher_gid=bytes.fromhex(gid)),received_ns=received)
        except Exception as exc:
            self.commands.clear();self.on_reset('IPC_REJECTED',str(self.epoch))
            self.emit(dict(event='INPUT_REJECTED',reason=str(exc)))

    def receive(self, role: str, message: object, info: object, *, received_ns=None) -> None:
        if self.closed:
            return
        received=self.monotonic() if received_ns is None else received_ns
        try:
            gid=bytes(info.publisher_gid).hex()
            if gid!=self.expected_gids[role]:
                raise ValueError('PUBLISHER_GID_CHANGED')
            if role=='clock':
                ns, _=extract_message_stamp('nominal', type('ClockMessage',(),{'stamp':message.clock})(),
                                           {'stamp_source':'stamp','message_frame':None})
                if self.last_ros_ns is not None and ns<self.last_ros_ns:
                    self.epoch+=1;self.commands.clear()
                    self.on_reset('CLOCK_RESET',str(self.epoch))
                self.last_ros_ns=ns
            elif self.last_ros_ns is None:
                raise ValueError('CLOCK_NOT_OBSERVED')
            if role=='command':
                ns,_=extract_message_stamp('nominal',message,{'stamp_source':'stamp','message_frame':None})
                stamp=Stamp(ns,received,self.monotonic(),clock_id=self.clock_id,
                            epoch=str(self.epoch),monotonic_id=self.monotonic_id)
                command=self.binding.decode(message,stamp,producer_id=gid,external_controller=True)
                if len(self.commands)==self.commands.maxlen:
                    self.emit(dict(event='COMMAND_BUFFER_EVICT',epoch=str(self.epoch)))
                self.commands.append(command)
            else:
                self.on_input(role,message,received,str(self.epoch))
        except Exception as exc:
            self.commands.clear()
            self.on_reset('INPUT_REJECTED',str(self.epoch))
            self.emit(dict(event='INPUT_REJECTED',role=role,reason=type(exc).__name__+':'+str(exc)))

    def command_snapshot(self) -> tuple:
        """Existing SpatialInputV4 applies availability/past-slot/50ms checks."""
        return tuple(self.commands)

    def close(self) -> None:
        self.closed=True;self.commands.clear()
        for sub in self.subscriptions:
            self.node.destroy_subscription(sub)
        self.subscriptions.clear()
