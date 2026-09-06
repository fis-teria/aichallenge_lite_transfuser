"""Input-only V4 wrapper, dependency-injected for non-DDS tests.

No V3 node inheritance. No output middleware endpoints. Live bootstrap is blocked
until passive bindings and a live execution envelope receive separate approval.
"""
from __future__ import annotations

from collections import deque
from copy import deepcopy
import time

import numpy as np

from aic_transfuser_lite.runtime.spatial_input_v4 import GridObservation, PassiveCommand, SpatialInputV4, Stamp
from aic_transfuser_lite.runtime.spatial_recording_v4 import stamp


class SpatialPathShadowWrapperV4:
    def __init__(self, node: object, runtime: object, adapter: SpatialInputV4, message_types: dict,
                 topics: dict, *, clock=None, clock_id: str | None = None,
                 grid_period_ns: int=100_000_000, max_sync_wait_ns: int=300_000_000,
                 candidate_capacity: int=16):
        if any(not topics.get(k) for k in ('image','lidar','velocity','steering','nominal')):
            raise ValueError('BLOCKED_REAL_INPUT_BINDING: explicit topics required')
        for name,value in (('grid_period_ns',grid_period_ns),('max_sync_wait_ns',max_sync_wait_ns),
                           ('candidate_capacity',candidate_capacity)):
            if type(value) is not int or value<=0:
                raise ValueError(name+' must be a positive Python int (bool excluded)')
        self.runtime,self.adapter=runtime,adapter
        self.clock=clock if clock is not None else runtime.records.clock
        # Different clock callables are NOT relabeled as the Records clock.
        self.transport_id=clock_id or (runtime.records.session if self.clock==runtime.records.clock else 'wrapper:'+str(id(self.clock)))
        self.grid_period_ns,self.max_sync_wait_ns,self.candidate_capacity=grid_period_ns,max_sync_wait_ns,candidate_capacity
        self.epoch=0
        self.monotonic_epoch=0
        self.last_image=None
        self.last_tick=None
        self.pending=deque()
        self.buffers={k:deque(maxlen=16) for k in ('lidar','velocity','steering')}
        self.results=deque(maxlen=4)
        self.completed=deque(maxlen=64)
        self.rejections=deque(maxlen=64)  # Other-topic diagnostics, not camera candidates.
        self.sensor_received={k:0 for k in ('lidar','velocity','steering','nominal')}
        for role in ('image','lidar','velocity','steering','nominal'):
            qos=message_types['sensor_qos'] if role in ('image','lidar') else 10
            node.create_subscription(message_types[role],topics[role],lambda msg,r=role:self.receive(r,msg),qos)

    def _received(self, ns: int) -> dict:
        return stamp(ns,clock=self.transport_id,epoch=str(self.monotonic_epoch),source='wrapper camera callback monotonic')

    def _complete(self, context: object, reason: str | None = None) -> None:
        self.runtime.finish(context,reason)
        if not any(c is context for c in self.completed):
            self.completed.append(context)
            self.results.append(context.event)

    def _clear_waiting(self, reason: str) -> None:
        while self.pending:
            context,_,_=self.pending.popleft()
            self._complete(context,reason)
        for q in self.buffers.values():
            q.clear()
        self.adapter.reset(reason)

    def receive(self, role: str, msg: object) -> None:
        received=self.clock()
        if self.last_tick is not None and received<self.last_tick:
            self.tick(received)  # Establish new epoch before assigning this receipt.
        context=self.runtime.accept_candidate(self._received(received)) if role=='image' else None
        if role in self.sensor_received:
            self.sensor_received[role]+=1
        # Expire before admitting late ego/scan: a dropped camera cannot revive.
        self.tick(received)
        try:
            header=getattr(msg,'header',None)
            value=header.stamp if header is not None else getattr(msg,'stamp',None)
            if value is None:
                raise ValueError('MISSING_HEADER:'+role)
            ns=int(value.sec)*1_000_000_000+int(value.nanosec)
            if role=='image' and self.last_image is not None and ns<self.last_image:
                self._clear_waiting('CAMERA_CLOCK_RESET')
                self.epoch+=1
            if role=='image':
                if ns==self.last_image:
                    self._complete(context,'DUPLICATE_IMAGE')
                    return
                self.last_image=ns
            s=Stamp(ns,received,self.clock(),clock_id='ros_header',epoch=str(self.epoch),monotonic_id=self.transport_id,monotonic_epoch=str(self.monotonic_epoch))
            if role=='nominal':
                result=self.adapter.add_command(PassiveCommand(s,float(msg.lateral.steering_tire_angle),float(msg.longitudinal.speed),float(msg.longitudinal.acceleration)))
                if result!='ACCEPTED':
                    self.rejections.append(result)
            elif role=='image':
                if len(self.pending)>=self.candidate_capacity:
                    old,_,_=self.pending.popleft()
                    self._complete(old,'CAMERA_QUEUE_OVERFLOW')
                self.pending.append((context,s,deepcopy(msg)))
            else:
                if len(self.buffers[role])==self.buffers[role].maxlen:
                    self.rejections.append('TRANSPORT_QUEUE_DROP:'+role)
                self.buffers[role].append((s,deepcopy(msg)))
        except Exception as exc:
            reason='CALLBACK_ERROR:'+type(exc).__name__+': '+str(exc)[:500]
            if context is not None:
                self._complete(context,reason)
            else:
                self.rejections.append(reason)
        self.tick()

    def drain(self) -> None:
        self.tick()

    def tick(self, now_ns: int | None = None) -> None:
        """Pure polling entry; no ROS timer or background worker is created."""
        cutoff=self.clock() if now_ns is None else now_ns
        if self.last_tick is not None and cutoff<self.last_tick:
            self.monotonic_epoch+=1
            if self.transport_id==self.runtime.records.session and self.clock==self.runtime.records.clock:
                self.runtime.records.clock_epoch=str(self.monotonic_epoch)
            self._clear_waiting('MONOTONIC_CLOCK_RESET')
            self.epoch+=1
            self.last_image=None
        self.last_tick=cutoff
        while self.pending:
            context,camera,msg=self.pending[0]
            if cutoff-camera.received_ns>=self.max_sync_wait_ns:
                self.pending.popleft()
                self._complete(context,'SYNC_DEADLINE')
                continue
            if self.runtime.writer and self.runtime.writer.state!='OPEN':
                self.pending.popleft()
                self._complete(context,'LOGGER_'+self.runtime.writer.state)
                continue
            matched={}
            for role in ('velocity','steering'):
                matches=[v for v in self.buffers[role] if v[0].header_ns==camera.header_ns and v[0].epoch==camera.epoch and v[0].available_ns<=cutoff]
                if not matches:
                    return
                matched[role]=matches[-1]
            scans=[v for v in self.buffers['lidar'] if abs(v[0].header_ns-camera.header_ns)<=30_000_000 and v[0].epoch==camera.epoch and v[0].available_ns<=cutoff]
            if not scans:
                return
            lidar,scan=min(scans,key=lambda v:(abs(v[0].header_ns-camera.header_ns),v[0].header_ns))
            self.pending.popleft()
            context.selection_cutoff=stamp(cutoff,clock=self.transport_id,epoch=str(self.monotonic_epoch),source='selection cutoff before preprocessing')
            try:
                if msg.encoding!='rgb8' or msg.step<msg.width*3:
                    raise ValueError('RGB8_REQUIRED_NO_ENCODING_GUESS')
                image=np.frombuffer(bytes(msg.data),dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3).copy()
                velocity=matched['velocity'][1]
                steering=matched['steering'][1]
                grid=((camera.header_ns+self.grid_period_ns//2)//self.grid_period_ns)*self.grid_period_ns
                item=GridObservation(grid,camera,lidar,matched['velocity'][0],image,np.asarray(scan.ranges,dtype=np.float32),
                    (float(velocity.longitudinal_velocity),float(velocity.lateral_velocity),float(velocity.heading_rate),float(steering.steering_tire_angle)),
                    range_min_m=float(scan.range_min),range_max_m=float(scan.range_max))
                status=self.adapter.append(item,cutoff)
                if status!='ACCEPTED':
                    raise ValueError(status)
                batch,provenance=self.adapter.build(cutoff)
                self.runtime.infer(batch,provenance,candidate=context)
                self._complete(context)
            except Exception as exc:
                # The same context retains any forward/output already obtained.
                self._complete(context,'PROCESSING_ERROR:'+type(exc).__name__+': '+str(exc)[:500])
            cutoff=self.clock()


def create_ros_node(runtime: object, adapter: SpatialInputV4, topics: dict) -> object:
    """Construction hook for separately approved bootstrap; never called here.

    No rclpy.init/spin inside this function. Caller owns an authorized context.
    Tests inject a fake Node into SpatialPathShadowWrapperV4 instead.
    """
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, LaserScan
    from autoware_auto_vehicle_msgs.msg import VelocityReport, SteeringReport
    from autoware_auto_control_msgs.msg import AckermannControlCommand
    node=Node('spatial_path_shadow_v4',enable_rosout=False,start_parameter_services=False)
    try:
        node.v4_wrapper=SpatialPathShadowWrapperV4(node,runtime,adapter,
            dict(image=Image,lidar=LaserScan,velocity=VelocityReport,steering=SteeringReport,nominal=AckermannControlCommand,sensor_qos=qos_profile_sensor_data),topics)
    except Exception:
        node.destroy_node()
        raise
    return node


def main(args: list[str] | None = None) -> None:
    # Deliberately before importing rclpy: current task authorizes no DDS session.
    # Wrapper above is usable with a ROS Node supplied by separately approved
    # live bootstrap, or the fake interface in the current test harness.
    raise RuntimeError('LIVE_BOOTSTRAP_BLOCKED: passive binding, live envelope and explicit authorization required; no ROS initialization performed')
