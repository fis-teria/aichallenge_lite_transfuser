"""Input-only V4 wrapper, dependency-injected for non-DDS tests.

No V3 node inheritance. No output middleware endpoints. Live bootstrap is blocked
until passive bindings and a live execution envelope receive separate approval.
"""
from __future__ import annotations

from collections import deque
import time

import numpy as np

from aic_transfuser_lite.runtime.spatial_input_v4 import GridObservation, PassiveCommand, SpatialInputV4, Stamp


class SpatialPathShadowWrapperV4:
    def __init__(self, node: object, runtime: object, adapter: SpatialInputV4, message_types: dict,
                 topics: dict, *, clock=time.monotonic_ns, grid_period_ns: int=100_000_000):
        if any(not topics.get(k) for k in ('image','lidar','velocity','steering','nominal')):
            raise ValueError('BLOCKED_REAL_INPUT_BINDING: explicit topics required')
        self.runtime,self.adapter,self.clock=node and runtime,adapter,clock
        self.grid_period_ns=grid_period_ns
        self.epoch=0
        self.last_image=None
        self.buffers={k:deque(maxlen=16) for k in ('image','lidar','velocity','steering')}
        self.results=deque(maxlen=4)
        self.rejections=deque(maxlen=64)
        self.transport_id=runtime.records.session
        for role in ('image','lidar','velocity','steering','nominal'):
            # QoS matches existing V3 sensor best-effort and vehicle depth10;
            # exact live compatibility remains unverified.
            qos=message_types['sensor_qos'] if role in ('image','lidar') else 10
            node.create_subscription(message_types[role],topics[role],lambda msg,r=role:self.receive(r,msg),qos)

    def receive(self, role: str, msg: object) -> None:
        received=self.clock()
        header=getattr(msg,'header',None)
        value=header.stamp if header is not None else getattr(msg,'stamp',None)
        if value is None:
            self.rejections.append('MISSING_HEADER:'+role)
            return
        ns=int(value.sec)*1_000_000_000+int(value.nanosec)
        if role=='image' and self.last_image is not None and ns<self.last_image:
            self.epoch+=1
            self.adapter.reset('WRAPPER_CLOCK_RESET')
            for q in self.buffers.values():
                q.clear()
        if role=='image':
            if ns==self.last_image:
                self.rejections.append('DUPLICATE_IMAGE')
                return
            self.last_image=ns
        s=Stamp(ns,received,self.clock(),clock_id='ros_header',epoch=str(self.epoch),monotonic_id=self.transport_id)
        if role=='nominal':
            self.adapter.add_command(PassiveCommand(s,float(msg.lateral.steering_tire_angle),float(msg.longitudinal.speed),float(msg.longitudinal.acceleration)))
        else:
            if len(self.buffers[role])==self.buffers[role].maxlen:
                self.rejections.append('TRANSPORT_QUEUE_DROP:'+role)
            self.buffers[role].append((s,msg))
        self.drain()

    def drain(self) -> None:
        while self.buffers['image']:
            camera,msg=self.buffers['image'][0]
            matched={}
            # Minimal safe binding: exact ego-time match only. No unapproved
            # interpolation/nearest-value relabeling or assumed timing parity.
            for role in ('velocity','steering'):
                matches=[v for v in self.buffers[role] if v[0].header_ns==camera.header_ns and v[0].epoch==camera.epoch]
                if not matches:
                    return
                matched[role]=matches[-1]
            scans=[v for v in self.buffers['lidar'] if abs(v[0].header_ns-camera.header_ns)<=30_000_000 and v[0].epoch==camera.epoch]
            if not scans:
                return
            lidar,scan=min(scans,key=lambda v:(abs(v[0].header_ns-camera.header_ns),v[0].header_ns))
            self.buffers['image'].popleft()
            try:
                if msg.encoding!='rgb8' or msg.step<msg.width*3:
                    raise ValueError('RGB8_REQUIRED_NO_ENCODING_GUESS')
                image=np.frombuffer(bytes(msg.data),dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3).copy()
                velocity=matched['velocity'][1]
                steering=matched['steering'][1]
                grid=((camera.header_ns+self.grid_period_ns//2)//self.grid_period_ns)*self.grid_period_ns
                item=GridObservation(grid,camera,lidar,matched['velocity'][0],image,np.asarray(scan.ranges,dtype=np.float32),
                    (float(velocity.longitudinal_velocity),float(velocity.lateral_velocity),float(velocity.heading_rate),float(steering.steering_tire_angle)))
                finalized=self.clock()
                status=self.adapter.append(item,finalized)
                if status!='ACCEPTED':
                    raise ValueError(status)
                batch,provenance=self.adapter.build(finalized)
                self.results.append(self.runtime.infer(batch,provenance))
            except (ValueError,RuntimeError) as exc:
                self.rejections.append(str(exc))
                event=self.runtime.records.accept()
                event['payload'].update(event_type='DROP',status='DROPPED',reason=str(exc))
                self.runtime.records.counts['dropped'].add(event['payload']['candidate_sequence'])
                self.results.append(event)
                if self.runtime.writer:
                    self.runtime.writer.enqueue(event)
                    self.runtime.writer.drain()


def main(args: list[str] | None = None) -> None:
    # Deliberately before importing rclpy: current task authorizes no DDS session.
    # Wrapper above is usable with a ROS Node supplied by separately approved
    # live bootstrap, or the fake interface in the current test harness.
    raise RuntimeError('LIVE_BOOTSTRAP_BLOCKED: passive binding, live envelope and explicit authorization required; no ROS initialization performed')
