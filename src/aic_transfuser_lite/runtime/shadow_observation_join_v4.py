"""Finite observation-time assembly reusing the audited simulator interpolation.

No receipt-time pose substitution; no inference until all streams are supported.
Pose samples must already denote base_link in one evidenced fixed local frame.
"""
from collections import deque
from dataclasses import replace
import numpy as np
from .spatial_sim_adapter_v4 import Sample, align_observation, interpolate, quaternion_yaw
from aic_transfuser_lite.control.path_control_bridge import PathPose


class ShadowObservationJoin:
    def __init__(self, session, commands, emit, *, clock_id: str, monotonic_id: str,
                 pose_frame: str, pose_evidence: str, wait_ns: int = 300_000_000):
        if not all((clock_id,monotonic_id,pose_frame,pose_evidence)) or not 0 < wait_ns <= 300_000_000:
            raise ValueError('JOIN_CONTRACT_REQUIRED')
        self.session,self.commands,self.emit=session,commands,emit
        self.clock_id,self.monotonic_id=clock_id,monotonic_id
        self.pose_frame,self.pose_evidence=pose_frame,pose_evidence
        self.wait_ns=wait_ns;self.epoch=None;self.phase=None
        self.streams={r:deque(maxlen=64) for r in ('lidar','velocity','steering','pose')}
        self.pending=deque();self.last_camera_ns=None

    def on_input(self, role: str, message: object, received_ns: int, epoch: str) -> None:
        """ROS transport callback adapter; Odometry must explicitly be base_link pose."""
        if role=='clock': return
        header=getattr(message,'header',None)
        t=header.stamp if header is not None else message.stamp
        if type(t.sec) is not int or type(t.nanosec) is not int or t.sec<0 or not 0<=t.nanosec<1_000_000_000:
            raise ValueError('MESSAGE_TIME')
        ns=t.sec*1_000_000_000+t.nanosec
        frame=header.frame_id if header is not None else 'steering_tire_angle'
        if role=='image':
            if message.encoding not in ('rgb8','bgr8') or (message.height,message.width)!=(256,384):
                raise ValueError('CAMERA_CONTRACT')
            value=np.frombuffer(message.data,dtype=np.uint8).reshape(message.height,message.step)
            value=value[:,:message.width*3].reshape(message.height,message.width,3)
            value=(value[...,::-1] if message.encoding=='bgr8' else value).copy()
            role='camera'
        elif role=='lidar':
            value={k:getattr(message,k) for k in ('angle_min','angle_increment','range_min','range_max')}
            value['ranges']=np.asarray(message.ranges,dtype=np.float32).copy()
        elif role=='velocity':
            if frame!='base_link': raise ValueError('VELOCITY_FRAME')
            value=[message.longitudinal_velocity,message.lateral_velocity,message.heading_rate]
        elif role=='steering': value=[message.steering_tire_angle]
        elif role=='odometry':
            if message.child_frame_id!='base_link' or frame!=self.pose_frame:
                raise ValueError('ODOMETRY_BASE_FRAME_UNPROVEN')
            p=message.pose.pose.position;q=message.pose.pose.orientation
            value=[p.x,p.y,quaternion_yaw([q.x,q.y,q.z,q.w])];role='pose'
        else: raise ValueError('UNKNOWN_INPUT_ROLE')
        self.add(role,Sample(ns,received_ns,value,frame,epoch))

    def reset(self, reason: str, epoch: str) -> None:
        for camera in self.pending:
            self.emit(dict(event='JOIN_REJECTED',camera_ns=camera.ns,reason=reason))
        self.pending.clear()
        for stream in self.streams.values(): stream.clear()
        self.session.adapter.reset(reason);self.session.bridge._invalidate(reason)
        self.epoch=epoch;self.phase=None;self.last_camera_ns=None

    def add(self, role: str, sample: Sample) -> None:
        if self.epoch is None: self.epoch=sample.epoch
        if sample.epoch!=self.epoch:
            raise ValueError('EPOCH_RESET_REQUIRED')
        if role=='camera':
            if self.last_camera_ns is not None and sample.ns<=self.last_camera_ns:
                raise ValueError('CAMERA_ORDER')
            self.last_camera_ns=sample.ns
            if self.phase is None: self.phase=sample.ns
            if len(self.pending)>=16:
                old=self.pending.popleft()
                self.emit(dict(event='JOIN_REJECTED',camera_ns=old.ns,reason='QUEUE_FULL'))
            self.pending.append(sample)
        else:
            if role=='pose' and sample.frame!=self.pose_frame: raise ValueError('POSE_FRAME')
            self.streams[role].append(sample)

    def tick(self, finalized_ns: int, now_ros_s: float) -> None:
        """Call from bounded worker, not the ROS receive callback (forward can block)."""
        while self.pending:
            camera=self.pending[0]
            if finalized_ns>=camera.received_ns+self.wait_ns:
                self.pending.popleft()
                self.emit(dict(event='JOIN_REJECTED',camera_ns=camera.ns,reason='JOIN_DEADLINE'))
                self.session.bridge._invalidate('JOIN_DEADLINE')
                continue
            streams={r:sorted((s for s in q if s.received_ns<=finalized_ns),key=lambda s:s.ns)
                     for r,q in self.streams.items()}
            try:
                obs, provenance=align_observation(camera,streams['lidar'],streams['velocity'],
                    streams['steering'],cutoff_ns=finalized_ns,grid_phase_ns=self.phase)
                xyh, pose_source=interpolate(streams['pose'],camera.ns,angle_columns=(2,))
                if xyh.shape!=(3,): raise ValueError('POSE_SHAPE')
            except ValueError as exc:
                self.emit(dict(event='JOIN_WAIT',camera_ns=camera.ns,reason=str(exc)))
                return
            def stamp(s): return replace(s,clock_id=self.clock_id,monotonic_id=self.monotonic_id)
            obs=replace(obs,camera=stamp(obs.camera),lidar=stamp(obs.lidar),ego_stamp=stamp(obs.ego_stamp))
            pose=PathPose(obs.sample_id,camera.ns*1e-9,self.clock_id,self.epoch,
                          tuple(float(x) for x in xyh),self.pose_evidence)
            self.pending.popleft()  # consumed once, including rejected forward
            self.emit(dict(event='JOIN_READY',input_id=obs.sample_id,alignment=provenance,
                           pose=pose_source,pose_frame=self.pose_frame,pose_evidence=self.pose_evidence))
            self.session.observation(obs,self.commands(),pose,finalized_ns=finalized_ns,now_s=now_ros_s)
