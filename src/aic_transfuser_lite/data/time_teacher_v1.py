"""Epoch/frame-safe time XY labels independent of velocity and command availability."""
from __future__ import annotations
from dataclasses import dataclass
from typing import Sequence
import math
import numpy as np
from .time_history_v1 import TimeEvent
from .mcap_converter_v2 import TimedPose
from .canonical_converter_v3 import _interpolate_pose_indexed
from .synchronization_v3 import IndexedTimedValues, TimedValue, linear_interpolate

@dataclass(frozen=True)
class TimeTeacher:
    xy_m: np.ndarray
    xy_mask: np.ndarray
    velocity_mps: np.ndarray
    velocity_mask: np.ndarray
    interval_mask: np.ndarray
    endpoints_ns: tuple[tuple[int, ...], ...]
    reasons: tuple[str, ...]


def build_time_teacher(events: Sequence[TimeEvent], anchor: TimeEvent, *,
                       epoch_start_ns: int, epoch_end_ns: int,
                       intervention_ns: int | None = None, tolerance_ms: float = 50.0) -> TimeTeacher:
    """30 points, dt=0.1s in body@camera observation; 50ms is EACH-endpoint tolerance.

    Anchor payload is a pose at the camera observation time, not a nearest pose
    relabelled as exact. XY and measured longitudinal velocity masks are independent.
    Conservative cruising policy excludes an anchor whose horizon meets intervention.
    """
    obs = anchor.payload
    if not isinstance(obs, TimedPose) or obs.timestamp_ns != anchor.capture_ns:
        raise ValueError("anchor requires pose at observation timestamp")
    if not obs.frame_id or not obs.child_frame_id or not np.isfinite([obs.x_world_m,obs.y_world_m,obs.yaw_world_rad]).all():
        raise ValueError("invalid anchor frame/pose")
    if not epoch_start_ns <= anchor.capture_ns <= epoch_end_ns or not math.isfinite(tolerance_ms) or tolerance_ms < 0:
        raise ValueError("invalid epoch/tolerance")
    selected = {}
    for e in events:
        if (e.run,e.epoch,e.capture_clock) != (anchor.run,anchor.epoch,anchor.capture_clock):continue
        if not epoch_start_ns <= e.capture_ns <= epoch_end_ns:continue
        key = (e.role,e.capture_ns)
        old = selected.get(key)
        if old is None or (e.available_ns,e.sequence) > (old.available_ns,old.sequence):selected[key]=e
    poses = IndexedTimedValues.from_values(tuple(TimedValue(t,e.payload) for (role,t),e in sorted(selected.items()) if role=='pose'))
    velocities = IndexedTimedValues.from_values(tuple(TimedValue(t,float(e.payload.longitudinal_mps)) for (role,t),e in sorted(selected.items()) if role=='velocity'))
    xy = np.full((30,2),np.nan,dtype=np.float32); velocity=np.full(30,np.nan,dtype=np.float32)
    xm=np.zeros(30,dtype=bool);vm=xm.copy(); endpoints=[];reasons=[]
    blocked=intervention_ns is not None and anchor.capture_ns+3_000_000_000 >= intervention_ns
    c,s=math.cos(obs.yaw_world_rad),math.sin(obs.yaw_world_rad)
    for i in range(30):
        t=anchor.capture_ns+(i+1)*100_000_000
        ep=();reason='OK'
        if blocked: reason='COLLECTION_INTERVENTION'
        elif t>epoch_end_ns:reason='EPOCH_END'
        else:
            try:
                p,timing=_interpolate_pose_indexed(poses,t,tolerance_ms=tolerance_ms)
                ep=(t+timing.before_delta_ns,t+timing.after_delta_ns)
                if (p.frame_id,p.child_frame_id)!=(obs.frame_id,obs.child_frame_id):raise ValueError('FRAME_MISMATCH')
                if intervention_ns is not None and max(ep)>=intervention_ns:raise ValueError('INTERVENTION_ENDPOINT')
                dx,dy=p.x_world_m-obs.x_world_m,p.y_world_m-obs.y_world_m
                value=[c*dx+s*dy,-s*dx+c*dy]
                if not np.isfinite(value).all():raise ValueError('NONFINITE_POSE')
                xy[i]=value;xm[i]=True
            except ValueError as exc:reason=str(exc)
            v=linear_interpolate(velocities,target_ns=t,tolerance_ns=int(round(tolerance_ms*1e6)))
            if v.valid and np.isfinite(v.value):
                if intervention_ns is None or max(v.source_stamps_ns)<intervention_ns:
                    velocity[i]=v.value;vm[i]=True
        endpoints.append(ep);reasons.append(reason)
    interval=xm & np.r_[True,xm[:-1]]
    return TimeTeacher(xy,xm,velocity,vm,interval,tuple(endpoints),tuple(reasons))
