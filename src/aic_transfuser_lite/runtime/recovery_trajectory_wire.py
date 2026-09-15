"""Bounded CDR1 metadata validation for the pinned Autoware Trajectory IDL.

The collector needs header time, frame, count and every point's target speed.
Avoid creating thousands of nested Python ROS point objects in the receiver
that also handles /clock. The official PP and rosbag retain the full message.
TrajectoryPoint is Duration (8 bytes), Pose (7 float64), then 6 float32; stride
88 bytes, with 8-byte alignment relative to the CDR body after encapsulation.
Unsupported encodings/layouts fail closed, never fall back to a slow decoder.
"""
from __future__ import annotations

from dataclasses import dataclass
import struct

import numpy as np

from aic_transfuser_lite.data.time_recovery_collection_v1 import TARGET_MPS


@dataclass(frozen=True)
class WireStamp:
    sec: int
    nanosec: int


@dataclass(frozen=True)
class WireHeader:
    stamp: WireStamp
    frame_id: str


@dataclass(frozen=True)
class TrajectorySummary:
    header: WireHeader
    point_count: int
    target_speed_valid: bool


def decode_trajectory_summary(blob: bytes) -> TrajectorySummary:
    """Validate CDR1 bytes; read all speeds [N] in m/s without ROS objects.

    Stamp units are integer seconds/nanoseconds. N is bounded by the official
    IDL's 10,000-point capacity. This verifies wire structure and the same speed
    predicate as math.isclose(rel_tol=1e-9, abs_tol=1e-5); the caller still
    enforces map frame, N >= 20, receipt/capture freshness and publisher identity.
    """
    if not isinstance(blob, bytes) or len(blob) < 20:
        raise ValueError('TRAJECTORY_CDR_HEADER')
    if blob[:4] not in (b'\x00\x01\x00\x00', b'\x00\x00\x00\x00'):
        raise ValueError('TRAJECTORY_CDR_ENCODING')
    endian = '<' if blob[1] == 1 else '>'
    sec, nanosec, frame_size = struct.unpack_from(endian+'iII', blob, 4)
    if sec < 0 or nanosec >= 1_000_000_000 or not 1 <= frame_size <= 256:
        raise ValueError('TRAJECTORY_CDR_HEADER')
    end_frame = 16+frame_size
    count_offset = 4+((end_frame-4+3)//4)*4
    if len(blob) < count_offset+4 or blob[end_frame-1] != 0:
        raise ValueError('TRAJECTORY_CDR_HEADER')
    try:
        frame = blob[16:end_frame-1].decode('utf-8', errors='strict')
    except UnicodeDecodeError as exc:
        raise ValueError('TRAJECTORY_CDR_FRAME') from exc
    if '\x00' in frame:
        raise ValueError('TRAJECTORY_CDR_FRAME')
    count = struct.unpack_from(endian+'I', blob, count_offset)[0]
    if not 1 <= count <= 10_000:
        raise ValueError('TRAJECTORY_CDR_COUNT')
    points_offset = 4+((count_offset+7)//8)*8
    if len(blob) != points_offset+count*88:
        raise ValueError('TRAJECTORY_CDR_SIZE')
    speeds = np.ndarray((count,), dtype=endian+'f4', buffer=blob,
                        offset=points_offset+64, strides=(88,)).astype(np.float64)
    assert speeds.shape == (count,)
    tolerance = np.maximum(1e-5, 1e-9*np.maximum(np.abs(speeds), abs(TARGET_MPS)))
    valid = bool(np.all(np.isfinite(speeds) & (np.abs(speeds-TARGET_MPS) <= tolerance)))
    return TrajectorySummary(WireHeader(WireStamp(sec, nanosec), frame), count, valid)
