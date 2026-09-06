"""Bounded live-sim synchronization, no ROS or sensor/asset readers.

Camera grid 100 ms, nearest LiDAR <=30 ms, bracketing ego <=50 ms:
the existing converter's scalar/angle interpolation, not a receipt-time label.
GNSS/IMU are separately joined for diagnostic planar pose; no heading CSV.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math
import queue
import time
import numpy as np

from aic_transfuser_lite.data.synchronization_v3 import (
    TimedValue, nearest, linear_interpolate, angle_interpolate,
)
from .spatial_input_v4 import GridObservation, Stamp


@dataclass(frozen=True)
class Sample:
    ns: int
    received_ns: int
    value: object
    frame: str
    epoch: str


def interpolate(samples: list[Sample], target_ns: int, *, angle_columns: tuple[int, ...] = (),
                tolerance_ns: int = 50_000_000) -> tuple[np.ndarray, dict]:
    """Finite vector from received bracketing values, never extrapolate."""
    if not samples:
        raise ValueError('EMPTY_STREAM')
    if len({s.epoch for s in samples}) != 1:
        raise ValueError('MIXED_EPOCH')
    values = np.asarray([s.value for s in samples], dtype=float)
    if values.ndim != 2 or not np.isfinite(values).all():
        raise ValueError('INVALID_VECTOR_STREAM')
    result = []
    source_ns: set[int] = set()
    for col in range(values.shape[1]):
        stream = [TimedValue(s.ns, float(v)) for s, v in zip(samples, values[:, col])]
        fn = angle_interpolate if col in angle_columns else linear_interpolate
        r = fn(stream, target_ns=target_ns, tolerance_ns=tolerance_ns)
        if not r.valid:
            raise ValueError('INTERPOLATION_' + r.reason)
        result.append(r.value)
        source_ns.update(r.source_stamps_ns)
    sources = [s for s in samples if s.ns in source_ns]
    return np.asarray(result), dict(method='CONVERTER_V3_BRACKET', source_stamps_ns=sorted(source_ns),
                                   source_received_ns=[s.received_ns for s in sources],
                                   available_ns=max(s.received_ns for s in sources), epoch=samples[0].epoch)


def align_observation(camera: Sample, lidar: list[Sample], velocity: list[Sample],
                      steering: list[Sample], *, cutoff_ns: int, grid_phase_ns: int) -> tuple[GridObservation, dict]:
    """RGB uint8 HWC + exact 750-beam scan + ego [vx,vy,wz,delta] SI."""
    streams = [lidar, velocity, steering]
    if camera.received_ns > cutoff_ns or any(s.received_ns > cutoff_ns or s.epoch != camera.epoch
                                             for stream in streams for s in stream):
        raise ValueError('UNAVAILABLE_OR_EPOCH_MISMATCH')
    grid_ns = grid_phase_ns + round((camera.ns-grid_phase_ns)/100_000_000)*100_000_000
    if abs(camera.ns-grid_ns) > 40_000_000:
        raise ValueError('CAMERA_GRID_TOLERANCE')
    match = nearest([TimedValue(s.ns, s) for s in lidar], target_ns=camera.ns, tolerance_ns=30_000_000)
    if not match.valid:
        raise ValueError('LIDAR_' + match.reason)
    scan = match.value
    geo = scan.value
    actual = np.array([geo['angle_min'], geo['angle_increment'], geo['range_min'], geo['range_max']])
    expected = np.array([-1.5666074752807617, .004188789986073971, 0., 25.])
    if len(geo['ranges']) != 750 or not np.allclose(actual, expected, rtol=0, atol=1e-6):
        raise ValueError('UNSUPPORTED_LIDAR_GEOMETRY')
    if camera.frame != 'camera_optical_link' or scan.frame != 'lidar':
        raise ValueError('SENSOR_FRAME_MISMATCH')
    ego, ev = interpolate(velocity, camera.ns)
    delta, es = interpolate(steering, camera.ns, angle_columns=(0,))
    def stamp(s: Sample, available: int | None = None) -> Stamp:
        return Stamp(s.ns, s.received_ns, s.received_ns if available is None else available,
                     'AWSIM_ROS', camera.epoch, 'HOST_MONOTONIC', acquisition_ns=None)
    available = max(ev['available_ns'], es['available_ns'])
    aligned = Stamp(camera.ns, available, available, 'AWSIM_ROS', camera.epoch, 'HOST_MONOTONIC')
    item = GridObservation(grid_ns, stamp(camera), stamp(scan), aligned, camera.value,
                           np.asarray(geo['ranges'], dtype=np.float32), tuple(np.r_[ego, delta]),
                           sample_id=f'{camera.epoch}:camera:{camera.ns}')
    return item, dict(velocity=ev, steering=es, camera=asdict(stamp(camera)), lidar=asdict(stamp(scan)),
                      grid_ns=grid_ns, grid_phase_ns=grid_phase_ns, cutoff_ns=cutoff_ns,
                      scan_geometry=actual.tolist(), training_lidar_geometry_parity='NOT_FULLY_PROVEN')


def quaternion_yaw(q: list[float]) -> float:
    """ROS quaternion [x,y,z,w] to yaw, reject invalid/non-planar orientation."""
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1.) > .01:
        raise ValueError('INVALID_QUATERNION')
    x, y, z, w = q/np.linalg.norm(q)
    roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
    pitch = math.asin(float(np.clip(2*(w*y-z*x), -1, 1)))
    if max(abs(roll), abs(pitch)) > .15:
        raise ValueError('NONPLANAR_VEHICLE')
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def base_pose_from_gnss_imu(gnss_utm_xy: np.ndarray, imu_q: list[float]) -> np.ndarray:
    """Selected immutable scene: GNSS x=-.26m, IMU local ROS yaw=+pi/2.

    UTM horizontal axes match the simulator MGRS grid (not an ENU yaw swap).
    This uses current sensor pose, never a route-derived heading. GNSS source
    acquisition time is unknown when the publisher stamps after async delay;
    callers MUST retain that limitation and cannot mark frame timing proven.
    """
    xy = np.asarray(gnss_utm_xy, dtype=float)
    if xy.shape != (2,) or not np.isfinite(xy).all():
        raise ValueError('INVALID_GNSS_POSITION')
    yaw = (quaternion_yaw(imu_q)-math.pi/2+math.pi) % (2*math.pi)-math.pi
    return np.r_[xy+.26*np.array([math.cos(yaw), math.sin(yaw)]), yaw]


def join_control_state(bundle: dict, observation_ns: int, scan_ns: int) -> dict:
    """Late control-only join; callers never rebuild or re-forward model input."""
    pose, source = interpolate(bundle['pose'], observation_ns, angle_columns=(2,))
    scan_pose, scan_source = interpolate(bundle['pose'], scan_ns, angle_columns=(2,))
    current_ns = min(bundle[k][-1].ns for k in ('pose','velocity','steering'))
    current_pose, p = interpolate(bundle['pose'], current_ns, angle_columns=(2,))
    velocity, v = interpolate(bundle['velocity'],current_ns)
    steering, d = interpolate(bundle['steering'],current_ns,angle_columns=(0,))
    return dict(pose_at_observation=pose, pose_provenance=source, scan_pose=scan_pose,
                scan_pose_provenance=scan_source, current_pose=current_pose, current_ns=current_ns,
                velocity=velocity, steering=steering[0], current_provenance=dict(pose=p,velocity=v,steering=d))


def await_control_join(original: dict, observation_ns: int, scan_ns: int, deadline_ns: int,
                       read_update, stopped, *, clock=time.monotonic_ns) -> tuple[dict,dict]:
    """Fixed deadline and immutable original input; only state updates are used."""
    current=original
    while True:
        if clock() >= deadline_ns: raise ValueError('POSE_JOIN_EXPIRED')
        if stopped(): raise ValueError('POSE_JOIN_STOPPED')
        try:
            joined=join_control_state(current,observation_ns,scan_ns)
            if clock() >= deadline_ns: raise ValueError('POSE_JOIN_EXPIRED')
            return current,joined
        except (ValueError,IndexError) as exc:
            if str(exc)=='POSE_JOIN_EXPIRED': raise
            try: update=read_update()
            except queue.Empty: continue
            if update['epoch'] != original['epoch']: raise ValueError('POSE_JOIN_EPOCH_RESET')
            # Never permit a state update to replace Camera/LiDAR/command input.
            allowed={k:update[k] for k in ('pose','velocity','steering','cutoff_ns','previous_acceleration')}
            if 'sim_ns' in update: allowed['sim_ns']=update['sim_ns']
            current=dict(original,**allowed)


def stopped_forward_initial_speed(raw_speed: float, samples: list[Sample], *, drive_gear_sent: bool) -> tuple[float,dict]:
    """One sim-only controller state policy; model ego and signed raw stay intact.

    Only numerical negative drift <=1 mm/s after a continuous 1 s observed
    stop band (the existing .03 m/s stop criterion) in requested Drive is zeroed.
    A real backward measurement below -1 mm/s is never converted by abs/clamp.
    """
    proof = dict(raw_signed_mps=float(raw_speed),policy='SIM_STOPPED_DRIVE_NUMERICAL_DRIFT_V1',changed=False)
    recent = [s for s in samples if samples[-1].ns-s.ns <= 1_100_000_000] if samples else []
    continuous = (len(recent)>1 and recent[-1].ns-recent[0].ns >= 1_000_000_000
        and all(0 < b.ns-a.ns <= 100_000_000 for a,b in zip(recent,recent[1:]))
        and all(np.isfinite(s.value[0]) and -.001 <= s.value[0] <= .03 for s in recent))
    if -.001 <= raw_speed < 0 and drive_gear_sent and continuous:
        proof['changed'] = True
        return 0.,proof
    return raw_speed,proof


def close_transport_queues(queues: dict) -> dict:
    """After worker termination: discard pending transport, never join its feeder.

    A no-longer-consumed camera bundle can exceed a pipe buffer and indefinitely
    block Python's exit finalizer. Authoritative records are already in files;
    buffered transport is not a sent control or a new inference.
    """
    result={}
    for name,channel in queues.items():
        errors=[]
        try: pending=channel.qsize()
        except (NotImplementedError,OSError): pending=None
        for action in ('cancel_join_thread','close'):
            try: getattr(channel,action)()
            except Exception as exc: errors.append(type(exc).__name__+': '+str(exc))
        result[name]=dict(pending_estimate=pending,discarded_after_worker_stop=True,errors=errors)
    return result
