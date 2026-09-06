"""Offline-only frozen-path contracts. No model, teacher, ROS or actuator API."""
from dataclasses import dataclass
import hashlib
import json
import numpy as np


def sha(blob: bytes) -> str:
    return hashlib.sha256(blob).hexdigest()


def plain(value):
    if isinstance(value, np.ndarray): return plain(value.tolist())
    if isinstance(value, np.generic): return plain(value.item())
    if isinstance(value, float) and not np.isfinite(value): return None
    if isinstance(value, dict): return {str(k): plain(v) for k,v in value.items()}
    if isinstance(value, (tuple,list)): return [plain(v) for v in value]
    return value


def canonical(value) -> bytes:
    return json.dumps(plain(value),sort_keys=True,separators=(',',':'),allow_nan=False).encode()


@dataclass(frozen=True)
class SpatialPathCandidate:
    raw_xy: np.ndarray  # original float32 [N,2], never modified
    nominal_s: np.ndarray
    source_id: str
    frame: str = 'base_link@t_obs'
    t_obs: int | None = None
    units: str = 'm'

    @property
    def raw_hash(self) -> str:
        return sha(self.raw_xy.tobytes(order='C'))


@dataclass
class PreparedPath:
    world_xy: np.ndarray
    actual_s: np.ndarray
    source_indices: np.ndarray  # -1 is separate known origin
    heading: np.ndarray
    curvature: np.ndarray
    diagnostics: dict
    reason: str | None


@dataclass
class TimedReference:
    time: np.ndarray
    xy: np.ndarray
    yaw: np.ndarray
    speed: np.ndarray
    acceleration: np.ndarray
    s: np.ndarray
    indices: np.ndarray
    caps: dict
    endpoint_s: float
    duration: float


@dataclass
class MpcResult:
    states: np.ndarray
    controls: np.ndarray
    first_control: np.ndarray
    accepted: bool
    diagnostics: dict
