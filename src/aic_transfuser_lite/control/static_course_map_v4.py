"""Read-only static raster interpretation; never grants runtime motion permission."""
from __future__ import annotations
from dataclasses import dataclass
import hashlib
from pathlib import Path
import numpy as np
from PIL import Image
import yaml

UNKNOWN, FREE, OCCUPIED = -1, 0, 100


@dataclass(frozen=True)
class StaticCourseMap:
    """Image-row-order int8 grid. Origin is bottom-left XY m/yaw rad.

    Frame is the YAML map frame, NOT the V4 spawn-local or ego frame.
    Pixel centre (col,row) maps to ((col+.5)*res,(height-row-.5)*res)
    before rotation and translation. Out-of-bounds stays UNKNOWN, not clipped.
    """
    grid: np.ndarray
    resolution_m: float
    origin_xy_yaw: tuple[float, float, float]
    provenance: dict

    def query(self, xy_m: np.ndarray) -> np.ndarray:
        """Query float[N,2] in this map's exact frame; returns int8[N]."""
        xy = np.asarray(xy_m, dtype=float)
        if xy.ndim != 2 or xy.shape[1] != 2 or not np.isfinite(xy).all():
            raise ValueError('MAP_QUERY_SHAPE_OR_FINITE')
        x, y, yaw = self.origin_xy_yaw
        c, s = np.cos(yaw), np.sin(yaw)
        local = (xy-np.array([x,y])) @ np.array([[c,-s],[s,c]])
        h,w = self.grid.shape
        inside = ((local[:,0]>=0)&(local[:,1]>=0)&
                  (local[:,0]<w*self.resolution_m)&(local[:,1]<h*self.resolution_m))
        result = np.full(len(xy), UNKNOWN, dtype=np.int8)
        col = np.floor(local[inside,0]/self.resolution_m).astype(int)
        row = h-1-np.floor(local[inside,1]/self.resolution_m).astype(int)
        result[inside] = self.grid[row,col]
        return result


def classify(pixels: np.ndarray, *, negate: int, free_thresh: float, occupied_thresh: float) -> np.ndarray:
    """uint8[H,W], strict occupancy thresholds. Equality is UNKNOWN."""
    p = np.asarray(pixels)
    if p.ndim != 2 or not p.size or p.dtype != np.uint8:
        raise ValueError('MAP_EXPECTS_UINT8_GRAYSCALE')
    if (type(negate) is not int or negate not in (0,1) or
            not np.isfinite([free_thresh,occupied_thresh]).all() or
            not 0 <= free_thresh < occupied_thresh <= 1):
        raise ValueError('INVALID_MAP_THRESHOLDS')
    probability = p.astype(float)/255.
    if not negate: probability = 1.-probability
    grid = np.full(p.shape, UNKNOWN, dtype=np.int8)
    grid[probability < free_thresh] = FREE
    grid[probability > occupied_thresh] = OCCUPIED
    grid.flags.writeable = False
    return grid


def load_map(yaml_path: Path) -> StaticCourseMap:
    """Read only explicit <=64KiB YAML and its sibling <=16MiB PGM, <=4M pixels."""
    if yaml_path.stat().st_size > 65536: raise ValueError('MAP_METADATA_BUDGET')
    metadata_bytes = yaml_path.read_bytes()
    metadata = yaml.safe_load(metadata_bytes)
    name = metadata['image']
    if not isinstance(name,str) or Path(name).name != name or '/' in name or '\\' in name:
        raise ValueError('ONLY_EXPLICIT_SIBLING_MAP_IMAGE')
    image_path = yaml_path.parent/name
    if image_path.suffix.lower() != '.pgm' or image_path.resolve().parent != yaml_path.resolve().parent:
        raise ValueError('MAP_IMAGE_SCOPE')
    if image_path.stat().st_size > 16*1024**2: raise ValueError('MAP_IMAGE_BUDGET')
    image_bytes = image_path.read_bytes()
    from io import BytesIO
    with Image.open(BytesIO(image_bytes)) as image:
        if image.mode != 'L' or image.width*image.height > 4_000_000:
            raise ValueError('MAP_IMAGE_MODE_OR_PIXEL_BUDGET')
        pixels = np.asarray(image).copy()
    resolution = metadata['resolution']; origin = np.asarray(metadata['origin'],dtype=float)
    if (isinstance(resolution,bool) or not np.isfinite(resolution) or resolution<=0 or
            origin.shape!=(3,) or not np.isfinite(origin).all()):
        raise ValueError('MAP_METRIC_METADATA')
    if metadata.get('mode','trinary') != 'trinary': raise ValueError('UNSUPPORTED_MAP_MODE')
    grid = classify(pixels,negate=metadata['negate'],free_thresh=metadata['free_thresh'],
                    occupied_thresh=metadata['occupied_thresh'])
    proof = dict(yaml_path=str(yaml_path), image_path=str(image_path),
        yaml_sha256=hashlib.sha256(metadata_bytes).hexdigest(),
        image_sha256=hashlib.sha256(image_bytes).hexdigest(), metadata=metadata,
        frame='MAP_YAML_XY_CRS_UNVERIFIED', runtime_frame_transform='MISSING',
        awsim_geometry_match='UNKNOWN', semantic_object_types='UNKNOWN',
        dynamic_obstacles_covered=False, runtime_permission=False,
        modifications='NONE: no holes filled, inflation, erosion, or threshold tuning')
    return StaticCourseMap(grid,float(resolution),tuple(origin.tolist()),proof)
