"""Check interpolated vehicle footprints between GNSS samples of a closed episode."""
from __future__ import annotations
import json
import math
from pathlib import Path
import sys
import numpy as np
if __package__:
    from .geometry import convex_overlap, vehicle_polygon
else:
    from geometry import convex_overlap, vehicle_polygon


def verify(root: Path, episode: str, vehicle: int | None = None) -> dict:
    output=root/'episodes'/episode
    if vehicle is not None and vehicle not in range(1, 5):
        raise ValueError('Vehicle must be in 1..4')
    suffix=f'-d{vehicle}' if vehicle is not None else ''
    rows=np.genfromtxt(output/f'track{suffix}.csv',delimiter=',',names=True)
    zone=np.asarray(json.loads((root/'calibration.json').read_text())['ot_lane_polygon_map_m'])
    low,high=zone.min(axis=0),zone.max(axis=0)
    overlaps=0;tested=0;duration=0.0
    for first,second in zip(rows,rows[1:]):
        distance=math.hypot(second['x_m']-first['x_m'],second['y_m']-first['y_m'])
        angle=math.remainder(second['yaw_rad']-first['yaw_rad'],2*math.pi)
        count=max(1,math.ceil(distance/.05),math.ceil(abs(angle)/.01))
        if count>200:raise ValueError('Unexpected trajectory jump')
        for index in range(count):
            alpha=index/count
            x=first['x_m']+alpha*(second['x_m']-first['x_m'])
            y=first['y_m']+alpha*(second['y_m']-first['y_m'])
            body=vehicle_polygon(float(x),float(y),float(first['yaw_rad']+alpha*angle))
            tested+=1
            if np.any(body.max(axis=0)+.1<low) or np.any(high+.1<body.min(axis=0)):continue
            if convex_overlap(body,zone,.1):
                overlaps+=1;duration+=(second['stamp_s']-first['stamp_s'])/count
    result={'episode':episode+suffix,'interpolation_max_translation_m':.05,'interpolation_max_yaw_rad':.01,
            'margin_m':.1,'samples_tested':tested,'overlap_samples':overlaps,'overlap_s':float(duration),
            'passed':overlaps==0,
            'limit':'Interpolated measured GNSS XY and EKF yaw, not an additional simulator collision sensor'}
    (output/f'dense_check{suffix}.json').write_text(json.dumps(result,indent=2))
    return result


if __name__=='__main__':print(json.dumps(verify(Path(sys.argv[1]),sys.argv[2])))
