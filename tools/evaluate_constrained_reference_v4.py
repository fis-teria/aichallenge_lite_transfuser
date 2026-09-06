"""One fixed-12 geometry-only pass; no inference/ROS/Dataset/control loop."""
from __future__ import annotations
import argparse
import io
import json
import os
from pathlib import Path
import sys
import time
import zipfile
import numpy as np
import yaml

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from aic_transfuser_lite.control.constrained_reference_v4 import constrained_reference
from aic_transfuser_lite.control.spatial_tracking_contracts_v4 import SpatialPathCandidate,canonical,sha,plain

FIXED_ROWS=(0,20,41,58,78,100,125,123,162,156,26,2)
ENTRY='spatial_v4_validation_review_20260906_153a22a/evidence/validation_predictions.npz'
EXPECTED='d033997378bdcb9fa0cfc7efb038632ee840a0057561b43142ba75b53ee3d90f'


def main() -> int:
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--packet',type=Path,required=True)
    ap.add_argument('--config',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    cfg_blob=args.config.read_bytes(); cfg=yaml.safe_load(cfg_blob)
    with zipfile.ZipFile(args.packet) as z:
        if z.namelist().count(ENTRY)!=1 or z.getinfo(ENTRY).file_size>2_000_000:
            raise ValueError('PACKET_ENTRY')
        blob=z.read(ENTRY)
    if sha(blob)!=EXPECTED: raise ValueError('PREDICTION_HASH')
    with np.load(io.BytesIO(blob),allow_pickle=False) as arrays:
        xy=arrays['xy']; ids=arrays['sample_ids']; processed=arrays['processed']
    if xy.shape!=(180,20,2) or xy.dtype!=np.float32 or len(ids)!=180 or not processed[list(FIXED_ROWS)].all():
        raise ValueError('ARRAY_CONTRACT')
    rows=[]; start=time.monotonic()
    for i in FIXED_ROWS:
        candidate=SpatialPathCandidate(xy[i].copy(),np.arange(1,21)/10,str(ids[i]))
        path=constrained_reference(candidate,cfg,np.zeros(5))
        row=dict(row=i,input_id=str(ids[i]),raw_xy=candidate.raw_xy,reason=path.reason,
                 reference_xy=path.world_xy,reference_actual_s=path.actual_s,
                 diagnostics=path.diagnostics,initial_pose_basis='OFFLINE_SYNTHETIC_BASE_EQUALS_REAR_NOT_LIVE',
                 source_commit=os.environ.get('V4_SOURCE_COMMIT','UNKNOWN'))
        (args.output/f'row_{i}.json').write_bytes(canonical(row))
        rows.append(dict(row=i,reason=path.reason,bound_m=path.diagnostics.get('maximum_deviation_bound_m'),
                         length_m=path.diagnostics.get('usable_prefix_s_m'),fit_wall_s=path.diagnostics.get('fit_wall_s'),
                         fit_solves=path.diagnostics['fit_solve_count']))
    summary=dict(scope='FIXED_SAVED_GEOMETRY_ONLY',fixed_rows=FIXED_ROWS,rows=rows,
                 geometry_accepted=sum(r['reason'] is None for r in rows),input_count=12,
                 config_sha256=sha(cfg_blob),prediction_sha256=sha(blob),elapsed_wall_s=time.monotonic()-start,
                 reference_optimizer_solves=sum(r['fit_solves'] for r in rows),
                 mpc_solves=0,control_cycles=0,fixed_forwards=0,live_control_calls=0,
                 source_commit=os.environ.get('V4_SOURCE_COMMIT','UNKNOWN'),
                 geometry_acceptance_is_clearance_or_mpc_or_driving_success=False)
    (args.output/'summary.json').write_bytes(canonical(summary))
    print(json.dumps(plain(summary),indent=2))
    return 0


if __name__=='__main__': raise SystemExit(main())
