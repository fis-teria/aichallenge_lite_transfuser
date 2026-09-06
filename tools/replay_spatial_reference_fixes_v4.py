"""One bounded offline fit per saved stable V4 output. No model/data/ROS."""
from __future__ import annotations
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
import yaml
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from aic_transfuser_lite.control.constrained_reference_v4 import constrained_reference, ordered_polyline_error
from aic_transfuser_lite.control.spatial_tracking_contracts_v4 import SpatialPathCandidate, plain


def main() -> None:
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    names=('run_af1c7f1_03','run_b7a3fae_02','run_d41022c_01')
    inputs={}; records=[]
    for name in names:
        root=args.input/name
        log=(root/'worker.jsonl').read_bytes(); config=(root/'resolved_config.yaml').read_bytes()
        inputs[name]={key:hashlib.sha256(blob).hexdigest() for key,blob in [('worker.jsonl',log),('resolved_config.yaml',config)]}
        cfg=yaml.safe_load(config)
        for line in log.splitlines():
            r=json.loads(line)
            if r.get('event')=='CYCLE' and r.get('stable_history') and 'raw_xy_m' in r:
                records.append((name,cfg,r))
    if len(records)!=46: raise ValueError('EXPECTED_FIXED_46_STABLE_OUTPUTS')
    args.output.mkdir(parents=True,exist_ok=False)
    results=[]
    for name,cfg,r in records:
        raw=np.asarray(r['raw_xy_m'],dtype='<f4')
        if raw.tobytes().hex()!=r['raw_bits_hex'] or hashlib.sha256(raw.tobytes()).hexdigest()!=r['raw_sha256']:
            raise ValueError('SAVED_RAW_IDENTITY_MISMATCH')
        old=r['reference']['diagnostics']
        candidate=SpatialPathCandidate(raw,np.asarray(old['nominal_s_m']),r['forward_id'])
        path=constrained_reference(candidate,cfg,np.asarray(old['initial_rear_in_base']))
        d=path.diagnostics
        residual_xy=None
        if 'steering_knots_rad' in d:
            q=np.asarray(d['ordered_parameter_m']); ds=np.diff(q); z=np.asarray(d['initial_rear_in_base'])
            delta=np.interp((q[1:]+q[:-1])/2,np.linspace(0,q[-1],7),d['steering_knots_rad'])
            dyaw=ds*np.tan(delta)/cfg['wheelbase_m']; yaw=z[2]+np.r_[0.,np.cumsum(dyaw)]
            angle=yaw[:-1]+dyaw/2
            xy=z[:2]+np.vstack([np.zeros(2),np.cumsum(ds[:,None]*np.c_[np.cos(angle),np.sin(angle)],axis=0)])
            used=d['used_raw_indices']; connection=d['initial_connection_length_m']
            ts=np.r_[0.,connection+np.asarray(d['raw_actual_s_m'])[used]]; target=np.vstack([z[:2],raw[used]])
            if connection<1e-9: ts=ts[1:];target=target[1:]
            _,difference=ordered_polyline_error(q,xy,ts,target)
            maximum=float(np.linalg.norm(difference,axis=1).max())
            if not np.isclose(maximum+1e-9,d['maximum_deviation_bound_m'],atol=1e-12,rtol=0):
                raise ValueError('INDEPENDENT_CERTIFICATE_MISMATCH')
            residual_xy=dict(max_abs_dx_m=float(abs(difference[:,0]).max()),max_abs_dy_m=float(abs(difference[:,1]).max()))
        assert candidate.raw_xy.tobytes().hex()==r['raw_bits_hex']
        results.append(dict(run=name,forward_id=r['forward_id'],raw_sha256=r['raw_sha256'],
            old_reason=r['reference']['reason'],old_bound_m=old['maximum_deviation_bound_m'],
            old_exact_sample_max_m=max(old['ordered_error_samples_m']),new_reason=path.reason,
            new_diagnostics=d,components=residual_xy,new_reference_xy_m=path.world_xy.tolist()))
    value=plain(dict(schema='V4_FIXED_OUTPUT_REFERENCE_REPAIR_REPLAY_V1',input_hashes=inputs,
        fixed_output_count=46,additional_fit_calls=sum(r['new_diagnostics']['fit_solve_count'] for r in results),
        new_inference=0,mpc_calls=0,training_steps=0,simulator_starts=0,control_publish=0,
        deviation_limit_m=.1,clearance_or_motion_permission=False,
        reasons=dict(Counter(r['new_reason'] or 'GEOMETRY_ACCEPTED_NOT_RUN' for r in results)),records=results))
    (args.output/'replay.json').write_text(json.dumps(value,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in value.items() if k!='records'},indent=2))


if __name__=='__main__':main()
