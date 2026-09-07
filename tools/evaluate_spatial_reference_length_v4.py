"""Exactly 41 saved rejected outputs, one endpoint-length fit each; offline."""
from __future__ import annotations
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import numpy as np
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from aic_transfuser_lite.control.constrained_reference_v4 import constrained_reference, ordered_polyline_error
from aic_transfuser_lite.control.spatial_tracking_contracts_v4 import SpatialPathCandidate, plain


def main() -> None:
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--input',type=Path,required=True);ap.add_argument('--configs',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    blob=args.input.read_bytes();identity=hashlib.sha256(blob).hexdigest()
    if identity!='8ca6e490f99cbc86ef57aa45246d4a1c3d4a890b2230d75aa7c4849591cbb927':raise ValueError('FIXED_INPUT_HASH')
    saved=json.loads(blob); records=[r for r in saved['records'] if r['new_reason'] is not None]
    if len(records)!=41:raise ValueError('FIXED_REJECTED_41')
    args.output.mkdir(parents=True,exist_ok=False);results=[]
    for i,r in enumerate(records):
        old=r['new_diagnostics'];config_blob=(args.configs/r['run']/'resolved_config.yaml').read_bytes()
        if hashlib.sha256(config_blob).hexdigest()!=saved['input_hashes'][r['run']]['resolved_config.yaml']:raise ValueError('CONFIG_HASH')
        cfg=yaml.safe_load(config_blob); raw_bytes=bytes.fromhex(old['raw_bits_hex'])
        if hashlib.sha256(raw_bytes).hexdigest()!=r['raw_sha256']:raise ValueError('RAW_HASH')
        raw=np.frombuffer(raw_bytes,dtype='<f4').reshape(20,2).copy()
        candidate=SpatialPathCandidate(raw,np.asarray(old['nominal_s_m']),r['forward_id'])
        p=constrained_reference(candidate,cfg,np.asarray(old['initial_rear_in_base']),length_policy='ENDPOINT_NORMALIZED_LENGTH_V1')
        d=p.diagnostics
        assert raw.tobytes()==raw_bytes
        assert d['used_raw_indices']==old['used_raw_indices'] and d['unused_tail_indices']==old['unused_tail_indices']
        independent=None
        if 'steering_knots_rad' in d:
            q=np.asarray(d['ordered_parameter_m']);physical=np.asarray(d['integration_parameter_s_m'])
            z=np.asarray(d['initial_rear_in_base']);delta=np.interp((q[1:]+q[:-1])/2,np.linspace(0,q[-1],7),d['steering_knots_rad'])
            ds=np.diff(physical);dyaw=ds*np.tan(delta)/cfg['wheelbase_m'];yaw=z[2]+np.r_[0.,np.cumsum(dyaw)]
            angle=yaw[:-1]+dyaw/2
            xy=z[:2]+np.vstack([np.zeros(2),np.cumsum(ds[:,None]*np.c_[np.cos(angle),np.sin(angle)],axis=0)])
            used=d['used_raw_indices'];conn=d['initial_connection_length_m']
            ts=np.r_[0.,conn+np.asarray(d['raw_actual_s_m'])[used]];target=np.vstack([z[:2],raw[used]])
            if conn<1e-9:ts=ts[1:];target=target[1:]
            _,diff=ordered_polyline_error(q,xy,ts,target)
            exact=float(np.linalg.norm(diff,axis=1).max())
            assert abs(exact+1e-9-d['maximum_deviation_bound_m'])<1e-10
            rate=np.diff(d['steering_knots_rad'])/(d['integration_length_m']/6)*cfg['maximum_speed_mps']
            independent=dict(bound_m=exact+1e-9,endpoint_error_m=float(np.linalg.norm(xy[-1]-target[-1])),
                max_abs_dx_m=float(abs(diff[:,0]).max()),max_abs_dy_m=float(abs(diff[:,1]).max()),
                max_steering_rate_rad_s=float(abs(rate).max()))
            if p.reason is None:
                assert np.allclose(xy,p.world_xy,atol=1e-10,rtol=0)
                assert exact+1e-9<=.1 and abs(rate).max()<=cfg['steering_rate_limit_rad_s']+1e-8
        result=plain(dict(run=r['run'],forward_id=r['forward_id'],raw_sha256=r['raw_sha256'],
            old_reason=r['new_reason'],old_bound_m=old['maximum_deviation_bound_m'],new_reason=p.reason,
            independent_check=independent,diagnostics=d,reference_xy_m=p.world_xy.tolist()))
        (args.output/f'case_{i:02d}.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
        results.append(dict(run=r['run'],forward_id=r['forward_id'],reason=p.reason,
            old_bound_m=old['maximum_deviation_bound_m'],new_bound_m=d.get('maximum_deviation_bound_m'),
            endpoint_error_m=d.get('endpoint_error_m'),length_scale=d.get('length_scale'),
            old_length_m=old['ordered_parameter_m'][-1],new_length_m=d.get('integration_length_m'),
            fit_wall_s=d.get('fit_wall_s'),fit_iterations=d.get('fit_iterations')))
    summary=dict(source_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),input_sha256=identity,
        parameter_search=False,fit_calls=41,new_inference=0,training_steps=0,mpc_solves=0,controller_trials=0,
        simulator_starts=0,control_publish=0,live_activation=False,default_length_policy_unchanged=True,
        reasons=dict(Counter(r['reason'] or 'GEOMETRY_ACCEPTED_NOT_RUN' for r in results)),cases=results)
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in summary.items() if k!='cases'},indent=2))


if __name__=='__main__':main()
