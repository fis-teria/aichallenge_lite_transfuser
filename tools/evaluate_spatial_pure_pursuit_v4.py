"""Evaluate exactly the five already accepted references; never re-fit or infer."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from aic_transfuser_lite.control.spatial_tracking_contracts_v4 import PreparedPath
from aic_transfuser_lite.evaluation.spatial_pure_pursuit_v4 import evaluate


def main() -> None:
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--input',type=Path,required=True);ap.add_argument('--configs',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    raw=args.input.read_bytes(); digest=hashlib.sha256(raw).hexdigest()
    if digest!='8ca6e490f99cbc86ef57aa45246d4a1c3d4a890b2230d75aa7c4849591cbb927': raise ValueError('FIXED_INPUT_HASH')
    packet=json.loads(raw); accepted=[r for r in packet['records'] if r['new_reason'] is None]
    if len(accepted)!=5: raise ValueError('EXPECTED_FIVE_ACCEPTED')
    args.output.mkdir(parents=True,exist_ok=False); summaries=[]
    for i,r in enumerate(accepted):
        config_raw=(args.configs/r['run']/'resolved_config.yaml').read_bytes()
        if hashlib.sha256(config_raw).hexdigest()!=packet['input_hashes'][r['run']]['resolved_config.yaml']:
            raise ValueError('SAVED_CONFIG_HASH')
        cfg=yaml.safe_load(config_raw);d=r['new_diagnostics'];xy=np.asarray(r['new_reference_xy_m']);s=np.asarray(d['reference_actual_s_m'])
        yaw=np.asarray(d['reference_yaw_rad'])
        path=PreparedPath(xy,s,np.full(len(s),-1),yaw[:-1],np.zeros(len(s)),d,None)
        result=evaluate(path,np.asarray(d['initial_rear_in_base']),cfg)
        result.update(run=r['run'],forward_id=r['forward_id'],raw_sha256=r['raw_sha256'],
            reference_xy_sha256=hashlib.sha256(xy.tobytes()).hexdigest())
        (args.output/f'trial_{i}.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
        summaries.append({k:v for k,v in result.items() if k!='rows'})
    value=dict(input_sha256=digest,selected_by='PREEXISTING_GEOMETRY_ACCEPTANCE_ONLY',trials=summaries,
        source_commit=__import__('subprocess').check_output(['git','rev-parse','HEAD'],text=True).strip(),
        rejected_41_not_retested=True,parameter_search=False,new_inference=0,reference_fit=0,mpc_solves=0,
        simulator_starts=0,control_publish=0,all_pass=all(r['tracking_stop_pass'] for r in summaries))
    (args.output/'summary.json').write_text(json.dumps(value,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps(value,indent=2))


if __name__=='__main__':main()
