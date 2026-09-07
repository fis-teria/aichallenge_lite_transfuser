"""Frozen accepted length-fit 41 references: identical offline PP, no refit."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import numpy as np
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from aic_transfuser_lite.control.spatial_tracking_contracts_v4 import PreparedPath
from aic_transfuser_lite.evaluation.spatial_pure_pursuit_v4 import evaluate


def prepared(record: dict) -> PreparedPath:
    """Only accepted saved geometry, XY[N,2] metres in observation base frame."""
    d=record['diagnostics'];xy=np.asarray(record['reference_xy_m'],dtype=float)
    s=np.asarray(d['reference_actual_s_m'],dtype=float);yaw=np.asarray(d['reference_yaw_rad'],dtype=float)
    if (record['new_reason'] is not None or d['length_policy']!='ENDPOINT_NORMALIZED_LENGTH_V1' or
        d['reference_frame']!='base_link@t_obs' or not d['fit_success'] or
        not np.isfinite(d['maximum_deviation_bound_m']) or d['maximum_deviation_bound_m']>.1 or
        s.ndim!=1 or len(s)<2 or xy.shape!=(len(s),2) or yaw.shape!=s.shape or
        not np.isfinite(xy).all() or not np.isfinite(s).all() or not np.isfinite(yaw).all() or
        s[0]!=0 or np.any(np.diff(s)<=0)):
        raise ValueError('INVALID_SAVED_ACCEPTED_REFERENCE')
    actual=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(xy,axis=0),axis=1))]
    if not np.allclose(s,actual,rtol=0,atol=1e-10):raise ValueError('ACTUAL_ARCLENGTH_MISMATCH')
    return PreparedPath(xy,s,np.full(len(s),-1),yaw[:-1],np.zeros(len(s)),d,None)


def main() -> None:
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--input',type=Path,required=True)
    ap.add_argument('--baseline',type=Path,required=True);ap.add_argument('--configs',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    index_blob=(args.input/'summary.json').read_bytes();index=json.loads(index_blob)
    baseline_blob=args.baseline.read_bytes();baseline=json.loads(baseline_blob)
    if hashlib.sha256(index_blob).hexdigest()!='0b85d2cd459ce5c608feb2fe6e7a9a779682b055445c88ae2e5655abd43da659':raise ValueError('INDEX_HASH')
    if hashlib.sha256(baseline_blob).hexdigest()!='8ca6e490f99cbc86ef57aa45246d4a1c3d4a890b2230d75aa7c4849591cbb927':raise ValueError('BASELINE_HASH')
    if len(index['cases'])!=41 or any(c['reason'] is not None for c in index['cases']):raise ValueError('EXPECTED_FIXED_41')
    old={(r['run'],r['forward_id']):r for r in baseline['records']}
    loaded=[];hashes={}
    for i,entry in enumerate(index['cases']):
        name=f'case_{i:02d}.json';blob=(args.input/name).read_bytes();r=json.loads(blob)
        key=(entry['run'],entry['forward_id'])
        if (r['run'],r['forward_id'])!=key or r['raw_sha256']!=old[key]['raw_sha256'] or old[key]['new_reason'] is None:
            raise ValueError('CASE_IDENTITY_MISMATCH')
        d=r['diagnostics'];original=old[key]['new_diagnostics']
        if any(d[k]!=original[k] for k in ('raw_bits_hex','used_raw_indices','unused_tail_indices','initial_rear_in_base')):
            raise ValueError('SAVED_RAW_OR_PREFIX_CHANGED')
        config_blob=(args.configs/r['run']/'resolved_config.yaml').read_bytes()
        if hashlib.sha256(config_blob).hexdigest()!=baseline['input_hashes'][r['run']]['resolved_config.yaml']:
            raise ValueError('CONFIG_HASH')
        hashes[name]=hashlib.sha256(blob).hexdigest()
        loaded.append((r,prepared(r),yaml.safe_load(config_blob)))
    args.output.mkdir(parents=True,exist_ok=False);summaries=[]
    for i,(record,path,cfg) in enumerate(loaded):
        before=path.world_xy.tobytes()
        result=evaluate(path,np.asarray(record['diagnostics']['initial_rear_in_base']),cfg)
        assert path.world_xy.tobytes()==before
        result.update(run=record['run'],forward_id=record['forward_id'],source_case=f'case_{i:02d}.json',
            raw_sha256=record['raw_sha256'],reference_xy_sha256=hashlib.sha256(before).hexdigest())
        (args.output/f'trial_{i:02d}.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
        summaries.append({k:v for k,v in result.items() if k!='rows'})
    for name,expected in hashes.items():
        if hashlib.sha256((args.input/name).read_bytes()).hexdigest()!=expected:raise ValueError('INPUT_CHANGED_DURING_TEST')
    value=dict(source_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        input_index_sha256=hashlib.sha256(index_blob).hexdigest(),case_sha256=hashes,trials=summaries,
        fixed_trials=41,parameter_search=False,new_inference=0,reference_fit=0,mpc_solves=0,
        simulator_starts=0,control_publish=0,all_pass=all(r['tracking_stop_pass'] for r in summaries))
    (args.output/'summary.json').write_text(json.dumps(value,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps(dict(trials=41,passed=sum(r['tracking_stop_pass'] for r in summaries),
        max_cross_track_m=max(r['max_cross_track_m'] for r in summaries),
        max_goal_distance_m=max(r['goal_distance_m'] for r in summaries),
        all_stopped=all(r['stopped_after_motion'] for r in summaries),
        violation_trials=sum(bool(r['violations']) for r in summaries),source_commit=value['source_commit']),indent=2))


if __name__=='__main__':main()
