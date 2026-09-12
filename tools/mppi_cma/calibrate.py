"""Reproduce coordinate calibration from a closed stationary probe episode."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import numpy as np
from geometry import fit_similarity


def calibrate(root: Path, episode: str, output: Path) -> None:
    probe=root/'episodes'/episode
    config=json.loads((probe/'config.json').read_text())
    result=json.loads((probe/'runtime_result.json').read_text())
    if not result['ok'] or result['reason']!='calibration_observed':
        raise ValueError('Probe did not complete')
    targets=[]
    for i in range(1,5):
        observed=np.array(result['observations']['d'+str(i)][-20:])
        center=observed.mean(axis=0)
        if len(observed)!=20 or np.linalg.norm(observed-center,axis=1).max()>.01:
            raise ValueError('Calibration probe vehicle not stationary')
        targets.append(center)
    fit=fit_similarity(np.asarray(config['calibration_points']),np.asarray(targets))
    if fit['max_residual_m']>.02 or abs(fit['scale']-1)>.001:
        raise ValueError('Coordinate calibration residual/scale limit exceeded')
    region=json.loads((root/'snapshot/ot_lane_region.json').read_text())
    fit['ot_lane_polygon_map_m']=(np.asarray(region['unity_world']['polygon'])@
                                  np.asarray(fit['matrix']).T+fit['translation']).tolist()
    fit['source']='Frozen AWSIM placements and cross-domain V2X, 20 stationary observations per point'
    with output.open('x') as stream:json.dump(fit,stream,indent=2)
    print(json.dumps(fit))


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('root',type=Path);parser.add_argument('episode');parser.add_argument('output',type=Path)
    args=parser.parse_args();calibrate(args.root,args.episode,args.output)
