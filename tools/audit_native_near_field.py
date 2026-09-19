"""Audit observed native teacher and saved predictions at 1/2/3 m forward stations."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from aic_transfuser_lite.data.time_native_replay_v1 import NativeReplayDataset
from aic_transfuser_lite.data.time_training_cache_v1 import _sha


def stations(xy: np.ndarray) -> list[float] | None:
    """First monotone forward crossing at x=1,2,3 m; no extrapolation."""
    end = np.flatnonzero(xy[:, 0] >= 3.)
    if not len(end):
        return None
    prefix = xy[:int(end[0]) + 1]
    if len(prefix) < 2 or prefix[0, 0] > 1. or np.any(np.diff(prefix[:, 0]) <= 0):
        return None
    return np.interp([1., 2., 3.], prefix[:, 0], prefix[:, 1]).tolist()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = args.root
    prep = root / 'runs/time_native701_prepare_20260919/preparation'
    proof = json.loads((prep / 'proof.json').read_text())
    native = NativeReplayDataset(prep, manifest_sha256=proof['native_manifest_sha256'])
    prior = root / 'runs/native701_retention_losses_20260919'
    predictions = np.load(prior / 'retained_step0512_native_predictions.npy')
    assert predictions.shape == (len(native), 30, 2) and np.isfinite(predictions).all()
    geometry_file = root / 'runs/avoidance_diagnosis_20260918/training_windows.jsonl'
    assert _sha(geometry_file) == '25673a909c39d7c77b6385a8d127a9bb7efcc63c0f2abc00dc8ab1684925771c'
    geometry = {r['anchor_id']: r['cone_body_xy_m'] for r in map(json.loads, geometry_file.read_text().splitlines())}
    geometry_hashes = {str(geometry_file.relative_to(root)): _sha(geometry_file)}
    for folder in ('front_validation_v1', 'collect10_validation_v1'):
        for path in (root / 'runs/mppi_v45_pc10_20260918' / folder).glob('*-geometry.json'):
            geometry_hashes[str(path.relative_to(root))] = _sha(path)
            for row in json.loads(path.read_text()):
                assert row['anchor_id'] not in geometry
                geometry[row['anchor_id']] = row['object_body_xy_m']
    rows = []
    for i in range(len(native)):
        sample = native[i]
        teacher = stations(sample.teacher.xy_m)
        prediction = stations(predictions[i])
        x, y = geometry[sample.anchor_id]
        rows.append(dict(index=i, anchor_id=sample.anchor_id, run_id=sample.run,
            use=native.uses[i], object_body_xy_m=[x,y], speed_mps=float(sample.inputs.ego[0,-1,0]),
            teacher_y_m=teacher, prediction_y_m=prediction,
            teacher_points_1_3_m=int(((sample.teacher.xy_m[:,0] >= 1.) & (sample.teacher.xy_m[:,0] <= 3.)).sum())))
    groups = {}
    for name, group in {
        'front': [r for r in rows if r['use']=='static_cone_xy_speed' and r['object_body_xy_m'][0]>0],
        'front_8m_width1p5': [r for r in rows if r['use']=='static_cone_xy_speed' and 0<r['object_body_xy_m'][0]<=8 and abs(r['object_body_xy_m'][1])<=1.5],
        'front_6m_width1': [r for r in rows if r['use']=='static_cone_xy_speed' and 0<r['object_body_xy_m'][0]<=6 and abs(r['object_body_xy_m'][1])<=1],
    }.items():
        supported = [r for r in group if r['teacher_y_m'] is not None and r['prediction_y_m'] is not None]
        t = np.array([r['teacher_y_m'] for r in supported]).reshape(-1,3)
        p = np.array([r['prediction_y_m'] for r in supported]).reshape(-1,3)
        groups[name] = dict(anchors=len(group), station_supported=len(supported), runs=len({r['run_id'] for r in group}),
            teacher_abs_y_m_mean=np.abs(t).mean(0).tolist() if len(t) else None,
            prediction_abs_y_m_mean=np.abs(p).mean(0).tolist() if len(t) else None,
            lateral_mae_m=np.abs(p-t).mean(0).tolist() if len(t) else None,
            teacher_abs_y_ge_0p1_counts=(np.abs(t)>=.1).sum(0).tolist(),
            teacher_abs_y_ge_0p2_counts=(np.abs(t)>=.2).sum(0).tolist(),
            opposite_sign_counts=((p*t<0)&(np.abs(t)>=.1)).sum(0).tolist())
    result = dict(native_manifest_sha256=proof['native_manifest_sha256'],
        prediction_sha256=_sha(prior/'retained_step0512_native_predictions.npy'),
        geometry_hashes=geometry_hashes, stations_x_m=[1.,2.,3.], groups=groups, rows=rows,
        scope='train-only recorded teacher geometry; not collision clearance or heldout performance')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
    print(json.dumps(groups, indent=2))


if __name__ == '__main__':
    main()
