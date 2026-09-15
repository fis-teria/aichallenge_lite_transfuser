"""Screen measured-normal lateral excursions; no AWSIM or live control changes.

Coordinates are map metres, course progress metres, and body yaw radians.
The 1.4 m circular map check is an existing planning screen, not proof that a
physical vehicle can follow or stop from a perturbed state. Labels are not made.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
from typing import Any

import numpy as np

from aic_transfuser_lite.data.recovery_reference_v3 import load_occupancy_map_v3
from aic_transfuser_lite.data.time_recovery_collection_v1 import (
    check_collection_decision_age, collection_snapshot_retry_allowed,
)


def read(path: Path) -> Any:
    return json.loads(path.read_bytes())


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def normal_trace(raw: Path, expected_hashes: dict[str, str]) -> np.ndarray:
    """Return the longest measured lap [N,5]: progress, x, y, yaw, speed (SI)."""
    for name, expected in expected_hashes.items():
        assert sha(raw/name) == expected, (raw.name, name)
    result = read(raw/'result.json')
    assert result['status'] == 'COMPLETE_LAP' and not result['last_control']['fault']
    assert result['nodes']['closed_bag'] and result['last_control']['stop_confirmed']
    segments: list[list[dict[str, Any]]] = [[]]
    for row in [json.loads(s) for s in (raw/'control.jsonl').read_bytes().splitlines()]:
        if row.get('reason') != 'RECOVERY_TEACHER_TRACKING' or not row.get('publication') or not row.get('projection'):
            continue
        if segments[-1] and row['projection']['s_m'] < segments[-1][-1]['projection']['s_m']-5.:
            segments.append([])
        segments[-1].append(row)
    unique = {}
    for row in max(segments, key=len):
        p = row['current_pose']; s = row['projection']['s_m']
        unique.setdefault(s, [s, p['x_m'], p['y_m'], p['yaw_rad'], row['speed_mps']])
    values = np.array([unique[s] for s in sorted(unique)], dtype=np.float64)
    assert values.ndim == 2 and values.shape[1] == 5 and len(values) > 1000
    assert np.isfinite(values).all() and np.all(np.diff(values[:, 0]) > 0.)
    values[:, 3] = np.unwrap(values[:, 3])
    return values


def excursion_profile(progress: np.ndarray, release_m: float, offset_m: float,
                      return_length_m: float = 10.) -> np.ndarray:
    """Planning displacement [N] m: 8 m approach, 2 m hold, explicit return."""
    if (progress.ndim != 1 or not np.isfinite(progress).all()
            or not math.isfinite(release_m) or not math.isfinite(offset_m)
            or return_length_m not in (4.,6.,10.)):
        raise ValueError('EXCURSION_FINITE_SHAPE')
    approach = np.clip((progress-(release_m-10.))/8., 0., 1.)
    recovery = np.clip((progress-release_m)/return_length_m, 0., 1.)
    smooth_approach = approach**2*(3.-2.*approach)
    smooth_recovery = recovery**2*(3.-2.*recovery)
    return offset_m*smooth_approach*(1.-smooth_recovery)


def densify_with_progress(progress: np.ndarray, x: np.ndarray, y: np.ndarray,
                          maximum_step_m: float) -> np.ndarray:
    """Preserve each source segment and return dense [N,3] progress/x/y [m]."""
    values = np.column_stack((progress, x, y))
    if (values.ndim != 2 or values.shape[1] != 3 or len(values) < 2
            or not np.isfinite(values).all() or np.any(np.diff(progress) <= 0.)
            or not math.isfinite(maximum_step_m) or maximum_step_m <= 0.):
        raise ValueError('DENSE_EXCURSION_CONTRACT')
    blocks = []
    for a,b in zip(values[:-1],values[1:]):
        parts = max(1,math.ceil(float(np.linalg.norm(b[1:]-a[1:]))/maximum_step_m))
        fraction = np.arange(parts,dtype=float)/parts
        blocks.append(a[None,:]+fraction[:,None]*(b-a)[None,:])
    result = np.concatenate([*blocks,values[-1:]],axis=0)
    assert np.all(np.diff(result[:,0]) > 0.)
    assert np.linalg.norm(np.diff(result[:,1:],axis=0),axis=1).max() <= maximum_step_m+1e-9
    return result


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--return-length-m', type=float, choices=(4.,6.,10.), default=10.)
    args = ap.parse_args()
    assert not args.output.exists()
    inputs = Path('/home/thistle/e2e_autonomous/runs/time_recovery_collection_20260913/inputs')
    prior = Path('/home/thistle/e2e_autonomous/runs/time_recovery_separated_20260915')
    raw_root = Path('/home/thistle/e2e_autonomous/raw/time_recovery_sites_20260915')
    plan = read(prior/'selected_site_plan.json')
    deployment = read(Path('docs/evidence/time_recovery_separated_collection_20260915/final_deployment.json'))
    input_hashes = {}
    for name in ('base.csv', 'occupancy_grid_map.yaml', 'occupancy_grid_map.pgm'):
        digest = sha(inputs/name)
        assert digest == deployment['files']['inputs/'+name], name
        input_hashes[name] = digest
    occupancy = load_occupancy_map_v3(inputs/'occupancy_grid_map.yaml')
    traces = {name: normal_trace(raw_root/name, plan['source_hashes'][name]) for name in plan['normal_runs']}
    # One real-coordinate smoke: zero shift preserves each source exactly, and
    # translating along a unit normal has the requested Euclidean magnitude.
    for values in traces.values():
        normal = np.column_stack((-np.sin(values[:, 3]), np.cos(values[:, 3])))
        np.testing.assert_allclose(values[:, 1:3]+0.*normal, values[:, 1:3], rtol=0., atol=0.)
        np.testing.assert_allclose(np.linalg.norm(.6*normal, axis=1), .6, rtol=0., atol=1e-12)
    try:
        excursion_profile(np.zeros((2, 2)), 100., .6)
    except ValueError as exc:
        assert str(exc) == 'EXCURSION_FINITE_SHAPE'
    else:
        raise AssertionError('INVALID_SHAPE_NOT_REJECTED')
    dense_smoke=densify_with_progress(np.array([0.,1.,2.]),np.array([0.,.2,.2]),np.array([0.,0.,.2]),.05)
    np.testing.assert_allclose(dense_smoke[[0,4,8]],[[0.,0.,0.],[1.,.2,0.],[2.,.2,.2]],rtol=0.,atol=1e-12)
    # Confirm the actual deadline, without changing the runtime policy.
    check_collection_decision_age(started_ns=0, now_ns=100_000_000)
    try:
        check_collection_decision_age(started_ns=0, now_ns=100_000_001)
    except ValueError as exc:
        assert str(exc) == 'COLLECTION_COMPUTATION_TIMEOUT'
    else:
        raise AssertionError('DEADLINE_CONTRACT_DRIFT')
    assert collection_snapshot_retry_allowed('STALE_steering', attempt=0, elapsed_ns=80_000_000)
    assert not collection_snapshot_retry_allowed('STALE_steering', attempt=0, elapsed_ns=80_000_001)
    release_sites = (25., 45., 60., 70., 85., 90., 95., 100., 105., 110., 113., 116., 118., 140., 160., 210., 235., 265., 285., 305.)
    rows = []
    for release in release_sites:
        for offset in (.0, .2, -.2, .4, -.4, .6, -.6):
            per_normal = []
            for name, trace in traces.items():
                sample = np.linspace(release-11., release+args.return_length_m+1.,
                                     round((12.+args.return_length_m)/.05)+1)
                assert trace[0, 0] <= sample[0] <= sample[-1] <= trace[-1, 0]
                x, y, yaw = [np.interp(sample, trace[:, 0], trace[:, k]) for k in (1, 2, 3)]
                lateral = excursion_profile(sample, release, offset, args.return_length_m)
                xx = x-np.sin(yaw)*lateral; yy = y+np.cos(yaw)*lateral
                assert np.isfinite(xx).all() and np.isfinite(yy).all()
                dense = densify_with_progress(sample,xx,yy,min(.05,occupancy.resolution_m_per_px))
                sample,xx,yy = dense.T
                gaps = np.hypot(np.diff(xx), np.diff(yy))
                # Spatial sampling is no coarser than one occupancy-map cell.
                assert gaps.max() <= occupancy.resolution_m_per_px
                mask = np.array([occupancy.footprint_is_free(np.array([a]), np.array([b]), 1.4)
                                 for a,b in zip(xx, yy)], dtype=bool)
                release_mask = abs(sample-release) <= .10000001
                before = sample <= release
                after = sample >= release
                per_normal.append(dict(normal_run_id=name,whole_excursion_map_pass=bool(mask.all()),
                    approach_map_pass=bool(mask[before].all()),return_map_pass=bool(mask[after].all()),
                    release_neighborhood_map_pass=bool(mask[release_mask].all()),failed_points=int((~mask).sum()),
                    failed_progress_min_m=float(sample[~mask].min()) if (~mask).any() else None,
                    failed_progress_max_m=float(sample[~mask].max()) if (~mask).any() else None,
                    maximum_sample_gap_m=float(gaps.max()),
                    release_map_xy_m=[float(np.interp(release,sample,xx)),float(np.interp(release,sample,yy))]))
            rows.append(dict(release_progress_m=release,offset_m=offset,
                side='baseline' if offset == 0. else 'left' if offset > 0. else 'right',
                start_progress_m=release-10.,return_end_progress_m=release+args.return_length_m,
                both_normal_whole_path_pass=all(r['whole_excursion_map_pass'] for r in per_normal),
                both_normal_release_neighborhood_pass=all(r['release_neighborhood_map_pass'] for r in per_normal),
                per_normal=per_normal))
    report = dict(scope='OFFLINE_MEASURED_NORMAL_MAP_SCREEN_NOT_DRIVING_OR_TEACHER_DATA',
        source_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        source_script_sha256=sha(Path(__file__)),input_hashes=input_hashes,
        normal_source_hashes=plan['source_hashes'],normal_runs=plan['normal_runs'],
        shifted_direction='normal_to_measured_body_yaw_not_exact_road_centerline',
        offset_metric='commanded_planning_displacement_not_observed_recovery_anchor_offset',
        map_radius_m=1.4,initial_progress_step_m=.05,maximum_physical_sample_step_m=.05,
        approach_length_m=8.,hold_length_m=2.,return_length_m=args.return_length_m,
        existing_runtime_changes=False,new_awsim_run=False,new_teacher_samples=0,
        full_body_collision_or_dynamic_feasibility_proven=False,
        deadline_ms=100,retry_start_deadline_ms=80,deadline_changed=False,
        actual_coordinate_smoke='PASS',shape_exception_smoke='PASS',dense_corner_smoke='PASS',rows=rows)
    args.output.parent.mkdir(parents=True,exist_ok=False)
    with args.output.open('x') as f:json.dump(report,f,indent=2,allow_nan=False)
    summary={str(v):[r['release_progress_m'] for r in rows if r['offset_m']==v and r['both_normal_whole_path_pass']]
             for v in (.0,.2,-.2,.4,-.4,.6,-.6)}
    print(json.dumps(dict(output=str(args.output),candidate_count=len(rows),both_normal_path_pass=summary)))


if __name__ == '__main__':
    main()
