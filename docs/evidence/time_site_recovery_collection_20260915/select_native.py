"""Hash-verified current nominal laps -> fixed stop site + ten seeded sites."""
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import random
import sys

import numpy as np

sys.path.insert(0, str(Path.cwd()/'tools'))
from analyze_time_normal_lap_attribution import verify_files
from aic_transfuser_lite.data.time_steering_pulse_v1 import SteeringPulseConfig, nominal_recovery_errors
from aic_transfuser_lite.data.time_random_steering_pulse_v1 import (
    SCHEMA, PulseSite, RandomPulseConfig, validate_random_guide,
)

OUT = Path('/home/thistle/e2e_autonomous/runs/time_recovery_sites_20260915')
RAW = Path('/home/thistle/e2e_autonomous/raw/time_recovery_sites_20260915')
NORMALS = ('codex-time-recovery-sites-normal-n03', 'codex-time-recovery-sites-normal-n02')
SEED = 915110


def write(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)


traces = {}; guides = {}; proofs = {}; references = {}
for name in NORMALS:
    raw = RAW/name
    proofs[name] = verify_files(raw, ('control.jsonl', 'reference.json', 'result.json'))
    result = json.loads((raw/'result.json').read_text())
    assert result['status'] == 'COMPLETE_LAP' and result['last_control']['stop_confirmed']
    assert not result['last_control']['fault'] and result['nodes']['closed_bag'] and not result['cleanup_errors']
    rows = [json.loads(r) for r in (raw/'control.jsonl').read_text().splitlines()]
    rows = [r for r in rows if r.get('reason') == 'RECOVERY_TEACHER_TRACKING' and r.get('projection') and r.get('publication')]
    segments = [[]]
    for row in rows:
        if segments[-1] and row['projection']['s_m'] < segments[-1][-1]['projection']['s_m']-5.:
            segments.append([])
        segments[-1].append(row)
    rows = max(segments, key=len)
    traces[name] = rows
    unique = {}
    for row in rows:
        unique.setdefault(row['projection']['s_m'], [row['projection']['s_m'], row['projection']['offset_m'], row['current_pose']['yaw_rad']])
    guide = np.asarray([unique[s] for s in sorted(unique)], dtype=float)
    assert guide.ndim == 2 and guide.shape[1] == 3 and np.isfinite(guide).all()
    assert np.all(np.diff(guide[:, 0]) > 0) and guide[0, 0] < 2. and guide[-1, 0] > 350.
    guides[name] = guide
    references[name] = json.loads((raw/'reference.json').read_text())
    assert not references[name].get('steering_pulse')
    write(OUT/(name+'_full_guide.json'), dict(run_id=name, control_sha256=proofs[name]['control.jsonl'], guide=guide.tolist()))
assert references[NORMALS[0]]['baseline_xy_m'] == references[NORMALS[1]]['baseline_xy_m']
guide = guides[NORMALS[0]]
for name, rows in traces.items():
    for r in rows:
        if guide[0, 0] <= r['projection']['s_m'] <= guide[-1, 0]:
            r['matched_error'] = nominal_recovery_errors(guide, s_m=r['projection']['s_m'],
                offset_m=r['projection']['offset_m'], yaw_rad=r['current_pose']['yaw_rad'])


def probe(rows, s):
    since = None; previous = None; good = []
    for r in rows:
        progress = r['projection']['s_m']
        if not s-3. <= progress <= s+2.:
            continue
        t = r['publication']['sim_ns']
        error = r.get('matched_error')
        valid = (error is not None and abs(error[0]) <= .05 and abs(error[1]) <= math.radians(1.)
                 and 1.15 <= r['speed_mps'] <= 1.4 and abs(r['nominal_angle_rad'])+.1 <= .5
                 and r['guard'].get('minimum_ray_margin_m', -1.) >= .5)
        if previous is None or not 0 < t-previous <= 150_000_000:
            since = None
        since = (t if since is None else since) if valid else None
        if since is not None and t-since >= 1_000_000_000 and s <= progress <= s+2.:
            good.append(r)
        previous = t
    return good


candidates = []
for s in range(15, 326):
    if s+2.+3.+1.7*13. > guide[-1, 0]:
        continue
    by_run = {name:probe(rows, s) for name, rows in traces.items()}
    # At least 5 eligible control publications in each current normal lap.
    if not all(len(rows) >= 5 for rows in by_run.values()):
        continue
    first = next(iter(by_run.values()))[0]
    candidates.append(dict(start_s_m=float(s), map_xy_m=[first['current_pose']['x_m'], first['current_pose']['y_m']],
        eligible_counts={n:len(rows) for n,rows in by_run.items()},
        minimum_nominal_margin_m=min(r['guard']['minimum_ray_margin_m'] for rows in by_run.values() for r in rows)))
stop_candidates = [r for r in candidates if 111. <= r['start_s_m'] <= 117.]
if not stop_candidates:
    raise ValueError('STOP_SITE_HAS_NO_MATCHED_NOMINAL_ENTRY')
stop = {**min(stop_candidates, key=lambda r:abs(r['start_s_m']-116.)), 'site_id':'S00', 'role':'fixed_stop_region', 'target_progress_m':[119., 120.]}
pool = [r for r in candidates if r['start_s_m']+2.+22.4 < 109. or r['start_s_m']-5. > 130.]
if len(pool) < 10:
    raise ValueError('INSUFFICIENT_ADDITIONAL_SITE_CANDIDATES')
strata = [list(rows) for rows in np.array_split(np.arange(len(pool)), 10)]
rng = random.Random(SEED)
for draw in range(1, 101):
    chosen = [{**pool[int(rng.choice(indices))], 'site_id':f'R{i+1:02d}', 'role':'additional_random', 'stratum':i+1}
              for i, indices in enumerate(strata)]
    ordered = sorted([stop, *chosen], key=lambda r:r['start_s_m'])
    if any(b['start_s_m']-a['start_s_m'] < 10. for a,b in zip(ordered, ordered[1:])):
        continue
    positions = np.asarray([r['map_xy_m'] for r in ordered])
    distance = np.linalg.norm(positions[:,None]-positions[None,:], axis=2)
    np.fill_diagonal(distance, np.inf)
    if distance.min() < 5.:
        continue
    groups = [ordered[i::4] for i in range(4)]
    if any(any(b['start_s_m']-a['start_s_m'] < 28. for a,b in zip(g,g[1:])) for g in groups):
        continue
    break
else:
    raise ValueError('FINITE_RANDOM_SITE_DRAW_EXHAUSTED')
stop_group = next(i for i,g in enumerate(groups) if any(s['site_id']=='S00' for s in g))
groups[0], groups[stop_group] = groups[stop_group], groups[0]
destination = OUT/'planned_references'; destination.mkdir(exist_ok=False)
runs = []; template = SteeringPulseConfig(10., .1, duration_s=2., plateau_s=1.5, start_window_m=2.)
for group_index, group in enumerate(groups, 1):
    for side, sign in (('left', 1), ('right', -1)):
        sites = tuple(PulseSite(s['site_id'], s['start_s_m'], sign) for s in group)
        cfg = RandomPulseConfig(SEED+2*group_index+(sign < 0), max_events=len(sites), sites=sites,
                                start_min_m=sites[0].start_s_m, start_max_m=sites[-1].start_s_m)
        validate_random_guide(cfg, template, guide)
        ref = {**references[NORMALS[0]], 'side':side}
        ref['steering_pulse'] = dict(schema=SCHEMA, config=asdict(template), random_config=asdict(cfg),
            nominal_run_id=NORMALS[0], nominal_control_sha256=proofs[NORMALS[0]]['control.jsonl'],
            nominal_guide=guide.tolist(), site_selection_seed=SEED)
        filename=f'group{group_index:02d}_{side}.json'; path=destination/filename; write(path, ref)
        runs.append(dict(run_id=f'codex-time-recovery-sites-g{group_index:02d}-{side}', side=side, kind='recovery',
            pair=group_index, split='validation' if group_index==4 else 'train',
            site_selection_seed=SEED, seed=cfg.seed, sites=[asdict(s) for s in sites],
            reference=filename, reference_sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
assert len(runs) == 8 and sum(len(r['sites']) for r in runs) == 22
plan = dict(schema='stop_plus_ten_random_site_collection_v1', seed=SEED, draw_number=draw,
            normal_runs=list(NORMALS), source_hashes=proofs, stop_site=stop, additional_sites=chosen,
            minimum_site_xy_distance_m=float(distance.min()), candidate_count=len(candidates),
            random_pool_count=len(pool), selection='stratified_seeded_draw_from_matched_current_nominals',
            maximum_recovery_runs=8, maximum_pulse_events=22, batch_size=2, runs=runs,
            guard_thresholds_changed=False, new_model_training=False, sealed_test_read=False)
write(OUT/'selected_site_plan.json', plan)
write(OUT/'candidate_site_audit.json', dict(candidates=candidates, source_hashes=proofs))
print(json.dumps(dict(status='SITES_SELECTED',draw_number=draw,sites=[(s['site_id'],s['start_s_m']) for s in ordered],
                     groups=[[s['site_id'] for s in g] for g in groups],runs=len(runs),events=22)), flush=True)
