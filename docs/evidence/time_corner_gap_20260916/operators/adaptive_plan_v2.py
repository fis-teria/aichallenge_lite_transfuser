"""Generate map-screened targeted retries without changing the requested goals."""
from dataclasses import asdict, replace
from pathlib import Path
import argparse
import itertools
import json
import math
import subprocess
import sys

sys.path.insert(0, 'tools')
from generate_time_large_recovery_reference import measured_normal_trace
from aic_transfuser_lite.data.time_large_recovery_reference_v1 import preparation_course
from aic_transfuser_lite.data.time_large_recovery_v1 import LargeRecoveryConfig, LargeRecoverySite
from aic_transfuser_lite.data.time_recovery_collection_v1 import load_pose_course
from aic_transfuser_lite.data.recovery_reference_v3 import load_occupancy_map_v3

ROOT = Path('/home/thistle/e2e_autonomous')
OUT = ROOT / 'runs/time_corner_gap_20260916'
INPUTS = ROOT / 'runs/time_recovery_collection_20260913/inputs'
NORMAL = ROOT / 'raw/time_recovery_sites_20260915/codex-time-recovery-sites-normal-n03'
PROOF = ROOT / 'runs/time_recovery_separated_20260915/selected_site_plan.json'
read = lambda path: json.loads(path.read_bytes())

parser = argparse.ArgumentParser()
parser.add_argument('--pair', type=int, required=True)
parser.add_argument('--train', nargs='+', required=True)
parser.add_argument('--validation', nargs='+', required=True)
parser.add_argument('--overrides')
parser.add_argument('--heading-bias-deg', type=float, default=.3)
args = parser.parse_args()
assert 2 <= args.pair <= 6 and 0. <= args.heading_bias_deg <= 1.
screen = read(OUT / 'candidate_screen.json')
overrides = read(OUT / args.overrides) if args.overrides else {}
base = load_pose_course(INPUTS / 'base.csv')
normal = measured_normal_trace(NORMAL, read(PROOF)['source_hashes'][NORMAL.name])
occ = load_occupancy_map_v3(INPUTS / 'occupancy_grid_map.yaml')
record = dict(pair=args.pair, runtime_commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(), plans=[])
for split, keys in [('train', args.train), ('validation', args.validation)]:
    assert 1 <= len(keys) <= 3 and len(set(keys)) == len(keys)
    sites = []
    selections = []
    for key in keys:
        if key in overrides:
            selected = LargeRecoverySite(**overrides[key])
            cm, corner = key.split(':')
            original = LargeRecoverySite(**next(c['site'] for c in screen['catalogs'][cm]['corners'] if c['site']['corner_id']==corner))
            for field in ('site_id','corner_id','release_s_m','target_offset_m','target_heading_rad','heading_tolerance_rad'):
                assert getattr(selected,field)==getattr(original,field)
        else:
            selected = LargeRecoverySite(**screen['selected'][key]['site'])
            selected = replace(selected, preparation_heading_bias_rad=math.copysign(math.radians(args.heading_bias_deg), selected.target_heading_rad))
        variants = [(selected.approach_distance_m, selected.settle_distance_m, selected.return_length_m, selected.preparation_origin)]
        variants += list(itertools.product((4., 6., 8.), (2., 4.), (4., 6., 10.), ('nominal_path', 'measured_normal')))
        errors = []
        for approach, settle, ret, origin in dict.fromkeys(variants):
            site = replace(selected, approach_distance_m=approach, settle_distance_m=settle, return_length_m=ret, preparation_origin=origin)
            cfg = LargeRecoveryConfig((site,), speed_policy='record_actual_v1', entry_heading_tolerance_rad=math.radians(2.),
                recovery_duration_s=15., map_screen_policy='oriented_body_v1', failed_site_policy='continue_after_target_miss_v1')
            try:
                _, evidence = preparation_course(base, normal, cfg, occ)
            except ValueError as exc:
                if not str(exc).startswith(('LARGE_SITE_MAP_REJECTED', 'LARGE_PREPARATION_PREVIEW_TOO_SHORT')):
                    raise
                errors.append(str(exc))
                continue
            sites.append(site)
            selections.append(dict(goal=key, site=asdict(site), map_evidence=evidence, earlier_rejections=errors))
            break
        else:
            raise RuntimeError('NO_MAP_FEASIBLE_VARIANT:' + key)
    assert all(b.start_s_m - a.start_s_m >= 40. for a, b in zip(sites, sites[1:]))
    name = f'{split}_p{args.pair:02}_bias{round(args.heading_bias_deg*10):02}'
    plan = dict(name=name, split=split, seed=20260916 + args.pair + (0 if split=='train' else 100),
        event_cap=len(sites), candidates=[asdict(s) for s in sites], required_site_ids=[s.site_id for s in sites],
        speed_policy='record_actual_v1', entry_heading_tolerance_rad=math.radians(2.), recovery_duration_s=15.,
        map_screen_policy='oriented_body_v1', failed_site_policy='continue_after_target_miss_v1', goal_keys=keys)
    plan_path = OUT / 'plans' / (name + '.json')
    with plan_path.open('x') as f:
        json.dump(plan, f, indent=2)
    command = [sys.executable, 'tools/generate_time_large_recovery_reference.py', '--inputs', str(INPUTS),
        '--normal-run', str(NORMAL), '--normal-proof', str(PROOF), '--plan', str(plan_path),
        '--side', 'left' if split=='train' else 'right', '--output', str(OUT / 'references' / name)]
    log = OUT / (name + '_generation.log')
    with log.open('x') as stream:
        result = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT, timeout=120)
    if result.returncode:
        raise RuntimeError(log.read_text()[-4000:])
    record['plans'].append(dict(name=name, selections=selections, goal_keys=keys, split=split))
    print(json.dumps(dict(generated=name, goals=keys)), flush=True)
with (OUT / f'adaptive_pair{args.pair:02}.json').open('x') as stream:
    json.dump(record, stream, indent=2)
