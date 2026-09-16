"""Write finite all-corner lap plans; missed targets require later audited runs."""
from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path

from aic_transfuser_lite.data.time_corner_recovery_v1 import partition_corner_sites, pending_corner_laps
from aic_transfuser_lite.data.time_large_recovery_v1 import LargeRecoverySite


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--catalog', type=Path, required=True)
    ap.add_argument('--base', type=Path, required=True)
    ap.add_argument('--coverage', type=Path)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    catalog = json.loads(args.catalog.read_bytes())
    if (catalog['schema'] != 'corner_recovery_catalog_v1'
            or hashlib.sha256(args.base.read_bytes()).hexdigest() != catalog['base_csv_sha256']
            or type(catalog['maximum_runs']) is not int or not 2 <= catalog['maximum_runs'] <= 12):
        raise ValueError('CORNER_CAMPAIGN_IDENTITY_OR_BUDGET')
    sites = [LargeRecoverySite(**r['site']) for r in catalog['corners']]
    initial = partition_corner_sites(sites, event_cap=catalog['per_lap_cap'])
    coverage = json.loads(args.coverage.read_bytes()) if args.coverage else None
    plans = []
    for split in ('train', 'validation'):
        laps = pending_corner_laps(sites, coverage, split=split, event_cap=catalog['per_lap_cap']) if coverage else initial
        for i, lap in enumerate(laps):
            plans.append(dict(name=f'{split}_lap{i+1:02}', split=split, seed=lap.seed,
                event_cap=len(lap.sites), candidates=[asdict(s) for s in lap.sites],
                required_site_ids=[s.site_id for s in lap.sites]))
    if len(plans) > catalog['maximum_runs']:
        raise ValueError('CORNER_INITIAL_RUN_BUDGET')
    args.output.mkdir(parents=True, exist_ok=False)
    for plan in plans:
        (args.output/(plan['name']+'.json')).write_text(json.dumps(plan, indent=2)+'\n')
    summary = dict(schema='corner_recovery_lap_plan_v1', plans=plans, maximum_runs=catalog['maximum_runs'],
        catalog_sha256=hashlib.sha256(args.catalog.read_bytes()).hexdigest(),
        previous_coverage_sha256=hashlib.sha256(args.coverage.read_bytes()).hexdigest() if args.coverage else None,
        completion='audited_entry_state_and_recovery_per_corner_per_split',
        missed_or_failed_targets_are_complete=False, aws_sim_modified=False)
    (args.output/'plan.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary))


if __name__ == '__main__':
    main()
