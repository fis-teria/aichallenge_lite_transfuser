"""Native WSL audit and input/teacher preparation for a frozen production pair."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess

from aic_transfuser_lite.data.time_dataset_v1 import TimeDatasetConfig
from aic_transfuser_lite.data.time_recovery_training_v1 import materialize_recovery_run
from aic_transfuser_lite.data.time_training_cache_v1 import _prepare_run

PLAN_SHA = '04d757dde75e6b42062a6966d5f9d05bdad1649c7e4d51980f47ded5f68bb462'
ROOT = Path('/home/thistle/e2e_autonomous')


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--pair', type=int, choices=range(2, 8), required=True)
    args = ap.parse_args()
    plan_path = Path(__file__).with_name('production_plan.json')
    assert hashlib.sha256(plan_path.read_bytes()).hexdigest() == PLAN_SHA
    plan = json.loads(plan_path.read_text())
    assigned = plan['runs'][2*(args.pair-2):2*(args.pair-1)]
    assert len(assigned) == 2
    out = ROOT/'runs/time_recovery_expansion_20260914'
    raw = ROOT/'raw/time_recovery_expansion_20260914'
    types = ROOT/'runs/time_recovery_collection_20260913/types'
    prefix = f'pair{args.pair:02d}_20260914'
    receipt = json.loads((out/(prefix+'_verified.json')).read_text())
    assert receipt['all_files_and_directory_structure_identical'] and receipt['all_sqlite_quick_checks_passed']
    assert [r['run_id'] for r in receipt['runs']] == [r['run_id'] for r in assigned]
    suffixes = []
    for row in assigned:
        name = row['run_id']; suffix = name.split('-')[-1]; suffixes.append(suffix)
        commands = [
            ('bag_audit', ['tools/audit_time_recovery_collection.py', '--run', str(raw/name), '--types', str(types)]),
            ('causal_probe', ['docs/evidence/time_recovery_collection_20260913/causal_replay_probe.py', '--run', str(raw/name), '--types', str(types)]),
            ('state_audit', ['docs/evidence/time_steering_pulse_20260914/analyze_pulse.py', '--run', str(raw/name), '--probe', str(out/(suffix+'_causal_probe.json'))]),
        ]
        for kind, command in commands:
            target = out/(suffix+'_'+kind+'.json')
            assert not target.exists()
            with target.with_suffix('.log').open('x') as stream:
                result = subprocess.run(['.venv/bin/python', *command, '--output', str(target)], stdout=stream, stderr=subprocess.STDOUT)
            print('AUDIT_STAGE', name, kind, result.returncode, flush=True)
            if result.returncode:
                raise RuntimeError(target.with_suffix('.log').read_text()[-2000:])
    summary_path = out/(prefix+'_summary.json')
    command = ['.venv/bin/python', str(Path(__file__).with_name('summarize_expansion.py')),
        '--analysis', str(out), '--raw', str(raw), '--alternate-guide',
        str(ROOT/'runs/time_steering_pulse_20260914/alternate_nominal_guide_r31.json'),
        '--runs', *suffixes, '--output', str(summary_path)]
    with summary_path.with_suffix('.log').open('x') as stream:
        subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT, check=True)
    summary = json.loads(summary_path.read_text())
    for row in summary['runs']:
        assert row['status'] == 'COMPLETE_LAP' and row['recovery']['confirmed']
        assert row['target_count_agreeing_with_both_nominals'] > 0, 'Frozen production stop rule: no target anchors'
    prepared = []
    for row in assigned:
        name = row['run_id']; suffix = name.split('-')[-1]
        if row['split'] == 'evaluation_reserved':
            print('EVALUATION_RESERVED_NOT_MATERIALIZED', name, flush=True)
            continue
        assert row['split'] in ('train', 'validation')
        destination = out/'materialized'/name
        audit = materialize_recovery_run(raw/name, destination, split=row['split'], types=types,
            previous_probe=out/(suffix+'_causal_probe.json'))
        cache = out/'prepared'/row['split']/name
        prepared_run = _prepare_run(destination, cache, name, TimeDatasetConfig())
        assert prepared_run['input_valid'] == prepared_run['anchors'] == audit['accepted']
        assert not prepared_run['input_reason_refinements']
        files = {str(p.relative_to(out)): hashlib.sha256(p.read_bytes()).hexdigest()
            for directory in (cache, destination) for p in sorted(directory.iterdir()) if p.is_file()}
        prepared.append(dict(**prepared_run, split=row['split'], files=files))
        print('ALL_INPUTS_AND_TEACHERS_PREPARED', name, prepared_run['anchors'], flush=True)
    report = dict(status='PASS', production_plan_sha256=PLAN_SHA, runs=prepared,
        independently_collected_run_ids=[r['run_id'] for r in assigned],
        evaluated_model=False, trained_model=False, combined_training_identity_created=False,
        total_accepted=summary['total_accepted'],
        total_targets_agreeing_with_both_nominals=summary['total_targets_agreeing_with_both_nominals'])
    with (out/(prefix+'_prepared.json')).open('x') as stream:
        stream.write(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(report), flush=True)


if __name__ == '__main__':
    main()
