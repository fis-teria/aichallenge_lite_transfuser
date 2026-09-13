"""Apply the predeclared measured-recovery gates to a verified pair in WSL."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ANALYSIS = Path('/home/thistle/e2e_autonomous/runs/time_recovery_batches_20260914')
RAW = Path('/home/thistle/e2e_autonomous/raw/time_recovery_batches_20260914')


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--pair', type=int, choices=(1, 2, 3, 4), required=True)
    ap.add_argument('--runs', nargs=2, required=True)
    args = ap.parse_args()
    assert len(set(args.runs)) == 2
    prefix = f'pair{args.pair:02d}_20260914'
    receipt = json.loads((ANALYSIS/(prefix+'_verified.json')).read_text())
    assert [r['run_id'] for r in receipt['runs']] == args.runs
    output = ANALYSIS/(prefix+'_summary.json'); assert not output.exists()
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.7), sharey=True)
    records = []
    for axis, name in zip(axes, args.runs):
        run = RAW/name
        result = json.loads((run/'result.json').read_text())
        audit = json.loads((ANALYSIS/(name+'_audit.json')).read_text())
        replay = json.loads((ANALYSIS/(name+'_causal_probe.json')).read_text())
        ref = json.loads((run/'reference.json').read_text())
        raw_manifest_sha = hashlib.sha256((run/'transfer_manifest.json').read_bytes()).hexdigest()
        assert replay['raw_manifest_sha256'] == raw_manifest_sha
        assert result['nodes']['closed_bag'] and audit['sqlite_quick_check'] == 'ok'
        assert audit['reference_sha256'] == result['reference_sha256'] == ref['reference_sha256']
        requested = ref['signed_offset_m']; sign = float(np.sign(requested))
        hold = audit['phase_metrics']['hold'].get('median_signed_offset_m')
        after = audit['phase_metrics']['after_recovery'].get('max_absolute_offset_m')
        gates = dict(
            measured_hold_magnitude=(hold is not None and sign*hold >= abs(requested)/2),
            recovered_within_10cm=(after is not None and after <= .1),
            reduced_by_at_least_5cm=(hold is not None and after is not None and abs(hold)-after >= .05),
            traversed_recovery=audit['fully_traversed_recovery_intervals'] >= 1,
            one_epoch=replay['epochs'] == 1,
            full_future_available=replay['audited_full_observed_future'] > 0,
            all_phase_candidates_audited=replay['all_phase_candidates_audited'],
            three_tensor_replays=(len(replay['replay_smoke']) == 3 and all(s['status'] == 'PASS' for s in replay['replay_smoke'])),
            complete_lap=result['status'] == 'COMPLETE_LAP',
            normal_stop=bool(audit['stop_confirmed'] and not audit['fault']))
        accepted = all(gates.values())
        record = dict(run_id=name, side=result['side'], requested_offset_m=requested,
            geometry=ref['selected_segments'][0]['geometry'], source_sha=result['source_sha'],
            reference_sha256=result['reference_sha256'], run_status=result['status'],
            category='COMPLETE_RECOVERY_PILOT' if accepted else 'DIAGNOSTIC_OR_PARTIAL',
            complete_lap_pilot=accepted, gates=gates, failed_gates=[k for k,v in gates.items() if not v],
            hold_median_signed_offset_m=hold, after_recovery_max_absolute_offset_m=after,
            moving_speed_median_kmh=audit['moving_speed_median_kmh'],
            max_measured_speed_kmh=audit['max_measured_speed_kmh'], target_kmh=audit['tracking_target_kmh'],
            camera_messages=audit['sensors']['camera']['count'], lidar_messages=audit['sensors']['lidar']['count'],
            audited_anchors=replay['audited_anchors'], full_observed_future_anchors=replay['audited_full_observed_future'],
            image_lidar_tensor_replay_examples=len(replay['replay_smoke']),
            invalid_raw_heading_message_count=replay['invalid_raw_heading_message_count'],
            bag_bytes=sum(p.stat().st_size for p in (run/'bag').iterdir() if p.is_file()),
            verified_files=audit['verified_files'], raw_path=str(run), raw_manifest_sha256=raw_manifest_sha,
            fault=audit['fault'], training_materialized=False, split_assigned=False)
        records.append(record)
        controls = [json.loads(line) for line in (run/'control.jsonl').read_text().splitlines()]
        start = ref['intervals'][0]['start_s_m']; end = ref['intervals'][-1]['end_s_m']
        selected = []
        for row in controls:
            if row.get('reason') != 'RECOVERY_TEACHER_TRACKING' or not row.get('projection'):
                continue
            s = row['projection']['s_m']
            if start-2 <= s < end+5:
                selected.append(row)
            elif selected and s >= end+5:
                break
        if selected:
            axis.plot([r['projection']['s_m'] for r in selected], [r['projection']['offset_m'] for r in selected],
                      color='#c65c25' if sign > 0 else '#2471a3', linewidth=2, label='Measured pose (first pass)')
        xy = np.asarray(ref['baseline_xy_m']); changed = np.asarray(ref['reference_xy_m'])
        base_s = np.r_[0., np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
        axis.plot(base_s, sign*np.linalg.norm(changed-xy, axis=1), '--', color='#555555', label='PP reference')
        for interval, color in zip(ref['intervals'], ('#f7d694', '#dedede', '#b8dbbf')):
            axis.axvspan(interval['start_s_m'], interval['end_s_m'], color=color, alpha=.45)
            axis.text((interval['start_s_m']+interval['end_s_m'])/2, .63, interval['phase'], ha='center', va='top', fontsize=9)
        axis.axhline(0., color='#777777', linewidth=.7)
        axis.set(xlim=(start-2, end+5), ylim=(-.65, .65), xlabel='Progress along base course [m]',
                 title=f'{name}\n'+('Accepted recovery pilot' if accepted else 'Not accepted: '+', '.join(record['failed_gates'])))
        axis.grid(alpha=.2); axis.legend(loc='lower left', fontsize=8)
    axes[0].set_ylabel('Lateral offset [m] (+ left)')
    fig.suptitle('Measured Pure Pursuit recovery; target 5 km/h; no model evaluation', fontsize=12)
    fig.tight_layout(); fig.savefig(ANALYSIS/(prefix+'_measured_recovery.png'), dpi=160); plt.close(fig)
    summary = dict(records=records, complete_recovery_pilots=sum(r['complete_lap_pilot'] for r in records),
        recorded_runs=len(records), total_bag_bytes=sum(r['bag_bytes'] for r in records),
        full_observed_future_anchors=sum(r['full_observed_future_anchors'] for r in records),
        scope='ADDITIONAL_MEASURED_RECOVERY_NOT_MODEL_EVALUATION',
        note='Overlapping anchors are not independent recovery trials. Unaccepted runs remain diagnostics.',
        training_materialized=False, split_assigned=False, full_body_collision_free_verified=False)
    output.write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
    print(json.dumps(summary, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
