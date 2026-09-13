"""Summarize closed, hash-audited pilots in native WSL; no training or split."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--raw-root', type=Path, required=True)
    parser.add_argument('--audit-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(exist_ok=False)
    names = ['codex-time-recovery-left020-r17', 'codex-time-recovery-left020-r18',
             'codex-time-recovery-right020-r19']
    records = []
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5), sharey=True)
    for name in names:
        run = args.raw_root/name
        result = json.loads((run/'result.json').read_text())
        audit = json.loads((args.audit_root/(name+'_audit.json')).read_text())
        replay = json.loads((args.audit_root/(name+'_causal_probe.json')).read_text())
        reference = json.loads((run/'reference.json').read_text())
        assert result['nodes']['closed_bag'] and audit['sqlite_quick_check'] == 'ok'
        assert replay['raw_manifest_sha256'] == hashlib.sha256((run/'transfer_manifest.json').read_bytes()).hexdigest()
        side = result['side']; sign = 1. if side == 'left' else -1.
        hold = audit['phase_metrics']['hold']['median_signed_offset_m']
        after = audit['phase_metrics']['after_recovery']
        # A stricter maximum-error check also proves the prescribed median-abs
        # condition; opposite signed residuals cannot cancel into a false pass.
        motion_gate = (sign*hold >= .1 and after['max_absolute_offset_m'] <= .1
                       and abs(hold)-after['max_absolute_offset_m'] >= .05)
        segment_valid = (motion_gate and audit['fully_traversed_recovery_intervals'] >= 1
                         and replay['epochs'] == 1 and replay['audited_full_observed_future'] > 0)
        complete = segment_valid and result['status'] == 'COMPLETE_LAP' and audit['stop_confirmed']
        records.append(dict(run_id=name, side=side, source_sha=result['source_sha'],
            reference_sha256=result['reference_sha256'], run_status=result['status'],
            complete_lap_pilot=complete, validated_recovery_segment=segment_valid,
            hold_median_signed_offset_m=hold,
            after_recovery_median_signed_offset_m=after['median_signed_offset_m'],
            after_recovery_max_absolute_offset_m=after['max_absolute_offset_m'],
            moving_speed_median_kmh=audit['moving_speed_median_kmh'],
            max_measured_speed_kmh=audit['max_measured_speed_kmh'],
            target_kmh=audit['tracking_target_kmh'], camera_messages=audit['sensors']['camera']['count'],
            lidar_messages=audit['sensors']['lidar']['count'],
            audited_anchors=replay['audited_anchors'],
            full_observed_future_anchors=replay['audited_full_observed_future'],
            all_phase_candidates_audited=replay['all_phase_candidates_audited'],
            image_lidar_tensor_replay_examples=len(replay['replay_smoke']),
            bag_bytes=sum(p.stat().st_size for p in (run/'bag').iterdir() if p.is_file()),
            verified_files=audit['verified_files'], raw_path=str(run),
            raw_manifest_sha256=replay['raw_manifest_sha256'],
            fault=audit['fault'], training_materialized=False, split_assigned=False))
        if name.endswith('r17'):
            continue
        axis = axes[0 if side == 'left' else 1]
        controls = [json.loads(line) for line in (run/'control.jsonl').read_text().splitlines()]
        tracking = [row for row in controls if row['reason'] == 'RECOVERY_TEACHER_TRACKING' and row['projection']]
        start = reference['intervals'][0]['start_s_m']; end = reference['intervals'][-1]['end_s_m']
        # Show the first traversal only. Do not join it to a later lap pass.
        selected = []; entered = False
        for row in tracking:
            s = row['projection']['s_m']
            if start-2 <= s < end+5:
                entered = True; selected.append(row)
            elif entered and s >= end+5:
                break
        assert selected
        axis.plot([r['projection']['s_m'] for r in selected],
                  [r['projection']['offset_m'] for r in selected],
                  color='#c65c25' if side == 'left' else '#2471a3', label='Measured pose', linewidth=2)
        # The reference is debug-only. Plot its actual sparse CSV vertices.
        xy = np.asarray(reference['baseline_xy_m']); changed = np.asarray(reference['reference_xy_m'])
        base_s = np.r_[0., np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
        offset = sign*np.linalg.norm(changed-xy, axis=1)
        axis.plot(base_s, offset, '--', color='#555555', label='PP reference', linewidth=1.3)
        for interval, color in zip(reference['intervals'], ('#f7d694','#dedede','#b8dbbf')):
            axis.axvspan(interval['start_s_m'], interval['end_s_m'], color=color, alpha=.45)
            axis.text((interval['start_s_m']+interval['end_s_m'])/2, .315, interval['phase'],
                      ha='center', va='top', fontsize=9)
        axis.axhline(0., color='#777777', linewidth=.7)
        axis.set(xlim=(start-2,end+5), ylim=(-.34,.34), xlabel='Progress along base course [m]',
                 title=side.capitalize()+' 0.20 m / '+('complete lap' if complete else 'later timing stop'))
        axis.grid(alpha=.2); axis.legend(loc='lower left')
    axes[0].set_ylabel('Lateral offset [m] (+ left)')
    fig.suptitle('Measured straight recovery at target 5 km/h (Pure Pursuit teacher)', fontsize=12)
    fig.tight_layout(); fig.savefig(args.output/'measured_recovery.png',dpi=160); plt.close(fig)
    summary = dict(scope='INITIAL_MEASURED_RECOVERY_PILOT_NOT_MODEL_EVALUATION', records=records,
        condition_count=2, recording_run_count=len(records),
        complete_lap_pilots=sum(r['complete_lap_pilot'] for r in records),
        validated_recovery_passes=sum(r['validated_recovery_segment'] for r in records),
        full_observed_future_anchors=sum(r['full_observed_future_anchors'] for r in records),
        total_bag_bytes=sum(r['bag_bytes'] for r in records),
        note='Overlapping anchors are not independent recovery trials. Interrupted runs remain explicitly marked.',
        training_materialized=False, split_assigned=False, full_body_collision_free_verified=False)
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False))
    print(json.dumps(summary,indent=2,allow_nan=False))


if __name__ == '__main__':
    main()
