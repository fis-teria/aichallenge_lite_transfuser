"""Explain skipped collection windows from preserved measured controls."""
from pathlib import Path
import json
import math

OUT = Path('/home/thistle/e2e_autonomous/runs/time_corner_recovery_20260916')
RAW = Path('/home/thistle/e2e_autonomous/raw/time_corner_recovery_20260916')


def main() -> None:
    results = []
    for summary in sorted(OUT.glob('*_collection_summary.json')):
        report = json.loads(summary.read_bytes())
        raw = RAW / report['run_id']
        final = json.loads((raw / 'result.json').read_bytes())
        state = (final['last_control'].get('large_recovery') or {}).get('state') or {}
        skipped = state.get('skipped_sites', [])
        if not skipped:
            continue
        rows = [json.loads(line) for line in (raw / 'control.jsonl').read_text().splitlines()]
        for number in skipped:
            site = report['planned'][number]
            start = site['release_s_m'] - site.get('approach_distance_m', 8.) - site['settle_distance_m']
            nearby = [r for r in rows if start - 2. <= (r.get('projection') or {}).get('s_m', -1.) <= start + 1.
                      and r.get('large_recovery', {}).get('state', {}).get('site_cursor') == number]
            assert nearby
            speed = [r['speed_mps'] for r in nearby]
            data = [r['large_recovery'] for r in nearby]
            # stable_since_ns is the runtime state, including the real previous-scan guard gate.
            start_window = [r for r in nearby if start <= r['projection']['s_m'] <= start + 1.]
            stable_s = [(r['large_recovery']['state']['last_sim_ns'] - r['large_recovery']['state']['stable_since_ns']) / 1e9
                        for r in start_window if r['large_recovery']['state']['stable_since_ns'] is not None]
            results.append(dict(run_id=report['run_id'], site_id=site['site_id'],
                preparation_start_s_m=start, samples=len(nearby),
                speed_min_mps=min(speed), speed_max_mps=max(speed),
                outside_speed_gate=sum(not 1.15 <= v <= 1.4 for v in speed),
                outside_lateral_gate=sum(abs(d['lateral_error_m']) > .05 for d in data),
                outside_heading_gate=sum(abs(d['heading_error_rad']) > math.radians(1.) for d in data),
                readiness_measurement_window_m=[start, start + 1.],
                maximum_reported_ready_s=max(stable_s, default=0.), required_ready_s=1.,
                minimum_current_guard_margin_m=min(r['guard']['minimum_ray_margin_m'] for r in nearby),
                outcome='SKIPPED_NOT_TRAINING_COVERAGE'))
    with (OUT / 'skipped_window_diagnosis.json').open('x') as stream:
        json.dump(results, stream, indent=2, allow_nan=False)
    print(json.dumps(results), flush=True)


if __name__ == '__main__':
    main()
