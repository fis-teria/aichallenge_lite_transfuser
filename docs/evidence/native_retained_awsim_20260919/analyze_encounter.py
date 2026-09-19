"""Summarize recorded pre-fault predictions; no simulator or control writes."""
from pathlib import Path
import json
import math
import numpy as np

root = Path('/home/thistle/e2e_autonomous/runs/time_native_retained_awsim_20260919')
results = {}
for kind in ('cone', 'box'):
    run = root / kind / ('codex-time-retained-' + kind + '-01')
    def rows(name: str) -> list[dict]:
        return [json.loads(s) for s in (run / name).read_text().splitlines()]
    host = json.loads((run / 'host_result.json').read_text())
    armed = host['last_control']['armed_ns']
    commands = [r for r in rows('control.jsonl') if r.get('event') == 'COMMAND_SENT' and r.get('sim_ns', 0) >= armed]
    first_fault = next(r for r in commands if r['reason'] != 'TIME_PATH_TRACKING')
    predictions = {r['plan_id']: r for r in rows('inference.jsonl') if r.get('event') == 'PLAN'}
    motion = [r for r in rows('vehicle_observations.jsonl') if r.get('role') == 'velocity' and r['stamp_ns'] >= armed]
    minimum = min(motion, key=lambda r: r['longitudinal_lateral_mps_heading_radps'][0])
    before = []
    for seconds in (2., 1., .5, .1):
        target_ns = first_fault['sim_ns'] - round(seconds * 1e9)
        valid = [r for r in commands if r['sim_ns'] <= target_ns and r['reason'] == 'TIME_PATH_TRACKING']
        command = valid[-1]
        plan = predictions[command['plan_id']]
        xy = np.asarray(plan['raw_xy_m'], dtype=float)
        assert xy.shape == (30, 2) and np.isfinite(xy).all()
        monotone = bool(np.all(np.diff(xy[:, 0]) > 0))
        lateral = {str(x): float(np.interp(x, xy[:, 0], xy[:, 1]))
                   if monotone and xy[0, 0] <= x <= xy[-1, 0] else None for x in (1., 2., 3.)}
        before.append(dict(seconds_before_first_fault=(first_fault['sim_ns']-command['sim_ns'])/1e9,
            command_sim_ns=command['sim_ns'], since_armed_s=(command['sim_ns']-armed)/1e9,
            plan_id=command['plan_id'], speed_kmh=command['speed_mps']*3.6,
            target_speed_kmh=command['target_speed_mps']*3.6,
            issued_steer_rad=command['steer_rad'], acceleration_mps2=command['acceleration_mps2'],
            raw_endpoint_xy_m=xy[-1].tolist(), raw_lateral_y_at_forward_x_m=lateral,
            raw_xy_m=xy.tolist(), plan_age_s=command['details'].get('plan_age_sec')))
    official = list(run.rglob('result.json'))
    results[kind] = dict(first_fault_reason=first_fault['reason'],
        first_fault_since_armed_s=(first_fault['sim_ns']-armed)/1e9,
        minimum_recorded_longitudinal_mps=minimum['longitudinal_lateral_mps_heading_radps'][0],
        minimum_velocity_observation=minimum, pre_fault_predictions=before,
        official_result_paths=[str(p.relative_to(run)) for p in official],
        coordinate_scope='Raw model XY in observation base_link. Endpoint y is not obstacle clearance; no object position was supplied to the model.',
        timing_scope='Offsets refer to first controller fault, not a synchronized ground-truth contact timestamp.')
with (root / 'encounter_analysis.json').open('x') as stream:
    json.dump(results, stream, indent=2, allow_nan=False)
for kind, report in results.items():
    print(json.dumps(dict(kind=kind, first_fault=report['first_fault_reason'],
        first_fault_since_armed_s=report['first_fault_since_armed_s'],
        minimum_longitudinal_mps=report['minimum_recorded_longitudinal_mps'],
        immediately_before={k:v for k,v in report['pre_fault_predictions'][-1].items() if k != 'raw_xy_m'})))
