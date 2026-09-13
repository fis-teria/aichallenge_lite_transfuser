"""Compare bounded segment fallback with legacy PP on immutable AWSIM records."""
from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
import subprocess
import time

import numpy as np

from aic_transfuser_lite.control.awsim_steering import command_steering
from aic_transfuser_lite.control.awsim_steering_response import compensate_steering_response
from aic_transfuser_lite.control.time_reference_v1 import TimePlan, TimedBodyPose
from aic_transfuser_lite.control.time_trial_v1 import time_trial_control, SEGMENT_LOOKAHEAD_POLICY
from aic_transfuser_lite.evaluation.time_smoothing_v1 import distribution
from evaluate_time_awsim_trial import replay_recorded_control
from evaluate_time_smoothing import read_json, write_json, sha


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--records', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    repo = Path(__file__).resolve().parents[1]
    result = {'source_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
              'scope': 'recorded vehicle states; hypothetical actuator sequence from zero, no scan or closed-loop replay',
              'trials': {}}
    for name, rid in (('baseline', 'codex-time-recovery-model-base01'), ('candidate', 'codex-time-recovery-model-candidate01')):
        root = args.records/name/rid
        expected = read_json(repo/'docs/evidence/time_recovery_finetune_20260914'/f'{name}_evaluation/summary.json')
        for filename, digest in expected['source_sha256'].items():
            if sha(root/filename) != digest:
                raise ValueError('SOURCE_HASH_CHANGED:'+filename)
        cfg = read_json(root/'trial_config.json')
        commands = [json.loads(line) for line in (root/'control.jsonl').read_text().splitlines()]
        plans = [json.loads(line) for line in (root/'inference.jsonl').read_text().splitlines()]
        plans = [p for p in plans if p.get('event') == 'PLAN']
        by_id = {p['plan_id']: p for p in plans}
        start = next(c['sim_ns'] for c in commands if c.get('event') == 'ARMED')
        end = min([start+int(cfg['drive_limit_sim_s']*1e9)] + [c['sim_ns'] for c in commands if c.get('event') == 'STOP_REQUESTED'])
        active = [c for c in commands if c.get('event') == 'COMMAND_SENT' and start <= c['sim_ns'] < end]
        replay = replay_recorded_control(active, plans, cfg['geometry']['rear_axle_forward_in_base_link_m'],
            **{k: cfg[k] for k in ('speed_policy', 'obstacle_policy', 'steering_policy', 'lookahead_policy', 'vehicle_model_policy')})
        stats = Counter(); reasons = Counter(); rows = []; latency_ms = []
        response_state = None; previous_input = 0.; last_wall_ns = None
        for command in active:
            detail = command.get('details', {})
            if not command.get('plan_id') or command.get('speed_mps') is None or not all(k in detail for k in ('observation_pose', 'current_pose')):
                stats['unavailable'] += 1; response_state = None
                last_wall_ns = command['monotonic_ns']
                continue
            stats['eligible'] += 1
            plan = TimePlan(command['plan_id'], TimedBodyPose(**detail['observation_pose']), np.asarray(by_id[command['plan_id']]['raw_xy_m']))
            current = TimedBodyPose(**detail['current_pose'])
            options = dict(speed_mps=command['speed_mps'], speed_policy=cfg['speed_policy'],
                           vehicle_model_policy=cfg['vehicle_model_policy'],
                           rear_axle_offset_m=(cfg['geometry']['rear_axle_forward_in_base_link_m'], 0.))
            def calculate(policy: str) -> tuple[dict | None, str]:
                try:
                    return time_trial_control(plan, current, **options, lookahead_policy=policy), 'PP_OK'
                except ValueError as exc:
                    return None, str(exc)
            old, old_reason = calculate(cfg['lookahead_policy'])
            began = time.perf_counter()
            new, new_reason = calculate(SEGMENT_LOOKAHEAD_POLICY)
            latency_ms.append((time.perf_counter()-began)*1000)
            row = {'sim_ns': command['sim_ns'], 'plan_id': command['plan_id'], 'speed_mps': command['speed_mps'],
                   'old_reason': old_reason, 'new_reason': new_reason}
            dt = .05 if last_wall_ns is None else min(.1, max(0., (command['monotonic_ns']-last_wall_ns)/1e9))
            last_wall_ns = command['monotonic_ns']
            if old is not None:
                assert new is not None, 'EXISTING_ADMISSION_LOST'
                for key in ('steer_rad', 'acceleration_mps2', 'target_speed_mps', 'lookahead_rear_m', 'reference_xy_rear_m'):
                    assert old[key] == new[key], 'EXISTING_COMMAND_CHANGED:'+key
                assert new['lookahead_selection']['kind'] == 'vertex'
                stats['existing_pp_identical'] += 1
            if new is not None:
                stats['new_pp_pass'] += 1
                stats[new['lookahead_selection']['kind']] += 1
                if old is None:
                    assert old_reason == 'STEERING_FEASIBLE_LOOKAHEAD_MISSING'
                    stats['rescued'] += 1
                target, response_state, response = compensate_steering_response(new['steer_rad'], command['sim_ns'], response_state, policy=cfg['steering_policy'])
                mapped = command_steering(target, previous_input, dt, policy=cfg['steering_policy'])
                assert abs(mapped['issued_input_rad']) <= .5 and abs(mapped['issued_input_rad']-previous_input) <= .8*dt+1e-12
                previous_input = mapped['issued_input_rad']
                row.update(selection=new['lookahead_selection'], target_speed_mps=new['target_speed_mps'],
                           response=response, hypothetical_actuator=mapped)
            else:
                assert old_reason == new_reason, 'UNRELATED_REJECTION_CHANGED'
                response_state = None
            reasons[new_reason] += 1; rows.append(row)
        with (args.output/f'{name}_commands.jsonl').open('x') as stream:
            for row in rows:
                stream.write(json.dumps(row, allow_nan=False)+'\n')
        assert stats['eligible'] == expected['control_replay']['matched_commands']
        unchanged_replay = None
        if not stats['rescued']:
            unchanged_replay = replay_recorded_control(active, plans, cfg['geometry']['rear_axle_forward_in_base_link_m'],
                **{k: cfg[k] for k in ('speed_policy', 'obstacle_policy', 'steering_policy', 'vehicle_model_policy')},
                lookahead_policy=SEGMENT_LOOKAHEAD_POLICY)
        result['trials'][name] = {'old_replay': replay, 'counts': dict(stats), 'new_reasons': dict(reasons),
            'unchanged_full_actuator_replay': unchanged_replay,
            'policy_time_ms': distribution(latency_ms),
            'segment_margin_rad': distribution([r['selection']['steering_margin_rad'] for r in rows if r.get('selection', {}).get('kind') == 'segment']),
            'first_command': rows[0]}
    result['status'] = 'PASS'
    write_json(args.output/'summary.json', result)
    print(json.dumps({name: t['counts'] for name, t in result['trials'].items()}), flush=True)


if __name__ == '__main__':
    main()
