from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root = Path('/home/thistle/e2e_autonomous/runs/time_turning_20260913')
run = root/'codex-time-turn-14'
rows = [json.loads(line) for line in (run/'control.jsonl').read_text().splitlines()]
armed = next(r['sim_ns'] for r in rows if r['event'] == 'ARMED')
commands = [r for r in rows if r.get('reason') == 'TIME_PATH_TRACKING' and r['event'] == 'COMMAND_SENT']
def stats(a):
    a = np.asarray(a)
    return dict(min=float(np.min(a)), median=float(np.median(a)), p95=float(np.quantile(a,.95)), max=float(np.max(a)))
t = np.array([(r['sim_ns']-armed)/1e9 for r in commands])
report_t = np.array([(r['steering_observation']['stamp_ns']-armed)/1e9 for r in commands])
nominal = np.array([r['details']['steer_rad'] for r in commands])
actual = np.array([r['measured_steer_rad'] for r in commands])
correction = np.array([r['details']['steering_response']['applied_correction_rad'] for r in commands])
target = np.array([r['details']['steering_actuator']['issued_tire_target_rad'] for r in commands])
keep = (t > 10.) & (np.abs(nominal) > .04)
lag_grid = np.arange(-.1,.301,.005)
lag_errors = np.array([np.mean(np.abs(actual[keep]-np.interp(report_t[keep]-lag,t,nominal))) for lag in lag_grid])
diagnostic = {
    'scope':'RECORDED_CLOSED_LOOP_COMMAND_AND_STAMPED_STEERING_NOT_PATH_ACCURACY',
    'tracking_commands':len(commands), 'tracking_elapsed_sim_s':float(t[-1]),
    'nominal_speed_target_mps':stats([r['target_speed_mps'] for r in commands]),
    'measured_speed_mps':stats([r['speed_mps'] for r in commands]),
    'report_age_sim_s':stats([r['steering_observation']['age_sim_s'] for r in commands]),
    'report_age_wall_s':stats([r['steering_observation']['age_wall_s'] for r in commands]),
    'compensation_rad':stats(correction),
    'compensation_active_commands':int(np.count_nonzero(np.abs(correction) > .00001)),
    'headroom_limited_commands':sum(r['details']['steering_response']['headroom_limited'] for r in commands),
    'turning_current_report_vs_nominal_mae_rad':float(np.mean(np.abs(actual[keep]-nominal[keep]))),
    'turning_capture_aligned_vs_nominal_mae_rad':float(np.mean(np.abs(actual[keep]-np.interp(report_t[keep],t,nominal)))),
    'best_fit_capture_aligned_lag_s':float(lag_grid[np.argmin(lag_errors)]),
    'best_fit_mae_rad':float(np.min(lag_errors)),
    'lag_fit_boundary':'Descriptive fit of this changing closed-loop sequence, not identified actuator transport delay.',
}
rejected = [r for r in rows if r['event'] == 'SCAN_GUARD_REJECTED']
if rejected:
    first = rejected[0]
    diagnostic['first_rejection'] = {k:first[k] for k in ('sim_ns','reason','measured_steer_rad','issued_steer_rad','previous_steer_rad','steering_observation','speed_mps','current_pose','steering_response')}
    diagnostic['first_rejection']['elapsed_sim_s'] = (first['sim_ns']-armed)/1e9
(root/'turn14_response_diagnostic.json').write_text(json.dumps(diagnostic,indent=2))
fig, axes = plt.subplots(2,1,figsize=(11,7),sharex=True)
axes[0].plot(t,nominal,label='Raw PP nominal tire angle',linewidth=1)
axes[0].plot(t,target,label='Compensated issued physical target',linewidth=.8)
axes[0].plot(report_t,actual,label='Reported tire angle at capture',linewidth=.8)
axes[0].set_ylabel('Physical tire angle [rad]'); axes[0].legend(); axes[0].grid(alpha=.25)
axes[1].plot(t,correction,label='Lead correction [rad]')
axes[1].set_ylabel('Lead correction [rad]'); axes[1].set_xlabel('Simulation seconds from arm'); axes[1].grid(alpha=.25)
fig.suptitle('turn14: fixed 5 km/h nominal target, unchanged E2E path')
fig.tight_layout(); fig.savefig(root/'turn14_steering_response.png',dpi=130)
print(json.dumps(diagnostic,indent=2))
