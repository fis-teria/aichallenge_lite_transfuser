"""Diagnose the recorded section-4 stop in native WSL, without ROS writes.

Measured completed teacher trajectories are comparison lines, not road-center
ground truth. Common-progress decomposition is geometric, not causal attribution.
"""
from __future__ import annotations

from collections import Counter
import json
import math
from pathlib import Path
import sqlite3
import subprocess
import sys

import numpy as np

sys.path.insert(0, str(Path.cwd() / "tools"))
from analyze_time_normal_lap_attribution import compare, verify_files
from compare_time_corner_tracking import runtime_poses
from compare_time_teacher_clearance import records, read, stats, write
from aic_transfuser_lite.control.time_reference_v1 import TimedBodyPose
from aic_transfuser_lite.evaluation.time_clearance_v1 import scan_margin
from aic_transfuser_lite.evaluation.time_corner_comparison_v1 import RecordedLine

ROOT = Path("/home/thistle/e2e_autonomous")
TRIAL = ROOT / "runs/time_multiscale_model_lap_20260916/raw/codex-time-multiscale-lap01"
OUT = ROOT / "runs/time_multiscale_model_lap_20260916/corner4_diagnosis"
REFERENCE_PROGRESS_M = (100., 235.)
WINDOW_START_S = 100.
SNAPSHOTS_S = (100., 110., 120., 125., 130., 135., 140., 145., 150., 155., 160., 164.74)


def measured_reference(run: Path, config: dict) -> tuple[RecordedLine, dict]:
    hashes = verify_files(run, ("control.jsonl", "result.json", "bag/bag_0.db3"))
    result = read(run / "result.json")
    assert result["status"] == "COMPLETE_LAP"
    assert result["simulator_assets"] == {"AWSIM_Data/level1": config["geometry"]["scene_sha256"], **config["steering_asset_sha256"]}
    assert result["fixed_target_mps"] == 5. / 3.6
    controls = [r for r in records(run / "control.jsonl") if r.get("reason") == "RECOVERY_TEACHER_TRACKING"
                and r.get("current_pose") and REFERENCE_PROGRESS_M[0] <= r["projection"]["s_m"] <= REFERENCE_PROGRESS_M[1]]
    assert controls and all(r["phase"] == "baseline" for r in controls)
    low, high = min(r["current_pose"]["stamp_ns"] for r in controls), max(r["current_pose"]["stamp_ns"] for r in controls)
    from rosbags.typesys import Stores, get_typestore
    store = get_typestore(Stores.ROS2_HUMBLE)
    poses = []
    con = sqlite3.connect(f"file:{(run/'bag/bag_0.db3').resolve()}?mode=ro&immutable=1", uri=True)
    try:
        topics = con.execute("select id,type from topics where name='/localization/kinematic_state'").fetchall()
        assert len(topics) == 1 and topics[0][1] == "nav_msgs/msg/Odometry"
        for data, in con.execute("select data from messages where topic_id=? order by id", (topics[0][0],)):
            msg = store.deserialize_cdr(data, topics[0][1])
            stamp = int(msg.header.stamp.sec) * 10**9 + int(msg.header.stamp.nanosec)
            if not low <= stamp <= high:
                continue
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            poses.append(TimedBodyPose(stamp, "sim", "0", msg.header.frame_id, msg.child_frame_id, p.x, p.y,
                                     math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))))
    finally:
        con.close()
    reference = RecordedLine(poses)
    errors, excluded = [], Counter()
    for r in controls[::10]:
        pose = TimedBodyPose(**r["current_pose"])
        try:
            match = reference.index.at(pose.stamp_ns)
        except ValueError as exc:
            excluded[str(exc)] += 1
            continue
        errors.append(math.hypot(pose.x_m-match.x_m, pose.y_m-match.y_m))
    assert errors and max(errors) < .002
    return reference, dict(run=run.name, hashes=hashes, pose_audit=reference.audit,
                          progress_window_m=REFERENCE_PROGRESS_M, capture_ns=[low, high],
                          control_pose_difference_m=stats(errors), control_pose_exclusions=dict(excluded),
                          median_speed_mps=float(np.median([r["speed_mps"] for r in controls])),
                          lap_status=result["status"], assets_verified=True)


def compact_command(row: dict) -> dict:
    result = {k: v for k, v in row.items() if k != "normal_line_pp"}
    sensitivity = row.get("normal_line_pp", {})
    result["normal_line_pp"] = {k: sensitivity[k] for k in ("scope", "status", "required_tire_rad", "search_band_m") if k in sensitivity}
    return result


def grouped(cs: list[dict], ps: list[dict], first_fault_s: float) -> dict:
    result = {}
    for low, high in ((100., 120.), (120., 140.), (140., 150.), (150., 155.), (155., 160.), (160., first_fault_s+.0001)):
        cr = [r for r in cs if low <= r["time_s"] < high and r["status"] == "EVALUATED"]
        pr = [r for r in ps if low <= r["time_s"] < high and r["status"] == "EVALUATED"]
        offsets = [r for r in pr if abs(r["anchor_left_m"]) >= .2]
        parts = {}
        for h in (1, 2, 3):
            valid = [r["following"][str(h)] for r in pr if r["following"][str(h)]["status"] == "EVALUATED"]
            parts[str(h)] = dict(status_counts=dict(Counter(r["following"][str(h)]["status"] for r in pr)),
                signed={k: stats([r[k] for r in valid]) for k in ("actual_left_m", "prediction_left_m", "following_left_m", "sum_error_m")},
                absolute={k: stats([abs(r[k]) for r in valid]) for k in ("actual_left_m", "prediction_left_m", "following_left_m")})
        result[f"{low:g}_{high:.3f}s"] = dict(commands=len(cr), plans=len(pr), actual_left_m=stats([r["left_m"] for r in cr]),
            heading_error_rad=stats([r["heading_error_rad"] for r in cr]),
            tire_error_abs_rad=stats([abs(r["tire_error_rad"]) for r in cr]),
            required_tire_abs_rad=stats([abs(r["required_tire_rad"]) for r in cr]),
            plan_age_s=stats([r["plan_age_s"] for r in cr]),
            speed_mps=stats([r["speed_mps"] for r in cr]), rate_limited_count=sum(r["rate_limited"] for r in cr),
            prediction_offset_reduction={str(h): dict(eligible=len(offsets), reduces_abs_offset=sum(
                abs(r["horizons"][str(h)]["left_m"]) < abs(r["anchor_left_m"]) for r in offsets)) for h in (1, 2, 3)},
            common_progress_decomposition=parts)
    return result


def plot(references: list[RecordedLine], commands: list[dict], plans: list[dict], first_fault_s: float) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    fonts = [Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'), Path('/mnt/c/Windows/Fonts/meiryo.ttc')]
    font = next(p for p in fonts if p.exists())
    font_manager.fontManager.addfont(str(font))
    plt.rcParams['font.family'] = font_manager.FontProperties(fname=str(font)).get_name()
    plt.rcParams['axes.unicode_minus'] = False
    cs = [r for r in commands if r["status"] == "EVALUATED"]
    ps = [r for r in plans if r["status"] == "EVALUATED"]
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    axes[0].plot([r["time_s"] for r in cs], [r["left_m"] for r in cs], color="black", label="車体の実測ずれ")
    for h, color in ((1, '#2784bf'), (3, '#cc3693')):
        axes[0].plot([r["time_s"] for r in ps], [r["horizons"][str(h)]["left_m"] for r in ps], color=color, label=f"モデルの{h}秒先予測点のずれ", alpha=.8)
    axes[0].set_ylabel("教師の実測線から左方向 [m]")
    for key, label, color in (("prediction_left_m", "予測経路のずれ（同じ進行位置）", '#cc3693'),
                              ("following_left_m", "実走行と予測の差（1秒後）", '#2784bf')):
        axes[1].plot([r["time_s"] for r in ps], [r["following"]["1"][key] if r["following"]["1"]["status"] == "EVALUATED" else np.nan for r in ps], color=color, label=label)
    axes[1].set_ylabel("横方向成分 [m]")
    for key, label in (("required_tire_rad", "PPが要求したタイヤ角"), ("measured_tire_rad", "実測タイヤ角")):
        axes[2].plot([r["time_s"] for r in cs], np.rad2deg([r[key] for r in cs]), label=label)
    axes[2].set_ylabel("タイヤ角 [°]"); axes[2].set_xlabel("発進許可からの時間 [s]")
    for ax in axes:
        ax.axhline(0, color='gray', lw=.6); ax.axvline(first_fault_s, color='#ba2735', ls='--')
        ax.grid(alpha=.2); ax.legend(fontsize=9)
    fig.suptitle("区間4までの予測と実走行：完走した教師r30の実測線と比較\n横ずれの分解は幾何比較であり、原因の寄与率ではない", fontsize=12)
    fig.tight_layout(); fig.savefig(OUT/'timeline.png', dpi=150); plt.close(fig)
    origin = np.array(cs[-1]['world_xy_m'])
    fig, ax = plt.subplots(figsize=(8, 8))
    for i, ref in enumerate(references):
        ax.plot(*(ref.xy-origin).T, color=('#249452', '#85c1a0')[i], lw=(2, 1)[i], label=f"完走教師 r{30+i}")
    pts = np.array([r['world_xy_m'] for r in cs if r['time_s'] >= 140.])-origin
    ax.plot(*pts.T, color='black', lw=2, label='E2E実走行（140秒以降）')
    for i, t in enumerate((140., 145., 150., 155., 160., first_fault_s)):
        row = min(ps, key=lambda r: abs(r['time_s']-t)); xy = np.array(row['world_xy_m'])-origin
        ax.plot(*xy.T, '.-', ms=2, color='#cc3693', label='モデルの生予測（3秒分）' if i == 0 else None)
        ax.annotate(f"{row['time_s']:.1f}s", xy[0], xytext=(5, 5), textcoords='offset points', fontsize=9)
    ax.scatter(0, 0, s=80, color='#bd2938', zorder=8, label='最初の停止監視')
    ax.set(xlim=(-10, 4), ylim=(-7, 9), xlabel='停止位置からmap X [m]', ylabel='停止位置からmap Y [m]',
           title='右カーブ：教師の実測線・予測経路・実走行\n教師線は道路中心の真値ではない')
    ax.set_aspect('equal'); ax.grid(alpha=.2); ax.legend(fontsize=9)
    fig.tight_layout(); fig.savefig(OUT/'paths.png', dpi=155); plt.close(fig)


def main() -> None:
    assert sys.platform == 'linux' and Path.cwd() == ROOT/'e2e_lite_transfuser'
    OUT.mkdir(exist_ok=False)
    hashes = verify_files(TRIAL, ('control.jsonl', 'inference.jsonl', 'vehicle_observations.jsonl', 'trial_config.json', 'host_result.json'))
    config = read(TRIAL/'trial_config.json')
    rows = records(TRIAL/'control.jsonl')
    armed = next(r['sim_ns'] for r in rows if r['event'] == 'ARMED')
    fault = next(r for r in rows if r['event'] == 'SCAN_GUARD_REJECTED')
    first_fault_s = (fault['sim_ns']-armed)/1e9
    commands = [r for r in rows if r['event'] == 'COMMAND_SENT' and r.get('details', {}).get('current_pose')
                and armed+round(WINDOW_START_S*1e9) <= r['sim_ns'] <= fault['sim_ns']]
    plans = {r['plan_id']: r for r in records(TRIAL/'inference.jsonl') if r['event'] == 'PLAN'}
    poses = runtime_poses(TRIAL/'vehicle_observations.jsonl')
    summary = dict(source_commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        scope='RECORDED_GEOMETRY_AND_STEERING_NOT_CAUSAL_PERCENTAGES_OR_NEW_DRIVE', new_inference=False, runtime_modified=False,
        trial=TRIAL.name, trial_hashes=hashes, first_fault_after_arm_s=first_fault_s, analysis_start_after_arm_s=WINDOW_START_S,
        runtime_pose_ambiguities=len(poses.ambiguous), references={},
        yaw_invalid_commands=[dict(time_s=(r['sim_ns']-armed)/1e9, speed_mps=r['speed_mps']) for r in rows if r.get('reason') == 'MOTION_YAW_RATE_INVALID'])
    reference_lines, outputs = [], []
    for name in ('codex-time-recovery-speedbase-r30', 'codex-time-recovery-speedbase-r31'):
        reference, meta = measured_reference(ROOT/'raw/time_recovery_speed_20260914'/name, config)
        cr, pr = compare(reference, commands, plans, poses, config, armed, fault['sim_ns'])
        cs = [r for r in cr if r['status'] == 'EVALUATED']
        ps = [r for r in pr if r['status'] == 'EVALUATED']
        meta.update(command_status_counts=dict(Counter(r['status'] for r in cr)), plan_status_counts=dict(Counter(r['status'] for r in pr)),
                    groups=grouped(cr, pr, first_fault_s), snapshots=[compact_command(min(cs, key=lambda r: abs(r['time_s']-t))) for t in SNAPSHOTS_S],
                    prediction_snapshots=[min(ps, key=lambda r: abs(r['time_s']-t)) for t in SNAPSHOTS_S])
        assert max(abs(r['following'][str(h)].get('sum_error_m', 0.)) for r in ps for h in (1, 2, 3)) < 1e-9
        if reference_lines:
            interior = (reference.arc > 2.) & (reference.arc < reference.arc[-1]-2.)
            errors = [reference_lines[0].project(p, max_distance_m=.2).distance_m for p in reference.xy[interior][::20]]
            meta['reference_repeat_difference_m'] = stats(errors)
        summary['references'][name] = meta
        write(OUT/(name+'_commands.json'), cr); write(OUT/(name+'_plans.json'), pr)
        reference_lines.append(reference); outputs.append((cr, pr))
        print(json.dumps(dict(reference=name, command_status=meta['command_status_counts'], plan_status=meta['plan_status_counts'], last=meta['snapshots'][-1])), flush=True)
    motion = fault['motion_observation']
    summary['first_scan_replay'] = scan_margin(fault['scan'], fault['scan_in_current_rear'], speed_mps=fault['speed_mps'],
        measured_rad=fault['measured_steer_rad'], issued_rad=fault['issued_steer_rad'], previous_rad=fault['previous_steer_rad'],
        yaw_rate_radps=motion['heading_rate_radps'], lateral_mps=motion['reported_lateral_mps'])
    assert summary['first_scan_replay']['reason'] == fault['reason']
    plot(reference_lines, *outputs[0], first_fault_s)
    write(OUT/'summary.json', summary)
    print(json.dumps(dict(status='COMPLETE_RECORDED_COMPARISON', output=str(OUT))), flush=True)


if __name__ == '__main__':
    main()
