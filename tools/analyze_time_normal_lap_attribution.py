"""Split recorded normal-lap geometry from following and actuator diagnostics.

Native WSL only. Uses two completed same-host teacher runs as measured lines;
does not read held-out recovery bags, rerun inference, or send ROS commands.
"""
from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict
import json
import math
from pathlib import Path
import sqlite3
import subprocess
from typing import Any

import numpy as np

from aic_transfuser_lite.control.time_reference_v1 import TimedBodyPose
from aic_transfuser_lite.evaluation.time_clearance_v1 import scan_margin
from aic_transfuser_lite.evaluation.time_corner_comparison_v1 import (
    RecordedLine, angle_delta, future_tracking_error, world_points)
from aic_transfuser_lite.evaluation.time_path_attribution_v1 import lateral_decomposition, reference_preview
from compare_time_corner_tracking import runtime_poses
from compare_time_teacher_clearance import records, read, sha, stats, write


PHASES = (("entry", 55., 65.), ("corner", 65., 80.), ("return", 80., 90.), ("late", 90., 101.))


def verify_files(run: Path, names: tuple[str, ...]) -> dict[str, str]:
    manifest = read(run/"transfer_manifest.json")
    if isinstance(manifest, list):
        manifest = {r["path"]: r for r in manifest}
    hashes = {}
    for name in names:
        path = run/name
        hashes[name] = sha(path)
        if path.stat().st_size != manifest[name]["bytes"] or hashes[name] != manifest[name]["sha256"]:
            raise ValueError("SOURCE_FILE_MISMATCH:"+str(path))
    return hashes


def measured_reference(run: Path, config: dict[str, Any]) -> tuple[RecordedLine, dict[str, Any]]:
    hashes = verify_files(run, ("control.jsonl", "result.json", "bag/bag_0.db3"))
    result = read(run/"result.json")
    expected_assets = {"AWSIM_Data/level1": config["geometry"]["scene_sha256"], **config["steering_asset_sha256"]}
    if (result["status"] != "COMPLETE_LAP" or result["simulator_assets"] != expected_assets
            or result["fixed_target_mps"] != 5./3.6):
        raise ValueError("REFERENCE_LAP_OR_ENVIRONMENT_MISMATCH")
    controls = [r for r in records(run/"control.jsonl") if r.get("reason") == "RECOVERY_TEACHER_TRACKING"
                and r.get("current_pose") and 55. <= r["projection"]["s_m"] <= 142.]
    if not controls or any(r["phase"] != "baseline" for r in controls):
        raise ValueError("REFERENCE_NORMAL_PASSAGE_REQUIRED")
    low, high = min(r["current_pose"]["stamp_ns"] for r in controls), max(r["current_pose"]["stamp_ns"] for r in controls)
    # Only standard Odometry is needed. Loading unrelated custom vehicle message
    # definitions is unnecessary and can disagree across older collection bags.
    from rosbags.typesys import Stores, get_typestore
    store = get_typestore(Stores.ROS2_HUMBLE)
    poses = []
    con = sqlite3.connect(f"file:{(run/'bag/bag_0.db3').resolve()}?mode=ro&immutable=1", uri=True)
    try:
        topics = con.execute("select id,type from topics where name='/localization/kinematic_state'").fetchall()
        if len(topics) != 1 or topics[0][1] != "nav_msgs/msg/Odometry":
            raise ValueError("REFERENCE_ODOMETRY_TOPIC")
        for data, in con.execute("select data from messages where topic_id=? order by id", (topics[0][0],)):
            msg = store.deserialize_cdr(data, topics[0][1])
            stamp = int(msg.header.stamp.sec)*10**9+int(msg.header.stamp.nanosec)
            if not low <= stamp <= high:
                continue
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            poses.append(TimedBodyPose(stamp, "sim", "0", msg.header.frame_id, msg.child_frame_id,
                p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))))
    finally:
        con.close()
    line = RecordedLine(poses)
    checks, excluded = [], Counter()
    for r in controls[::10]:
        current = TimedBodyPose(**r["current_pose"])
        try:
            measured = line.index.at(current.stamp_ns)
        except ValueError as exc:
            excluded[str(exc)] += 1
            continue
        checks.append(math.hypot(current.x_m-measured.x_m, current.y_m-measured.y_m))
    if not checks or max(checks) > .002:
        raise ValueError("REFERENCE_CONTROL_ODOMETRY_DISAGREES")
    return line, {"run": run.name, "hashes": hashes, "pose_audit": line.audit,
                  "selected_capture_ns": [low, high], "control_pose_difference_m": stats(checks),
                  "control_pose_check_exclusions": dict(excluded),
                  "median_speed_mps": float(np.median([r["speed_mps"] for r in controls])),
                  "lap_status": result["status"], "assets_verified": True}


def group_metrics(commands: list[dict], plans: list[dict]) -> dict[str, Any]:
    output = {}
    for name, low, high in PHASES:
        cs = [r for r in commands if low <= r["time_s"] < high and r["status"] == "EVALUATED"]
        ps = [r for r in plans if low <= r["time_s"] < high and r["status"] == "EVALUATED"]
        ds = {}
        for horizon in (1, 2, 3):
            hs = [r["following"][str(horizon)] for r in ps]
            valid = [r for r in hs if r["status"] == "EVALUATED"]
            ds[str(horizon)] = {"status_counts": dict(Counter(r["status"] for r in hs)),
                **{key: stats([abs(r[key]) for r in valid]) for key in
                   ("actual_left_m", "prediction_left_m", "following_left_m", "sum_error_m")},
                "following_abs_greater_than_prediction_count": sum(abs(r["following_left_m"]) > abs(r["prediction_left_m"]) for r in valid)}
        moving_away = {}
        for horizon in (1, 2, 3):
            eligible = [r for r in ps if abs(r["anchor_left_m"]) >= .2]
            moving_away[str(horizon)] = {"count": len(eligible), "abs_offset_reduced_count": sum(
                abs(r["horizons"][str(horizon)]["left_m"]) < abs(r["anchor_left_m"]) for r in eligible)}
        output[name] = {"time_interval_s": [low, high], "commands": len(cs), "plans": len(ps),
            "actual_left_m": stats([r["left_m"] for r in cs]),
            "tire_error_abs_rad": stats([abs(r["tire_error_rad"]) for r in cs]),
            "max_required_tire_abs_rad": max([abs(r["required_tire_rad"]) for r in cs], default=None),
            "rate_limited_commands": sum(r["rate_limited"] for r in cs),
            "geometry_sensitivity_status": dict(Counter(r["normal_line_pp"]["status"] for r in cs)),
            "prediction_return_diagnostic": moving_away, "common_progress_decomposition": ds}
    return output


def compare(reference: RecordedLine, commands: list[dict], plans_by_id: dict,
            actual_poses: Any, config: dict, arm_ns: int, fault_ns: int) -> tuple[list[dict], list[dict]]:
    command_rows, plan_rows, first_by_id = [], [], {}
    for command in commands:
        d = command["details"]; current = TimedBodyPose(**d["current_pose"])
        row = {"time_s": (command["sim_ns"]-arm_ns)/1e9, "status": "EVALUATED", "plan_id": command["plan_id"]}
        try:
            projected = reference.project(np.array([current.x_m, current.y_m]), yaw_hint_rad=current.yaw_rad)
            actuator = d["steering_actuator"]
            row.update(left_m=projected.left_m, heading_error_rad=angle_delta(current.yaw_rad, projected.body_yaw_rad),
                required_tire_rad=d["steer_rad"], measured_tire_rad=command["measured_steer_rad"],
                tire_error_rad=command["measured_steer_rad"]-d["steer_rad"],
                rate_limited=abs(actuator["issued_input_rad"]-actuator["requested_input_rad"]) > 1e-9,
                world_xy_m=[current.x_m, current.y_m], speed_mps=command["speed_mps"],
                plan_age_s=d["plan_age_sec"], guard_margin_m=d.get("obstacle_guard", {}).get("minimum_ray_margin_m"))
            try:
                row["normal_line_pp"] = {"status": "EVALUATED", **reference_preview(reference, current,
                    speed_mps=command["speed_mps"], remaining_horizon_s=3.-d["plan_age_sec"],
                    rear_axle_offset_m=config["geometry"]["rear_axle_forward_in_base_link_m"])}
            except ValueError as exc:
                row["normal_line_pp"] = {"status": str(exc)}
        except ValueError as exc:
            row["status"] = str(exc)
        command_rows.append(row)
        first_by_id.setdefault(command["plan_id"], command)
    for plan_id, command in first_by_id.items():
        source = plans_by_id[plan_id]; observed = TimedBodyPose(**command["details"]["observation_pose"])
        if source["observation_ns"] != observed.stamp_ns or source["checkpoint_sha256"] != config["checkpoint_sha256"]:
            raise ValueError("PREDICTION_IDENTITY_CHANGED")
        raw = np.asarray(source["raw_xy_m"], float)
        if raw.shape != (30, 2) or not np.isfinite(raw).all():
            raise ValueError("INVALID_RAW_PREDICTION")
        world = world_points(np.vstack([np.zeros((1, 2)), raw]), observed)
        row = {"time_s": (observed.stamp_ns-arm_ns)/1e9, "plan_id": plan_id, "status": "EVALUATED",
               "world_xy_m": world.tolist(), "observation_pose": asdict(observed), "horizons": {}, "following": {}}
        try:
            anchor = reference.project(world[0], yaw_hint_rad=observed.yaw_rad)
            bounds = (max(0., anchor.progress_m-2.), min(float(reference.arc[-1]), anchor.progress_m+8.))
            projected = [anchor]+[reference.project(p, progress_bounds_m=bounds) for p in world[1:]]
            progress = np.array([p.progress_m for p in projected])
            row.update(anchor_left_m=anchor.left_m, anchor_progress_m=anchor.progress_m)
            for horizon in (1, 2, 3):
                p = projected[horizon*10]
                row["horizons"][str(horizon)] = {"left_m": p.left_m, "change_from_anchor_m": p.left_m-anchor.left_m}
                try:
                    future_ns = observed.stamp_ns+horizon*10**9
                    if future_ns > fault_ns:
                        raise ValueError("FUTURE_AFTER_FIRST_FAULT")
                    future = actual_poses.at(future_ns)
                    q = reference.project(np.array([future.x_m, future.y_m]), progress_bounds_m=bounds)
                    parts = lateral_decomposition(world, progress, np.array([future.x_m, future.y_m]),
                                                  np.array(q.xy_m), q.tangent_yaw_rad, q.progress_m)
                    temporal = future_tracking_error(raw, observed, actual_poses, float(horizon), before_ns=fault_ns)
                    row["following"][str(horizon)] = {"status": "EVALUATED", **parts, "time_point_metrics": temporal}
                except ValueError as exc:
                    row["following"][str(horizon)] = {"status": str(exc)}
        except ValueError as exc:
            row["status"] = str(exc)
        plan_rows.append(row)
    return command_rows, plan_rows


def plots(out: Path, reference: RecordedLine, commands: list[dict], plans: list[dict]) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    cs = [r for r in commands if r["status"] == "EVALUATED"]
    ps = [r for r in plans if r["status"] == "EVALUATED"]
    fig, axes = plt.subplots(4, 1, figsize=(11, 11), sharex=True)
    axes[0].plot([r["time_s"] for r in cs], [r["left_m"] for r in cs], c="black", label="Measured vehicle vs normal line")
    for h, color in ((1, "#237cb8"), (3, "#ce318b")):
        axes[0].plot([r["time_s"] for r in ps], [r["horizons"][str(h)]["left_m"] for r in ps],
                     color=color, label=f"Model {h}s point vs normal line (issued now)", alpha=.8)
    axes[0].set_ylabel("Left offset [m]")
    for key, label, color in (("prediction_left_m", "Prediction component at common progress", "#ce318b"),
                               ("following_left_m", "Following/replanning residual at 1s", "#237cb8")):
        values = [r["following"]["1"][key] if r["following"]["1"]["status"] == "EVALUATED"
                  else np.nan for r in ps]
        axes[1].plot([r["time_s"] for r in ps], values, label=label, color=color)
    axes[1].set_ylabel("Lateral components [m]")
    for key, label in (("required_tire_rad", "Recorded PP request"), ("measured_tire_rad", "Measured tire")):
        axes[2].plot([r["time_s"] for r in cs], [r[key] for r in cs], label=label)
    normal_tire = [r["normal_line_pp"]["required_tire_rad"] if r["normal_line_pp"]["status"] == "EVALUATED"
                   else np.nan for r in cs]
    axes[2].plot([r["time_s"] for r in cs], normal_tire,
                 label="Same PP geometry aimed at normal line (offline)", alpha=.6)
    axes[2].set_ylabel("Physical tire [rad]")
    axes[3].plot([r["time_s"] for r in cs], [r["guard_margin_m"] for r in cs], label="Recorded stopping-ray margin")
    axes[3].set_ylabel("Ray margin [m]"); axes[3].set_xlabel("Time after authorization [sim s]")
    for ax in axes:
        ax.grid(alpha=.2); ax.axhline(0., color="gray", lw=.6); ax.legend(fontsize=8)
    fig.suptitle("Normal E2E lap: recorded path and following diagnostics\nSame-progress decomposition is not a causal percentage")
    fig.tight_layout(); fig.savefig(out/"attribution_timeline.png", dpi=150); plt.close(fig)
    origin = np.array(cs[-1]["world_xy_m"])
    fig, ax = plt.subplots(figsize=(9, 8))
    ax.plot(*(reference.xy-origin).T, c="#239357", label="Measured normal run r30")
    ax.plot(*(np.array([r["world_xy_m"] for r in cs])-origin).T, c="black", label="Measured E2E vehicle")
    for i, t in enumerate((65., 70., 75., 80., 85., 90., 95., 100.)):
        row = min(ps, key=lambda r: abs(r["time_s"]-t)); xy = np.array(row["world_xy_m"])-origin
        ax.plot(*xy.T, ".-", ms=2, color="#ce318b", label="Original model 3s predictions" if i == 0 else None)
        ax.annotate(f"{row['time_s']:.1f}s", xy[0], xytext=(6, 6), textcoords="offset points", fontsize=8)
    ax.plot(0., 0., "rx", ms=9, label="First monitor rejection")
    ax.set_aspect("equal"); ax.grid(alpha=.2); ax.legend(fontsize=9)
    ax.set(xlabel="Map X relative to rejection [m]", ylabel="Map Y relative to rejection [m]",
           title="Measured normal line, E2E travel and unmodified predictions\nNo map registration or synthetic recovery labels")
    fig.tight_layout(); fig.savefig(out/"attribution_paths.png", dpi=150); plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trial", type=Path, required=True)
    parser.add_argument("--reference-runs", type=Path, nargs=2, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(); out = args.output; out.mkdir(parents=True, exist_ok=False)
    hashes = verify_files(args.trial, ("control.jsonl", "inference.jsonl", "vehicle_observations.jsonl", "trial_config.json", "host_result.json"))
    config = read(args.trial/"trial_config.json")
    if config["lookahead_policy"] != "stopping_preview_extended_v1":
        raise ValueError("EXPECTED_NORMAL_LAP_POLICY")
    rows = records(args.trial/"control.jsonl")
    armed = next(r for r in rows if r["event"] == "ARMED")
    fault = next(r for r in rows if r["event"] == "SCAN_GUARD_REJECTED")
    fault_command = next(r for r in rows if r["event"] == "COMMAND_SENT" and r["reason"] == fault["reason"])
    commands = [r for r in rows if r["event"] == "COMMAND_SENT" and r["monotonic_ns"] <= fault_command["monotonic_ns"]
                and r["sim_ns"] >= armed["sim_ns"]+55_000_000_000 and r.get("details", {}).get("current_pose")]
    plans = {r["plan_id"]: r for r in records(args.trial/"inference.jsonl") if r["event"] == "PLAN"}
    pose_index = runtime_poses(args.trial/"vehicle_observations.jsonl")
    summary = {"source_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
        "scope": "RECORDED_NORMAL_LAP_ATTRIBUTION_NOT_CAUSAL_PERCENTAGES_OR_NEW_DRIVE_TEST",
        "runtime_modified": False, "new_inference": False, "reserved_recovery_test_read": False,
        "trial": args.trial.name, "trial_hashes": hashes, "references": {},
        "first_fault_after_arm_s": (fault["sim_ns"]-armed["sim_ns"])/1e9,
        "runtime_pose_ambiguities": len(pose_index.ambiguous)}
    previous = None
    for reference_run in args.reference_runs:
        reference, meta = measured_reference(reference_run, config)
        cr, pr = compare(reference, commands, plans, pose_index, config, armed["sim_ns"], fault["sim_ns"])
        valid = [r for r in cr if r["status"] == "EVALUATED"]
        meta.update(command_status_counts=dict(Counter(r["status"] for r in cr)),
            plan_status_counts=dict(Counter(r["status"] for r in pr)), groups=group_metrics(cr, pr),
            snapshots=[min(valid, key=lambda r: abs(r["time_s"]-t)) for t in (65., 70., 75., 80., 85., 90., 95., 100.35)])
        if previous is None:
            plots(out, reference, cr, pr); previous = reference
        else:
            interior = (reference.arc > 2.) & (reference.arc < reference.arc[-1]-2.)
            differences = [previous.project(p, max_distance_m=.2).distance_m for p in reference.xy[interior][::20]]
            meta["normal_line_repeat_difference_m"] = stats(differences)
        summary["references"][reference_run.name] = meta
        write(out/(reference_run.name+"_commands.json"), cr)
        write(out/(reference_run.name+"_plans.json"), pr)
        print(json.dumps({"reference": reference_run.name, "commands": len(cr), "plans": len(pr), "status": meta["command_status_counts"]}), flush=True)
    motion = fault["motion_observation"]
    summary["first_scan_replay"] = scan_margin(fault["scan"], fault["scan_in_current_rear"],
        speed_mps=fault["speed_mps"], measured_rad=fault["measured_steer_rad"], issued_rad=fault["issued_steer_rad"],
        previous_rad=fault["previous_steer_rad"], yaw_rate_radps=motion["heading_rate_radps"], lateral_mps=motion["reported_lateral_mps"])
    if summary["first_scan_replay"]["reason"] != fault["reason"]:
        raise ValueError("SCAN_REPLAY_MISMATCH")
    write(out/"summary.json", summary)
    print(json.dumps({"status": "COMPLETE_RECORDED_COMPARISON", "output": str(out)}), flush=True)


if __name__ == "__main__":
    main()
