"""Compare the completed segment trial with measured teacher lines in native WSL.

No fitting of map frames, no synthetic recovery labels, no test split reads.
Nominal validation predictions are the saved epoch-3 outputs of the deployed
checkpoint; hashes are recorded now, not claimed as historical file seals.
"""
from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict
import json
import math
from pathlib import Path
import subprocess
from typing import Any

import numpy as np

from aic_transfuser_lite.control.time_reference_v1 import TimePlan, TimedBodyPose
from aic_transfuser_lite.control.time_trial_v1 import time_trial_control
from aic_transfuser_lite.data.time_split_v1 import content_sha256, validate_time_split
from aic_transfuser_lite.evaluation.time_clearance_v1 import PoseIndex
from aic_transfuser_lite.evaluation.time_corner_comparison_v1 import (
    RecordedLine, angle_delta, future_tracking_error, replay_observation_pose, world_points)
from compare_time_teacher_clearance import Bag, records, read, sha, stamp, stats, write


NOMINAL = ("5kmh_run03", "5kmh_run06", "8kmh_run06", "8kmh_run09")
SAME_HOST = ("codex-time-recovery-right040-r22", "codex-time-recovery-left020-r23")
PRIMARY = SAME_HOST[1]


def anchor_observation(bag: Bag, anchor: dict[str, Any]) -> TimedBodyPose:
    sources = []
    ids = anchor["observation_pose_row_ids"]
    if len(set(ids)) != len(ids):
        raise ValueError("ANCHOR_POSE_DUPLICATE_ROW_ID")
    for row_id in ids:
        message, receipt = bag.message(row_id)
        if message.__msgtype__ != "nav_msgs/msg/Odometry":
            raise ValueError("ANCHOR_POSE_NOT_ODOMETRY")
        p, q = message.pose.pose.position, message.pose.pose.orientation
        sources.append((TimedBodyPose(stamp(message), "sim", "0", message.header.frame_id, message.child_frame_id,
            p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))), receipt))
    return replay_observation_pose(sources, observation_ns=anchor["observation_ns"], freeze_receipt_ns=anchor["freeze_ns"])


def runtime_poses(path: Path) -> PoseIndex:
    result = []
    for row in records(path):
        if row.get("role") != "pose":
            continue
        x, y, z, w = row["quaternion_xyzw"]
        result.append(TimedBodyPose(row["stamp_ns"], "sim", row["epoch"], row["frame"], row["child_frame"],
            *row["position_xyz_m"][:2], math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))))
    return PoseIndex(result)


def compact_stats(rows: list[dict[str, Any]], keys: tuple[str, ...]) -> dict[str, Any]:
    return {k: stats([r[k] for r in rows if k in r and r[k] is not None]) for k in keys}


def matched_scan_cloud(bag: Bag, center_ns: int, rejection: dict[str, Any], current: TimedBodyPose,
                       rear_offset_m: float) -> dict[str, Any]:
    """Nearest observed returns in original map coordinates, without registration."""
    from scipy.spatial import cKDTree
    topic = [(i, typ) for name, (i, typ) in bag.topics.items() if typ == "sensor_msgs/msg/LaserScan"]
    if len(topic) != 1:
        raise ValueError("UNIQUE_SCAN_TOPIC_REQUIRED")
    cloud, row_ids = [], []
    excluded: Counter[str] = Counter()
    for row_id, data in bag.con.execute("select id,data from messages where topic_id=?", (topic[0][0],)):
        message = bag.store.deserialize_cdr(data, topic[0][1])
        if abs(stamp(message)-center_ns) > 1_000_000_000:
            continue
        try:
            pose = bag.poses.at(stamp(message))
        except ValueError as exc:
            excluded[str(exc)] += 1
            continue
        if message.header.frame_id != "lidar":
            raise ValueError("TEACHER_SCAN_FRAME")
        ranges = np.asarray(message.ranges, float)
        angles = message.angle_min+np.arange(len(ranges))*message.angle_increment
        keep = np.isfinite(ranges) & (ranges >= message.range_min) & (ranges <= message.range_max)
        points = ranges[keep, None]*np.column_stack([np.cos(angles[keep]), np.sin(angles[keep])])
        cloud.append(world_points(points+[1.65, 0.], pose))
        row_ids.append(row_id)
    if not cloud:
        raise ValueError("NO_MATCHED_TEACHER_SCANS")
    scan = rejection["scan"]
    sensor = rejection["scan_in_current_rear"]
    ranges = np.asarray(scan["ranges"], float)
    angles = scan["angle_min"]+np.arange(len(ranges))*scan["angle_increment"]+sensor[2]
    keep = np.isfinite(ranges) & (ranges >= scan["range_min"]) & (ranges <= min(6., scan["range_max"]))
    points = np.array(sensor[:2])+ranges[keep, None]*np.column_stack([np.cos(angles[keep]), np.sin(angles[keep])])
    target = world_points(points+[rear_offset_m, 0.], current)
    distances, _ = cKDTree(np.concatenate(cloud)).query(target)
    return {"method": "nearest_returns_no_fitted_registration_not_ground_truth",
        "teacher_scan_row_ids": row_ids, "runtime_returns": len(target), "teacher_exclusions": dict(excluded),
        "distance_m": stats(distances.tolist()), "within_0p1m_fraction": float(np.mean(distances <= .1))}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    root, out = args.root.resolve(), args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    cache = root/"datasets/cache/time_recovery_20260914"
    corpus = root/"datasets/processed/time_teacher_20laps_20260913"
    training = root/"runs/time_recovery_finetune_20260914"
    trial = root/"runs/time_segment_awsim_evidence_20260914/codex-time-segment01"
    identity = read(cache/"identity.json")
    if identity["manifest_sha256"] != content_sha256({k: v for k, v in identity.items() if k != "manifest_sha256"}):
        raise ValueError("CACHE_IDENTITY_MISMATCH")
    validate_time_split(identity["split_manifest"], require_verified=True)
    metadata = {r["run_id"]: r for r in identity["split_manifest"]["runs"]}
    expected_files = {r["path"]: r["sha256"] for r in identity["cache_files"]}
    config = read(trial/"trial_config.json")
    controls = records(trial/"control.jsonl")
    inference = records(trial/"inference.jsonl")
    ready = next(r for r in inference if r["event"] == "MODEL_READY")
    trained = read(training/"result.json")
    if trained["best_epoch"] != 3 or sha(training/"best.pt") != ready["checkpoint_sha256"]:
        raise ValueError("DEPLOYED_CHECKPOINT_MISMATCH")
    predictions_path = training/"validation_epoch_03.npy"
    predictions = np.load(predictions_path, mmap_mode="r", allow_pickle=False)
    offsets, count = {}, 0
    for run in identity["runs"]:
        if run["split"] == "validation":
            offsets[run["run_id"]] = count
            count += run["anchors"]
    if predictions.shape != (count, 30, 2):
        raise ValueError("VALIDATION_PREDICTION_ORDER_OR_SHAPE")
    arm = next(r for r in controls if r["event"] == "ARMED")
    fault = next(r for r in controls if r["event"] == "COMMAND_SENT" and r["reason"] == "STOPPING_SWEEP_OCCUPIED")
    rejection = next(r for r in controls if r["event"] == "SCAN_GUARD_REJECTED")
    current_fault = TimedBodyPose(**fault["details"]["current_pose"])
    commands = [r for r in controls if r["event"] == "COMMAND_SENT"
        and arm["monotonic_ns"] <= r["monotonic_ns"] <= fault["monotonic_ns"]
        and "current_pose" in r.get("details", {})]
    pose_index = runtime_poses(trial/"vehicle_observations.jsonl")
    plans_by_id = {r["plan_id"]: r for r in inference if r["event"] == "PLAN"}
    issued: dict[str, dict[str, Any]] = {}
    for r in commands:
        if r.get("plan_id") and "observation_pose" in r["details"]:
            if r["plan_id"] in issued and issued[r["plan_id"]]["details"]["observation_pose"] != r["details"]["observation_pose"]:
                raise ValueError("PLAN_OBSERVATION_CHANGED")
            issued.setdefault(r["plan_id"], r)
    plan_rows = []
    for plan_id, command in issued.items():
        plan = plans_by_id[plan_id]
        obs = TimedBodyPose(**command["details"]["observation_pose"])
        if obs.stamp_ns != plan["observation_ns"] or plan["checkpoint_sha256"] != ready["checkpoint_sha256"]:
            raise ValueError("PLAN_PROVENANCE_MISMATCH")
        row = {"plan_id": plan_id, "time_to_fault_s": (obs.stamp_ns-fault["sim_ns"])/1e9,
               "observation": asdict(obs), "world_xy_m": world_points(np.asarray(plan["raw_xy_m"]), obs).tolist(),
               "following": {}}
        for horizon in (1., 2., 3.):
            try:
                row["following"][str(int(horizon))] = {"reason": "EVALUATED", **future_tracking_error(
                    np.asarray(plan["raw_xy_m"]), obs, pose_index, horizon, before_ns=fault["sim_ns"])}
            except ValueError as exc:
                row["following"][str(int(horizon))] = {"reason": str(exc)}
        plan_rows.append(row)
    summary: dict[str, Any] = {"source_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
        "scope": "recorded_localization_and_prediction_diagnostics_not_causal_or_collision_proof",
        "runtime_modified": False, "test_evaluated": False, "map_registration_fitted": False,
        "reference_runs": list(NOMINAL+SAME_HOST), "primary_reference": PRIMARY,
        "prediction_file_provenance": "saved_epoch3_validation_array_current_hash_and_checkpoint_identity; historical_byte_seal_unavailable",
        "checkpoint_sha256": ready["checkpoint_sha256"], "cache_manifest_sha256": identity["manifest_sha256"],
        "input_hashes": {str(p.relative_to(root)): sha(p) for p in [cache/"identity.json", training/"result.json",
             predictions_path, trial/"control.jsonl", trial/"inference.jsonl", trial/"vehicle_observations.jsonl"]},
        "fault_after_arm_s": (fault["sim_ns"]-arm["sim_ns"])/1e9,
        "fault_pose": asdict(current_fault), "command_count": len(commands), "used_plan_count": len(plan_rows),
        "ambiguous_runtime_pose_stamps": len(pose_index.ambiguous), "references": {}}
    lines, centers, plot_rows = {}, {}, {}
    for run_id in NOMINAL+SAME_HOST:
        if metadata[run_id]["split"] != "validation":
            raise ValueError("REFERENCE_SPLIT_CHANGED")
        raw = corpus/"validation"/run_id/"raw" if run_id in NOMINAL else cache/"materialized"/run_id/"raw"
        expected_bag = next(s["sha256"] for s in metadata[run_id]["sources"] if s["path"].endswith("bag_0.db3"))
        actual_bag = sha(raw/"bag/bag_0.db3")
        if actual_bag != expected_bag:
            raise ValueError("TEACHER_BAG_HASH_MISMATCH:"+run_id)
        bag = Bag(raw)
        reference = RecordedLine(bag.poses.rows)
        # Preserve the raw reader's ambiguity rule; no interpolated brackets may cross it.
        reference.usable &= np.array([a.stamp_ns not in bag.poses.ambiguous and b.stamp_ns not in bag.poses.ambiguous
                                      for a, b in zip(reference.rows, reference.rows[1:])])
        center = reference.project(np.array([current_fault.x_m, current_fault.y_m]), yaw_hint_rad=current_fault.yaw_rad)
        bounds = (max(0., center.progress_m-130.), min(float(reference.arc[-1]), center.progress_m+12.))
        lines[run_id], centers[run_id] = reference, center
        run_rows = []
        for command in commands:
            pose = TimedBodyPose(**command["details"]["current_pose"])
            row = {"time_to_fault_s": (command["sim_ns"]-fault["sim_ns"])/1e9,
                   "time_after_arm_s": (command["sim_ns"]-arm["sim_ns"])/1e9, "reason": command["reason"],
                   "world_xy_m": [pose.x_m, pose.y_m], "speed_mps": command["speed_mps"]}
            try:
                projected = reference.project(np.array([pose.x_m, pose.y_m]), yaw_hint_rad=pose.yaw_rad, progress_bounds_m=bounds)
                row.update(projection=asdict(projected), left_m=projected.left_m,
                    progress_to_fault_m=projected.progress_m-center.progress_m,
                    body_yaw_error_rad=angle_delta(pose.yaw_rad, projected.body_yaw_rad),
                    path_yaw_error_rad=angle_delta(pose.yaw_rad, projected.tangent_yaw_rad), status="EVALUATED")
                d = command["details"]
                if "steer_rad" in d:
                    row.update(nominal_tire_rad=d["steer_rad"], measured_tire_rad=command["measured_steer_rad"],
                        tire_error_rad=command["measured_steer_rad"]-d["steer_rad"],
                        plan_age_s=(pose.stamp_ns-d["observation_pose"]["stamp_ns"])/1e9)
            except ValueError as exc:
                row["status"] = str(exc)
            run_rows.append(row)
        projections = []
        for p in plan_rows:
            obs = TimedBodyPose(**p["observation"])
            row = {"plan_id": p["plan_id"], "time_to_fault_s": p["time_to_fault_s"]}
            try:
                anchor = reference.project(np.array([obs.x_m, obs.y_m]), yaw_hint_rad=obs.yaw_rad, progress_bounds_m=bounds)
                row.update(anchor_left_m=anchor.left_m, anchor_progress_to_fault_m=anchor.progress_m-center.progress_m,
                           heading_error_rad=angle_delta(obs.yaw_rad, anchor.body_yaw_rad), horizons={}, status="EVALUATED")
                for horizon in (1, 2, 3):
                    point = np.asarray(p["world_xy_m"][horizon*10-1])
                    target = reference.project(point, progress_bounds_m=(max(0., anchor.progress_m-2.),
                        min(float(reference.arc[-1]), anchor.progress_m+10.)), max_distance_m=5.)
                    row["horizons"][str(horizon)] = {"left_m": target.left_m,
                        "change_from_anchor_left_m": target.left_m-anchor.left_m,
                        "forward_progress_m": target.progress_m-anchor.progress_m}
            except ValueError as exc:
                row["status"] = str(exc)
            projections.append(row)
        cp = cache/"validation"/run_id
        cache_hashes = {}
        for name in ("anchors.jsonl", "labels.npz", "inputs.npz"):
            actual = sha(cp/name)
            if actual != expected_files[f"validation/{run_id}/{name}"]:
                raise ValueError("CACHE_FILE_HASH_MISMATCH:"+run_id+":"+name)
            cache_hashes[name] = actual
        anchors = records(cp/"anchors.jsonl")
        with np.load(cp/"labels.npz") as z:
            labels, masks = z["xy_m"], z["xy_mask"]
        with np.load(cp/"inputs.npz") as z:
            valid_inputs = z["input_valid"]
        evaluated = []
        for i, a in enumerate(anchors):
            progress = float(np.interp(a["observation_ns"], reference.times, reference.arc)-center.progress_m)
            if not -40. <= progress <= 8.:
                continue
            row = {"anchor_id": a["anchor_id"], "progress_to_fault_m": progress}
            try:
                if not valid_inputs[i] or not masks[i].all():
                    raise ValueError("INPUT_OR_FULL_TEACHER_INVALID")
                obs = anchor_observation(bag, a)
                pred = np.asarray(predictions[offsets[run_id]+i])
                if not np.isfinite(pred).all():
                    raise ValueError("INVALID_SAVED_PREDICTION")
                velocity = bag.state(a, "velocity")
                error = pred-labels[i]
                row.update(status="EVALUATED", speed_mps=float(velocity.longitudinal_velocity),
                    ade_m=float(np.linalg.norm(error, axis=1).mean()), fde1_m=float(np.linalg.norm(error[9])),
                    fde2_m=float(np.linalg.norm(error[19])), fde3_m=float(np.linalg.norm(error[29])),
                    endpoint_body_left_error_m=float(error[29, 1]), pp={})
                for condition, speed in (("actual_teacher_speed", float(velocity.longitudinal_velocity)),
                                         ("common_runtime_speed", fault["speed_mps"])):
                    values = {}
                    for kind, xy in (("teacher", labels[i]), ("prediction", pred)):
                        try:
                            ctrl = time_trial_control(TimePlan(a["anchor_id"], obs, xy), obs,
                                speed_mps=speed, rear_axle_offset_m=(config["geometry"]["rear_axle_forward_in_base_link_m"], 0.),
                                speed_policy=config["speed_policy"], lookahead_policy=config["lookahead_policy"],
                                vehicle_model_policy=config["vehicle_model_policy"])
                            values[kind] = {"status": "PASS", "tire_rad": ctrl["steer_rad"], "lookahead_m": ctrl["lookahead_rear_m"]}
                        except ValueError as exc:
                            values[kind] = {"status": str(exc)}
                    if all(v["status"] == "PASS" for v in values.values()):
                        values["prediction_minus_teacher_rad"] = values["prediction"]["tire_rad"]-values["teacher"]["tire_rad"]
                    row["pp"][condition] = values
            except ValueError as exc:
                row["status"] = str(exc)
            evaluated.append(row)
        valid = [r for r in run_rows if r["status"] == "EVALUATED"]
        valid_pred = [r for r in projections if r["status"] == "EVALUATED"]
        valid_offline = [r for r in evaluated if r["status"] == "EVALUATED"]
        summary["references"][run_id] = {
            "host": "192.168.3.13" if run_id in NOMINAL else "192.168.3.10", "split": "validation",
            "bag_sha256": actual_bag, "cache_hashes": cache_hashes, "pose_audit": reference.audit,
            "raw_ambiguous_pose_stamps": len(bag.poses.ambiguous), "fault_projection": asdict(center),
            "fault_body_yaw_error_rad": angle_delta(current_fault.yaw_rad, center.body_yaw_rad),
            "command_status_counts": dict(Counter(r["status"] for r in run_rows)),
            "prediction_status_counts": dict(Counter(r["status"] for r in projections)),
            "last30s": compact_stats([r for r in valid if r["time_to_fault_s"] >= -30.],
                                      ("left_m", "body_yaw_error_rad", "tire_error_rad", "speed_mps", "plan_age_s")),
            "snapshots": [min(valid, key=lambda r: abs(r["time_to_fault_s"]-t)) for t in (-60., -40., -30., -20., -15., -10., -5., 0.)],
            "offline_counts": dict(Counter(r["status"] for r in evaluated)),
            "offline_errors": compact_stats(valid_offline, ("ade_m", "fde1_m", "fde2_m", "fde3_m", "endpoint_body_left_error_m", "speed_mps")),
            "offline_common_speed_pp_delta_rad": stats([r["pp"]["common_runtime_speed"]["prediction_minus_teacher_rad"]
                 for r in valid_offline if "prediction_minus_teacher_rad" in r["pp"]["common_runtime_speed"]]),
            "offline_common_speed_pp_status": {kind: dict(Counter(r["pp"]["common_runtime_speed"][kind]["status"]
                 for r in valid_offline)) for kind in ("teacher", "prediction")},
            "scan_alignment_at_fault": matched_scan_cloud(bag, center.stamp_ns, rejection, current_fault,
                config["geometry"]["rear_axle_forward_in_base_link_m"])}
        if run_id in SAME_HOST:
            raw_row = next(r for r in identity["plan"]["runs"] if r["run_id"] == run_id)
            source = root/identity["plan"]["raw_roots"][raw_row["root"]]/run_id
            transferred = read(source/"transfer_manifest.json")
            if sha(source/"control.jsonl") != transferred["control.jsonl"]["sha256"]:
                raise ValueError("RECOVERY_PHASE_HASH_MISMATCH")
            phase_rows = [r for r in records(source/"control.jsonl") if r.get("sim_ns") is not None and "phase" in r]
            phase = min(phase_rows, key=lambda r: abs(r["sim_ns"]-center.stamp_ns))
            summary["references"][run_id]["matched_recovery_phase"] = {"phase": phase["phase"],
                "stamp_delta_s": (phase["sim_ns"]-center.stamp_ns)/1e9,
                "control_sha256": sha(source/"control.jsonl")}
        write(out/(run_id+"_commands.json"), run_rows)
        write(out/(run_id+"_predictions.json"), projections)
        write(out/(run_id+"_offline.json"), evaluated)
        plot_rows[run_id] = (valid, valid_pred, valid_offline)
        bag.con.close()
        print(json.dumps({"reference_done": run_id, "fault_left_m": center.left_m,
                          "offline_anchors": len(valid_offline)}), flush=True)
    primary = lines[PRIMARY]
    primary_center = centers[PRIMARY]
    coverage = []
    # Audit all accepted recovery anchors, including train, without re-inference.
    for spec in identity["plan"]["runs"]:
        run_id, split = spec["run_id"], spec["split"]
        if split not in ("train", "validation"):
            raise ValueError("RECOVERY_TEST_FORBIDDEN")
        source = cache/"materialized"/run_id/"raw"
        expected = next(r["sha256"] for r in metadata[run_id]["sources"] if r["path"].endswith("bag_0.db3"))
        if sha(source/"bag/bag_0.db3") != expected:
            raise ValueError("COVERAGE_BAG_HASH_MISMATCH")
        bag = Bag(source)
        apath = cache/split/run_id/"anchors.jsonl"
        if sha(apath) != expected_files[f"{split}/{run_id}/anchors.jsonl"]:
            raise ValueError("COVERAGE_ANCHOR_HASH_MISMATCH")
        rows = []
        for a in records(apath):
            p = anchor_observation(bag, a)
            result = primary.project(np.array([p.x_m, p.y_m]), yaw_hint_rad=p.yaw_rad, max_distance_m=5.)
            rows.append({"anchor_id": a["anchor_id"], "progress_to_fault_m": result.progress_m-primary_center.progress_m,
                         "left_m": result.left_m, "map_xy_m": [p.x_m, p.y_m]})
        coverage.append({"run_id": run_id, "split": split, "accepted_anchors": len(rows),
            "bag_sha256": expected, "anchors_sha256": sha(apath),
            "progress_to_fault_m": stats([r["progress_to_fault_m"] for r in rows]),
            "in_fault_minus40_plus8m": sum(-40. <= r["progress_to_fault_m"] <= 8. for r in rows)})
        write(out/(run_id+"_recovery_coverage.json"), rows)
        bag.con.close()
    summary["recovery_anchor_coverage"] = coverage
    selected_plans = [p for p in plan_rows if p["time_to_fault_s"] >= -30.]
    summary["last30s_future_following"] = {}
    for h in ("1", "2", "3"):
        rows = [p["following"][h] for p in selected_plans]
        valid = [r for r in rows if r["reason"] == "EVALUATED"]
        summary["last30s_future_following"][h] = {"counts": dict(Counter(r["reason"] for r in rows)),
            "metrics": compact_stats(valid, ("point_distance_m", "point_left_m", "point_along_m", "polyline_distance_m")),
            "polyline_endpoint_count": sum(r["polyline_endpoint"] for r in valid)}
    _, primary_pred, _ = plot_rows[PRIMARY]
    summary["primary_prediction_windows"] = []
    for low, high in ((-40., -30.), (-30., -20.), (-20., -10.), (-10., -5.), (-5., 0.)):
        rows = [p for p in primary_pred if low <= p["time_to_fault_s"] < high]
        summary["primary_prediction_windows"].append({"window_s": [low, high], "plans": len(rows),
            "anchor_left_m": stats([p["anchor_left_m"] for p in rows]),
            "heading_error_rad": stats([p["heading_error_rad"] for p in rows]),
            "future": {h: {k: stats([p["horizons"][h][k] for p in rows]) for k in
                ("left_m", "change_from_anchor_left_m", "forward_progress_m")} for h in ("1", "2", "3")}})
    write(out/"runtime_plans.json", plan_rows)
    write(out/"summary.json", summary)
    render(out, lines, centers, plot_rows, plan_rows, current_fault)
    print(json.dumps({"status": "COMPLETE", "output": str(out)}), flush=True)


def render(out: Path, lines: dict[str, RecordedLine], centers: dict[str, Any],
           rows: dict[str, Any], plans: list[dict[str, Any]], fault: TimedBodyPose) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    origin = np.array([fault.x_m, fault.y_m])
    actual, predicted, _ = rows[PRIMARY]
    fig, axes = plt.subplots(1, 2, figsize=(14, 7))
    for ax, window in zip(axes, (40., 12.)):
        for run_id, line in lines.items():
            keep = (line.arc >= centers[run_id].progress_m-window) & (line.arc <= centers[run_id].progress_m+8.)
            ax.plot(*(line.xy[keep]-origin).T, lw=1.1, alpha=.8, label=run_id)
        trace = [r for r in actual if r["progress_to_fault_m"] >= -window]
        ax.plot(*(np.array([r["world_xy_m"] for r in trace])-origin).T, c="#b52d7e", lw=2.5, label="Actual E2E localization")
        for t in ((-25., -15., -8., -.2) if window == 40. else (-8., -4., -.2)):
            plan = min(plans, key=lambda p: abs(p["time_to_fault_s"]-t))
            xy = np.vstack([[plan["observation"]["x_m"], plan["observation"]["y_m"]], plan["world_xy_m"]])-origin
            ax.plot(*xy.T, c="#333333", lw=1.3, ls="--")
            ax.annotate(f'{plan["time_to_fault_s"]:.1f}s', xy[0], xytext=(4, 4), textcoords="offset points", fontsize=8)
        ax.scatter(0., 0., marker="x", c="red", s=80, label="First guard rejection")
        ax.set_aspect("equal"); ax.grid(alpha=.2)
        ax.set_xlabel("Map x relative to rejection (m)"); ax.set_ylabel("Map y relative to rejection (m)")
        ax.set_title(f"Recorded lines: last {window:.0f} m; dashed = unchanged predictions")
    axes[0].legend(fontsize=7, loc="best")
    fig.suptitle("Same map coordinates; no fitted registration / estimated poses, not ground truth")
    fig.tight_layout(); fig.savefig(out/"corner_paths.png", dpi=160); plt.close(fig)
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    late = [r for r in actual if r["time_to_fault_s"] >= -40.]
    pred = [r for r in predicted if r["time_to_fault_s"] >= -40.]
    axes[0].plot([r["time_to_fault_s"] for r in late], [r["left_m"] for r in late], label="Actual offset", c="#b52d7e", lw=2)
    for h in ("1", "3"):
        axes[0].plot([r["time_to_fault_s"] for r in pred], [r["horizons"][h]["left_m"] for r in pred], label=f"Predicted offset at +{h} s", alpha=.8)
    axes[0].set_ylabel("Left offset to measured teacher (m)")
    axes[1].plot([r["time_to_fault_s"] for r in late], [math.degrees(r["body_yaw_error_rad"]) for r in late], label="Actual heading minus teacher heading")
    axes[1].set_ylabel("Heading error (degrees)")
    steering = [r for r in late if "nominal_tire_rad" in r]
    for key, label in (("nominal_tire_rad", "Nominal PP tire request"), ("measured_tire_rad", "Measured tire angle")):
        axes[2].plot([r["time_to_fault_s"] for r in steering], [r[key] for r in steering], label=label)
    axes[2].set_ylabel("Tire angle (rad)"); axes[2].set_xlabel("Simulation seconds relative to first guard rejection")
    for ax in axes:
        ax.axhline(0., c="black", lw=.6); ax.axvline(0., c="red", ls=":"); ax.grid(alpha=.2); ax.legend(fontsize=8)
    fig.suptitle("Reference: same-host validation run r23, matched by position, not elapsed time")
    fig.tight_layout(); fig.savefig(out/"corner_timeline.png", dpi=160); plt.close(fig)


if __name__ == "__main__":
    main()
