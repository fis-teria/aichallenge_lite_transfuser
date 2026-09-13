"""Compare sealed-split-safe teacher/FP32 predictions with recorded turn16.

Offline only. See docs/time_teacher_clearance_comparison_20260913.md for the
predeclared population, normalization and limits of this counterfactual.
"""
from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sqlite3
from typing import Any

import numpy as np

from aic_transfuser_lite.control.time_reference_v1 import TimePlan, TimedBodyPose
from aic_transfuser_lite.control.time_trial_v1 import time_trial_control
from aic_transfuser_lite.control.turning_scan_guard import scan_pose_in_rear
from aic_transfuser_lite.control.vehicle_motion_v1 import AWSIM_POLICY, COM_FORWARD_OF_REAR_M
from aic_transfuser_lite.data.time_split_v1 import content_sha256, validate_time_split
from aic_transfuser_lite.data.time_sqlite_reader_v1 import _store
from aic_transfuser_lite.evaluation.time_clearance_v1 import PoseIndex, project_to_polyline, scan_margin

REAR_OFFSET_M = .0010000169277191162
RUN_IDS = tuple(f"5kmh_run{i:02d}" for i in [1, 2, 3, 4, 5, 6, 7, 9])


def read(path: Path) -> Any:
    return json.loads(path.read_text())


def records(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in path.open()]


def write(path: Path, value: Any) -> None:
    with path.open("x") as stream:
        json.dump(value, stream, indent=2, allow_nan=False)


def sha(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4*1024*1024), b""):
            h.update(block)
    return h.hexdigest()


def stamp(message: Any) -> int:
    s = message.header.stamp if hasattr(message, "header") else message.stamp
    return int(s.sec)*1_000_000_000+int(s.nanosec)


def body_pose(msg: Any) -> TimedBodyPose:
    q, p = msg.pose.orientation, msg.pose.position
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    return TimedBodyPose(stamp(msg), "sim", "0", msg.header.frame_id, "base_link", p.x, p.y, yaw)


def stats(values: list[float]) -> dict[str, Any]:
    if not values:
        return {"count": 0}
    a = np.asarray(values)
    return {"count": len(a), "min": float(a.min()), "median": float(np.median(a)),
            "p95": float(np.quantile(a, .95)), "max": float(a.max()), "mean": float(a.mean())}


class Bag:
    def __init__(self, root: Path) -> None:
        self.store = _store(root)
        self.con = sqlite3.connect(f"file:{(root/'bag/bag_0.db3').resolve()}?mode=ro&immutable=1", uri=True)
        self.topics = {name: (i, typ) for i, name, typ in self.con.execute("select id,name,type from topics")}
        self.types = {i: typ for i, typ in self.topics.values()}
        self.small: dict[int, tuple[Any, int]] = {}
        poses = []
        for name in ["/localization/pose", "/vehicle/status/velocity_status", "/vehicle/status/steering_status"]:
            i, typ = self.topics[name]
            for row, receipt, data in self.con.execute("select id,timestamp,data from messages where topic_id=?", (i,)):
                msg = self.store.deserialize_cdr(data, typ)
                self.small[row] = msg, receipt
                if name == "/localization/pose":
                    poses.append(body_pose(msg))
        self.poses = PoseIndex(poses)
        self.tf: dict[str, Any] = {}
        i, typ = self.topics["/tf_static"]
        for data, in self.con.execute("select data from messages where topic_id=?", (i,)):
            for t in self.store.deserialize_cdr(data, typ).transforms:
                v, q = t.transform.translation, t.transform.rotation
                key = t.header.frame_id+"->"+t.child_frame_id
                value = {"xyz_m": [v.x, v.y, v.z], "quaternion_xyzw": [q.x, q.y, q.z, q.w]}
                if key in self.tf and self.tf[key] != value:
                    raise ValueError("TF_STATIC_CHANGED:"+key)
                self.tf[key] = value
        if self.tf["base_link->lidar_base_link"] != {
                "xyz_m": [1.65, 0., 0.], "quaternion_xyzw": [0., 0., 0., 1.]}:
            raise ValueError("TEACHER_LIDAR_EXTRINSIC_DIFFERS")

    def message(self, row: int) -> tuple[Any, int]:
        if row in self.small:
            return self.small[row]
        result = self.con.execute("select topic_id,timestamp,data from messages where id=?", (int(row),)).fetchone()
        if result is None:
            raise ValueError("MESSAGE_ROW_MISSING")
        topic, receipt, data = result
        return self.store.deserialize_cdr(data, self.types[topic]), receipt

    def state(self, anchor: dict[str, Any], name: str) -> Any:
        ids = {i for group in anchor["history_row_ids"][name] for i in group}
        valid = [msg for i in ids for msg, receipt in [self.message(i)]
                 if receipt <= anchor["freeze_ns"] and 0 <= anchor["observation_ns"]-stamp(msg) <= 50_000_000]
        if not valid:
            raise ValueError("FRESH_STATE_MISSING:"+name)
        times = [stamp(x) for x in valid]
        t = max(times)
        if times.count(t) > 1:
            raise ValueError("STATE_STAMP_AMBIGUOUS:"+name)
        return valid[times.index(t)]


def compare_path(xy: np.ndarray, current: TimedBodyPose, scan: dict[str, Any], sensor: tuple[float, ...],
                 state: dict[str, float], anchor_id: str) -> dict[str, Any]:
    try:
        pp = time_trial_control(TimePlan(anchor_id, current, xy), current, speed_mps=state["speed_mps"],
            rear_axle_offset_m=(REAR_OFFSET_M, 0.), speed_policy="fixed_5kmh",
            lookahead_policy="stopping_preview_v1", vehicle_model_policy=AWSIM_POLICY)
    except ValueError as exc:
        return {"reason": "PP:"+str(exc), "minimum_ray_margin_m": None}
    result = scan_margin(scan, sensor, issued_rad=pp["steer_rad"], **state)
    return {**result, "pp_physical_steer_rad": pp["steer_rad"], "lookahead_rear_m": pp["lookahead_rear_m"]}


def to_world(points: np.ndarray, pose: TimedBodyPose) -> np.ndarray:
    """Base-link points [N,2] metres to the pose's unchanged world frame."""
    points = np.asarray(points, float)
    if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all():
        raise ValueError("WORLD_POINTS_SHAPE_OR_NONFINITE")
    c, s = math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)
    return points @ np.array([[c, s], [-s, c]])+np.array([pose.x_m, pose.y_m])


def scan_alignment(bag: Bag, rows: list[dict[str, Any]], center_ns: int,
                   reject: dict[str, Any], fault: TimedBodyPose) -> dict[str, Any]:
    """Nearest observed returns in unchanged map coordinates; no fitted registration."""
    from scipy.spatial import cKDTree
    cloud, ids = [], set()
    for row in rows:
        if row["reason"] != "EVALUATED" or abs(row["observation_ns"]-center_ns) > 1_000_000_000:
            continue
        if row["scan_row_id"] in ids:
            continue
        ids.add(row["scan_row_id"])
        msg, _ = bag.message(row["scan_row_id"])
        pose = bag.poses.at(stamp(msg))
        ranges = np.asarray(msg.ranges, float)
        angles = msg.angle_min+np.arange(len(ranges))*msg.angle_increment
        keep = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        points = ranges[keep, None]*np.column_stack([np.cos(angles[keep]), np.sin(angles[keep])])
        cloud.append(to_world(points+np.array([1.649999976158142, 0.]), pose))
    if not cloud:
        return {"reason": "NO_MATCHED_SCAN_CLOUD"}
    sensor = np.asarray(reject["scan_in_current_rear"])
    scan = reject["scan"]
    ranges = np.asarray(scan["ranges"], float)
    angles = scan["angle_min"]+np.arange(len(ranges))*scan["angle_increment"]+sensor[2]
    keep = np.isfinite(ranges) & (ranges <= 6.) & (ranges >= scan["range_min"])
    points = sensor[:2]+ranges[keep, None]*np.column_stack([np.cos(angles[keep]), np.sin(angles[keep])])
    world = to_world(points+np.array([REAR_OFFSET_M, 0.]), fault)
    distance, _ = cKDTree(np.concatenate(cloud)).query(world)
    return {"scope": "NEAREST_RETURNS_NO_FITTED_REGISTRATION_NOT_GROUND_TRUTH",
        "teacher_scan_row_ids": sorted(ids), "teacher_points": sum(map(len, cloud)),
        "runtime_return_count": int(keep.sum()), "runtime_range_limit_m": 6.,
        "nearest_return_distance_m": stats(distance.tolist()),
        "fraction_within_0p1m": float(np.mean(distance <= .1))}


def following_error(trial: Path, tracking: list[dict[str, Any]], reject_ns: int) -> dict[str, Any]:
    poses = []
    for row in records(trial/"vehicle_observations.jsonl"):
        if row.get("role") != "pose":
            continue
        x, y, z, w = row["quaternion_xyzw"]
        poses.append(TimedBodyPose(row["stamp_ns"], "sim", row["epoch"], row["frame"], row["child_frame"],
            *row["position_xyz_m"][:2], math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))))
    index = PoseIndex(poses)
    errors, reasons = [], Counter()
    for row in tracking:
        if row["sim_ns"] < reject_ns-12_000_000_000:
            continue
        current = TimedBodyPose(**row["details"]["current_pose"])
        target_ns = current.stamp_ns+1_000_000_000
        if target_ns > reject_ns:
            reasons["FUTURE_AFTER_REJECTION"] += 1
            continue
        try:
            future = index.at(target_ns)
            path = to_world(np.asarray(row["details"]["reference_xy_rear_m"])+[REAR_OFFSET_M, 0.], current)
            errors.append(project_to_polyline(np.array([future.x_m, future.y_m]), path)["distance_m"])
        except ValueError as exc:
            reasons[str(exc)] += 1
    return {"scope": "ONE_SECOND_FUTURE_ESTIMATED_POSE_TO_RECORDED_RAW_POLYLINE_NOT_TEACHER_ERROR",
        "distance_m": stats(errors), "excluded": dict(reasons), "ambiguous_pose_stamps": len(index.ambiguous),
        "vehicle_observations_sha256": sha(trial/"vehicle_observations.jsonl")}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--root", type=Path, required=True, help="Native e2e_autonomous root")
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args()
    args.output.mkdir(exist_ok=False)
    root, out = args.root, args.output
    cache = root/"datasets/cache/time_training_20260913_v2"
    corpus = root/"datasets/processed/time_teacher_20laps_20260913"
    inference = root/"runs/time_p1_training_evidence_20260913/float32"
    trial = root/"runs/time_turning_20260913/codex-time-turn-16"
    identity = read(cache/"identity.json")
    assert identity["manifest_sha256"] == content_sha256({k: v for k, v in identity.items() if k != "manifest_sha256"})
    validate_time_split(identity["split_manifest"], require_verified=True)
    precision = read(inference/"summary.json")
    assert precision["cache_manifest_sha256"] == identity["manifest_sha256"]
    checkpoint = Path(precision["arms"]["command_off"]["source_best_checkpoint"])
    assert sha(checkpoint) == precision["arms"]["command_off"]["source_best_sha256"]
    predictions_file = inference/"command_off_predictions.npy"
    predictions = np.load(predictions_file, mmap_mode="r", allow_pickle=False)
    offsets, offset = {}, 0
    for run in identity["runs"]:
        if run["split"] == "validation":
            offsets[run["run_id"]] = offset
            offset += run["anchors"]
    assert predictions.shape == (offset, 30, 2)
    expected_files = {f["path"]: f["sha256"] for f in identity["cache_files"]}
    split_runs = {r["run_id"]: r for r in identity["split_manifest"]["runs"]}
    controls = records(trial/"control.jsonl")
    reject = next(r for r in controls if r["event"] == "SCAN_GUARD_REJECTED")
    fault_cmd = next(r for r in controls if r["event"] == "COMMAND_SENT" and r["sim_ns"] == reject["sim_ns"]
                     and r.get("details", {}).get("current_pose"))
    fault = TimedBodyPose(**fault_cmd["details"]["current_pose"])
    fault_xy = np.array([fault.x_m, fault.y_m])
    common_speed = reject["speed_mps"]
    tracking = [r for r in controls if r["event"] == "COMMAND_SENT" and r.get("reason") == "TIME_PATH_TRACKING"]
    runtime_xy = np.array([[r["details"]["current_pose"]["x_m"], r["details"]["current_pose"]["y_m"]] for r in tracking])
    runtime_time = np.array([r["sim_ns"] for r in tracking])
    summary: dict[str, Any] = {"scope": "OFFLINE_MATCHED_SECTION_NOT_CLOSED_LOOP_OR_COLLISION_PROOF",
        "test_evaluated": False, "retraining": False, "runtime_modified": False,
        "section_half_arc_m": 12., "common_speed_mps": common_speed,
        "cache_manifest_sha256": identity["manifest_sha256"], "checkpoint_sha256": sha(checkpoint),
        "predictions_sha256": sha(predictions_file), "control_sha256": sha(trial/"control.jsonl"),
        "fault_pose": asdict(fault), "runs": {}}
    plot_data, full_teacher_xy = {}, {}
    for run_id in RUN_IDS:
        metadata = split_runs[run_id]
        split = metadata["split"]
        assert split in {"train", "validation"}
        cp = cache/split/run_id
        hashes = {}
        for name in ["anchors.jsonl", "labels.npz", "inputs.npz"]:
            actual = sha(cp/name)
            assert actual == expected_files[f"{split}/{run_id}/{name}"]
            hashes[name] = actual
        raw = corpus/split/run_id/"raw"
        hashes["bag_0.db3"] = sha(raw/"bag/bag_0.db3")
        assert hashes["bag_0.db3"] == next(s["sha256"] for s in metadata["sources"] if s["path"].endswith("bag_0.db3"))
        bag = Bag(raw)
        poses = bag.poses.rows
        xy = np.array([[p.x_m, p.y_m] for p in poses])
        times = np.array(bag.poses.times)
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
        yaw_diff = np.array([math.atan2(math.sin(p.yaw_rad-fault.yaw_rad), math.cos(p.yaw_rad-fault.yaw_rad)) for p in poses])
        distances = np.linalg.norm(xy-fault_xy, axis=1)
        center = int(np.argmin(np.where(abs(yaw_diff) <= math.pi/4, distances, np.inf)))
        if distances[center] > 3.:
            raise ValueError("MATCHED_SECTION_NOT_FOUND:"+run_id)
        section = abs(arc-arc[center]) <= 12.
        anchors = records(cp/"anchors.jsonl")
        with np.load(cp/"labels.npz") as z:
            labels, masks = z["xy_m"], z["xy_mask"]
        with np.load(cp/"inputs.npz") as z:
            valid_inputs = z["input_valid"]
        rows = []
        for index, a in enumerate(anchors):
            progress = float(np.interp(a["observation_ns"], times, arc)-arc[center])
            if abs(progress) > 12.:
                continue
            record: dict[str, Any] = {"anchor_id": a["anchor_id"], "observation_ns": a["observation_ns"],
                "progress_m": progress, "label_index": index}
            rows.append(record)
            try:
                if not valid_inputs[index] or not masks[index].all():
                    raise ValueError("INPUT_OR_FULL_TEACHER_INVALID")
                current = bag.poses.at(a["observation_ns"])
                velocity, steering = bag.state(a, "velocity"), bag.state(a, "actual_steering")
                ids = a["history_row_ids"]["lidar"][-1]
                if len(ids) != 1:
                    raise ValueError("CURRENT_SCAN_COUNT")
                scan_msg, receipt = bag.message(ids[0])
                if receipt > a["freeze_ns"] or scan_msg.header.frame_id != "lidar":
                    raise ValueError("SCAN_RECEIPT_OR_FRAME")
                captured = bag.poses.at(stamp(scan_msg))
                sensor = scan_pose_in_rear(captured, current, REAR_OFFSET_M)
                scan = {k: float(getattr(scan_msg, k)) for k in ["angle_min", "angle_increment", "range_min", "range_max"]}
                scan["ranges"] = scan_msg.ranges
                v, yaw, lateral = float(velocity.longitudinal_velocity), float(velocity.heading_rate), float(velocity.lateral_velocity)
                if v < .2:
                    raise ValueError("NORMALIZATION_SPEED_BELOW_0P2")
                measured = float(steering.steering_tire_angle)
                rear_lateral = lateral-COM_FORWARD_OF_REAR_M*yaw
                record.update({"pose": asdict(current), "speed_mps": v, "rear_lateral_mps": rear_lateral,
                    "scan_capture_ns": stamp(scan_msg), "scan_row_id": ids[0], "results": {}})
                for condition, speed in [("actual_speed", v), ("common_speed", common_speed)]:
                    rate = yaw*speed/v
                    state = dict(speed_mps=speed, measured_rad=measured, previous_rad=measured,
                        yaw_rate_radps=rate, lateral_mps=rear_lateral+COM_FORWARD_OF_REAR_M*rate)
                    results = {"observed_steer": scan_margin(scan, sensor, issued_rad=measured, **state),
                        "teacher_pp": compare_path(labels[index], current, scan, sensor, state, a["anchor_id"])}
                    if split == "validation":
                        results["model_pp"] = compare_path(predictions[offsets[run_id]+index], current, scan, sensor, state, a["anchor_id"])
                    record["results"][condition] = results
                if split == "validation":
                    error = np.linalg.norm(predictions[offsets[run_id]+index]-labels[index], axis=1)
                    record["ade_m"], record["fde3s_m"] = float(error.mean()), float(error[-1])
                record["reason"] = "EVALUATED"
            except ValueError as exc:
                record["reason"] = str(exc)
        valid = [r for r in rows if r["reason"] == "EVALUATED"]
        run_summary = {"split": split, "input_hashes": hashes, "tf_static": bag.tf,
            "ambiguous_pose_stamps": len(bag.poses.ambiguous), "selection_counts": dict(Counter(r["reason"] for r in rows)),
            "fault_to_teacher_projection": project_to_polyline(fault_xy, xy[section]),
            "nearest_teacher_pose": asdict(poses[center]), "speed_mps": stats([r["speed_mps"] for r in valid]),
            "ade_m": stats([r["ade_m"] for r in valid if "ade_m" in r]),
            "fde3s_m": stats([r["fde3s_m"] for r in valid if "fde3s_m" in r]), "comparisons": {}}
        if valid:
            nearest = min(valid, key=lambda r: abs(r["progress_m"]))
            run_summary["nearest_evaluated_anchor"] = nearest
        if split == "validation":
            run_summary["scan_alignment_to_runtime"] = scan_alignment(bag, rows, poses[center].stamp_ns, reject, fault)
        bag.con.close()
        for condition in ["actual_speed", "common_speed"]:
            for kind in ["observed_steer", "teacher_pp"]+(["model_pp"] if split == "validation" else []):
                results = [r["results"][condition][kind] for r in valid]
                run_summary["comparisons"][condition+"/"+kind] = {
                    "counts": dict(Counter(r["reason"] for r in results)),
                    "ray_margin_m": stats([r["minimum_ray_margin_m"] for r in results if r["minimum_ray_margin_m"] is not None])}
        write(out/(run_id+".json"), rows)
        summary["runs"][run_id] = run_summary
        plot_data[run_id] = (xy[section], valid)
        full_teacher_xy[run_id] = xy
        print(json.dumps({"completed": run_id, "counts": run_summary["selection_counts"],
            "offset": run_summary["fault_to_teacher_projection"], "comparisons": run_summary["comparisons"]}), flush=True)
    f = reject
    summary["runtime_rejection"] = scan_margin(f["scan"], f["scan_in_current_rear"], speed_mps=f["speed_mps"],
        measured_rad=f["measured_steer_rad"], issued_rad=f["issued_steer_rad"], previous_rad=f["previous_steer_rad"],
        yaw_rate_radps=f["motion_observation"]["heading_rate_radps"], lateral_mps=f["motion_observation"]["reported_lateral_mps"])
    late = runtime_time >= reject["sim_ns"]-12_000_000_000
    summary["runtime_last12s_ray_margin_m"] = stats([r["details"]["obstacle_guard"]["minimum_ray_margin_m"]
        for r in tracking if r["sim_ns"] >= reject["sim_ns"]-12_000_000_000])
    summary["runtime_last12s_teacher_distance_m"] = {run_id: stats([
        project_to_polyline(p, xy)["distance_m"] for p in runtime_xy[late]]) for run_id, xy in full_teacher_xy.items()}
    summary["runtime_last12s_following"] = following_error(trial, tracking, reject["sim_ns"])
    write(out/"summary.json", summary)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    ax = axes[0]
    for run_id, (xy, _) in plot_data.items():
        ax.plot(*(xy-fault_xy).T, alpha=.7, label=run_id, lw=1)
    ax.plot(*(runtime_xy[late]-fault_xy).T, color="#d02caa", lw=2, label="E2E last 12 s")
    ax.plot(0, 0, "rx", ms=9, label="Guard rejection")
    ax.set_aspect("equal"); ax.set_xlabel("Map east relative to rejection [m]"); ax.set_ylabel("Map north [m]")
    ax.set_title("Recorded localization trajectories"); ax.legend(fontsize=6)
    for ax, run_id in zip(axes[1:], ["5kmh_run03", "5kmh_run06"]):
        _, valid = plot_data[run_id]
        for kind, color in [("teacher_pp", "#087e53"), ("model_pp", "#ac2692")]:
            rows = [r for r in valid if r["results"]["common_speed"][kind]["minimum_ray_margin_m"] is not None]
            ax.plot([r["progress_m"] for r in rows], [r["results"]["common_speed"][kind]["minimum_ray_margin_m"] for r in rows],
                    color=color, label=kind, lw=1)
        ax.axhline(0., color="black", lw=.7); ax.axvline(0., color="gray", lw=.7)
        ax.set_title(run_id+" / common speed %.3f m/s" % common_speed)
        ax.set_xlabel("Teacher arc relative to matched location [m]"); ax.set_ylabel("Stopping ray margin [m]"); ax.legend()
    for ax in axes:
        ax.grid(alpha=.25)
    fig.suptitle("Offline matched section: same scan, same footprint, nominal PP demand (no actuation replay)")
    fig.tight_layout(); fig.savefig(out/"comparison.png", dpi=150)
    print(json.dumps({"status": "COMPLETE", "output": str(out), "test_evaluated": False}), flush=True)


if __name__ == "__main__":
    main()
