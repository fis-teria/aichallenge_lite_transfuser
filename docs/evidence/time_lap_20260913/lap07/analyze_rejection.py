"""Saved lap07 diagnostic, run in native WSL; never authorizes vehicle motion.

XY is metres; yaw is radians. Static scene transforms plus scan-header pose are
used for the plot. A point-to-polyline distance is NOT swept-body clearance.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from aic_transfuser_lite.control.long_sim_tracking_v4 import check_scan
from aic_transfuser_lite.control.time_reference_v1 import TimePlan, TimedBodyPose
from aic_transfuser_lite.control.time_trial_v1 import interpolate_body_pose, time_trial_control


def nearest(point: np.ndarray, path: np.ndarray) -> tuple[int, float, np.ndarray, float]:
    """Project a finite XY[2] onto finite nondegenerate segments of XY[N,2]."""
    point, path = np.asarray(point, float), np.asarray(path, float)
    if point.shape != (2,) or path.ndim != 2 or path.shape[1] != 2 or len(path) < 2:
        raise ValueError("XY_SHAPE")
    if not np.isfinite(point).all() or not np.isfinite(path).all():
        raise ValueError("XY_FINITE")
    delta = np.diff(path, axis=0)
    squared = np.sum(delta * delta, axis=1)
    indices = np.flatnonzero(squared > 1e-12)
    if not len(indices):
        raise ValueError("NO_NONDEGENERATE_SEGMENT")
    fraction = np.clip(np.sum((point-path[:-1][indices])*delta[indices], axis=1)/squared[indices], 0., 1.)
    projections = path[:-1][indices] + fraction[:, None]*delta[indices]
    distances = np.linalg.norm(point-projections, axis=1)
    i = int(np.argmin(distances))
    return int(indices[i]), float(fraction[i]), projections[i], float(distances[i])


def rotation(yaw_rad: float) -> np.ndarray:
    """Row-vector local-to-world rotation, shape [2,2]."""
    c, s = np.cos(yaw_rad), np.sin(yaw_rad)
    return np.array([[c, s], [-s, c]])


def smoke() -> None:
    i, fraction, point, distance = nearest(np.array([1., 2.]), np.array([[0., 0.], [2., 0.]]))
    assert i == 0 and fraction == .5 and distance == 2.
    np.testing.assert_allclose(point, [1., 0.])
    assert nearest(np.array([3., 0.]), np.array([[0., 0.], [2., 0.]]))[3] == 1.
    assert nearest(np.array([1., 0.]), np.array([[0., 0.], [0., 0.], [2., 0.]]))[0] == 1
    try:
        nearest(np.zeros(2), np.zeros((2, 2)))
    except ValueError as exc:
        assert str(exc) == "NO_NONDEGENERATE_SEGMENT"
    else:
        raise AssertionError("degenerate path accepted")
    np.testing.assert_allclose(np.array([1., 0.])@rotation(np.pi/2), [0., 1.], atol=1e-12)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args()
    smoke()
    run = args.root / "codex-time-lap-07"
    rows = [json.loads(line) for line in (run/"control.jsonl").read_text().splitlines()]
    snapshots = [row for row in rows if row.get("event") == "SCAN_GUARD_REJECTED"]
    assert len(snapshots) == 1
    event = snapshots[0]
    assert event["scan_frame"] == "lidar" and event["reason"] == "STOPPING_CORRIDOR_OCCUPIED"
    scan, speed = event["scan"], event["speed_mps"]
    ranges = np.asarray(scan["ranges"], float)
    try:
        check_scan(ranges, scan["angle_min"], scan["angle_increment"], scan["range_min"], scan["range_max"], speed)
    except ValueError as exc:
        assert str(exc) == event["reason"]
    else:
        raise AssertionError("saved guard rejection was not reproduced")
    angles = scan["angle_min"] + np.arange(len(ranges))*scan["angle_increment"]
    clipped = np.minimum(ranges, scan["range_max"])
    # Keep the ORIGINAL root-frame expression for the exact guard replay.
    root_xy = np.column_stack([1.165+clipped*np.cos(angles), clipped*np.sin(angles)])
    mask = (abs(angles) <= 1.3) & (abs(root_xy[:, 1]) <= .85) & (root_xy[:, 0] >= 0.)
    candidates = np.flatnonzero(mask)
    critical = int(candidates[np.argmin(root_xy[mask, 0])])
    clearance = float(root_xy[critical, 0])-1.5
    required = .4+max(0., speed)*.5+speed**2/2.
    assert clearance < required

    config_bytes = (run/"trial_config.json").read_bytes()
    config = json.loads(config_bytes)
    host = json.loads((run/"host_result.json").read_text())
    assert hashlib.sha256(config_bytes).hexdigest() == host["trial_config_sha256"]
    value = json.loads(event["latest_plan_json"])
    expected = {"event": "PLAN", "run_id": run.name, "epoch": "0", "clock": "sim", "frame": "base_link",
                "checkpoint_sha256": config["checkpoint_sha256"], "dt_s": .1, "precision": "float32",
                "producer_kind": "LEARNED_TIME_MODEL"}
    assert all(value.get(k) == v for k, v in expected.items())
    current = TimedBodyPose(**event["current_pose"])
    poses = [TimedBodyPose(**p) for p in event["pose_history"]]
    observed = interpolate_body_pose(poses, value["observation_ns"])
    captured = interpolate_body_pose(poses, event["scan_stamp_ns"])
    offset = config["geometry"]["rear_axle_forward_in_base_link_m"]
    calculation = time_trial_control(TimePlan(value["plan_id"], observed, np.array(value["raw_xy_m"])), current,
        speed_mps=speed, rear_axle_offset_m=(offset, 0.), speed_policy=config["speed_policy"])
    reference = np.asarray(calculation["reference_xy_rear_m"])+[offset, 0.]

    geometry_path = Path(__file__).with_name("scene_transforms.json")
    geometry = json.loads(geometry_path.read_text())
    assert geometry["scene_sha256"] == config["geometry"]["scene_sha256"]
    transforms = {row["path_id"]: row["data"] for row in geometry["objects"]}
    base_in_root = transforms[796]["m_LocalPosition"]["z"]
    lidar_in_base = transforms[840]["m_LocalPosition"]["z"]
    assert abs(base_in_root-config["geometry"]["base_link_forward_in_root_m"]) < 1e-12
    assert abs(base_in_root+lidar_in_base-1.165) < 1e-6

    def at_current(points: np.ndarray) -> np.ndarray:
        world = points@rotation(captured.yaw_rad)+[captured.x_m, captured.y_m]
        return (world-[current.x_m, current.y_m])@rotation(current.yaw_rad).T

    # A root point must move +0.485 m before comparing to a base_link path.
    cloud = at_current(root_xy-[base_in_root, 0.])
    point = cloud[critical]
    segment, fraction, projection, distance = nearest(point, reference)
    valid_hits = np.isfinite(ranges) & (ranges >= scan["range_min"]) & (ranges < scan["range_max"])

    # Reference centerline is a diagnostic illustration, never inference/PP input.
    center_path = args.root/"course_centerline.csv"
    center = np.genfromtxt(center_path, delimiter=",", names=True)
    center_xy = np.column_stack([center["x_m"], center["y_m"]])
    closed = np.vstack([center_xy, center_xy[0]])
    command_poses = {c["details"]["current_pose"]["stamp_ns"]: c["details"]["current_pose"] for c in rows
                     if c.get("event") == "COMMAND_SENT" and c.get("reason") == "TIME_PATH_TRACKING"}
    ordered = [p for _, p in sorted(command_poses.items())]
    track = np.array([[p["x_m"], p["y_m"]] for p in ordered])
    comparisons = []
    for label, p in (("first_tracking", track[0]), ("at_scan_rejection", np.array([current.x_m, current.y_m]))):
        index, alpha, projected, d = nearest(p, closed)
        delta = closed[index+1]-closed[index]
        error = p-projected
        signed = float((delta[0]*error[1]-delta[1]*error[0])/np.linalg.norm(delta))
        comparisons.append({"at": label, "segment_index": index, "projection_fraction": alpha,
            "distance_m": d, "signed_left_m": signed, "s_m": float(center["s_m"][index]+alpha*np.linalg.norm(delta))})

    arm = next(r["sim_ns"] for r in rows if r.get("event") == "ARMED")
    result = {"status": "SAVED_REJECTION_REPRODUCED", "run_id": run.name, "self_checks": "PASS",
        "first_rejection_since_arm_s": (event["sim_ns"]-arm)/1e9, "speed_mps": speed,
        "scan_age_s": (current.stamp_ns-captured.stamp_ns)/1e9, "plan_age_s": calculation["plan_age_sec"],
        "exact_guard": {"reason": event["reason"], "clearance_m": clearance, "required_m": required,
            "deficit_m": required-clearance, "critical_ray_index": critical, "range_m": float(ranges[critical]),
            "angle_rad": float(angles[critical]), "point_vehicle_root_m": root_xy[critical].tolist(),
            "lidar_forward_in_root_m": 1.165, "front_in_root_m": 1.5, "half_width_m": .85},
        "diagnostic_alignment": {"base_link_forward_in_root_m": base_in_root,
            "lidar_forward_in_base_link_m": lidar_in_base, "critical_point_current_base_m": point.tolist(),
            "critical_distance_to_reference_polyline_m": distance, "nearest_reference_xy_m": projection.tolist(),
            "nearest_segment_index": segment, "nearest_segment_fraction": fraction,
            "nearest_is_final_endpoint": bool(segment == len(reference)-2 and fraction == 1.),
            "reference_endpoint_m": reference[-1].tolist(), "calculated_steer_rad": calculation["steer_rad"],
            "lookahead_rear_m": calculation["lookahead_rear_m"], "geometry": calculation["geometry"]},
        "plan_validation_scope": "Recorded identity, original capture pose and mathematical PP only; scan rejected first. Plan receive age and publisher graph are not in this snapshot.",
        "reference_centerline_comparison": comparisons,
        "boundary": "Single-scan static scene alignment, all rays at header pose; no per-ray deskew, dynamic TF or full swept footprint proof. Point/polyline gap, especially at its endpoint, does not establish a safe turn. Reference centerline is not a teacher or mandatory lane center.",
        "source_sha256": {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in
            (run/"control.jsonl", run/"trial_config.json", center_path, geometry_path, Path(__file__))}}
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/"scan_diagnosis.json").write_text(json.dumps(result, indent=2, allow_nan=False))

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    fig, axes = plt.subplots(1, 2, figsize=(12, 6))
    ax = axes[0]
    hits = cloud[valid_hits]
    ax.scatter(hits[:, 0], hits[:, 1], s=5, color="#727c86", label="Recorded LiDAR hits")
    corridor = np.array([[0., -.85], [1.5+required, -.85], [1.5+required, .85], [0., .85]])
    corridor = at_current(corridor-[base_in_root, 0.])
    ax.add_patch(Polygon(corridor, facecolor="#ebad31", edgecolor="#b67b09", alpha=.22, label="Existing straight guard"))
    ax.plot(reference[:, 0], reference[:, 1], "o-", color="#e123ae", markersize=3, label="E2E reference, age aligned")
    ax.scatter(*point, s=80, marker="x", color="#c82b2b", label="Guard-triggering ray")
    ax.plot([point[0], projection[0]], [point[1], projection[1]], "--", color="#c82b2b")
    ax.scatter(0., 0., s=40, color="#204b80", label="Current base_link")
    ax.set(xlim=(-.5, 6.), ylim=(-2., 3.), xlabel="Current base_link forward [m]", ylabel="Left [m]",
           title=f"First rejected scan at {result['first_rejection_since_arm_s']:.2f} s\nPolyline distance is not body clearance")
    ax.legend(loc="upper right", fontsize=8)
    origin = track[0]
    ax = axes[1]
    ax.plot(*(closed-origin).T, color="#4eaa65", label="Reference centerline (diagnostic only)")
    ax.plot(*(track-origin).T, color="#245c98", linewidth=2, label="Measured base_link track")
    pred_world = reference@rotation(current.yaw_rad)+[current.x_m, current.y_m]
    ax.plot(*(pred_world-origin).T, color="#e123ae", linewidth=2, label="Last captured E2E plan")
    ax.scatter(*(track[-1]-origin), marker="x", color="#c82b2b")
    local_track = track-origin
    ax.set(xlim=(local_track[:, 0].min()-6., local_track[:, 0].max()+6.),
           ylim=(local_track[:, 1].min()-6., local_track[:, 1].max()+6.),
           xlabel="Map X from first tracking pose [m]", ylabel="Map Y from first tracking pose [m]",
           title="35.7 m attempt; lap not completed")
    ax.legend(fontsize=8)
    for ax in axes:
        ax.set_aspect("equal", adjustable="box"); ax.grid(alpha=.2)
    fig.suptitle("TimePath B0 + Pure Pursuit, target 5 km/h: saved scan diagnosis")
    fig.tight_layout(); fig.savefig(args.output/"scan_diagnosis.png", dpi=150); plt.close(fig)
    print(json.dumps(result, allow_nan=False))


if __name__ == "__main__":
    main()
