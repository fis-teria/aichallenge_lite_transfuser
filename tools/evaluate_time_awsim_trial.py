"""Native WSL evaluation of recorded time-model trial, no runtime mutations."""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
from typing import Any

import numpy as np

from aic_transfuser_lite.control.time_geometry_v2 import validate_time_geometry
from aic_transfuser_lite.control.time_reference_v1 import TimePlan, TimedBodyPose
from aic_transfuser_lite.control.time_trial_v1 import time_trial_control, validate_trial_config
from aic_transfuser_lite.control.awsim_steering import command_steering


def replay_recorded_control(commands: list[dict[str, Any]], plans: list[dict[str, Any]],
                            rear_axle_forward_m: float, *,
                            speed_policy: str = "source_capped_0p25", obstacle_policy: str = "straight_v1",
                            steering_policy: str = "identity_v1", lookahead_policy: str = "fixed_1m_v1") -> dict[str, Any]:
    """Reproduce decisions from recorded raw predictions, poses, and measured speed.

    This covers the calculation before the separate output steering-rate clamp.
    Older records without pose/plan identity are explicitly outside the replay.
    """
    by_id = {p["plan_id"]: p for p in plans}
    matched = skipped = 0
    maximum_error = 0.
    post_control_rejections: Counter[str] = Counter()
    actuator_matched = 0
    scan_reasons = {"OBSERVATION_POSE_MISSING", "FRESH_ALIGNED_SCAN_MISSING", "POSE_ENDPOINT_IDENTITY",
                    "SCAN_FRAME", "SCAN_POSE_IDENTITY_OR_AGE", "SCAN_POSE_ALIGNMENT", "SCAN_CONTRACT",
                    "SCAN_COVERAGE", "SCAN_UNKNOWN", "STOPPING_SWEEP_OCCUPIED", "SWEEP_VEHICLE_STATE"}
    for command in commands:
        details = command.get("details", {})
        if not command.get("plan_id") or not all(k in details for k in ("observation_pose", "current_pose")):
            skipped += 1
            continue
        plan = by_id[command["plan_id"]]
        observed = TimedBodyPose(**details["observation_pose"])
        current = TimedBodyPose(**details["current_pose"])
        try:
            calculated = time_trial_control(TimePlan(plan["plan_id"], observed, np.array(plan["raw_xy_m"])),
                current, speed_mps=command["speed_mps"], rear_axle_offset_m=(rear_axle_forward_m, 0.),
                speed_policy=speed_policy, lookahead_policy=lookahead_policy)
        except ValueError as exc:
            if command["reason"] != str(exc):
                raise ValueError("recorded rejection could not be reproduced") from exc
        else:
            if command["reason"] != "TIME_PATH_TRACKING":
                post_guard = obstacle_policy == "steering_sweep_v1" and command["reason"] in scan_reasons
                actuator_rejected = (steering_policy == "awsim_grip_0p6_v1"
                                     and command["reason"] == "STEERING_ACTUATOR_INFEASIBLE"
                                     and abs(calculated["steer_rad"]) > .3)
                if not (post_guard or actuator_rejected):
                    raise ValueError("recorded admission could not be reproduced")
                if not (command["target_speed_mps"] == 0 and command["acceleration_mps2"] < 0):
                    raise ValueError("post-control rejection did not brake")
                post_control_rejections[command["reason"]] += 1
            for key in ("steer_rad", "acceleration_mps2", "target_speed_mps", "reference_xy_rear_m"):
                error = float(np.max(np.abs(np.asarray(calculated[key]) - np.asarray(details[key]))))
                maximum_error = max(maximum_error, error)
                if not np.isfinite(error) or error > 1e-9:
                    raise ValueError(f"recorded control differs: {key} error={error}")
            if "steering_actuator" in details:
                stored = details["steering_actuator"]
                mapping = command_steering(calculated["steer_rad"], stored["previous_input_rad"],
                                           stored["dt_s"], policy=steering_policy)
                if stored["policy"] != steering_policy:
                    raise ValueError("recorded steering policy differs")
                for key, expected in mapping.items():
                    if key != "policy" and (not np.isfinite(stored[key]) or abs(stored[key]-expected) > 1e-9):
                        raise ValueError("recorded steering mapping differs: " + key)
                if (command["reason"] == "TIME_PATH_TRACKING"
                        and abs(command["steer_rad"]-mapping["issued_input_rad"]) > 1e-9):
                    raise ValueError("recorded issued steering differs")
                actuator_matched += 1
        matched += 1
    return {"status": "PASS" if matched else "NOT_RECORDED", "matched_commands": matched,
            "unavailable_commands": skipped, "maximum_absolute_error": maximum_error,
            "tolerance": 1e-9, "scope": "geometry_and_PP_plus_recorded_actuator_mapping_not_scan_admission",
            "post_control_rejections": dict(post_control_rejections), "actuator_mapping_matched": actuator_matched,
            "scan_guard_decisions_replayed": False}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    def rows(name: str) -> list[dict[str, Any]]:
        return [json.loads(line) for line in (args.run/name).read_text().splitlines()]
    host = json.loads((args.run/"host_result.json").read_text())
    if host["status"] not in ("COMPLETE_BOUNDED_TRIAL", "COMPLETE_LAP", "STOPPED_NO_LAP", "FAILED") or host["scope"] not in (
            "10_SIM_SECOND_LOW_SPEED_MODEL_TRIAL", "10_SIM_SECOND_MODEL_TRIAL", "ONE_LAP_MODEL_TRIAL"):
        raise ValueError("recognized recorded bounded trial required")
    control = rows("control.jsonl")
    inference = rows("inference.jsonl")
    plans = [p for p in inference if p.get("event") == "PLAN"]
    commands = [c for c in control if c.get("event") == "COMMAND_SENT"]
    armed = [r for r in control if r.get("event") == "ARMED"]
    if len(armed) != 1 or not plans:
        raise ValueError("one authorized trial and nonempty predictions required")
    start = armed[0]["sim_ns"]
    publishers = [r for r in control if r.get("event") == "PUBLISHER_CREATED"]
    if len(publishers) != 1:
        raise ValueError("one recorded controller publisher required")
    end = start + round(publishers[0].get("drive_limit_sim_s", 10.)*1e9)
    stopped = [r["sim_ns"] for r in control if r.get("event") == "STOP_REQUESTED"]
    if stopped:
        end = min(end, min(stopped))
    if commands:
        end = min(end, max(c["sim_ns"] for c in commands)+1)
    duration_s = (end-start)/1e9
    active = [c for c in commands if start <= c["sim_ns"] < end]
    active_plans = [p for p in plans if start <= p["observation_ns"] < end]
    if not active or not active_plans:
        raise ValueError("authorized interval must contain commands and observations")
    def quantiles(values: list[float] | np.ndarray) -> dict[str, float]:
        return dict(zip(("min", "median", "p95", "max"), np.quantile(values, [0, .5, .95, 1]).tolist()))
    xy = np.array([p["raw_xy_m"] for p in active_plans], dtype=float)
    if xy.shape[1:] != (30, 2) or not np.isfinite(xy).all():
        raise ValueError("invalid recorded time predictions")
    turns = []; first_fold_indices = []
    for path in xy:
        steps = np.diff(np.vstack((np.zeros((1, 2)), path)), axis=0)
        indices = np.flatnonzero(np.linalg.norm(steps, axis=1) > .01)
        moving = steps[indices]
        heading = np.arctan2(moving[:, 1], moving[:, 0])
        delta = np.arctan2(np.sin(np.diff(heading)), np.cos(np.diff(heading)))
        bad = np.flatnonzero(np.abs(delta) > 1.2)
        first_fold_indices.append(int(indices[bad[0]]) if len(bad) else None)
        turns.append(float(abs(delta[0])) if len(delta) else 0.)
    source = np.concatenate((np.zeros((len(xy), 1, 2)), xy[:, :3]), axis=1)
    source_speed = np.linalg.norm(np.diff(source, axis=1), axis=2).sum(axis=1) / .3
    positive = sum(c["acceleration_mps2"] > 0 for c in active)
    geometry_reasons: Counter[str] = Counter()
    for path in xy:
        try:
            geometry = validate_time_geometry(path)
            geometry_reasons["RESOLVED" if geometry["motion_resolved"] else "MOTION_UNRESOLVED"] += 1
        except ValueError as exc:
            geometry_reasons[str(exc)] += 1
    pose_by_stamp = {c["details"]["current_pose"]["stamp_ns"]: c["details"]["current_pose"] for c in active
                     if c.get("details", {}).get("current_pose") is not None}
    measured_xy = np.array([[p["x_m"], p["y_m"]] for _, p in sorted(pose_by_stamp.items())])
    speed_policy = publishers[0].get("speed_policy", "source_capped_0p25")
    if host["scope"] in ("10_SIM_SECOND_MODEL_TRIAL", "ONE_LAP_MODEL_TRIAL"):
        config_bytes = (args.run/"trial_config.json").read_bytes()
        config_sha = hashlib.sha256(config_bytes).hexdigest()
        if (config_sha != host["trial_config_sha256"] or config_sha != publishers[0]["trial_config_sha256"]
                or speed_policy != validate_trial_config(json.loads(config_bytes))
                or speed_policy != host["speed_policy"]):
            raise ValueError("recorded runtime/config speed policy mismatch")
        config = json.loads(config_bytes)
        for key, default in (("obstacle_policy", "straight_v1"), ("steering_policy", "identity_v1"),
                             ("lookahead_policy", "fixed_1m_v1")):
            if config.get(key, default) != publishers[0].get(key, default):
                raise ValueError("recorded runtime/config mismatch: " + key)
    replay = replay_recorded_control(active, plans, publishers[0]["rear_axle_forward_m"], speed_policy=speed_policy,
        obstacle_policy=publishers[0].get("obstacle_policy", "straight_v1"),
        steering_policy=publishers[0].get("steering_policy", "identity_v1"),
        lookahead_policy=publishers[0].get("lookahead_policy", "fixed_1m_v1"))
    late_speeds = [c["speed_mps"] for c in active if c["sim_ns"] >= end - 3_000_000_000 and c["speed_mps"] is not None]
    result = {"status": ("LAP_COMPLETED" if host.get("judge_lap_confirmed") and host["status"] == "COMPLETE_LAP" else "LAP_NOT_COMPLETED") if host["scope"] == "ONE_LAP_MODEL_TRIAL" else ("COMPLETED_NO_POSITIVE_DRIVE" if positive == 0 else "COMPLETED_POSITIVE_COMMANDS_OBSERVED"),
        "evaluator_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "scope": "BOUNDED_SAME_SCENE_TEST_NOT_LAP_OR_AVOIDANCE_ACCEPTANCE",
        "source_host_status": host["status"], "official_start_requested": host["official_start_requested"],
        "armed_sim_ns": start, "active_duration_sim_s": duration_s,
        "judge_lap_confirmed": host.get("judge_lap_confirmed", False), "judge_laps": host.get("judge_laps", []),
        "judge_sections": [s["next"] for s in host.get("judge_section_events", [])],
        "host_error": host.get("error"), "requested_stop_reason": host["last_control"].get("requested_stop_reason"),
        "active_commands": len(active), "positive_acceleration_commands": positive,
        "active_command_reasons": dict(Counter(c["reason"] for c in active)),
        "all_predictions": len(plans), "active_observation_predictions": len(active_plans),
        "foldback_statistics_policy": "legacy_step_heading_v1_not_current_admission",
        "recorded_geometry_policy_versions": sorted({c["details"]["geometry"]["version"] for c in active
                                                     if "geometry" in c.get("details", {})}),
        "active_new_geometry_reasons": dict(geometry_reasons),
        "control_replay": replay,
        "speed_policy": speed_policy,
        "active_target_speed_mps": quantiles([c["target_speed_mps"] for c in active]),
        "final_3s_measured_speed_kmh": quantiles(np.asarray(late_speeds)*3.6) if late_speeds else None,
        "active_samples_within_0p25_kmh_of_5": sum(c["speed_mps"] is not None and abs(c["speed_mps"]*3.6-5.) <= .25 for c in active),
        "recorded_pose_count": len(measured_xy),
        "recorded_pose_travel_m": float(np.linalg.norm(np.diff(measured_xy, axis=0), axis=1).sum()) if len(measured_xy) > 1 else None,
        "recorded_pose_net_displacement_m": float(np.linalg.norm(measured_xy[-1] - measured_xy[0])) if len(measured_xy) > 1 else None,
        "active_first_fold_source_segment_indices": dict(Counter(str(i) for i in first_fold_indices)),
        "active_first_turn_abs_rad": quantiles(turns), "foldback_threshold_rad": 1.2,
        "active_first_point_x_m": quantiles(xy[:,0,0]), "active_first_point_y_m": quantiles(xy[:,0,1]),
        "active_3s_endpoint_norm_m": quantiles(np.linalg.norm(xy[:,-1], axis=1)),
        "active_source_speed_0_to_0p3s_mps": quantiles(source_speed),
        "inference_duration_ms": quantiles([p["inference_ns"]/1e6 for p in plans]),
        "max_measured_speed_mps": max(abs(c["speed_mps"]) for c in commands if c["speed_mps"] is not None),
        "stop_confirmed_before_cleanup": host["last_control"]["stop_confirmed"],
        "runtime_inference_rejections": dict(Counter(r.get("reason") for r in inference if r.get("event") in ("INPUT_REJECTED", "ANCHOR_REJECTED"))),
        "cleanup_errors": host["cleanup_errors"], "wall_s": host["wall_s"],
        "checkpoint_sha256": sorted({p["checkpoint_sha256"] for p in plans}),
        "raw_predictions_modified": False,
        "boundary": "Recorded command and prediction analysis; no measured lane-tracking accuracy or model teacher error.",
        "source_sha256": {name: hashlib.sha256((args.run/name).read_bytes()).hexdigest()
                          for name in ("host_result.json", "control.jsonl", "inference.jsonl")}}
    (args.output/"summary.json").write_text(json.dumps(result, indent=2, allow_nan=False))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
    for path in xy[::max(1, len(xy)//20)]:
        axes[0].plot(path[:,0], path[:,1], color="#7fb3e0", alpha=.35)
    example = xy[len(xy)//2]
    axes[0].plot(example[:,0], example[:,1], "o-", color="#145894", markersize=3)
    near = np.vstack((np.zeros((1,2)), example[:5]))
    axes[1].plot(near[:,0], near[:,1], "o-", color="#145894")
    axes[1].plot(0, 0, "x", color="#cb3333", markersize=10, label="Known observation origin")
    axes[1].annotate("0.1 s", example[0], xytext=(12, 20), textcoords="offset points",
                     arrowprops={"arrowstyle": "->", "color": "#333333"})
    axes[0].set_title("Raw 3 s predictions while drive was authorized")
    axes[1].set_title("First points: origin-to-first-step heading change")
    for ax in axes:
        ax.set_xlabel("Observation base_link forward [m]"); ax.set_ylabel("Left [m]")
        ax.grid(alpha=.25); ax.set_aspect("equal", adjustable="datalim")
    axes[1].legend(loc="lower right", fontsize=8)
    fig.suptitle(f"Proposed time model: {duration_s:.1f} s AWSIM trial ({positive} positive acceleration commands)")
    fig.tight_layout(); fig.savefig(args.output/"raw_time_paths.png", dpi=150); plt.close(fig)
    fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    timeline = [c for c in commands if start <= c["sim_ns"] < end + 2_000_000_000]
    times = [(c["sim_ns"] - start) / 1e9 for c in timeline]
    axes[0].plot(times, [c["speed_mps"] if c["speed_mps"] is not None else np.nan for c in timeline], label="Measured speed")
    axes[0].plot(times, [c["target_speed_mps"] for c in timeline], label="Commanded target speed", alpha=.7)
    axes[0].set_ylabel("Speed [m/s]"); axes[0].legend()
    axes[1].plot(times, [c["acceleration_mps2"] for c in timeline]); axes[1].set_ylabel("Command accel [m/s²]")
    axes[2].plot(times, [c["steer_rad"] for c in timeline]); axes[2].set_ylabel("Command steer [rad]")
    axes[2].set_xlabel("Time since drive authorization [sim s]")
    for ax in axes:
        ax.grid(alpha=.25)
        ax.axvline(duration_s, linestyle="--", color="#666666", linewidth=1)
    fig.suptitle("Proposed time model: bounded AWSIM control and measured response")
    fig.tight_layout(); fig.savefig(args.output/"control_timeline.png", dpi=150); plt.close(fig)
    print(json.dumps(result, allow_nan=False))


if __name__ == "__main__":
    main()
