"""Read only this saved Tiny GUI trial; no ROS, model, dataset or new inference."""
from __future__ import annotations
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import struct


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    args.output.mkdir(parents=True)
    files = ["tiny_supervisor.jsonl", "tiny_worker.jsonl", "tiny_supervisor_summary.json",
             "tiny_worker_summary.json", "host_summary.json", "finalization_freeze.json", "budget_after.json"]
    contents = {name: (args.input/name).read_bytes() for name in files}
    records = [json.loads(line) for line in contents[files[0]].splitlines()]
    outputs = [json.loads(line) for line in contents[files[1]].splitlines()]
    summary, worker, host, freeze, budget = [json.loads(contents[name]) for name in files[2:]]
    sent = [r for r in records if r["event"] == "COMMAND_SENT_NOT_APPLIED_ACK"]
    tiny = [r for r in sent if r["source"] == "TINY_STEERING_WITH_SPEED_LIMITER"]
    brakes = [r for r in sent if r["source"] == "INDEPENDENT_SUPERVISOR_BRAKE"]
    requests = {r["operation_id"]: r for r in records if r["event"] == "COMMAND_REQUEST"}
    indexed = {(r["input_id"], r["tiny_forward_index"]): r for r in outputs}
    velocities = [r for r in records if r["event"] == "VELOCITY_RECEIVED"]
    velocity_by_stamp = {r["source_ns"]: r for r in velocities}
    bad_join, bad_ready, bad_request = [], [], []
    moving = []
    for command in sent:
        request = requests.get(command["operation_id"], {})
        if any(request.get(k) != command.get(k) for k in ("source", "input_id", "tiny_forward_index", "acceleration_mps2", "steering_rad")):
            bad_request.append(command["operation_id"])
    for command in tiny:
        output = indexed.get((command["input_id"], command["tiny_forward_index"]), {})
        if (output.get("steering_rad") != command["steering_rad"] or
                output.get("source_ns") != command["output_source_ns"] or
                output.get("received_ns") != command["output_received_ns"] or
                output.get("scan_sequence") != command["output_scan_sequence"]):
            bad_join.append(command["operation_id"])
        if not (command["output_received_ns"] > command["ready_received_ns"] and
                command["output_source_ns"] > command["ready_sim_ns"] and
                command["output_scan_sequence"] > command["ready_scan_sequence"]):
            bad_ready.append(command["operation_id"])
        velocity = velocity_by_stamp.get(command["velocity_source_ns"])
        if velocity and abs(velocity["speed_mps"]) >= .1:
            moving.append(command)
    stopping = [r for r in velocities if r["stopping"]]
    streak = []
    for velocity in stopping:
        if abs(velocity["speed_mps"]) <= .03:
            streak.append(velocity)
        else:
            streak = []
    assert len(streak) >= 5 and streak[-1]["source_ns"]-streak[0]["source_ns"] >= 500_000_000
    assert all(a["source_ns"] < b["source_ns"] for a, b in zip(streak, streak[1:]))
    assert len(outputs) == worker["tiny_forward_calls"] == summary["counters"]["tiny_outputs"]
    assert len(tiny) == summary["counters"]["tiny_steering_sent"]
    assert len({r["input_id"] for r in tiny}) == summary["distinct_tiny_inputs_sent"]
    assert not bad_join and not bad_ready and not bad_request
    stop = next(r for r in records if r["event"] == "STOP_BEGIN")
    first_positive = next(r for r in sent if r["first_positive_request"])
    positive = [r for r in tiny if r["acceleration_mps2"] > 0]
    assert streak[-1]["received_ns"] < freeze["monotonic_ns"]
    assert any(r["acceleration_mps2"] < 0 for r in brakes)
    assert summary["observed_motion"] and summary["observed_braking_stop"]
    assert max(abs(r["speed_mps"]) for r in velocities) == summary["max_abs_speed_mps"]
    result = dict(schema="TINY_GUI_RETRY2_SAVED_LOG_CHECK_V1", new_execution=False,
        execution_commit=summary["source_commit"], event_counts=dict(Counter(r["event"] for r in records)),
        tiny_forward_calls=len(outputs), tiny_commands=len(tiny), distinct_tiny_scans=len({r["input_id"] for r in tiny}),
        tiny_commands_with_moving_velocity=len(moving), distinct_tiny_scans_with_moving_velocity=len({r["input_id"] for r in moving}),
        request_sent_mismatches=bad_request, worker_sent_mismatches=bad_join, ready_fence_violations=bad_ready,
        brake_commands=len(brakes), negative_acceleration_brake_commands=sum(r["acceleration_mps2"] < 0 for r in brakes),
        stop_reason=stop["reason"], stop_was_initial_stationarity=stop["initially_stationary"],
        stop_streak_samples=len(streak), stop_streak_sim_s=(streak[-1]["source_ns"]-streak[0]["source_ns"])/1e9,
        stop_streak_last_speed_mps=streak[-1]["speed_mps"], stop_observation_precedes_host_freeze=True,
        first_positive_sim_ns=first_positive["sim_ns"], stop_begin_sim_ns=stop["sim_ns"],
        stop_begin_after_first_positive_sim_s=(stop["sim_ns"]-first_positive["sim_ns"])/1e9,
        last_positive_after_first_positive_sim_s=(positive[-1]["sim_ns"]-first_positive["sim_ns"])/1e9,
        powered_sim_seconds_including_braking=summary["powered_sim_seconds"], max_speed_mps=summary["max_abs_speed_mps"],
        host_error=host["error"], natural_braking_stop_observed=True, overall_clean_exit=False,
        lap_confirmed=host["judge_lap_confirmed"], collision_free="UNKNOWN", departure_free="UNKNOWN",
        final_used=budget["used"], current_attempt=budget["attempts"][-1],
        remaining={k: budget["tiny_authorized_limits"][k]-budget["used"][k] for k in budget["tiny_authorized_limits"]},
        input_sha256={name: hashlib.sha256(data).hexdigest() for name, data in contents.items()})
    result["remaining"]["forward"] -= budget["used"]["tiny_forward"]
    # Format conversion of an already captured screenshot; no new snapshot.
    xwd = (args.input/"gui_awsim.xwd").read_bytes()
    h = struct.unpack(">25I", xwd[:100])
    assert h[1:4] == (7, 2, 24) and h[6:8] == (0, 0) and h[11] == 32
    assert h[14:17] == (0xFF0000, 0xFF00, 0xFF)
    pixels = xwd[h[0]+h[19]*12:]
    assert len(pixels) == h[12]*h[5]
    from PIL import Image
    Image.frombytes("RGB", (h[4], h[5]), pixels, "raw", "BGRX", h[12], 1).save(args.output/"awsim_saved_frame.png")
    result["screenshot"] = dict(source_sha256=hashlib.sha256(xwd).hexdigest(), width=h[4], height=h[5],
                                resized=False, generated=False, capture_phase="HOST_FREEZE_AFTER_STOP")
    result["rviz_xwd_bytes"] = (args.input/"gui_rviz.xwd").stat().st_size
    (args.output/"saved_log_check.json").write_text(json.dumps(result, indent=2, ensure_ascii=False)+"\n", encoding="utf-8")
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
