"""Saved lap attempt evidence only; no runtime, model load or inference."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import struct


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    names = ["tiny_supervisor.jsonl", "tiny_worker.jsonl", "tiny_supervisor_summary.json",
             "tiny_worker_summary.json", "host_summary.json", "finalization_freeze.json", "budget_after.json"]
    raw = {name: (args.input/name).read_bytes() for name in names}
    records = [json.loads(line) for line in raw[names[0]].splitlines()]
    outputs = [json.loads(line) for line in raw[names[1]].splitlines()]
    sup, worker, host, freeze, budget = [json.loads(raw[name]) for name in names[2:]]
    sent = [r for r in records if r["event"] == "COMMAND_SENT_NOT_APPLIED_ACK"]
    tiny = [r for r in sent if r["source"] == "TINY_STEERING_WITH_SPEED_LIMITER"]
    brakes = [r for r in sent if r["source"] == "INDEPENDENT_SUPERVISOR_BRAKE"]
    requests = {r["operation_id"]:r for r in records if r["event"] == "COMMAND_REQUEST"}
    indexed = {(r["input_id"],r["tiny_forward_index"]):r for r in outputs}
    request_bad = [r["operation_id"] for r in sent if any(requests.get(r["operation_id"],{}).get(k) != r[k]
        for k in ("source","input_id","tiny_forward_index","acceleration_mps2","steering_rad"))]
    output_bad = [r["operation_id"] for r in tiny if any(indexed.get((r["input_id"],r["tiny_forward_index"]),{}).get(a) != r[b]
        for a,b in (("steering_rad","steering_rad"),("source_ns","output_source_ns"),
                    ("received_ns","output_received_ns"),("scan_sequence","output_scan_sequence")))]
    ready_bad = [r["operation_id"] for r in tiny if not (r["output_source_ns"] > r["ready_sim_ns"] and
        r["output_received_ns"] > r["ready_received_ns"] and r["output_scan_sequence"] > r["ready_scan_sequence"])]
    velocities = [r for r in records if r["event"] == "VELOCITY_RECEIVED"]
    velocity_by_stamp = {r["source_ns"]:r for r in velocities}
    moving = [r for r in tiny if abs(velocity_by_stamp.get(r["velocity_source_ns"],{}).get("speed_mps",0)) >= .1]
    streak = []
    for r in velocities:
        if not r["stopping"]: continue
        if abs(r["speed_mps"]) <= .03: streak.append(r)
        else: streak = []
    stopped = bool(len(streak) >= 5 and streak[-1]["source_ns"]-streak[0]["source_ns"] >= 500_000_000
                   and all(a["source_ns"] < b["source_ns"] for a,b in zip(streak,streak[1:]))
                   and streak[-1]["received_ns"] < freeze["monotonic_ns"])
    assert len(outputs) == worker["tiny_forward_calls"] == sup["counters"]["tiny_outputs"]
    assert len(tiny) == sup["counters"]["tiny_steering_sent"]
    remaining = {k:limit-budget["used"][k] for k,limit in budget["tiny_authorized_limits"].items()}
    remaining["forward"] -= budget["used"]["tiny_forward"]
    stop_begin = next(r for r in records if r["event"] == "STOP_BEGIN")
    before_stop = [r for r in velocities if r["monotonic_ns"] < stop_begin["monotonic_ns"]]
    last_motion = next((r for r in reversed(before_stop) if abs(r["speed_mps"]) >= .1), None)
    result = dict(schema="TINY_GUI_LAP_SAVED_EVIDENCE_V1", new_execution=False,
        execution_commit=sup["source_commit"], tiny_forward_calls=len(outputs), tiny_commands=len(tiny),
        distinct_tiny_inputs=len({r["input_id"] for r in tiny}), moving_tiny_commands=len(moving),
        moving_distinct_tiny_inputs=len({r["input_id"] for r in moving}),
        request_sent_mismatches=request_bad, worker_sent_mismatches=output_bad, ready_fence_violations=ready_bad,
        brake_commands=len(brakes), negative_brake_commands=sum(r["acceleration_mps2"] < 0 for r in brakes),
        max_speed_mps=sup["max_abs_speed_mps"], observed_motion=sup["observed_motion"],
        observed_braking_stop=sup["observed_braking_stop"], fresh_stop_before_freeze=stopped,
        runtime_braking_label_is_not_causal_proof=True,
        negative_braking_demonstrated=any(r["acceleration_mps2"] < 0 for r in brakes),
        speed_before_stop_request_mps=before_stop[-1]["speed_mps"] if before_stop else None,
        sim_s_since_last_speed_ge_0p1_at_stop=(stop_begin["sim_ns"]-last_motion["source_ns"])/1e9 if last_motion else None,
        stop_streak_samples=len(streak), final_speed_mps=velocities[-1]["speed_mps"] if velocities else None,
        stop_reason=sup["stop_reason"], supervisor_exception=sup["exception"], host_error=host["error"],
        runtime_exitcode=host["runtime_exitcode"], cleanup=host["cleanup"],
        judge_lap_confirmed=host["judge_lap_confirmed"], judge_laps=host["judge_laps"],
        judge_section_events=host["judge_section_events"], collision_free="UNKNOWN", departure_free="UNKNOWN",
        simulator_assets_unchanged=host["simulator_assets_unchanged"],
        wall_s=host["wall_s"], powered_sim_seconds=sup["powered_sim_seconds"],
        used=budget["used"], remaining=remaining, attempt=budget["attempts"][-1],
        input_sha256={name:hashlib.sha256(data).hexdigest() for name,data in raw.items()}, captures={})
    for kind in ("awsim", "rviz"):
        path = args.input/f"gui_{kind}.xwd"
        if not path.exists() or path.stat().st_size < 100:
            result["captures"][kind] = "MISSING_OR_EMPTY"
            continue
        xwd = path.read_bytes(); h = struct.unpack(">25I",xwd[:100])
        if not (h[1:4] == (7,2,24) and h[6:8] == (0,0) and h[11] == 32):
            result["captures"][kind] = dict(status="SAVED_UNSUPPORTED_XWD_LAYOUT", header=list(h),
                                            sha256=hashlib.sha256(xwd).hexdigest())
            continue
        assert h[14:17] == (0xFF0000,0xFF00,0xFF)
        pixels = xwd[h[0]+h[19]*12:]
        assert len(pixels) == h[12]*h[5]
        from PIL import Image
        Image.frombytes("RGB",(h[4],h[5]),pixels,"raw","BGRX",h[12],1).save(args.output/f"{kind}_saved_frame.png")
        result["captures"][kind] = dict(sha256=hashlib.sha256(xwd).hexdigest(),width=h[4],height=h[5],
                                       capture_phase="HOST_FREEZE_AFTER_STOP",generated=False)
    (args.output/"saved_evidence.json").write_text(json.dumps(result,indent=2,ensure_ascii=False)+"\n",encoding="utf-8")
    print(json.dumps(result,indent=2,ensure_ascii=False))


if __name__ == "__main__":
    main()
