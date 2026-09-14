"""Audit the actual accepted anchor states against the failed AWSIM passage.

Run in native WSL under the worktree lock. Uses recorded pose source IDs and
receipt-eligible velocity endpoints. All offsets are against measured nominal
map/base_link poses; no synthetic teachers, training, or split assignment.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import subprocess
import sys
from typing import Any

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO/"tools"))
sys.path.insert(0, str(REPO/"docs/evidence/time_corner_recovery_20260914"))
from compare_local import nominal
from compare_time_corner_tracking import anchor_observation
from compare_time_teacher_clearance import Bag, read, records, sha, stamp, stats, write
from aic_transfuser_lite.control.time_reference_v1 import TimedBodyPose
from aic_transfuser_lite.data.time_recovery_collection_v1 import load_pose_course, project_course
from aic_transfuser_lite.evaluation.time_corner_comparison_v1 import angle_delta

PRIMARY = "codex-time-recovery-left020-r23"
SECONDARY = "codex-time-recovery-right040-r22"


def anchor_velocity(bag: Bag, anchor: dict[str, Any]) -> np.ndarray:
    """Interpolate current [longitudinal m/s, lateral m/s, yaw rad/s] [3]."""
    ids = anchor["history_row_ids"]["velocity"][-1]
    if len(ids) not in (1, 2) or len(set(ids)) != len(ids):
        raise ValueError("VELOCITY_SOURCE_IDS")
    messages = [bag.message(i) for i in ids]
    if any(receipt > anchor["freeze_ns"] for _, receipt in messages):
        raise ValueError("VELOCITY_NOT_AVAILABLE")
    pairs = sorted((stamp(m), m) for m, _ in messages)
    ts = [t for t, _ in pairs]
    observation = anchor["observation_ns"]
    if not ts[0] <= observation <= ts[-1] or any(abs(t-observation) > 50_000_000 for t in ts):
        raise ValueError("VELOCITY_CAPTURE_BRACKETS")
    if len(ts) == 2 and ts[0] == ts[1]:
        raise ValueError("VELOCITY_DUPLICATE_STAMP")
    vals = np.array([[m.longitudinal_velocity, m.lateral_velocity, m.heading_rate] for _, m in pairs])
    assert vals.shape == (len(ids), 3) and np.isfinite(vals).all()
    fraction = 0. if len(ids) == 1 else (observation-ts[0])/(ts[1]-ts[0])
    return vals[0]+fraction*(vals[-1]-vals[0])


def summary(rows: list[dict[str, Any]]) -> dict[str, Any]:
    """Descriptive bins, not independent sample counts or model success gates."""
    result: dict[str, Any] = {k: stats([r[k] for r in rows]) for k in (
        "left_m", "heading_error_rad", "speed_mps", "base_s_m")}
    result["heading_error_deg"] = stats([math.degrees(r["heading_error_rad"]) for r in rows])
    result["speed_kmh"] = stats([3.6*r["speed_mps"] for r in rows])
    result["count"] = len(rows)
    result["independent_run_ids"] = sorted({r["run_id"] for r in rows})
    result["absolute_lateral_bins_m"] = {
        f"[{a},{b})": sum(a <= abs(r["left_m"]) < b for r in rows)
        for a, b in ((0., .05), (.05, .1), (.1, .2), (.2, .4), (.4, .8), (.8, 2.))}
    result["absolute_heading_bins_deg"] = {
        f"[{a},{b})": sum(a <= abs(math.degrees(r["heading_error_rad"])) < b for r in rows)
        for a, b in ((0., 1.), (1., 3.), (3., 6.), (6., 15.))}
    significant = [r for r in rows if abs(r["left_m"]) >= .05
                   and abs(r["heading_error_rad"]) >= math.radians(1.)]
    result["heading_quadrants_at_abs_offset_ge_5cm_and_abs_heading_ge_1deg"] = dict(
        toward_nominal=sum(r["left_m"]*r["heading_error_rad"] < 0 for r in significant),
        away_from_nominal=sum(r["left_m"]*r["heading_error_rad"] > 0 for r in significant),
        excluded_small_offset_or_heading=len(rows)-len(significant))
    return result


def envelope(rows: list[dict[str, Any]], samples: list[dict[str, Any]]) -> dict[str, Any]:
    """Axis bounding box only: presence inside does not establish coverage."""
    keys = ("left_m", "heading_error_rad", "speed_mps")
    bounds = {k: (min(r[k] for r in samples), max(r[k] for r in samples)) for k in keys}
    return {"method": "axis_bounding_box_is_only_a_necessary_condition_not_joint_coverage",
            "bounds": bounds, "count": len(rows),
            "outside_counts": {k: sum(not bounds[k][0] <= r[k] <= bounds[k][1] for r in rows) for k in keys},
            "outside_any_count": sum(any(not bounds[k][0] <= r[k] <= bounds[k][1] for k in keys) for r in rows)}


def smoke() -> None:
    from types import SimpleNamespace
    class FakeBag:
        def message(self, i: int) -> tuple[Any, int]:
            return (SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=100_000_000+i*20_000_000),
                longitudinal_velocity=float(i), lateral_velocity=0., heading_rate=.1*i), 1000)
    anchor = dict(history_row_ids={"velocity": [[0, 1]]}, observation_ns=110_000_000, freeze_ns=1000)
    np.testing.assert_allclose(anchor_velocity(FakeBag(), anchor), [.5, 0., .05])
    for changes, expected in ((dict(freeze_ns=999), "VELOCITY_NOT_AVAILABLE"),
                              (dict(observation_ns=121_000_000), "VELOCITY_CAPTURE_BRACKETS")):
        try:
            anchor_velocity(FakeBag(), {**anchor, **changes})
        except ValueError as exc:
            assert str(exc) == expected
        else:
            raise AssertionError(expected)
    row = dict(run_id="test", left_m=.1, heading_error_rad=-math.radians(2.), speed_mps=1., base_s_m=90.)
    other = {**row, "left_m": -.2}
    s = summary([row, other])
    assert s["absolute_lateral_bins_m"]["[0.1,0.2)"] == 1
    assert s["absolute_lateral_bins_m"]["[0.2,0.4)"] == 1
    assert s["heading_quadrants_at_abs_offset_ge_5cm_and_abs_heading_ge_1deg"] == dict(
        toward_nominal=1, away_from_nominal=1, excluded_small_offset_or_heading=0)
    assert s["speed_kmh"]["min"] == 3.6
    assert envelope([row, {**row, "speed_mps": 2.}], [row])["outside_any_count"] == 1
    print("COVERAGE_STATS_SMOKE_PASS")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--smoke-only", action="store_true")
    args = parser.parse_args()
    smoke()
    if args.smoke_only:
        return
    if args.root is None or args.output is None:
        parser.error("--root and --output required")
    root, out = args.root.resolve(), args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    old = root/"runs/time_corner_recovery_20260914"
    evidence = REPO/"docs/evidence/time_corner_recovery_20260914"
    index = read(evidence/"collection_index.json")
    base_points = load_pose_course(root/"runs/time_recovery_collection_20260913/inputs/base.csv")
    base = np.array([[p.x_m, p.y_m] for p in base_points])
    input_hashes = {str(evidence/"collection_index.json"): sha(evidence/"collection_index.json")}
    sources = {}
    lines = {}
    for name in (PRIMARY, SECONDARY):
        run = root/"raw/time_recovery_batches_20260914"/name
        for relative in ("bag/bag_0.db3", "control.jsonl", "result.json"):
            expected = read(run/"transfer_manifest.json")[relative]
            actual = sha(run/relative)
            assert actual == expected["sha256"] and (run/relative).stat().st_size == expected["bytes"]
            input_hashes[str(run/relative)] = actual
        _, lines[name] = nominal(run)
        sources[name] = lines[name].audit
    anchor_rows = []
    for record in index["records"]:
        name = record["run_id"]
        if not record.get("accepted_anchor_ids"):
            continue
        run = root/"raw/time_corner_recovery_20260914"/name
        probe_path = evidence/(name+"_causal_probe.json")
        probe = read(probe_path)
        assert sha(run/"transfer_manifest.json") == probe["raw_manifest_sha256"]
        for relative in ("bag/bag_0.db3", "control.jsonl", "result.json", "reference.json"):
            expected = read(run/"transfer_manifest.json")[relative]
            actual = sha(run/relative)
            assert actual == expected["sha256"] and (run/relative).stat().st_size == expected["bytes"]
            input_hashes[str(run/relative)] = actual
        input_hashes[str(probe_path)] = sha(probe_path)
        view = old/(name+"_causal_view_recovery_freeze50000000")
        assert (view/"bag/bag_0.db3").samefile(run/"bag/bag_0.db3")
        bag = Bag(view)
        accepted = [a for a in probe["anchors"] if a["usable_full"] and a.get("phase_and_xy_full")]
        assert [a["anchor_id"] for a in accepted] == record["accepted_anchor_ids"]
        for anchor in accepted:
            pose = anchor_observation(bag, anchor)
            velocity = anchor_velocity(bag, anchor)
            cp = project_course(base, [pose.x_m, pose.y_m], pose.yaw_rad)
            row = dict(run_id=name, anchor_id=anchor["anchor_id"], observation_ns=anchor["observation_ns"],
                pose=asdict(pose), observation_pose_row_ids=anchor["observation_pose_row_ids"],
                current_velocity_row_ids=anchor["history_row_ids"]["velocity"][-1],
                base_s_m=cp["s_m"], speed_mps=float(velocity[0]), lateral_speed_mps=float(velocity[1]),
                yaw_rate_rps=float(velocity[2]), references={})
            for ref_name, line in lines.items():
                proj = line.project(np.array([pose.x_m, pose.y_m]), yaw_hint_rad=pose.yaw_rad)
                row["references"][ref_name] = dict(left_m=proj.left_m,
                    heading_error_rad=angle_delta(pose.yaw_rad, proj.body_yaw_rad))
            row.update(row["references"][PRIMARY])
            anchor_rows.append(row)
        bag.con.close()
    assert len(anchor_rows) == index["total_valid_recovery_anchors"] == 118
    trial = root/"runs/time_segment_awsim_evidence_20260914/codex-time-segment01"
    control_path = trial/"control.jsonl"
    prior = read(REPO/"docs/evidence/time_corner_teacher_comparison_20260914/summary.json")
    assert sha(control_path) == prior["input_hashes"][str(control_path.relative_to(root))]
    input_hashes[str(control_path)] = sha(control_path)
    controls = records(control_path)
    fault = next(r for r in controls if r["event"] == "COMMAND_SENT" and r["reason"] == "STOPPING_SWEEP_OCCUPIED")
    failure_rows = []
    for r in controls:
        if not (r["event"] == "COMMAND_SENT" and r.get("sim_ns") is not None
                and fault["sim_ns"]-10_000_000_000 <= r["sim_ns"] <= fault["sim_ns"]):
            continue
        pose = TimedBodyPose(**r["details"]["current_pose"])
        cp = project_course(base, [pose.x_m, pose.y_m], pose.yaw_rad)
        proj = lines[PRIMARY].project(np.array([pose.x_m, pose.y_m]), yaw_hint_rad=pose.yaw_rad)
        failure_rows.append(dict(run_id="codex-time-segment01", time_to_fault_s=(r["sim_ns"]-fault["sim_ns"])/1e9,
            pose=asdict(pose), base_s_m=cp["s_m"], left_m=proj.left_m,
            heading_error_rad=angle_delta(pose.yaw_rad, proj.body_yaw_rad), speed_mps=r["speed_mps"]))
    shared_s = (min(r["base_s_m"] for r in anchor_rows), max(r["base_s_m"] for r in anchor_rows))
    failure_groups = dict(last10s=failure_rows, last5s=[r for r in failure_rows if r["time_to_fault_s"] >= -5.],
        same_anchor_course_range=[r for r in failure_rows if shared_s[0] <= r["base_s_m"] <= shared_s[1]])
    report = dict(scope="ACCEPTED_ANCHOR_STATE_COVERAGE_NOT_MODEL_SUCCESS_OR_DATA_SUFFICIENCY",
        source_commit=subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
        primary_nominal=PRIMARY, nominal_source_audits=sources, input_hashes=input_hashes,
        independent_recovery_episodes=index["independent_recovery_episodes"],
        accepted=summary(anchor_rows), by_run={name:summary([r for r in anchor_rows if r["run_id"] == name])
                                            for name in sorted({r["run_id"] for r in anchor_rows})},
        secondary_nominal=summary([{**r, **r["references"][SECONDARY]} for r in anchor_rows]),
        failure={k:dict(summary=summary(v), relative_to_anchor_envelope=envelope(v, anchor_rows))
                 for k,v in failure_groups.items()},
        shared_course_range_m=shared_s, trained_with_new_anchors=False, split_assigned=False,
        heading_quadrant_note="Body yaw versus nominal body yaw is a state descriptor, not measured lateral-error derivative",
        sufficiency="Not established: overlapping frames, two episodes, no independent evaluation of these new anchors")
    write(out/"summary.json", report)
    write(out/"accepted_anchor_states.json", anchor_rows)
    write(out/"failure_states.json", failure_rows)
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.8))
    for name in report["by_run"]:
        rows = [r for r in anchor_rows if r["run_id"] == name]
        label = "20cm left recovery" if "left" in name else "20cm right recovery"
        axes[0].scatter([r["left_m"] for r in rows], [math.degrees(r["heading_error_rad"]) for r in rows], s=15,label=label)
        axes[1].scatter([r["base_s_m"] for r in rows], [3.6*r["speed_mps"] for r in rows],s=15,label=label)
    for ax in axes:
        ax.grid(alpha=.3)
    axes[0].plot([r["left_m"] for r in failure_rows], [math.degrees(r["heading_error_rad"]) for r in failure_rows],
        color="#a52a2a",label="Failed E2E: last 10s")
    axes[0].set(xlabel="Left offset from measured nominal [m]",ylabel="Body yaw difference [deg]")
    axes[0].axhline(0,color="gray",lw=.7);axes[0].axvline(0,color="gray",lw=.7)
    axes[0].legend(fontsize=8)
    axes[1].plot([r["base_s_m"] for r in failure_rows], [3.6*r["speed_mps"] for r in failure_rows],color="#a52a2a")
    axes[1].set(xlabel="Original course progress [m]",ylabel="Measured speed [km/h]")
    fig.suptitle("Accepted states: 118 overlapping anchors from 2 independent recovery runs")
    fig.tight_layout();fig.savefig(out/"coverage.png",dpi=150);plt.close(fig)
    print(json.dumps({k:v for k,v in report.items() if k not in ("input_hashes",)},indent=2,allow_nan=False))


if __name__ == "__main__":
    main()
