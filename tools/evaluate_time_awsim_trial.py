"""Native WSL evaluation of recorded time-model trial, no runtime mutations."""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
from typing import Any

import numpy as np


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    def rows(name: str) -> list[dict[str, Any]]:
        return [json.loads(line) for line in (args.run/name).read_text().splitlines()]
    host = json.loads((args.run/"host_result.json").read_text())
    if host["status"] != "COMPLETE_BOUNDED_TRIAL" or host["scope"] != "10_SIM_SECOND_LOW_SPEED_MODEL_TRIAL":
        raise ValueError("completed 10-second bounded trial required")
    control = rows("control.jsonl")
    inference = rows("inference.jsonl")
    plans = [p for p in inference if p.get("event") == "PLAN"]
    commands = [c for c in control if c.get("event") == "COMMAND_SENT"]
    armed = [r for r in control if r.get("event") == "ARMED"]
    if len(armed) != 1 or not plans:
        raise ValueError("one authorized trial and nonempty predictions required")
    start = armed[0]["sim_ns"]
    active = [c for c in commands if start <= c["sim_ns"] < start + 10_000_000_000]
    active_plans = [p for p in plans if start <= p["observation_ns"] < start + 10_000_000_000]
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
    result = {"status": "COMPLETED_NO_POSITIVE_DRIVE" if positive == 0 else "COMPLETED_POSITIVE_COMMANDS_OBSERVED",
        "evaluator_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "scope": "BOUNDED_SAME_SCENE_TEST_NOT_LAP_OR_AVOIDANCE_ACCEPTANCE",
        "source_host_status": host["status"], "official_start_requested": host["official_start_requested"],
        "armed_sim_ns": start, "active_commands": len(active), "positive_acceleration_commands": positive,
        "active_command_reasons": dict(Counter(c["reason"] for c in active)),
        "all_predictions": len(plans), "active_observation_predictions": len(active_plans),
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
        "raw_predictions_modified": False, "guard_relaxed": False,
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
    fig.suptitle(f"Proposed time model: 10 s AWSIM trial ({positive} positive acceleration commands)")
    fig.tight_layout(); fig.savefig(args.output/"raw_time_paths.png", dpi=150); plt.close(fig)
    print(json.dumps(result, allow_nan=False))


if __name__ == "__main__":
    main()
