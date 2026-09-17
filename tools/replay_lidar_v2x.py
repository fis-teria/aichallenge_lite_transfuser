#!/usr/bin/env python3
"""Causal, native-WSL LiDAR adapter replay. V2X is evaluation-only, never input.

Consumes recorded map->base_link TF and base_link->lidar static TF. Reports
association to native V2X centres; this is not annotated detection recall or a
closed-loop driving test. Raw bags and generated rows stay outside Git.
"""
from __future__ import annotations

import argparse
from collections import Counter, deque
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
from rosbags.highlevel import AnyReader

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/aic_lidar_v2x"))
from aic_lidar_v2x.core import Config, Detector, Pose2, Scan, Tracker, scan_points, v2x_payload
from aic_lidar_v2x.io import PoseHistory, load_map, load_reference, planar_pose


def seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def transform_pose(transform) -> Pose2:
    p, q = transform.translation, transform.rotation
    return planar_pose(p.x, p.y, (q.x, q.y, q.z, q.w))


def mount_pose(static: dict, child: str) -> Pose2:
    chain = []
    seen = set()
    while child != "base_link":
        if child in seen or child not in static:
            raise LookupError(f"Missing/invalid base_link -> {child} TF chain")
        seen.add(child)
        parent, pose = static[child]
        chain.append(pose)
        child = parent
    pose = Pose2(0, 0, 0)
    for part in reversed(chain):
        pose = pose.compose(part)
    return pose


def percentiles(values: list[float]) -> dict | None:
    return {key: float(np.percentile(values, q)) for key, q in [("p50", 50), ("p95", 95), ("max", 100)]} if values else None


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--map-yaml", type=Path, required=True)
    parser.add_argument("--reference-csv", type=Path)
    parser.add_argument("--object-model", choices=["surface", "known_vehicle"], default="surface")
    parser.add_argument("--motion-model", choices=["rolling", "snapshot"], default="rolling")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(f"Use a new output directory: {args.output}")
    cfg = Config(object_model=args.object_model, motion_model=args.motion_model)
    detector = Detector(cfg, load_map(args.map_yaml, cfg.wall_margin_m),
                        load_reference(args.reference_csv) if args.reference_csv else None)
    tracker, history = Tracker(cfg), PoseHistory()
    static, native = {}, {}
    pending = deque()
    counts, totals = Counter(), Counter()
    rows, processing_ms, matches, plots = [], [], [], []
    plot_targets = [30., 36., 46., 52.]
    args.output.mkdir(parents=True)

    def process_pending(receipt_s: float, reader) -> None:
        while pending:
            msg, arrived = pending[0]
            stamp = seconds(msg.header.stamp)
            if receipt_s - arrived > .30:
                pending.popleft()
                counts["dropped_tf_timeout"] += 1
                continue
            scan = Scan(stamp, np.asarray(msg.ranges), msg.angle_min, msg.angle_increment,
                        msg.range_min, msg.range_max, msg.time_increment)
            try:
                mount = mount_pose(static, msg.header.frame_id)
                base = history.at(stamp)
                end = history.at(stamp + (scan.duration_s if cfg.motion_model == "rolling" else 0))
            except LookupError:
                break
            pending.popleft()
            started = time.perf_counter()
            a, b = base.compose(mount), end.compose(mount)
            try:
                detections, stats = detector.detect(scan, a, b, base)
                tracks = tracker.update(detections, stamp)
                payload = v2x_payload(tracks, stamp)
            except ValueError as error:
                counts["rejected_input"] += 1
                rows.append(dict(stamp_s=stamp, error=str(error)))
                continue
            processing_ms.append((time.perf_counter() - started) * 1000)
            counts["processed_scans"] += 1
            counts["scans_with_tracks"] += bool(tracks)
            totals.update(stats)
            visible_candidates = []
            used_tracks = set()
            cloud, _ = scan_points(scan, a, b, cfg)
            for key, (t, xy) in native.items():
                if abs(stamp - t) > .3:
                    continue
                local = a.inverse_apply(np.array([xy]))[0]
                distance = float(np.linalg.norm(local))
                if not (2.0 <= distance <= 15.0 and local[0] > 0 and abs(np.arctan2(local[1], local[0])) < 1.5):
                    continue
                # Geometric candidate denominator, not hand-labelled visibility.
                nearby_returns = int((np.linalg.norm(cloud - xy, axis=1) < 1.5).sum())
                if nearby_returns < 4:
                    continue
                distances = [(float(np.linalg.norm(np.array(t.detection.xy_m) - xy)), i) for i, t in enumerate(tracks) if i not in used_tracks]
                match = min(distances, default=(float('inf'), -1))
                record = dict(stamp_s=stamp, native_id=key, range_m=distance, nearby_returns=nearby_returns,
                              matched=match[0] <= 2.0, error_m=None, track_id=None)
                if record["matched"]:
                    error, index = match
                    used_tracks.add(index)
                    record.update(error_m=error, track_id=tracks[index].track_id,
                                  surface_error_m=float(np.linalg.norm(np.array(tracks[index].detection.surface_xy_m)-xy)),
                                  fit_rmse_m=tracks[index].detection.fit_rmse_m,
                                  ambiguity_m=tracks[index].detection.ambiguity_m)
                matches.append(record)
                visible_candidates.append(record)
            row = dict(stamp_s=stamp, tracks=[asdict(t) for t in tracks], stats=stats,
                       v2x=payload, evaluation=visible_candidates,
                       unmatched_track_count=len(tracks) - len(used_tracks))
            rows.append(row)
            if plot_targets and stamp >= plot_targets[0]:
                plot_targets.pop(0)
                plots.append(dict(stamp_s=stamp, cloud=cloud, base=base, tracks=row["tracks"], native=dict(native)))

    wanted = {"/tf", "/tf_static", "/sensing/lidar/scan", "/v2x/vehicle_positions"}
    with AnyReader([args.bag]) as reader:
        present = {c.topic for c in reader.connections}
        if not {"/tf", "/tf_static", "/sensing/lidar/scan"}.issubset(present):
            raise ValueError("Replay requires timestamped dynamic/static TF and LaserScan")
        for connection, receipt, raw in reader.messages(connections=[c for c in reader.connections if c.topic in wanted]):
            message = reader.deserialize(raw, connection.msgtype)
            topic = connection.topic
            if topic in {"/tf", "/tf_static"}:
                for tf in message.transforms:
                    if topic == "/tf_static":
                        # Camera optical/IMU TFs are nonplanar and irrelevant.
                        if tf.child_frame_id in {"lidar", "lidar_base_link", "sensor_kit_base_link"}:
                            static[tf.child_frame_id] = (tf.header.frame_id, transform_pose(tf.transform))
                    elif tf.header.frame_id == "map" and tf.child_frame_id == "base_link":
                        history.add(seconds(tf.header.stamp), transform_pose(tf.transform))
            elif topic == "/v2x/vehicle_positions":
                for vehicle in message.vehicles:
                    if vehicle.header.frame_id != "map":
                        raise ValueError("Evaluation V2X is not in map frame")
                    native[vehicle.vehicle_id] = (seconds(vehicle.header.stamp),
                                                  (vehicle.position.x, vehicle.position.y))
            else:
                counts["input_scans"] += 1
                pending.append((message, receipt * 1e-9))
            process_pending(receipt * 1e-9, reader)
    counts["unprocessed_at_bag_end"] = len(pending)
    errors = [m["error_m"] for m in matches if m["matched"]]
    ids = sorted({m["track_id"] for m in matches if m["matched"]})
    summary = dict(schema_version=1, git_commit=subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        bag=str(args.bag), config=asdict(cfg), counts=dict(counts), point_totals=dict(totals),
        processing_ms=percentiles(processing_ms), geometric_candidate_scans=len(matches),
        matched_candidate_scans=len(errors), matched_fraction=len(errors)/len(matches) if matches else None,
        centre_error_m=percentiles(errors), matching_track_ids=ids,
        first_match=next((m for m in matches if m["matched"]), None),
        candidate_unmatched_tracks=sum(row.get("unmatched_track_count", 0) for row in rows if row.get("evaluation")),
        evaluation_oracle="native V2X positions; evaluation only; no simulator truth supplied to detector",
        denominator="2-15m ahead, bearing within 1.5rad, >=4 raw returns within 1.5m of native V2X centre",
        match_gate_m=2.0, teacher_ready=False, closed_loop_tested=False,
        notes=["Geometric candidate association is not annotated object detection recall.",
               "Surface model reports observed surface centre, not full physical object centre.",
               "Known-vehicle model uses explicit size and route-heading priors; fit ambiguity remains.",
               "No wall-adjacent obstacle guarantee or arbitrary-object geometry support in V44 V2X."])
    provenance = {str(args.map_yaml): hashlib.sha256(args.map_yaml.read_bytes()).hexdigest()}
    if args.reference_csv:
        provenance[str(args.reference_csv)] = hashlib.sha256(args.reference_csv.read_bytes()).hexdigest()
    summary["input_sha256"] = provenance
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    with (args.output / "frames.jsonl").open("w") as stream:
        for row in rows:
            stream.write(json.dumps(row, allow_nan=False) + "\n")
    if plots:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        for ax, sample in zip(axes.flat, plots):
            origin = np.array([sample["base"].x_m, sample["base"].y_m])
            xy = sample["cloud"] - origin
            ax.scatter(xy[:, 0], xy[:, 1], s=2, label="LiDAR")
            for _, (_, point) in sample["native"].items():
                p = np.array(point) - origin
                ax.plot(p[0], p[1], 'rx', markersize=10, label="Native V2X (evaluation)")
            for tr in sample["tracks"]:
                p = np.array(tr["detection"]["xy_m"]) - origin
                ax.plot(p[0], p[1], 'go', fillstyle='none')
                ax.annotate(tr["track_id"], p, fontsize=7)
            ax.plot(0, 0, 'k^', label="Ego base_link")
            ax.set(xlim=(-15, 15), ylim=(-15, 15), aspect="equal", title=f"sim {sample['stamp_s']:.2f}s", xlabel="map X relative to ego [m]", ylabel="map Y relative to ego [m]")
            ax.grid(True)
        axes.flat[0].legend(fontsize=8)
        fig.suptitle(f"LiDAR -> V2X shadow replay: {cfg.object_model}, {cfg.motion_model}")
        fig.tight_layout()
        fig.savefig(args.output / "replay.png", dpi=130)
        plt.close(fig)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
