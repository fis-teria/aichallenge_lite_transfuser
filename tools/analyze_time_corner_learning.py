"""Audit corner-state coverage and frozen-checkpoint fit in native WSL.

Read train/validation only. Keep all original membership and observations;
never turn the failed E2E trajectory into a recovery label or update weights.
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
import sys
from typing import Any

import numpy as np

from aic_transfuser_lite.control.time_reference_v1 import TimedBodyPose
from aic_transfuser_lite.data.time_recovery_collection_v1 import project_course
from aic_transfuser_lite.data.time_split_v1 import content_sha256
from aic_transfuser_lite.data.time_training_cache_v1 import TimeTrainingCacheDataset, verify_time_training_cache, _sha
from aic_transfuser_lite.evaluation.time_corner_comparison_v1 import RecordedLine, angle_delta, replay_observation_pose


CORNER_M = (165., 210.)
NEIGHBORHOODS = {"narrow": (3., .15, math.radians(5.), .25),
                 "wide": (5., .25, math.radians(10.), .4)}
STATE_KEYS = ("base_s_m", "left_m", "heading_rad", "speed_mps")


def read(path: Path) -> Any:
    return json.loads(path.read_text())


def write(path: Path, value: Any) -> None:
    with path.open("x") as stream:
        json.dump(value, stream, indent=2, allow_nan=False)


def neighborhood(rows: list[dict], query: dict, tolerance: tuple[float, ...]) -> list[int]:
    """Indices within [progress m, left m, wrapped heading rad, speed m/s]."""
    tol = np.asarray(tolerance, float)
    q = np.asarray([query[k] for k in STATE_KEYS], float)
    if tol.shape != (4,) or not np.isfinite(tol).all() or (tol <= 0).any() or not np.isfinite(q).all():
        raise ValueError("NEIGHBORHOOD_UNITS_OR_FINITE")
    result = []
    for i, row in enumerate(rows):
        if row["status"] != "CORNER":
            continue
        p = np.asarray([row[k] for k in STATE_KEYS], float)
        if not np.isfinite(p).all():
            raise ValueError("NONFINITE_CORNER_STATE")
        delta = p-q
        delta[2] = angle_delta(p[2], q[2])
        if (np.abs(delta) <= tol+1e-12).all():
            result.append(i)
    return result


def population(rows: list[dict], presentations: dict[str, int] | None = None) -> dict:
    """Unique anchors, independent run/event counts, and repeated presentations."""
    if len({r["anchor_id"] for r in rows}) != len(rows):
        raise ValueError("DUPLICATE_POPULATION_ANCHOR")
    runs = Counter(r["run_id"] for r in rows)
    events = {(r["run_id"], r["event_id"]) for r in rows if r.get("event_id") is not None}
    result = dict(anchors=len(rows), runs=len(runs), per_run=dict(runs), recovery_events=len(events))
    if presentations is not None:
        if any(r["split"] != "train" for r in rows):
            raise ValueError("VALIDATION_HAS_NO_TRAINING_PRESENTATIONS")
        result["presentations_per_epoch"] = sum(presentations[r["anchor_id"]] for r in rows)
    return result


def fit_indices(rows: list[dict]) -> list[int]:
    """All supported recovery anchors plus every supported nominal corner anchor."""
    return [i for i, r in enumerate(rows) if r["eligible"] and (r["recovery"] or r["status"] == "CORNER")]


def message_pose(msg: Any, epoch: str) -> TimedBodyPose:
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    return TimedBodyPose(int(msg.header.stamp.sec)*10**9+int(msg.header.stamp.nanosec), "sim", epoch,
        msg.header.frame_id, msg.child_frame_id, float(p.x), float(p.y),
        math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))


def connection(path: Path) -> sqlite3.Connection:
    return sqlite3.connect(f"file:{path.resolve()}?mode=ro&immutable=1", uri=True)


def nominal_reference(root: Path, trial_config: dict) -> tuple[RecordedLine, np.ndarray, dict]:
    from analyze_time_normal_lap_attribution import verify_files
    from compare_time_teacher_clearance import records
    from rosbags.typesys import Stores, get_typestore
    run = root/"raw/time_recovery_speed_20260914/codex-time-recovery-speedbase-r30"
    hashes = verify_files(run, ("control.jsonl", "result.json", "reference.json", "bag/bag_0.db3"))
    result = read(run/"result.json")
    assert result["status"] == "COMPLETE_LAP" and result["fixed_target_mps"] == 5./3.6
    assert result["simulator_assets"] == {"AWSIM_Data/level1": trial_config["geometry"]["scene_sha256"], **trial_config["steering_asset_sha256"]}
    base = np.asarray(read(run/"reference.json")["baseline_xy_m"], float)
    controls = [r for r in records(run/"control.jsonl") if r.get("reason") == "RECOVERY_TEACHER_TRACKING"
                and 100. <= r["projection"]["s_m"] <= 235.]
    assert controls and all(r["phase"] == "baseline" for r in controls)
    low, high = min(r["current_pose"]["stamp_ns"] for r in controls), max(r["current_pose"]["stamp_ns"] for r in controls)
    store = get_typestore(Stores.ROS2_HUMBLE)
    poses = []
    with connection(run/"bag/bag_0.db3") as con:
        topics = con.execute("select id,type from topics where name='/localization/kinematic_state'").fetchall()
        assert len(topics) == 1 and topics[0][1] == "nav_msgs/msg/Odometry"
        for data, in con.execute("select data from messages where topic_id=? order by id", (topics[0][0],)):
            p = message_pose(store.deserialize_cdr(data, topics[0][1]), "0")
            if low <= p.stamp_ns <= high:
                poses.append(p)
    line = RecordedLine(poses)
    return line, base, dict(run=run.name, hashes=hashes, pose_audit=line.audit,
        scope="MEASURED_COMPLETE_TEACHER_NOT_ROAD_CENTER_GROUND_TRUTH", reference_progress_window_m=[100., 235.])


def locate(pose: TimedBodyPose, speed: float, reference: RecordedLine, base: np.ndarray) -> dict:
    try:
        coarse = project_course(base, [pose.x_m, pose.y_m], pose.yaw_rad)
        result = dict(base_s_m=coarse["s_m"], speed_mps=speed, world_pose=asdict(pose))
        if not CORNER_M[0] <= coarse["s_m"] <= CORNER_M[1]:
            return dict(result, status="OUTSIDE_CORNER")
        projected = reference.project(np.array([pose.x_m, pose.y_m]), yaw_hint_rad=pose.yaw_rad)
        return dict(result, status="CORNER", left_m=projected.left_m,
            heading_rad=angle_delta(pose.yaw_rad, projected.body_yaw_rad), reference_progress_m=projected.progress_m)
    except ValueError as exc:
        return dict(status=str(exc), speed_mps=speed, world_pose=asdict(pose))


def source_bag(root: Path, cache: Path, run: dict) -> Path:
    rid = run["run_id"]
    preferred = (cache/"materialized"/rid/"raw/bag/bag_0.db3" if rid.startswith("codex-time-recovery-") else
                 root/"datasets/processed/time_teacher_20laps_20260913"/run["split"]/rid/"raw/bag/bag_0.db3")
    if not preferred.is_file():
        found = list((root/"raw").glob("*/"+rid+"/bag/bag_0.db3"))
        if len(found) != 1:
            raise ValueError("UNIQUE_RAW_BAG_MISSING:"+rid)
        preferred = found[0]
    sources = [s for s in run["sources"] if s["path"] == f"runs/{rid}/bag/bag_0.db3"]
    if len(sources) != 1 or _sha(preferred) != sources[0]["sha256"]:
        raise ValueError("RAW_BAG_IDENTITY_MISMATCH:"+rid)
    return preferred.resolve()


def states(ds: TimeTrainingCacheDataset, root: Path, reference: RecordedLine,
           base: np.ndarray, split: str, out: Path) -> tuple[list[dict], dict]:
    from rosbags.typesys import Stores, get_typestore
    store = get_typestore(Stores.ROS2_HUMBLE)
    metadata = {r["run_id"]: r for r in ds.split_manifest["runs"] if r["split"] == split}
    result, proof = [], {}
    offset = 0
    for local_run in ds._runs:
        rid = ds.run_ids[offset]; n = len(local_run["inputs"]["input_valid"])
        anchors = ds._anchors[offset:offset+n]
        assert all(r["run_id"] == rid for r in anchors)
        raw = source_bag(root, ds.root, metadata[rid])
        needed = sorted({j for i, a in enumerate(anchors) if ds.input_valid[offset+i] and ds.xy_mask[offset+i].all()
                         for j in a["observation_pose_row_ids"]})
        sources = {}
        with connection(raw) as con:
            types = dict(con.execute("select id,type from topics"))
            for start in range(0, len(needed), 400):
                ids = needed[start:start+400]
                for sequence, receipt, topic, data in con.execute(
                    "select id,timestamp,topic_id,data from messages where id in ("+",".join("?" for _ in ids)+")", ids):
                    if types[topic] != "nav_msgs/msg/Odometry":
                        raise ValueError("OBSERVATION_POSE_ROW_TYPE")
                    sources[sequence] = (store.deserialize_cdr(data, types[topic]), int(receipt))
        assert len(sources) == len(needed)
        for local, a in enumerate(anchors):
            i = offset+local
            row = dict(index=i, anchor_id=a["anchor_id"], run_id=rid, split=split,
                observation_ns=a["observation_ns"], recovery=rid.startswith("codex-time-recovery-"),
                event_id=a.get("recovery_event_id"), site_id=a.get("recovery_site_id"),
                eligible=bool(ds.input_valid[i] and ds.xy_mask[i].all()), status="INPUT_OR_FULL_TEACHER_UNSUPPORTED")
            if row["eligible"]:
                pairs = [(message_pose(sources[j][0], a["epoch"]), sources[j][1]) for j in a["observation_pose_row_ids"]]
                pose = replay_observation_pose(pairs, observation_ns=a["observation_ns"], freeze_receipt_ns=a["freeze_ns"])
                speed = float(local_run["inputs"]["ego"][local, -1, 0])
                row.update(locate(pose, speed, reference, base))
            result.append(row)
        selected = result[offset:offset+n]
        proof[rid] = dict(bag=str(raw), sha256=metadata[rid]["sources"][0]["sha256"],
            anchors=n, statuses=dict(Counter(r["status"] for r in selected)), replayed_pose_rows=len(needed))
        offset += n
        print("CORNER_STATE_AUDITED", split, rid, proof[rid]["statuses"], flush=True)
    assert len(result) == len(ds)
    write(out/(split+"_states.json"), result)
    return result, proof


def groups(rows: list[dict]) -> dict[str, list[int]]:
    result = {"all_recovery": [i for i, r in enumerate(rows) if r["recovery"]],
              "corner_all": [i for i, r in enumerate(rows) if r["status"] == "CORNER"]}
    for name, predicate in (
        ("nominal_5kmh", lambda r: r["run_id"].startswith("5kmh_")),
        ("nominal_8kmh", lambda r: r["run_id"].startswith("8kmh_")),
        ("recovery", lambda r: r["recovery"]),
        ("left_ge_20cm", lambda r: r["left_m"] >= .2),
        ("left_ge_40cm", lambda r: r["left_m"] >= .4),
        ("left_ge_60cm", lambda r: r["left_m"] >= .6),
        ("outward_heading_ge_5deg", lambda r: r["heading_rad"] >= math.radians(5.))):
        result["corner_"+name] = [i for i, r in enumerate(rows) if r["status"] == "CORNER" and predicate(r)]
    return result


def main() -> None:
    import resource
    import time
    import torch
    from torch.utils.data import Subset
    from analyze_time_normal_lap_attribution import verify_files
    from analyze_time_recovery_fit import report_group
    from compare_time_teacher_clearance import records
    from compare_time_training_methods import pp_rows
    from aic_transfuser_lite.evaluation.time_recovery_fit_v1 import predict_recovery_fit
    from aic_transfuser_lite.training.time_checkpoint_v1 import TimeCheckpointIdentity, load_time_checkpoint
    from aic_transfuser_lite.training.time_config_v1 import TimeModelConfig, build_time_model
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--root", type=Path, default=Path(".."))
    ap.add_argument("--plan", type=Path, default=Path("configs/time_path_p1/recovery_multiscale_20260916.json"))
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args(); root = args.root.resolve(); out = args.output.resolve()
    if sys.platform != "linux" or not torch.cuda.is_available():
        raise RuntimeError("NATIVE_WSL_CUDA_REQUIRED")
    if subprocess.check_output(["git", "status", "--porcelain"], text=True).strip():
        raise RuntimeError("CLEAN_COMMITTED_SOURCE_REQUIRED")
    torch.set_num_threads(4)
    soft, hard = resource.getrlimit(resource.RLIMIT_NOFILE)
    resource.setrlimit(resource.RLIMIT_NOFILE, (max(soft, min(hard, 8192)), hard))
    started = time.monotonic(); out.mkdir(parents=True, exist_ok=False)
    plan = read(args.plan); cache_path = root/plan["cache"]
    cache = verify_time_training_cache(cache_path)
    assert cache["plan"] == plan and cache["manifest_sha256"] == "f2b463b73f1d1cb7993b151754ca16488fcbd5748887375ee71b30d56316efc9"
    training = root/plan["output"]
    verification = read(training/"multiscale_verification.json")
    assert verification["status"] == "PASS" and verification["reload_predictions_exact"]
    checkpoints = {"initial": root/plan["initialization"], "epoch_01": training/"epoch_01.pt",
                   "epoch_02": training/"epoch_02.pt", "selected_epoch_03": training/"best.pt"}
    hashes = {k: _sha(p) for k, p in checkpoints.items()}
    assert hashes["initial"] == plan["initialization_sha256"]
    assert hashes["selected_epoch_03"] == verification["best_checkpoint_sha256"] == "685f8b8f9936ab7e272be12616982374fd8a8d61fbe4a304b25475dc2a206ba8"
    trial = root/"runs/time_multiscale_model_lap_20260916/raw/codex-time-multiscale-lap01"
    trial_hashes = verify_files(trial, ("control.jsonl", "trial_config.json"))
    controller = read(trial/"trial_config.json")
    reference, base, ref_proof = nominal_reference(root, controller)
    commands = records(trial/"control.jsonl")
    armed = next(r["sim_ns"] for r in commands if r["event"] == "ARMED")
    fault = next(r for r in commands if r["event"] == "SCAN_GUARD_REJECTED")
    moving = [r for r in commands if r["event"] == "COMMAND_SENT" and r.get("details", {}).get("current_pose")
              and armed <= r["sim_ns"] <= fault["sim_ns"]]
    queries = []
    for target_s in (150., 155., 160., (fault["sim_ns"]-armed)/1e9):
        c = min(moving, key=lambda r: abs((r["sim_ns"]-armed)/1e9-target_s))
        state = locate(TimedBodyPose(**c["details"]["current_pose"]), c["speed_mps"], reference, base)
        assert state["status"] == "CORNER"
        queries.append(dict(time_s=(c["sim_ns"]-armed)/1e9, **state))
    proof_path = root/plan["audit_output"]/"data_and_budget_verification.json"
    prep = read(proof_path)
    assert prep["cache_sha256"] == cache["manifest_sha256"]
    summary = dict(status="AUDITING", source_commit=subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
        scope="POST_SELECTION_TRAIN_FIT_AND_STATE_COVERAGE_NOT_CAUSAL_OR_CLOSED_LOOP",
        cache_sha256=cache["manifest_sha256"], plan_sha256=_sha(args.plan), preparation_sha256=_sha(proof_path),
        trial_hashes=trial_hashes, reference=ref_proof, checkpoints={k: dict(path=str(p), sha256=hashes[k]) for k, p in checkpoints.items()},
        corner_base_progress_m=CORNER_M, neighborhoods_m_rad_mps=NEIGHBORHOODS, runtime_queries=queries,
        reserved_test_read=False, optimizer_updates=0, checkpoint_selection_changed=False,
        new_awsim_trials=0, batch_size=32, workers=4, precision="float32", datasets={}, models={})
    write(out/"diagnostic_plan.json", summary)
    data = {}
    for split in ("train", "validation"):
        ds = TimeTrainingCacheDataset(cache_path, split, verify_hashes=False)
        rows, raw_proof = states(ds, root, reference, base, split, out)
        ix = fit_indices(rows); selected = [rows[i] for i in ix]; gs = groups(selected)
        presentations = None
        if split == "train":
            presentations = {r["anchor_id"]: prep["sampler"]["per_anchor_presentations"][r["anchor_id"]] if r["recovery"] else 1 for r in rows}
            assert sum(presentations.values()) == plan["expected"]["presentations_per_epoch"]
        neighbors = [{name: population([rows[i] for i in neighborhood(rows, q, tol)], presentations)
                      for name, tol in NEIGHBORHOODS.items()} for q in queries]
        report = dict(all_cache_anchors=len(rows), statuses=dict(Counter(r["status"] for r in rows)),
            groups={k: population([selected[i] for i in v], presentations) for k, v in gs.items()},
            neighborhood_counts=neighbors, raw_sources=raw_proof, fit_anchor_order_sha256=content_sha256([r["anchor_id"] for r in selected]))
        summary["datasets"][split] = report
        data[split] = dict(ds=ds, indices=ix, rows=selected, groups=gs,
                          teacher_pp=pp_rows(ds, ix, ds.targets[ix], controller, teacher=True))
        print("CORNER_COVERAGE", split, json.dumps({k: v for k, v in report.items() if k != "raw_sources"}), flush=True)
    assert not set(r["run_id"] for r in data["train"]["rows"]) & set(r["run_id"] for r in data["validation"]["rows"])
    write(out/"coverage.json", summary)
    summary["status"] = "EVALUATING_FROZEN_CHECKPOINTS"
    for name, checkpoint in checkpoints.items():
        payload = torch.load(checkpoint, map_location="cpu", weights_only=False)
        config = TimeModelConfig.from_dict(payload["config"])
        identity = TimeCheckpointIdentity(**payload["identity"])
        assert not config.use_command_history and config.dataset_config() == data["train"]["ds"].config
        if name != "initial":
            assert identity.split_manifest_sha256 == cache["split_manifest"]["manifest_sha256"]
            assert payload["teacher_manifest"]["preparation_sha256"] == summary["preparation_sha256"]
        model = build_time_model(config).to("cuda")
        load_time_checkpoint(checkpoint, config=config, identity=identity, model=model, mode="finetune")
        stage = dict(epoch=payload["epoch"], checkpoint_sha256=hashes[name], splits={})
        for split, d in data.items():
            ds, ix, rows = d["ds"], d["indices"], d["rows"]
            values = predict_recovery_fit(model, Subset(ds, ix), run_ids=[r["run_id"] for r in rows],
                anchor_ids=[r["anchor_id"] for r in rows], split_manifest=cache["split_manifest"], split=split,
                batch_size=32, workers=4).numpy()
            np.save(out/(name+"_"+split+"_predictions.npy"), values, allow_pickle=False)
            proposed = pp_rows(ds, ix, values, controller)
            report = {}
            for group, indices in d["groups"].items():
                if not indices:
                    report[group] = dict(anchor_count=0, run_count=0)
                    continue
                target = ds.targets[[ix[i] for i in indices]]
                truth_pp, model_pp = [d["teacher_pp"][i] for i in indices], [proposed[i] for i in indices]
                report[group] = report_group(values[indices], target, [rows[i]["run_id"] for i in indices], truth_pp, model_pp)
                pairs = [b["steer_rad"]-a["steer_rad"] for a, b in zip(truth_pp, model_pp) if a["accepted"] and b["accepted"]]
                report[group]["accepted_pp_signed_bias_rad"] = float(np.mean(pairs)) if pairs else None
            stage["splits"][split] = report
            write(out/(name+"_"+split+"_metrics.json"), report)
            if name == "selected_epoch_03" and split == "validation":
                saved = training/"comparison/after_predictions.npy"
                old = np.load(saved, allow_pickle=False)[ix]
                np.testing.assert_allclose(values, old, rtol=1e-5, atol=2e-6)
                summary["saved_validation_replay"] = dict(prediction_sha256=_sha(saved),
                    max_abs_difference_m=float(np.max(np.abs(values-old))), rtol=1e-5, atol_m=2e-6)
            print("CORNER_FIT_COMPLETE", name, split, len(rows), flush=True)
        summary["models"][name] = stage
        write(out/(name+"_complete.json"), stage)
        del model, payload
        torch.cuda.empty_cache()
    assert all(_sha(p) == hashes[k] for k, p in checkpoints.items())
    summary.update(status="COMPLETE", elapsed_s=time.monotonic()-started)
    write(out/"summary.json", summary)
    write(out/"artifact_manifest.json", dict(files=[dict(path=p.name, bytes=p.stat().st_size, sha256=_sha(p)) for p in sorted(out.iterdir()) if p.is_file()]))
    print("CORNER_LEARNING_DIAGNOSIS_COMPLETE", summary["elapsed_s"], flush=True)


if __name__ == "__main__":
    main()
