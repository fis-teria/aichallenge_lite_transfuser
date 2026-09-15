"""Inspect unperturbed raw passages in existing train/validation runs, native WSL.

This produces candidate control-tick counts, never validated Camera anchors or
new teachers. Current split membership and the frozen training cache stay intact.
"""
from __future__ import annotations

from collections import Counter
from pathlib import Path
import json
import subprocess
import sys

import numpy as np

sys.path.insert(0, str(Path.cwd()/"tools"))
from analyze_time_corner_learning import read, write, locate, nominal_reference
from analyze_time_normal_lap_attribution import verify_files
from compare_time_teacher_clearance import records
from aic_transfuser_lite.control.time_reference_v1 import TimedBodyPose
from aic_transfuser_lite.data.time_training_cache_v1 import _sha


def stats(values: list[float]) -> dict:
    if not values:
        return {"count": 0}
    a = np.asarray(values, float)
    assert np.isfinite(a).all()
    return dict(count=len(a), min=float(a.min()), median=float(np.median(a)), max=float(a.max()))


def main() -> None:
    assert sys.platform == "linux"
    assert not subprocess.check_output(["git", "status", "--porcelain"], text=True).strip()
    root = Path("/home/thistle/e2e_autonomous")
    out = root/"runs/time_corner4_learning_diagnosis_20260916_fp32"
    summary = read(out/"summary.json")
    assert summary["status"] == "COMPLETE"
    files = {r["path"]: r for r in read(out/"artifact_manifest.json")["files"]}
    for name in ("summary.json", "train_states.json", "validation_states.json"):
        assert _sha(out/name) == files[name]["sha256"]
    trial = root/"runs/time_multiscale_model_lap_20260916/raw/codex-time-multiscale-lap01"
    assert _sha(trial/"trial_config.json") == summary["trial_hashes"]["trial_config.json"]
    controller = read(trial/"trial_config.json")
    reference, base, proof = nominal_reference(root, controller)
    assert proof == summary["reference"]
    plan = read(Path("configs/time_path_p1/recovery_multiscale_20260916.json"))
    result = dict(scope="RAW_NOMINAL_CANDIDATES_NOT_VALIDATED_CAMERA_ANCHORS", new_teachers=0,
        reference=proof, source_commit=subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
        summary_sha256=_sha(out/"summary.json"), runs=[])
    for split, suffix in (("train", "left-r04"), ("validation", "left-r05")):
        rid = "codex-time-recovery-60cm-d60-g03-"+suffix
        assert next(r for r in plan["additions"] if r["run_id"] == rid)["split"] == split
        run = root/"raw/time_recovery_60cm_20260916"/rid
        hashes = verify_files(run, ("control.jsonl", "result.json", "reference.json"))
        outcome = read(run/"result.json")
        assert outcome["status"] == "COMPLETE_LAP" and outcome["last_control"].get("fault") is None
        assert outcome["simulator_assets"] == {"AWSIM_Data/level1": controller["geometry"]["scene_sha256"],
                                               **controller["steering_asset_sha256"]}
        states = []
        for row in records(run/"control.jsonl"):
            if (row.get("reason") != "RECOVERY_TEACHER_TRACKING" or row.get("phase") != "baseline"
                    or not row.get("current_pose")):
                continue
            state = locate(TimedBodyPose(**row["current_pose"]), row["speed_mps"], reference, base)
            states.append(state)
        cached = [r for r in read(out/(split+"_states.json")) if r["run_id"] == rid]
        windows = {}
        for label, (low, high) in {"whole_corner": (165., 210.), "early_failure": (178., 193.)}.items():
            chosen = [r for r in states if r["status"] == "CORNER" and low <= r["base_s_m"] <= high]
            cache_rows = [r for r in cached if r["status"] == "CORNER" and low <= r["base_s_m"] <= high]
            windows[label] = dict(progress_m=[low, high], baseline_control_ticks=len(chosen),
                cached_anchors_in_window=len(cache_rows),
                **{k: stats([r[k] for r in chosen]) for k in ("base_s_m", "left_m", "heading_rad", "speed_mps")})
        assert windows["early_failure"]["baseline_control_ticks"] > 0
        result["runs"].append(dict(run_id=rid, split=split, source_hashes=hashes,
            locate_statuses=dict(Counter(r["status"] for r in states)), windows=windows))
    write(out/"unused_nominal_candidates.json", result)
    write(out/"unused_nominal_manifest.json", {"unused_nominal_candidates.json": _sha(out/"unused_nominal_candidates.json")})
    print(json.dumps(result))


if __name__ == "__main__":
    main()
