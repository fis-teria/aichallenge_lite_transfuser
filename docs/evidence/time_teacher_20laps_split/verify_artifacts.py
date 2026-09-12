"""One-shot WSL audit of generated labels and references; no model evaluation."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import sqlite3

import numpy as np
from aic_transfuser_lite.data.time_split_v1 import validate_time_split


def main(root: Path) -> None:
    manifest = json.loads((root / "split_verified.json").read_text())
    validate_time_split(manifest, require_verified=True)
    report = json.loads((root / "audit.json").read_text())
    per_run = {r["run_id"]: r for r in report["runs"]}
    records = []
    files = []
    for run in manifest["runs"]:
        path = root / run["split"] / run["run_id"]
        raw = (path / "raw").resolve(strict=True)
        assert raw.name == run["run_id"]
        assert raw == Path(per_run[run["run_id"]]["raw_run_dir"])
        with sqlite3.connect(f"file:{raw.as_posix()}/bag/bag_0.db3?mode=ro&immutable=1", uri=True) as conn:
            camera_receipts = dict(conn.execute("SELECT m.id,m.timestamp FROM messages m JOIN topics t ON m.topic_id=t.id WHERE t.name='/sensing/camera/image_raw'"))
        with np.load(path / "teachers.npz", allow_pickle=False) as data:
            xy, mask = data["xy_m"], data["xy_mask"]
            n = len(xy)
            assert xy.shape == (n, 30, 2) and xy.dtype == np.float32
            assert mask.shape == (n, 30) and mask.dtype == np.bool_
            assert np.isfinite(xy[mask]).all() and np.isnan(xy[~mask]).all()
            vel, vm = data["velocity_mps"], data["velocity_mask"]
            assert vel.shape == (n, 30) and vm.shape == (n, 30)
            assert np.isfinite(vel[vm]).all() and np.isnan(vel[~vm]).all()
            np.testing.assert_array_equal(data["interval_mask"], mask & np.c_[np.ones(n, bool), mask[:, :-1]])
            np.testing.assert_array_equal(data["usable_full"], data["input_eligible"] & mask.all(axis=1))
            np.testing.assert_array_equal(data["usable_partial"], data["input_eligible"] & mask.any(axis=1))
            assert np.all(np.diff(data["observation_ns"]) > 0)
            count = 0
            with (path / "anchors.jsonl").open() as stream:
                for i, line in enumerate(stream):
                    row = json.loads(line)
                    assert row["label_index"] == i and row["run_id"] == run["run_id"] and row["split"] == run["split"]
                    assert row["camera_row_id"] == data["camera_row_id"][i]
                    assert row["observation_ns"] == data["observation_ns"][i]
                    assert row["freeze_ns"] == data["freeze_ns"][i]
                    assert row["freeze_ns"] == camera_receipts[row["camera_row_id"]] + 50_000_000
                    assert row["usable_full"] == bool(data["usable_full"][i])
                    assert row["usable_partial"] == bool(data["usable_partial"][i])
                    if row["stop_reason"] == "COLLECTION_INTERVENTION":
                        assert not mask[i].any() and not vm[i].any()
                    count += 1
            assert count == n == per_run[run["run_id"]]["anchors"]
            assert int(data["usable_full"].sum()) == per_run[run["run_id"]]["usable_full"]
            steps = np.diff(np.concatenate([np.zeros((n, 1, 2), np.float32), xy], axis=1), axis=1)
            speed = np.linalg.norm(steps, axis=2) / .1
            supported = data["interval_mask"] & vm
            difference = np.abs(speed[supported] - np.abs(vel[supported]))
            records.append({"run_id": run["run_id"], "split": run["split"], "anchors": n,
                "usable_full": int(data["usable_full"].sum()),
                "geometry_speed_difference_mps_quantiles": np.quantile(difference, [.5, .95, .99, 1]).tolist(),
                "interpretation": "0.1s XY chord speed vs endpoint longitudinal speed; data consistency only"})
        for name in ("anchors.jsonl", "teachers.npz", "audit.json"):
            artifact = path / name
            digest = hashlib.sha256()
            with artifact.open("rb") as stream:
                for block in iter(lambda: stream.read(1024 * 1024), b""):
                    digest.update(block)
            files.append({"path": artifact.relative_to(root).as_posix(), "bytes": artifact.stat().st_size, "sha256": digest.hexdigest()})
    with (root / "artifact_manifest.json").open("x") as stream:
        json.dump({"format": "time_corpus_artifacts_v1", "files": files}, stream, indent=2)
    result = {"status": "PASS", "runs": records, "artifacts_hashed": len(files),
              "total_anchors": sum(r["anchors"] for r in records),
              "usable_full": sum(r["usable_full"] for r in records), "model_evaluated": False}
    with (root / "post_generation_verification.json").open("x") as stream:
        json.dump(result, stream, indent=2)
    print(json.dumps(result), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    main(parser.parse_args().root)
