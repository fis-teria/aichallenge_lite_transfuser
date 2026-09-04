from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path

from aic_transfuser_lite.data.mcap_reader_v3 import read_teacher_state_streams_v3
from aic_transfuser_lite.data.recovery_phase_view_v3 import (
    build_recovery_phase_labels_v3,
    write_recovery_phase_view_v3,
)
from aic_transfuser_lite.data.storage_v3 import validate_complete_dataset
from aic_transfuser_lite.data.topic_profile_v3 import load_topic_profile_v3


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Build an external recovery-phase view for a Canonical Dataset V3."
    )
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--raw-root", type=Path, required=True)
    parser.add_argument("--topic-profile", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-pose-delta-ms", type=float, default=50.0)
    args = parser.parse_args()

    dataset_manifest = validate_complete_dataset(args.dataset_root)
    with (args.dataset_root / "samples.csv").open(newline="", encoding="utf-8") as stream:
        sample_rows = list(csv.DictReader(stream))
    profile = load_topic_profile_v3(args.topic_profile)
    labels = []
    sources = []
    run_ids = sorted({row["run_id"] for row in sample_rows})
    for run_id in run_ids:
        raw_run = args.raw_root / run_id
        bag = raw_run / run_id
        generated_reference = raw_run / "recovery_reference_v3.csv"
        base_reference = raw_run / "base_mpc_collection_reference.csv"
        intervals = raw_run / "recovery_reference_v3.intervals.csv"
        for path in (bag / "metadata.yaml", generated_reference, base_reference, intervals):
            if not path.is_file():
                raise FileNotFoundError(f"run {run_id!r} source is missing: {path}")
        streams = read_teacher_state_streams_v3(bag, profile=profile)
        run_labels = build_recovery_phase_labels_v3(
            sample_rows,
            streams.poses,
            run_id=run_id,
            generated_reference_path=generated_reference,
            base_reference_path=base_reference,
            intervals_path=intervals,
            max_pose_delta_ms=args.max_pose_delta_ms,
        )
        labels.extend(run_labels)
        sources.append(
            {
                "run_id": run_id,
                "bag_metadata_sha256": _sha256(bag / "metadata.yaml"),
                "generated_reference_sha256": _sha256(generated_reference),
                "base_reference_sha256": _sha256(base_reference),
                "intervals_sha256": _sha256(intervals),
                "pose_count": len(streams.poses),
                "label_count": len(run_labels),
            }
        )
    if len(labels) != len(sample_rows):
        raise ValueError(
            f"phase-label count {len(labels)} does not match Dataset V3 "
            f"sample count {len(sample_rows)}"
        )
    payload = write_recovery_phase_view_v3(
        args.output,
        dataset_manifest_sha256=str(dataset_manifest["manifest_sha256"]),
        labels=labels,
        source_records=sources,
    )
    print(json.dumps({
        "status": "COMPLETE",
        "sample_count": payload["sample_count"],
        "eligible_sample_count": payload["eligible_sample_count"],
        "phase_counts": payload["phase_counts"],
        "max_pose_delta_ms": payload["pose_delta_ms"]["max"],
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
