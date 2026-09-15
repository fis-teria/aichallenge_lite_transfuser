"""Copy a fixed, small evidence allowlist; retain all weights and arrays in WSL."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import shutil
import sys


def read(path: Path) -> dict:
    return json.loads(path.read_text())


def main(root: Path) -> None:
    run = root / "runs/time_recovery_objective_comparison_20260915"
    execution = run.with_name(run.name + "_execution")
    receipt = read(execution / "experiment_receipt.json")
    assert receipt["status"] == "COMPLETE"
    assert all(stage["exit_code"] == 0 for stage in receipt["stages"])
    assert read(run / "comparison/summary.json")["status"] == "COMPLETE"
    assert read(execution / "post_analysis/paired_analysis.json")["status"] == "COMPLETE"
    files: dict[str, Path] = {}
    for name in ("data_and_budget_verification.json", "geometry_identity.json", "recovery_geometry_targets.json"):
        files[name] = run / name
    for arm in ("uniform_l1", "balanced_l1", "uniform_geometry", "balanced_geometry"):
        directory = root / "runs/time_recovery_random_update_20260915" if arm == "uniform_l1" else run / arm
        assert read(directory / "result.json")["status"] == "COMPLETE"
        for name in ("result.json", "history.json", "plan.json", "best_validation.json"):
            files[f"{arm}/{name}"] = directory / name
        if arm != "uniform_l1":
            assert read(directory / "comparison_verification.json")["status"] == "PASS"
            files[f"{arm}/comparison_verification.json"] = directory / "comparison_verification.json"
    for name in ("recovery_objective_comparison.png", "artifact_manifest.json"):
        files[f"comparison/{name}"] = run / "comparison" / name
    files["comparison/paired_analysis.json"] = execution / "post_analysis/paired_analysis.json"
    for path in sorted(execution.iterdir()):
        if path.is_file() and (path.suffix == ".log" or path.name.endswith("receipt.json")):
            files[f"execution/{path.name}"] = path
    destination = execution / "windows_evidence"
    destination.mkdir(exist_ok=False)
    manifest = []
    for relative, source in files.items():
        assert source.suffix in {".json", ".log", ".png"} and source.stat().st_size < 5_000_000
        target = destination / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        data = source.read_bytes()
        digest = hashlib.sha256(data).hexdigest()
        shutil.copyfile(source, target)
        assert hashlib.sha256(target.read_bytes()).hexdigest() == digest
        manifest.append({"relative_path": relative, "source_path": str(source),
                         "bytes": len(data), "sha256": digest})
    evidence = {"status": "PASS", "training_source_commit": receipt["source_commit"],
                "raw_arrays_and_checkpoints_copied": False, "files": manifest}
    (destination / "native_transfer_manifest.json").write_text(json.dumps(evidence, indent=2) + "\n")
    print(json.dumps({"status": "PASS", "destination": str(destination), "files": len(manifest),
                      "bytes": sum(row["bytes"] for row in manifest)}, indent=2))


if __name__ == "__main__":
    main(Path(sys.argv[1]).resolve())
