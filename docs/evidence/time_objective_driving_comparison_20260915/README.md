# Evidence index

- `comparison/comparison.json`: six fixed-control trials; hashes, poses, replay, reference-line decomposition and exclusions.
- `codex-time-obj-*/`: per-trial judge, controller replay, transfer and environment receipts.
- `departure_data_audit.json`, `departure_comparison/summary.json`: existing train/validation departure support and same-data FP32/PP evaluation.
- `collection_locations.json`: nearest normal teacher progress for the measured runtime states, computed in native WSL.
- `runtime_export/`: original B/D checkpoints preserved, model state and prediction equality of runtime metadata exports.
- `initial_readiness_failure/`: pre-driving D smoke failure; retained as provenance, not counted as a driving trial.
- `deployment/`: source manifests and ROS smoke/build results.
- `live_visual/`: ordinary RViz capture, converted from XWD in WSL; the original raw XWD stays in native runs.
- `reproduce/`: copies of task-specific scripts executed in native WSL. Use `tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python <script>` from the native checkout. Inputs have absolute native paths; choose a fresh output location in a copy before replay. Existing output directories intentionally reject overwrite.

`transfer_manifest.json` covers the original 74 harvested files. `artifact_manifest.json` covers the final directory except itself. `.gitattributes` preserves exact bytes through Git, including Windows-generated diagnostic receipts.
Raw data, per-plan attribution details, predictions and checkpoints remain in `/home/thistle/e2e_autonomous/runs/time_objective_driving_comparison_20260915_r2` and the paths in the experiment plan. Large raw files and checkpoints are not committed.
No sealed test bags were read. These are bounded same-scene trials, not a general success-rate estimate.
