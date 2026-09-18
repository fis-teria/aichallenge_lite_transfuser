# Native obstacle fit diagnostic

This separate experiment preserves the existing full training recipe, checkpoints,
model inputs and recovery objective. It compares a 2x2 intervention from the same
native-replay epoch-3 checkpoint: old native sampling fraction / 8 of 32 native
samples, and native geometry objective off / on. Each arm uses seed 42, 384
optimizer updates, batch 32, FP32, AdamW learning rate 1e-5, weight decay 1e-4.
The focused batches contain four uniformly sampled native anchors and four
explicit front-approach anchors. This short draw is not a full replay epoch.

The frozen population contains 338 native training anchors, 110 with the selected
cone ahead. These are overlapping windows, not independent avoidance events.
Geometry supervision uses teacher-fixed PP horizon and response length, with
1 m/rad times absolute tire-angle error plus 0.5 times far-lateral metre error.
Labels remain outside inference inputs. Only original train runs enter updates.

Run on native WSL after committing on Windows and synchronizing the clean source:

```bash
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src timeout 7200 .venv/bin/python tools/compare_native_fit.py --root /home/thistle/e2e_autonomous
```

Evaluation covers all 20,467 existing eligible validation windows, per-run recovery,
210 launch cases, native training fit and runtime PP calculations on the 110 front
windows. Reports distinguish the fine-tuning initializer from the deployed model.
Test runs stay sealed. Neither training-fit improvement nor this retention gate
proves obstacle avoidance. No checkpoint is automatically deployed.

In parallel, the unchanged PC10 MPPI V45 teacher collects single native objects,
starting at the failed E2E cone placement and ending 25 m after it, at 5 km/h.
Use the existing `collect_mppi_v45.py` with `--rviz --wall-timeout-s 480
--run-budget-gib 0.75 --free-reserve-gib 2 --execute`. Preserve raw recordings,
scenario and runtime identity, transfer with all-file hash verification, and reject
collision, drift or incomplete approach/pass/recovery events before adding labels.
AWSIM binaries/assets are unchanged. New collection does not enter this comparison.

The curation tool also supports verified single-object collection without changing
the original twelve-object default. Run/scenario split membership is preserved;
validation/test placements cannot be admitted by the train curation path.

```bash
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python tools/curate_native_teacher_data.py \
  --root /home/thistle/e2e_autonomous/runs/mppi_v45_pc10_20260918 \
  --output /home/thistle/e2e_autonomous/runs/mppi_v45_pc10_20260918/front_curated_v1 \
  --run-pattern 'lidar-v45-pc10-front-*' \
  --prefix-directory front_pose_prefix_v1 --clearance-directory front_prefix_clearance_v1
```

This command requires prior pose-prefix and clearance audits. An absent object
kind uses a finite 1e6 m sentinel; it does not bypass dynamic-box exclusion, the
0.30 m clearance screen, the additional 0.30 m cone projection margin, or the
full-history/future scan/map and teacher-command checks. Native box pose during
motion remains unverified, so nearby box windows remain held.
