# Astra review P0 implementation

Scope: fix reproduced teacher/history bugs, then synthetic time oracle -> existing
Pure Pursuit steering/proportional longitudinal control. No training, raw transfer,
ROS/AWSIM or actuator output. Review source: `Astra Pro/astra_2026_1006_review_ja.md`.

## P0-A

B01/B02: canonical indexes are clipped to each epoch BEFORE interpolation and future
pose frame/child is checked against anchor. Legacy schema retains combined validity.
New `data/time_teacher_v1.py` supplies independent XY/velocity masks and source endpoints,
rejects cross-run/epoch/frame and intervention endpoint mixing. 50 ms is per endpoint.
Missing commands/velocity cannot invalidate XY; interval mask requires both XY endpoints.
The caller must supply a pose at the camera observation time, not relabel a nearest pose.

D02: legacy reader retains its default behavior, with an optional pre-dedup event sink.
`read_time_events` captures every decoded event and assigns epoch using bag receipt bounds.
Receipt remains an explicitly labelled availability proxy, not actual processing completion.
`select_time_history` filters run/epoch/clock/availability before duplicate resolution,
uses fixed 0.1 s past-only slots, and leaves missing slots unfilled. Shared materializer
is usable offline/online. Actual ROS ingestion and full camera/ego interpolation remain
integration work; no claim that real-bag availability has been reconstructed exactly.

## P0-B

TimePath alone now uses valid-frame gather/CNN/scatter; invalid camera AND LiDAR do not
enter BatchNorm. Fixed relative-slot encoding is added for camera/LiDAR/ego/command GRUs.
Old V4/default V3 backbone behavior is unchanged. Current required sensors remain required.
Output contract revision is `proposed_time_path_v1_p0`. Numeric/bool type aliases rejected.
30-point decoder setting is separate from old backbone head dimensions. PyTorch extra state
records and verifies input configuration, command option and time/history semantics.
This is not the complete future trainer checkpoint format/resume implementation (P1).

## P0-C

`control/time_reference_v1.py` contains typed time/pose identities, fractional-age interpolation,
rigid ego-motion transformation and speed aggregation in the ORIGINAL source time intervals.
Speed is never recomputed from tracking-error distance to current origin. Horizon end does
not impose speed zero. Stationary paths avoid heading division; short references use existing
PP endpoint fallback. Cached plans retire on reset/expiry. No ROS command publisher exists.
Pose origin is REQUIRED to be rear_axle; a base_link input needs an explicit calibrated
transform upstream. This prevents silently assuming the two body reference points coincide.
The 0.3 s speed window and 0.5 s age limit are synthetic test settings, not vehicle calibration.
This P0 validates control calculation, not collision avoidance, calibrated dynamics or safety.

## Commands

Windows commit each stage, then `tools/sync_to_wsl.ps1 -CheckOnly` and `tools/sync_to_wsl.ps1`.
WSL native checkout:

```bash
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q tests/test_time_teacher_p0.py tests/test_time_backbone_p0.py tests/test_time_reference_p0.py tests/test_time_path_v1.py tests/test_dataset_v3_converter.py
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q
```

Regression cases: epoch endpoint/frame changes, independent masks, 90-degree frame rotation,
intervention endpoint, delayed duplicates and tensor replay, train-mode camera/LiDAR mask
isolation, padding-count BN invariance, time-gap identification, metadata/config mismatch,
0.23 s age, rotating ego transform, 0.45 m path, stationary/launch/reset/expiry, synthetic
receding oracle with nonzero progress through existing PP and longitudinal calculation.

## Remaining

P1: real-data adoption/noise/frame/availability audit after SSD, full dataset/checkpoint
pipeline, supported-anchor accumulation weighting, split/lineage, command OFF/ON comparison.
P2: calibrated body transforms, actual timing, feasibility/Supervisor wiring and AWSIM.
P3: optional heads/losses only after baseline evidence. No performance gain claimed.
